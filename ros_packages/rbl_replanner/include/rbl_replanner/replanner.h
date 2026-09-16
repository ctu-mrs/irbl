#ifndef RBL_REPLANNER_H
#define RBL_REPLANNER_H

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <cmath>
#include <tuple>
#include <vector>
#include <optional>
#include <memory>
#include <chrono>
#include <queue>



struct VoxelGrid {
  int X, Y, Z;
  std::vector<int> data;

  VoxelGrid(int x, int y, int z) : X(x), Y(y), Z(z), data(x * y * z, 0) {}

  int& at(int x, int y, int z) {
    return data[x * Y * Z + y * Z + z];
  }

  // Const version for reading
  const int& at(int x, int y, int z) const {
    return data[x * Y * Z + y * Z + z];
  }

  int& at(std::tuple<int, int, int> pos) {
    return data[std::get<0>(pos) * Y * Z + std::get<1>(pos) * Z + std::get<2>(pos)];
  }

  // Const version for reading
  const int& at(std::tuple<int, int, int> pos) const {
    return data[std::get<0>(pos) * Y * Z + std::get<1>(pos) * Z + std::get<2>(pos)];
  }

  void clear() {
    std::fill(data.begin(), data.end(), 0);
  }

};

struct Node {
  Node* parent;
  std::tuple<int, int, int> position;

  double g;
  double h;
  double f;

  Node(Node* parent = nullptr, std::tuple<int, int, int> position = {0, 0, 0}) {
    this->parent = parent;
    this->position = position;
    this->g = 0;
    this->h = 0;
    this->f = 0;
  }

  bool operator==(const Node& other) const {
    return position == other.position;
  }
};

struct CompareNode {
  bool operator()(const Node* a, const Node* b) const {
      return a->f > b->f;
  }
};

struct ReplannerParams {
  double                                                    encumbrance;
  double                                                    voxel_size;
  double                                                    map_width;
  double                                                    map_height;
  double                                                    replanner_freq;
  double                                                    inflation_bonus       = 0.1;
  double                                                    replanner_vox_size    = 0.1;
  // [m] altitude (AGL, same convention as RBLParams::z_min/z_max in rbl_controller.yaml) band the
  // returned path is allowed to occupy -- baked directly into the occupancy grid as a hard block,
  // so A* structurally cannot produce a waypoint outside it. rbl_controller_core's constructor
  // doesn't set these two (would need a one-line wire-up there to track rbl_controller.yaml's
  // z_min/z_max live), so they default to that yaml's current values; adjustable at runtime via
  // setZMin()/setZMax() in the meantime.
  double                                                    z_min                 = 0.5;
  double                                                    z_max                 = 10.0;
  // Weight on the direction-consistency cost added to A*'s edge cost (see AStarPlan()) -- biases
  // the *start* of each new plan towards continuing in roughly the same heading as the UAV's own
  // real, measured heading (see setCurrentHeading()) -- or, until the first heading reading
  // arrives, the previous plan's own start heading as a fallback -- fading out with distance so
  // only the near-agent portion is affected. Without this, a plan recomputed from scratch every
  // cycle has no reason to prefer one of two near-equal-cost routes over the other, so the agent's
  // heading can flip-flop between them every replan; this is what damps that. Anchoring on the
  // real measured heading rather than only the previous plan's own (self-referential) output
  // matters: a bias that only ever compares a new plan against the planner's own last plan can
  // still drift/oscillate together with it, whereas the vehicle's actual heading is an independent
  // ground truth that cannot itself flip-flop from one replan to the next. Set high (well above
  // what a few meters of extra route length would cost) because in practice the UAV
  // reversing/zig-zagging its heading is worse than taking a mildly longer path -- a heading
  // change should only win out when the alternative is meaningfully shorter or an obstacle
  // actually forces it, not on a near-tie. Adjustable at runtime via
  // setDirectionConsistencyWeight().
  double                                                    direction_consistency_weight = 20.0;
  // [m] Distance over which the direction-consistency bias fades out (see AStarPlan()) -- e^-1 at
  // this distance from the plan's start. Now that full A* searches are the exception rather than
  // happening every tick (see plan()'s doc comment), each one covers a much longer, largely-fixed
  // horizon; a decay distance shorter than that horizon leaves the far portion of the route free to
  // flip to a different-but-equally-valid option on the next full replan with no bias pulling it
  // back, which reads as the whole path (not just its start) occasionally jumping sideways. Set
  // close to the local grid's own half-width (map_width/2) so the bias stays meaningfully active
  // across essentially the whole visible horizon rather than fading out well before it.
  double                                                    direction_decay_meters = 18.0;
  // [m] How far into a fresh full A* search (from its start) the path_deviation_weight cost below
  // stays active. Deliberately short and separate from direction_decay_meters: this term follows
  // the previous plan's *local heading at that same distance along it* (see
  // RBLReplanner::pathHeadingAtLength()), i.e. its actual near-start curve, rather than
  // direction_penalty's single fixed initial bearing -- so a full replan doesn't yank the
  // near-agent portion of the path sideways even when the previous route remains perfectly valid
  // there. direction_decay_meters/consistency handles the (weaker, single-bearing) bias further out.
  double                                                    path_deviation_distance = 4.0;
  // Weight on angular misalignment (0 aligned .. 2 opposite, same scale as
  // direction_consistency_weight's term) between a candidate edge's own heading and the previous
  // plan's local heading at that point (see path_deviation_distance above), within
  // path_deviation_distance of the search start. Strong enough to keep the near-agent heading
  // stable across a full replan, but still soft, so it never overrides an actual obstacle. Kept
  // above direction_consistency_weight since this term covers the portion of the route closest to
  // the agent, where a heading reversal is both the most physically disruptive and the most
  // visible as "flip-flopping".
  double                                                    path_deviation_weight = 25.0;
  // [deg] Half-angle, measured from the UAV's real current heading (see setCurrentHeading()), of
  // the cone A* is allowed to take its very first step into. Any of the start node's neighbors
  // whose direction lies outside this cone (i.e. more than this many degrees from straight ahead)
  // is hard-excluded from that first expansion -- not just soft-penalized like
  // direction_consistency_weight/path_deviation_weight above -- so the immediate next waypoint is
  // always somewhere in front of the UAV rather than beside or behind it, matching what a
  // forward-facing, limited-FOV sensor (see RBLParams::limited_fov/lidar_fov) can actually see.
  // Only applied to the start node's own first expansion (not deeper into the search) and only
  // when at least one neighbor still satisfies it -- if every reachable neighbor happens to fall
  // outside the cone (e.g. boxed in on every forward side), the exclusion is skipped for that
  // expansion instead of reporting no path found, so this can never turn a solvable planning
  // problem into an unsolvable one. 100 degrees excludes only the roughly-rearward third of
  // directions (the ones behind the UAV) while still leaving most sideways motion open. No effect
  // until setCurrentHeading() has been called at least once.
  double                                                    forward_lock_half_angle_deg = 100.0;
  // Fraction of the *current* path's remaining arc length the agent must cover before plan()
  // abandons the cheap advance-and-validate path (see plan()) and runs a full A* search instead --
  // keeps the local horizon being pushed out towards the goal even when nothing is blocking the
  // path already known within it.
  double                                                    progress_threshold    = 0.5;
};

// Discrete, obstacle-inflated 3D grid A* planner. Safety comes entirely from the grid: any cell
// within (encumbrance + inflation_bonus) of an obstacle point is hard-blocked, so A* is
// structurally unable to route through it -- there is no soft/weighted "prefer more clearance"
// cost layered on top, which keeps the search itself a plain shortest-path problem and therefore
// fast and predictable.
//
// plan() is meant to be called at a steady, fairly high rate (e.g. 5 Hz). Most calls do NOT re-run
// A*: they trim the previous plan() result down to the vertices still ahead of the agent, splice
// in a fresh segment from the agent's exact current position, and validate the result -- vertices
// and the line-of-sight between them -- against the freshly rebuilt inflated grid. Only when that
// validation fails (something now blocks the remaining path), the goal changed, or the agent has
// covered progress_threshold of what's left does plan() fall back to a full A* search. This is what
// lets consecutive plans share most of their waypoints instead of independently re-deriving a route
// that can differ from the last one in irrelevant ways -- the actual source of route-to-route
// "chattering" when replanning from scratch every cycle.
class RBLReplanner {
public:
  RBLReplanner(const ReplannerParams& par);
  void setCurrentPosition(const Eigen::Vector3d& point);
  void setGoal(const Eigen::Vector3d& point);
  void setAltitude(const double& alt);
  // The UAV's real, measured heading (e.g. from odometry yaw) as a direction vector -- need not be
  // normalized or purely horizontal, but only its projection onto the XY plane is used (see
  // AStarPlan()); a near-zero XY component (e.g. a heading vector that is all Z) is treated as "no
  // heading" for that call and leaves the previously-set heading (or, before the first call, the
  // previous-plan-direction fallback) in effect. This is what grounds the direction-consistency
  // bias and the forward-lock cone (see ReplannerParams::direction_consistency_weight and
  // forward_lock_half_angle_deg) in the vehicle's actual physical orientation instead of only the
  // planner's own prior output. Expected to be called once per replan cycle, before plan().
  void setCurrentHeading(const Eigen::Vector3d& heading);
  void setPCL(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud);
  // Adjusts the altitude band (see ReplannerParams::z_min/z_max) at runtime.
  void setZMin(double z_min);
  void setZMax(double z_max);
  // Adjusts the direction-consistency weight (see ReplannerParams::direction_consistency_weight)
  // at runtime.
  void setDirectionConsistencyWeight(double weight);

  std::vector<Eigen::Vector3d> getInflatedCloud();

  std::vector<Eigen::Vector3d> plan();
  bool replanTimer();

  // Stateless (touches none of RBLReplanner's own mutable members -- _inflated_grid_, agent_pos_,
  // etc), so unlike plan() this is safe to call from any thread, including the calling thread
  // itself every control cycle, concurrently with an in-flight async plan(). Checks whether any
  // point of `path` comes within `safety_radius` of any point in `cloud`. Meant to be called far
  // more often than the full replan cycle (e.g. every control tick against the live cloud), so a
  // newly-revealed obstacle on the still-active path is caught immediately instead of only at the
  // next scheduled replan -- the caller should force an immediate replan when this returns true.
  static bool pathInCollision(const std::vector<Eigen::Vector3d>&                     path,
                               const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud,
                               double                                                  safety_radius);

  // Stateless, same spirit as pathInCollision() above. True once the agent has covered at least
  // `percentage` of `path`'s own arc length -- measured by projecting agent_pos onto the path's
  // nearest segment (not just its nearest vertex, which would be a poor progress estimate once
  // smoothPath() has collapsed the path down to a few long segments) and comparing arc length up
  // to that projection against the path's total length. Meant to force a fresh replan well before
  // the agent runs off the end of a stale path, instead of only relying on the periodic
  // replanner_freq ceiling.
  static bool pathMostlyTraveled(const std::vector<Eigen::Vector3d>& path,
                                  const Eigen::Vector3d&              agent_pos,
                                  double                               percentage);

private:
  ReplannerParams                                           params_;
  double                                                    voxel_size_;
  double                                                    inflation_;
  double                                                    replanner_period_;
  int                                                       inflation_coeff_;
  double                                                    altitude_;
  double                                                    z_min_;
  double                                                    z_max_;
  // Grid z-index bounds corresponding to [z_min_, z_max_] AGL, recomputed each
  // initializationPlan() call. Everything outside this range is hard-blocked in
  // fillAndInflateGrid(), so A* can never route through it.
  int                                                        z_min_idx_ = 0;
  int                                                        z_max_idx_ = 0;
  double                                                    direction_consistency_weight_;
  // Heading (unit vector) of the previous plan's own first segment, used by AStarPlan() to bias
  // the new plan's start towards continuing that same direction. Empty (zero vector) until the
  // first successful plan. EMA-smoothed run-to-run (see plan()) rather than hard-overwritten, so a
  // single transient A* tie-break flip doesn't itself become the next plan's bias target.
  Eigen::Vector3d                                           last_plan_direction_ = Eigen::Vector3d::Zero();
  bool                                                       have_last_plan_direction_ = false;
  // UAV's real, measured heading (unit vector, XY-only -- see setCurrentHeading()), used in place
  // of last_plan_direction_ above whenever available: an independent ground-truth reference that
  // cannot itself drift/flip-flop the way a bias derived from the planner's own prior output can.
  Eigen::Vector3d                                           agent_heading_ = Eigen::Vector3d::Zero();
  bool                                                       have_agent_heading_ = false;
  Eigen::Vector3d                                           agent_pos_;
  Eigen::Vector3d                                           goal_;
  // True from setGoal() until the next plan() that actually observes it -- forces that plan() to
  // do a full A* search (a changed goal isn't something advanceAndValidate()'s trim-and-check can
  // account for) instead of trying to advance the old, now-stale-goal path.
  bool                                                      goal_changed_ = false;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>           cloud_;
  // Previous plan() result (world coords) -- the basis advanceAndValidate() trims/extends from on
  // the next call. Empty until the first successful plan.
  std::vector<Eigen::Vector3d>                              path_;
  // Arc length of path_ at the moment it was last produced by a full A* search (not updated on a
  // cheap advance-and-validate tick). plan()'s progress_threshold check compares the *current*
  // remaining length of path_ against this fixed reference -- deliberately NOT against path_'s own
  // current total length, which shrinks every single tick (advanceAndValidate() re-roots path_ at
  // the agent's position every call) and so would make the threshold effectively compare "this
  // tick's movement" against "whatever's left", firing only once almost nothing is left rather than
  // with real margin to spare.
  double                                                    horizon_length_ = 0.0;
  std::chrono::high_resolution_clock::time_point            last_replan;
  bool                                                      first_plan;
  //Variables related to grid
  int                                                       _X_;
  int                                                       _Y_;
  int                                                       _Z_;
  std::tuple<int, int, int>                                 _agent_pos_;
  std::tuple<int, int, int>                                 _goal_;
  std::optional<VoxelGrid>                                  _inflated_grid_;

  void initializationPlan();
  double roundToNextMultiple(double value, double multiple);
  Eigen::Vector3d gridIdxToWorldCoords(const std::tuple<int, int, int>& _point);
  Eigen::Vector3d gridIdxToWorldCoords(const int& x, const int& y, const int& z);
  std::tuple<int, int, int> worldCoordsToGridIdx(const Eigen::Vector3d& point);
  std::tuple<int, int, int> worldCoordsToGridIdx(const pcl::PointXYZI& point);
  void fillAndInflateGrid(std::optional<VoxelGrid>& grid, const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud);
  std::vector<Eigen::Vector3d> gridPathToWorldPath(std::vector<std::tuple<int, int ,int>>& _path);
  std::vector<std::tuple<int, int, int>> smoothPath(const std::vector<std::tuple<int, int ,int>>& _path, const std::optional<VoxelGrid>& grid);
  bool canConnectPoints(const std::tuple<int, int, int>& p1, const std::tuple<int, int, int>& p2, const std::optional<VoxelGrid>& grid);
  // Trims `prev_path` to the vertices still ahead of agent_pos_ (dropping the consumed prefix),
  // splices in a fresh leading segment from agent_pos_ itself, and validates every remaining vertex
  // plus the line-of-sight between consecutive vertices against `grid`. Returns false (leaving
  // `advanced_path` unspecified) if prev_path is too short to trim or anything fails validation --
  // the caller should fall back to a full AStarPlan() in that case.
  bool advanceAndValidate(const std::vector<Eigen::Vector3d>& prev_path, const std::optional<VoxelGrid>& grid, std::vector<Eigen::Vector3d>& advanced_path);
  // Arc length of `path` still ahead of agent_pos_ -- i.e. total length minus the length up to
  // agent_pos_'s projection onto path's nearest segment. Used against horizon_length_ to decide
  // when plan() needs a full A* search (see horizon_length_'s doc comment).
  double remainingPathLength(const std::vector<Eigen::Vector3d>& path);
  // Unit tangent direction of `path` at arc length `length` from its front -- i.e. the local
  // heading `path` itself had at that point of travel, clamped to path's own extent (returns the
  // final segment's heading once `length` reaches or exceeds it). Zero vector if path has fewer
  // than 2 points. Used to make the near-start heading bias in AStarPlan() follow the previous
  // path's actual curve within path_deviation_distance, not just a single fixed initial bearing.
  Eigen::Vector3d pathHeadingAtLength(const std::vector<Eigen::Vector3d>& path, double length);
  std::vector<std::tuple<int, int ,int>> AStarPlan(const std::tuple<int, int, int> _start, const std::tuple<int, int, int> _goal, const std::optional<VoxelGrid>& grid);
  double euclideanDistance(const std::tuple<int, int, int>& p1, const std::tuple<int, int, int>& p2);
  std::tuple<int, int, int> closestFreeIdx(const std::tuple<int, int, int>& _position, const std::optional<VoxelGrid>& grid);
};


#endif
