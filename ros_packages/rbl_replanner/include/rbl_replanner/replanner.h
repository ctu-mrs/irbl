#ifndef RBL_REPLANNER_H
#define RBL_REPLANNER_H

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <cmath>
#include <numeric> 
#include <tuple>
#include <vector>
#include <optional>
#include <memory>
#include <chrono>
#include <queue>
#include <set>



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
  double                                                    weight_safety;
  double                                                    weight_deviation;
  double                                                    replanner_freq;
  double                                                    eps                   = 0.001;
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
  // the *start* of each new plan towards continuing in roughly the same heading as the previous
  // plan's start, fading out with distance so only the near-agent portion is affected. Always
  // plans from the agent's actual current position (never a frozen/locked prefix); this is what
  // keeps the direction from flip-flopping between replans instead. Adjustable at runtime via
  // setDirectionConsistencyWeight().
  double                                                    direction_consistency_weight = 5.0;
  // [s] If the agent hasn't moved more than stuck_distance over this long, shouldReplan() forces a
  // fresh replan (a different route than the one it's apparently stuck on), instead of waiting for
  // the path to reach stuck_check_percentage or become blocked.
  double                                                    stuck_timeout         = 3.0;
  // [m] Movement below this over stuck_timeout counts as "no progress" for the stuck check above.
  double                                                    stuck_distance        = 0.3;
};

class RBLReplanner {
public:
  RBLReplanner(const ReplannerParams& par);
  void setCurrentPosition(const Eigen::Vector3d& point);
  void setGoal(const Eigen::Vector3d& point);
  void setAltitude(const double& alt);
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

private:
  ReplannerParams                                           params_;
  double                                                    voxel_size_;
  double                                                    inflation_;
  double                                                    replanner_period_;
  int                                                       inflation_coeff_;
  double                                                    altitude_;
  double                                                    z_min_;
  double                                                    z_max_;
  double                                                    direction_consistency_weight_;
  // Heading (unit vector) of the previous plan's own first segment, used by AStarPlan() to bias
  // the new plan's start towards continuing that same direction. Empty (zero vector) until the
  // first successful plan.
  Eigen::Vector3d                                           last_plan_direction_ = Eigen::Vector3d::Zero();
  bool                                                       have_last_plan_direction_ = false;
  // Grid z-index bounds corresponding to [z_min_, z_max_] AGL, recomputed each
  // initializationPlan() call. Everything outside this range is hard-blocked in
  // fillAndInflateGrid(), so A* can never route through it.
  int                                                        z_min_idx_ = 0;
  int                                                        z_max_idx_ = 0;
  Eigen::Vector3d                                           agent_pos_;
  // Progress-tracking state for the "stuck" check in shouldReplan() -- see isStuck(). Reset to the
  // agent's current position/time whenever it's moved more than stuck_distance_ since the last
  // reset, so this only fires on a genuine, sustained lack of progress.
  Eigen::Vector3d                                           stuck_check_pos_ = Eigen::Vector3d::Zero();
  std::chrono::high_resolution_clock::time_point            stuck_check_time_;
  bool                                                       have_stuck_check_ = false;
  Eigen::Vector3d                                           goal_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>           cloud_;
  std::vector<Eigen::Vector3d>                              path_;
  std::vector<Eigen::Vector3d>                              smooth_path_;
  std::chrono::high_resolution_clock::time_point            last_replan;
  bool                                                      first_plan;
  bool                                                      goal_changed_;
  //Variables related to grid
  int                                                       _X_;
  int                                                       _Y_;
  int                                                       _Z_;
  std::tuple<int, int, int>                                 _agent_pos_;
  std::tuple<int, int, int>                                 _goal_;
  std::vector<std::tuple<int, int, int>>                    _path_;
  std::optional<VoxelGrid>                                  _inflated_grid_;
  std::optional<VoxelGrid>                                  _clearance_grid_;

  bool shouldReplan(const std::vector<Eigen::Vector3d>& path, Eigen::Vector3d& agent_pos, std::vector<std::tuple<int, int, int>> _path, std::optional<VoxelGrid>& grid);
  bool percentageCompleted(const double percentage, const std::vector<Eigen::Vector3d>& path, Eigen::Vector3d& agent_pos);
  bool pathBlocked(std::vector<std::tuple<int, int, int>> _path, std::optional<VoxelGrid>& grid);
  // True if the agent has made less than stuck_distance_ of progress over the last stuck_timeout_
  // seconds -- meant to catch cases where the path itself looks fine (not blocked, not near
  // completion) but something downstream of the plan (local reactive avoidance, an oscillation) is
  // keeping the agent from actually making headway along it, so a different route is worth trying.
  bool isStuck(const Eigen::Vector3d& agent_pos);

  void initializationPlan();
  double roundToNextMultiple(double value, double multiple);
  Eigen::Vector3d gridIdxToWorldCoords(const std::tuple<int, int, int>& _point);
  Eigen::Vector3d gridIdxToWorldCoords(const int& x, const int& y, const int& z);
  std::tuple<int, int, int> worldCoordsToGridIdx(const Eigen::Vector3d& point);
  std::tuple<int, int, int> worldCoordsToGridIdx(const pcl::PointXYZI& point);
  void fillAndInflateGrid(std::optional<VoxelGrid>& grid, const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud);
  void calculateClearanceGrid(std::optional<VoxelGrid>& clearance_grid, const std::optional<VoxelGrid>& input_grid);
  void calculate1dSquaredDistance(std::vector<int>& data, int length, int stride);
  std::vector<Eigen::Vector3d> gridPathToWorldPath(std::vector<std::tuple<int, int ,int>>& _path);
  std::vector<std::tuple<int, int, int>> worldPathToGridPath(const std::vector<Eigen::Vector3d>& path);
  std::vector<std::tuple<int, int, int>> smoothPath(const std::vector<std::tuple<int, int ,int>>& _path, const std::optional<VoxelGrid>& grid);
  bool canConnectPoints(const std::tuple<int, int, int>& p1, const std::tuple<int, int, int>& p2, const std::optional<VoxelGrid>& grid);
  std::vector<std::tuple<int, int ,int>> AStarPlan(const std::tuple<int, int, int> _start, const std::tuple<int, int, int> _goal, const std::vector<std::tuple<int, int, int>>& _path, const std::optional<VoxelGrid>& grid, const std::optional<VoxelGrid>& clearance_grid);
  double deviationPenalty(const std::vector<std::tuple<int, int, int>>& _path, const std::tuple<int, int, int>& _p1, const std::tuple<int, int, int>& _p2);
  double euclideanDistance(const std::tuple<int, int, int>& p1, const std::tuple<int, int, int>& p2);
  std::tuple<int, int, int> closestFreeIdx(const std::tuple<int, int, int>& _position, const std::optional<VoxelGrid>& grid);
};


#endif