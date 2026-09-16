#include "rbl_replanner/replanner.h"

RBLReplanner::RBLReplanner(const ReplannerParams& params) : params_(params)  // //{
{
  std::cout << "[RBLReplanner]: Replanner initialization" << std::endl;
  voxel_size_ = roundToNextMultiple(params.voxel_size, params.replanner_vox_size);
  inflation_  = roundToNextMultiple(params.encumbrance + params.inflation_bonus, params.replanner_vox_size);
  // inflation_coeff_ cells around an obstacle cell get hard-blocked (see fillAndInflateGrid()), so
  // a free cell's center sits at least (inflation_coeff_ + 1) * replanner_vox_size from the nearest
  // *obstacle cell center*. That is NOT the same as the true world-space clearance the caller
  // actually wants (encumbrance + inflation_bonus), because of quantization on both ends: the raw
  // obstacle point can be up to replanner_vox_size/2 closer to that free cell than its own cell
  // center is (worldCoordsToGridIdx() rounds to the nearest cell), and a naive
  // ceil(margin / vox) - 1 doesn't budget for that at all. Solving
  // (inflation_coeff_ + 1) * vox - vox / 2 >= margin for the smallest integer inflation_coeff_
  // gives the formula below, plus one extra cell of blanket buffer (sensor/localization noise,
  // a path point that isn't exactly a grid cell center after smoothing, etc).
  const double margin = params.encumbrance + params.inflation_bonus;
  inflation_coeff_    = std::max(0, static_cast<int>(std::ceil(margin / params.replanner_vox_size - 0.5))) + 1;
  std::cout << "Inflation coef: " << inflation_coeff_ << std::endl;

  _X_ = static_cast<int>(std::ceil(params.map_width / params.replanner_vox_size));
  _Y_ = static_cast<int>(std::ceil(params.map_width / params.replanner_vox_size));
  _Z_ = static_cast<int>(std::ceil(params.map_height / params.replanner_vox_size));
  // Make them even
  _X_ = (_X_ % 2 == 0) ? _X_ : _X_ + 1;
  _Y_ = (_Y_ % 2 == 0) ? _Y_ : _Y_ + 1;
  _Z_ = (_Z_ % 2 == 0) ? _Z_ : _Z_ + 1;

  replanner_period_ = 1.0 / params.replanner_freq;
  first_plan        = true;
  z_min_            = params.z_min;
  z_max_            = params.z_max;
  direction_consistency_weight_ = params.direction_consistency_weight;

  _inflated_grid_ = VoxelGrid(_X_, _Y_, _Z_);
}  // //}

void RBLReplanner::setCurrentPosition(const Eigen::Vector3d& point)  // //{
{
  agent_pos_ = point;
}  // //}

void RBLReplanner::setGoal(const Eigen::Vector3d& point)  // //{
{
  constexpr double tolerance = 1e-6;
  if (!goal_.isApprox(point, tolerance)) {
    goal_changed_ = true;
  }
  goal_ = point;
}  // //}

void RBLReplanner::setAltitude(const double& alt)  // //{
{
  altitude_ = alt;
}  // //}

void RBLReplanner::setPCL(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud)  // //{
{
  cloud_ = cloud;
}  // //}

void RBLReplanner::setCurrentHeading(const Eigen::Vector3d& heading)  // //{
{
  Eigen::Vector3d flat = heading;
  flat.z()              = 0.0;
  if (flat.norm() > 1e-6) {
    agent_heading_      = flat.normalized();
    have_agent_heading_ = true;
  }
}  // //}

void RBLReplanner::setZMin(double z_min)  // //{
{
  z_min_ = z_min;
}  // //}

void RBLReplanner::setZMax(double z_max)  // //{
{
  z_max_ = z_max;
}  // //}

void RBLReplanner::setDirectionConsistencyWeight(double weight)  // //{
{
  direction_consistency_weight_ = weight;
}  // //}

std::vector<Eigen::Vector3d> RBLReplanner::getInflatedCloud()  // //{
{
  std::vector<Eigen::Vector3d> points;
  for (int x = 0; x < _inflated_grid_->X; ++x) {
    for (int y = 0; y < _inflated_grid_->Y; ++y) {
      for (int z = 0; z < _inflated_grid_->Z; ++z) {
        if (_inflated_grid_->at(x, y, z) == 1) {
          points.push_back(gridIdxToWorldCoords(x, y, z));
        }
      }
    }
  }
  return points;
}  // //}

std::vector<Eigen::Vector3d> RBLReplanner::plan()  // //{
{
  initializationPlan();

  fillAndInflateGrid(_inflated_grid_, cloud_);

  const bool need_full_replan =
      goal_changed_ || path_.size() < 2 ||
      (horizon_length_ > 1e-6 && remainingPathLength(path_) <= (1.0 - params_.progress_threshold) * horizon_length_);

  if (!need_full_replan) {
    std::vector<Eigen::Vector3d> advanced_path;
    if (advanceAndValidate(path_, _inflated_grid_, advanced_path)) {
      path_ = advanced_path;
      return path_;
    }
  }
  goal_changed_ = false;

  std::vector<std::tuple<int, int, int>> _path = AStarPlan(_agent_pos_, _goal_, _inflated_grid_);

  if (_path.size() >= 2) {
    // Walk forward along the fresh path until ~1.5m out (not just the first grid cell -- a single
    // quantized A* step, noisy and zigzag-prone on its own), then EMA-smooth that heading into the
    // reference used by AStarPlan()'s direction-consistency bias instead of hard-overwriting it
    // every plan. A hard overwrite means a single transient A* tie-break flip becomes next plan's
    // bias target, which can amplify oscillation instead of damping it; smoothing the reference
    // itself is what actually damps a transient flip.
    std::vector<Eigen::Vector3d> world_path = gridPathToWorldPath(_path);
    constexpr double              kRefDist   = 1.5;
    double                         accum      = 0.0;
    Eigen::Vector3d                ref_point  = world_path.back();
    for (size_t i = 1; i < world_path.size(); ++i) {
      const double seg = (world_path[i] - world_path[i - 1]).norm();
      if (accum + seg >= kRefDist) {
        const double t = (kRefDist - accum) / std::max(seg, 1e-9);
        ref_point       = world_path[i - 1] + t * (world_path[i] - world_path[i - 1]);
        break;
      }
      accum += seg;
    }
    const Eigen::Vector3d dir = ref_point - world_path.front();
    if (dir.norm() > 1e-6) {
      const Eigen::Vector3d new_dir = dir.normalized();
      if (have_last_plan_direction_) {
        // Low alpha on purpose: at 0.5 a single transient A* tie-break flip already moves the
        // reference heading halfway to the new direction, which is itself most of the way to
        // becoming next plan's bias target -- letting one flip beget another instead of damping
        // it out. 0.2 makes the reference heading track the *sustained* direction of travel and
        // stay put through a one-off flip.
        constexpr double       kEmaAlpha = 0.2;
        const Eigen::Vector3d blended    = (1.0 - kEmaAlpha) * last_plan_direction_ + kEmaAlpha * new_dir;
        last_plan_direction_             = blended.norm() > 1e-6 ? blended.normalized() : new_dir;
      }
      else {
        last_plan_direction_ = new_dir;
      }
      have_last_plan_direction_ = true;
    }
  }

  std::vector<std::tuple<int, int, int>> _smooth_path = smoothPath(_path, _inflated_grid_);
  path_                                                = gridPathToWorldPath(_smooth_path);
  horizon_length_                                      = remainingPathLength(path_);
  return path_;
}  // //}

bool RBLReplanner::replanTimer()  // //{
{
  auto current_time = std::chrono::high_resolution_clock::now();
  if (first_plan) {
    first_plan  = false;
    last_replan = current_time;
    return true;
  }

  std::chrono::duration<double> diff = current_time - last_replan;

  if (diff.count() >= replanner_period_) {
    last_replan = current_time;
    return true;
  }
  return false;
}  // //}

void RBLReplanner::initializationPlan()  // //{
{
  _inflated_grid_->clear();
  _agent_pos_ =
      std::make_tuple(_X_ / 2, _Y_ / 2, std::max(static_cast<int>(altitude_ / params_.replanner_vox_size), 0));

  // Grid z-indices for the [z_min_, z_max_] AGL band. agent_pos_.z() - altitude_ is the world z of
  // "the ground directly below the agent" (grid index 0's own definition -- see
  // fillAndInflateGrid()'s old floor-only fill), so building world points at that ground level
  // plus z_min_/z_max_ and reusing the same worldCoordsToGridIdx() everything else already goes
  // through avoids re-deriving a parallel index formula by hand.
  Eigen::Vector3d z_min_world_point = agent_pos_;
  z_min_world_point.z()             = agent_pos_.z() - altitude_ + z_min_;
  Eigen::Vector3d z_max_world_point = agent_pos_;
  z_max_world_point.z()             = agent_pos_.z() - altitude_ + z_max_;
  z_min_idx_                        = std::get<2>(worldCoordsToGridIdx(z_min_world_point));
  z_max_idx_                        = std::get<2>(worldCoordsToGridIdx(z_max_world_point));

  auto _point = worldCoordsToGridIdx(goal_);
  int  x_goal = std::clamp(std::get<0>(_point), 0, _X_ - 1);
  int  y_goal = std::clamp(std::get<1>(_point), 0, _Y_ - 1);
  // Also clamp to the altitude band, in addition to the grid bounds -- a goal above z_max_ or
  // below z_min_ gets planned towards the closest point within the allowed band instead.
  int  z_goal = std::clamp(std::get<2>(_point), std::max(z_min_idx_, 0), std::min(z_max_idx_, _Z_ - 1));
  _goal_      = std::make_tuple(x_goal, y_goal, z_goal);
}  // //}

double RBLReplanner::roundToNextMultiple(double value,
                                         double multiple)  // //{
{
  if (multiple == 0.0) {
    return value;
  }
  return std::ceil(value / multiple) * multiple;
}  // //}

Eigen::Vector3d RBLReplanner::gridIdxToWorldCoords(const std::tuple<int,
                                                                    int,
                                                                    int>& _point)  // //{
{
  Eigen::Vector3d point;
  point.x() = (std::get<0>(_point) - std::get<0>(_agent_pos_)) * params_.replanner_vox_size + agent_pos_.x();
  point.y() = (std::get<1>(_point) - std::get<1>(_agent_pos_)) * params_.replanner_vox_size + agent_pos_.y();
  point.z() = (std::get<2>(_point) - std::get<2>(_agent_pos_)) * params_.replanner_vox_size + agent_pos_.z();
  return point;
}  // //}

Eigen::Vector3d RBLReplanner::gridIdxToWorldCoords(const int& x,
                                                   const int& y,
                                                   const int& z)  // //{
{
  Eigen::Vector3d point;
  point.x() = (x - std::get<0>(_agent_pos_)) * params_.replanner_vox_size + agent_pos_.x();
  point.y() = (y - std::get<1>(_agent_pos_)) * params_.replanner_vox_size + agent_pos_.y();
  point.z() = (z - std::get<2>(_agent_pos_)) * params_.replanner_vox_size + agent_pos_.z();
  return point;
}  // //}

std::tuple<int,
           int,
           int>
RBLReplanner::worldCoordsToGridIdx(const Eigen::Vector3d& point)  // //{
{
  int x = static_cast<int>(std::round((point.x() - agent_pos_.x()) / params_.replanner_vox_size)) +
          std::get<0>(_agent_pos_);
  int y = static_cast<int>(std::round((point.y() - agent_pos_.y()) / params_.replanner_vox_size)) +
          std::get<1>(_agent_pos_);
  int z = static_cast<int>(std::round((point.z() - agent_pos_.z()) / params_.replanner_vox_size)) +
          std::get<2>(_agent_pos_);
  return std::make_tuple(x, y, z);
}  // //}

std::tuple<int,
           int,
           int>
RBLReplanner::worldCoordsToGridIdx(const pcl::PointXYZI& point)  // //{
{
  int x =
      static_cast<int>(std::round((point.x - agent_pos_.x()) / params_.replanner_vox_size)) + std::get<0>(_agent_pos_);
  int y =
      static_cast<int>(std::round((point.y - agent_pos_.y()) / params_.replanner_vox_size)) + std::get<1>(_agent_pos_);
  int z =
      static_cast<int>(std::round((point.z - agent_pos_.z()) / params_.replanner_vox_size)) + std::get<2>(_agent_pos_);
  return std::make_tuple(x, y, z);
}  // //}

void RBLReplanner::fillAndInflateGrid(std::optional<VoxelGrid>&                              grid,
                                      const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud)  // //{
{
  int                           x, y, z;
  std::vector<std::vector<int>> idx_to_inflate;
  for (const auto& point : cloud->points) {
    auto _point = worldCoordsToGridIdx(point);
    x           = std::get<0>(_point);
    y           = std::get<1>(_point);
    z           = std::get<2>(_point);
    if (x > 0 && x < grid->X && y > 0 && y < grid->Y && z > 0 && z < grid->Z) {
      grid->at(x, y, z) = 1;
      idx_to_inflate.push_back({ x, y, z });
    }
  }

  // Encumbrance + inflation_bonus (see inflation_coeff_'s derivation above) hard-blocked around
  // every obstacle cell -- this is the planner's only safety mechanism: A* simply cannot route
  // through a cell this close to an obstacle, so there is no need for a softer "prefer more
  // clearance" cost on top.
  for (const auto& idx : idx_to_inflate) {
    x = idx[0];
    y = idx[1];
    z = idx[2];
    for (int dx = -inflation_coeff_; dx <= inflation_coeff_; dx++) {
      for (int dy = -inflation_coeff_; dy <= inflation_coeff_; dy++) {
        for (int dz = -inflation_coeff_; dz <= inflation_coeff_; dz++) {
          int nx = x + dx, ny = y + dy, nz = z + dz;
          if (nx >= 0 && nx < grid->X && ny >= 0 && ny < grid->Y && nz >= 0 && nz < grid->Z) {
            grid->at(nx, ny, nz) = 1;
          }
        }
      }
    }
  }

  // Hard-block every layer outside [z_min_idx_, z_max_idx_] (the [z_min_, z_max_] AGL band) --
  // z_min_idx_ alone already subsumes the old "block the ground plane at z=0" behavior whenever
  // z_min_ > 0, since the agent should never be operating below its own configured floor.
  const int z_lo = std::clamp(z_min_idx_, 0, grid->Z);
  const int z_hi = std::clamp(z_max_idx_ + 1, 0, grid->Z);
  for (int x = 0; x < grid->X; ++x) {
    for (int y = 0; y < grid->Y; ++y) {
      for (int z = 0; z < z_lo; ++z) {
        grid->at(x, y, z) = 1;
      }
      for (int z = z_hi; z < grid->Z; ++z) {
        grid->at(x, y, z) = 1;
      }
    }
  }

  // Hard-block an inflation_coeff_-thick border around the whole local grid (X/Y faces -- the
  // world extends well past this bounded window, which is re-centered on the agent every plan()
  // call). The window has no information about obstacles just past its own edge (they're never
  // loaded into `cloud`, so never inflated), so without this a path could legitimately hug the
  // window boundary -- most commonly at the goal, which gets clamped to the grid edge whenever the
  // real goal lies outside the window (the normal case for any goal farther away than half the
  // window width) -- and end up closer than the intended safety margin to whatever's just beyond
  // it. Blocking this border and letting the existing closestFreeIdx() BFS (already used for both
  // the start and the goal in AStarPlan()) push the goal inward is what actually enforces the same
  // margin against the unknown as against a known, in-window obstacle.
  const int bx = std::min(inflation_coeff_, grid->X / 2);
  const int by = std::min(inflation_coeff_, grid->Y / 2);
  for (int y = 0; y < grid->Y; ++y) {
    for (int z = 0; z < grid->Z; ++z) {
      for (int x = 0; x < bx; ++x) {
        grid->at(x, y, z)             = 1;
        grid->at(grid->X - 1 - x, y, z) = 1;
      }
    }
  }
  for (int x = 0; x < grid->X; ++x) {
    for (int z = 0; z < grid->Z; ++z) {
      for (int y = 0; y < by; ++y) {
        grid->at(x, y, z)             = 1;
        grid->at(x, grid->Y - 1 - y, z) = 1;
      }
    }
  }
}  // //}

std::vector<Eigen::Vector3d> RBLReplanner::gridPathToWorldPath(std::vector<std::tuple<int,
                                                                                      int,
                                                                                      int>>& _path)  // //{
{
  std::vector<Eigen::Vector3d> path;
  if (_path.empty()) {
    return path;
  }
  for (auto _point : _path) {
    path.push_back(gridIdxToWorldCoords(_point));
  }
  return path;
}  // //}

std::vector<std::tuple<int,
                       int,
                       int>>
RBLReplanner::smoothPath(const std::vector<std::tuple<int,
                                                      int,
                                                      int>>& _path,
                         const std::optional<VoxelGrid>&     grid)  // //{
{
  std::vector<std::tuple<int, int, int>> _smooth_path_fwrd;
  if (_path.empty()) {
    return _smooth_path_fwrd;
  }
  if (_path.size() <= 2) {
    return _path;
  }
  _smooth_path_fwrd.push_back(_path.front());

  // Forward pass
  size_t last_smooth_idx = 0;
  for (size_t i = 1; i < _path.size(); ++i) {
    if (canConnectPoints(_path[last_smooth_idx], _path[i], grid)) {
      continue;
    }
    else {
      _smooth_path_fwrd.push_back(_path[i - 1]);
      last_smooth_idx = i - 1;
      i               = last_smooth_idx + 1;
    }
  }
  _smooth_path_fwrd.push_back(_path.back());

  // Backward pass
  std::vector<std::tuple<int, int, int>> final_smooth_path;
  final_smooth_path.push_back(_smooth_path_fwrd.back());
  size_t last_smooth_idx_bck = _smooth_path_fwrd.size() - 1;
  for (size_t i = _smooth_path_fwrd.size() - 1; i > 0; --i) {
    if (canConnectPoints(_smooth_path_fwrd[last_smooth_idx_bck], _smooth_path_fwrd[i], grid)) {
      continue;
    }
    else {
      final_smooth_path.push_back(_smooth_path_fwrd[i + 1]);
      last_smooth_idx_bck = i + 1;
      i                   = last_smooth_idx_bck - 1;
    }
  }
  final_smooth_path.push_back(_smooth_path_fwrd.front());
  std::reverse(final_smooth_path.begin(), final_smooth_path.end());

  return final_smooth_path;
}  // //}

bool RBLReplanner::canConnectPoints(const std::tuple<int,
                                                     int,
                                                     int>&          p1,
                                    const std::tuple<int,
                                                     int,
                                                     int>&          p2,
                                    const std::optional<VoxelGrid>& grid)  // //{
{
  int x1 = std::get<0>(p1);
  int y1 = std::get<1>(p1);
  int z1 = std::get<2>(p1);

  int x2 = std::get<0>(p2);
  int y2 = std::get<1>(p2);
  int z2 = std::get<2>(p2);


  double dist = std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) + std::pow(z2 - z1, 2));
  if (dist == 0.0) {
    return grid.value().at(x1, y1, z1) == 0;
  }
  double step = 0.5;

  for (double t = 0; t <= dist; t += step) {
    double ratio = t / dist;
    int    x     = static_cast<int>(std::round(x1 + (x2 - x1) * ratio));
    int    y     = static_cast<int>(std::round(y1 + (y2 - y1) * ratio));
    int    z     = static_cast<int>(std::round(z1 + (z2 - z1) * ratio));

    if (grid.value().at(x, y, z) == 1) {
      return false;
    }
  }

  return true;
}  // //}

double RBLReplanner::remainingPathLength(const std::vector<Eigen::Vector3d>& path)  // //{
{
  if (path.size() < 2) {
    return 0.0;
  }

  std::vector<double> cumulative_length(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i) {
    cumulative_length[i] = cumulative_length[i - 1] + (path[i] - path[i - 1]).norm();
  }
  const double total_length = cumulative_length.back();

  double best_dist_sq          = std::numeric_limits<double>::max();
  double length_at_closest_pt = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector3d segment    = path[i] - path[i - 1];
    const double           seg_len_sq = segment.squaredNorm();
    double                 t          = seg_len_sq > 1e-12 ? (agent_pos_ - path[i - 1]).dot(segment) / seg_len_sq : 0.0;
    t                                 = std::clamp(t, 0.0, 1.0);
    const Eigen::Vector3d projection = path[i - 1] + t * segment;
    const double           dist_sq    = (projection - agent_pos_).squaredNorm();
    if (dist_sq < best_dist_sq) {
      best_dist_sq          = dist_sq;
      length_at_closest_pt = cumulative_length[i - 1] + t * segment.norm();
    }
  }

  return total_length - length_at_closest_pt;
}  // //}

Eigen::Vector3d RBLReplanner::pathHeadingAtLength(const std::vector<Eigen::Vector3d>& path, double length)  // //{
{
  if (path.size() < 2) {
    return Eigen::Vector3d::Zero();
  }
  double accum = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector3d segment = path[i] - path[i - 1];
    const double           seg_len = segment.norm();
    if (seg_len < 1e-9) {
      continue;
    }
    if (accum + seg_len >= length || i == path.size() - 1) {
      return segment / seg_len;
    }
    accum += seg_len;
  }
  return Eigen::Vector3d::Zero();
}  // //}

bool RBLReplanner::advanceAndValidate(const std::vector<Eigen::Vector3d>& prev_path,
                                      const std::optional<VoxelGrid>&     grid,
                                      std::vector<Eigen::Vector3d>&       advanced_path)  // //{
{
  if (prev_path.size() < 2) {
    return false;
  }

  // Project agent_pos_ onto prev_path's nearest segment (same projection pathMostlyTraveled() uses)
  // to find which of its vertices still lie ahead of the agent.
  double best_dist_sq = std::numeric_limits<double>::max();
  size_t best_seg      = 1;
  for (size_t i = 1; i < prev_path.size(); ++i) {
    const Eigen::Vector3d segment    = prev_path[i] - prev_path[i - 1];
    const double           seg_len_sq = segment.squaredNorm();
    double                 t          = seg_len_sq > 1e-12 ? (agent_pos_ - prev_path[i - 1]).dot(segment) / seg_len_sq : 0.0;
    t                                 = std::clamp(t, 0.0, 1.0);
    const Eigen::Vector3d projection = prev_path[i - 1] + t * segment;
    const double           dist_sq    = (projection - agent_pos_).squaredNorm();
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      best_seg      = i;
    }
  }

  // Advance: drop the consumed prefix, replace it with a fresh segment starting at the agent's
  // exact current position, and keep every vertex from the projection onward untouched -- this is
  // what lets consecutive plans share most of their waypoints instead of A* independently
  // re-deriving a route that can differ from the last one for no reason relevant to safety.
  advanced_path.clear();
  advanced_path.push_back(agent_pos_);
  for (size_t i = best_seg; i < prev_path.size(); ++i) {
    advanced_path.push_back(prev_path[i]);
  }
  if (advanced_path.size() < 2) {
    return false;
  }

  // Validate every remaining vertex, and the line-of-sight between consecutive vertices, against
  // the freshly-inflated grid -- the exact same check (and therefore the exact same
  // encumbrance + inflation_bonus safety margin) A* itself is built on, just far cheaper than a
  // full search since it only walks the handful of vertices already on the path.
  std::vector<std::tuple<int, int, int>> grid_path;
  grid_path.reserve(advanced_path.size());
  for (const auto& p : advanced_path) {
    const auto idx = worldCoordsToGridIdx(p);
    const int  x   = std::get<0>(idx);
    const int  y   = std::get<1>(idx);
    const int  z   = std::get<2>(idx);
    if (x < 0 || x >= grid->X || y < 0 || y >= grid->Y || z < 0 || z >= grid->Z || grid->at(x, y, z) != 0) {
      return false;
    }
    grid_path.push_back(idx);
  }
  for (size_t i = 1; i < grid_path.size(); ++i) {
    if (!canConnectPoints(grid_path[i - 1], grid_path[i], grid)) {
      return false;
    }
  }

  return true;
}  // //}

std::vector<std::tuple<int,
                       int,
                       int>>
RBLReplanner::AStarPlan(const std::tuple<int,
                                         int,
                                         int>               _start,
                        const std::tuple<int,
                                         int,
                                         int>               _goal,
                        const std::optional<VoxelGrid>&     grid)  // //{
{
  Node* start_node = new Node(nullptr, closestFreeIdx(_start, grid));
  Node* end_node   = new Node(nullptr, closestFreeIdx(_goal, grid));
  // Reference point/decay scale for the direction-consistency bias below -- computed once since
  // it's the same for every child expanded during this whole search.
  const Eigen::Vector3d start_world = gridIdxToWorldCoords(start_node->position);
  // Reference heading for direction_penalty below -- the UAV's real measured heading when known
  // (see setCurrentHeading()), otherwise the previous plan's own start heading as a fallback (e.g.
  // before the first odometry-derived heading has arrived). Computed once: same for every child
  // expanded during this whole search.
  const bool             have_direction_ref = have_agent_heading_ || have_last_plan_direction_;
  const Eigen::Vector3d  direction_ref      = have_agent_heading_ ? agent_heading_ : last_plan_direction_;
  const double            forward_lock_cos   = std::cos(params_.forward_lock_half_angle_deg * M_PI / 180.0);

  std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_list;
  VoxelGrid                                                   closed_voxels(_X_, _Y_, _Z_);

  open_list.push(start_node);
  std::vector<Node*> all_allocated_nodes;
  all_allocated_nodes.push_back(start_node);
  all_allocated_nodes.push_back(end_node);

  while (!open_list.empty()) {
    Node* current_node = open_list.top();
    open_list.pop();
    if (closed_voxels.at(current_node->position)) {
      continue;
    }

    closed_voxels.at(current_node->position) = 1;

    if (*current_node == *end_node) {  // reconstruct the path
      Node*                                  current = current_node;
      std::vector<std::tuple<int, int, int>> _path;
      while (current != nullptr) {  // nullptr means start
        _path.push_back(current->position);
        current = current->parent;
      }
      std::reverse(_path.begin(), _path.end());

      for (Node* node : all_allocated_nodes)
        delete node;
      return _path;
    }
    std::vector<Node*> children;
    int                new_positions[][3] = { { 0, -1, 0 },   { 0, 1, 0 },   { -1, 0, 0 },  { 1, 0, 0 },
                                              { 0, 0, -1 },   { 0, 0, 1 },                                 // Face
                                              { -1, -1, 0 },  { -1, 1, 0 },  { 1, -1, 0 },  { 1, 1, 0 },   // Edge XY
                                              { 0, -1, -1 },  { 0, -1, 1 },  { 0, 1, -1 },  { 0, 1, 1 },   // Edge YZ
                                              { -1, 0, -1 },  { -1, 0, 1 },  { 1, 0, -1 },  { 1, 0, 1 },   // Edge XZ
                                              { -1, -1, -1 }, { -1, -1, 1 }, { -1, 1, -1 }, { -1, 1, 1 },  // Corner
                                              { 1, -1, -1 },  { 1, -1, 1 },  { 1, 1, -1 },  { 1, 1, 1 } };

    for (int i = 0; i < 26; ++i) {
      std::tuple<int, int, int> node_position = { std::get<0>(current_node->position) + new_positions[i][0],
                                                  std::get<1>(current_node->position) + new_positions[i][1],
                                                  std::get<2>(current_node->position) + new_positions[i][2] };

      if (std::get<0>(node_position) > grid->X - 1 ||
          std::get<0>(node_position) < 0 ||  // check if outside of local map where mapping and planning is happening
          std::get<1>(node_position) > grid->Y - 1 || std::get<1>(node_position) < 0 ||
          std::get<2>(node_position) > grid->Z - 1 || std::get<2>(node_position) < 0) {
        continue;
      }

      if (grid->at(std::get<0>(node_position), std::get<1>(node_position), std::get<2>(node_position)) !=
          0) {  // check if free space
        continue;
      }

      // No corner-cutting: for a diagonal move (more than one axis changing at once), also
      // require that the axis-aligned cells it "cuts across" are free, not just the endpoint.
      // Without this, two blocked cells that only share an edge or corner still leave a diagonal
      // gap A* is free to slip straight through -- even though that gap can be narrower than the
      // inflation margin actually guarantees (the margin is only proven safe axis-aligned; cutting
      // diagonally past a blocked cell's corner can come meaningfully closer to it).
      {
        const int cx = std::get<0>(current_node->position);
        const int cy = std::get<1>(current_node->position);
        const int cz = std::get<2>(current_node->position);
        const int dx = new_positions[i][0];
        const int dy = new_positions[i][1];
        const int dz = new_positions[i][2];
        bool corner_blocked = false;
        if (dx != 0 && grid->at(cx + dx, cy, cz) != 0) corner_blocked = true;
        if (!corner_blocked && dy != 0 && grid->at(cx, cy + dy, cz) != 0) corner_blocked = true;
        if (!corner_blocked && dz != 0 && grid->at(cx, cy, cz + dz) != 0) corner_blocked = true;
        if (corner_blocked) {
          continue;
        }
      }

      Node* new_node = new Node(current_node, node_position);
      all_allocated_nodes.push_back(new_node);
      children.push_back(new_node);
    }

    // Forward lock: on the start node's own first expansion only, hard-exclude any neighbor whose
    // direction from the start falls outside the forward_lock_half_angle_deg cone around the UAV's
    // real current heading -- see ReplannerParams::forward_lock_half_angle_deg for the full
    // rationale. current_node->parent == nullptr uniquely identifies start_node (every other node
    // is created with a non-null parent above), so this never fires again deeper into the search.
    // Skipped (not applied) when it would leave zero candidates, so a boxed-in agent can still find
    // a path out the only side available instead of AStarPlan() reporting none found.
    if (current_node->parent == nullptr && have_agent_heading_ && !children.empty()) {
      std::vector<Node*> forward_children;
      forward_children.reserve(children.size());
      for (Node* c : children) {
        const Eigen::Vector3d dir = gridIdxToWorldCoords(c->position) - start_world;
        if (dir.norm() > 1e-9 && dir.normalized().dot(agent_heading_) >= forward_lock_cos) {
          forward_children.push_back(c);
        }
      }
      if (!forward_children.empty()) {
        children = std::move(forward_children);
      }
    }

    for (Node* child : children) {
      if (closed_voxels.at(child->position))
        continue;

      // Mostly-plain shortest-path cost: accumulated Euclidean distance plus the Euclidean-distance
      // heuristic to the goal. Safety is already guaranteed structurally by the inflated grid (see
      // fillAndInflateGrid()), so no clearance term is needed here -- the two additions below are
      // both soft biases towards the *previous* plan, so consecutive full replans don't
      // independently re-derive routes that differ from each other for no reason relevant to
      // safety (the actual source of route-to-route "chattering" when replanning from scratch).
      const Eigen::Vector3d child_world      = gridIdxToWorldCoords(child->position);
      const double           dist_from_start = (child_world - start_world).norm();

      // direction_penalty: biases the plan's start towards continuing in roughly the same *heading*
      // as direction_ref (the UAV's real current heading when known, else the previous plan's own
      // start heading). Fades out over direction_decay_meters, so only the near-agent portion of
      // the route is nudged -- further out is free to go wherever the goal/obstacles actually
      // require.
      double direction_penalty = 0.0;
      if (have_direction_ref && dist_from_start > 1e-6) {
        const double cos_angle           = ((child_world - start_world) / dist_from_start).dot(direction_ref);
        const double angular_misalignment = 1.0 - cos_angle;  // 0 aligned .. 2 opposite
        const double fade                 = std::exp(-dist_from_start / params_.direction_decay_meters);
        direction_penalty                 = direction_consistency_weight_ * angular_misalignment * fade;
      }

      // path_deviation_penalty: within path_deviation_distance of the start, additionally biases
      // each edge's own *local heading* towards the previous plan's local heading *at that same
      // distance along it* (path_ -- still the previous result here; plan() only overwrites it
      // after AStarPlan() returns) -- i.e. it follows the old path's actual curve near the agent,
      // rather than direction_penalty's single fixed initial bearing. Deliberately heading-based
      // rather than snapping to the old path's exact position: matching position fights the grid's
      // quantization (the new search is very unlikely to pass through the exact same cells) and can
      // itself reintroduce jitter, whereas matching heading only asks the route to keep going the
      // same way, which is what actually reads as smooth.
      double path_deviation_penalty = 0.0;
      if (path_.size() >= 2 && dist_from_start < params_.path_deviation_distance) {
        const Eigen::Vector3d parent_world   = gridIdxToWorldCoords(current_node->position);
        const Eigen::Vector3d edge           = child_world - parent_world;
        const double           edge_len       = edge.norm();
        const Eigen::Vector3d prev_heading   = pathHeadingAtLength(path_, dist_from_start);
        if (edge_len > 1e-9 && prev_heading.norm() > 1e-9) {
          const double cos_angle           = (edge / edge_len).dot(prev_heading);
          const double angular_misalignment = 1.0 - cos_angle;  // 0 aligned .. 2 opposite
          const double fade                 = 1.0 - dist_from_start / params_.path_deviation_distance;
          path_deviation_penalty            = params_.path_deviation_weight * angular_misalignment * fade;
        }
      }

      child->g = current_node->g + euclideanDistance(current_node->position, child->position);
      child->h = euclideanDistance(child->position, end_node->position);
      child->f = child->g + child->h + direction_penalty + path_deviation_penalty;

      open_list.push(child);
    }
  }

  for (Node* node : all_allocated_nodes)
    delete node;

  std::cout << "[RBLReplanner]: No path found" << std::endl;

  return {};
}  // //}

double RBLReplanner::euclideanDistance(const std::tuple<int,
                                                        int,
                                                        int>& p1,
                                       const std::tuple<int,
                                                        int,
                                                        int>& p2)  // //{
{
  return params_.replanner_vox_size *
         sqrt(pow(std::get<0>(p1) - std::get<0>(p2), 2) + pow(std::get<1>(p1) - std::get<1>(p2), 2) +
              pow(std::get<2>(p1) - std::get<2>(p2), 2));
}  // //}

std::tuple<int,
           int,
           int>
RBLReplanner::closestFreeIdx(const std::tuple<int,
                                              int,
                                              int>&          _position,
                             const std::optional<VoxelGrid>& grid)  // //{
{
  std::tuple<int, int, int> clamped_position = {
      std::clamp(std::get<0>(_position), 0, grid->X - 1),
      std::clamp(std::get<1>(_position), 0, grid->Y - 1),
      std::clamp(std::get<2>(_position), 0, grid->Z - 1),
  };

  if (grid->at(clamped_position) == 0) {
    return clamped_position;
  }
  std::queue<std::tuple<int, int, int>> q;
  VoxelGrid                             visited(_X_, _Y_, _Z_);

  q.push(clamped_position);
  visited.at(clamped_position) = 1;

  int new_positions[][3] = { { 0, -1, 0 },   { 0, 1, 0 },   { -1, 0, 0 },  { 1, 0, 0 },  { 0, 0, -1 },  { 0, 0, 1 },
                             { -1, -1, 0 },  { -1, 1, 0 },  { 1, -1, 0 },  { 1, 1, 0 },  { 0, -1, -1 }, { 0, -1, 1 },
                             { 0, 1, -1 },   { 0, 1, 1 },   { -1, 0, -1 }, { -1, 0, 1 }, { 1, 0, -1 },  { 1, 0, 1 },
                             { -1, -1, -1 }, { -1, -1, 1 }, { -1, 1, -1 }, { -1, 1, 1 }, { 1, -1, -1 }, { 1, -1, 1 },
                             { 1, 1, -1 },   { 1, 1, 1 } };

  while (!q.empty()) {
    std::tuple<int, int, int> current_pos = q.front();
    q.pop();

    for (int i = 0; i < 26; ++i) {
      std::tuple<int, int, int> next_pos = { std::get<0>(current_pos) + new_positions[i][0],
                                             std::get<1>(current_pos) + new_positions[i][1],
                                             std::get<2>(current_pos) + new_positions[i][2] };

      if (std::get<0>(next_pos) < 0 || std::get<0>(next_pos) >= grid->X || std::get<1>(next_pos) < 0 ||
          std::get<1>(next_pos) >= grid->Y || std::get<2>(next_pos) < 0 || std::get<2>(next_pos) >= grid->Z) {
        continue;
      }

      if (visited.at(next_pos)) {
        continue;
      }

      if (grid->at(std::get<0>(next_pos), std::get<1>(next_pos), std::get<2>(next_pos)) == 0) {
        return next_pos;  // Found the closest free spot
      }

      visited.at(next_pos) = 1;
      q.push(next_pos);
    }
  }
  return clamped_position;
}  // //}

bool RBLReplanner::pathMostlyTraveled(const std::vector<Eigen::Vector3d>& path,
                                      const Eigen::Vector3d&              agent_pos,
                                      double                               percentage)  // //{
{
  if (path.size() < 2) {
    return true;
  }

  std::vector<double> cumulative_length(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i) {
    cumulative_length[i] = cumulative_length[i - 1] + (path[i] - path[i - 1]).norm();
  }
  const double total_length = cumulative_length.back();
  if (total_length < 1e-6) {
    return true;
  }

  double best_dist_sq          = std::numeric_limits<double>::max();
  double length_at_closest_pt = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector3d segment    = path[i] - path[i - 1];
    const double           seg_len_sq = segment.squaredNorm();
    double                 t          = seg_len_sq > 1e-12 ? (agent_pos - path[i - 1]).dot(segment) / seg_len_sq : 0.0;
    t                                 = std::clamp(t, 0.0, 1.0);
    const Eigen::Vector3d projection = path[i - 1] + t * segment;
    const double           dist_sq    = (projection - agent_pos).squaredNorm();
    if (dist_sq < best_dist_sq) {
      best_dist_sq          = dist_sq;
      length_at_closest_pt = cumulative_length[i - 1] + t * segment.norm();
    }
  }

  return (length_at_closest_pt / total_length) >= percentage;
}  // //}

bool RBLReplanner::pathInCollision(const std::vector<Eigen::Vector3d>&                     path,
                                   const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud,
                                   double                                                  safety_radius)  // //{
{
  if (path.empty() || !cloud || cloud->points.empty()) {
    return false;
  }

  const double r2 = safety_radius * safety_radius;
  for (const auto& p : path) {
    for (const auto& pt : cloud->points) {
      const double dx = pt.x - p.x();
      const double dy = pt.y - p.y();
      const double dz = pt.z - p.z();
      if (dx * dx + dy * dy + dz * dz <= r2) {
        return true;
      }
    }
  }
  return false;
}  // //}
