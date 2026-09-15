#include "rbl_replanner/replanner.h"

RBLReplanner::RBLReplanner(const ReplannerParams& params) : params_(params)  // //{
{
  std::cout << "[RBLReplanner]: Replanner initialization" << std::endl;
  voxel_size_ = roundToNextMultiple(params.voxel_size, params.replanner_vox_size);
  inflation_  = roundToNextMultiple(params.encumbrance + params.inflation_bonus, params.replanner_vox_size);
  // std::cout << "[RBLReplanner]: voxel_size_: " << voxel_size_ << ", inflation_: " << inflation_ << std::endl;
  // inflation_coeff_ cells around an obstacle cell get hard-blocked (see fillAndInflateGrid()), so
  // a free cell's center sits at least (inflation_coeff_ + 1) * replanner_vox_size from the nearest
  // *obstacle cell center*. That is NOT the same as the true world-space clearance the caller
  // actually wants (encumbrance + inflation_bonus), because of quantization on both ends: the raw
  // obstacle point can be up to replanner_vox_size/2 closer to that free cell than its own cell
  // center is (worldCoordsToGridIdx() rounds to the nearest cell), and a naive
  // ceil(margin / vox) - 1 doesn't budget for that at all -- e.g. margin=0.6, vox=0.3 rounds to
  // exactly 2 cells with zero slack, so the true worst-case clearance can fall short of the
  // intended margin by up to half a voxel. Solving
  // (inflation_coeff_ + 1) * vox - vox / 2 >= margin for the smallest integer inflation_coeff_
  // gives the formula below, which budgets for that slop explicitly instead of relying on
  // roundToNextMultiple() happening to leave enough incidental rounding headroom.
  const double margin       = params.encumbrance + params.inflation_bonus;
  inflation_coeff_          = std::max(0, static_cast<int>(std::ceil(margin / params.replanner_vox_size - 0.5)));
  std::cout << "Inflation coef: " << inflation_coeff_ << std::endl;
  //   int inflation_coeff = std::ceil(encumbrance / map_resolution);

  _X_ = static_cast<int>(std::ceil(params.map_width / params.replanner_vox_size));
  _Y_ = static_cast<int>(std::ceil(params.map_width / params.replanner_vox_size));
  _Z_ = static_cast<int>(std::ceil(params.map_height / params.replanner_vox_size));
  // Make them even
  _X_ = (_X_ % 2 == 0) ? _X_ : _X_ + 1;
  _Y_ = (_Y_ % 2 == 0) ? _Y_ : _Y_ + 1;
  _Z_ = (_Z_ % 2 == 0) ? _Z_ : _Z_ + 1;

  replanner_period_ = 1.0 / params.replanner_freq;
  first_plan        = true;
  goal_changed_     = false;
  z_min_            = params.z_min;
  z_max_            = params.z_max;
  direction_consistency_weight_ = params.direction_consistency_weight;

  _inflated_grid_  = VoxelGrid(_X_, _Y_, _Z_);
  _clearance_grid_ = VoxelGrid(_X_, _Y_, _Z_);
}  // //}

void RBLReplanner::setCurrentPosition(const Eigen::Vector3d& point)  // //{
{
  agent_pos_ = point;
  // std::cout << "[RBLReplanner] Setting Agent position to x: " << agent_pos_.x() << ", y: " << agent_pos_.y() << ", z:
  // " << agent_pos_.z() << std::endl;
}  // //}

void RBLReplanner::setGoal(const Eigen::Vector3d& point)  // //{
{
  double tolerance = 1e-6;
  if (!goal_.isApprox(point, tolerance)) {
    goal_changed_ = true;
    goal_         = point;
  }
}  // //}

void RBLReplanner::setAltitude(const double& alt)  // //{
{
  altitude_ = alt;
}  // //}

void RBLReplanner::setPCL(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud)  // //{
{
  cloud_ = cloud;
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
  // std::cout << "[RBLReplanner]: getting inflated cloud of size: " << points.size() << std::endl;
  return points;
}  // //}

std::vector<Eigen::Vector3d> RBLReplanner::plan()  // //{
{
  initializationPlan();

  auto start = std::chrono::high_resolution_clock::now();
  fillAndInflateGrid(_inflated_grid_, cloud_);
  auto stop             = std::chrono::high_resolution_clock::now();
  auto duration_inflate = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  // std::cout << "[RBLReplanner]: Time taken by fillAndInflateGrid: " << duration_inflate.count() << " microseconds" <<
  // std::endl;

  _path_ = worldPathToGridPath(path_);

  if (!shouldReplan(path_, agent_pos_, _path_, _inflated_grid_)) {
    std::vector<std::tuple<int, int, int>> _smooth_path = smoothPath(_path_, _inflated_grid_);
    std::vector<Eigen::Vector3d>           smooth_path_ = gridPathToWorldPath(_smooth_path);
    return smooth_path_;
  }

  start = std::chrono::high_resolution_clock::now();
  calculateClearanceGrid(_clearance_grid_, _inflated_grid_);
  stop                    = std::chrono::high_resolution_clock::now();
  auto duration_clearance = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  // std::cout << "[RBLReplanner]: Time taken by calculateClearanceGrid: " << duration_clearance.count() << "
  // microseconds" << std::endl;

  // Always plans from the agent's actual current position -- see AStarPlan()'s direction-
  // consistency bias for how the near-agent heading is kept from flip-flopping between replans
  // without locking any geometry in place.
  start                = std::chrono::high_resolution_clock::now();
  _path_               = AStarPlan(_agent_pos_, _goal_, _path_, _inflated_grid_, _clearance_grid_);
  path_                = gridPathToWorldPath(_path_);
  if (path_.size() >= 2) {
    // Walk forward along the fresh path until ~1.5m out (not just the first grid cell -- a single
    // quantized A* step, noisy and zigzag-prone on its own), then EMA-smooth that heading into the
    // reference used by AStarPlan()'s direction-consistency bias instead of hard-overwriting it
    // every plan. A hard overwrite means a single transient A* tie-break flip becomes next plan's
    // bias target, which can amplify oscillation instead of damping it (confirmed empirically:
    // simply raising the bias weight made worst-case direction swings *worse*, not better, since a
    // stronger bias just chases the most recent -- possibly wrong -- direction harder). Smoothing
    // the reference itself is what actually damps a transient flip.
    constexpr double kRefDist = 1.5;
    double           accum     = 0.0;
    Eigen::Vector3d  ref_point = path_.back();
    for (size_t i = 1; i < path_.size(); ++i) {
      const double seg = (path_[i] - path_[i - 1]).norm();
      if (accum + seg >= kRefDist) {
        const double t = (kRefDist - accum) / std::max(seg, 1e-9);
        ref_point       = path_[i - 1] + t * (path_[i] - path_[i - 1]);
        break;
      }
      accum += seg;
    }
    const Eigen::Vector3d dir = ref_point - path_.front();
    if (dir.norm() > 1e-6) {
      const Eigen::Vector3d new_dir = dir.normalized();
      if (have_last_plan_direction_) {
        constexpr double kEmaAlpha = 0.5;
        const Eigen::Vector3d blended = (1.0 - kEmaAlpha) * last_plan_direction_ + kEmaAlpha * new_dir;
        last_plan_direction_ = blended.norm() > 1e-6 ? blended.normalized() : new_dir;
      }
      else {
        last_plan_direction_ = new_dir;
      }
      have_last_plan_direction_ = true;
    }
  }
  stop                 = std::chrono::high_resolution_clock::now();
  auto duration_a_star = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  // std::cout << "[RBLReplanner]: Time taken by AStarPlan: " << duration_a_star.count() << " microseconds" <<
  // std::endl; std::cout << "[RBLReplanner]: Overall planning took: " << (duration_inflate.count() +
  // duration_clearance.count() + duration_a_star.count()) / 1000 << " miliseconds" << std::endl;

  std::vector<std::tuple<int, int, int>> _smooth_path = smoothPath(_path_, _inflated_grid_);
  // std::cout << "[RBLReplanner]: Path from A*: " << _path_.size() << ", smoothed path: " << _smooth_path.size() <<
  // std::endl;
  std::vector<Eigen::Vector3d> smooth_path_ = gridPathToWorldPath(_smooth_path);
  // path_ = gridPathToWorldPath(_smooth_path);
  // interpolate path

  return smooth_path_;
}  // //}

bool RBLReplanner::shouldReplan(const std::vector<Eigen::Vector3d>& path,
                                Eigen::Vector3d&                    agent_pos,
                                std::vector<std::tuple<int,
                                                       int,
                                                       int>>        _path,
                                std::optional<VoxelGrid>&           grid)  // //{
{
  if (goal_changed_) {
    std::cout << "[RBLReplanner]: Replanning, goal_changed " << std::endl;
    goal_changed_ = false;
    return true;
  }

  if (path.size() == 0) {
    // path is empty -> replan
    std::cout << "[RBLReplanner]: Replanning, because path lenght is 0. " << std::endl;
    return true;
  }

  // 50% completed? -> replan
  if (percentageCompleted(0.3, path, agent_pos)) {
    std::cout << "[RBLReplanner]: Replanning, because completed 50 percent of the path. " << std::endl;
    return true;
  }

  // path blocked? -> replan
  if (pathBlocked(_path, grid)) {
    std::cout << "[RBLReplanner]: Replanning, because path is blocked now. " << std::endl;
    return true;
  }

  return false;
}  // //}

bool RBLReplanner::percentageCompleted(const double                        percentage,
                                       const std::vector<Eigen::Vector3d>& path,
                                       Eigen::Vector3d&                    agent_pos)  // //{
// true -> will replan
// false will not replan based on this condition
{
  double min_dist_sq         = std::numeric_limits<double>::max();
  double path_length_sq      = 0.0;
  std::optional<size_t> closest_point_index;

  for (size_t i = 0; i < path.size(); ++i) {
    if (i > 0) {
      path_length_sq += (path[i] - path[i - 1]).squaredNorm();
    }
    double dist_sq = (path[i] - agent_pos).squaredNorm();
    if (dist_sq < min_dist_sq) {
      min_dist_sq         = dist_sq;
      closest_point_index = i;
    }
  }

  if (!closest_point_index || *closest_point_index + 1 >= path.size()) {
    return true;
  }

  double length_to_closest_point_sq = 0.0;
  for (size_t i = 1; i < *closest_point_index; ++i) {
    length_to_closest_point_sq += (path[i] - path[i - 1]).squaredNorm();
  }

  if (length_to_closest_point_sq / path_length_sq >= percentage) {
    return true;
  }
  return false;
}  // //}

bool RBLReplanner::pathBlocked(std::vector<std::tuple<int,
                                                      int,
                                                      int>> _path,
                               std::optional<VoxelGrid>&    grid)  // //{
{
  int x, y, z;
  for (size_t i = 0; i < _path.size(); ++i) {
    x = std::get<0>(_path[i]);
    y = std::get<1>(_path[i]);
    z = std::get<2>(_path[i]);
    // The stored path is world coordinates re-quantized against the *current* (re-centered on the
    // agent) grid every call, via worldCoordsToGridIdx(), which never clamps -- so a path point
    // left over from a few cycles ago can land outside the grid entirely once the agent has moved
    // far enough. grid->at() has no bounds check (segfaults on an out-of-range index), so treat a
    // point that has left the local map as blocked -- it's stale/invalid either way and this
    // forces the caller (shouldReplan) to trigger a fresh, in-bounds replan.
    if (x < 0 || x >= grid->X || y < 0 || y >= grid->Y || z < 0 || z >= grid->Z) {
      return true;
    }
    if (grid->at(x, y, z) == 1) {
      return true;
    }
  }
  return false;
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
  _clearance_grid_->clear();
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
  // std::cout << "[RBLReplanner] Returning x: " << x << ", y: " << y << ", z: " << z << std::endl;
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
  // std::cout << "[RBLReplanner] Returning x: " << x << ", y: " << y << ", z: " << z << std::endl;
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
    else {
      // std::cout << "Error indexing the grid" << std::endl; // TODO THINK about it because it misses a lot - its + 1
      // somewhere ._.
    }
  }

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

// Felzenszwalb & Huttenlocher (F&H) algorithm
void RBLReplanner::calculateClearanceGrid(std::optional<VoxelGrid>&       clearance_grid,
                                          const std::optional<VoxelGrid>& input_grid)  // //{
{
  if (!input_grid.has_value() || !clearance_grid.has_value()) {
    return;
  }

  const VoxelGrid& in_grid  = *input_grid;
  VoxelGrid&       out_grid = *clearance_grid;

  const int occupied_cost = 0;
  const int free_cost     = std::numeric_limits<int>::max();

  for (int x = 0; x < in_grid.X; ++x) {
    for (int y = 0; y < in_grid.Y; ++y) {
      for (int z = 0; z < in_grid.Z; ++z) {
        if (in_grid.at(x, y, z) == 0) {  // Free space
          out_grid.at(x, y, z) = free_cost;
        }
        else {  // Obstacle
          out_grid.at(x, y, z) = occupied_cost;
        }
      }
    }
  }

  // Pass 1: X-axis pass
  for (int y = 0; y < out_grid.Y; ++y) {
    for (int z = 0; z < out_grid.Z; ++z) {
      calculate1dSquaredDistance(out_grid.data, out_grid.X, out_grid.Y * out_grid.Z);
    }
  }

  // Pass 2: Y-axis pass
  for (int x = 0; x < out_grid.X; ++x) {
    for (int z = 0; z < out_grid.Z; ++z) {
      calculate1dSquaredDistance(out_grid.data, out_grid.Y, out_grid.Z);
    }
  }

  // Pass 3: Z-axis pass
  for (int x = 0; x < out_grid.X; ++x) {
    for (int y = 0; y < out_grid.Y; ++y) {
      calculate1dSquaredDistance(out_grid.data, out_grid.Z, 1);
    }
  }

  // Final step: take the square root of all values
  for (size_t i = 0; i < out_grid.data.size(); ++i) {
    out_grid.data[i] = static_cast<int>(std::sqrt(out_grid.data[i]));
  }
}  // //}

void RBLReplanner::calculate1dSquaredDistance(std::vector<int>& data,
                                              int               length,
                                              int               stride)  // //{
{
  std::vector<int>    f(length);
  std::vector<int>    v(length);
  std::vector<double> z(length + 1);

  // Pass 1: find lower envelope of parabolas
  int k = 0;
  v[0]  = 0;
  z[0]  = -std::numeric_limits<double>::infinity();
  z[1]  = std::numeric_limits<double>::infinity();

  for (int q = 1; q < length; ++q) {
    double s;
    do {
      int r = v[k];
      // Intersection point of two parabolas
      s = (double)((data[q * stride] + q * q) - (data[r * stride] + r * r)) / (2.0 * (q - r));
    } while (s <= z[k--]);
    k += 2;
    v[k]     = q;
    z[k]     = s;
    z[k + 1] = std::numeric_limits<double>::infinity();
  }

  // Pass 2: compute final distances
  k = 0;
  for (int q = 0; q < length; ++q) {
    while (z[k + 1] < q) {
      k++;
    }
    int r            = v[k];
    data[q * stride] = data[r * stride] + (q - r) * (q - r);
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
RBLReplanner::worldPathToGridPath(const std::vector<Eigen::Vector3d>& path)  // //{
{
  std::vector<std::tuple<int, int, int>> _path;
  for (const auto& point : path) {
    _path.push_back(worldCoordsToGridIdx(point));
  }
  return _path;
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

std::vector<std::tuple<int,
                       int,
                       int>>
RBLReplanner::AStarPlan(const std::tuple<int,
                                         int,
                                         int>               _start,
                        const std::tuple<int,
                                         int,
                                         int>               _goal,
                        const std::vector<std::tuple<int,
                                                     int,
                                                     int>>& _path,
                        const std::optional<VoxelGrid>&     grid,
                        const std::optional<VoxelGrid>&     clearance_grid)  // //{
{
  Node* start_node = new Node(nullptr, closestFreeIdx(_start, grid));
  Node* end_node   = new Node(nullptr, closestFreeIdx(_goal, grid));
  // Reference point/decay scale for the direction-consistency bias below -- computed once since
  // it's the same for every child expanded during this whole search.
  const Eigen::Vector3d start_world           = gridIdxToWorldCoords(start_node->position);
  constexpr double       kDirectionDecayMeters = 5.0;

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
      // std::cout << "[RBLReplanner]: Reconstructing path." << std::endl;
      Node*                                  current = current_node;
      std::vector<std::tuple<int, int, int>> _path;
      // if (grid->at(_goal) == 1) {
      //   path_idx.push_back(_goal); // because at the start of the func I find the closest free idx on the grid
      // }
      while (current != nullptr) {  // nullptr means start
        _path.push_back(current->position);
        current = current->parent;
      }
      // if (grid->at(_start) == 1) {
      //   path_idx.push_back(_start); // because at the start of the func I find the closest free idx on the grid
      // }
      std::reverse(_path.begin(), _path.end());

      // std::vector<Eigen::Vector3d> path;
      // for (auto _point: _path) {
      //   path.push_back(gridIdxToWorldCoords(_point));
      // }

      for (Node* node : all_allocated_nodes)
        delete node;
      // std::cout << "[RBLReplanner]: Path found returning path of this size: " << _path.size() << std::endl;
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

      Node* new_node = new Node(current_node, node_position);
      all_allocated_nodes.push_back(new_node);
      children.push_back(new_node);
    }

    for (Node* child : children) {
      if (closed_voxels.at(child->position))
        continue;

      double dist_parent_child = euclideanDistance(current_node->position, child->position);
      double clearance         = params_.replanner_vox_size * clearance_grid->at(child->position);
      double safety_penalty    = params_.weight_safety / (clearance + params_.eps);
      double deviation_penalty =
          params_.weight_deviation * deviationPenalty(_path, current_node->position, child->position);

      // Biases the plan's own start towards continuing in roughly the same heading as the
      // *previous* plan's start, so the direction near the agent doesn't flip-flop between
      // replans even though every plan starts fresh from the agent's actual position (nothing is
      // geometrically locked in place, unlike a frozen prefix would be). Fades out with distance
      // from the start, so only the near-agent portion of the route is nudged -- further out is
      // free to go wherever the goal/obstacles actually require.
      double direction_penalty = 0.0;
      if (have_last_plan_direction_) {
        const Eigen::Vector3d child_world      = gridIdxToWorldCoords(child->position);
        const Eigen::Vector3d to_child         = child_world - start_world;
        const double           dist_from_start = to_child.norm();
        if (dist_from_start > 1e-6) {
          const double cos_angle           = (to_child / dist_from_start).dot(last_plan_direction_);
          const double angular_misalignment = 1.0 - cos_angle;  // 0 aligned .. 2 opposite
          const double fade                 = std::exp(-dist_from_start / kDirectionDecayMeters);
          direction_penalty                 = direction_consistency_weight_ * angular_misalignment * fade;
        }
      }

      child->g = current_node->g + dist_parent_child;
      child->h = euclideanDistance(child->position, end_node->position);
      child->f = child->g + child->h + safety_penalty + deviation_penalty + direction_penalty;

      open_list.push(child);
    }
  }

  for (Node* node : all_allocated_nodes)
    delete node;

  std::cout << "[RBLReplanner]: No path found" << std::endl;

  return {};
}  // //}

double RBLReplanner::deviationPenalty(const std::vector<std::tuple<int,
                                                                   int,
                                                                   int>>& _path,
                                      const std::tuple<int,
                                                       int,
                                                       int>&              _p1,
                                      const std::tuple<int,
                                                       int,
                                                       int>&              _p2)  // //{
{
  for (size_t i = 0; i < _path.size(); ++i) {
    if (_path[i] == _p1) {
      if (i + 1 < _path.size()) {
        if (_path[i + 1] == _p2) {
          return 0.0;
        }
      }
    }
  }
  return 1.0;
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
  std::set<std::tuple<int, int, int>>   visited;

  q.push(clamped_position);
  visited.insert(clamped_position);

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

      if (visited.count(next_pos)) {
        continue;
      }

      if (std::get<0>(next_pos) < 0 || std::get<0>(next_pos) >= grid->X || std::get<1>(next_pos) < 0 ||
          std::get<1>(next_pos) >= grid->Y || std::get<2>(next_pos) < 0 || std::get<2>(next_pos) >= grid->Z) {
        continue;
      }

      if (grid->at(std::get<0>(next_pos), std::get<1>(next_pos), std::get<2>(next_pos)) == 0) {
        return next_pos;  // Found the closest free spot
      }

      visited.insert(next_pos);
      q.push(next_pos);
    }
  }
  return clamped_position;
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

