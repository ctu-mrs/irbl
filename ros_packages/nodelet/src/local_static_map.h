#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <unordered_set>

// A small local occupancy map of static obstacles, persisted across cycles -- simpler than octomap
// and independent of rbl_replanner's own grid. update() marks every point in a live scan as
// occupied, and carves free space along the ray from the agent to each point (clearing any
// previously-marked cell that ray passes through before reaching its target), so a since-moved or
// spurious obstacle doesn't linger forever. Occupied voxels are keyed by absolute world coordinates
// (not a re-anchored local window), so there is no fixed "square" boundary that could reject a
// point that's genuinely within range of the agent -- inclusion is purely "distance to the agent
// <= max_range", checked fresh every update() call, which also means the far edge fades out
// gradually as the agent moves instead of the map being wiped in one shot.
class LocalStaticMap
{
public:
  struct Params
  {
    double voxel_size = 0.3;   // [m]
    // [m] Any occupied cell farther than this from the *current* agent position is dropped every
    // update() call, and a live-scan hit farther than this is never added in the first place.
    double max_range   = 15.0;
  };

  explicit LocalStaticMap(const Params& params);

  // live_cloud must already be in the same world/control_frame as agent_pos.
  void update(const pcl::PointCloud<pcl::PointXYZI>& live_cloud, const Eigen::Vector3d& agent_pos);

  // One point (at cell center, low/zero intensity -- a static obstacle, not a reflective
  // other-agent marker) per currently-occupied cell.
  pcl::PointCloud<pcl::PointXYZI> getOccupiedCloud() const;

private:
  Params                            params_;
  std::unordered_set<std::int64_t>  occupied_;

  std::int64_t    voxelKey(const Eigen::Vector3d& p) const;
  Eigen::Vector3d keyToWorld(std::int64_t key) const;
};
