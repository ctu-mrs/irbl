#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <vector>

// A small, bounded local occupancy map of static obstacles, persisted across cycles -- simpler
// than octomap and independent of rbl_replanner's own grid. update() marks every point in a live
// scan as occupied, and carves free space along the ray from the agent to each point (clearing
// any previously-marked cell that ray passes through before reaching its target), so a since-moved
// or spurious obstacle doesn't linger forever. The window is fixed-size and only re-anchors (which
// fully resets it -- the simplest correct bounded-memory behavior) once the agent nears its edge,
// same "stable, snap-to-lattice" approach used elsewhere in this stack to avoid quantization jitter.
class LocalStaticMap
{
public:
  struct Params
  {
    double voxel_size = 0.3;   // [m]
    double width       = 30.0;  // [m] X/Y extent of the local window
    double height       = 10.0;  // [m] Z extent of the local window
  };

  explicit LocalStaticMap(const Params& params);

  // live_cloud must already be in the same world/control_frame as agent_pos.
  void update(const pcl::PointCloud<pcl::PointXYZI>& live_cloud, const Eigen::Vector3d& agent_pos);

  // One point (at cell center, low/zero intensity -- a static obstacle, not a reflective
  // other-agent marker) per currently-occupied cell.
  pcl::PointCloud<pcl::PointXYZI> getOccupiedCloud() const;

private:
  Params                params_;
  int                   nx_, ny_, nz_;
  std::vector<uint8_t>  occupied_;
  Eigen::Vector3d       anchor_world_ = Eigen::Vector3d::Zero();
  bool                  have_anchor_  = false;

  int             index(int ix, int iy, int iz) const;
  bool            worldToIdx(const Eigen::Vector3d& p, int& ix, int& iy, int& iz) const;
  Eigen::Vector3d idxToWorld(int ix, int iy, int iz) const;
  void            maybeReanchor(const Eigen::Vector3d& agent_pos);
};
