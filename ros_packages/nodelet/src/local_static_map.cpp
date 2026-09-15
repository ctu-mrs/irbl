#include "local_static_map.h"

#include <algorithm>
#include <cmath>

LocalStaticMap::LocalStaticMap(const Params& params) : params_(params)
{
  nx_ = std::max(1, static_cast<int>(std::ceil(params_.width / params_.voxel_size)));
  ny_ = nx_;
  nz_ = std::max(1, static_cast<int>(std::ceil(params_.height / params_.voxel_size)));
  occupied_.assign(static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_) * static_cast<std::size_t>(nz_), 0);
}

int LocalStaticMap::index(int ix, int iy, int iz) const
{
  return (ix * ny_ + iy) * nz_ + iz;
}

void LocalStaticMap::maybeReanchor(const Eigen::Vector3d& agent_pos)
{
  const double vox               = params_.voxel_size;
  const double margin_cells      = 3.0;
  bool         need_new_anchor   = !have_anchor_;

  if (have_anchor_) {
    const double dx_cells = (agent_pos.x() - anchor_world_.x()) / vox;
    const double dy_cells = (agent_pos.y() - anchor_world_.y()) / vox;
    const double dz_cells = (agent_pos.z() - anchor_world_.z()) / vox;
    if (std::abs(dx_cells) > (nx_ / 2.0 - margin_cells) || std::abs(dy_cells) > (ny_ / 2.0 - margin_cells) ||
        std::abs(dz_cells) > (nz_ / 2.0 - margin_cells)) {
      need_new_anchor = true;
    }
  }

  if (need_new_anchor) {
    anchor_world_ = Eigen::Vector3d(std::round(agent_pos.x() / vox) * vox, std::round(agent_pos.y() / vox) * vox,
                                    std::round(agent_pos.z() / vox) * vox);
    have_anchor_  = true;
    std::fill(occupied_.begin(), occupied_.end(), 0);
  }
}

bool LocalStaticMap::worldToIdx(const Eigen::Vector3d& p, int& ix, int& iy, int& iz) const
{
  const double vox = params_.voxel_size;
  ix                = static_cast<int>(std::round((p.x() - anchor_world_.x()) / vox)) + nx_ / 2;
  iy                = static_cast<int>(std::round((p.y() - anchor_world_.y()) / vox)) + ny_ / 2;
  iz                = static_cast<int>(std::round((p.z() - anchor_world_.z()) / vox)) + nz_ / 2;
  return ix >= 0 && ix < nx_ && iy >= 0 && iy < ny_ && iz >= 0 && iz < nz_;
}

Eigen::Vector3d LocalStaticMap::idxToWorld(int ix, int iy, int iz) const
{
  const double vox = params_.voxel_size;
  return Eigen::Vector3d((ix - nx_ / 2) * vox + anchor_world_.x(), (iy - ny_ / 2) * vox + anchor_world_.y(),
                         (iz - nz_ / 2) * vox + anchor_world_.z());
}

void LocalStaticMap::update(const pcl::PointCloud<pcl::PointXYZI>& live_cloud, const Eigen::Vector3d& agent_pos)
{
  maybeReanchor(agent_pos);

  const double vox = params_.voxel_size;

  for (const auto& pt : live_cloud.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }

    const Eigen::Vector3d target(pt.x, pt.y, pt.z);
    const Eigen::Vector3d ray  = target - agent_pos;
    const double          dist = ray.norm();
    if (dist < 1e-6) {
      continue;
    }

    // Carve free space along the ray up to (but not including) the hit, clearing any cell there
    // that a previous scan marked occupied.
    const int steps = std::max(1, static_cast<int>(dist / vox));
    for (int s = 0; s < steps; ++s) {
      const double           t = static_cast<double>(s) / steps;
      const Eigen::Vector3d  p = agent_pos + ray * t;
      int                     ix, iy, iz;
      if (worldToIdx(p, ix, iy, iz)) {
        occupied_[static_cast<std::size_t>(index(ix, iy, iz))] = 0;
      }
    }

    int ix, iy, iz;
    if (worldToIdx(target, ix, iy, iz)) {
      occupied_[static_cast<std::size_t>(index(ix, iy, iz))] = 1;
    }
  }
}

pcl::PointCloud<pcl::PointXYZI> LocalStaticMap::getOccupiedCloud() const
{
  pcl::PointCloud<pcl::PointXYZI> cloud;

  for (int ix = 0; ix < nx_; ++ix) {
    for (int iy = 0; iy < ny_; ++iy) {
      for (int iz = 0; iz < nz_; ++iz) {
        if (occupied_[static_cast<std::size_t>(index(ix, iy, iz))]) {
          const Eigen::Vector3d p = idxToWorld(ix, iy, iz);
          pcl::PointXYZI         pt;
          pt.x         = static_cast<float>(p.x());
          pt.y         = static_cast<float>(p.y());
          pt.z         = static_cast<float>(p.z());
          pt.intensity = 0.0f;
          cloud.points.push_back(pt);
        }
      }
    }
  }

  cloud.width    = static_cast<std::uint32_t>(cloud.points.size());
  cloud.height   = 1;
  cloud.is_dense = true;
  return cloud;
}
