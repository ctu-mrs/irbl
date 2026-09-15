#include "local_static_map.h"

#include <algorithm>
#include <cmath>

namespace
{
// Packs a 3D voxel index into a single int64 key: each axis biased into [0, 2^21) (supports
// +-1,048,576 cells, i.e. +-104km at a 0.1m voxel -- far beyond any plausible local map extent)
// then bit-packed. Absolute (world-fixed) voxel coordinates, not relative to any local origin, so
// no re-anchoring is ever needed.
constexpr std::int64_t kOffset = 1 << 20;
constexpr int          kBits   = 21;
constexpr std::int64_t kMask   = (std::int64_t{ 1 } << kBits) - 1;

std::int64_t packKey(int ix, int iy, int iz)
{
  const std::int64_t x = static_cast<std::int64_t>(ix) + kOffset;
  const std::int64_t y = static_cast<std::int64_t>(iy) + kOffset;
  const std::int64_t z = static_cast<std::int64_t>(iz) + kOffset;
  return (x << (2 * kBits)) | (y << kBits) | z;
}

void unpackKey(std::int64_t key, int& ix, int& iy, int& iz)
{
  iz = static_cast<int>((key & kMask) - kOffset);
  iy = static_cast<int>(((key >> kBits) & kMask) - kOffset);
  ix = static_cast<int>(((key >> (2 * kBits)) & kMask) - kOffset);
}
}  // namespace

LocalStaticMap::LocalStaticMap(const Params& params) : params_(params)
{
}

std::int64_t LocalStaticMap::voxelKey(const Eigen::Vector3d& p) const
{
  const double vox = params_.voxel_size;
  const int    ix  = static_cast<int>(std::round(p.x() / vox));
  const int    iy  = static_cast<int>(std::round(p.y() / vox));
  const int    iz  = static_cast<int>(std::round(p.z() / vox));
  return packKey(ix, iy, iz);
}

Eigen::Vector3d LocalStaticMap::keyToWorld(std::int64_t key) const
{
  int ix, iy, iz;
  unpackKey(key, ix, iy, iz);
  const double vox = params_.voxel_size;
  return Eigen::Vector3d(ix * vox, iy * vox, iz * vox);
}

void LocalStaticMap::update(const pcl::PointCloud<pcl::PointXYZI>& live_cloud, const Eigen::Vector3d& agent_pos)
{
  const double vox           = params_.voxel_size;
  const double max_range_sq  = params_.max_range * params_.max_range;

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
      occupied_.erase(voxelKey(p));
    }

    // Only add the hit itself if it's within max_range -- no point persisting something we're
    // about to fade out on the very same call anyway.
    if (dist <= params_.max_range) {
      occupied_.insert(voxelKey(target));
    }
  }

  // Gradual fade: drop any occupied cell that has fallen farther than max_range from the agent's
  // *current* position, every call.
  for (auto it = occupied_.begin(); it != occupied_.end();) {
    const Eigen::Vector3d p = keyToWorld(*it);
    if ((p - agent_pos).squaredNorm() > max_range_sq) {
      it = occupied_.erase(it);
    }
    else {
      ++it;
    }
  }
}

pcl::PointCloud<pcl::PointXYZI> LocalStaticMap::getOccupiedCloud() const
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.points.reserve(occupied_.size());

  for (const auto key : occupied_) {
    const Eigen::Vector3d p = keyToWorld(key);
    pcl::PointXYZI         pt;
    pt.x         = static_cast<float>(p.x());
    pt.y         = static_cast<float>(p.y());
    pt.z         = static_cast<float>(p.z());
    pt.intensity = 0.0f;
    cloud.points.push_back(pt);
  }

  cloud.width    = static_cast<std::uint32_t>(cloud.points.size());
  cloud.height   = 1;
  cloud.is_dense = true;
  return cloud;
}
