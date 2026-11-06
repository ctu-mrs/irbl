// Global planner for a ground vehicle on the known map (the obstacle boxes of the world SDF, see
// coverage_planner.hpp loadWorldObstacles): planned once per leg, start -> goal, the whole way.
//
//   grid      2D, `resolution`, over the obstacles' footprint (+ margin). Only obstacles overlapping
//             the height band [z_lo, z_hi] (the vehicle and what it carries) count; the floor and
//             overhangs above the band do not.
//   distance  per cell, the planar distance to the nearest such obstacle (capped). A cell closer than
//             `clearance` is blocked; closer than `preferred_clearance` costs extra
//             (`clearance_weight`), so the path keeps to the middle of aisles where it can.
//   A*        8-connected, octile heuristic. Start / goal on a blocked cell are moved to the nearest
//             free cell within `snap_radius`.
//   smoothing greedy line-of-sight shortcuts that never get closer to the obstacles than the A* path
//             they replace did (capped at preferred_clearance), then resampled every
//             `sample_spacing` (a dense path for rbl_controller's path following).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "rbl_mission/coverage_planner.hpp"

namespace rbl_mission
{

class GlobalPlanner2D
{
public:
  struct Params
  {
    double resolution{0.2};           // [m] grid cell
    double clearance{0.6};            // [m] min planar distance of the path from every obstacle
    double preferred_clearance{1.2};  // [m] closer than this costs extra
    double clearance_weight{2.0};     // cost per metre travelled, per metre below preferred_clearance
    double z_lo{0.05}, z_hi{1.0};     // [m] obstacle height band that blocks the vehicle
    double margin{2.0};               // [m] grid extent beyond the obstacles
    double snap_radius{1.0};          // [m] start / goal may move this far to a free cell
    double sample_spacing{0.5};       // [m] spacing of the returned path
  };

  GlobalPlanner2D(const std::vector<Box>& obstacles, Params p) : p_(p)
  {
    std::vector<Box> band;
    double           x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
    for (const auto& b : obstacles) {
      if (b.hi.z < p_.z_lo || b.lo.z > p_.z_hi) continue;
      band.push_back(b);
      x0 = std::min(x0, b.lo.x), y0 = std::min(y0, b.lo.y);
      x1 = std::max(x1, b.hi.x), y1 = std::max(y1, b.hi.y);
    }
    if (band.empty()) {  // nothing in the way: a small grid, every leg is a straight line
      x0 = y0 = -1.0, x1 = y1 = 1.0;
    }
    ox_ = x0 - p_.margin, oy_ = y0 - p_.margin;
    nx_ = static_cast<int>(std::ceil((x1 - x0 + 2 * p_.margin) / p_.resolution)) + 1;
    ny_ = static_cast<int>(std::ceil((y1 - y0 + 2 * p_.margin) / p_.resolution)) + 1;
    cap_ = std::max(p_.clearance, p_.preferred_clearance) + p_.resolution;
    dist_.assign(static_cast<size_t>(nx_) * ny_, static_cast<float>(cap_));
    for (const auto& b : band) {
      const int i0 = std::max(0, cellX(b.lo.x - cap_)), i1 = std::min(nx_ - 1, cellX(b.hi.x + cap_));
      const int j0 = std::max(0, cellY(b.lo.y - cap_)), j1 = std::min(ny_ - 1, cellY(b.hi.y + cap_));
      for (int i = i0; i <= i1; ++i) {
        for (int j = j0; j <= j1; ++j) {
          const double x = wx(i), y = wy(j);
          const double dx = std::max({b.lo.x - x, 0.0, x - b.hi.x});
          const double dy = std::max({b.lo.y - y, 0.0, y - b.hi.y});
          float&       d  = dist_[idx(i, j)];
          d               = std::min(d, static_cast<float>(std::hypot(dx, dy)));
        }
      }
    }
  }

  const Params& params() const { return p_; }

  // Path from `from` to `to` (z of every point = to.z), or nullopt + reason. If `to` itself is not free
  // the path ends at the free cell it was snapped to (likewise for `from`).
  std::optional<std::vector<Vec3>> plan(const Vec3& from, const Vec3& to, std::string& why) const
  {
    const auto s = snap(from), g = snap(to);
    if (!s) {
      why = "start [" + fmt(from) + "] is not within " + fmt(p_.snap_radius) + " m of a free cell of the map";
      return std::nullopt;
    }
    if (!g) {
      why = "goal [" + fmt(to) + "] is not within " + fmt(p_.snap_radius) + " m of a free cell of the map";
      return std::nullopt;
    }

    // A*
    const size_t       n = dist_.size();
    std::vector<float> gcost(n, std::numeric_limits<float>::infinity());
    std::vector<int>   parent(n, -1);
    std::vector<char>  closed(n, 0);
    using QE = std::pair<float, int>;
    std::priority_queue<QE, std::vector<QE>, std::greater<QE>> open;
    const int gi = g->first, gj = g->second;
    auto      h  = [&](int i, int j) {
      const double dx = std::abs(i - gi), dy = std::abs(j - gj);
      return static_cast<float>(p_.resolution * (std::max(dx, dy) + (M_SQRT2 - 1.0) * std::min(dx, dy)));
    };
    const int start = static_cast<int>(idx(s->first, s->second)), goal = static_cast<int>(idx(gi, gj));
    gcost[start]    = 0.0f;
    open.emplace(h(s->first, s->second), start);
    bool found = false;
    while (!open.empty()) {
      const int c = open.top().second;
      open.pop();
      if (closed[c]) continue;
      closed[c] = 1;
      if (c == goal) {
        found = true;
        break;
      }
      const int ci = c / ny_, cj = c % ny_;
      for (int di = -1; di <= 1; ++di) {
        for (int dj = -1; dj <= 1; ++dj) {
          if (!di && !dj) continue;
          const int i = ci + di, j = cj + dj;
          if (i < 0 || j < 0 || i >= nx_ || j >= ny_) continue;
          const int k = static_cast<int>(idx(i, j));
          if (closed[k] || !free(k)) continue;
          const double step = p_.resolution * ((di && dj) ? M_SQRT2 : 1.0);
          const double pen  = p_.clearance_weight * std::max(0.0, p_.preferred_clearance - dist_[k]);
          const float  gn   = gcost[c] + static_cast<float>(step * (1.0 + pen));
          if (gn < gcost[k]) {
            gcost[k]  = gn;
            parent[k] = c;
            open.emplace(gn + h(i, j), k);
          }
        }
      }
    }
    if (!found) {
      why = "no free path on the map from [" + fmt(from) + "] to [" + fmt(to) + "]";
      return std::nullopt;
    }

    std::vector<std::pair<int, int>> cells;
    for (int c = goal; c != -1; c = parent[c]) cells.emplace_back(c / ny_, c % ny_);
    std::reverse(cells.begin(), cells.end());

    // line-of-sight shortcuts, never closer to the obstacles than the A* cells they replace
    std::vector<std::pair<int, int>> keep{cells.front()};
    size_t                           a = 0;
    while (a + 1 < cells.size()) {
      size_t b       = a + 1;
      float  min_sub = std::min(dist_[idx(cells[a])], dist_[idx(cells[b])]);
      size_t best    = b;
      for (size_t k = b + 1; k < cells.size(); ++k) {
        min_sub = std::min(min_sub, dist_[idx(cells[k])]);
        const float need = std::min(min_sub, static_cast<float>(p_.preferred_clearance));
        if (lineClear(cells[a], cells[k], need)) best = k;
      }
      keep.push_back(cells[best]);
      a = best;
    }

    // exact start / goal at the ends if they are free themselves, else the free cells they were
    // snapped to (the path never leads into an obstacle); resampled
    auto end_point = [&](const Vec3& exact, const std::pair<int, int>& cell) {
      return clearanceAt(exact) >= p_.clearance ? Vec3{exact.x, exact.y, to.z}
                                                : Vec3{wx(cell.first), wy(cell.second), to.z};
    };
    std::vector<Vec3> corners{end_point(from, keep.front())};
    for (size_t k = 1; k + 1 < keep.size(); ++k) corners.push_back(Vec3{wx(keep[k].first), wy(keep[k].second), to.z});
    corners.push_back(end_point(to, keep.back()));
    std::vector<Vec3> out{corners.front()};
    for (size_t k = 1; k < corners.size(); ++k) {
      const Vec3&  p0 = corners[k - 1];
      const Vec3&  p1 = corners[k];
      const double L  = std::hypot(p1.x - p0.x, p1.y - p0.y);
      const int    m  = std::max(1, static_cast<int>(std::ceil(L / p_.sample_spacing)));
      for (int t = 1; t <= m; ++t) {
        const double u = static_cast<double>(t) / m;
        out.push_back(Vec3{p0.x + (p1.x - p0.x) * u, p0.y + (p1.y - p0.y) * u, to.z});
      }
    }
    return out;
  }

  static double length(const std::vector<Vec3>& path)
  {
    double L = 0;
    for (size_t k = 1; k < path.size(); ++k) L += std::hypot(path[k].x - path[k - 1].x, path[k].y - path[k - 1].y);
    return L;
  }

  // planar distance from p to the nearest obstacle of the band (capped)
  double clearanceAt(const Vec3& p) const
  {
    const int i = cellX(p.x), j = cellY(p.y);
    if (i < 0 || j < 0 || i >= nx_ || j >= ny_) return cap_;
    return dist_[idx(i, j)];
  }

private:
  int    cellX(double x) const { return static_cast<int>(std::floor((x - ox_) / p_.resolution + 0.5)); }
  int    cellY(double y) const { return static_cast<int>(std::floor((y - oy_) / p_.resolution + 0.5)); }
  double wx(int i) const { return ox_ + i * p_.resolution; }
  double wy(int j) const { return oy_ + j * p_.resolution; }
  size_t idx(int i, int j) const { return static_cast<size_t>(i) * ny_ + j; }
  size_t idx(const std::pair<int, int>& c) const { return idx(c.first, c.second); }
  bool   free(size_t k) const { return dist_[k] >= p_.clearance; }

  std::optional<std::pair<int, int>> snap(const Vec3& p) const
  {
    const int ci = cellX(p.x), cj = cellY(p.y);
    const int r  = static_cast<int>(std::ceil(p_.snap_radius / p_.resolution));
    std::optional<std::pair<int, int>> best;
    int                                best_d2 = std::numeric_limits<int>::max();
    for (int i = ci - r; i <= ci + r; ++i) {
      for (int j = cj - r; j <= cj + r; ++j) {
        if (i < 0 || j < 0 || i >= nx_ || j >= ny_ || !free(idx(i, j))) continue;
        const int d2 = (i - ci) * (i - ci) + (j - cj) * (j - cj);
        if (d2 < best_d2 && d2 <= r * r) {
          best_d2 = d2;
          best    = std::make_pair(i, j);
        }
      }
    }
    return best;
  }

  // every cell on the straight line a-b (sampled every half cell) has dist >= need and is free
  bool lineClear(const std::pair<int, int>& a, const std::pair<int, int>& b, float need) const
  {
    const double di = b.first - a.first, dj = b.second - a.second;
    const int    m  = std::max(1, static_cast<int>(std::ceil(2.0 * std::max(std::abs(di), std::abs(dj)))));
    for (int t = 0; t <= m; ++t) {
      const double u = static_cast<double>(t) / m;
      const int    i = static_cast<int>(std::lround(a.first + di * u));
      const int    j = static_cast<int>(std::lround(a.second + dj * u));
      const size_t k = idx(i, j);
      if (!free(k) || dist_[k] < need) return false;
    }
    return true;
  }

  static std::string fmt(const Vec3& p)
  {
    char b[64];
    std::snprintf(b, sizeof(b), "%.1f, %.1f", p.x, p.y);
    return b;
  }
  static std::string fmt(double d)
  {
    char b[32];
    std::snprintf(b, sizeof(b), "%.1f", d);
    return b;
  }

  Params             p_;
  double             ox_{0}, oy_{0}, cap_{1};
  int                nx_{1}, ny_{1};
  std::vector<float> dist_;
};

}  // namespace rbl_mission
