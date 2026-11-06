// Shelf coverage planner for the warehouse inspection mission (no ROS in here, so the node and the
// offline checker share exactly the same code).
//
// Input: a prior map of the warehouse (aisle centrelines, rack rows, rack segments along x) plus
// every collision shape of the Gazebo world (SDF), an aisle position, the shelf side and the UAV
// start position. Output: entry goal, coverage goals and exit goal, all at least `clearance` from
// every obstacle (and every straight segment between consecutive coverage goals checked too).
//
// Coverage = boustrophedon over one rack face: one pass along x per level, climbing at the ends.
// Each pass is sampled every `sample_step` m; every sample is placed, in this order of preference:
//   1. on the scan lane (rack face + standoff, at the level height),
//   2. shifted sideways inside the lane band (standoff in [clearance, standoff_max]) -- the
//      smallest shift that is clear,
//   3. at a nearby height (level +- search_dz), again with the smallest sideways shift,
//   4. hopping over (any height up to z_max) -- e.g. over a pallet stack standing in the lane,
//   5. blocked (skipped; reported).
// The samples are then reduced to goals: one every `waypoint_spacing` m on straight stretches, plus
// a goal at both ends of every deviation; any reduced segment that is not clear gets its original
// samples back. Sliding along the pass is never used, so the scan always covers the whole face.
// Finally every leg of entry -> coverage goals -> exit (approach, level changes, gaps over blocked
// samples, leaving the aisle) that is still not clear is routed with an A* search on a 3D grid
// inside the aisle (keeping `clearance`), smoothed by line of sight.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <cstdio>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <tinyxml2.h>

namespace rbl_mission
{

struct Vec3
{
  double x{0}, y{0}, z{0};
};

inline double dist(const Vec3& a, const Vec3& b)
{
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

// ------------------------------------------------------------------------------------------------
// obstacles from the world SDF
// ------------------------------------------------------------------------------------------------

struct Box  // axis-aligned bounding box in the world frame
{
  Vec3 lo, hi;
};

inline double clearance(const Vec3& p, const Box& b)  // 0 inside
{
  const double dx = std::max({b.lo.x - p.x, 0.0, p.x - b.hi.x});
  const double dy = std::max({b.lo.y - p.y, 0.0, p.y - b.hi.y});
  const double dz = std::max({b.lo.z - p.z, 0.0, p.z - b.hi.z});
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

struct Pose
{
  std::array<std::array<double, 3>, 3> R{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
  Vec3                                 t;

  static Pose parse(const tinyxml2::XMLElement* parent)
  {
    Pose        p;
    const auto* e = parent ? parent->FirstChildElement("pose") : nullptr;
    if (!e || !e->GetText()) {
      return p;
    }
    double             v[6] = {0, 0, 0, 0, 0, 0};
    std::istringstream ss(e->GetText());
    for (double& x : v) {
      ss >> x;
    }
    p.t = Vec3{v[0], v[1], v[2]};
    const double cr = std::cos(v[3]), sr = std::sin(v[3]), cp = std::cos(v[4]), sp = std::sin(v[4]),
                 cy = std::cos(v[5]), sy = std::sin(v[5]);
    p.R = {{{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
            {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
            {-sp, cp * sr, cp * cr}}};  // Rz(yaw) * Ry(pitch) * Rx(roll), SDF convention
    return p;
  }

  Pose operator*(const Pose& o) const  // this o o
  {
    Pose r;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        r.R[i][j] = R[i][0] * o.R[0][j] + R[i][1] * o.R[1][j] + R[i][2] * o.R[2][j];
      }
    }
    r.t = Vec3{R[0][0] * o.t.x + R[0][1] * o.t.y + R[0][2] * o.t.z + t.x,
               R[1][0] * o.t.x + R[1][1] * o.t.y + R[1][2] * o.t.z + t.y,
               R[2][0] * o.t.x + R[2][1] * o.t.y + R[2][2] * o.t.z + t.z};
    return r;
  }

  Box aabb(const Vec3& half) const  // world AABB of a box with these local half extents
  {
    const double h[3] = {half.x, half.y, half.z};
    double       e[3];
    for (int i = 0; i < 3; ++i) {
      e[i] = std::abs(R[i][0]) * h[0] + std::abs(R[i][1]) * h[1] + std::abs(R[i][2]) * h[2];
    }
    return Box{Vec3{t.x - e[0], t.y - e[1], t.z - e[2]}, Vec3{t.x + e[0], t.y + e[1], t.z + e[2]}};
  }
};

// Collision boxes / cylinders / spheres of every <model><link><collision> (nested models too).
// Planes (ground) are skipped, meshes are counted as unsupported.
inline void collectModel(const tinyxml2::XMLElement* model, const Pose& parent, std::vector<Box>& out,
                         int& unsupported)
{
  const Pose pm = parent * Pose::parse(model);
  for (auto* sub = model->FirstChildElement("model"); sub; sub = sub->NextSiblingElement("model")) {
    collectModel(sub, pm, out, unsupported);
  }
  for (auto* link = model->FirstChildElement("link"); link; link = link->NextSiblingElement("link")) {
    const Pose pl = pm * Pose::parse(link);
    for (auto* col = link->FirstChildElement("collision"); col; col = col->NextSiblingElement("collision")) {
      const Pose  pc   = pl * Pose::parse(col);
      const auto* geom = col->FirstChildElement("geometry");
      if (!geom) {
        continue;
      }
      if (const auto* b = geom->FirstChildElement("box")) {
        Vec3 sz{1, 1, 1};
        if (const auto* e = b->FirstChildElement("size"); e && e->GetText()) {
          std::istringstream ss(e->GetText());
          ss >> sz.x >> sz.y >> sz.z;
        }
        out.push_back(pc.aabb(Vec3{sz.x / 2, sz.y / 2, sz.z / 2}));
      } else if (const auto* c = geom->FirstChildElement("cylinder")) {
        double r = 0.5, l = 1.0;
        if (const auto* e = c->FirstChildElement("radius")) e->QueryDoubleText(&r);
        if (const auto* e = c->FirstChildElement("length")) e->QueryDoubleText(&l);
        out.push_back(pc.aabb(Vec3{r, r, l / 2}));
      } else if (const auto* sp = geom->FirstChildElement("sphere")) {
        double r = 0.5;
        if (const auto* e = sp->FirstChildElement("radius")) e->QueryDoubleText(&r);
        out.push_back(pc.aabb(Vec3{r, r, r}));
      } else if (!geom->FirstChildElement("plane")) {
        ++unsupported;
      }
    }
  }
}

// Top-level models whose name starts with this are *temporary* obstacles (e.g. the red crates in the
// warehouse main aisle): really in the gz world, so the onboard sensors / rbl see them, but not part
// of the prior map, so the global and coverage planners don't route around them.
inline constexpr const char* kTemporaryModelPrefix = "temp_obstacle";

// Throws on unreadable files. `unsupported` = collision shapes that could not be represented.
// Returns the prior map (static obstacles only). The temporary obstacles go to `*temporary` if given,
// else are dropped.
inline std::vector<Box> loadWorldObstacles(const std::string& sdf_path, int& unsupported,
                                           std::vector<Box>* temporary = nullptr)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(sdf_path.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("cannot read world SDF '" + sdf_path + "': " + doc.ErrorStr());
  }
  const auto* sdf   = doc.FirstChildElement("sdf");
  const auto* world = sdf ? sdf->FirstChildElement("world") : nullptr;
  if (!world) {
    throw std::runtime_error("world SDF '" + sdf_path + "' has no <sdf><world>");
  }
  std::vector<Box> out;
  unsupported = 0;
  std::vector<Box> tmp;
  for (auto* m = world->FirstChildElement("model"); m; m = m->NextSiblingElement("model")) {
    const char* name = m->Attribute("name");
    const bool  is_tmp =
        name && std::string(name).rfind(kTemporaryModelPrefix, 0) == 0;
    collectModel(m, Pose{}, is_tmp ? tmp : out, unsupported);
  }
  if (temporary) {
    *temporary = std::move(tmp);
  }
  if (world->FirstChildElement("include")) {
    ++unsupported;
  }
  return out;
}

// ------------------------------------------------------------------------------------------------
// planner
// ------------------------------------------------------------------------------------------------

class CoveragePlanner
{
public:
  struct Params
  {
    // map
    double              rack_depth{1.2};
    double              rack_height{4.5};        // incl. what stands on the top shelf
    double              aisle_half_width{1.5};   // a request must be this close to an aisle centreline
    double              cross_aisle_reach{2.0};  // ... and at most this far (x) outside a rack segment
    std::vector<double> aisle_y, aisle_left_row, aisle_right_row, segment_x_min, segment_x_max;
    std::vector<double> cross_aisle_x;           // passages across the rack rows (incl. the end strips)
    // coverage
    double              standoff{1.2};      // nominal distance scan lane <-> rack face
    double              standoff_max{1.8};  // lane band: samples may move away from the face up to this
    std::vector<double> levels{1.0, 2.2, 3.4};
    double              end_margin{1.0};
    double              waypoint_spacing{3.0};
    double              sample_step{0.25};
    double              clearance{0.7};
    double              search_dz{0.6};
    double              entry_search{1.5};  // how far (x, along the aisle) entry/exit goals may move
    double              z_min{0.6}, z_max{4.4};
  };

  enum class SampleKind
  {
    LANE,
    SHIFTED,
    HOPPED,
    BLOCKED
  };

  struct Plan
  {
    bool        ok{false};
    std::string error;
    int         aisle{-1}, segment{-1};  // 0-based
    bool        left{true};
    double      lane_y{0}, x_near{0}, x_far{0};
    Vec3        entry, exit;
    std::vector<Vec3> approach; // via goals entry -> first coverage goal (only if the direct leg is blocked)
    std::vector<Vec3> scan;     // coverage goals, in flight order (incl. routed via goals)
    std::vector<Vec3> leave;    // via goals last coverage goal -> exit (only if blocked)
    std::vector<Vec3> dense; // all placed samples (debug / visualisation)
    // statistics
    size_t samples{0}, shifted{0}, hopped{0}, blocked{0};
    double max_lane_dev{0};    // [m] largest |y - lane_y| of a coverage goal
    double max_level_dev{0};   // [m] largest |z - level| of a coverage goal
    double min_clearance{1e9}; // [m] along the whole entry -> coverage -> exit polyline
    Vec3   worst_from, worst_to;  // segment with min_clearance
    size_t routed{0};          // legs that needed the grid search
    std::vector<std::string> notes;
  };

  CoveragePlanner(Params p, std::vector<Box> obstacles) : p_(std::move(p)), obstacles_(std::move(obstacles))
  {
    const auto n = p_.aisle_y.size();
    if (n == 0 || p_.aisle_left_row.size() != n || p_.aisle_right_row.size() != n || p_.segment_x_min.empty() ||
        p_.segment_x_min.size() != p_.segment_x_max.size()) {
      throw std::runtime_error("map: empty or mismatched aisle / segment arrays");
    }
    if (p_.levels.empty() || p_.waypoint_spacing <= 0 || p_.sample_step <= 0 || p_.clearance < 0 ||
        p_.standoff_max < p_.standoff || p_.z_min >= p_.z_max) {
      throw std::runtime_error("coverage: invalid parameters");
    }
    if (p_.z_max >= p_.rack_height - p_.clearance) {
      throw std::runtime_error("coverage.z_max must stay below map.rack_height - coverage.clearance: the UAV "
                               "must not fly over the racks");
    }
    for (double z : p_.levels) {
      if (z < p_.z_min || z > p_.z_max) {
        throw std::runtime_error("coverage.levels must lie inside [coverage.z_min, coverage.z_max]");
      }
    }
  }

  const Params& params() const { return p_; }

  // Route for a ground vehicle along the aisle grid: [target] if it is in the same aisle, else via
  // the cross-aisle minimising |from.x - x_c| + |to.x - x_c|: (x_c, from's aisle y) -> (x_c, to's
  // aisle y) -> target. Every intermediate point is moved to a free spot (groundStop with `clear`).
  std::vector<Vec3> groundRoute(const Vec3& from, const Vec3& to, double clear) const
  {
    auto aisle_of = [&](double y) {
      size_t a = 0;
      for (size_t i = 1; i < p_.aisle_y.size(); ++i) {
        if (std::abs(p_.aisle_y[i] - y) < std::abs(p_.aisle_y[a] - y)) a = i;
      }
      return a;
    };
    const size_t a0 = aisle_of(from.y), a1 = aisle_of(to.y);
    if (a0 == a1 || p_.cross_aisle_x.empty()) return {to};
    double xc = p_.cross_aisle_x.front();
    for (double c : p_.cross_aisle_x) {
      if (std::abs(from.x - c) + std::abs(to.x - c) < std::abs(from.x - xc) + std::abs(to.x - xc)) xc = c;
    }
    std::vector<Vec3> route;
    for (double y : {p_.aisle_y[a0], p_.aisle_y[a1]}) {
      std::string why;
      route.push_back(groundStop(xc, y, to.z, clear, why).value_or(Vec3{xc, y, to.z}));
    }
    route.push_back(to);
    return route;
  }

  // Free spot for a ground vehicle at height z near (x, y): the request must be inside an aisle
  // (same rules as plan()), the result stays inside that aisle, within entry_search along it, at
  // least `clear` from every obstacle at height z (planar). nullopt + reason otherwise.
  std::optional<Vec3> groundStop(double x, double y, double z, double clear, std::string& why) const
  {
    size_t a = 0;
    for (size_t i = 1; i < p_.aisle_y.size(); ++i) {
      if (std::abs(p_.aisle_y[i] - y) < std::abs(p_.aisle_y[a] - y)) a = i;
    }
    if (std::abs(p_.aisle_y[a] - y) > p_.aisle_half_width) {
      why = fmt("y=%.2f is not inside any aisle of the map", y);
      return std::nullopt;
    }
    double best_dx = 1e9;
    for (size_t i = 0; i < p_.segment_x_min.size(); ++i) {
      best_dx = std::min(best_dx, std::max({p_.segment_x_min[i] - x, x - p_.segment_x_max[i], 0.0}));
    }
    if (best_dx > p_.cross_aisle_reach) {
      why = fmt("x=%.2f is %.1f m outside the nearest rack segment", x, best_dx);
      return std::nullopt;
    }
    const double lo = p_.aisle_y[a] - p_.aisle_half_width, hi = p_.aisle_y[a] + p_.aisle_half_width;
    const auto   obs = nearby(x - p_.entry_search - 2.0, x + p_.entry_search + 2.0, lo - 2.0, hi + 2.0);
    auto planar = [&](const Vec3& q) {
      double c = 1e9;
      for (const auto& b : obs) {
        if (b.hi.z < 0.02) continue;  // floor-level slabs (e.g. ground markings) don't block
        const double dx = std::max({b.lo.x - q.x, 0.0, q.x - b.hi.x});
        const double dy = std::max({b.lo.y - q.y, 0.0, q.y - b.hi.y});
        c = std::min(c, std::sqrt(dx * dx + dy * dy));
      }
      return c;
    };
    std::optional<Vec3> best;
    double              best_cost = 1e9;
    const double        step      = 0.1;
    const int           nx = static_cast<int>(std::round(p_.entry_search / step));
    const int           ny = static_cast<int>(std::round((hi - lo) / step));
    for (int ix = -nx; ix <= nx; ++ix) {
      for (int iy = 0; iy <= ny; ++iy) {
        const Vec3   q{x + ix * step, lo + iy * step, z};
        const double cost = std::abs(q.x - x) + std::abs(q.y - y);
        if (cost < best_cost && planar(q) >= clear) {
          best      = q;
          best_cost = cost;
        }
      }
    }
    if (!best) why = "no free spot for the ground vehicle near the requested position";
    return best;
  }
  size_t        obstacleCount() const { return obstacles_.size(); }
  const std::vector<Box>& obstacles() const { return obstacles_; }

  // aisle_pos: requested aisle position (z <= 0 -> first level); start: where the UAV is now (the
  // mission returns there).
  Plan plan(const Vec3& aisle_pos, bool left, const Vec3& start) const
  {
    Plan pl;
    pl.left = left;

    // ---- aisle / segment ----------------------------------------------------------------------
    size_t a = 0;
    for (size_t i = 1; i < p_.aisle_y.size(); ++i) {
      if (std::abs(p_.aisle_y[i] - aisle_pos.y) < std::abs(p_.aisle_y[a] - aisle_pos.y)) a = i;
    }
    if (std::abs(p_.aisle_y[a] - aisle_pos.y) > p_.aisle_half_width) {
      pl.error = fmt("y=%.2f is not inside any aisle of the map", aisle_pos.y);
      return pl;
    }
    size_t s = 0;
    double best_dx = 1e9;
    for (size_t i = 0; i < p_.segment_x_min.size(); ++i) {
      const double dx = std::max({p_.segment_x_min[i] - aisle_pos.x, aisle_pos.x - p_.segment_x_max[i], 0.0});
      if (dx < best_dx) {
        best_dx = dx;
        s       = i;
      }
    }
    if (best_dx > p_.cross_aisle_reach) {
      pl.error = fmt("x=%.2f is %.1f m outside the nearest rack segment (max %.1f)", aisle_pos.x, best_dx,
                     p_.cross_aisle_reach);
      return pl;
    }
    pl.aisle   = static_cast<int>(a);
    pl.segment = static_cast<int>(s);

    const double half   = p_.rack_depth / 2.0;
    const double face_y = left ? p_.aisle_left_row[a] - half : p_.aisle_right_row[a] + half;
    const double away   = left ? -1.0 : 1.0;  // from the face into the aisle
    pl.lane_y           = face_y + away * p_.standoff;

    const double x_lo    = p_.segment_x_min[s] + p_.end_margin;
    const double x_hi    = p_.segment_x_max[s] - p_.end_margin;
    const bool   lo_near = std::abs(aisle_pos.x - x_lo) <= std::abs(aisle_pos.x - x_hi);
    pl.x_near            = lo_near ? x_lo : x_hi;
    pl.x_far             = lo_near ? x_hi : x_lo;

    const double band_lo = p_.aisle_y[a] - p_.aisle_half_width, band_hi = p_.aisle_y[a] + p_.aisle_half_width;
    const auto   obs     = nearby(std::min(x_lo, aisle_pos.x) - p_.entry_search - 3.0,
                                  std::max(x_hi, aisle_pos.x) + p_.entry_search + 3.0, band_lo - 3.0, band_hi + 3.0);

    // ---- entry ---------------------------------------------------------------------------------
    const Vec3 entry_raw{aisle_pos.x, aisle_pos.y,
                         aisle_pos.z > 0.0 ? std::clamp(aisle_pos.z, p_.z_min, p_.z_max) : p_.levels.front()};
    const auto entry = freeNear(entry_raw, obs, band_lo, band_hi);
    if (!entry) {
      pl.error = "no free spot (coverage.clearance) near the requested aisle position";
      return pl;
    }
    pl.entry = *entry;
    if (dist(*entry, entry_raw) > 1e-6) {
      pl.notes.push_back(fmt("aisle position [%.1f, %.1f, %.1f] too close to an obstacle -> [%.1f, %.1f, %.1f]",
                             entry_raw.x, entry_raw.y, entry_raw.z, entry->x, entry->y, entry->z));
    }

    // ---- dense samples, level by level (boustrophedon) -----------------------------------------
    const double s_min = std::min(p_.clearance, p_.standoff), s_max = p_.standoff_max;
    const int    n     = std::max(1, static_cast<int>(std::ceil(std::abs(pl.x_far - pl.x_near) / p_.sample_step)));
    std::vector<std::vector<Vec3>> passes;
    for (size_t k = 0; k < p_.levels.size(); ++k) {
      const double level = p_.levels[k];
      std::vector<double>              xs;
      std::vector<std::optional<Vec3>> qs;
      std::vector<SampleKind>          kinds;
      for (int i = 0; i <= n; ++i) {
        const int    j = (k % 2 == 0) ? i : n - i;
        const double x = pl.x_near + (pl.x_far - pl.x_near) * j / n;
        SampleKind   kind;
        qs.push_back(placeSample(x, level, face_y, away, s_min, s_max, obs, kind));
        xs.push_back(x);
        kinds.push_back(kind);
      }
      // one consistent detour per run of off-lane samples (per-sample choices zig-zag at the rounded
      // corners of an obstacle: shift, back, hop, ...)
      for (size_t i = 0; i < kinds.size();) {
        if (kinds[i] == SampleKind::LANE) {
          ++i;
          continue;
        }
        size_t e = i;
        while (e + 1 < kinds.size() && kinds[e + 1] != SampleKind::LANE) ++e;
        if (auto d = uniformDetour(xs, i, e, level, face_y, away, s_min, s_max, obs)) {
          for (size_t t = i; t <= e; ++t) {
            qs[t]    = Vec3{xs[t], face_y + away * d->first, d->second};
            kinds[t] = std::abs(d->second - level) > p_.search_dz + 1e-9 ? SampleKind::HOPPED : SampleKind::SHIFTED;
          }
        }
        i = e + 1;
      }
      std::vector<Vec3> pass;
      for (size_t i = 0; i < qs.size(); ++i) {
        ++pl.samples;
        if (kinds[i] == SampleKind::SHIFTED) ++pl.shifted;
        if (kinds[i] == SampleKind::HOPPED) ++pl.hopped;
        if (!qs[i]) {
          ++pl.blocked;
          continue;
        }
        pass.push_back(*qs[i]);
        pl.dense.push_back(*qs[i]);
      }
      if (pass.empty()) {
        pl.notes.push_back(fmt("level %.1f m: every sample blocked, level skipped", p_.levels[k]));
        continue;
      }
      passes.push_back(std::move(pass));
    }
    if (passes.empty()) {
      pl.error = "every coverage sample of this shelf is blocked";
      return pl;
    }
    if (pl.blocked > 0) {
      pl.notes.push_back(fmt("%zu of %zu samples blocked (no free spot in the lane band or above)", pl.blocked,
                             pl.samples));
    }

    // ---- reduce to goals, re-insert samples where a reduced segment is not clear ---------------
    for (const auto& pass : passes) {
      const auto idx = reduce(pass);
      std::vector<Vec3> goals{pass[idx.front()]};
      for (size_t m = 1; m < idx.size(); ++m) {
        const Vec3& prev = pass[idx[m - 1]];
        const Vec3& cur  = pass[idx[m]];
        if (segmentClearance(prev, cur, obs) < p_.clearance - kSegTol) {
          for (size_t t = idx[m - 1] + 1; t < idx[m]; ++t) goals.push_back(pass[t]);
        }
        goals.push_back(cur);
      }
      pl.scan.insert(pl.scan.end(), goals.begin(), goals.end());
    }
    // ---- exit: back on the aisle centreline where the coverage ended, at the start height ------
    const Vec3 exit_raw{pl.scan.back().x, p_.aisle_y[a], std::clamp(start.z, p_.z_min, p_.z_max)};
    pl.exit = freeNear(exit_raw, obs, band_lo, band_hi).value_or(pl.scan.back());

    // ---- route every leg that is still not clear (A* in the aisle) -----------------------------
    const double rx0 = std::min({x_lo, pl.entry.x, pl.exit.x}) - p_.entry_search - 1.0;
    const double rx1 = std::max({x_hi, pl.entry.x, pl.exit.x}) + p_.entry_search + 1.0;
    auto route = [&](const Vec3& from, const Vec3& to, std::vector<Vec3>& out) {
      if (segmentClearance(from, to, obs) >= p_.clearance - kSegTol) return;
      ++pl.routed;
      if (auto via = connect(from, to, obs, rx0, rx1, band_lo, band_hi)) {
        out.insert(out.end(), via->begin(), via->end());
      } else {
        pl.notes.push_back(fmt("no clear route from [%.1f, %.1f, %.1f] to [%.1f, %.1f, %.1f] inside the aisle: "
                               "rbl_controller must find its way",
                               from.x, from.y, from.z, to.x, to.y, to.z));
      }
    };
    route(pl.entry, pl.scan.front(), pl.approach);
    {
      std::vector<Vec3> scan{pl.scan.front()};
      for (size_t m = 1; m < pl.scan.size(); ++m) {
        route(pl.scan[m - 1], pl.scan[m], scan);
        scan.push_back(pl.scan[m]);
      }
      pl.scan = std::move(scan);
    }
    route(pl.scan.back(), pl.exit, pl.leave);

    // ---- statistics -----------------------------------------------------------------------------
    for (const auto& g : pl.scan) {
      pl.max_lane_dev = std::max(pl.max_lane_dev, std::abs(g.y - pl.lane_y));
      double dz = 1e9;
      for (double z : p_.levels) dz = std::min(dz, std::abs(g.z - z));
      pl.max_level_dev = std::max(pl.max_level_dev, dz);
    }
    std::vector<Vec3> poly{pl.entry};
    poly.insert(poly.end(), pl.approach.begin(), pl.approach.end());
    poly.insert(poly.end(), pl.scan.begin(), pl.scan.end());
    poly.insert(poly.end(), pl.leave.begin(), pl.leave.end());
    poly.push_back(pl.exit);
    for (size_t m = 1; m < poly.size(); ++m) {
      const double c = segmentClearance(poly[m - 1], poly[m], obs);
      if (c < pl.min_clearance) {
        pl.min_clearance = c;
        pl.worst_from    = poly[m - 1];
        pl.worst_to      = poly[m];
      }
    }
    pl.ok = true;
    return pl;
  }

private:
  template <typename... A>
  static std::string fmt(const char* f, A... a)
  {
    char buf[512];
    std::snprintf(buf, sizeof(buf), f, a...);
    return buf;
  }

  std::vector<Box> nearby(double x0, double x1, double y0, double y1) const
  {
    std::vector<Box> out;
    for (const auto& b : obstacles_) {
      if (b.hi.x >= x0 && b.lo.x <= x1 && b.hi.y >= y0 && b.lo.y <= y1) out.push_back(b);
    }
    return out;
  }

  static double clearanceTo(const Vec3& q, const std::vector<Box>& obs)
  {
    double c = 1e9;
    for (const auto& b : obs) c = std::min(c, clearance(q, b));
    return c;
  }

  bool isFree(const Vec3& q, const std::vector<Box>& obs) const { return clearanceTo(q, obs) >= p_.clearance; }

  // min clearance along the straight segment a-b (sampled every 5 cm)
  static double segmentClearance(const Vec3& a, const Vec3& b, const std::vector<Box>& obs)
  {
    const int n = std::max(1, static_cast<int>(std::ceil(dist(a, b) / 0.05)));
    double    c = 1e9;
    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / n;
      c = std::min(c, clearanceTo(Vec3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t}, obs));
    }
    return c;
  }

  // smallest-deviation standoff in [s_min, s_max] at height z that is free
  std::optional<Vec3> lateral(double x, double z, double face_y, double away, double s_min, double s_max,
                              const std::vector<Box>& obs) const
  {
    constexpr double ds = 0.05;
    const int        m  = static_cast<int>(std::ceil(std::max(p_.standoff - s_min, s_max - p_.standoff) / ds));
    for (int i = 0; i <= m; ++i) {
      for (int sgn : {1, -1}) {
        const double s = p_.standoff + sgn * i * ds;
        if (s < s_min - 1e-9 || s > s_max + 1e-9 || (i == 0 && sgn < 0)) continue;
        const Vec3 q{x, face_y + away * s, z};
        if (isFree(q, obs)) return q;
      }
    }
    return std::nullopt;
  }

  std::optional<Vec3> placeSample(double x, double level, double face_y, double away, double s_min, double s_max,
                                  const std::vector<Box>& obs, SampleKind& kind) const
  {
    const Vec3 lane{x, face_y + away * p_.standoff, level};
    if (isFree(lane, obs)) {
      kind = SampleKind::LANE;
      return lane;
    }
    constexpr double dz = 0.1;
    // same height, then nearby heights (closest first), always the smallest sideways shift
    const int nz = static_cast<int>(std::round(p_.search_dz / dz));
    for (int i = 0; i <= nz; ++i) {
      for (int sgn : {1, -1}) {
        if (i == 0 && sgn < 0) continue;
        const double z = level + sgn * i * dz;
        if (z < p_.z_min - 1e-9 || z > p_.z_max + 1e-9) continue;
        if (auto q = lateral(x, z, face_y, away, s_min, s_max, obs)) {
          kind = SampleKind::SHIFTED;
          return q;
        }
      }
    }
    // hop over: lowest free height above the search window
    for (double z = level + p_.search_dz + dz; z <= p_.z_max + 1e-9; z += dz) {
      if (auto q = lateral(x, z, face_y, away, s_min, s_max, obs)) {
        kind = SampleKind::HOPPED;
        return q;
      }
    }
    kind = SampleKind::BLOCKED;
    return std::nullopt;
  }

  // single (standoff, height) free for every sample x in xs[i..e], cheapest first:
  // |standoff change| + 2 |height change|. nullopt if none (samples keep their own placement).
  std::optional<std::pair<double, double>> uniformDetour(const std::vector<double>& xs, size_t i, size_t e,
                                                         double level, double face_y, double away, double s_min,
                                                         double s_max, const std::vector<Box>& obs) const
  {
    struct Cand
    {
      double cost, s, z;
    };
    std::vector<Cand> cands;
    for (double z = p_.z_min; z <= p_.z_max + 1e-9; z += 0.1) {
      for (double so = s_min; so <= s_max + 1e-9; so += 0.05) {
        cands.push_back({std::abs(so - p_.standoff) + 2.0 * std::abs(z - level), so, z});
      }
    }
    std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) { return a.cost < b.cost; });
    for (const auto& c : cands) {
      bool ok = true;
      for (size_t t = i; t <= e && ok; ++t) ok = isFree(Vec3{xs[t], face_y + away * c.s, c.z}, obs);
      if (ok) return std::make_pair(c.s, c.z);
    }
    return std::nullopt;
  }

  // free spot near q for entry/exit goals: anywhere across the aisle, +- entry_search along it,
  // +- search_dz in height; cheapest = least displacement (height counts double)
  std::optional<Vec3> freeNear(const Vec3& q, const std::vector<Box>& obs, double band_lo, double band_hi) const
  {
    if (isFree(q, obs)) return q;
    constexpr double step = 0.1;
    std::optional<Vec3> best;
    double              best_cost = 1e9;
    const int nx = static_cast<int>(std::round(p_.entry_search / step));
    const int nz = static_cast<int>(std::round(p_.search_dz / step));
    const int ny = static_cast<int>(std::round((band_hi - band_lo) / step));
    for (int iz = -nz; iz <= nz; ++iz) {
      const double z = q.z + iz * step;
      if (z < p_.z_min - 1e-9 || z > p_.z_max + 1e-9) continue;
      for (int ix = -nx; ix <= nx; ++ix) {
        const double x = q.x + ix * step;
        for (int iy = 0; iy <= ny; ++iy) {
          const double y    = band_lo + iy * step;
          const double cost = std::abs(x - q.x) + std::abs(y - q.y) + 2.0 * std::abs(z - q.z);
          if (cost >= best_cost) continue;
          const Vec3 c{x, y, z};
          if (isFree(c, obs)) {
            best      = c;
            best_cost = cost;
          }
        }
      }
    }
    return best;
  }

  // indices of the samples kept as goals: both ends, both ends of every deviation (y or z change),
  // and one every waypoint_spacing on straight stretches
  std::vector<size_t> reduce(const std::vector<Vec3>& pass) const
  {
    std::vector<size_t> idx{0};
    for (size_t i = 1; i < pass.size(); ++i) {
      const bool bend = std::abs(pass[i].y - pass[i - 1].y) > 1e-3 || std::abs(pass[i].z - pass[i - 1].z) > 1e-3;
      if (bend) {
        if (idx.back() != i - 1) idx.push_back(i - 1);
        idx.push_back(i);
      } else if (std::abs(pass[i].x - pass[idx.back()].x) >= p_.waypoint_spacing - 1e-6) {
        idx.push_back(i);
      }
    }
    if (idx.back() != pass.size() - 1) idx.push_back(pass.size() - 1);
    return idx;
  }

  // A* on a grid of `sample_step` cells inside [x0,x1] x [y0,y1] x [z_min,z_max], 26-connected,
  // cells free when >= clearance from every obstacle; returns the via points strictly between a and
  // b after line-of-sight smoothing, or nullopt if b is unreachable.
  std::optional<std::vector<Vec3>> connect(const Vec3& a, const Vec3& b, const std::vector<Box>& obs, double x0,
                                           double x1, double y0, double y1) const
  {
    const double h  = p_.sample_step;
    const int    nx = static_cast<int>(std::ceil((x1 - x0) / h)) + 1;
    const int    ny = static_cast<int>(std::ceil((y1 - y0) / h)) + 1;
    const int    nz = static_cast<int>(std::ceil((p_.z_max - p_.z_min) / h)) + 1;
    auto idx  = [&](int i, int j, int k) { return (static_cast<long>(k) * ny + j) * nx + i; };
    auto pos  = [&](int i, int j, int k) { return Vec3{x0 + i * h, y0 + j * h, p_.z_min + k * h}; };
    std::vector<signed char> free_cache(static_cast<size_t>(nx) * ny * nz, -1);
    auto isFreeCell = [&](int i, int j, int k) {
      auto& c = free_cache[idx(i, j, k)];
      if (c < 0) c = isFree(pos(i, j, k), obs) ? 1 : 0;
      return c == 1;
    };
    // nearest free cell to a point (search a small neighbourhood)
    auto snap = [&](const Vec3& q) -> std::optional<std::array<int, 3>> {
      const int ci = static_cast<int>(std::round((q.x - x0) / h)), cj = static_cast<int>(std::round((q.y - y0) / h)),
                ck = static_cast<int>(std::round((q.z - p_.z_min) / h));
      std::optional<std::array<int, 3>> best;
      double                            bd = 1e9;
      for (int dk = -2; dk <= 2; ++dk)
        for (int dj = -2; dj <= 2; ++dj)
          for (int di = -2; di <= 2; ++di) {
            const int i = ci + di, j = cj + dj, k = ck + dk;
            if (i < 0 || j < 0 || k < 0 || i >= nx || j >= ny || k >= nz || !isFreeCell(i, j, k)) continue;
            const double d = dist(pos(i, j, k), q);
            if (d < bd && segmentClearance(q, pos(i, j, k), obs) >= p_.clearance - kSegTol) {
              bd   = d;
              best = std::array<int, 3>{i, j, k};
            }
          }
      return best;
    };
    const auto sa = snap(a), sb = snap(b);
    if (!sa || !sb) return std::nullopt;

    using Node = std::pair<double, long>;
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::unordered_map<long, double> g;
    std::unordered_map<long, long>   parent;
    const long start = idx((*sa)[0], (*sa)[1], (*sa)[2]), goal = idx((*sb)[0], (*sb)[1], (*sb)[2]);
    const Vec3 goal_p = pos((*sb)[0], (*sb)[1], (*sb)[2]);
    g[start] = 0.0;
    open.push({0.0, start});
    bool found = false;
    while (!open.empty()) {
      const auto [f, cur] = open.top();
      open.pop();
      if (cur == goal) {
        found = true;
        break;
      }
      const int  ci = static_cast<int>(cur % nx), cj = static_cast<int>((cur / nx) % ny),
                ck = static_cast<int>(cur / (static_cast<long>(nx) * ny));
      const Vec3 cp = pos(ci, cj, ck);
      if (f - dist(cp, goal_p) > g[cur] + 1e-9) continue;  // stale entry
      for (int dk = -1; dk <= 1; ++dk)
        for (int dj = -1; dj <= 1; ++dj)
          for (int di = -1; di <= 1; ++di) {
            if (!di && !dj && !dk) continue;
            const int i = ci + di, j = cj + dj, k = ck + dk;
            if (i < 0 || j < 0 || k < 0 || i >= nx || j >= ny || k >= nz || !isFreeCell(i, j, k)) continue;
            const long   nb = idx(i, j, k);
            const double ng = g[cur] + h * std::sqrt(static_cast<double>(di * di + dj * dj + dk * dk));
            auto         it = g.find(nb);
            if (it != g.end() && it->second <= ng) continue;
            g[nb]      = ng;
            parent[nb] = cur;
            open.push({ng + dist(pos(i, j, k), goal_p), nb});
          }
    }
    if (!found) return std::nullopt;

    std::vector<Vec3> cells;
    for (long c = goal; c != start; c = parent[c]) {
      cells.push_back(pos(static_cast<int>(c % nx), static_cast<int>((c / nx) % ny),
                          static_cast<int>(c / (static_cast<long>(nx) * ny))));
    }
    cells.push_back(pos((*sa)[0], (*sa)[1], (*sa)[2]));
    std::reverse(cells.begin(), cells.end());

    // line-of-sight smoothing over a -> cells -> b
    std::vector<Vec3> full{a};
    full.insert(full.end(), cells.begin(), cells.end());
    full.push_back(b);
    std::vector<Vec3> via;
    size_t            i = 0;
    while (i + 1 < full.size()) {
      size_t j = full.size() - 1;
      while (j > i + 1 && segmentClearance(full[i], full[j], obs) < p_.clearance - kSegTol) --j;
      if (j != full.size() - 1) via.push_back(full[j]);
      i = j;
    }
    return via;
  }

  // a straight leg counts as clear down to clearance - kSegTol (sampling between 0.25 m samples)
  static constexpr double kSegTol = 0.1;

  Params           p_;
  std::vector<Box> obstacles_;
};

}  // namespace rbl_mission
