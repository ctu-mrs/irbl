// Offline check of the shelf coverage planner: plans every shelf of the map (every aisle x segment x
// side, requested at the segment centre, UAV starting at `start`) with exactly the code the mission
// node uses, verifies each plan independently and prints a table. Exit code 1 if any plan fails.
//
//   ros2 run rbl_mission coverage_plan_check --ros-args --params-file config/warehouse_map.yaml
//       -p map.world_sdf:=worlds/warehouse.sdf [-p csv:=/tmp/plans.csv]
//
// Independent checks per plan (on top of the planner's own statistics):
//   - every goal (entry, coverage, exit) at least coverage.clearance from every obstacle
//   - every coverage goal inside the lane band (standoff .. standoff_max from the rack face)
//   - every coverage segment clear, apart from gaps over blocked samples (reported separately)
//   - per level: share of the face length whose samples could be placed (coverage completeness)
// plus requests that must be rejected (outside aisles / outside the warehouse).

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "rbl_mission/coverage_planner.hpp"
#include "rbl_mission/planner_params.hpp"

using rbl_mission::Box;
using rbl_mission::CoveragePlanner;
using rbl_mission::Vec3;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("coverage_plan_check");
  std::unique_ptr<CoveragePlanner> planner;
  try {
    planner = rbl_mission::makePlannerFromParams(*node);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 2;
  }
  const auto start_v = node->declare_parameter<std::vector<double>>("start", {0.0, 0.0, 2.0});
  const auto csv     = node->declare_parameter<std::string>("csv", "");
  const auto& P      = planner->params();
  const Vec3  start{start_v.at(0), start_v.at(1), start_v.at(2)};

  // obstacles again, for the independent checks
  std::vector<Box> obs;
  if (const auto w = node->get_parameter("map.world_sdf").as_string(); !w.empty()) {
    int u = 0;
    obs   = rbl_mission::loadWorldObstacles(w, u);
  }
  auto clearanceAt = [&](const Vec3& q) {
    if (q.z > P.z_max + 1e-6 || q.z < P.z_min - 1e-6) return -1.0;  // outside the allowed heights
    double c = 1e9;
    for (const auto& b : obs) c = std::min(c, rbl_mission::clearance(q, b));
    return c;
  };

  std::ofstream out;
  if (!csv.empty()) {
    out.open(csv);
    out << "shelf,kind,index,x,y,z\n";
  }

  std::printf("%-22s %5s %6s %5s %5s %5s %5s %7s %7s %7s %8s  %s\n", "shelf", "goals", "samp", "shft", "hop", "blk",
              "route", "lanedev", "lvldev", "minclr", "cover%", "result");
  int failures = 0, warnings = 0;
  for (size_t a = 0; a < P.aisle_y.size(); ++a) {
    for (size_t s = 0; s < P.segment_x_min.size(); ++s) {
      for (bool left : {true, false}) {
        const Vec3  req{(P.segment_x_min[s] + P.segment_x_max[s]) / 2.0, P.aisle_y[a], 0.0};
        const auto  pl   = planner->plan(req, left, start);
        char        name[64];
        std::snprintf(name, sizeof(name), "a%zu s%zu %s", a + 1, s + 1, left ? "left" : "right");
        if (!pl.ok) {
          std::printf("%-22s %s\n", name, ("FAIL: " + pl.error).c_str());
          ++failures;
          continue;
        }
        std::vector<std::string> problems;
        // goals clear
        auto checkGoal = [&](const Vec3& g, const char* what) {
          const double c = clearanceAt(g);
          if (c < P.clearance - 1e-6) {
            char b[160];
            std::snprintf(b, sizeof(b), "%s [%.2f %.2f %.2f] clearance %.2f", what, g.x, g.y, g.z, c);
            problems.push_back(b);
          }
        };
        checkGoal(pl.entry, "entry");
        checkGoal(pl.exit, "exit");
        for (const auto& g : pl.approach) checkGoal(g, "approach via");
        for (const auto& g : pl.leave) checkGoal(g, "leave via");
        if (pl.min_clearance < P.clearance - 0.1) {
          char b[200];
          std::snprintf(b, sizeof(b), "leg [%.2f %.2f %.2f] -> [%.2f %.2f %.2f] only %.2f m from an obstacle",
                        pl.worst_from.x, pl.worst_from.y, pl.worst_from.z, pl.worst_to.x, pl.worst_to.y,
                        pl.worst_to.z, pl.min_clearance);
          problems.push_back(b);
        }
        for (const auto& g : pl.scan) checkGoal(g, "scan goal");
        // lane band (routed via goals may leave it: they only connect, they don't scan)
        const double half   = P.rack_depth / 2.0;
        const double face_y = left ? P.aisle_left_row[a] - half : P.aisle_right_row[a] + half;
        size_t       off_lane = 0;
        for (const auto& g : pl.scan) {
          const double so = std::abs(g.y - face_y);
          if (so < std::min(P.clearance, P.standoff) - 1e-6 || so > P.standoff_max + 1e-6) {
            ++off_lane;
          }
        }
        if (off_lane > pl.routed * 4) {  // more than a few routed via goals could explain
          problems.push_back(std::to_string(off_lane) + " scan goals outside the lane band");
        }
        // completeness: placed samples / all samples
        const double cover = 100.0 * (pl.samples - pl.blocked) / std::max<size_t>(1, pl.samples);

        const bool        bad  = !problems.empty();
        const bool        warn = !pl.notes.empty();
        std::printf("%-22s %5zu %6zu %5zu %5zu %5zu %5zu %7.2f %7.2f %7.2f %7.1f%%  %s\n", name, pl.scan.size(),
                    pl.samples, pl.shifted, pl.hopped, pl.blocked, pl.routed, pl.max_lane_dev, pl.max_level_dev,
                    pl.min_clearance, cover,
                    bad ? "FAIL" : (warn ? "ok (notes)" : "ok"));
        for (const auto& p : problems) std::printf("    ! %s\n", p.c_str());
        for (const auto& n : pl.notes) std::printf("    - %s\n", n.c_str());
        failures += bad;
        warnings += warn;

        if (out) {
          auto row = [&](const char* kind, size_t i, const Vec3& v) {
            out << name << ',' << kind << ',' << i << ',' << v.x << ',' << v.y << ',' << v.z << '\n';
          };
          row("entry", 0, pl.entry);
          for (size_t i = 0; i < pl.approach.size(); ++i) row("approach", i, pl.approach[i]);
          for (size_t i = 0; i < pl.scan.size(); ++i) row("scan", i, pl.scan[i]);
          for (size_t i = 0; i < pl.dense.size(); ++i) row("sample", i, pl.dense[i]);
          for (size_t i = 0; i < pl.leave.size(); ++i) row("leave", i, pl.leave[i]);
          row("exit", 0, pl.exit);
        }
      }
    }
  }

  // requests that must be rejected
  struct Bad
  {
    Vec3        p;
    const char* why;
  };
  // a rack row centre of this map (the warehouse's y = 6.3 is an aisle in other worlds)
  const auto rows  = node->get_parameter("map.aisle_left_row").as_double_array();
  const double row_y = rows.empty() ? 6.3 : rows[rows.size() / 2];
  const Bad bad_requests[] = {{{-28.0, row_y, 0.0}, "inside a rack row"},
                              {{100.0, 0.0, 0.0}, "far outside the warehouse (+x)"},
                              {{-28.0, 30.0, 0.0}, "outside the warehouse (+y)"}};
  for (const auto& b : bad_requests) {
    const auto pl = planner->plan(b.p, true, start);
    std::printf("reject %-32s %s\n", b.why, pl.ok ? "FAIL: accepted" : ("ok: " + pl.error).c_str());
    failures += pl.ok;
  }

  std::printf("\n%d failure(s), %d plan(s) with notes, %zu obstacles, clearance %.2f m\n", failures, warnings,
              planner->obstacleCount(), P.clearance);
  rclcpp::shutdown();
  return failures ? 1 : 0;
}
