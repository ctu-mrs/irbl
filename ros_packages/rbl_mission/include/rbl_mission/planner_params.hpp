// Declares the map.* / coverage.* parameters (see warehouse_map.yaml) on a node and builds the
// CoveragePlanner from them. Shared by warehouse_inspection_node and coverage_plan_check.
#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "rbl_mission/coverage_planner.hpp"

namespace rbl_mission
{

inline std::unique_ptr<CoveragePlanner> makePlannerFromParams(rclcpp::Node& n)
{
  CoveragePlanner::Params p;
  using V = std::vector<double>;
  p.rack_depth        = n.declare_parameter<double>("map.rack_depth", p.rack_depth);
  p.rack_height       = n.declare_parameter<double>("map.rack_height", p.rack_height);
  p.aisle_half_width  = n.declare_parameter<double>("map.aisle_half_width", p.aisle_half_width);
  p.cross_aisle_reach = n.declare_parameter<double>("map.cross_aisle_reach", p.cross_aisle_reach);
  p.aisle_y           = n.declare_parameter<V>("map.aisle_y", V{});
  p.aisle_left_row    = n.declare_parameter<V>("map.aisle_left_row", V{});
  p.aisle_right_row   = n.declare_parameter<V>("map.aisle_right_row", V{});
  p.segment_x_min     = n.declare_parameter<V>("map.segment_x_min", V{});
  p.segment_x_max     = n.declare_parameter<V>("map.segment_x_max", V{});
  p.cross_aisle_x     = n.declare_parameter<V>("map.cross_aisle_x", V{});
  const auto world    = n.declare_parameter<std::string>("map.world_sdf", "");

  p.standoff         = n.declare_parameter<double>("coverage.standoff", p.standoff);
  p.standoff_max     = n.declare_parameter<double>("coverage.standoff_max", p.standoff_max);
  p.levels           = n.declare_parameter<V>("coverage.levels", p.levels);
  p.end_margin       = n.declare_parameter<double>("coverage.end_margin", p.end_margin);
  p.waypoint_spacing = n.declare_parameter<double>("coverage.waypoint_spacing", p.waypoint_spacing);
  p.sample_step      = n.declare_parameter<double>("coverage.sample_step", p.sample_step);
  p.clearance        = n.declare_parameter<double>("coverage.clearance", p.clearance);
  p.search_dz        = n.declare_parameter<double>("coverage.search_dz", p.search_dz);
  p.entry_search     = n.declare_parameter<double>("coverage.entry_search", p.entry_search);
  p.z_min            = n.declare_parameter<double>("coverage.z_min", p.z_min);
  p.z_max            = n.declare_parameter<double>("coverage.z_max", p.z_max);

  std::vector<Box> obstacles;
  if (world.empty()) {
    RCLCPP_WARN(n.get_logger(), "map.world_sdf not set: goals are NOT checked against obstacles");
  } else {
    int unsupported = 0;
    std::vector<Box> temporary;
    obstacles = loadWorldObstacles(world, unsupported, &temporary);
    RCLCPP_INFO(n.get_logger(), "loaded %zu obstacle shapes from %s (%zu temporary ones left out of the map)",
                obstacles.size(), world.c_str(), temporary.size());
    if (unsupported > 0) {
      RCLCPP_WARN(n.get_logger(), "%d collision shapes / includes could not be represented and are ignored",
                  unsupported);
    }
  }
  return std::make_unique<CoveragePlanner>(std::move(p), std::move(obstacles));
}

}  // namespace rbl_mission
