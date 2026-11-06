// Simplified rviz picture of the UGV + UAV mission (visualization only, nothing reads it):
//
//   ~/warehouse  visualization_msgs/MarkerArray (latched)  every collision box of the gz world
//                (map.world_sdf): racks / cartons / pallets in grey, walls faint, temporary obstacles
//                (models temp_obstacle*, not in the planners' prior map) in red
//   ~/grid       visualization_msgs/MarkerArray (latched)  the row / column numbering of deploy.sh, from
//                map.aisle_y + map.segment_x_min / segment_x_max (pass config/warehouse_map.yaml):
//                "row N" at both ends of every aisle, "col M" above every segment, a green disc +
//                "N,M" at every UGV stop (segment middle on the aisle centreline), aisle centrelines,
//                and "left (+y)" / "right (-y)" next to one stop
//   ~/vehicles   visualization_msgs/MarkerArray (10 Hz)
//                UGV: orange body (r1_rover size) drawn at its real floor height -- its odometry z
//                     is the 0.5 m planning plane, see the session's env.sh -- black camera, label
//                UAV: blue x500-like body, two arms, four rotor discs, label
//                text: the deployment phase (/ugv_uav_mission/state) above the UGV and the UAV's
//                     own task (/<uav>/warehouse_inspection/state) above the UAV

#include <algorithm>
#include <cmath>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rbl_mission/coverage_planner.hpp"

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class MissionViz : public rclcpp::Node
{
public:
  MissionViz() : Node("mission_viz")
  {
    frame_      = declare_parameter<std::string>("frame", "world");
    ugv_        = declare_parameter<std::string>("ugv", "ugv1");
    uav_        = declare_parameter<std::string>("uav", "uav1");
    ugv_body_z_ = declare_parameter<double>("ugv_body_z", 0.2);  // [m] body centre above the floor
    const auto world = declare_parameter<std::string>("world_sdf", "");

    auto latched = rclcpp::QoS(1).transient_local();
    pub_world_   = create_publisher<MarkerArray>("~/warehouse", latched);
    pub_veh_     = create_publisher<MarkerArray>("~/vehicles", rclcpp::QoS(1));
    pub_grid_    = create_publisher<MarkerArray>("~/grid", latched);
    publishGrid(declare_parameter<std::vector<double>>("map.aisle_y", std::vector<double>{}),
                declare_parameter<std::vector<double>>("map.segment_x_min", std::vector<double>{}),
                declare_parameter<std::vector<double>>("map.segment_x_max", std::vector<double>{}),
                declare_parameter<double>("grid_label_z", 5.5));  // [m] row / col labels above the racks

    if (!world.empty()) {
      int  unsupported = 0;
      std::vector<rbl_mission::Box> temporary;
      auto boxes = rbl_mission::loadWorldObstacles(world, unsupported, &temporary);
      MarkerArray arr;
      int         id = 0;
      const auto  n_static = boxes.size();
      boxes.insert(boxes.end(), temporary.begin(), temporary.end());
      for (size_t i = 0; i < boxes.size(); ++i) {
        const auto& b = boxes[i];
        Marker m = base("warehouse", id++, Marker::CUBE);
        m.pose.position.x = (b.lo.x + b.hi.x) / 2;
        m.pose.position.y = (b.lo.y + b.hi.y) / 2;
        m.pose.position.z = (b.lo.z + b.hi.z) / 2;
        m.scale.x = b.hi.x - b.lo.x;
        m.scale.y = b.hi.y - b.lo.y;
        m.scale.z = b.hi.z - b.lo.z;
        const bool wall = std::max(m.scale.x, m.scale.y) > 30.0;
        if (i >= n_static) {
          color(m, 0.95, 0.12, 0.08, 0.9);  // temporary: not in the planners' prior map
        } else {
          color(m, 0.55, 0.55, 0.58, wall ? 0.15 : 0.55);
        }
        arr.markers.push_back(m);
      }
      pub_world_->publish(arr);
      RCLCPP_INFO(get_logger(), "warehouse: %zu static + %zu temporary boxes from %s", n_static,
                  temporary.size(), world.c_str());
    }

    auto odom = [this](std::optional<geometry_msgs::msg::Pose>& dst) {
      return [this, &dst](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        std::scoped_lock l(mtx_);
        dst = m->pose.pose;
      };
    };
    sub_ugv_ = create_subscription<nav_msgs::msg::Odometry>("/" + ugv_ + "/odometry", 5, odom(ugv_pose_));
    sub_uav_ = create_subscription<nav_msgs::msg::Odometry>("/" + uav_ + "/odometry", 5, odom(uav_pose_));
    auto text = [this](std::string& dst) {
      return [this, &dst](std_msgs::msg::String::ConstSharedPtr m) {
        std::scoped_lock l(mtx_);
        dst = m->data;
      };
    };
    sub_phase_ = create_subscription<std_msgs::msg::String>("/ugv_uav_mission/state", latched, text(phase_));
    sub_task_  = create_subscription<std_msgs::msg::String>("/" + uav_ + "/warehouse_inspection/state", latched,
                                                           text(task_));
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&MissionViz::publishVehicles, this));
  }

private:
  // row r = aisle_y[r-1] (1 = most -y), column c = [segment_x_min, segment_x_max][c-1] (1 = most -x),
  // exactly as deploy.sh counts them
  void publishGrid(const std::vector<double>& ys, const std::vector<double>& x0, const std::vector<double>& x1,
                   double label_z)
  {
    if (ys.empty() || x0.empty() || x0.size() != x1.size()) {
      RCLCPP_WARN(get_logger(), "no row/column grid: pass config/warehouse_map.yaml (map.aisle_y, map.segment_x_*)");
      return;
    }
    MarkerArray arr;
    int         id   = 0;
    const double xlo = *std::min_element(x0.begin(), x0.end()), xhi = *std::max_element(x1.begin(), x1.end());
    const double ytop = *std::max_element(ys.begin(), ys.end());
    auto txt = [&](double x, double y, double z, const std::string& s, double size, double r, double g, double b) {
      Marker m = label("grid", id++, x, y, z, s, size);
      color(m, r, g, b);
      arr.markers.push_back(m);
    };
    Marker lines = base("grid", id++, Marker::LINE_LIST);
    lines.scale.x = 0.06;
    color(lines, 0.2, 0.9, 0.3, 0.6);
    for (size_t r = 0; r < ys.size(); ++r) {
      const std::string name = "row " + std::to_string(r + 1);
      txt(xlo - 2.0, ys[r], label_z, name, 1.0, 1.0, 0.85, 0.1);
      txt(xhi + 2.0, ys[r], label_z, name, 1.0, 1.0, 0.85, 0.1);
      for (size_t c = 0; c < x0.size(); ++c) {
        geometry_msgs::msg::Point a, b;
        a.x = x0[c], b.x = x1[c];
        a.y = b.y = ys[r];
        a.z = b.z = 0.05;
        lines.points.push_back(a);
        lines.points.push_back(b);
        const double xm = (x0[c] + x1[c]) / 2;
        Marker disc = base("grid", id++, Marker::CYLINDER);
        disc.pose.position.x = xm;
        disc.pose.position.y = ys[r];
        disc.pose.position.z = 0.05;
        disc.scale.x = disc.scale.y = 0.8;
        disc.scale.z = 0.05;
        color(disc, 0.2, 0.9, 0.3, 0.9);
        arr.markers.push_back(disc);
        txt(xm, ys[r], 1.0, std::to_string(r + 1) + "," + std::to_string(c + 1), 0.6, 0.6, 1.0, 0.6);
      }
    }
    arr.markers.push_back(lines);
    for (size_t c = 0; c < x0.size(); ++c)
      txt((x0[c] + x1[c]) / 2, ytop + 3.5, label_z, "col " + std::to_string(c + 1), 1.0, 0.3, 0.8, 1.0);
    // which rack is "left" / "right" (same for every stop): next to the first stop
    const double xm = (x0[0] + x1[0]) / 2;
    txt(xm, ys[0] + 1.2, 1.6, "left (+y)", 0.4, 1, 1, 1);
    txt(xm, ys[0] - 1.2, 1.6, "right (-y)", 0.4, 1, 1, 1);
    pub_grid_->publish(arr);
    RCLCPP_INFO(get_logger(), "grid: %zu rows x %zu columns", ys.size(), x0.size());
  }

  Marker base(const std::string& ns, int id, int type) const
  {
    Marker m;
    m.header.frame_id    = frame_;
    m.ns                 = ns;
    m.id                 = id;
    m.type               = type;
    m.action             = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 1.0;
    return m;
  }
  static void color(Marker& m, double r, double g, double b, double a = 1.0)
  {
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
  }
  static double yawOf(const geometry_msgs::msg::Quaternion& q)
  {
    return std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  }
  // a part of a vehicle: offset (dx, dy, dz) in its body frame, rotated by yaw
  Marker part(const std::string& ns, int id, int type, double x, double y, double z, double yaw, double dx, double dy,
              double dz, double sx, double sy, double sz, double part_yaw = 0.0) const
  {
    Marker m          = base(ns, id, type);
    m.pose.position.x = x + std::cos(yaw) * dx - std::sin(yaw) * dy;
    m.pose.position.y = y + std::sin(yaw) * dx + std::cos(yaw) * dy;
    m.pose.position.z = z + dz;
    m.pose.orientation.z = std::sin((yaw + part_yaw) / 2);
    m.pose.orientation.w = std::cos((yaw + part_yaw) / 2);
    m.scale.x = sx;
    m.scale.y = sy;
    m.scale.z = sz;
    return m;
  }
  Marker label(const std::string& ns, int id, double x, double y, double z, const std::string& s, double size) const
  {
    Marker m          = base(ns, id, Marker::TEXT_VIEW_FACING);
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = z;
    m.scale.z         = size;
    m.text            = s;
    color(m, 1, 1, 1);
    return m;
  }
  static std::string head(const std::string& s)  // "PHASE: details" -> "PHASE"
  {
    return s.substr(0, s.find(':'));
  }

  // what the UGV is doing in each deployment phase (/ugv_uav_mission/state)
  std::string ugvActivity() const
  {
    const std::string p = head(phase_);
    if (p == "DRIVE") return "carrying the UAV";
    if (p == "TAKEOFF") return "stopped, UAV taking off";
    if (p == "INSPECT") return "stopped, waiting for the UAV";
    if (p == "PICKUP") return "driving to the UAV";
    if (p == "RENDEZVOUS") return "stopped, UAV coming above it";
    if (p == "LAND") return "UAV landing on it";
    if (p == "ATTACH" || p == "HOME") return "carrying the UAV home";
    if (p == "DONE") return "home, UAV on board";
    return p.empty() ? "idle" : p;
  }

  void publishVehicles()
  {
    std::scoped_lock l(mtx_);
    MarkerArray arr;
    if (ugv_pose_) {
      const double x = ugv_pose_->position.x, y = ugv_pose_->position.y, yaw = yawOf(ugv_pose_->orientation);
      const double z = ugv_body_z_;
      Marker body    = part("ugv", 0, Marker::CUBE, x, y, z, yaw, 0, 0, 0, 0.4, 0.4, 0.25);
      color(body, 1.0, 0.55, 0.05);
      Marker cam = part("ugv", 1, Marker::CUBE, x, y, z, yaw, 0.21, 0, 0.08, 0.04, 0.10, 0.04);
      color(cam, 0.05, 0.05, 0.05);
      arr.markers.push_back(body);
      arr.markers.push_back(cam);
      for (int i = 0; i < 4; ++i) {  // wheels
        Marker w = part("ugv", 2 + i, Marker::CYLINDER, x, y, z, yaw, i < 2 ? 0.15 : -0.15, i % 2 ? 0.2 : -0.2,
                        -0.13, 0.14, 0.14, 0.05);
        // axle along the body y axis: q = yaw (about z) * 90 deg roll (about x)
        const double c = std::cos(yaw / 2), s = std::sin(yaw / 2), r = std::sqrt(0.5);
        w.pose.orientation.w = c * r;
        w.pose.orientation.x = c * r;
        w.pose.orientation.y = s * r;
        w.pose.orientation.z = s * r;
        color(w, 0.1, 0.1, 0.1);
        arr.markers.push_back(w);
      }
      arr.markers.push_back(label("ugv", 10, x, y, z + 0.45, "UGV  " + ugvActivity(), 0.3));
    }
    if (uav_pose_) {
      const double x = uav_pose_->position.x, y = uav_pose_->position.y, z = uav_pose_->position.z;
      const double yaw = yawOf(uav_pose_->orientation);
      Marker body      = part("uav", 0, Marker::CUBE, x, y, z, yaw, 0, 0, 0, 0.22, 0.22, 0.07);
      color(body, 0.1, 0.35, 0.95);
      arr.markers.push_back(body);
      for (int a = 0; a < 2; ++a) {
        Marker arm = part("uav", 1 + a, Marker::CUBE, x, y, z, yaw, 0, 0, 0.02, 0.62, 0.03, 0.03, M_PI / 4 + a * M_PI / 2);
        color(arm, 0.15, 0.15, 0.2);
        arr.markers.push_back(arm);
      }
      for (int r = 0; r < 4; ++r) {
        const double ang = M_PI / 4 + r * M_PI / 2;
        Marker rot = part("uav", 3 + r, Marker::CYLINDER, x, y, z, yaw, 0.22 * std::cos(ang), 0.22 * std::sin(ang), 0.05,
                          0.25, 0.25, 0.01);
        color(rot, 0.6, 0.8, 1.0, 0.6);
        arr.markers.push_back(rot);
      }
      // skids
      for (int s = 0; s < 2; ++s) {
        Marker sk = part("uav", 7 + s, Marker::CUBE, x, y, z, yaw, 0, s ? 0.13 : -0.13, -0.2, 0.25, 0.02, 0.02);
        color(sk, 0.15, 0.15, 0.2);
        arr.markers.push_back(sk);
      }
      arr.markers.push_back(label("uav", 10, x, y, z + 0.75, "UAV  " + (task_.empty() ? std::string("on the UGV") : head(task_)), 0.3));
    }
    pub_veh_->publish(arr);
  }

  std::string frame_, ugv_, uav_;
  double      ugv_body_z_;

  std::mutex                              mtx_;
  std::optional<geometry_msgs::msg::Pose> ugv_pose_, uav_pose_;
  std::string                             phase_, task_;

  rclcpp::Publisher<MarkerArray>::SharedPtr                pub_world_, pub_veh_, pub_grid_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ugv_, sub_uav_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr   sub_phase_, sub_task_;
  rclcpp::TimerBase::SharedPtr                             timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionViz>());
  rclcpp::shutdown();
  return 0;
}
