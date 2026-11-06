// Warehouse shelf-inspection mission on top of the unchanged rbl_controller_node.
//
// rbl_controller only exposes "fly to this 3D goal" (~/goto) plus activation, so every task here is
// a sequence of goals. One request to ~/inspect runs:
//
//   GOTO      fly to the requested aisle position
//   COVERAGE  boustrophedon scan of the rack face on the requested side of that aisle segment:
//             one pass along the shelf per coverage level, climbing at the pass ends
//   RETURN    leave the aisle where the coverage ended, then fly back to where the UAV was when
//             the request came (mission.return_to_start:=false: stop and hover where it left the
//             aisle instead, e.g. for a UGV to come and pick it up there)
//
// The goals come from rbl_mission::CoveragePlanner (include/rbl_mission/coverage_planner.hpp):
// prior map of the aisles (parameters, see
// tmux/px4_sim_warehouse_inspection/config/warehouse_map.yaml) + every collision shape of the
// Gazebo world (map.world_sdf) as obstacles, every goal and every coverage segment kept
// coverage.clearance away from them. `ros2 run rbl_mission coverage_plan_check` plans every shelf
// offline with the same code. The node sends one goal at a time and waits on odometry until the UAV
// is within tolerance (or a timeout) before sending the next. Obstacle avoidance / replanning stay
// entirely inside rbl_controller.
//
// Interface (relative names, run it in the UAV namespace):
//   ~/inspect  rbl_msgs/srv/Vec4  goal = [x, y, z, side]  aisle position (control frame) + side of
//                                 the shelf to scan: >= 0 left (+y rack row), < 0 right (-y row).
//                                 z <= 0 -> first coverage level.
//   ~/abort    std_srvs/Trigger   park at the current position, back to IDLE
//   ~/state    std_msgs/String    (latched) IDLE / GOTO / COVERAGE / RETURN / DONE / ABORTED + info
//   ~/coverage_path nav_msgs/Path (latched) planned goals, for rviz / foxglove
//   ~/goal     geometry_msgs/PoseStamped (latched) goal currently sent to rbl_controller
//   ~/waypoints visualization_msgs/MarkerArray (latched) the planned goals as numbered spheres:
//              yellow aisle position / approach, cyan coverage, magenta leave / return; the
//              current goal green and larger, reached ones grey
//   uses: odometry (nav_msgs/Odometry), rbl_controller/goto, rbl_controller/activation

#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rbl_msgs/srv/goto_path.hpp>
#include <rbl_msgs/srv/vec4.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rbl_mission/coverage_planner.hpp"
#include "rbl_mission/planner_params.hpp"

using rbl_mission::Vec3;

namespace
{

enum class State
{
  IDLE,
  GOTO,
  COVERAGE,
  RETURN,
  DONE
};

const char* toString(State s)
{
  switch (s) {
    case State::IDLE: return "IDLE";
    case State::GOTO: return "GOTO";
    case State::COVERAGE: return "COVERAGE";
    case State::RETURN: return "RETURN";
    case State::DONE: return "DONE";
  }
  return "?";
}

struct Goal
{
  Vec3        p;
  State       task;
  double      tol;
  std::string label;
};

}  // namespace

class WarehouseInspection : public rclcpp::Node
{
public:
  WarehouseInspection() : Node("warehouse_inspection")
  {
    planner_ = rbl_mission::makePlannerFromParams(*this);

    tol_           = declare_parameter<double>("mission.tolerance", 0.6);
    tol_pass_      = declare_parameter<double>("mission.tolerance_pass", 1.0);
    timeout_base_  = declare_parameter<double>("mission.timeout_base", 30.0);
    timeout_per_m_ = declare_parameter<double>("mission.timeout_per_meter", 4.0);
    min_alt_       = declare_parameter<double>("mission.min_altitude", 1.5);
    return_start_  = declare_parameter<bool>("mission.return_to_start", true);
    // "global": every goal is sent as a straight path from where the UAV is (rbl_controller/goto_path):
    // the legs are already planned against the known map (CoveragePlanner), rbl_controller follows
    // them without its local replanner. "local": rbl_controller/goto, its local replanner routes.
    const auto mode = declare_parameter<std::string>("planner.mode", "global");
    if (mode != "global" && mode != "local") {
      throw std::invalid_argument("planner.mode must be \"global\" or \"local\", got \"" + mode + "\"");
    }
    global_ = mode == "global";
    RCLCPP_INFO(get_logger(), "planner: %s", global_ ? "global (map-planned legs, rbl_controller/goto_path)"
                                                     : "local (rbl_controller/goto, its local replanner)");

    auto latched = rclcpp::QoS(1).transient_local();
    pub_state_   = create_publisher<std_msgs::msg::String>("~/state", latched);
    pub_path_    = create_publisher<nav_msgs::msg::Path>("~/coverage_path", latched);
    pub_goal_    = create_publisher<geometry_msgs::msg::PoseStamped>("~/goal", latched);
    pub_wp_      = create_publisher<visualization_msgs::msg::MarkerArray>("~/waypoints", latched);

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "odometry", rclcpp::QoS(5), [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          std::scoped_lock lck(mtx_);
          pos_   = Vec3{msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z};
          frame_ = msg->header.frame_id;
        });

    cli_goto_ = create_client<rbl_msgs::srv::Vec4>("rbl_controller/goto");
    cli_goto_path_ = create_client<rbl_msgs::srv::GotoPath>("rbl_controller/goto_path");
    cli_act_  = create_client<std_srvs::srv::Trigger>("rbl_controller/activation");

    srv_inspect_ = create_service<rbl_msgs::srv::Vec4>(
        "~/inspect", std::bind(&WarehouseInspection::cbInspect, this, std::placeholders::_1, std::placeholders::_2));
    srv_abort_ = create_service<std_srvs::srv::Trigger>(
        "~/abort", std::bind(&WarehouseInspection::cbAbort, this, std::placeholders::_1, std::placeholders::_2));

    goal_start_ = now();  // same clock type as every later now() (sim time)
    last_log_   = goal_start_;
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&WarehouseInspection::cbTimer, this));

    publishState("IDLE: waiting for ~/inspect [x, y, z, side]");
  }

private:
  // ---------------------------------------------------------------------------------------------
  // services
  // ---------------------------------------------------------------------------------------------

  void cbInspect(const std::shared_ptr<rbl_msgs::srv::Vec4::Request> req,
                 std::shared_ptr<rbl_msgs::srv::Vec4::Response>      res)
  {
    std::scoped_lock lck(mtx_);
    if (state_ != State::IDLE && state_ != State::DONE) {
      res->success = false;
      res->message = std::string("mission already running (") + toString(state_) + "), call ~/abort first";
      return;
    }
    if (!pos_ || pos_->z < min_alt_) {
      res->success = false;
      res->message = "UAV not airborne yet (no odometry or below mission.min_altitude)";
      return;
    }
    const Vec3 start = *pos_;
    const auto pl    = planner_->plan(Vec3{req->goal[0], req->goal[1], req->goal[2]}, req->goal[3] >= 0.0, start);
    if (!pl.ok) {
      res->success = false;
      res->message = pl.error;
      RCLCPP_WARN(get_logger(), "inspect rejected: %s", pl.error.c_str());
      return;
    }

    start_ = start;
    shelf_ = "aisle " + std::to_string(pl.aisle + 1) + " segment " + std::to_string(pl.segment + 1) + " " +
             (pl.left ? "left" : "right") + " shelf";
    goals_.clear();
    goals_.push_back({pl.entry, State::GOTO, tol_, "aisle position"});
    for (size_t i = 0; i < pl.approach.size(); ++i) {
      goals_.push_back({pl.approach[i], State::GOTO, tol_pass_, "approach via " + std::to_string(i + 1)});
    }
    for (size_t i = 0; i < pl.scan.size(); ++i) {
      const bool stop = i == 0 || i + 1 == pl.scan.size();
      goals_.push_back({pl.scan[i], State::COVERAGE, stop ? tol_ : tol_pass_,
                        "scan " + std::to_string(i + 1) + "/" + std::to_string(pl.scan.size())});
    }
    for (size_t i = 0; i < pl.leave.size(); ++i) {
      goals_.push_back({pl.leave[i], State::RETURN, tol_pass_, "leave via " + std::to_string(i + 1)});
    }
    goals_.push_back({pl.exit, State::RETURN, return_start_ ? tol_pass_ : tol_, "leave the aisle"});
    if (return_start_) goals_.push_back({start, State::RETURN, tol_, "previous position"});
    end_ = goals_.back().p;
    n_coverage_ = pl.scan.size();

    char buf[400];
    std::snprintf(buf, sizeof(buf),
                  "%s: lane y=%.2f, x %.1f -> %.1f, %zu scan goals from %zu samples (%zu shifted, %zu hopped, %zu "
                  "blocked, %zu legs routed), max lane deviation %.2f m, max level deviation %.2f m, min clearance %.2f m",
                  shelf_.c_str(), pl.lane_y, pl.x_near, pl.x_far, pl.scan.size(), pl.samples, pl.shifted, pl.hopped,
                  pl.blocked, pl.routed, pl.max_lane_dev, pl.max_level_dev, pl.min_clearance);
    RCLCPP_INFO(get_logger(), "inspect accepted, %s; return position [%.1f, %.1f, %.1f]", buf, start.x, start.y,
                start.z);
    for (const auto& n : pl.notes) {
      RCLCPP_WARN(get_logger(), "  plan: %s", n.c_str());
    }
    publishPath();

    if (cli_act_->service_is_ready()) {
      cli_act_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    } else {
      RCLCPP_WARN(get_logger(), "rbl_controller/activation not available, assuming it is already active");
    }

    reached_ = 0;
    idx_     = 0;
    state_   = State::IDLE;
    startGoal();
    res->success = true;
    res->message = buf;
  }

  void cbAbort(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    std::scoped_lock lck(mtx_);
    goals_.clear();
    state_ = State::IDLE;
    if (pos_) {
      sendGoal(*pos_);
    }
    publishState("ABORTED: holding at the current position");
    res->success = true;
    res->message = "aborted";
  }

  // ---------------------------------------------------------------------------------------------
  // execution (mtx_ held by the callers)
  // ---------------------------------------------------------------------------------------------

  void startGoal()
  {
    const Goal& g = goals_[idx_];
    if (g.task != state_) {
      state_ = g.task;
      switch (state_) {
        case State::GOTO: publishState("GOTO: flying to the aisle position for the " + shelf_); break;
        case State::COVERAGE: publishState("COVERAGE: scanning the " + shelf_); break;
        case State::RETURN:
          publishState("RETURN: coverage " + std::to_string(reached_) + "/" + std::to_string(n_coverage_) +
                       (return_start_ ? ", flying back to the previous position" : ", leaving the aisle"));
          break;
        default: break;
      }
    }
    const double d = pos_ ? rbl_mission::dist(*pos_, g.p) : 0.0;
    goal_timeout_  = timeout_base_ + timeout_per_m_ * d;
    goal_start_    = now();
    last_log_      = goal_start_;
    goal_sent_     = sendGoal(g.p);
    publishWaypoints();
    RCLCPP_INFO(get_logger(), "  %s: goal [%.1f, %.1f, %.1f] (%.1f m away, timeout %.0f s)", g.label.c_str(), g.p.x,
                g.p.y, g.p.z, d, goal_timeout_);
  }

  void cbTimer()
  {
    std::scoped_lock lck(mtx_);
    if (goals_.empty() || idx_ >= goals_.size() || !pos_) {
      return;
    }
    const Goal& g = goals_[idx_];
    if (!goal_sent_) {  // rbl_controller/goto wasn't up yet: keep retrying, the timeout starts once sent
      goal_sent_  = sendGoal(g.p);
      goal_start_ = now();
      return;
    }
    const double d  = rbl_mission::dist(*pos_, g.p);
    const double el = (now() - goal_start_).seconds();

    bool advance = false;
    if (d < g.tol) {
      advance = true;
      if (g.task == State::COVERAGE) {
        ++reached_;
      }
    } else if (el > goal_timeout_) {
      RCLCPP_WARN(get_logger(), "  %s: not reached after %.0f s (%.1f m left), moving on", g.label.c_str(),
                  goal_timeout_, d);
      advance = true;
    } else if ((now() - last_log_).seconds() > 5.0) {
      last_log_ = now();
      RCLCPP_INFO(get_logger(), "  %s: %.1f m to go", g.label.c_str(), d);
    }
    if (!advance) {
      return;
    }

    if (++idx_ < goals_.size()) {
      startGoal();
      return;
    }
    publishWaypoints();  // all reached
    goals_.clear();
    state_ = State::DONE;
    char buf[200];
    std::snprintf(buf, sizeof(buf), "DONE: %s [%.1f, %.1f, %.1f], %s covered %zu/%zu",
                  return_start_ ? "back at" : "hovering at", end_.x, end_.y, end_.z, shelf_.c_str(), reached_,
                  n_coverage_);
    publishState(buf);
  }

  bool sendGoal(const Vec3& p)
  {
    if (global_ && pos_) {
      if (!cli_goto_path_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "rbl_controller/goto_path not available yet, retrying");
        return false;
      }
      auto req                  = std::make_shared<rbl_msgs::srv::GotoPath::Request>();
      req->path.header.frame_id = frame_;
      req->path.header.stamp    = now();
      for (const Vec3& q : {*pos_, p}) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header             = req->path.header;
        ps.pose.position.x    = q.x;
        ps.pose.position.y    = q.y;
        ps.pose.position.z    = q.z;
        ps.pose.orientation.w = 1.0;
        req->path.poses.push_back(ps);
      }
      cli_goto_path_->async_send_request(req);
    } else {
      if (!cli_goto_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "rbl_controller/goto not available yet, retrying");
        return false;
      }
      auto req  = std::make_shared<rbl_msgs::srv::Vec4::Request>();
      req->goal = {p.x, p.y, p.z, 0.0};
      cli_goto_->async_send_request(req);
    }

    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id    = frame_;
    ps.header.stamp       = now();
    ps.pose.position.x    = p.x;
    ps.pose.position.y    = p.y;
    ps.pose.position.z    = p.z;
    ps.pose.orientation.w = 1.0;
    pub_goal_->publish(ps);
    return true;
  }

  void publishPath()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame_;
    path.header.stamp    = now();
    for (const auto& g : goals_) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header             = path.header;
      ps.pose.position.x    = g.p.x;
      ps.pose.position.y    = g.p.y;
      ps.pose.position.z    = g.p.z;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    pub_path_->publish(path);
  }

  // one sphere + index label per goal; goals before idx_ reached (grey), idx_ current (green)
  void publishWaypoints()
  {
    using visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray arr;
    Marker clear;
    clear.header.frame_id = frame_;
    clear.action          = Marker::DELETEALL;
    arr.markers.push_back(clear);
    for (size_t i = 0; i < goals_.size(); ++i) {
      const Goal& g       = goals_[i];
      const bool  current = i == idx_;
      const bool  reached = i < idx_;
      Marker m;
      m.header.frame_id    = frame_;
      m.header.stamp       = now();
      m.ns                 = "waypoints";
      m.id                 = static_cast<int>(i);
      m.type               = Marker::SPHERE;
      m.action             = Marker::ADD;
      m.pose.position.x    = g.p.x;
      m.pose.position.y    = g.p.y;
      m.pose.position.z    = g.p.z;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = current ? 0.35 : 0.2;
      m.color.a = 1.0;
      if (current) {
        m.color.r = 0.1, m.color.g = 1.0, m.color.b = 0.2;
      } else if (reached) {
        m.color.r = m.color.g = m.color.b = 0.5, m.color.a = 0.6;
      } else if (g.task == State::COVERAGE) {
        m.color.r = 0.0, m.color.g = 0.8, m.color.b = 1.0;
      } else if (g.task == State::RETURN) {
        m.color.r = 1.0, m.color.g = 0.2, m.color.b = 0.9;
      } else {
        m.color.r = 1.0, m.color.g = 0.85, m.color.b = 0.1;
      }
      arr.markers.push_back(m);

      Marker t          = m;
      t.ns              = "waypoint_ids";
      t.type            = Marker::TEXT_VIEW_FACING;
      t.pose.position.z = g.p.z + 0.3;
      t.scale.x = t.scale.y = 0.0;
      t.scale.z         = 0.25;
      t.color.r = t.color.g = t.color.b = 1.0;
      t.color.a         = reached ? 0.5 : 1.0;
      t.text            = std::to_string(i);
      arr.markers.push_back(t);
    }
    pub_wp_->publish(arr);
  }

  void publishState(const std::string& text)
  {
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
    std_msgs::msg::String m;
    m.data = text;
    pub_state_->publish(m);
  }

  std::unique_ptr<rbl_mission::CoveragePlanner> planner_;
  double                                        tol_, tol_pass_, timeout_base_, timeout_per_m_, min_alt_;
  bool                                          return_start_;

  // state
  std::mutex          mtx_;
  std::optional<Vec3> pos_;
  std::string         frame_{"world_origin"};
  State               state_{State::IDLE};
  std::vector<Goal>   goals_;
  size_t              idx_{0}, reached_{0}, n_coverage_{0};
  Vec3                start_, end_;
  std::string         shelf_;
  rclcpp::Time        goal_start_, last_log_;
  double              goal_timeout_{0};
  bool                goal_sent_{false};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           pub_state_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             pub_path_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_wp_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      sub_odom_;
  rclcpp::Client<rbl_msgs::srv::Vec4>::SharedPtr                cli_goto_;
  rclcpp::Client<rbl_msgs::srv::GotoPath>::SharedPtr            cli_goto_path_;
  bool                                                          global_{true};
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr             cli_act_;
  rclcpp::Service<rbl_msgs::srv::Vec4>::SharedPtr               srv_inspect_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr            srv_abort_;
  rclcpp::TimerBase::SharedPtr                                  timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WarehouseInspection>();
  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  ex.spin();
  rclcpp::shutdown();
  return 0;
}
