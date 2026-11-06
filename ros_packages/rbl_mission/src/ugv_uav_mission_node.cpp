// UGV + UAV warehouse shelf inspection. The UGV (PX4 differential rover) carries the UAV (PX4
// multicopter) on its deck; both run the unchanged 3D rbl_controller, which only ever receives
// ~/goto goals and ~/activation from here.
//
// One ~/deploy request (rbl_mission/srv/Deploy) runs the phases below for one shelf; ~/deploy_sequence
// (rbl_mission/srv/DeploySequence) runs them for several shelves in order, the UGV driving straight on
// from ATTACH to the next shelf's DRIVE with the UAV on board, and HOME only after the last one:
//   DRIVE      UGV: activation + goto a free stop at (shelf_x, shelf_y) (planner groundStop), until it
//              is there and stopped; then its rbl_controller is deactivated (px4_bridge holds the last
//              setpoint), so it stands still there until the UAV is back on it (reactivated for HOME)
//   TAKEOFF    release the UAV from the deck (std_msgs/Empty on detach_topic, bridged to the gz
//              DetachableJoint), UAV px4_bridge ~/arm (takeoff_at_current_position), until it climbed
//              above mission.takeoff_check_altitude
//   INSPECT    UAV warehouse_inspection ~/inspect [shelf_x, shelf_y, <current height>, side]: shelf
//              coverage, then the UAV leaves the aisle where the coverage ended and hovers there
//              (warehouse_inspection mission.return_to_start:=false; with true it flies back over the
//              takeoff spot and PICKUP finds the UGV already below it)
//   PICKUP     the UGV comes to the UAV: activation + route (aisle grid) to a free ground stop below the
//              hovering UAV (planner groundStop), until it is there and stopped, then deactivated again.
//              Skipped if the UGV is already within mission.pickup_skip_distance; if there is no free
//              stop below the UAV, the UGV stays and the UAV flies to it instead (RENDEZVOUS)
//   RENDEZVOUS UAV rbl_controller goto 1.2 m above the UGV's deck; within mission.handover_distance,
//              rbl_controller is deactivated and this node holds the UAV exactly above the deck centre
//              (publishing the reference rbl would, on /<uav>/reference -> px4_bridge): rbl keeps
//              ~0.2 m off a goal right above an obstacle (the UGV, in its own cloud), too much for a
//              0.4 m deck. Until it hovers within mission.rendezvous_tolerance and still
//   LAND       mission.landing "px4_precland" (default): PX4's built-in precision landing. As soon
//              as the UAV is within mission.handover_distance above the deck (RENDEZVOUS rbl transit),
//              its rbl_controller is deactivated and px4_bridge ~/precland switches PX4 to
//              AUTO_PRECLAND: PX4 centres over the ArUco marker on the deck (down camera ->
//              aruco_landing_target.py -> MAVLink TARGET_RELATIVE -> vision_target_estimator ->
//              landing_target_pose), descends onto it and auto-disarms (PLD_* parameters in
//              config/uav_px4.yaml). No precise hold by this node in this mode.
//              mission.landing "tracked": this node keeps the UAV's xy reference on the live landing target
//              (the deck centre: UGV odometry, or mission.landing_target_topic, e.g. a marker detector)
//              and lowers its z at mission.precland_speed only while the UAV is within
//              mission.precland_xy_tolerance of it -- holding height beyond that and climbing back
//              towards the rendezvous height beyond mission.precland_abort_xy. At
//              mission.precland_touchdown_height above the deck, px4_bridge ~/land (PX4 Land mode)
//              does the touchdown, until PX4 has landed and auto-disarmed
//   ATTACH     UAV px4_bridge ~/disarm (forced; PX4's own auto-disarm needs its land detector, which
//              may not trigger on the deck), std_msgs/Empty on attach_topic (gz DetachableJoint holds
//              the UAV again); its rbl_controller stays deactivated, so the next takeoff starts from
//              wherever it is carried. Next shelf of the sequence (DRIVE), or HOME after the last
//   HOME       UGV drives back, carrying the UAV, to where it was when ~/deploy was called
//   DONE       ready for the next ~/deploy
//
// The UGV's legs (to the stop, to the UAV, home), planner.mode:
//   global (default) planned ONCE per leg, start -> goal, by A* on the known map (GlobalPlanner2D on
//          the world SDF's obstacles, global_planner.hpp) and handed whole to rbl_controller ~/goto_path:
//          it follows that path without its local replanner, rbl's reactive avoidance on top. The
//          UAV's legs from here (RENDEZVOUS) go as straight paths too. Published on ~/ugv_global_path.
//   local  the aisle grid (CoveragePlanner::groundRoute: along its aisle to a cross-aisle, through it,
//          into the target aisle), one intermediate rbl_controller ~/goto at a time, rbl's local
//          replanner routing in between. (With only the goal, its local view -- 180 deg forward lidar, 15 m
//          local map -- sometimes picked a wrong corridor, seen in sim.)
//
// Interface (global names from the ugv / uav parameters):
//   ~/deploy   rbl_mission/srv/Deploy        start; while a deployment runs it replaces it: the current
//              shelf is abandoned, a flying UAV stops inspecting and is picked up / lands on the UGV
//              first (PICKUP -> RENDEZVOUS -> LAND -> ATTACH), then the new shelves (home stays the
//              start of the first deployment). Also picks up a UAV left in the air by ~/abort or a failure
//   ~/deploy_sequence rbl_mission/srv/DeploySequence  several shelves in order (every one validated first)
//   ~/abort    std_srvs/Trigger              UGV and UAV hold where they are
//   ~/state    std_msgs/String (latched)     current phase + info

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rbl_msgs/msg/reference_stamped.hpp>
#include <rbl_msgs/srv/goto_path.hpp>
#include <rbl_msgs/srv/vec4.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rbl_mission/coverage_planner.hpp"
#include "rbl_mission/global_planner.hpp"
#include "rbl_mission/planner_params.hpp"
#include "rbl_mission/srv/deploy.hpp"
#include "rbl_mission/srv/deploy_sequence.hpp"

using rbl_mission::Vec3;

namespace
{
enum class Phase
{
  IDLE,
  DRIVE,
  TAKEOFF,
  INSPECT,
  PICKUP,
  RENDEZVOUS,
  LAND,
  ATTACH,
  HOME,
  DONE
};

const char* toString(Phase p)
{
  switch (p) {
    case Phase::IDLE: return "IDLE";
    case Phase::DRIVE: return "DRIVE";
    case Phase::TAKEOFF: return "TAKEOFF";
    case Phase::INSPECT: return "INSPECT";
    case Phase::PICKUP: return "PICKUP";
    case Phase::RENDEZVOUS: return "RENDEZVOUS";
    case Phase::LAND: return "LAND";
    case Phase::ATTACH: return "ATTACH";
    case Phase::HOME: return "HOME";
    case Phase::DONE: return "DONE";
  }
  return "?";
}

double distXY(const Vec3& a, const Vec3& b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}
}  // namespace

class UgvUavMission : public rclcpp::Node
{
public:
  UgvUavMission() : Node("ugv_uav_mission")
  {
    planner_ = rbl_mission::makePlannerFromParams(*this);

    ugv_ = declare_parameter<std::string>("ugv", "ugv1");
    uav_ = declare_parameter<std::string>("uav", "uav1");
    const auto detach_topic = declare_parameter<std::string>("detach_topic", "/ugv1/uav_detach");
    const auto attach_topic = declare_parameter<std::string>("attach_topic", "/ugv1/uav_attach");
    ugv_clearance_      = declare_parameter<double>("mission.ugv_clearance", 0.6);
    ugv_tol_            = declare_parameter<double>("mission.ugv_tolerance", 0.5);
    ugv_stopped_speed_  = declare_parameter<double>("mission.ugv_stopped_speed", 0.1);
    takeoff_alt_        = declare_parameter<double>("mission.takeoff_check_altitude", 1.6);
    drive_timeout_      = declare_parameter<double>("mission.drive_timeout", 240.0);
    takeoff_timeout_    = declare_parameter<double>("mission.takeoff_timeout", 60.0);
    rdv_height_         = declare_parameter<double>("mission.rendezvous_height", 1.2);     // above the deck
    rdv_tol_            = declare_parameter<double>("mission.rendezvous_tolerance", 0.06); // [m] xy
    rdv_speed_          = declare_parameter<double>("mission.rendezvous_speed", 0.08);
    rdv_timeout_        = declare_parameter<double>("mission.rendezvous_timeout", 90.0);
    handover_dist_      = declare_parameter<double>("mission.handover_distance", 0.5);   // [m] rbl -> precise hold
    deck_tol_           = declare_parameter<double>("mission.deck_tolerance", 0.15);        // [m] xy, landed on it
    land_timeout_       = declare_parameter<double>("mission.land_timeout", 40.0);
    route_pass_tol_     = declare_parameter<double>("mission.route_pass_tolerance", 0.8);  // [m] intermediate points
    pickup_skip_        = declare_parameter<double>("mission.pickup_skip_distance", 0.5);  // [m] UGV already below
    pl_speed_           = declare_parameter<double>("mission.precland_speed", 0.25);       // [m/s] descent
    pl_xy_tol_          = declare_parameter<double>("mission.precland_xy_tolerance", 0.08); // [m] descend within
    pl_abort_xy_        = declare_parameter<double>("mission.precland_abort_xy", 0.25);    // [m] climb back beyond
    pl_touchdown_h_     = declare_parameter<double>("mission.precland_touchdown_height", 0.25);  // [m] -> PX4 Land (> the 0.15 m "down" check of LAND)
    pl_timeout_         = declare_parameter<double>("mission.precland_timeout", 60.0);
    const auto target_topic = declare_parameter<std::string>("mission.landing_target_topic", "");
    target_timeout_     = declare_parameter<double>("mission.landing_target_timeout", 0.5);  // [s]
    const auto landing  = declare_parameter<std::string>("mission.landing", "px4_precland");  // or "tracked"
    if (landing != "px4_precland" && landing != "tracked") {
      throw std::invalid_argument("mission.landing must be \"px4_precland\" or \"tracked\", got \"" + landing + "\"");
    }
    px4_precland_       = landing == "px4_precland";
    px4_land_timeout_   = declare_parameter<double>("mission.px4_precland_timeout", 120.0);  // [s] incl. PX4's target search
    RCLCPP_INFO(get_logger(), "landing on the UGV: %s", px4_precland_ ? "PX4 precision landing (AUTO_PRECLAND, ArUco)"
                                                                      : "tracked descent by this node");

    // planner.mode "global" (default): every UGV leg is planned once, start -> goal, on the known map
    // (GlobalPlanner2D on the world SDF's obstacles) and handed to rbl_controller/goto_path, which follows it
    // without its local replanner. "local": the aisle-grid route (groundRoute) one rbl_controller/goto at
    // a time, rbl's local replanner does the routing in between.
    const auto mode = declare_parameter<std::string>("planner.mode", "global");
    if (mode != "global" && mode != "local") {
      throw std::invalid_argument("planner.mode must be \"global\" or \"local\", got \"" + mode + "\"");
    }
    global_ = mode == "global";
    rbl_mission::GlobalPlanner2D::Params gp;
    gp.resolution          = declare_parameter<double>("planner.resolution", gp.resolution);
    gp.clearance           = declare_parameter<double>("planner.ugv_clearance", ugv_clearance_);
    gp.preferred_clearance = declare_parameter<double>("planner.ugv_preferred_clearance", gp.preferred_clearance);
    gp.clearance_weight    = declare_parameter<double>("planner.clearance_weight", gp.clearance_weight);
    gp.z_lo                = declare_parameter<double>("planner.ugv_z_lo", gp.z_lo);
    gp.z_hi                = declare_parameter<double>("planner.ugv_z_hi", gp.z_hi);
    gp.snap_radius         = declare_parameter<double>("planner.snap_radius", gp.snap_radius);
    gp.sample_spacing      = declare_parameter<double>("planner.sample_spacing", gp.sample_spacing);
    gplanner_ = std::make_unique<rbl_mission::GlobalPlanner2D>(planner_->obstacles(), gp);
    RCLCPP_INFO(get_logger(), "planner: %s", global_ ? "global (UGV legs planned once on the map, rbl_controller/goto_path)"
                                                     : "local (aisle-grid route, rbl_controller/goto + its local replanner)");

    auto latched = rclcpp::QoS(1).transient_local();
    pub_state_   = create_publisher<std_msgs::msg::String>("~/state", latched);
    pub_detach_  = create_publisher<std_msgs::msg::Empty>(detach_topic, rclcpp::QoS(1));
    pub_attach_  = create_publisher<std_msgs::msg::Empty>(attach_topic, rclcpp::QoS(1));
    pub_uav_ref_ = create_publisher<rbl_msgs::msg::ReferenceStamped>("/" + uav_ + "/reference", rclcpp::QoS(1));
    pub_ugv_path_ = create_publisher<nav_msgs::msg::Path>("~/ugv_global_path", latched);

    auto odom_cb = [this](std::optional<Vec3>& pos, std::optional<double>& speed, double& yaw) {
      return [this, &pos, &speed, &yaw](nav_msgs::msg::Odometry::ConstSharedPtr m) {
        std::scoped_lock lck(mtx_);
        pos   = Vec3{m->pose.pose.position.x, m->pose.pose.position.y, m->pose.pose.position.z};
        const auto& q = m->pose.pose.orientation;
        yaw   = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
        speed = std::hypot(m->twist.twist.linear.x, m->twist.twist.linear.y, m->twist.twist.linear.z);
      };
    };
    sub_ugv_odom_ = create_subscription<nav_msgs::msg::Odometry>("/" + ugv_ + "/odometry", rclcpp::QoS(5),
                                                                 odom_cb(ugv_pos_, ugv_speed_, ugv_yaw_));
    sub_uav_odom_ = create_subscription<nav_msgs::msg::Odometry>("/" + uav_ + "/odometry", rclcpp::QoS(5),
                                                                 odom_cb(uav_pos_, uav_speed_, uav_yaw_));
    // optional external landing target (deck centre in the control frame, e.g. from a marker detector
    // on a downward camera); without it the deck centre is the UGV's odometry
    if (!target_topic.empty()) {
      sub_target_ = create_subscription<geometry_msgs::msg::PointStamped>(
          target_topic, rclcpp::SensorDataQoS(), [this](geometry_msgs::msg::PointStamped::ConstSharedPtr m) {
            std::scoped_lock lck(mtx_);
            target_      = Vec3{m->point.x, m->point.y, m->point.z};
            target_time_ = now();
          });
      RCLCPP_INFO(get_logger(), "precision landing target: %s", target_topic.c_str());
    }
    sub_uav_state_ = create_subscription<std_msgs::msg::String>(
        "/" + uav_ + "/warehouse_inspection/state", latched, [this](std_msgs::msg::String::ConstSharedPtr m) {
          std::scoped_lock lck(mtx_);
          inspection_state_ = m->data;
        });

    cli_ugv_goto_ = create_client<rbl_msgs::srv::Vec4>("/" + ugv_ + "/rbl_controller/goto");
    cli_ugv_act_  = create_client<std_srvs::srv::Trigger>("/" + ugv_ + "/rbl_controller/activation");
    cli_ugv_deact_ = create_client<std_srvs::srv::Trigger>("/" + ugv_ + "/rbl_controller/deactivation");
    cli_uav_arm_  = create_client<std_srvs::srv::Trigger>("/" + uav_ + "/px4_bridge/arm");
    cli_uav_land_ = create_client<std_srvs::srv::Trigger>("/" + uav_ + "/px4_bridge/land");
    cli_uav_precland_ = create_client<std_srvs::srv::Trigger>("/" + uav_ + "/px4_bridge/precland");
    cli_uav_inspect_ = create_client<rbl_msgs::srv::Vec4>("/" + uav_ + "/warehouse_inspection/inspect");
    cli_uav_abort_   = create_client<std_srvs::srv::Trigger>("/" + uav_ + "/warehouse_inspection/abort");
    cli_uav_goto_    = create_client<rbl_msgs::srv::Vec4>("/" + uav_ + "/rbl_controller/goto");
    cli_uav_deact_   = create_client<std_srvs::srv::Trigger>("/" + uav_ + "/rbl_controller/deactivation");
    cli_uav_act_     = create_client<std_srvs::srv::Trigger>("/" + uav_ + "/rbl_controller/activation");
    cli_uav_disarm_  = create_client<std_srvs::srv::Trigger>("/" + uav_ + "/px4_bridge/disarm");
    cli_ugv_goto_path_ = create_client<rbl_msgs::srv::GotoPath>("/" + ugv_ + "/rbl_controller/goto_path");
    cli_uav_goto_path_ = create_client<rbl_msgs::srv::GotoPath>("/" + uav_ + "/rbl_controller/goto_path");

    srv_deploy_ = create_service<rbl_mission::srv::Deploy>(
        "~/deploy", [this](const std::shared_ptr<rbl_mission::srv::Deploy::Request> req,
                           std::shared_ptr<rbl_mission::srv::Deploy::Response>      res) {
          std::string msg;
          res->success = startSequence({*req}, msg);
          res->message = msg;
        });
    srv_deploy_seq_ = create_service<rbl_mission::srv::DeploySequence>(
        "~/deploy_sequence", [this](const std::shared_ptr<rbl_mission::srv::DeploySequence::Request> req,
                                    std::shared_ptr<rbl_mission::srv::DeploySequence::Response>      res) {
          std::string msg;
          res->success = false;
          if (req->shelf_x.size() != req->shelf_y.size() || req->shelf_x.size() != req->side.size()) {
            res->message = "shelf_x, shelf_y and side must have the same length";
            return;
          }
          std::vector<rbl_mission::srv::Deploy::Request> targets(req->shelf_x.size());
          for (size_t i = 0; i < targets.size(); ++i) {
            targets[i].shelf_x = req->shelf_x[i];
            targets[i].shelf_y = req->shelf_y[i];
            targets[i].side    = req->side[i];
          }
          res->success = startSequence(targets, msg);
          res->message = msg;
        });
    srv_abort_ = create_service<std_srvs::srv::Trigger>(
        "~/abort", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          std::scoped_lock lck(mtx_);
          const bool flying = phase_ == Phase::TAKEOFF || phase_ == Phase::INSPECT || phase_ == Phase::PICKUP ||
                              phase_ == Phase::RENDEZVOUS || (phase_ == Phase::LAND && land_stage_ == 0);
          if (ugv_pos_) gotoAsync(cli_ugv_goto_, *ugv_pos_);
          if (flying) callTrigger(cli_uav_abort_, "uav inspection abort");
          if ((phase_ == Phase::RENDEZVOUS || phase_ == Phase::LAND) && uav_pos_) {
            // rbl may be deactivated (precise hold / descent): hover where it is, rbl active again
            callTrigger(cli_uav_act_, "uav rbl activation");
            gotoAsync(cli_uav_goto_, *uav_pos_);
          }
          phase_ = Phase::IDLE;
          route_.clear();
          targets_.clear();
          pending_.reset();
          setState(std::string("ABORTED: UGV holds where it is") + (flying ? ", UAV too" : ""));
          res->success = true;
          res->message = "aborted";
        });

    phase_start_ = now();  // same clock type as every later now() (sim time)
    last_log_    = phase_start_;
    timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&UgvUavMission::cbTimer, this));
    setState("IDLE: waiting for ~/deploy");
  }

private:
  struct Target
  {
    rbl_mission::srv::Deploy::Request req;
    Vec3                              stop;
  };

  // ---------------------------------------------------------------------------------------------
  bool uavOnDeck() const
  {
    return uav_pos_->z - ugv_pos_->z < 1.0 && distXY(*uav_pos_, *ugv_pos_) < 0.4;
  }
  // in the air (not on the deck, not on the floor): can still be picked up
  bool uavAirborne() const
  {
    return !uavOnDeck() && uav_pos_->z > ugv_pos_->z;
  }

  // validates every target (aisle stop for the UGV, plannable shelf for the UAV), then starts the first.
  // While a deployment runs, the new one replaces it (see preempt())
  bool startSequence(const std::vector<rbl_mission::srv::Deploy::Request>& targets, std::string& msg)
  {
    std::scoped_lock lck(mtx_);
    auto fail = [&](const std::string& m) {
      msg = m;
      RCLCPP_WARN(get_logger(), "deploy rejected: %s", m.c_str());
      return false;
    };
    if (targets.empty()) return fail("no shelf given");
    if (!ugv_pos_ || !uav_pos_) return fail("no odometry from the UGV and/or the UAV yet");
    const bool busy = phase_ != Phase::IDLE && phase_ != Phase::DONE;
    if (!busy && !uavOnDeck() && !uavAirborne()) return fail("the UAV is neither on the UGV nor flying");

    std::vector<Target> checked;
    std::string         info;
    Vec3                from = *ugv_pos_;
    for (size_t i = 0; i < targets.size(); ++i) {
      const auto& req = targets[i];
      const auto  tag = "shelf " + std::to_string(i + 1) + "/" + std::to_string(targets.size()) + ": ";
      if (req.side != "left" && req.side != "right") return fail(tag + "side must be \"left\" or \"right\"");
      std::string why;
      const auto  stop = planner_->groundStop(req.shelf_x, req.shelf_y, ugv_pos_->z, ugv_clearance_, why);
      if (!stop) return fail(tag + "shelf stop: " + why);
      // the shelf itself must be plannable for the UAV (same check its inspection node will do)
      const auto pl = planner_->plan(Vec3{req.shelf_x, req.shelf_y, 0.0}, req.side == "left",
                                     Vec3{stop->x, stop->y, takeoff_alt_});
      if (!pl.ok) return fail(tag + "shelf: " + pl.error);
      // ... and reachable for the UGV on the map (only a check: each leg is planned when it starts)
      if (global_ && !gplanner_->plan(from, *stop, why)) return fail(tag + "UGV: " + why);
      from = *stop;
      checked.push_back(Target{req, *stop});
      char buf[160];
      std::snprintf(buf, sizeof(buf), "%s%d: aisle %d segment %d %s [%.1f, %.1f]", i ? ", " : "",
                    static_cast<int>(i + 1), pl.aisle + 1, pl.segment + 1, req.side.c_str(), stop->x, stop->y);
      info += buf;
    }
    // home: where the UGV was when the first (not preempted) deployment started
    const Vec3 home = busy || !uavOnDeck() ? home_ : *ugv_pos_;
    if (global_) {
      std::string why;
      if (!gplanner_->plan(from, home, why)) return fail("UGV back home: " + why);
    }

    char buf[160];
    std::snprintf(buf, sizeof(buf), " -- then home [%.1f, %.1f]", home.x, home.y);
    msg = std::to_string(checked.size()) + " shelf(s): " + info + buf;
    home_ = home;

    if (!busy && uavOnDeck()) {
      landed_z_ = uav_pos_->z;  // the UAV's height sitting on the deck
      RCLCPP_INFO(get_logger(), "deploy: %s", msg.c_str());
      pending_ = std::move(checked);
      takePending();
      return true;
    }
    msg = preempt(std::move(checked)) + "; new deployment: " + msg;
    RCLCPP_INFO(get_logger(), "deploy (preempting): %s", msg.c_str());
    return true;
  }

  // A new deployment while one runs (or while the UAV is still in the air after an abort / failure):
  // the current shelf is abandoned. UAV on the deck -> the UGV drives to the new first shelf right
  // away; UAV flying -> it stops inspecting, the UGV picks it up where it is (PICKUP -> RENDEZVOUS ->
  // LAND -> ATTACH as usual), and the new shelves start after ATTACH.
  std::string preempt(std::vector<Target> targets)
  {
    pending_ = std::move(targets);
    switch (phase_) {
      case Phase::DRIVE:
      case Phase::HOME:
      case Phase::IDLE:  // UAV on the deck is handled by the caller: here it is flying
      case Phase::DONE:
        if (uavOnDeck()) {
          takePending();
          return "UAV on board, UGV heads to the new shelf";
        }
        callTrigger(cli_uav_act_, "uav rbl activation");
        startPickup("UAV left in the air");
        return "UAV in the air: picking it up first";
      case Phase::TAKEOFF:
        return "UAV taking off: picked up again as soon as it is up";  // see cbTimer TAKEOFF
      case Phase::INSPECT:
        callTrigger(cli_uav_abort_, "uav inspection abort");
        startPickup("inspection abandoned");
        return "inspection abandoned, UGV picks up the UAV";
      case Phase::PICKUP:
      case Phase::RENDEZVOUS:
      case Phase::LAND:
        return std::string("UAV already being picked up (") + toString(phase_) + ")";
      case Phase::ATTACH:
        return "UAV just landed";  // see cbTimer ATTACH
    }
    return "";
  }

  // pending_ -> the shelves of this deployment, DRIVE to the first
  void takePending()
  {
    targets_    = std::move(*pending_);
    pending_.reset();
    target_idx_ = 0;
    startTarget();
  }

  // DRIVE to targets_[target_idx_] with the UAV on board
  void startTarget()
  {
    const auto& t = targets_[target_idx_];
    req_  = t.req;
    stop_ = t.stop;
    char buf[300];
    std::snprintf(buf, sizeof(buf),
                  "shelf %zu/%zu: UGV -> [%.1f, %.1f] and stops, UAV takes off, inspects the %s shelf, UGV picks it "
                  "up where it ends (precision landing), then %s",
                  target_idx_ + 1, targets_.size(), stop_.x, stop_.y, req_.side.c_str(),
                  target_idx_ + 1 < targets_.size() ? "on to the next shelf" : "home");
    plan_info_ = buf;
    callTrigger(cli_ugv_act_, "ugv activation");
    startRoute(stop_);
    enter(Phase::DRIVE, "DRIVE: UGV carrying the UAV to [" + fmt2(stop_) + "] (" + plan_info_ + ")");
  }

  // ---------------------------------------------------------------------------------------------
  void cbTimer()
  {
    std::scoped_lock lck(mtx_);
    if (!ugv_pos_ || !uav_pos_) return;
    const double el = (now() - phase_start_).seconds();

    switch (phase_) {
      case Phase::DRIVE: {
        const double d = routeLeft();
        if (routeDone()) {
          // stop for real: with rbl_controller active the UGV kept creeping towards the exact goal
          // (~0.3 m while the UAV was away, seen in sim); deactivated, px4_bridge holds its last setpoint
          callTrigger(cli_ugv_deact_, "ugv rbl deactivation");
          launch_ = *uav_pos_;
          pub_detach_->publish(std_msgs::msg::Empty());
          callTrigger(cli_uav_arm_, "uav arm");
          enter(Phase::TAKEOFF, "TAKEOFF: UGV at the shelf, UAV released and taking off from [" + fmt2(launch_) + "]");
        } else if (el > drive_timeout_) {
          fault("UGV did not reach the shelf stop in time (" + fmtd(d) + " m left)");
        } else {
          progress("DRIVE", d);
        }
        break;
      }
      case Phase::TAKEOFF: {
        if (uav_pos_->z > takeoff_alt_ && pending_) {  // preempted while taking off: land it again
          callTrigger(cli_uav_act_, "uav rbl activation");
          startPickup("takeoff of an abandoned shelf");
        } else if (uav_pos_->z > takeoff_alt_) {
          // GOTO goal at the current hover height: the UGV is right below the aisle position
          auto r  = std::make_shared<rbl_msgs::srv::Vec4::Request>();
          r->goal = {req_.shelf_x, req_.shelf_y, uav_pos_->z, req_.side == "left" ? 1.0 : -1.0};
          inspection_state_.clear();
          if (!cli_uav_inspect_->service_is_ready()) {
            fault("UAV warehouse_inspection/inspect not available");
            break;
          }
          cli_uav_inspect_->async_send_request(r, [this](rclcpp::Client<rbl_msgs::srv::Vec4>::SharedFuture f) {
            const auto res = f.get();
            std::scoped_lock lck(mtx_);
            if (!res->success && phase_ == Phase::INSPECT) fault("UAV inspection rejected: " + res->message);
          });
          enter(Phase::INSPECT, "INSPECT: UAV scanning the " + req_.side + " shelf, UGV waiting for it at [" +
                                    fmt2(*ugv_pos_) + "]");
        } else if (el > takeoff_timeout_) {
          fault("UAV did not take off (z " + fmtd(uav_pos_->z) + " m)");
        } else if (el > 3.0 && (now() - last_log_).seconds() > 5.0) {
          last_log_ = now();
          pub_detach_->publish(std_msgs::msg::Empty());  // harmless if already released
          callTrigger(cli_uav_arm_, "uav arm (retry)");
        }
        break;
      }
      case Phase::INSPECT: {
        if (inspection_state_.rfind("DONE", 0) == 0) {
          startPickup("shelf done (" + inspection_state_ + ")");
        } else if (inspection_state_.rfind("ABORTED", 0) == 0) {
          fault("UAV inspection aborted");
        }
        break;
      }
      case Phase::PICKUP: {
        const double d = routeLeft();
        if (routeDone()) {
          callTrigger(cli_ugv_deact_, "ugv rbl deactivation");
          enter(Phase::RENDEZVOUS, "RENDEZVOUS: UGV below the UAV (" + fmtd(distXY(*uav_pos_, *ugv_pos_)) +
                                       " m off), UAV descending above its deck");
          rdv_stage_ = 0;
        } else if (el > drive_timeout_) {
          fault("UGV did not reach the UAV in time (" + fmtd(d) + " m left)");
        } else {
          progress("PICKUP", d);
        }
        break;
      }
      case Phase::RENDEZVOUS: {
        const double hover_z = landed_z_ + rdv_height_;
        if (rdv_stage_ == 0) {  // the UGV has been parked at the stop since DRIVE; make sure it is still
          if (ugv_speed_.value_or(1.0) < ugv_stopped_speed_) {
            deck_ = *ugv_pos_;
            gotoUav(Vec3{deck_.x, deck_.y, hover_z});
            rdv_stage_   = 1;
            phase_start_ = now();
            setState("RENDEZVOUS: UAV flying above the UGV at [" + fmt2(deck_) + "]");
          } else if (el > 30.0) {
            fault("UGV is not standing still");
          }
          break;
        }
        const double d = distXY(*uav_pos_, deck_);
        if (rdv_stage_ == 1) {  // rbl transit
          if (d < handover_dist_ && std::abs(uav_pos_->z - hover_z) < 0.3 && px4_precland_) {
            // PX4 does the rest: centring over the marker, descent, touchdown, disarm
            callTrigger(cli_uav_deact_, "uav rbl deactivation");
            callTrigger(cli_uav_precland_, "uav precland");
            land_stage_ = 1;
            enter(Phase::LAND, "LAND: PX4 precision landing on the UGV's ArUco marker (" + fmtd(d * 100) +
                                   " cm off its centre at handover)");
          } else if (d < handover_dist_ && std::abs(uav_pos_->z - hover_z) < 0.3) {
            callTrigger(cli_uav_deact_, "uav rbl deactivation");
            hold_yaw_    = uav_yaw_;
            rdv_stage_   = 2;
            phase_start_ = now();
            setState("RENDEZVOUS: UAV above the UGV (" + fmtd(d * 100) + " cm), precise hold over the deck centre");
          } else if (el > rdv_timeout_) {
            fault("UAV did not get above the UGV (" + fmtd(d) + " m off)");
          } else {
            progress("RENDEZVOUS", d);
          }
          break;
        }
        // precise hold: the reference rbl would publish, straight above the (live) deck centre
        if (const auto t = landingTarget()) deck_ = *t;
        publishUavRef(deck_, hover_z);
        if (d < rdv_tol_ && uav_speed_.value_or(1.0) < rdv_speed_) {
          startPrecisionLanding(d, "");
        } else if (el > 30.0) {
          if (d < deck_tol_) {
            startPrecisionLanding(d, "not settled after 30 s, ");
          } else {
            fault("UAV could not hold over the UGV (" + fmtd(d) + " m off)");
          }
        }
        break;
      }
      case Phase::LAND: {
        if (land_stage_ == 0) {
          precisionDescent(el);
          break;
        }
        // down on the deck (back at the height it had when carried) and still; PX4 auto-disarms
        // COM_DISARM_LAND (2 s) after touchdown, wait for that before re-attaching
        // (speed threshold above the EKF's velocity noise of a landed vehicle: 0.05 never triggered)
        const bool down = uav_pos_->z < landed_z_ + 0.15 && uav_speed_.value_or(1.0) < 0.15;
        if (down) {
          if (!down_since_) down_since_ = now();
          if ((now() - *down_since_).seconds() > 3.0) {
            down_since_.reset();
            const double d = distXY(*uav_pos_, *ugv_pos_);
            if (d > deck_tol_ || uav_pos_->z < landed_z_ - 0.3) {
              fault("UAV landed next to the UGV (" + fmtd(d) + " m from its centre, z " + fmtd(uav_pos_->z) + ")");
              break;
            }
            // disarm explicitly: PX4's land detector may not trigger on the deck (auto-disarm then never
            // comes), and a UAV carried around armed is a hazard
            callTrigger(cli_uav_disarm_, "uav disarm");
            pub_attach_->publish(std_msgs::msg::Empty());
            enter(Phase::ATTACH, "ATTACH: UAV on the UGV deck (" + fmtd(d * 100) + " cm off centre), disarmed, re-attached");
          }
        } else {
          down_since_.reset();
          if (el > (px4_precland_ ? px4_land_timeout_ : land_timeout_)) {
            fault("UAV did not land (z " + fmtd(uav_pos_->z) + ")");
          } else if (px4_precland_ && (now() - last_log_).seconds() > 3.0) {
            last_log_ = now();
            RCLCPP_INFO(get_logger(), "  LAND (PX4 precland): %.2f m above the deck, %.1f cm off the UGV centre",
                        uav_pos_->z - landed_z_, distXY(*uav_pos_, *ugv_pos_) * 100);
          }
        }
        break;
      }
      case Phase::ATTACH: {
        if (el > 1.0) {
          if (el < 6.0 && (now() - last_log_).seconds() > 2.0) {  // disarm retry (cheap, idempotent)
            last_log_ = now();
            callTrigger(cli_uav_disarm_, "uav disarm (retry)");
          }
          if (el < 3.0) break;
          if (pending_) {  // a new deployment replaced this one
            takePending();
          } else if (target_idx_ + 1 < targets_.size()) {
            ++target_idx_;
            startTarget();
          } else {
            callTrigger(cli_ugv_act_, "ugv activation");
            startRoute(home_);
            enter(Phase::HOME, "HOME: UGV carrying the UAV back home [" + fmt2(home_) + "]");
          }
        }
        break;
      }
      case Phase::HOME: {
        const double d = routeLeft();
        if (routeDone()) {
          char buf[200];
          std::snprintf(buf, sizeof(buf), "DONE: %zu shelf(s), UGV back home [%.1f, %.1f] with the UAV on board (UAV at [%.1f, %.1f])",
                        targets_.size(), ugv_pos_->x, ugv_pos_->y, uav_pos_->x, uav_pos_->y);
          targets_.clear();
          enter(Phase::DONE, buf);
        } else if (el > drive_timeout_) {
          fault("UGV did not get back home in time (" + fmtd(d) + " m left)");
        } else {
          progress("HOME", d);
        }
        break;
      }
      default: break;
    }
  }

  // ---------------------------------------------------------------------------------------------
  // the UAV hovers where its coverage ended: the UGV drives below it
  void startPickup(const std::string& done)
  {
    const Vec3 uav = *uav_pos_;
    if (distXY(uav, *ugv_pos_) < pickup_skip_) {
      enter(Phase::RENDEZVOUS, "RENDEZVOUS: shelf done (" + done + "), the UGV is already below the UAV");
      rdv_stage_ = 0;
      return;
    }
    std::string why;
    const auto  stop = planner_->groundStop(uav.x, uav.y, ugv_pos_->z, ugv_clearance_, why);
    if (!stop) {
      RCLCPP_WARN(get_logger(), "  no UGV stop below the UAV at [%.1f, %.1f] (%s): the UAV flies to the UGV",
                  uav.x, uav.y, why.c_str());
      enter(Phase::RENDEZVOUS, "RENDEZVOUS: shelf done (" + done + "), no UGV stop below the UAV, UAV back to the UGV");
      rdv_stage_ = 0;
      return;
    }
    callTrigger(cli_ugv_act_, "ugv activation");
    startRoute(*stop);
    enter(Phase::PICKUP, "PICKUP: shelf done (" + done + "), UGV driving to the UAV hovering at [" + fmt2(uav) +
                             "], stop [" + fmt2(*stop) + "]");
  }

  // deck centre to land on: the external landing target while it is fresh, else the UGV odometry
  // (nullopt if a landing_target_topic is configured but stale: no descent on old data)
  std::optional<Vec3> landingTarget() const
  {
    if (!sub_target_) return *ugv_pos_;
    if (target_ && (now() - target_time_).seconds() < target_timeout_) return target_;
    return std::nullopt;
  }

  void publishUavRef(const Vec3& xy, double z)
  {
    rbl_msgs::msg::ReferenceStamped ref;
    ref.header.stamp         = now();
    ref.header.frame_id      = "world";
    ref.reference.position.x = xy.x;
    ref.reference.position.y = xy.y;
    ref.reference.position.z = z;
    ref.reference.heading    = hold_yaw_;
    pub_uav_ref_->publish(ref);
  }

  void startPrecisionLanding(double d, const std::string& note)
  {
    land_stage_  = 0;
    pl_z_        = landed_z_ + rdv_height_;
    pl_last_     = now();
    pl_log_xy_   = 0.0;
    enter(Phase::LAND, "LAND: precision landing on the UGV (" + note + fmtd(d * 100) + " cm off its centre)");
  }

  // one 10 Hz step of the tracked descent (see header comment)
  void precisionDescent(double el)
  {
    const rclcpp::Time t  = now();
    const double       dt = std::clamp((t - pl_last_).seconds(), 0.0, 0.5);
    pl_last_              = t;
    const double hover_z  = landed_z_ + rdv_height_;
    const double touch_z  = landed_z_ + pl_touchdown_h_;

    const auto tgt = landingTarget();
    if (tgt) deck_ = *tgt;
    const double e = distXY(*uav_pos_, deck_);
    const char*  mode;
    if (!tgt) {
      mode = "target lost, holding";
    } else if (e < pl_xy_tol_) {
      pl_z_ = std::max(touch_z, pl_z_ - pl_speed_ * dt);
      mode  = "descending";
    } else if (e > pl_abort_xy_) {
      pl_z_ = std::min(hover_z, pl_z_ + pl_speed_ * dt);
      mode  = "off the deck, climbing back";
    } else {
      mode = "re-centring";
    }
    publishUavRef(deck_, pl_z_);
    pl_log_xy_ = std::max(pl_log_xy_, e);

    if (pl_z_ <= touch_z + 1e-3 && uav_pos_->z < touch_z + 0.05 && e < pl_xy_tol_) {
      callTrigger(cli_uav_land_, "uav land");
      land_stage_  = 1;
      phase_start_ = now();
      setState("LAND: " + fmtd(pl_touchdown_h_ * 100) + " cm above the deck, " + fmtd(e * 100) +
               " cm off its centre: touchdown (PX4 Land)");
    } else if (el > pl_timeout_) {
      fault("precision landing did not converge (" + fmtd(e * 100) + " cm off, " +
            fmtd(uav_pos_->z - landed_z_) + " m above the deck)");
    } else if ((t - last_log_).seconds() > 2.0) {
      last_log_ = t;
      RCLCPP_INFO(get_logger(), "  LAND: %s, %.2f m above the deck, %.1f cm off (max %.1f cm in the last 2 s)", mode,
                  uav_pos_->z - landed_z_, e * 100, pl_log_xy_ * 100);
      pl_log_xy_ = 0.0;
    }
  }

  // ---------------------------------------------------------------------------------------------
  void enter(Phase p, const std::string& text)
  {
    phase_       = p;
    phase_start_ = now();
    last_log_    = phase_start_;
    setState(text);
  }

  void fault(const std::string& why)
  {
    if (ugv_pos_) gotoAsync(cli_ugv_goto_, *ugv_pos_);
    const std::string in = toString(phase_);
    phase_               = Phase::IDLE;
    route_.clear();
    pending_.reset();
    const std::string at = targets_.empty() ? "" : ", shelf " + std::to_string(target_idx_ + 1) + "/" +
                                                       std::to_string(targets_.size());
    targets_.clear();
    setState("FAILED (" + in + at + "): " + why + " -- UGV holds, call ~/deploy again");
  }

  void progress(const char* what, double d)
  {
    if ((now() - last_log_).seconds() > 5.0) {
      last_log_ = now();
      RCLCPP_INFO(get_logger(), "  %s: %.1f m to go", what, d);
    }
  }

  // UGV leg to `target`. planner.mode global: planned once on the map, the whole path to
  // rbl_controller/goto_path (route_ = {target}); local, or if the global planner finds no path: the aisle
  // grid route (see header comment), one rbl_controller/goto at a time
  void startRoute(const Vec3& target)
  {
    route_idx_ = 0;
    gpath_.clear();
    if (global_) {
      std::string why;
      if (auto p = gplanner_->plan(*ugv_pos_, target, why)) {
        gpath_ = std::move(*p);
        route_ = {gpath_.back()};  // = target, unless that was not free on the map (snapped)
        sendPath(cli_ugv_goto_path_, gpath_);
        publishUgvPath();
        RCLCPP_INFO(get_logger(), "  UGV global path: %zu points, %.1f m, [%s] -> [%s]", gpath_.size(),
                    rbl_mission::GlobalPlanner2D::length(gpath_), fmt2(*ugv_pos_).c_str(), fmt2(target).c_str());
        return;
      }
      RCLCPP_WARN(get_logger(), "  global planner: %s -- falling back to the aisle-grid route", why.c_str());
    }
    route_ = planner_->groundRoute(*ugv_pos_, target, ugv_clearance_);
    gotoAsync(cli_ugv_goto_, route_.front());
    publishUgvPath();
    std::string r;
    for (const auto& p : route_) r += " [" + fmt2(p) + "]";
    RCLCPP_INFO(get_logger(), "  UGV route:%s", r.c_str());
  }
  // advances through the intermediate points; true once the UGV is stopped at the last one
  bool routeDone()
  {
    if (route_.empty()) return false;
    if (route_idx_ + 1 < route_.size()) {
      if (distXY(*ugv_pos_, route_[route_idx_]) < route_pass_tol_) {
        ++route_idx_;
        gotoAsync(cli_ugv_goto_, route_[route_idx_]);
      }
      return false;
    }
    return distXY(*ugv_pos_, route_.back()) < ugv_tol_ && ugv_speed_.value_or(1.0) < ugv_stopped_speed_;
  }
  double routeLeft() const
  {
    if (!gpath_.empty()) {  // along the global path, from its point nearest to the UGV
      size_t k = 0;
      for (size_t i = 1; i < gpath_.size(); ++i) {
        if (distXY(*ugv_pos_, gpath_[i]) < distXY(*ugv_pos_, gpath_[k])) k = i;
      }
      double d = distXY(*ugv_pos_, gpath_[k]);
      for (size_t i = k + 1; i < gpath_.size(); ++i) d += distXY(gpath_[i - 1], gpath_[i]);
      return d;
    }
    if (route_.empty()) return 0.0;
    double d = distXY(*ugv_pos_, route_[route_idx_]);
    for (size_t i = route_idx_ + 1; i < route_.size(); ++i) d += distXY(route_[i - 1], route_[i]);
    return d;
  }

  // rbl_controller/goto_path with `path` (in the control frame "world")
  void sendPath(const rclcpp::Client<rbl_msgs::srv::GotoPath>::SharedPtr& cli, const std::vector<Vec3>& path)
  {
    if (!cli->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "%s not available", cli->get_service_name());
      return;
    }
    auto r = std::make_shared<rbl_msgs::srv::GotoPath::Request>();
    r->path = toPathMsg(path);
    cli->async_send_request(r, [this, name = std::string(cli->get_service_name())](
                                   rclcpp::Client<rbl_msgs::srv::GotoPath>::SharedFuture f) {
      const auto res = f.get();
      if (!res->success) RCLCPP_ERROR(get_logger(), "%s rejected the path: %s", name.c_str(), res->message.c_str());
    });
  }

  // the UAV to p: a straight path in global mode (short legs right above the UGV), else ~/goto
  void gotoUav(const Vec3& p)
  {
    if (global_ && uav_pos_) {
      sendPath(cli_uav_goto_path_, {*uav_pos_, p});
    } else {
      gotoAsync(cli_uav_goto_, p);
    }
  }

  nav_msgs::msg::Path toPathMsg(const std::vector<Vec3>& path) const
  {
    nav_msgs::msg::Path m;
    m.header.frame_id = "world";
    m.header.stamp    = now();
    for (const auto& q : path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header             = m.header;
      ps.pose.position.x    = q.x;
      ps.pose.position.y    = q.y;
      ps.pose.position.z    = q.z;
      ps.pose.orientation.w = 1.0;
      m.poses.push_back(ps);
    }
    return m;
  }

  void publishUgvPath()
  {
    std::vector<Vec3> p = gpath_;
    if (p.empty() && !route_.empty()) {
      p.push_back(*ugv_pos_);
      p.insert(p.end(), route_.begin(), route_.end());
    }
    pub_ugv_path_->publish(toPathMsg(p));
  }

  void gotoAsync(const rclcpp::Client<rbl_msgs::srv::Vec4>::SharedPtr& cli, const Vec3& p)
  {
    if (!cli->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "%s not available", cli->get_service_name());
      return;
    }
    auto r  = std::make_shared<rbl_msgs::srv::Vec4::Request>();
    r->goal = {p.x, p.y, p.z, 0.0};
    cli->async_send_request(r);
  }

  void callTrigger(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& cli, const std::string& what)
  {
    if (!cli->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "%s: %s not available", what.c_str(), cli->get_service_name());
      return;
    }
    cli->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
                            [this, what](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
                              const auto r = f.get();
                              RCLCPP_INFO(get_logger(), "  %s: %s", what.c_str(), r->message.c_str());
                            });
  }

  void setState(const std::string& text)
  {
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
    std_msgs::msg::String m;
    m.data = text;
    pub_state_->publish(m);
  }

  static std::string fmt2(const Vec3& p)
  {
    char b[64];
    std::snprintf(b, sizeof(b), "%.1f, %.1f", p.x, p.y);
    return b;
  }
  static std::string fmtd(double d)
  {
    char b[32];
    std::snprintf(b, sizeof(b), "%.1f", d);
    return b;
  }

  // parameters
  std::unique_ptr<rbl_mission::CoveragePlanner> planner_;
  std::string ugv_, uav_;
  double      ugv_clearance_, ugv_tol_, ugv_stopped_speed_, takeoff_alt_;
  double      drive_timeout_, takeoff_timeout_;
  double      rdv_height_, rdv_tol_, rdv_speed_, rdv_timeout_, deck_tol_, land_timeout_, route_pass_tol_;
  double      handover_dist_, pickup_skip_;
  double      pl_speed_, pl_xy_tol_, pl_abort_xy_, pl_touchdown_h_, pl_timeout_, target_timeout_;
  bool        px4_precland_{true};
  bool        global_{true};
  std::unique_ptr<rbl_mission::GlobalPlanner2D> gplanner_;
  double      px4_land_timeout_{120.0};

  // state
  std::mutex                     mtx_;
  std::optional<Vec3>            ugv_pos_, uav_pos_;
  std::optional<double>          ugv_speed_, uav_speed_;
  std::string                    inspection_state_;
  Phase                          phase_{Phase::IDLE};
  rclcpp::Time                   phase_start_, last_log_;
  rbl_mission::srv::Deploy::Request req_;
  Vec3                           home_, stop_, launch_, deck_;
  double                         landed_z_{0.0};

  int                            rdv_stage_{0};  // 0 wait for the UGV, 1 rbl transit, 2 precise hold
  int                            land_stage_{0}; // 0 tracked descent (this node), 1 PX4 Land / precland touchdown
  double                         pl_z_{0.0}, pl_log_xy_{0.0};
  rclcpp::Time                   pl_last_, target_time_;
  std::optional<Vec3>            target_;
  double                         ugv_yaw_{0.0}, uav_yaw_{0.0}, hold_yaw_{0.0};
  std::vector<Vec3>              route_;
  std::vector<Vec3>              gpath_;  // global path of the current UGV leg (planner.mode global)
  std::vector<Target>            targets_;  // the shelves of this deployment, in order
  std::optional<std::vector<Target>> pending_;  // a new deployment waiting for the UAV to be back on board
  size_t                         target_idx_{0};
  size_t                         route_idx_{0};
  std::optional<rclcpp::Time>    down_since_;
  std::string                    plan_info_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr        pub_state_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr         pub_detach_, pub_attach_;
  rclcpp::Publisher<rbl_msgs::msg::ReferenceStamped>::SharedPtr pub_uav_ref_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr          pub_ugv_path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr   sub_ugv_odom_, sub_uav_odom_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr     sub_uav_state_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_target_;
  rclcpp::Client<rbl_msgs::srv::Vec4>::SharedPtr             cli_ugv_goto_, cli_uav_inspect_, cli_uav_goto_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr          cli_ugv_act_, cli_uav_arm_, cli_uav_land_, cli_uav_abort_,
      cli_uav_precland_, cli_uav_deact_, cli_ugv_deact_, cli_uav_act_, cli_uav_disarm_;
  rclcpp::Client<rbl_msgs::srv::GotoPath>::SharedPtr         cli_ugv_goto_path_, cli_uav_goto_path_;
  rclcpp::Service<rbl_mission::srv::Deploy>::SharedPtr       srv_deploy_;
  rclcpp::Service<rbl_mission::srv::DeploySequence>::SharedPtr srv_deploy_seq_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr         srv_abort_;
  rclcpp::TimerBase::SharedPtr                               timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UgvUavMission>();
  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(node);
  ex.spin();
  rclcpp::shutdown();
  return 0;
}
