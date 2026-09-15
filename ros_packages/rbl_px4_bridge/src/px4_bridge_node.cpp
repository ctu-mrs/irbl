// Bridges the irbl rbl_controller_node to native PX4 offboard control (uXRCE-DDS / px4_msgs)
//
//   PX4 vehicle_odometry (NED/FRD) --> nav_msgs/Odometry (ENU/FLU)  --> rbl_controller "~/odom_in"
//   rbl_controller "~/ref_out" (rbl_msgs/ReferenceStamped, ENU)     --> PX4 trajectory_setpoint (NED)
//
// Streams OffboardControlMode + TrajectorySetpoint continuously (PX4 requires this before and
// during OFFBOARD mode) and, after a short warm-up, auto-arms and switches PX4 into OFFBOARD mode --
// this mirrors PX4's official ROS 2 offboard_control.cpp example.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <px4_ros_com/frame_transforms.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rbl_msgs/msg/reference_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>

using namespace std::chrono_literals;

namespace rbl_px4_bridge
{

class Px4BridgeNode : public rclcpp::Node
{
public:
  Px4BridgeNode() : Node("px4_bridge")
  {
    uav_name_         = declare_parameter<std::string>("uav_name", "uav1");
    control_frame_    = declare_parameter<std::string>("control_frame", uav_name_ + "/world_origin");
    takeoff_altitude_ = declare_parameter<double>("takeoff_altitude", 2.0);
    publish_rate_     = declare_parameter<double>("publish_rate", 20.0);
    // PX4's OFFBOARD TrajectorySetpoint has no internal smoothing (unlike Auto/mission waypoints,
    // it's fed straight into the position PID -- see MulticopterPositionControl.cpp), so a
    // discontinuous jump in the reference this node receives (a replan, a new goal) would
    // otherwise become an equally discontinuous jump in what PX4 tries to instantly chase. Instead
    // of forwarding rbl_controller's reference directly, this caps how fast the setpoint actually
    // sent to PX4 is allowed to move towards it -- smooth motion passes through close to 1:1 (as
    // long as its own implied speed stays under this), while a jump turns into a straight-line
    // approach at this speed instead of a teleport.
    max_setpoint_speed_ = declare_parameter<double>("max_setpoint_speed", 3.0);
    auto_arm_         = declare_parameter<bool>("auto_arm", true);
    arm_after_cycles_ = declare_parameter<int>("arm_after_cycles", 20);
    // PX4 multi-instance SITL namespaces every /fmu/... topic as /<px4_ns>/fmu/... for every
    // instance except instance 0 (whose default namespace is empty). Set this to match whatever
    // PX4_UXRCE_DDS_NS (or the "px4_<instance>" default) that vehicle's PX4 was started with.
    px4_ns_ = declare_parameter<std::string>("px4_ns", "");
    // rcS auto-sets each instance's own MAV_SYS_ID to px4_instance+1 (1 for instance 0, 2 for
    // instance 1, ...). PX4's commander drops any vehicle_command whose target_system doesn't
    // match its own MAV_SYS_ID -- topic namespacing alone does *not* protect against this, so for
    // any instance other than 0 this must be set to that instance's MAV_SYS_ID, or arming/mode
    // commands are silently ignored (the vehicle never actually arms even though this node logs
    // that it sent the command).
    target_system_ = declare_parameter<int>("target_system", 1);
    // Every PX4 instance's own vehicle_odometry is relative to *its own* EKF local origin, i.e.
    // its spawn point (PX4_GZ_MODEL_POSE), not to the shared control_frame's origin. For a
    // multi-UAV sim where every vehicle publishes into the same control_frame (so they can see
    // and avoid each other), this vehicle's spawn offset -- the same x/y/z given to
    // PX4_GZ_MODEL_POSE at launch -- must be added back in, or every vehicle appears to be
    // sitting on top of the others. Left at (0,0,0) for a single-UAV sim / this vehicle's origin.
    home_offset_.x() = declare_parameter<double>("home_offset_x", 0.0);
    home_offset_.y() = declare_parameter<double>("home_offset_y", 0.0);
    home_offset_.z() = declare_parameter<double>("home_offset_z", 0.0);

    const rclcpp::QoS px4_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();

    const std::string fmu_prefix = px4_ns_.empty() ? "" : ("/" + px4_ns_);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", rclcpp::QoS(5));
    // Broadcasts the same control_frame -> "<uav_name>/fcu" transform as the Odometry message
    // above, as live TF -- a static sensor extrinsic (e.g. published from sensor_config.yaml)
    // can then be chained onto "<uav_name>/fcu" so tf2 can transform sensor data (a real lidar's
    // point cloud, mounted rigidly on the vehicle) into control_frame. Nothing else in this stack
    // published this before, since the only prior pcl_topic source (map_generator) was already in
    // control_frame and never needed the lookup.
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
        fmu_prefix + "/fmu/in/offboard_control_mode", px4_qos);
    trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        fmu_prefix + "/fmu/in/trajectory_setpoint", px4_qos);
    vehicle_command_pub_ =
        create_publisher<px4_msgs::msg::VehicleCommand>(fmu_prefix + "/fmu/in/vehicle_command", px4_qos);

    vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        fmu_prefix + "/fmu/out/vehicle_odometry", px4_qos,
        std::bind(&Px4BridgeNode::odomCallback, this, std::placeholders::_1));

    reference_sub_ = create_subscription<rbl_msgs::msg::ReferenceStamped>(
        "reference", rclcpp::QoS(1), std::bind(&Px4BridgeNode::referenceCallback, this, std::placeholders::_1));

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_),
                                std::bind(&Px4BridgeNode::timerCallback, this));

    RCLCPP_INFO(get_logger(),
                "px4_bridge started: control_frame='%s', takeoff_altitude=%.2fm, auto_arm=%s, px4_ns='%s', "
                "target_system=%d",
                control_frame_.c_str(), takeoff_altitude_, auto_arm_ ? "true" : "false", px4_ns_.c_str(),
                target_system_);
  }

private:
  // | ------------------------- PX4 -> ROS ------------------------ |

  void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    // Single-UAV bridge: assume the common SITL/EKF2 default of NED pose/velocity frames.
    const Eigen::Vector3d pos_ned(msg->position[0], msg->position[1], msg->position[2]);
    const Eigen::Vector3d vel_ned(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    const Eigen::Quaterniond q_px4(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);  // w, x, y, z

    const Eigen::Vector3d    pos_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(pos_ned);
    const Eigen::Vector3d    vel_enu = px4_ros_com::frame_transforms::ned_to_enu_local_frame(vel_ned);
    const Eigen::Quaterniond q_enu   = px4_ros_com::frame_transforms::px4_to_ros_orientation(q_px4);

    const Eigen::Vector3d pos_enu_shared = pos_enu + home_offset_;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = now();
    odom.header.frame_id = control_frame_;
    odom.child_frame_id  = uav_name_ + "/fcu";

    odom.pose.pose.position.x = pos_enu_shared.x();
    odom.pose.pose.position.y = pos_enu_shared.y();
    odom.pose.pose.position.z = pos_enu_shared.z();

    odom.pose.pose.orientation.w = q_enu.w();
    odom.pose.pose.orientation.x = q_enu.x();
    odom.pose.pose.orientation.y = q_enu.y();
    odom.pose.pose.orientation.z = q_enu.z();

    // rbl_controller expects twist.linear expressed in the same (world) frame as the pose,
    // not in the body frame -- matches the world-frame odometry convention it was written against.
    odom.twist.twist.linear.x = vel_enu.x();
    odom.twist.twist.linear.y = vel_enu.y();
    odom.twist.twist.linear.z = vel_enu.z();

    odom.twist.twist.angular.x = msg->angular_velocity[0];
    odom.twist.twist.angular.y = -msg->angular_velocity[1];
    odom.twist.twist.angular.z = -msg->angular_velocity[2];

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header             = odom.header;
    tf_msg.child_frame_id     = odom.child_frame_id;
    tf_msg.transform.translation.x = odom.pose.pose.position.x;
    tf_msg.transform.translation.y = odom.pose.pose.position.y;
    tf_msg.transform.translation.z = odom.pose.pose.position.z;
    tf_msg.transform.rotation      = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);

    have_odom_ = true;
    last_position_enu_ = pos_enu;
  }

  // | ------------------------- ROS -> PX4 ------------------------ |

  void referenceCallback(const rbl_msgs::msg::ReferenceStamped::SharedPtr msg)
  {
    // Reference is expressed in the shared control_frame; PX4 needs it back in this vehicle's
    // own local (spawn-relative) frame, so undo the offset applied in odomCallback().
    const Eigen::Vector3d pos_enu(msg->reference.position.x, msg->reference.position.y, msg->reference.position.z);
    const Eigen::Vector3d pos_enu_local = pos_enu - home_offset_;
    const Eigen::Vector3d pos_ned       = px4_ros_com::frame_transforms::enu_to_ned_local_frame(pos_enu_local);

    const Eigen::Quaterniond q_enu(Eigen::AngleAxisd(msg->reference.heading, Eigen::Vector3d::UnitZ()));
    const Eigen::Quaterniond q_px4 = px4_ros_com::frame_transforms::ros_to_px4_orientation(q_enu);
    const double              yaw_ned = px4_ros_com::frame_transforms::utils::quaternion::quaternion_get_yaw(q_px4);

    latest_setpoint_ned_ = pos_ned;
    latest_yaw_ned_      = yaw_ned;
    have_reference_      = true;
  }

  // | --------------------------- timer ---------------------------- |

  void timerCallback()
  {
    publishOffboardControlMode();
    publishTrajectorySetpoint();

    // Do not even start counting towards arming until PX4 is actually streaming odometry
    // (i.e. the EKF has a valid local position estimate) -- arming/OFFBOARD would be rejected
    // otherwise, and counting down regardless would race PX4's own startup.
    if (auto_arm_ && !armed_and_offboard_ && have_odom_) {
      if (setpoint_count_ == arm_after_cycles_) {
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
        RCLCPP_INFO(get_logger(), "Requesting OFFBOARD mode");
      }
      if (setpoint_count_ == arm_after_cycles_ + 2) {
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(get_logger(), "Arming");
        armed_and_offboard_ = true;
      }
      if (setpoint_count_ < arm_after_cycles_ + 3) {
        ++setpoint_count_;
      }
    }
  }

  void publishOffboardControlMode()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position     = true;
    msg.velocity     = false;
    msg.acceleration = false;
    msg.attitude     = false;
    msg.body_rate    = false;
    msg.timestamp    = now().nanoseconds() / 1000;
    offboard_control_mode_pub_->publish(msg);
  }

  void publishTrajectorySetpoint()
  {
    px4_msgs::msg::TrajectorySetpoint msg{};

    if (have_reference_) {
      const rclcpp::Time stamp = now();
      if (!have_commanded_position_) {
        // First reference ever received: nothing to ramp from yet, jump straight to it.
        commanded_position_ned_ = latest_setpoint_ned_;
        have_commanded_position_ = true;
      }
      else {
        const double dt = (stamp - prev_command_stamp_).seconds();
        const Eigen::Vector3d delta    = latest_setpoint_ned_ - commanded_position_ned_;
        const double           dist    = delta.norm();
        const double           max_step = max_setpoint_speed_ * std::max(dt, 0.0);
        if (dist > max_step && dist > 1e-6) {
          commanded_position_ned_ += delta * (max_step / dist);
        }
        else {
          commanded_position_ned_ = latest_setpoint_ned_;
        }
      }
      prev_command_stamp_ = stamp;

      msg.position[0] = static_cast<float>(commanded_position_ned_.x());
      msg.position[1] = static_cast<float>(commanded_position_ned_.y());
      msg.position[2] = static_cast<float>(commanded_position_ned_.z());
      msg.yaw         = static_cast<float>(latest_yaw_ned_);
    }
    else {
      // pre-activation: climb straight up to the configured hover altitude and hold, above the
      // local-origin spawn point, until rbl_controller starts publishing real references.
      msg.position[0] = 0.0f;
      msg.position[1] = 0.0f;
      msg.position[2] = static_cast<float>(-takeoff_altitude_);
      msg.yaw         = 0.0f;
    }

    msg.timestamp = now().nanoseconds() / 1000;
    trajectory_setpoint_pub_->publish(msg);
  }

  void publishVehicleCommand(uint32_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1            = param1;
    msg.param2            = param2;
    msg.command           = command;
    msg.target_system     = static_cast<uint8_t>(target_system_);
    msg.target_component  = 1;
    msg.source_system     = 1;
    msg.source_component  = 1;
    msg.from_external     = true;
    msg.timestamp         = now().nanoseconds() / 1000;
    vehicle_command_pub_->publish(msg);
  }

  // | -------------------------- params -------------------------- |
  std::string     uav_name_;
  std::string     control_frame_;
  std::string     px4_ns_;
  int             target_system_;
  Eigen::Vector3d home_offset_ = Eigen::Vector3d::Zero();
  double          takeoff_altitude_;
  double          publish_rate_;
  double          max_setpoint_speed_;
  bool            auto_arm_;
  int             arm_after_cycles_;

  // | -------------------------- state ----------------------------- |
  bool            have_odom_          = false;
  bool            have_reference_     = false;
  bool            armed_and_offboard_ = false;
  int             setpoint_count_     = 0;
  Eigen::Vector3d last_position_enu_  = Eigen::Vector3d::Zero();
  Eigen::Vector3d latest_setpoint_ned_ = Eigen::Vector3d::Zero();
  double          latest_yaw_ned_      = 0.0;
  // The setpoint actually sent to PX4 -- rate-limited towards latest_setpoint_ned_, see
  // publishTrajectorySetpoint(). Deliberately separate from latest_setpoint_ned_ (the raw,
  // possibly-discontinuous reference as received).
  Eigen::Vector3d commanded_position_ned_   = Eigen::Vector3d::Zero();
  bool            have_commanded_position_ = false;
  rclcpp::Time    prev_command_stamp_;

  // | ------------------------ ROS interfaces ------------------------ |
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr             odom_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr  offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr   trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr       vehicle_command_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr   vehicle_odometry_sub_;
  rclcpp::Subscription<rbl_msgs::msg::ReferenceStamped>::SharedPtr  reference_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace rbl_px4_bridge

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rbl_px4_bridge::Px4BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
