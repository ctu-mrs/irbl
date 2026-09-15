// Feeds each rbl_controller_node instance the poses/velocities of the *other* UAVs in the group,
// so RBLController's collision-avoidance logic (group_odoms.enable/add_to_pcl) has something to
// avoid. Replaces the MRS multirotor_simulator's combined /multirotor_simulator/uav_poses topic:
// here, ground-truth comes straight from each vehicle's own rbl_px4_bridge odometry output.
//
// Publishes filter_reflective_uavs/PoseVelocityArray (the same message rbl_controller_node
// already subscribes to on "~/group_states_in") to every UAV containing all *other* UAVs' state.

#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <filter_reflective_uavs/msg/pose_velocity_array.hpp>

namespace rbl_px4_bridge
{

class GroupStateRelayNode : public rclcpp::Node
{
public:
  GroupStateRelayNode() : Node("group_state_relay")
  {
    uav_names_    = declare_parameter<std::vector<std::string>>("uav_names", { "uav1", "uav2" });
    control_frame_ = declare_parameter<std::string>("control_frame", "uav1/world_origin");
    publish_rate_ = declare_parameter<double>("publish_rate", 20.0);

    for (const auto& name : uav_names_) {
      odom_sub_.push_back(create_subscription<nav_msgs::msg::Odometry>(
          "/" + name + "/odometry", rclcpp::QoS(5),
          [this, name](const nav_msgs::msg::Odometry::SharedPtr msg) { last_odom_[name] = msg; }));

      pose_vel_pub_[name] = create_publisher<filter_reflective_uavs::msg::PoseVelocityArray>(
          "/" + name + "/filter_reflective_uavs/pose_vel", rclcpp::QoS(5));
    }

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / publish_rate_),
                                std::bind(&GroupStateRelayNode::timerCallback, this));

    RCLCPP_INFO(get_logger(), "group_state_relay started for %zu UAVs", uav_names_.size());
  }

private:
  void timerCallback()
  {
    for (const auto& target : uav_names_) {
      filter_reflective_uavs::msg::PoseVelocityArray msg;
      msg.header.stamp    = now();
      msg.header.frame_id = control_frame_;

      int32_t id = 0;
      for (const auto& other : uav_names_) {
        if (other == target) {
          continue;
        }

        const auto it = last_odom_.find(other);
        if (it == last_odom_.end() || !it->second) {
          continue;
        }

        msg.ids.push_back(id++);
        msg.poses.push_back(it->second->pose.pose);
        msg.velocities.push_back(it->second->twist.twist.linear);
      }

      pose_vel_pub_.at(target)->publish(msg);
    }
  }

  std::vector<std::string> uav_names_;
  std::string              control_frame_;
  double                   publish_rate_;

  std::map<std::string, nav_msgs::msg::Odometry::SharedPtr>                              last_odom_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>                   odom_sub_;
  std::map<std::string, rclcpp::Publisher<filter_reflective_uavs::msg::PoseVelocityArray>::SharedPtr> pose_vel_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rbl_px4_bridge

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rbl_px4_bridge::GroupStateRelayNode>());
  rclcpp::shutdown();
  return 0;
}
