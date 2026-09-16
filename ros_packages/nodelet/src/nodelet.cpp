// CUSTOM
#include <filter_reflective_uavs/msg/pose_velocity_array.hpp>
#include "rbl_controller_core/rbl_controller.h"
#include "local_static_map.h"

// Local message/service definitions (replacing mrs_msgs -- no MRS dependency of any kind)
#include <octomap_msgs/conversions.h>
#include <rbl_msgs/msg/float64_stamped.hpp>
#include <rbl_msgs/msg/reference.hpp>
#include <rbl_msgs/msg/reference_stamped.hpp>
#include <rbl_msgs/srv/float64_srv.hpp>
#include <rbl_msgs/srv/vec4.hpp>
#include <mutex>
#include <octomap_msgs/msg/octomap.hpp>

// MSGS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

// TF2 (replaces mrs_lib::Transformer)
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2/exceptions.h>

// OCTOMAP
#include <octomap/OcTree.h>

// EIGEN
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>


// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Standard CPP libs
#include <cmath>
#include <memory>
#include <string>
#include <optional>

namespace rbl_controller
{

  // | ------- minimal local replacements for mrs_lib helpers ------- |
  // A tiny "new message available" subscriber wrapper, replacing mrs_lib::SubscriberHandler.
  template <typename T>
  class SimpleSub
  {
  public:
    void subscribe(rclcpp::Node* node, const std::string& topic, const rclcpp::QoS& qos,
                   const rclcpp::CallbackGroup::SharedPtr& group)
    {
      rclcpp::SubscriptionOptions opts;
      opts.callback_group = group;
      sub_ = node->create_subscription<T>(
          topic, qos, [this](const typename T::SharedPtr msg) {
            msg_     = msg;
            has_new_ = true;
          },
          opts);
    }

    bool newMsg()
    {
      return has_new_;
    }

    typename T::SharedPtr getMsg()
    {
      has_new_ = false;
      return msg_;
    }

  private:
    typename rclcpp::Subscription<T>::SharedPtr sub_;
    typename T::SharedPtr                       msg_;
    bool                                         has_new_ = false;
  };

  // A tiny helper replacing mrs_lib::ParamLoader: declares (if not already declared) and reads a parameter.
  template <typename T>
  T getParam(rclcpp::Node* node, const std::string& name, const T& default_value)
  {
    if (!node->has_parameter(name)) {
      node->declare_parameter<T>(name, default_value);
    }
    T value{};
    node->get_parameter(name, value);
    return value;
  }

  class WrapperRosRBL : public rclcpp::Node
  {
  public:
    WrapperRosRBL(rclcpp::NodeOptions options);

    void initialize();

  private:
    rclcpp::Node::SharedPtr  node_;
    rclcpp::Clock::SharedPtr clock_;

    rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
    rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
    rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;


    std::vector<State>                               group_states_;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> last_obstacle_cloud_;
    bool                                             pcl_loaded_ = false;

    // Bounded local map of static obstacles, merged into the live cloud before it ever reaches
    // rbl_controller_ -- see cbTmSetRef()'s pcl branch. Feeds both the reactive CIRI partition and
    // (via RBLController::setPCL()'s cloud_, which the replanner reads too) the replanner's own
    // grid, from this one merge point.
    std::unique_ptr<LocalStaticMap> static_map_;
    Eigen::Vector3d                 agent_pos_world_     = Eigen::Vector3d::Zero();
    bool                             have_agent_pos_world_ = false;

    bool is_initialized_ = false;
    bool is_activated_   = false;

    bool        _group_odoms_enabled_ = false;
    bool        _add_agents_to_pcl_   = false;
    int         _group_odoms_size_    = 0;
    std::string _agent_name_;
    std::string _control_frame_;
    bool        _is_simulated_ = false;

    std::mutex                     mtx_rbl_;
    std::shared_ptr<RBLController> rbl_controller_;
    RBLParams                      rbl_params_;

    // | ----------------- sevice server callbacks ---------------- |

    bool cbSrvActivateControl(const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
                              const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_activate_control_;

    bool cbSrvDeactivateControl(const std::shared_ptr<std_srvs::srv::Trigger::Request>  req,
                                const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_deactivate_control_;

    bool cbSrvGotoPosition(const std::shared_ptr<rbl_msgs::srv::Vec4::Request>  req,
                           const std::shared_ptr<rbl_msgs::srv::Vec4::Response> res);
    rclcpp::Service<rbl_msgs::srv::Vec4>::SharedPtr srv_goto_position_;

    bool cbSrvSetBetaD(const std::shared_ptr<rbl_msgs::srv::Float64Srv::Request>  req,
                       const std::shared_ptr<rbl_msgs::srv::Float64Srv::Response> res);
    rclcpp::Service<rbl_msgs::srv::Float64Srv>::SharedPtr srv_set_betaD_;

    // | --------------------- reference output --------------------- |
    // Replaces the old mrs_msgs::srv::ReferenceStampedSrv call into the MRS control manager.
    // A PX4 offboard bridge node subscribes to this topic and forwards it to the autopilot.
    rclcpp::Publisher<rbl_msgs::msg::ReferenceStamped>::SharedPtr pub_reference_out_;

    // | --------------------- timer callbacks -------------------- |

    void                       cbTmSetRef();
    rclcpp::TimerBase::SharedPtr tm_set_ref_;

    void                       cbTmDiagnostics();
    rclcpp::TimerBase::SharedPtr tm_diagnostics_;

    // | ----------------------- publishers ----------------------- |

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_viz_position_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_viz_centroid_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_viz_seed_B_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_viz_target_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_viz_waypoint_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   pub_viz_cell_A_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   pub_viz_cell_A_sensed_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   pub_viz_inflated_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   pub_viz_cloud;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             pub_viz_path_;

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud_proc_;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud_raw_;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud_merged_;

    std::shared_ptr<sensor_msgs::msg::PointCloud2> getVizCellA(const std::vector<Eigen::Vector3d>& points,
                                                               const std::string&                  frame);
    std::shared_ptr<sensor_msgs::msg::PointCloud2> getVizInflatedMap(const std::vector<Eigen::Vector3d>& points,
                                                                     const std::string&                  frame);
    std::shared_ptr<sensor_msgs::msg::PointCloud2>
    getVizPCL(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& pcl,
              const std::string&                                      frame);
    std::shared_ptr<nav_msgs::msg::Path> getVizPath(const std::vector<Eigen::Vector3d>& path,
                                                    const std::string&                  frame);
    visualization_msgs::msg::Marker getVizPosition(const Eigen::Vector3d& point,
                                                   const double           scale,
                                                   const std::string&     frame);
    visualization_msgs::msg::Marker getVizCentroid(const Eigen::Vector3d& point,
                                                   const std::string&     frame);
    visualization_msgs::msg::Marker getVizModGroupGoal(const Eigen::Vector3d& point,
                                                       const double           scale,
                                                       const std::string&     frame);
    visualization_msgs::msg::Marker getVizWaypoint(const Eigen::Vector3d& point,
                                                   const double           scale,
                                                   const std::string&     frame);


    // | ----------------------- subscribers ---------------------- |
    bool                                                       octomap_msg_;
    SimpleSub<nav_msgs::msg::Odometry>                        sh_odom_;
    SimpleSub<rbl_msgs::msg::Float64Stamped>                  sh_alt_;
    SimpleSub<sensor_msgs::msg::PointCloud2>                  sh_pcl_;
    SimpleSub<octomap_msgs::msg::Octomap>                     sh_octomap_;
    SimpleSub<filter_reflective_uavs::msg::PoseVelocityArray> sh_group_states_;
    SimpleSub<geometry_msgs::msg::PoseArray>                  sh_sim_group_poses_;

    void updateGroupStates(const filter_reflective_uavs::msg::PoseVelocityArray::ConstSharedPtr& msg);
    void updateGroupStates(const geometry_msgs::msg::PoseArray::ConstSharedPtr& msg);

    // | --------------------- tf2 (replaces mrs_lib::Transformer) --------------------- |
    std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::optional<geometry_msgs::msg::PointStamped> transformPoint(const geometry_msgs::msg::PointStamped& in,
                                                                    const std::string& target_frame);
    std::optional<geometry_msgs::msg::Vector3Stamped> transformVector(const geometry_msgs::msg::Vector3Stamped& in,
                                                                       const std::string& target_frame);
    std::optional<sensor_msgs::msg::PointCloud2> transformCloud(const sensor_msgs::msg::PointCloud2& in,
                                                                  const std::string& target_frame);

    Eigen::Vector3d           pointToEigen(const geometry_msgs::msg::Point& point);
    Eigen::Vector3d           vectorToEigen(const geometry_msgs::msg::Vector3& vec);
    geometry_msgs::msg::Point pointFromEigen(const Eigen::Vector3d& vec);
    geometry_msgs::msg::Point createPoint(double x,
                                          double y,
                                          double z);
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>
    addAgents2PCL(std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud,
                  const std::vector<State>&                         group_states,
                  const double                                      voxel_size,
                  const double                                      encumbrance);
  };  // //}


  WrapperRosRBL::WrapperRosRBL(rclcpp::NodeOptions options)
    : Node("wrapper_ros_rbl",
           options)
  {
    initialize();
  }


  void WrapperRosRBL::initialize()  // //{
  {
    // non-owning shared_ptr aliasing `this`, so the rest of the class can keep using node_-> everywhere
    node_  = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {});
    clock_ = node_->get_clock();

    cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cbkgrp_ss_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    std::string odom_topic_name;
    double      rate_tm_set_ref;
    double      rate_tm_diagnostics;

    _agent_name_   = getParam<std::string>(node_.get(), "uav_name", "uav1");
    _is_simulated_ = getParam<bool>(node_.get(), "use_sim_time", false);
    _control_frame_ = getParam<std::string>(node_.get(), "control_frame", "uav1/world_origin");
    _group_odoms_enabled_ = getParam<bool>(node_.get(), "group_odoms.enable", false);
    _add_agents_to_pcl_   = getParam<bool>(node_.get(), "group_odoms.add_to_pcl", false);
    _group_odoms_size_    = getParam<int>(node_.get(), "group_odoms.size", 0);

    odom_topic_name     = getParam<std::string>(node_.get(), "odometry_topic", "odom");
    rate_tm_set_ref      = getParam<double>(node_.get(), "rate.timer_set_ref", 20.0);
    rate_tm_diagnostics  = getParam<double>(node_.get(), "rate.timer_diagnostics", 10.0);

    rbl_params_.only_2d      = getParam<bool>(node_.get(), "rbl_controller.only_2d", false);
    rbl_params_.z_min        = getParam<double>(node_.get(), "rbl_controller.z_min", 0.0);
    rbl_params_.z_max        = getParam<double>(node_.get(), "rbl_controller.z_max", 4.0);
    rbl_params_.z_ref        = getParam<double>(node_.get(), "rbl_controller.z_ref", 1.0);
    rbl_params_.d1           = getParam<double>(node_.get(), "rbl_controller.d1", 1.0);
    rbl_params_.d2           = getParam<double>(node_.get(), "rbl_controller.d2", 1.0);
    rbl_params_.d3           = getParam<double>(node_.get(), "rbl_controller.d3", 1.0);
    rbl_params_.d4           = getParam<double>(node_.get(), "rbl_controller.d4", 1.0);
    rbl_params_.d5           = getParam<double>(node_.get(), "rbl_controller.d5", 1.0);
    rbl_params_.d6           = getParam<double>(node_.get(), "rbl_controller.d6", 1.0);
    rbl_params_.d7           = getParam<double>(node_.get(), "rbl_controller.d7", 1.0);
    rbl_params_.radius       = getParam<double>(node_.get(), "rbl_controller.radius", 5.0);
    rbl_params_.encumbrance  = getParam<double>(node_.get(), "rbl_controller.encumbrance", 0.5);
    rbl_params_.step_size    = getParam<double>(node_.get(), "rbl_controller.step_size", 0.2);
    rbl_params_.betaD        = getParam<double>(node_.get(), "rbl_controller.betaD", 0.3);
    rbl_params_.beta_min     = getParam<double>(node_.get(), "rbl_controller.beta_min", 0.1);
    rbl_params_.use_z_rule   = getParam<bool>(node_.get(), "rbl_controller.use_z_rule", false);
    rbl_params_.dt           = getParam<double>(node_.get(), "rbl_controller.dt", 0.1);
    rbl_params_.cwvd_rob     = getParam<double>(node_.get(), "rbl_controller.cwvd_rob", 0.5);
    rbl_params_.cwvd_obs     = getParam<double>(node_.get(), "rbl_controller.cwvd_obs", 0.5);
    rbl_params_.use_garmin_alt = getParam<bool>(node_.get(), "rbl_controller.use_garmin_alt", false);
    rbl_params_.replanner    = getParam<bool>(node_.get(), "rbl_controller.replanner", true);
    rbl_params_.limited_fov  = getParam<bool>(node_.get(), "rbl_controller.limited_fov", true);
    rbl_params_.use_map      = getParam<bool>(node_.get(), "rbl_controller.use_map", true);
    rbl_params_.ciri         = getParam<bool>(node_.get(), "rbl_controller.ciri", false);
    rbl_params_.boundary_threshold       = getParam<double>(node_.get(), "rbl_controller.boundary_threshold", 0.2);
    rbl_params_.boundary_threshold_speed = getParam<double>(node_.get(), "rbl_controller.boundary_threshold_speed", 0.01);
    rbl_params_.lidar_tilt   = getParam<double>(node_.get(), "rbl_controller.lidar_tilt", 20.0);
    rbl_params_.lidar_fov    = getParam<double>(node_.get(), "rbl_controller.lidar_fov", 59.0);
    rbl_params_.move_centroid_to_sensed_cell =
        getParam<bool>(node_.get(), "rbl_controller.move_centroid_to_sensed_cell", false);
    octomap_msg_             = getParam<bool>(node_.get(), "rbl_controller.octomap.octomap_msg", false);
    rbl_params_.downsample_pcl = getParam<bool>(node_.get(), "rbl_controller.pcl.downsample", true);
    rbl_params_.voxel_size     = getParam<double>(node_.get(), "rbl_controller.pcl.voxel_size", 0.3);
    rbl_params_.add_estimates_as_voxels =
        getParam<bool>(node_.get(), "rbl_controller.add_estimates_as_voxels", true);
    rbl_params_.inflation_bonus = getParam<double>(node_.get(), "replanner.inflation_bonus", 0.2);
    rbl_params_.replanner_freq  = getParam<double>(node_.get(), "replanner.replanner_freq", 0.5);
    rbl_params_.replan_progress_threshold =
        getParam<double>(node_.get(), "replanner.replan_progress_threshold", 0.5);

    {
      LocalStaticMap::Params static_map_params;
      static_map_params.voxel_size = getParam<double>(node_.get(), "static_map.voxel_size", 0.3);
      static_map_params.max_range  = getParam<double>(node_.get(), "static_map.max_range", 15.0);
      static_map_ = std::make_unique<LocalStaticMap>(static_map_params);
    }

    // | ----------------------- subscribers ---------------------- |

    rclcpp::QoS qos_subs(rclcpp::KeepLast(5));

    sh_odom_.subscribe(node_.get(), "~/odom_in", qos_subs, cbkgrp_subs_);
    sh_alt_.subscribe(node_.get(), "~/alt_in", qos_subs, cbkgrp_subs_);
    if (octomap_msg_) {
      sh_octomap_.subscribe(node_.get(), "~/octomap_in", qos_subs, cbkgrp_subs_);
    }
    else {
      sh_pcl_.subscribe(node_.get(), "~/pcl_in", qos_subs, cbkgrp_subs_);
    }

    if (_is_simulated_) {
      sh_sim_group_poses_.subscribe(node_.get(), "~/sim_group_poses_in", qos_subs, cbkgrp_subs_);
    }
    else {
      sh_group_states_.subscribe(node_.get(), "~/group_states_in", qos_subs, cbkgrp_subs_);
    }

    // | ------------------------- timers ------------------------- |

    tm_set_ref_ = node_->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_tm_set_ref), std::bind(&WrapperRosRBL::cbTmSetRef, this),
        cbkgrp_timers_);

    tm_diagnostics_ = node_->create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_tm_diagnostics), std::bind(&WrapperRosRBL::cbTmDiagnostics, this),
        cbkgrp_timers_);

    // | --------------------- service servers -------------------- |

    srv_activate_control_ = node_->create_service<std_srvs::srv::Trigger>(
        "~/control_activation_in",
        std::bind(&WrapperRosRBL::cbSrvActivateControl, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        cbkgrp_ss_);

    srv_deactivate_control_ = node_->create_service<std_srvs::srv::Trigger>(
        "~/control_deactivation_in",
        std::bind(&WrapperRosRBL::cbSrvDeactivateControl, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        cbkgrp_ss_);

    srv_goto_position_ = node_->create_service<rbl_msgs::srv::Vec4>(
        "~/goto_out",
        std::bind(&WrapperRosRBL::cbSrvGotoPosition, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        cbkgrp_ss_);

    srv_set_betaD_ = node_->create_service<rbl_msgs::srv::Float64Srv>(
        "~/set_betaD",
        std::bind(&WrapperRosRBL::cbSrvSetBetaD, this, std::placeholders::_1, std::placeholders::_2),
        rclcpp::ServicesQoS(),
        cbkgrp_ss_);

    // | ----------------------- reference output ------------------ |

    pub_reference_out_ = node_->create_publisher<rbl_msgs::msg::ReferenceStamped>("~/ref_out", rclcpp::QoS(1));

    // | ----------------------- publishers ----------------------- |
    pub_viz_position_      = node_->create_publisher<visualization_msgs::msg::Marker>("~/position", rclcpp::QoS(1));
    pub_viz_centroid_      = node_->create_publisher<visualization_msgs::msg::Marker>("~/centroid", rclcpp::QoS(1));
    pub_viz_seed_B_        = node_->create_publisher<visualization_msgs::msg::Marker>("~/seed_B", rclcpp::QoS(1));
    pub_viz_target_        = node_->create_publisher<visualization_msgs::msg::Marker>("~/target", rclcpp::QoS(1));
    pub_viz_waypoint_      = node_->create_publisher<visualization_msgs::msg::Marker>("~/replanner_waypoint", rclcpp::QoS(1));
    pub_viz_cell_A_        = node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/cell_a", rclcpp::QoS(1));
    pub_viz_cell_A_sensed_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/actively_sensed_A", rclcpp::QoS(1));
    pub_viz_inflated_map_  = node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/inflated_map", rclcpp::QoS(1));
    pub_viz_cloud          = node_->create_publisher<sensor_msgs::msg::PointCloud2>("~/cloud", rclcpp::QoS(1));
    pub_viz_path_          = node_->create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(1));

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(clock_);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_);

    rbl_controller_ = std::make_shared<RBLController>(rbl_params_);
    RCLCPP_INFO_ONCE(node_->get_logger(), "Initialized RBLController with params");

    is_initialized_ = true;
    RCLCPP_INFO_ONCE(node_->get_logger(), "Initialization completed");
  }  // //}

  std::optional<geometry_msgs::msg::PointStamped> WrapperRosRBL::transformPoint(  // //{
      const geometry_msgs::msg::PointStamped& in,
      const std::string&                      target_frame)
  {
    if (in.header.frame_id == target_frame || in.header.frame_id.empty()) {
      return in;
    }

    try {
      return tf_buffer_->transform(in, target_frame, tf2::durationFromSec(0.1));
    }
    catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(node_->get_logger(), "TF: could not transform point from %s to %s: %s",
                  in.header.frame_id.c_str(), target_frame.c_str(), ex.what());
      return std::nullopt;
    }
  }  // //}

  std::optional<geometry_msgs::msg::Vector3Stamped> WrapperRosRBL::transformVector(  // //{
      const geometry_msgs::msg::Vector3Stamped& in,
      const std::string&                        target_frame)
  {
    if (in.header.frame_id == target_frame || in.header.frame_id.empty()) {
      return in;
    }

    try {
      return tf_buffer_->transform(in, target_frame, tf2::durationFromSec(0.1));
    }
    catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(node_->get_logger(), "TF: could not transform vector from %s to %s: %s",
                  in.header.frame_id.c_str(), target_frame.c_str(), ex.what());
      return std::nullopt;
    }
  }  // //}

  std::optional<sensor_msgs::msg::PointCloud2> WrapperRosRBL::transformCloud(  // //{
      const sensor_msgs::msg::PointCloud2& in,
      const std::string&                   target_frame)
  {
    if (in.header.frame_id == target_frame || in.header.frame_id.empty()) {
      return in;
    }

    try {
      return tf_buffer_->transform(in, target_frame, tf2::durationFromSec(0.1));
    }
    catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(node_->get_logger(), "TF: could not transform point cloud from %s to %s: %s",
                  in.header.frame_id.c_str(), target_frame.c_str(), ex.what());
      return std::nullopt;
    }
  }  // //}

  void WrapperRosRBL::updateGroupStates(const filter_reflective_uavs::msg::PoseVelocityArray::ConstSharedPtr& msg)
  {
    if (!msg) {
      RCLCPP_WARN(node_->get_logger(), "Received empty group states message");
      return;
    }

    if (msg->poses.size() != msg->velocities.size()) {
      RCLCPP_WARN(node_->get_logger(),
                  "Ignoring group states message with mismatched array sizes: poses=%zu velocities=%zu",
                  msg->poses.size(),
                  msg->velocities.size());
      return;
    }

    std::vector<State> updated_group_states;
    updated_group_states.reserve(msg->poses.size());

    for (std::size_t i = 0; i < msg->poses.size(); ++i) {
      geometry_msgs::msg::PointStamped point_msg;
      point_msg.header = msg->header;
      point_msg.point  = msg->poses[i].position;

      auto transformed_point = transformPoint(point_msg, _control_frame_);
      if (!transformed_point) {
        RCLCPP_WARN(node_->get_logger(),
                    "Failed to transform group state position %zu from %s to %s",
                    i,
                    msg->header.frame_id.c_str(),
                    _control_frame_.c_str());
        continue;
      }

      geometry_msgs::msg::Vector3Stamped velocity_msg;
      velocity_msg.header = msg->header;
      velocity_msg.vector = msg->velocities[i];

      auto transformed_velocity = transformVector(velocity_msg, _control_frame_);
      if (!transformed_velocity) {
        RCLCPP_WARN(node_->get_logger(),
                    "Failed to transform group state velocity %zu from %s to %s",
                    i,
                    msg->header.frame_id.c_str(),
                    _control_frame_.c_str());
        continue;
      }

      State state;
      state.position = pointToEigen(transformed_point->point);
      state.velocity = vectorToEigen(transformed_velocity->vector);
      updated_group_states.push_back(state);
    }

    group_states_ = std::move(updated_group_states);
    RCLCPP_INFO(node_->get_logger(), "Passing %zu group states to RBL", group_states_.size());

    {
      std::scoped_lock lck(mtx_rbl_);
      rbl_controller_->setGroupStates(group_states_);
    }
  }

  void WrapperRosRBL::updateGroupStates(const geometry_msgs::msg::PoseArray::ConstSharedPtr& msg)
  {
    if (!msg) {
      RCLCPP_WARN(node_->get_logger(), "Received empty message while updating group states");
      return;
    }

    Eigen::Vector3d cur_state;
    {
      std::scoped_lock lck(mtx_rbl_);
      cur_state = rbl_controller_->getCurrentPosition();
    }

    std::vector<State> updated_group_states;
    updated_group_states.reserve(msg->poses.size());

    for (const auto pose : msg->poses) {

      geometry_msgs::msg::PointStamped point_msg;
      point_msg.header = msg->header;
      point_msg.point  = pose.position;

      auto transformed_point = transformPoint(point_msg, _control_frame_);
      if (!transformed_point) {
        RCLCPP_WARN(node_->get_logger(),
                    "Failed to transform position from %s to %s",
                    msg->header.frame_id.c_str(),
                    _control_frame_.c_str());
        continue;
      }

      // do not add self pose
      if ((pointToEigen(transformed_point->point) - cur_state).norm() < 0.05) {
        continue;
      }

      State state;
      state.position = pointToEigen(transformed_point->point);
      state.velocity = Eigen::Vector3d(0, 0, 0);
      updated_group_states.push_back(state);
    }

    RCLCPP_INFO(node_->get_logger(), "Passing %zu group states to RBL", updated_group_states.size());

    {
      std::scoped_lock lck(mtx_rbl_);
      rbl_controller_->setGroupStates(updated_group_states);
    }
  }

  void WrapperRosRBL::cbTmSetRef()  // //{
  {
    if (!is_initialized_) {
      return;
    }

    if (!is_activated_) {
      RCLCPP_INFO_ONCE(node_->get_logger(), "Waiting for activation");
      return;
    }
    RCLCPP_INFO_ONCE(node_->get_logger(), "After activation");

    rbl_msgs::msg::ReferenceStamped ref_msg;
    ref_msg.header.frame_id = _control_frame_;
    ref_msg.header.stamp    = clock_->now();

    if (sh_odom_.newMsg()) {
      auto                             odom = sh_odom_.getMsg();
      geometry_msgs::msg::PointStamped tmp_pt;
      tmp_pt.header = odom->header;
      tmp_pt.point  = odom->pose.pose.position;
      auto res      = transformPoint(tmp_pt, _control_frame_);

      if (!res) {
        RCLCPP_ERROR(node_->get_logger(), "Could not transform odometry msg to control frame.");
        return;
      }

      {
        std::scoped_lock lck(mtx_rbl_);
        rbl_controller_->setCurrentPosition(pointToEigen(res.value().point));
      }
      agent_pos_world_      = pointToEigen(res.value().point);
      have_agent_pos_world_ = true;
      RCLCPP_INFO_ONCE(node_->get_logger(), "Setted cur position to rbl");

      geometry_msgs::msg::Vector3Stamped tmp_vel;
      tmp_vel.header = odom->header;
      tmp_vel.vector = odom->twist.twist.linear;
      auto vel_res   = transformVector(tmp_vel, _control_frame_);

      if (!vel_res) {
        RCLCPP_ERROR(node_->get_logger(), "Could not transform velocity to control frame.");
        return;
      }

      {
        std::scoped_lock lck(mtx_rbl_);
        rbl_controller_->setCurrentVelocity(vectorToEigen(vel_res->vector));
      }
      RCLCPP_INFO_ONCE(node_->get_logger(), "Setted velocity to rbl");

      Eigen::Vector3d euler;

      double q_x = odom->pose.pose.orientation.x;
      double q_y = odom->pose.pose.orientation.y;
      double q_z = odom->pose.pose.orientation.z;
      double q_w = odom->pose.pose.orientation.w;

      // Roll (X-axis rotation)
      double sinr_cosp = 2.0 * (q_w * q_x + q_y * q_z);
      double cosr_cosp = 1.0 - 2.0 * (q_x * q_x + q_y * q_y);
      euler.x()        = std::atan2(sinr_cosp, cosr_cosp);

      // Pitch (Y-axis rotation)
      double sinp = 2.0 * (q_w * q_y - q_z * q_x);
      if (std::abs(sinp) >= 1) {
        euler.y() = std::copysign(M_PI / 2, sinp);  // Use 90 degrees if out of range
      }
      else {
        euler.y() = std::asin(sinp);
      }

      // Yaw (Z-axis rotation)
      double siny_cosp = 2.0 * (q_w * q_z + q_x * q_y);
      double cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z);
      euler.z()        = std::atan2(siny_cosp, cosy_cosp);

      {
        std::scoped_lock lck(mtx_rbl_);
        rbl_controller_->setRollPitchYaw(euler);
      }
      RCLCPP_INFO_ONCE(node_->get_logger(), "Setted rpy to rbl");
    }

    if (sh_alt_.newMsg()) {
      auto alt = sh_alt_.getMsg();

      {
        std::scoped_lock lck(mtx_rbl_);
        rbl_controller_->setAltitude(alt->value);
      }
      RCLCPP_INFO_ONCE(node_->get_logger(), "Setted cur altitude to rbl");
    }

    if (_is_simulated_) {

      if (sh_sim_group_poses_.newMsg()) {
        RCLCPP_INFO_ONCE(node_->get_logger(), "Got new msg for update group states");
        updateGroupStates(sh_sim_group_poses_.getMsg());
        RCLCPP_INFO_ONCE(node_->get_logger(), "Updated group states");
      }
    }
    else {
      if (sh_group_states_.newMsg()) {
        RCLCPP_INFO_ONCE(node_->get_logger(), "Got new msg for update group states");
        updateGroupStates(sh_group_states_.getMsg());
        RCLCPP_INFO_ONCE(node_->get_logger(), "Updated group states");
      }
    }

    if (octomap_msg_) {
      if (sh_octomap_.newMsg()) {
        auto msg = sh_octomap_.getMsg();
        if (!msg) {
          RCLCPP_WARN(node_->get_logger(), "Received empty octomap message");
          return;
        }

        std::unique_ptr<octomap::AbstractOcTree> octree_base;
        if (msg->binary) {
          octree_base.reset(octomap_msgs::binaryMsgToMap(*msg));
        }
        else {
          octree_base.reset(octomap_msgs::fullMsgToMap(*msg));
        }
        if (!octree_base) {
          RCLCPP_ERROR(node_->get_logger(),
                       "Failed to convert %s octomap message to OcTree",
                       msg->binary ? "binary" : "full");
          return;
        }

        auto* octree = dynamic_cast<octomap::OcTree*>(octree_base.get());
        if (!octree) {
          RCLCPP_ERROR(node_->get_logger(), "Converted octomap is not an OcTree");
          return;
        }

        auto cloud             = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        cloud->header.frame_id = _control_frame_;
        cloud->points.reserve(octree->size());
        const bool  needs_transform        = msg->header.frame_id != _control_frame_;
        std::size_t total_leaf_count       = 0;
        std::size_t occupied_leaf_count    = 0;
        std::size_t transform_failed_count = 0;

        for (auto it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
          ++total_leaf_count;

          if (!octree->isNodeOccupied(*it)) {
            continue;
          }
          ++occupied_leaf_count;

          geometry_msgs::msg::Point point_msg;
          point_msg.x = it.getX();
          point_msg.y = it.getY();
          point_msg.z = it.getZ();

          if (needs_transform) {
            geometry_msgs::msg::PointStamped point_stamped;
            point_stamped.header = msg->header;
            point_stamped.point  = point_msg;

            auto transformed_point = transformPoint(point_stamped, _control_frame_);
            if (!transformed_point) {
              ++transform_failed_count;
              RCLCPP_WARN(node_->get_logger(),
                          "Failed to transform octomap point from %s to %s",
                          msg->header.frame_id.c_str(),
                          _control_frame_.c_str());
              continue;
            }

            point_msg = transformed_point->point;
          }

          pcl::PointXYZI point;
          point.x         = static_cast<float>(point_msg.x);
          point.y         = static_cast<float>(point_msg.y);
          point.z         = static_cast<float>(point_msg.z);
          point.intensity = 1.0f;
          cloud->points.push_back(point);
        }

        cloud->width    = static_cast<std::uint32_t>(cloud->points.size());
        cloud->height   = 1;
        cloud->is_dense = true;

        RCLCPP_INFO_THROTTLE(node_->get_logger(),
                             *clock_,
                             2000,
                             "Octomap stats: frame=%s -> %s, resolution=%.3f, total_leafs=%zu, occupied_leafs=%zu, "
                             "transformed_points=%zu, "
                             "transform_failures=%zu",
                             msg->header.frame_id.c_str(),
                             _control_frame_.c_str(),
                             octree->getResolution(),
                             total_leaf_count,
                             occupied_leaf_count,
                             cloud->points.size(),
                             transform_failed_count);

        if (cloud->empty()) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(),
                               *clock_,
                               2000,
                               "Converted octomap cloud is empty. total_leafs=%zu, occupied_leafs=%zu, "
                               "transform_failures=%zu",
                               total_leaf_count,
                               occupied_leaf_count,
                               transform_failed_count);
        }

        last_obstacle_cloud_ = cloud;
        pcl_loaded_          = true;

        {
          std::scoped_lock lck(mtx_rbl_);
          rbl_controller_->setPCL(last_obstacle_cloud_);
        }
        RCLCPP_INFO_ONCE(node_->get_logger(), "Setted last pcl to rbl");
      }
    }
    else {
      if (sh_pcl_.newMsg()) {
        auto msg = sh_pcl_.getMsg();

        // Unlike the octomap branch above, this used to feed *msg straight into pcl::fromROSMsg
        // with no transform at all -- harmless as long as pcl_topic was already published in
        // control_frame (e.g. map_generator's synthetic global cloud), but silently wrong for any
        // sensor whose cloud arrives in its own moving sensor frame (e.g. a real onboard lidar):
        // every point's raw sensor-relative (x,y,z) was being treated as if it were already a
        // control_frame coordinate.
        auto transformed = transformCloud(*msg, _control_frame_);
        if (!transformed) {
          return;
        }

        pcl::PointCloud<pcl::PointXYZI> tmp;
        pcl::fromROSMsg(*transformed, tmp);

        // Merge in the bounded local static-obstacle map (see LocalStaticMap) before this cloud
        // reaches rbl_controller_ at all -- both the reactive CIRI partition and the replanner's
        // grid read from the same setPCL()'d cloud downstream, so this one merge point covers
        // both. Needs a live agent position for the ray origin (free-space carving), so skip until
        // odometry has actually arrived at least once.
        if (have_agent_pos_world_) {
          static_map_->update(tmp, agent_pos_world_);
          tmp += static_map_->getOccupiedCloud();
        }

        last_obstacle_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(tmp);

        pcl_loaded_ = true;

        {
          std::scoped_lock lck(mtx_rbl_);
          rbl_controller_->setPCL(last_obstacle_cloud_);
        }
        RCLCPP_INFO_ONCE(node_->get_logger(), "Setted last pcl to rbl");
      }
    }

    if (!last_obstacle_cloud_) {
      RCLCPP_WARN(node_->get_logger(), "Waiting for obstacle cloud");
      return;
    }

    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*last_obstacle_cloud_);

    if (_group_odoms_enabled_ && _add_agents_to_pcl_) {
      cloud = addAgents2PCL(cloud, group_states_, rbl_params_.voxel_size, rbl_params_.encumbrance);
    }

    if (cloud->empty()) {
      RCLCPP_ERROR(node_->get_logger(), "PCL is empty");
      return;
    }

    {
      std::scoped_lock lck(mtx_rbl_);
      rbl_controller_->setPCL(cloud);
    }
    RCLCPP_INFO_ONCE(node_->get_logger(), "Setted curent pcl to rbl");
    pub_viz_cloud->publish(*getVizPCL(cloud, _control_frame_));

    {
      std::scoped_lock lck(mtx_rbl_);
      auto             ret = rbl_controller_->getNextRef();

      if (!ret) {
        RCLCPP_ERROR(node_->get_logger(), "Could not get next valid ref");
        return;
      }
      ref_msg.reference = ret.value();
    }

    pub_reference_out_->publish(ref_msg);

  }  // //}

  void WrapperRosRBL::cbTmDiagnostics()  // //{
  {

    if (!is_initialized_) {
      return;
    }

    Eigen::Vector3d              goal;
    Eigen::Vector3d              waypoint;
    Eigen::Vector3d              current_position;
    Eigen::Vector3d              centroid;
    Eigen::Vector3d              seed_b;
    std::vector<Eigen::Vector3d> cell_a_points;
    std::vector<Eigen::Vector3d> sensed_cell_a_points;
    std::vector<Eigen::Vector3d> inflated_map_points;
    std::vector<Eigen::Vector3d> path_points;

    {
      std::scoped_lock lck(mtx_rbl_);
      goal                 = rbl_controller_->getGoal();
      waypoint             = rbl_controller_->getWaypoint();
      current_position     = rbl_controller_->getCurrentPosition();
      centroid             = rbl_controller_->getCentroid();
      seed_b               = rbl_controller_->getSeedB();
      cell_a_points        = rbl_controller_->getCellA();
      sensed_cell_a_points = rbl_controller_->getSensedCellA();
      inflated_map_points  = rbl_controller_->getInflatedMap();
      path_points          = rbl_controller_->getPath();
    }

    pub_viz_target_->publish(getVizModGroupGoal(goal, 2 * rbl_params_.encumbrance, _control_frame_));
    pub_viz_waypoint_->publish(getVizWaypoint(waypoint, 2 * rbl_params_.encumbrance, _control_frame_));
    pub_viz_position_->publish(getVizPosition(current_position, 2 * rbl_params_.encumbrance, _control_frame_));
    pub_viz_centroid_->publish(getVizCentroid(centroid, _control_frame_));
    pub_viz_seed_B_->publish(getVizCentroid(seed_b, _control_frame_));

    auto cell_A = getVizCellA(cell_a_points, _control_frame_);
    if (cell_A) {
      pub_viz_cell_A_->publish(*cell_A);
    }
    else {
      RCLCPP_WARN(node_->get_logger(), "Failed to publish cell A");
    }

    auto cell_A_sensed = getVizCellA(sensed_cell_a_points, _control_frame_);
    if (cell_A_sensed) {
      pub_viz_cell_A_sensed_->publish(*cell_A_sensed);
    }
    else {
      RCLCPP_WARN(node_->get_logger(), "Failed to publish sensed cell A");
    }

    auto inflated_map = getVizInflatedMap(inflated_map_points, _control_frame_);
    if (inflated_map) {
      pub_viz_inflated_map_->publish(*inflated_map);
    }
    else {
      RCLCPP_WARN(node_->get_logger(), "Failed to publish inflated map");
    }

    auto path = getVizPath(path_points, _control_frame_);
    if (path) {
      pub_viz_path_->publish(*path);
    }
    else {
      RCLCPP_WARN(node_->get_logger(), "Failed to publish planned path");
    }
  }  // //}

  bool WrapperRosRBL::cbSrvActivateControl(
      [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,  // //{
      const std::shared_ptr<std_srvs::srv::Trigger::Response>                 res)
  {
    res->success = true;
    if (is_activated_) {
      res->message = "RBL is already active";
      RCLCPP_WARN(node_->get_logger(), "%s", res->message.c_str());
    }
    else {
      res->message  = "RBL activated";
      is_activated_ = true;
      RCLCPP_INFO(node_->get_logger(), "%s", res->message.c_str());
    }
    return true;
  }  // //}

  bool WrapperRosRBL::cbSrvDeactivateControl(
      [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req,  // //{
      const std::shared_ptr<std_srvs::srv::Trigger::Response>                 res)
  {
    res->success = true;
    if (!is_activated_) {
      res->message = "RBL is already deactivated";
      RCLCPP_WARN(node_->get_logger(), "%s", res->message.c_str());
    }
    else {
      res->message = "RBL deactivated";
      RCLCPP_INFO(node_->get_logger(), "%s", res->message.c_str());
    }
    return true;
  }  // //}

  bool WrapperRosRBL::cbSrvSetBetaD(const std::shared_ptr<rbl_msgs::srv::Float64Srv::Request>  req,
                                    const std::shared_ptr<rbl_msgs::srv::Float64Srv::Response> res)
  {
    {
      std::scoped_lock lck(mtx_rbl_);

      rbl_params_.betaD = req->value;
      rbl_controller_->setBetaD(req->value);
    }

    RCLCPP_INFO(node_->get_logger(), "betaD set to %.3f", req->value);

    res->success = true;
    res->message = "betaD updated";

    return true;
  }

  bool WrapperRosRBL::cbSrvGotoPosition(const std::shared_ptr<rbl_msgs::srv::Vec4::Request>  req,  // //{
                                        const std::shared_ptr<rbl_msgs::srv::Vec4::Response> res)
  {
    std::scoped_lock lck(mtx_rbl_);
    rbl_controller_->setGoal(Eigen::Vector3d{ req->goal[0], req->goal[1], req->goal[2] });
    RCLCPP_INFO(node_->get_logger(),
                "RBL goal set to [%.3f, %.3f, %.3f], heading %.3f",
                req->goal[0],
                req->goal[1],
                req->goal[2],
                req->goal[3]);
    res->success = true;
    res->message = "Goal set";
    RCLCPP_INFO(node_->get_logger(), "%s", res->message.c_str());
    return true;
  }  // //}

  visualization_msgs::msg::Marker WrapperRosRBL::getVizPosition(const Eigen::Vector3d& point,  // //{
                                                                const double           scale,
                                                                const std::string&     frame)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id    = frame;
    marker.header.stamp       = clock_->now();
    marker.ns                 = "position";
    marker.id                 = 0;
    marker.type               = visualization_msgs::msg::Marker::SPHERE;
    marker.action             = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x    = point.x();
    marker.pose.position.y    = point.y();
    marker.pose.position.z    = point.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = scale;
    marker.scale.y            = scale;
    marker.scale.z            = scale;
    marker.color.r            = 1.0;
    marker.color.g            = 0.0;
    marker.color.b            = 0.0;
    marker.color.a            = 0.3;

    return marker;
  }  // //}

  visualization_msgs::msg::Marker WrapperRosRBL::getVizModGroupGoal(const Eigen::Vector3d& point,  // //{
                                                                    const double           scale,
                                                                    const std::string&     frame)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id    = frame;
    marker.header.stamp       = clock_->now();
    marker.ns                 = "modified-group-goal";
    marker.id                 = 0;
    marker.type               = visualization_msgs::msg::Marker::SPHERE;
    marker.action             = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x    = point.x();
    marker.pose.position.y    = point.y();
    marker.pose.position.z    = point.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = scale;
    marker.scale.y            = scale;
    marker.scale.z            = scale;
    marker.color.r            = 0.0;
    marker.color.g            = 0.0;
    marker.color.b            = 1.0;
    marker.color.a            = 0.3;

    return marker;
  }  // //}

  visualization_msgs::msg::Marker WrapperRosRBL::getVizWaypoint(const Eigen::Vector3d& point,  // //{
                                                                const double           scale,
                                                                const std::string&     frame)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id    = frame;
    marker.header.stamp       = clock_->now();
    marker.ns                 = "waypoint";
    marker.id                 = 0;
    marker.type               = visualization_msgs::msg::Marker::SPHERE;
    marker.action             = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x    = point.x();
    marker.pose.position.y    = point.y();
    marker.pose.position.z    = point.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = scale;
    marker.scale.y            = scale;
    marker.scale.z            = scale;
    marker.color.r            = 0.0;
    marker.color.g            = 0.0;
    marker.color.b            = 1.0;
    marker.color.a            = 0.3;

    return marker;
  }  // //}

  std::shared_ptr<sensor_msgs::msg::PointCloud2>
  WrapperRosRBL::getVizCellA(const std::vector<Eigen::Vector3d>& points,  // //{
                             const std::string&                  frame)
  {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

    pcl_cloud.points.resize(points.size());
    pcl_cloud.width    = pcl_cloud.points.size();
    pcl_cloud.height   = 1;
    pcl_cloud.is_dense = true;

    for (size_t i = 0; i < points.size(); ++i) {
      pcl_cloud.points[i].x = points[i].x();
      pcl_cloud.points[i].y = points[i].y();
      pcl_cloud.points[i].z = points[i].z();
    }

    auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    pcl::toROSMsg(pcl_cloud, *ros_msg);

    ros_msg->header.frame_id = frame;
    ros_msg->header.stamp    = clock_->now();

    return ros_msg;
  }  // //}

  std::shared_ptr<sensor_msgs::msg::PointCloud2>
  WrapperRosRBL::getVizInflatedMap(const std::vector<Eigen::Vector3d>& points,  // //{
                                   const std::string&                  frame)
  {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

    pcl_cloud.points.resize(points.size());
    pcl_cloud.width    = pcl_cloud.points.size();
    pcl_cloud.height   = 1;
    pcl_cloud.is_dense = true;

    for (size_t i = 0; i < points.size(); ++i) {
      pcl_cloud.points[i].x = points[i].x();
      pcl_cloud.points[i].y = points[i].y();
      pcl_cloud.points[i].z = points[i].z();
    }

    auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    pcl::toROSMsg(pcl_cloud, *ros_msg);

    ros_msg->header.frame_id = frame;
    ros_msg->header.stamp    = clock_->now();

    return ros_msg;
  }  // //}

  std::shared_ptr<sensor_msgs::msg::PointCloud2>
  WrapperRosRBL::getVizPCL(const std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& pcl,  // //{
                           const std::string&                                      frame)
  {
    auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    pcl::toROSMsg(*pcl, *ros_msg);

    ros_msg->header.frame_id = frame;
    ros_msg->header.stamp    = clock_->now();

    return ros_msg;
  }  // //}

  std::shared_ptr<nav_msgs::msg::Path> WrapperRosRBL::getVizPath(const std::vector<Eigen::Vector3d>& path,  // //{
                                                                 const std::string&                  frame)
  {
    auto path_msg             = std::make_shared<nav_msgs::msg::Path>();
    path_msg->header.stamp    = clock_->now();
    path_msg->header.frame_id = frame;

    for (const auto& pt : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp    = clock_->now();
      pose.header.frame_id = frame;
      pose.pose.position.x = pt.x();
      pose.pose.position.y = pt.y();
      pose.pose.position.z = pt.z();

      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;

      path_msg->poses.push_back(pose);
    }

    return path_msg;
  }  // //}

  visualization_msgs::msg::Marker WrapperRosRBL::getVizCentroid(const Eigen::Vector3d& point,  // //{
                                                                const std::string&     frame)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id    = frame;
    marker.header.stamp       = clock_->now();
    marker.ns                 = "centroid";
    marker.id                 = 0;
    marker.type               = visualization_msgs::msg::Marker::SPHERE;
    marker.action             = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x    = point.x();
    marker.pose.position.y    = point.y();
    marker.pose.position.z    = point.z();
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = 0.2;
    marker.scale.y            = 0.2;
    marker.scale.z            = 0.2;
    marker.color.r            = 0.7;
    marker.color.g            = 0.5;
    marker.color.b            = 0.0;
    marker.color.a            = 1.0;

    return marker;
  }  // //}

  Eigen::Vector3d WrapperRosRBL::pointToEigen(const geometry_msgs::msg::Point& point)  // //{
  {
    return Eigen::Vector3d(point.x, point.y, point.z);
  }  // //}

  Eigen::Vector3d WrapperRosRBL::vectorToEigen(const geometry_msgs::msg::Vector3& vec)  // //{
  {
    return Eigen::Vector3d(vec.x, vec.y, vec.z);
  }  // //}

  geometry_msgs::msg::Point WrapperRosRBL::pointFromEigen(const Eigen::Vector3d& vec)  // //{
  {
    return createPoint(vec(0), vec(1), vec(2));
  }  // //}

  geometry_msgs::msg::Point WrapperRosRBL::createPoint(double x,  // //{
                                                       double y,
                                                       double z)
  {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  }  // //}

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>
  WrapperRosRBL::addAgents2PCL(std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>& cloud,
                               const std::vector<State>&                         group_states,
                               const double                                      voxel_size,
                               const double                                      encumbrance)
  {
    const int num_voxels_half_side = std::ceil(encumbrance / voxel_size);

    for (const auto& state : group_states) {
      const Eigen::Vector3d& position = state.position;
      RCLCPP_ERROR(node_->get_logger(),
                   "Adding points to PCL at: %.2f, %.2f, %.2f",
                   position.x(),
                   position.y(),
                   position.z());

      const int center_nx = std::floor(position.x() / voxel_size);
      const int center_ny = std::floor(position.y() / voxel_size);
      const int center_nz = std::floor(position.z() / voxel_size);

      for (int dx = -num_voxels_half_side; dx <= num_voxels_half_side; ++dx) {
        for (int dy = -num_voxels_half_side; dy <= num_voxels_half_side; ++dy) {
          for (int dz = -num_voxels_half_side; dz <= num_voxels_half_side; ++dz) {

            double dist = std::sqrt(dx * dx + dy * dy + dz * dz) * voxel_size;

            if (dist > encumbrance + voxel_size)
              continue;

            int current_nx = center_nx + dx;
            int current_ny = center_ny + dy;
            int current_nz = center_nz + dz;

            double voxel_center_x = (current_nx + 0.5) * voxel_size;
            double voxel_center_y = (current_ny + 0.5) * voxel_size;
            double voxel_center_z = (current_nz + 0.5) * voxel_size;

            pcl::PointXYZI pt;
            pt.x         = voxel_center_x;
            pt.y         = voxel_center_y;
            pt.z         = voxel_center_z;
            pt.intensity = 1.0f;
            cloud->push_back(pt);
          }
        }
      }
    }
    return cloud;
  }
}  // namespace rbl_controller

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rbl_controller::WrapperRosRBL);
