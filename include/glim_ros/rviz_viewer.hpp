#pragma once

#include <any>
#include <atomic>
#include <functional>
#include <mutex>
#include <thread>
#include <chrono>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <glim/odometry/estimation_frame.hpp>
#include <glim/mapping/sub_map.hpp>
#include <glim/util/callback_slot.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>

namespace spdlog {
class logger;
}

namespace glim {

class TrajectoryManager;

/**
 * @brief Rviz-based viewer
 */
class RvizViewer : public ExtensionModuleROS2 {
public:
  RvizViewer();
  ~RvizViewer();

  virtual std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override;

private:
  void set_callbacks();
  void odometry_new_frame(const EstimationFrame::ConstPtr& new_frame, bool corrected);
  void globalmap_on_update_submaps(const std::vector<SubMap::Ptr>& submaps);
  void publish_globalmap();
  void invoke(const std::function<void()>& task);

  void spin_once();

private:
  std::atomic_bool kill_switch;
  std::thread thread;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

  rclcpp::Clock::SharedPtr map_clock;
  std::chrono::steady_clock::time_point last_globalmap_pub_time;
  double ros_pub_interval_time;

  std::string imu_frame_id;
  std::string lidar_frame_id;
  std::string base_frame_id;
  std::string odom_frame_id;
  std::string map_frame_id;
  bool publish_imu2lidar;
  double tf_time_offset;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> map_pub;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> points_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> points_corrected_pub;

  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> aligned_points_pub;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> aligned_points_corrected_pub;

  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_scanend_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_scanend_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> lidar_odom_pub;  // Odometry in LiDAR frame
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> lidar_odom_scanend_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> lidar_odom_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> lidar_odom_scanend_corrected_pub;

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_scanend_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> pose_scanend_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> lidar_pose_pub;  // Pose in LiDAR frame
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> lidar_pose_scanend_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> lidar_pose_corrected_pub;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>> lidar_pose_scanend_corrected_pub;

  std::mutex trajectory_mutex;
  std::unique_ptr<TrajectoryManager> trajectory;

  std::vector<gtsam_points::PointCloud::ConstPtr> submaps;
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> submap_poses;
  sensor_msgs::msg::PointCloud2::SharedPtr globalmap_message;
  bool globalmap_dirty = false;

  std::mutex invoke_queue_mutex;
  std::vector<std::function<void()>> invoke_queue;

  // Logging
  std::shared_ptr<spdlog::logger> logger;
  CallbackConnections callback_connections;
};
}  // namespace glim
