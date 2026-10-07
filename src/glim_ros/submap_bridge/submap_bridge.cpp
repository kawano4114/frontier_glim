#include <glim_ros/submap_bridge/submap_bridge.hpp>

#include <array>
#include <mutex>
#include <string>
#include <utility>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <spdlog/spdlog.h>

#define GLIM_ROS2
#include <glim/mapping/callbacks.hpp>
#include <glim/util/config.hpp>
#include <glim/util/ros_cloud_converter.hpp>

namespace glim {
namespace {

geometry_msgs::msg::Pose to_pose(const Eigen::Isometry3d& transform) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();

  Eigen::Quaterniond rotation(transform.linear());
  rotation.normalize();
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}

}  // namespace

struct SubmapBridge::State {
  std::string odom_frame_id;
  std::string map_frame_id;
  rclcpp::Publisher<glim_ros::msg::Submap>::SharedPtr submap_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_pub;
  std::mutex mutex;
  std::vector<std::array<float, 3>> map_points;
  bool active = true;
};

SubmapBridge::SubmapBridge() : state(std::make_shared<State>()) {
  const Config config(GlobalConfig::get_config_path("config_ros"));
  state->odom_frame_id = config.param<std::string>("glim_ros", "odom_frame_id", "odom");
  state->map_frame_id = config.param<std::string>("glim_ros", "map_frame_id", "map");
}

SubmapBridge::~SubmapBridge() {
  std::lock_guard<std::mutex> lock(state->mutex);
  state->active = false;
  state->submap_pub.reset();
  state->map_cloud_pub.reset();
}

std::vector<GenericTopicSubscription::Ptr> SubmapBridge::create_subscriptions(rclcpp::Node& node) {
  state->submap_pub = node.create_publisher<glim_ros::msg::Submap>("/glim/submap", rclcpp::QoS(10).reliable());
  state->map_cloud_pub = node.create_publisher<sensor_msgs::msg::PointCloud2>("/glim/submaps", rclcpp::QoS(1).reliable().transient_local());

  // CallbackSlot has no synchronized removal. Keep a weak reference so a
  // mapping callback already in progress cannot access a destroyed module.
  std::weak_ptr<State> weak_state = state;
  SubMappingCallbacks::on_new_submap.add([weak_state](const SubMap::ConstPtr& submap) {
    if (auto state = weak_state.lock()) {
      SubmapBridge::on_new_submap(state, submap);
    }
  });
  return {};
}

void SubmapBridge::on_new_submap(const std::shared_ptr<State>& state, const SubMap::ConstPtr& submap) {
  std::lock_guard<std::mutex> lock(state->mutex);
  if (!state->active) {
    return;
  }

  if (!submap || !submap->frame || submap->frame->size() == 0 || submap->frames.empty() || !state->submap_pub || !state->map_cloud_pub) {
    spdlog::warn("SubmapBridge: incomplete or empty submap; skipping publication");
    return;
  }

  const auto origin_stamp = submap->origin_frame()->stamp;
  const auto submap_frame_id = "submap_" + std::to_string(submap->id);
  const Eigen::Isometry3d& T_odom_submap = submap->T_world_origin;
  const Eigen::Isometry3d T_submap_odom = T_odom_submap.inverse();

  glim_ros::msg::Submap msg;
  msg.header.frame_id = state->odom_frame_id;
  msg.header.stamp = from_sec(origin_stamp);
  msg.id = submap->id;
  msg.origin_pose = to_pose(T_odom_submap);
  msg.cloud = *frame_to_pointcloud2(submap_frame_id, origin_stamp, *submap->frame);

  msg.lidar_trajectory.header.frame_id = submap_frame_id;
  msg.lidar_trajectory.header.stamp = msg.header.stamp;
  msg.lidar_trajectory.poses.reserve(submap->frames.size());
  for (const auto& frame : submap->frames) {
    if (!frame) {
      continue;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = submap_frame_id;
    pose.header.stamp = from_sec(frame->stamp);
    pose.pose = to_pose(T_submap_odom * frame->T_world_lidar);
    msg.lidar_trajectory.poses.push_back(std::move(pose));
  }

  state->submap_pub->publish(msg);

  // With Global Mapping disabled, GLIM's world is odom and map -> odom
  // remains identity. Keep all completed submaps visible in RViz's map frame.
  sensor_msgs::msg::PointCloud2 map_cloud;
  map_cloud.header.frame_id = state->map_frame_id;
  map_cloud.header.stamp = msg.header.stamp;

  state->map_points.reserve(state->map_points.size() + submap->frame->size());
  for (int i = 0; i < submap->frame->size(); ++i) {
    const Eigen::Vector3d point = T_odom_submap * submap->frame->points[i].head<3>();
    state->map_points.push_back({static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z())});
  }

  sensor_msgs::PointCloud2Modifier modifier(map_cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(state->map_points.size());

  sensor_msgs::PointCloud2Iterator<float> x(map_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(map_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(map_cloud, "z");
  for (const auto& point : state->map_points) {
    *x = point[0];
    *y = point[1];
    *z = point[2];
    ++x;
    ++y;
    ++z;
  }

  state->map_cloud_pub->publish(map_cloud);
  spdlog::info("SubmapBridge: published completed submap {} ({} points)", submap->id, submap->frame->size());
}

}  // namespace glim

extern "C" glim::ExtensionModule* create_extension_module() {
  return new glim::SubmapBridge();
}
