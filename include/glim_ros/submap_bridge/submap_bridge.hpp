#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <glim/mapping/sub_map.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim_ros/msg/submap.hpp>

namespace glim {

/**
 * @brief Publish completed submaps and a map-frame RViz cloud.
 */
class SubmapBridge : public ExtensionModuleROS2 {
public:
  SubmapBridge();
  ~SubmapBridge() override;

  std::vector<GenericTopicSubscription::Ptr> create_subscriptions(rclcpp::Node& node) override;

private:
  struct State;
  static void on_new_submap(const std::shared_ptr<State>& state, const SubMap::ConstPtr& submap);

  std::shared_ptr<State> state;
};

}  // namespace glim
