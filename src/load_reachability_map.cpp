#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "reachability_map_visualizer/msg/work_space.hpp"
#include "reachability_map_visualizer/msg/ws_sphere.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "reachability_map_visualizer/hdf5_dataset.h"

using namespace std::chrono_literals;
using namespace hdf5_dataset;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2)
  {
    RCLCPP_ERROR(rclcpp::get_logger("load_reachability_map"), 
      "Please provide the name of the reachability map. If you have not created it yet, create it by running the appropriate launch file.");
    return 1;
  }

  auto node = rclcpp::Node::make_shared("workspace");

  auto publisher = node->create_publisher<reachability_map_visualizer::msg::WorkSpace>("reachability_map", 1);

  hdf5_dataset::Hdf5Dataset h5(argv[1]);
  h5.open();

  MultiMapPtr pose_col_filter;
  MapVecDoublePtr sphere_col;
  float res;
  h5.loadMapsFromDataset(pose_col_filter, sphere_col, res);

  RCLCPP_INFO(rclcpp::get_logger("load_reachability_map"), "Map loading complete");

  // Create message
  auto ws_msg = std::make_shared<reachability_map_visualizer::msg::WorkSpace>();
  ws_msg->header.stamp = node->get_clock()->now();
  ws_msg->header.frame_id = "base_footprint";
  ws_msg->resolution = res;

  for (const auto& sphere_pair : sphere_col)
  {
    reachability_map_visualizer::msg::WsSphere wss;
    wss.point.x = (*sphere_pair.first)[0];
    wss.point.y = (*sphere_pair.first)[1];
    wss.point.z = (*sphere_pair.first)[2];
    wss.ri = sphere_pair.second;

    for (auto it1 = pose_col_filter.lower_bound(sphere_pair.first);
              it1 != pose_col_filter.upper_bound(sphere_pair.first); ++it1)
    {
      geometry_msgs::msg::Pose pose;
      pose.position.x = (*it1->second)[0];
      pose.position.y = (*it1->second)[1];
      pose.position.z = (*it1->second)[2];
      pose.orientation.x = (*it1->second)[3];
      pose.orientation.y = (*it1->second)[4];
      pose.orientation.z = (*it1->second)[5];
      pose.orientation.w = (*it1->second)[6];
      wss.poses.push_back(pose);
    }

    ws_msg->ws_spheres.push_back(wss);
  }

  // Loop to publish
  rclcpp::Rate loop_rate(0.2);  // 0.2 Hz = every 5 seconds
  while (rclcpp::ok())
  {
    RCLCPP_INFO(rclcpp::get_logger("Hdf5Dataset"), "Extracted filename: %s", ws_msg);

    ws_msg->header.stamp = node->get_clock()->now();
    publisher->publish(*ws_msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
