

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



  double resolution_ = 0.08; 
  double size_ = 1.5;

  int index = 0;

  // Loop to publish
  rclcpp::Rate loop_rate(0.2);  // 0.2 Hz = every 5 seconds
  while (rclcpp::ok())
  {

  // extract data to pose and ri
  hdf5_dataset::Hdf5Dataset h5(argv[1], index);

  h5.open();

  MapVecDouble sphere_col;
  h5.h5ToSpheres(sphere_col, resolution_, size_);

 
  // Create message
  auto ws_msg = std::make_shared<reachability_map_visualizer::msg::WorkSpace>();

  ws_msg->header.stamp = node->get_clock()->now();
  ws_msg->header.frame_id = "base_footprint";
  ws_msg->resolution = resolution_;

  for (const auto& sphere_pair : sphere_col)
  {
    reachability_map_visualizer::msg::WsSphere wss;
    wss.point.x = (sphere_pair.first)[0];
    wss.point.y = (sphere_pair.first)[1];
    wss.point.z = (sphere_pair.first)[2];
    wss.ri = sphere_pair.second;


    ws_msg->ws_spheres.push_back(wss);
  }

    ws_msg->header.stamp = node->get_clock()->now();
    publisher->publish(*ws_msg);
    index = (index +1) % 10;
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}


