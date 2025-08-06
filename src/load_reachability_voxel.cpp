

#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "reachability_map_visualizer/msg/work_space.hpp"
#include "reachability_map_visualizer/msg/ws_sphere.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "reachability_map_visualizer/hdf5_dataset.h"
#include "geometry_msgs/msg/point.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/int16.hpp>
using namespace std::chrono_literals;
using namespace hdf5_dataset;

static int index_map = 0;
static    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  publisher_voxel;

void next_callback(const std_msgs::msg::Int16::SharedPtr msg){
    // delete previous markers
  visualization_msgs::msg::Marker del_marker;
  // del_marker.header.stamp = node->get_clock()->now();
  del_marker.header.frame_id = "base_footprint";
  // marker.id = index;
  del_marker.action = 3;
  publisher_voxel->publish(del_marker);
  index_map = msg->data;
}


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
  publisher_voxel = node->create_publisher<visualization_msgs::msg::Marker>("voxel_grid", 1);
  auto subscription = node->create_subscription<std_msgs::msg::Int16>(
            "/index", 10, next_callback );


  double resolution_ = 0.02; 
  double size_ = 1.5;

  // int index = 0;

  // Loop to publish
  rclcpp::Rate loop_rate(1);  // 0.2 Hz = every 5 seconds
  while (rclcpp::ok())
  {

  // extract data to pose and ri
  hdf5_dataset::Hdf5Dataset h5(argv[1], index_map);

  h5.open();

  MapVecDouble sphere_col;
  std::vector<std::array<double, 3>> voxels;
  h5.h5ToSpheres(sphere_col, resolution_, size_);
  h5.h5ToCollision(voxels, resolution_, size_);
 



  // Create voxel grid msg
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = node->get_clock()->now();
  marker.header.frame_id = "base_footprint";
  marker.id = index_map;
  marker.type = 6; // Cube list
  for (const auto& voxel : voxels)
  {
    geometry_msgs::msg::Point point;
    point.x = voxel[0];
    point.y = voxel[1];
    point.z = voxel[2];
    marker.points.push_back(point);

    std_msgs::msg::ColorRGBA color;
    color.r = 1.0;
    color.a = 0.50;
    marker.colors.push_back(color);
  }
  marker.scale.x = 0.08;
  marker.scale.y = 0.08;
  marker.scale.z = 0.08;
  // marker.color.r = .0;

  // send msg
  publisher_voxel->publish(marker);

  // Create message RM
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
    // index = (index +1) % modulo;
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}


