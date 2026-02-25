#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
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

using namespace hdf5_dataset;
using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class WsVizualisation : public rclcpp::Node
{
  public:
    WsVizualisation()
    : Node("workspace_vizualisation"), count_(0)
    {


      this->declare_parameter("h5_path", "/");
      this->declare_parameter("frame_id", "base_link");

      publisher = this->create_publisher<reachability_map_visualizer::msg::WorkSpace>("reachability_map", 1);
      publisher_voxel = this->create_publisher<visualization_msgs::msg::Marker>("voxel_grid", 1);
      subscription_ = this->create_subscription<std_msgs::msg::Int16>(
                "/index", 10, std::bind(&WsVizualisation::next_callback, this, _1) );
      timer_ = this->create_wall_timer(
      30ms, std::bind(&WsVizualisation::timer_callback, this));
    }
  public:

  private:

    void timer_callback()
    {
      if (index_map != previous_index_map){

        // extract data to pose and ri
        std::string h5_path;
        this->get_parameter("h5_path", h5_path);
        hdf5_dataset::Hdf5Dataset h5(h5_path, index_map);

        h5.open();
        double resolution_ = h5.get_resolution(); 
        double origine_offset = h5.get_origine_offset();
        MapVecDouble sphere_col;
        std::vector<std::array<double, 3>> voxels;
        h5.h5ToSpheres(sphere_col, resolution_, origine_offset);
        h5.h5ToCollision(voxels, resolution_, origine_offset);
      
        std::string frame_id;
        this->get_parameter("frame_id", frame_id);

        // Create voxel grid msg
        marker.header.stamp = this->get_clock()->now();
        marker.header.frame_id = frame_id;
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
        marker.scale.x = resolution_;
        marker.scale.y = resolution_;
        marker.scale.z = resolution_;




        ws_msg.header.stamp = this->get_clock()->now();

        ws_msg.header.frame_id = frame_id;
        ws_msg.resolution = resolution_;

        for (const auto& sphere_pair : sphere_col)
        { 
            reachability_map_visualizer::msg::WsSphere wss;
            wss.point.x = (sphere_pair.first)[0];
            wss.point.y = (sphere_pair.first)[1];
            wss.point.z = (sphere_pair.first)[2];
            wss.ri = sphere_pair.second;

            ws_msg.ws_spheres.push_back(wss);

          ws_msg.header.stamp = this->get_clock()->now();
          previous_index_map = index_map;
        }
      }
      // send msg
      this->publisher_voxel->publish(marker);
      this->publisher->publish(ws_msg);
    }
  void next_callback(const std_msgs::msg::Int16::SharedPtr msg){
    // delete previous markers
    visualization_msgs::msg::Marker del_marker;
    // del_marker.header.stamp = node->get_clock()->now();
    del_marker.header.frame_id = "base_footprint";
    // marker.id = index;
    del_marker.action = 3;
    this->publisher_voxel->publish(del_marker);
    index_map = msg->data;
  }
  rclcpp::TimerBase::SharedPtr timer_;

  size_t count_;
  int index_map = 0;
  int previous_index_map = -1;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  publisher_voxel;
  rclcpp::Publisher<reachability_map_visualizer::msg::WorkSpace>::SharedPtr publisher;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr subscription_;
  visualization_msgs::msg::Marker marker;
    // Create message RM
  reachability_map_visualizer::msg::WorkSpace ws_msg; 
  };

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WsVizualisation>());
  rclcpp::shutdown();
  return 0;
}
