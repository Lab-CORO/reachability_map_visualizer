

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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
using namespace std::chrono_literals;
using namespace hdf5_dataset;

static int index_map = 0;
static int previous_index_map = -1;
static rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_voxel;
static sensor_msgs::msg::PointCloud2 collision_cloud;
  // Create message RM
auto ws_msg = std::make_shared<reachability_map_visualizer::msg::WorkSpace>();



void next_callback(const std_msgs::msg::Int16::SharedPtr msg){
    // Change map index - next publish will load new map
  index_map = msg->data;
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);


  auto node = rclcpp::Node::make_shared("workspace");

  auto publisher = node->create_publisher<reachability_map_visualizer::msg::WorkSpace>("reachability_map", 1);
  publisher_voxel = node->create_publisher<sensor_msgs::msg::PointCloud2>("collision_voxels", 1);
  auto subscription = node->create_subscription<std_msgs::msg::Int16>(
            "/index", 10, next_callback );


  // Loop to publish
  rclcpp::Rate loop_rate(1);  // 0.2 Hz = every 5 seconds
  while (rclcpp::ok())
  {
  if (index_map != previous_index_map){

    // extract data to pose and ri
    std::string h5_path;
    node->declare_parameter("h5_path", "/");
    node->get_parameter("h5_path", h5_path);
    hdf5_dataset::Hdf5Dataset h5(h5_path, index_map);

    h5.open();
    double resolution_ = h5.get_resolution();
    double origine_offset = h5.get_origine_offset();

    // OPTIMIZED: Direct HDF5 reading for collision voxels with OpenMP parallelization
    std::vector<std::array<double, 3>> voxels;
    h5.h5ToCollision(voxels, resolution_, origine_offset);

    std::string frame_id;
    node->declare_parameter("frame_id", "base_link");
    node->get_parameter("frame_id", frame_id);

    // OPTIMIZED: PointCloud2 for efficient rendering (instead of CUBE_LIST marker)
    // RViz can render millions of points efficiently vs thousands of cubes
    collision_cloud.header.stamp = node->get_clock()->now();
    collision_cloud.header.frame_id = frame_id;
    collision_cloud.height = 1;
    collision_cloud.width = voxels.size();
    collision_cloud.is_dense = true;
    collision_cloud.is_bigendian = false;

    // Define PointCloud2 fields: x, y, z, rgb
    sensor_msgs::PointCloud2Modifier modifier(collision_cloud);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(voxels.size());

    // Iterators for fast access
    sensor_msgs::PointCloud2Iterator<float> iter_x(collision_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(collision_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(collision_cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(collision_cloud, "rgb");

    // Fill PointCloud2 with collision voxels (red color)
    for (const auto& voxel : voxels) {
      *iter_x = static_cast<float>(voxel[0]);
      *iter_y = static_cast<float>(voxel[1]);
      *iter_z = static_cast<float>(voxel[2]);

      // RGB color: red (255, 0, 0)
      iter_rgb[0] = 255;  // R
      iter_rgb[1] = 0;    // G
      iter_rgb[2] = 0;    // B

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_rgb;
    }

    RCLCPP_INFO(node->get_logger(), "Collision PointCloud2 created with %zu voxels", voxels.size());




    ws_msg->header.stamp = node->get_clock()->now();
    ws_msg->header.frame_id = frame_id;
    ws_msg->resolution = resolution_;

    // Remplir les dimensions de la grille depuis HDF5
    int size_x = h5.get_voxel_grid_size_x();
    int size_y = h5.get_voxel_grid_size_y();
    int size_z = h5.get_voxel_grid_size_z();

    ws_msg->size_x = size_x;
    ws_msg->size_y = size_y;
    ws_msg->size_z = size_z;

    // Remplir l'origine depuis HDF5
    double origine_x = h5.get_origine_x();
    double origine_y = h5.get_origine_y();
    double origine_z = h5.get_origine_z();

    ws_msg->origine.x = origine_x;
    ws_msg->origine.y = origine_y;
    ws_msg->origine.z = origine_z;

    // OPTIMIZED: Direct HDF5 → dense RI array (zero-copy + vectorized)
    if (!h5.h5ToRIArray(ws_msg->ri_values)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to read RI array from HDF5");
      continue;
    }

    RCLCPP_INFO(node->get_logger(), "Loaded RI array: %dx%dx%d = %zu voxels",
                size_x, size_y, size_z, ws_msg->ri_values.size());

    // Legacy: garder ws_spheres pour compatibilité (optionnel, peut être retiré)
    // Commenté pour économiser bande passante - décommenter si besoin
    /*
    for (const auto& sphere_pair : sphere_col) {
        reachability_map_visualizer::msg::WsSphere wss;
        wss.point.x = (sphere_pair.first)[0];
        wss.point.y = (sphere_pair.first)[1];
        wss.point.z = (sphere_pair.first)[2];
        wss.ri = sphere_pair.second;
        ws_msg->ws_spheres.push_back(wss);
    }
    */

    previous_index_map = index_map;
  }
  // send msg
  publisher_voxel->publish(collision_cloud);
  publisher->publish(*ws_msg);
  

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}


