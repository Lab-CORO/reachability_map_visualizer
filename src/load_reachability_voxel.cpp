

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

  // double resolution_;
  // double size_;
  // node->declare_parameter("my_parameter", 0.02);
  // node->declare_parameter("size", 1.3);

  double resolution_ = 0.08; //node->get_parameter("resolution").as_double();
  double size_ = 1.3; //node->get_parameter("size").as_double();

  // node.param("resolution", resolution_, 0.02);
  // node.param("size", size_, 1.3); //workspace cube size in meter

  // extract data to pose and ri
  hdf5_dataset::Hdf5Dataset h5(argv[1]);

  h5.open();

  MapVecDouble sphere_col;
  h5.h5ToSpheres(sphere_col, resolution_, size_);

  /**
  // MultiMapPtr pose_col_filter;
  MapVecDoublePtr sphere_col;
  // float res;
  h5.h5ToMultiMapSpheres(sphere_col);
  //
  // RCLCPP_INFO(rclcpp::get_logger("load_reachability_map"), "Map loading complete");

  // Create message
  */
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

  // Loop to publish
  rclcpp::Rate loop_rate(0.2);  // 0.2 Hz = every 5 seconds
  while (rclcpp::ok())
  {
    // RCLCPP_INFO(rclcpp::get_logger("Hdf5Dataset"), "Extracted filename: %s", ws_msg);

    ws_msg->header.stamp = node->get_clock()->now();
    publisher->publish(*ws_msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}






//
//
//
// public:
//     LoadReachabilityVoxel() : nh_("~") {
//         nh_.param("resolution", resolution_, 0.02);
//         nh_.param("size", size_, 1.3); //workspace size in meter
//     }
//
//     void publishReachabilityMap(const std::string& filename) {
//         // Open the HDF5 file
//         hid_t file = H5Fopen(filename.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
//
//         if (file < 0) {
//             ROS_ERROR("Could not open file '%s'", filename.c_str());
//             return;
//         }
//
//         // Read the reachability map data
//         std::vector<float> data = readReachabilityMap(file, "/reachability_map");
//
//         // Close the HDF5 file
//         H5Fclose(file);
//
//         // Create and publish ROS messages
//         WorkSpace msg_workspace;
//         msg_workspace.header.stamp = ros::Time::now();
//         msg_workspace.ws_spheres.resolution = resolution_;
//         msg_workspace.ws_spheres.spheres.resize(data.size());
//
//         for (size_t i = 0; i < data.size(); ++i) {
//             if (data[i] != 0.0f) { // Skip zero values
//                 WsSphere& sphere = msg_workspace.ws_spheres.spheres.back();
//                 geometry_msgs::Point32 point;
//
//                 // Extract the coordinates of the sphere's center from the reachability map data
//                 point.x = i % size_ / resolution_;
//                 point.y = (i / size_) % size_ / resolution_;
//                 point.z = i / (size_ * size_) / resolution_;
//
//                 sphere.point = point;
//                 sphere.ri = data[i]; // Assume ri is equal to the reachability map value
//
//                 // You can add additional fields like poses if needed
//                 // ...
//
//                 msg_workspace.ws_spheres.spheres.push_back(sphere);
//             }
//         }
//
//         workspace_pub_.publish(msg_workspace);
//     }
//
// private:
//     std::vector<float> readReachabilityMap(hid_t file, const std::string& path) {
//         // Implement HDF5 data reading logic here
//     }
//
//     ros::NodeHandle nh_;
//     double resolution_;
//     double size_;
//     ros::Publisher workspace_pub_ = nh_.advertise<WorkSpace>("workspace", 10);
// };
//
//
// #include <rclcpp/rclcpp.hpp>
//
// int main(int argc, char** argv) {
//     // Initialize ROS2
//     rclcpp::init(argc, argv);
//
//     // Create a Node instance with the name "my_node"
//     auto node = std::make_shared<rclcpp::Node>("my_node");
//
//     // Access a parameter from the ROS param server
//     int some_param;
//     if (node->hasParameter("some_param")) {
//         node->getParameter("some_param", some_param);
//     } else {
//         RCLCPP_WARN(node->get_logger(), "Param 'some_param' not found, using default value 10");
//         some_param = 10;
//     }
//
//     // Print an information message
//     RCLCPP_INFO(node->get_logger(), "The value of some_param is: %d", some_param);
//
//     // Spin to keep the node running and processing messages
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//
//     return 0;
// }