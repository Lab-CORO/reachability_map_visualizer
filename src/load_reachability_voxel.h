//
// Created by coro on 7/3/25.
//

#ifndef LOAD_REACHABILITY_VOXEL_H
#define LOAD_REACHABILITY_VOXEL_H



class load_reachability_voxel {

};
#include <hdf5/serial/hdf5.h>
#include <ros/ros.h>
#include "WorkSpace.msg"
#include "WsSphere.msg"

class load_reachability_voxel {
public:
    load_reachability_voxel() : nh_("~") {
        nh_.param("resolution", resolution_, 0.02);
        nh_.param("size", size_, 1.3);
    }

    void publishReachabilityMap(const std::string& filename) {
        // Open the HDF5 file
        hid_t file = H5Fopen(filename.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);

        if (file < 0) {
            ROS_ERROR("Could not open file '%s'", filename.c_str());
            return;
        }

        // Read the reachability map data
        std::vector<float> data = readReachabilityMap(file, "/reachability_map");

        // Close the HDF5 file
        H5Fclose(file);

        // Create and publish ROS messages
        WorkSpace msg_workspace;
        msg_workspace.header.stamp = ros::Time::now();
        msg_workspace.ws_spheres.resolution = resolution_;
        msg_workspace.ws_spheres.spheres.resize(data.size());

        for (size_t i = 0; i < data.size(); ++i) {
            WsSphere& sphere = msg_workspace.ws_spheres.spheres[i];
            // Fill the WorkSpace message fields with data
            // ...
        }

        workspace_pub_.publish(msg_workspace);
    }

private:
    std::vector<float> readReachabilityMap(hid_t file, const std::string& path) {
        // Implement HDF5 data reading logic here
    }

    ros::NodeHandle nh_;
    double resolution_;
    double size_;
    ros::Publisher workspace_pub_ = nh_.advertise<WorkSpace>("workspace", 10);
};


#endif //LOAD_REACHABILITY_VOXEL_H
