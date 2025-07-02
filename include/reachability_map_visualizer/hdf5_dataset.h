#ifndef REACHABILITY_MAP_VISUALIZER__HDF5_DATASET_HPP_
#define REACHABILITY_MAP_VISUALIZER__HDF5_DATASET_HPP_

#include <hdf5/serial/H5Cpp.h>
#include <hdf5/serial/hdf5.h>
#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <memory>



namespace hdf5_dataset
{

using MultiMapPtr = std::multimap<const std::vector<double>*, const std::vector<double>*>;
using MapVecDoublePtr = std::map<const std::vector<double>*, double>;
using MultiMap = std::multimap<std::vector<double>, std::vector<double>>;
using MapVecDouble = std::map<std::vector<double>, double>;
using VectorOfVectors = std::vector<std::vector<double>>;
struct stat st;

class Hdf5Dataset
{
public:
  Hdf5Dataset(std::string path, std::string filename);
  Hdf5Dataset(std::string fullpath);

  bool open();
  void close();

  bool saveReachMapsToDataset(MultiMapPtr& poses, MapVecDoublePtr& spheres, float resolution);

  bool loadMapsFromDataset(MultiMapPtr& poses, MapVecDoublePtr& spheres);
  bool loadMapsFromDataset(MultiMapPtr& poses, MapVecDoublePtr& spheres, float &resolution);
  bool loadMapsFromDataset(MultiMap& poses, MapVecDouble& spheres);
  bool loadMapsFromDataset(MultiMap& poses, MapVecDouble& spheres, float &resolution);

  bool h5ToResolution(float &resolution);

private:
  bool h5ToMultiMapPosesAndSpheres(MultiMapPtr& pose_col, MapVecDoublePtr& sphere_col);
  bool h5ToMultiMapPoses(MultiMap& pose_col, MapVecDouble& sphere_col);
  bool h5ToMultiMapPoses(MultiMap& pose_col);
  bool h5ToMultiMapSpheres(MapVecDouble& sphere_col);

  bool checkPath(std::string path);
  bool checkFileName(std::string filename);
  void createPath(std::string path);

  std::string path_;
  std::string filename_;

  hid_t file_ = -1;
  hid_t group_poses_ = -1;
  hid_t group_spheres_ = -1;
  hid_t poses_dataset_ = -1;
  hid_t sphere_dataset_ = -1;
  hid_t attr_ = -1;
  float res_ = 0.0;
};

}  // namespace reachability_map_visualizer

#endif  // REACHABILITY_MAP_VISUALIZER__HDF5_DATASET_HPP_
