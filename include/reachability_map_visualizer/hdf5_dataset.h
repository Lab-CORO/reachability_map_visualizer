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
  Hdf5Dataset(std::string fullpath, int index);

  bool open();
  void close();

  bool h5ToSpheres(MapVecDouble& sphere_col, double resolution, double origine_offset);
  bool h5ToCollision(std::vector<std::array<double, 3>> & obstacles, double resolution, double origine_offset);
  double get_resolution();
  double get_origine_offset();

private:

  bool checkPath(std::string path);
  bool checkFileName(std::string filename);
  void createPath(std::string path);

  std::string path_;
  std::string filename_;

  hid_t file_ = -1;
  hid_t group_poses_ = -1;
  hid_t group_reachability_map_ = -1;
  hid_t reachability_map = -1;
  hid_t voxel_grid = -1;
  hid_t group_spheres_ = -1;
  hid_t poses_dataset_ = -1;
  hid_t sphere_dataset_ = -1;
  hid_t attr_ = -1;
  double res_ = 0.0;
  double origine_offset = 0.0;
  int index = 0;
};

}  // namespace reachability_map_visualizer

#endif  // REACHABILITY_MAP_VISUALIZER__HDF5_DATASET_HPP_
