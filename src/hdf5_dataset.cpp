// ROS 2 port of the original Hdf5Dataset class
#include <reachability_map_visualizer/hdf5_dataset.h>
#include <rclcpp/rclcpp.hpp>

#define RANK_OUT 2

namespace hdf5_dataset
{

Hdf5Dataset::Hdf5Dataset(std::string fullpath)
{
  std::stringstream fp(fullpath);
  std::string segment;
  std::vector<std::string> seglist;
  char the_path[256];

  while(std::getline(fp, segment, '/'))
  {
    seglist.push_back(segment);
  }

  std::ostringstream oss_file;
  oss_file<< seglist.back();
  this->filename_ = oss_file.str();

  seglist.pop_back();
  std::ostringstream oss_path;
  if (!seglist.empty())
  {
    std::copy(seglist.begin(), seglist.end()-1,
    std::ostream_iterator<std::string>(oss_path, "/"));
    oss_path << seglist.back();
    oss_path<<"/";
  }
  else
  {
    getcwd(the_path, 255);
    strcat(the_path, "/");
    oss_path << the_path;
  }

  this->path_ = oss_path.str();
  checkPath(this->path_);
  checkFileName(this->filename_);
  RCLCPP_INFO(rclcpp::get_logger("Hdf5Dataset"), "Extracted filename: %s", this->filename_.c_str());
}
Hdf5Dataset::Hdf5Dataset(std::string fullpath, int index)
{
  this->index = index;



  this->path_ = fullpath;

  RCLCPP_INFO(rclcpp::get_logger("Hdf5Dataset"), "Extracted filename: %s", this->filename_.c_str());
}
Hdf5Dataset::Hdf5Dataset(std::string path, std::string filename)
{
  this->path_ = path;
  this->filename_ = filename;
  checkPath(this->path_);
  checkFileName(this->filename_);
}

bool Hdf5Dataset::open()
{
  std::string fullpath = this->path_;
  RCLCPP_INFO(rclcpp::get_logger("Hdf5Dataset"), "Opening map %s", fullpath.c_str());

  this->file_ = H5Fopen(fullpath.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
  if (this->file_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Hdf5Dataset"), "Failed to open file: %s", fullpath.c_str());
    return false;
  }

  std::string full_group_path = "/group/" + std::to_string(this->index);
  RCLCPP_INFO(rclcpp::get_logger("Hdf5Dataset"), "Opening group %s", full_group_path.c_str());

  this->group_reachability_map_ = H5Gopen(this->file_, full_group_path.c_str(), H5P_DEFAULT);
  if (this->group_reachability_map_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Hdf5Dataset"), "Group does not exist: %s", full_group_path.c_str());
    return false;
  }

  this->reachability_map = H5Dopen(this->group_reachability_map_, "reachability_map", H5P_DEFAULT);
  if (this->reachability_map < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Hdf5Dataset"), "Dataset 'reachability_map' not found in group: %s", full_group_path.c_str());
    return false;
  }

  this->voxel_grid = H5Dopen(this->group_reachability_map_, "voxel_grid", H5P_DEFAULT);
  if (this->reachability_map < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Hdf5Dataset"), "Dataset 'reachability_map' not found in group: %s", full_group_path.c_str());
    return false;
  }

  // Get attributs
  hid_t attr_resolution = H5Aopen(this->reachability_map, "voxel_size", H5P_DEFAULT);
  if (attr_resolution < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("Hdf5Dataset"), "Attribute 'voxel_size' not found");
      return false;
  }
  hid_t type_id = H5Aget_type(attr_resolution);
  if (H5Aread(attr_resolution, type_id, &this->res_ ) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Hdf5Dataset"), "Failed to read attribute 'voxel_size'");

    return false;
}


  hid_t attr_origine = H5Aopen(this->reachability_map, "origine_y", H5P_DEFAULT);
  if (attr_origine < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("Hdf5Dataset"), "Attribute 'origine_y' not found");
      return false;
  }
  type_id = H5Aget_type(attr_origine);
  if (H5Aread(attr_origine, type_id, &this->origine_offset ) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Hdf5Dataset"), "Failed to read attribute 'origine_y'");

    return false;
}


  return true;
}


void Hdf5Dataset::close()
{
  // H5Aclose(this->attr_);
  H5Dclose(this->reachability_map);
  H5Dclose(this->voxel_grid);
  // H5Gclose(this->group_poses_);
  // H5Dclose(this->sphere_dataset_);
  // H5Gclose(this->group_spheres_);
  H5Fclose(this->file_);
}

bool Hdf5Dataset::checkPath(std::string path)
{
  if (stat(path.c_str(), &st) != 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("Hdf5Dataset"), "Path does not exist yet");
  }
  return true;
}

void Hdf5Dataset::createPath(std::string path)
{
  RCLCPP_INFO(rclcpp::get_logger("Hdf5Dataset"), "Creating Directory");
  const int dir_err = mkdir(this->path_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  if(1 == dir_err)
  {
    RCLCPP_INFO(rclcpp::get_logger("Hdf5Dataset"),"Error Creating Directory");
    exit(1);
  }
}

bool Hdf5Dataset::checkFileName(std::string filename)
{
  if (filename.find(".h5") == std::string::npos)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Hdf5Dataset"), "Filename must have .h5 extension");
    exit(1);
  }
  return true;
}





bool Hdf5Dataset::h5ToSpheres(MapVecDouble& sphere_col, double resolution, double origine_offset)
{
  RCLCPP_INFO(rclcpp::get_logger("load_reachability_map"), "Generating map...");

  if (this->reachability_map < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("load_reachability_map"), "Invalid dataset handle.");
    return false;
  }

  hid_t dataset = this->reachability_map;
  hid_t dataspace = H5Dget_space(dataset);

  int ndims = H5Sget_simple_extent_ndims(dataspace);
  if (ndims != 3 && ndims != 4) {
    RCLCPP_ERROR(rclcpp::get_logger("load_reachability_map"), "Expected a 3D or 4D dataset, got %dD.", ndims);
    H5Sclose(dataspace);
    return false;
  }

  hsize_t dims[4] = {1, 0, 0, 0};  // default for 3D
  H5Sget_simple_extent_dims(dataspace, dims, NULL);

  size_t d0 = (ndims == 4) ? dims[0] : 1;
  size_t d1 = (ndims == 4) ? dims[1] : dims[0];
  size_t d2 = (ndims == 4) ? dims[2] : dims[1];
  size_t d3 = (ndims == 4) ? dims[3] : dims[2];

  size_t total_size = d1 * d2 * d3;
  std::vector<float> data(total_size);

  // Read only the first slice if 4D
  hsize_t mem_dims[3] = {d1, d2, d3};
  hid_t memspace = H5Screate_simple(3, mem_dims, NULL);  if (ndims == 4) {
    hsize_t offset[4] = {0, 0, 0, 0};
    hsize_t count[4]  = {1, d1, d2, d3};
    H5Sselect_hyperslab(dataspace, H5S_SELECT_SET, offset, NULL, count, NULL);
  }

  herr_t status = H5Dread(dataset, H5T_NATIVE_FLOAT, memspace, dataspace, H5P_DEFAULT, data.data());
  H5Sclose(dataspace);
  H5Sclose(memspace);

  if (status < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("load_reachability_map"), "Failed to read HDF5 dataset.");
    return false;
  }

  for (size_t i = 0; i < d1; ++i) {
    for (size_t j = 0; j < d2; ++j) {
      for (size_t k = 0; k < d3; ++k) {
        size_t index = i * d2 * d3 + j * d3 + k;
        float ri = data[index];
        if (ri != 0.0f) {
          std::vector<double> key = { 
            origine_offset + resolution * static_cast<double>(i),
            origine_offset + resolution * static_cast<double>(j),
            origine_offset + resolution * static_cast<double>(k)};
          sphere_col[key] = ri*100;
        }
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("load_reachability_map"), "Map generation complete.");
  return true;
}






bool Hdf5Dataset::h5ToCollision(std::vector<std::array<double, 3>> & obstacles, double resolution, double origine_offset){
  RCLCPP_INFO(rclcpp::get_logger("load_reachability_map"), "Generating map...");

  if (this->reachability_map < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("load_reachability_map"), "Invalid dataset handle.");
    return false;
  }

  hid_t dataset = this->voxel_grid;
  hid_t dataspace = H5Dget_space(dataset);

  int ndims = H5Sget_simple_extent_ndims(dataspace);
  if (ndims != 3 && ndims != 4) {
    RCLCPP_ERROR(rclcpp::get_logger("load_reachability_map"), "Expected a 3D or 4D dataset, got %dD.", ndims);
    H5Sclose(dataspace);
    return false;
  }

  hsize_t dims[4] = {1, 0, 0, 0};  // default for 3D
  H5Sget_simple_extent_dims(dataspace, dims, NULL);

  size_t d0 = (ndims == 4) ? dims[0] : 1;
  size_t d1 = (ndims == 4) ? dims[1] : dims[0];
  size_t d2 = (ndims == 4) ? dims[2] : dims[1];
  size_t d3 = (ndims == 4) ? dims[3] : dims[2];

  size_t total_size = d1 * d2 * d3;
  std::vector<float> data(total_size);

  // Read only the first slice if 4D
  hsize_t mem_dims[3] = {d1, d2, d3};
  hid_t memspace = H5Screate_simple(3, mem_dims, NULL);  if (ndims == 4) {
    hsize_t offset[4] = {0, 0, 0, 0};
    hsize_t count[4]  = {1, d1, d2, d3};
    H5Sselect_hyperslab(dataspace, H5S_SELECT_SET, offset, NULL, count, NULL);
  }

  herr_t status = H5Dread(dataset, H5T_NATIVE_FLOAT, memspace, dataspace, H5P_DEFAULT, data.data());
  H5Sclose(dataspace);
  H5Sclose(memspace);

  if (status < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("load_reachability_map"), "Failed to read HDF5 dataset.");
    return false;
  }

  for (size_t i = 0; i < d1; ++i) {
    for (size_t j = 0; j < d2; ++j) {
      for (size_t k = 0; k < d3; ++k) {
        size_t index = i * d2 * d3 + j * d3 + k;
        float ri = data[index];
        if (ri != 0.0f) {
          // add it in the vector
          double x = i * this->res_ + origine_offset;
          double y = j * this->res_ + origine_offset;
          double z = k * this->res_ + origine_offset;
          obstacles.push_back({x, y, z});
        }
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("load_reachability_map"), "Map generation complete.");
  return true;  
}

double Hdf5Dataset::get_resolution(){
  return this->res_;
}

double Hdf5Dataset::get_origine_offset(){
  return this->origine_offset;
}

}  // namespace hdf5_dataset
