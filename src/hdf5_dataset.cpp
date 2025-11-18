// ROS 2 port of the original Hdf5Dataset class
#include <reachability_map_visualizer/hdf5_dataset.h>
#include <rclcpp/rclcpp.hpp>

#ifdef _OPENMP
#include <omp.h>
#endif

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

Hdf5Dataset::~Hdf5Dataset()
{
  // Ensure all HDF5 resources are properly closed
  close();
}

bool Hdf5Dataset::open() //TODO add hdf5 path to dataset 
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
      // return false;
  }
  hid_t type_id = H5Aget_type(attr_resolution);
  if (H5Aread(attr_resolution, type_id, &this->res_ ) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Hdf5Dataset"), "Failed to read attribute 'voxel_size'");

    return false;
}


  // Lire origine_x
  hid_t attr_origine_x = H5Aopen(this->reachability_map, "origine_x", H5P_DEFAULT);
  if (attr_origine_x >= 0) {
    type_id = H5Aget_type(attr_origine_x);
    H5Aread(attr_origine_x, type_id, &this->origine_x_);
    H5Aclose(attr_origine_x);
  }

  // Lire origine_y
  hid_t attr_origine_y = H5Aopen(this->reachability_map, "origine_y", H5P_DEFAULT);
  if (attr_origine_y >= 0) {
    type_id = H5Aget_type(attr_origine_y);
    H5Aread(attr_origine_y, type_id, &this->origine_y_);
    this->origine_offset = this->origine_y_;  // Pour compatibilité
    H5Aclose(attr_origine_y);
  }

  // Lire origine_z
  hid_t attr_origine_z = H5Aopen(this->reachability_map, "origine_z", H5P_DEFAULT);
  if (attr_origine_z >= 0) {
    type_id = H5Aget_type(attr_origine_z);
    H5Aread(attr_origine_z, type_id, &this->origine_z_);
    H5Aclose(attr_origine_z);
  }

  // Lire voxel_grid_size_x
  hid_t attr_size_x = H5Aopen(this->reachability_map, "voxel_grid_size_x", H5P_DEFAULT);
  if (attr_size_x >= 0) {
    type_id = H5Aget_type(attr_size_x);
    H5Aread(attr_size_x, type_id, &this->voxel_grid_size_x_);
    H5Aclose(attr_size_x);
  }

  // Lire voxel_grid_size_y
  hid_t attr_size_y = H5Aopen(this->reachability_map, "voxel_grid_size_y", H5P_DEFAULT);
  if (attr_size_y >= 0) {
    type_id = H5Aget_type(attr_size_y);
    H5Aread(attr_size_y, type_id, &this->voxel_grid_size_y_);
    H5Aclose(attr_size_y);
  }

  // Lire voxel_grid_size_z
  hid_t attr_size_z = H5Aopen(this->reachability_map, "voxel_grid_size_z", H5P_DEFAULT);
  if (attr_size_z >= 0) {
    type_id = H5Aget_type(attr_size_z);
    H5Aread(attr_size_z, type_id, &this->voxel_grid_size_z_);
    H5Aclose(attr_size_z);
  }

  H5Aclose(attr_resolution);

  RCLCPP_INFO(rclcpp::get_logger("Hdf5Dataset"), "Grid parameters: size=(%d,%d,%d), origin=(%.2f,%.2f,%.2f), resolution=%.4f",
    this->voxel_grid_size_x_, this->voxel_grid_size_y_, this->voxel_grid_size_z_,
    this->origine_x_, this->origine_y_, this->origine_z_, this->res_);

  return true;
}


void Hdf5Dataset::close()
{
  // Close datasets if they are open
  if (this->reachability_map >= 0) {
    H5Dclose(this->reachability_map);
    this->reachability_map = -1;
  }
  if (this->voxel_grid >= 0) {
    H5Dclose(this->voxel_grid);
    this->voxel_grid = -1;
  }

  // Close group if it is open
  if (this->group_reachability_map_ >= 0) {
    H5Gclose(this->group_reachability_map_);
    this->group_reachability_map_ = -1;
  }

  // Close file if it is open
  if (this->file_ >= 0) {
    H5Fclose(this->file_);
    this->file_ = -1;
  }
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
  RCLCPP_INFO(rclcpp::get_logger("Hdf5Dataset"), "Loading collision voxels (optimized)...");

  if (this->reachability_map < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Hdf5Dataset"), "Invalid dataset handle.");
    return false;
  }

  hid_t dataset = this->voxel_grid;
  hid_t dataspace = H5Dget_space(dataset);

  int ndims = H5Sget_simple_extent_ndims(dataspace);
  if (ndims != 3 && ndims != 4) {
    RCLCPP_ERROR(rclcpp::get_logger("Hdf5Dataset"), "Expected a 3D or 4D dataset, got %dD.", ndims);
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
    RCLCPP_ERROR(rclcpp::get_logger("Hdf5Dataset"), "Failed to read HDF5 dataset.");
    return false;
  }

  // OPTIMISATION: 2-pass parallèle pour extraction sparse voxels

  // Pass 1: Compter les voxels d'obstacle (parallèle avec réduction OpenMP)
  size_t collision_count = 0;

#ifdef _OPENMP
  #pragma omp parallel for collapse(3) schedule(guided, 256) reduction(+:collision_count)
#endif
  for (size_t i = 0; i < d1; ++i) {
    for (size_t j = 0; j < d2; ++j) {
      for (size_t k = 0; k < d3; ++k) {
        size_t index = i * d2 * d3 + j * d3 + k;
        if (data[index] != 0.0f) {
          ++collision_count;
        }
      }
    }
  }

  // Pré-allouer le vecteur de sortie (évite réallocations)
  obstacles.clear();
  obstacles.reserve(collision_count);

  // Pass 2: Extraction parallèle avec buffers thread-local
#ifdef _OPENMP
  int num_threads = omp_get_max_threads();
#else
  int num_threads = 1;
#endif

  std::vector<std::vector<std::array<double, 3>>> thread_buffers(num_threads);

  // Pré-allouer les buffers thread-local
  for (auto& buf : thread_buffers) {
    buf.reserve(collision_count / num_threads + 256);
  }

#ifdef _OPENMP
  #pragma omp parallel
  {
    int thread_id = omp_get_thread_num();
    auto& local_buffer = thread_buffers[thread_id];

    #pragma omp for collapse(3) schedule(guided, 256) nowait
#endif
    for (size_t i = 0; i < d1; ++i) {
      for (size_t j = 0; j < d2; ++j) {
        for (size_t k = 0; k < d3; ++k) {
          size_t index = i * d2 * d3 + j * d3 + k;
          if (data[index] != 0.0f) {
            double x = i * this->res_ + origine_offset;
            double y = j * this->res_ + origine_offset;
            double z = k * this->res_ + origine_offset;
#ifdef _OPENMP
            local_buffer.push_back({x, y, z});
#else
            obstacles.push_back({x, y, z});
#endif
          }
        }
      }
    }
#ifdef _OPENMP
  }
#endif

  // Merge thread buffers (séquentiel, mais rapide)
#ifdef _OPENMP
  for (auto& buf : thread_buffers) {
    obstacles.insert(obstacles.end(), buf.begin(), buf.end());
  }
#endif

  RCLCPP_INFO(rclcpp::get_logger("Hdf5Dataset"),
              "Collision voxels loaded: %zu occupied voxels (%.1f%% of %zu total)",
              obstacles.size(),
              100.0 * obstacles.size() / total_size,
              total_size);
  return true;
}

bool Hdf5Dataset::h5ToRIArray(std::vector<float>& ri_array)
{
  RCLCPP_INFO(rclcpp::get_logger("load_reachability_map"), "Direct HDF5 → RI array (optimized)...");

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

  hsize_t dims[4] = {1, 0, 0, 0};
  H5Sget_simple_extent_dims(dataspace, dims, NULL);

  size_t d0 = (ndims == 4) ? dims[0] : 1;
  size_t d1 = (ndims == 4) ? dims[1] : dims[0];
  size_t d2 = (ndims == 4) ? dims[2] : dims[1];
  size_t d3 = (ndims == 4) ? dims[3] : dims[2];

  size_t total_size = d1 * d2 * d3;

  // Allouer le vector cible (zéro copie - lecture directe)
  ri_array.resize(total_size);

  // Lire directement depuis HDF5 dans le vector
  hsize_t mem_dims[3] = {d1, d2, d3};
  hid_t memspace = H5Screate_simple(3, mem_dims, NULL);

  if (ndims == 4) {
    // Lire seulement la première tranche si 4D
    hsize_t offset[4] = {0, 0, 0, 0};
    hsize_t count[4]  = {1, d1, d2, d3};
    H5Sselect_hyperslab(dataspace, H5S_SELECT_SET, offset, NULL, count, NULL);
  }

  herr_t status = H5Dread(dataset, H5T_NATIVE_FLOAT, memspace, dataspace, H5P_DEFAULT, ri_array.data());
  H5Sclose(dataspace);
  H5Sclose(memspace);

  if (status < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("load_reachability_map"), "Failed to read HDF5 dataset.");
    return false;
  }

  // Multiplication vectorisée par 100.0f (OpenMP SIMD)
  #ifdef _OPENMP
  #pragma omp simd
  #endif
  for (size_t i = 0; i < total_size; ++i) {
    ri_array[i] *= 100.0f;
  }

  RCLCPP_INFO(rclcpp::get_logger("load_reachability_map"),
              "Direct read complete: %zu voxels (grid %zux%zux%zu)",
              total_size, d1, d2, d3);
  return true;
}

double Hdf5Dataset::get_resolution(){
  return this->res_;
}

double Hdf5Dataset::get_origine_offset(){
  return this->origine_offset;
}

double Hdf5Dataset::get_origine_x() {
  return this->origine_x_;
}

double Hdf5Dataset::get_origine_y() {
  return this->origine_y_;
}

double Hdf5Dataset::get_origine_z() {
  return this->origine_z_;
}

int Hdf5Dataset::get_voxel_grid_size_x() {
  return this->voxel_grid_size_x_;
}

int Hdf5Dataset::get_voxel_grid_size_y() {
  return this->voxel_grid_size_y_;
}

int Hdf5Dataset::get_voxel_grid_size_z() {
  return this->voxel_grid_size_z_;
}

}  // namespace hdf5_dataset
