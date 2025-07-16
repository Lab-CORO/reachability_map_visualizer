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

bool Hdf5Dataset::saveReachMapsToDataset(MultiMapPtr& poses, MapVecDoublePtr& spheres, float resolution)
{
  if (!checkPath(this->path_))
    createPath(this->path_);

  RCLCPP_INFO(rclcpp::get_logger("Hdf5Dataset"), "Saving reachability map to %s", (this->path_ + this->filename_).c_str());

   //Creating Multimap to Straight vector<vector<double> > //Can take this function to a new class
  std::vector< std::vector< double > > pose_reach;
  for (MultiMapPtr::iterator it = poses.begin(); it != poses.end(); ++it)
  {
    const std::vector<double>* sphere_coord    = it->first;
    const std::vector<double>* point_on_sphere = it->second;
    std::vector< double > pose_and_sphere(10);
    //pose_and_sphere.reserve( sphere_coord->size() + point_on_sphere->size());
    for (int i = 0; i < 3; i++)
      {
        pose_and_sphere[i]=((*sphere_coord)[i]);
      }
    for (int j = 0; j < 7; j++)
      {
        pose_and_sphere[3+j] = ((*point_on_sphere)[j]);
      }
    pose_reach.push_back(pose_and_sphere);
   }

  std::string fullpath = this->path_ + this->filename_;
  RCLCPP_INFO(rclcpp::get_logger("Hdf5Dataset"),"Saving map %s", fullpath.c_str());

  this->file_ = H5Fcreate(fullpath.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
  this->group_poses_ = H5Gcreate(this->file_, "/Poses", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  this->group_spheres_ = H5Gcreate(this->file_, "/Spheres", H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  RCLCPP_INFO(rclcpp::get_logger("Hdf5Dataset"),"Saving poses in reachability map");
  const hsize_t ndims = 2;
  const hsize_t ncols = 10;

  int posSize = poses.size();
  int chunk_size;
  int PY = 10;
  if (posSize % 2)
      {
        chunk_size = (posSize / 2) + 1;
      }
  else
      {
        chunk_size = (posSize / 2);
      }
  // Create Dataspace
  hsize_t dims[ndims] = {0, ncols};  // Starting with an empty buffer
  hsize_t max_dims[ndims] = {H5S_UNLIMITED, ncols};  // Creating dataspace
  hid_t file_space = H5Screate_simple(ndims, dims, max_dims);

  // Create Dataset Property list
  hid_t plist = H5Pcreate(H5P_DATASET_CREATE);
  H5Pset_layout(plist, H5D_CHUNKED);
  hsize_t chunk_dims[ndims] = {chunk_size, ncols};
  H5Pset_chunk(plist, ndims, chunk_dims);


  // Create the datset
  this->poses_dataset_ = H5Dcreate(this->group_poses_, "poses_dataset", H5T_NATIVE_FLOAT, file_space, H5P_DEFAULT, plist, H5P_DEFAULT);
  // Closing resources
  H5Pclose(plist);
  H5Sclose(file_space);

  // Creating the first buffer
  hsize_t nlines = chunk_size;
  float *buffer = new float[nlines * ncols];
  float **dset1_data = new float *[nlines];
  for (hsize_t i = 0; i < nlines; ++i)
    {
      dset1_data[i] = &buffer[i * ncols];
    }

  // Data for the first chunk
  for (int i = 0; i < chunk_size; i++)
    {
      for (int j = 0; j < PY; j++)
      {
        dset1_data[i][j] = pose_reach[i][j];
      }
    }
  // Memory dataspace indicating size of the buffer
  dims[0] = chunk_size;
  dims[1] = ncols;
  hid_t mem_space = H5Screate_simple(ndims, dims, NULL);

  // Extending dataset
  dims[0] = chunk_size;
  dims[1] = ncols;
  H5Dset_extent(this->poses_dataset_, dims);

  // Selecting hyperslab on the dataset
  file_space = H5Dget_space(this->poses_dataset_);
  hsize_t start[2] = {0, 0};
  hsize_t count[2] = {chunk_size, ncols};
  H5Sselect_hyperslab(file_space, H5S_SELECT_SET, start, NULL, count, NULL);

  // Writing buffer to the dataset
  H5Dwrite(this->poses_dataset_, H5T_NATIVE_FLOAT, mem_space, file_space, H5P_DEFAULT, buffer);

  // Closing file dataspace
  H5Sclose(file_space);
  // Data for the Second chunk
  for (int i = chunk_size; i < posSize; i++)
    {
      for (int j = 0; j < PY; j++)
      {
        dset1_data[i - chunk_size][j] = pose_reach[i][j];
      }
    }

  // Resizing new memory dataspace indicating new size of the buffer
  dims[0] = posSize - chunk_size;
  dims[1] = ncols;
  H5Sset_extent_simple(mem_space, ndims, dims, NULL);

  // Extend dataset
  dims[0] = posSize;
  dims[1] = ncols;
  H5Dset_extent(this->poses_dataset_, dims);
  // Selecting hyperslab
  file_space = H5Dget_space(this->poses_dataset_);
  start[0] = chunk_size;
  start[1] = 0;
  count[0] = posSize - chunk_size;
  count[1] = ncols;
  H5Sselect_hyperslab(file_space, H5S_SELECT_SET, start, NULL, count, NULL);

  // Writing buffer to dataset
  H5Dwrite(this->poses_dataset_, H5T_NATIVE_FLOAT, mem_space, file_space, H5P_DEFAULT, buffer);

  // Closing all the resources
  delete[] dset1_data;
  delete[] buffer;


  // Creating Sphere dataset
  RCLCPP_INFO(rclcpp::get_logger("Hdf5Dataset"),"Saving spheres in Reachability map");
  hid_t sphere_dataspace;
  const int SX = spheres.size();
  const int SY = 4;

  hsize_t dims2[2];  // dataset dimensions
  dims2[0] = SX;
  dims2[1] = SY;
  double dset2_data[SX][SY];

  for (MapVecDoublePtr::iterator it =  spheres.begin(); it !=spheres.end(); ++it)
    {
      for (int j = 0; j < SY - 1; j++)
      {
        dset2_data[distance( spheres.begin(), it)][j] = (*it->first)[j];
      }
      for (int j = 3; j < SY; j++)
      {
        dset2_data[distance( spheres.begin(), it)][j] = it->second;
      }
    }
  sphere_dataspace = H5Screate_simple(2, dims2, NULL);
  this->sphere_dataset_ = H5Dcreate2(this->group_spheres_, "sphere_dataset", H5T_NATIVE_DOUBLE,
                                     sphere_dataspace, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
  H5Dwrite(this->sphere_dataset_, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, dset2_data);

  // Creating attribute


  hsize_t attr_dims;
  float attr_data[1];
  attr_data[0] = resolution;
  attr_dims = 1;
  sphere_dataspace = H5Screate_simple(1, &attr_dims, NULL);
  this->attr_ = H5Acreate2(this->sphere_dataset_, "Resolution", H5T_NATIVE_FLOAT, sphere_dataspace,
                           H5P_DEFAULT, H5P_DEFAULT);
  H5Awrite(this->attr_, H5T_NATIVE_FLOAT, attr_data);
  //H5Aclose(this->attr_);

  // Closing all

  H5Sclose(sphere_dataspace);
  H5Sclose(file_space);
  H5Sclose(mem_space);
  close();
}

bool Hdf5Dataset::h5ToMultiMapPosesAndSpheres(MultiMapPtr& pose_col, MapVecDoublePtr& sphere_col)
{
  //The process of creting typedef MultiMap is little bit tricky.
  //As the data are read from the h5 file as chuck of array, the retrieved sphere addresses comes different for every sphere
  //But that is not the case for our map structure. Every sphere with same coordinate has same address in h5
  //So to create the exact same Multimap and Map while maintaining the same strcture, we have to assign the spheres that have the same coordinates, with the same address
  //Otherwise when we will be loading the dataset for visualization or other taks such as inverse map, the comparison between the coordinates of sphere dataset and
  //coordinates of spheres in Poses dataset are going to fail.

  MultiMap sphere_and_poses;
  MapVecDouble sps;
  h5ToMultiMapPoses(sphere_and_poses, sps);

  MapVecDouble sp_col;
  h5ToMultiMapSpheres(sp_col);

  for(MapVecDouble::iterator it=sps.begin(); it!=sps.end();++it)
  {
    //const std::vector<double>* sp = &(it->first);
    std::vector<double>* sp = new std::vector<double>(3);
    (*sp)[0] = (it->first)[0];
    (*sp)[1] = (it->first)[1];
    (*sp)[2] = (it->first)[2];
    MultiMap::iterator it1;
    for( it1= sphere_and_poses.lower_bound(it->first); it1 != sphere_and_poses.upper_bound(it->first); ++it1)
    {
      std::vector<double>* ps = new std::vector<double>(7);
      (*ps)[0]=(it1->second[0]);
      (*ps)[1]=(it1->second[1]);
      (*ps)[2]=(it1->second[2]);
      (*ps)[3]=(it1->second[3]);
      (*ps)[4]=(it1->second[4]);
      (*ps)[5]=(it1->second[5]);
      (*ps)[6]=(it1->second[6]);
      pose_col.insert(std::make_pair(sp, ps));
    }

    for (MapVecDouble::iterator it2 = sp_col.lower_bound(it->first); it2 !=sp_col.upper_bound(it->first); ++it2)
    {
      sphere_col.insert(std::make_pair(sp, it2->second));
    }
  }
  return 0;
}

bool Hdf5Dataset::h5ToMultiMapPoses(MultiMap& pose_col)
{
  MapVecDouble sphere_col;
  return h5ToMultiMapPoses(pose_col, sphere_col);
}

bool Hdf5Dataset::h5ToMultiMapPoses(MultiMap& pose_col, MapVecDouble& sphere_col)
{
  hsize_t dims_out[2], count[2], offset[2];
  hid_t dataspace = H5Dget_space(this->poses_dataset_); /* dataspace handle */
  int rank = H5Sget_simple_extent_ndims(dataspace);
  herr_t status_n = H5Sget_simple_extent_dims(dataspace, dims_out, NULL);
  herr_t status;
  int chunk_size, chunk_itr;
  if (dims_out[0] % 10)
  {
    chunk_itr = 11;
  }
  else
  {
    chunk_itr = 10;
  }
  chunk_size = (dims_out[0] / 10);
  offset[0] = 0;

  for (int it = 0; it < chunk_itr; it++)
  {
    offset[1] = 0;
    if ((dims_out[0] - (chunk_size * it)) / chunk_size != 0)
    {
      count[0] = chunk_size;
      offset[0] = chunk_size * it;
    }
    else
    {
      count[0] = (dims_out[0] - (chunk_size * it));
      offset[0] = count[0];
    }
    count[1] = 10;

    double data_out[count[0]][count[1]];

    status = H5Sselect_hyperslab(dataspace, H5S_SELECT_SET, offset, NULL, count, NULL);
    hsize_t dimsm[2];
    dimsm[0] = count[0];
    dimsm[1] = count[1];
    hid_t memspace;
    memspace = H5Screate_simple(RANK_OUT, dimsm, NULL);
    status = H5Dread(this->poses_dataset_, H5T_NATIVE_DOUBLE, memspace, dataspace, H5P_DEFAULT, data_out);

    for(int i=0; i<count[0]; i++)
    {
      std::vector<double> sphere_center(3);
      std::vector<double> Poses(7);
      for(int j=0; j<3;j++)
      {
        sphere_center[j] = data_out[i][j];
      }
      for(int k=3;k<10; k++)
      {
        Poses[k-3] = data_out[i][k];
      }
      pose_col.insert(std::make_pair(sphere_center, Poses));
      sphere_col.insert(std::make_pair(sphere_center, double(i)));
    }
  }
  return 0;
}

bool Hdf5Dataset::h5ToMultiMapSpheres(MapVecDouble& sphere_col)
{
  hsize_t dims_out[2], count[2], offset[2], dimsm[2];
  hid_t dataspace = H5Dget_space(this->sphere_dataset_); // dataspace handle
  int rank = H5Sget_simple_extent_ndims(dataspace);
  herr_t status_n = H5Sget_simple_extent_dims(dataspace, dims_out, NULL);
  herr_t status;
  offset[0] = 0;
  offset[1] = 0;
  count[0] = dims_out[0];
  count[1] = 4;
  double data_out[count[0]][count[1]];
  status = H5Sselect_hyperslab(dataspace, H5S_SELECT_SET, offset, NULL, count, NULL);
  dimsm[0] = count[0];
  dimsm[1] = count[1];
  hid_t memspace;
  memspace = H5Screate_simple(RANK_OUT, dimsm, NULL);
  status = H5Dread(this->sphere_dataset_, H5T_NATIVE_DOUBLE, memspace, dataspace, H5P_DEFAULT, data_out);
  for (int i = 0; i < count[0]; i++)
  {
    std::vector< double > sphere_center(3);
    double ri;
    for (int j = 0; j < 3; j++)
    {
      sphere_center[j] = data_out[i][j];
    }
    for (int k = 3; k < 4; k++)
    {
      ri = data_out[i][k];
    }
    sphere_col.insert(std::pair< std::vector< double >, double >(sphere_center, ri));
  }
  return 0;
}

bool Hdf5Dataset::h5ToResolution(float &resolution)
{
  resolution = this->res_;
  return true;
}

bool Hdf5Dataset::loadMapsFromDataset(MultiMapPtr& poses, MapVecDoublePtr& spheres, float &resolution)
{
  h5ToMultiMapPosesAndSpheres(poses, spheres);
  h5ToResolution(resolution);
  close();
  return true;
}

bool Hdf5Dataset::loadMapsFromDataset(MultiMapPtr& poses, MapVecDoublePtr& spheres)
{
  h5ToMultiMapPosesAndSpheres(poses, spheres);
  close();
  return true;
}

bool Hdf5Dataset::loadMapsFromDataset(MultiMap& poses, MapVecDouble& spheres)
{
  h5ToMultiMapPoses(poses);
  h5ToMultiMapSpheres(spheres);
  close();
  return true;
}

bool Hdf5Dataset::loadMapsFromDataset(MultiMap& poses, MapVecDouble& spheres, float &resolution)
{
  h5ToMultiMapPoses(poses);
  h5ToMultiMapSpheres(spheres);
  h5ToResolution(resolution);
  close();
  return true;
}



bool Hdf5Dataset::h5ToSpheres(MapVecDouble& sphere_col, double resolution, double size)
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
            -size + resolution * static_cast<double>(i),
            -size + resolution * static_cast<double>(j),
            -size + resolution * static_cast<double>(k)};
          sphere_col[key] = ri*100;
        }
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("load_reachability_map"), "Map generation complete.");
  return true;
}






bool Hdf5Dataset::h5ToCollision(std::vector<std::array<double, 3>> & obstacles, double resolution, double size){
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
          double x = i * 0.08 - size;
          double y = j * 0.08 - size;
          double z = k * 0.08 - size;
          obstacles.push_back({x, y, z});
        }
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("load_reachability_map"), "Map generation complete.");
  return true;  
}

}  // namespace hdf5_dataset
