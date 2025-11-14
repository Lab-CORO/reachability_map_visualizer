std_msgs/Header header

# Grid metadata (required)
float32 resolution
int32 size_x # The number of voxel on the axis X
int32 size_y
int32 size_z
geometry_msgs/Point origine

# Optimized: Flat array of reachability indices (size = size_x * size_y * size_z)
# Indexed as: ri_values[x * size_y * size_z + y * size_z + z]
float32[] ri_values

# Legacy format (deprecated - use ri_values for better performance)
reachability_map_visualizer/WsSphere[] ws_spheres