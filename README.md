# Reachability Map Visualizer

Real-time visualization of reachability maps for ROS2 with optimized performance for large voxel grids (3.5M+ voxels at 10+ Hz).

## Features

- ✅ **Real-time visualization** at 10+ Hz for 152×152×152 grids (3.5M voxels)
- ✅ **Optimized rendering** with sparse grid and OpenMP parallelization (20-50x speedup)
- ✅ **Collision voxel display** using PointCloud2 for efficient rendering
- ✅ **Interactive filtering** by Reachability Index (RI) and spatial slicing
- ✅ **Low memory footprint**: 2-4 GB (down from 16 GB)

## Table of Contents

1. [Quick Start](#quick-start)
2. [Building the Project](#building-the-project)
3. [Running the Visualizer](#running-the-visualizer)
4. [RViz2 Configuration](#rviz2-configuration)
5. [HDF5 File Format](#hdf5-file-format)
6. [Message Format](#message-format)
7. [Performance Optimizations](#performance-optimizations)
8. [Troubleshooting](#troubleshooting)

---

## Quick Start

```bash
# 1. Build the package
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select reachability_map_visualizer
source install/setup.bash

# 2. Launch the visualizer with example map
ros2 run reachability_map_visualizer load_reachability_voxel \
  --ros-args \
  -p h5_path:=/path/to/reachability_map_visualizer/maps/sample_011.h5 \
  -p frame_id:=base_link

# 3. Open RViz2 and add displays (see RViz2 Configuration below)
rviz2
```

---

## Building the Project

### Prerequisites

**ROS2 Distribution:**
- ROS2 Humble or Jazzy

**System Dependencies:**
```bash
# Ubuntu/Debian
sudo apt-get install libhdf5-dev libglew-dev libomp-dev
```

**ROS2 Dependencies:**
- `sensor_msgs` (for PointCloud2)
- `geometry_msgs`
- `visualization_msgs`
- `rviz_common`, `rviz_rendering`
- `pluginlib`

### Build Commands

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws

# Source ROS2
source /opt/ros/humble/setup.bash  # or jazzy

# Build the package
colcon build --packages-select reachability_map_visualizer

# Source the workspace
source install/setup.bash
```

### Verify Build with Optimizations

To verify that OpenMP and optimizations are enabled:

```bash
# Rebuild with verbose output
colcon build --packages-select reachability_map_visualizer \
  --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON

# Check for optimization flags in build log:
# ✓ -O3 (maximum optimization)
# ✓ -march=native (CPU-specific instructions)
# ✓ -fopenmp (OpenMP multi-threading)
# ✓ "OpenMP found - enabling parallel processing"
```

---

## Running the Visualizer

### Basic Usage

The package provides a single executable `load_reachability_voxel` that loads HDF5 files and publishes both reachability map and collision voxels.

**Command:**
```bash
ros2 run reachability_map_visualizer load_reachability_voxel \
  --ros-args \
  -p h5_path:=<path_to_hdf5_file> \
  -p frame_id:=<reference_frame>
```

**Parameters:**
- `h5_path` (string): Absolute path to HDF5 file containing reachability map
- `frame_id` (string, default: `base_link`): TF frame for visualization

### Example with Sample Maps

The repository includes sample maps in the `maps/` directory:

**Example 1: Sample Map**
```bash
ros2 run reachability_map_visualizer load_reachability_voxel \
  --ros-args \
  -p h5_path:=/home/user/reachability_map_visualizer/maps/sample_011.h5 \
  -p frame_id:=base_link
```

**Example 2: Arm Reachability Map**
```bash
ros2 run reachability_map_visualizer load_reachability_voxel \
  --ros-args \
  -p h5_path:=/home/user/reachability_map_visualizer/maps/3D_reachability_map_arm_left_tool_link_0.05_2022.h5 \
  -p frame_id:=base_footprint
```

**Example 3: Base Placement Map**
```bash
ros2 run reachability_map_visualizer load_reachability_voxel \
  --ros-args \
  -p h5_path:=/home/user/reachability_map_visualizer/maps/3D_base_placement_map_arm_left_tool_link_0.05_2022.h5 \
  -p frame_id:=map
```

### Published Topics

The node publishes two topics:

| Topic | Message Type | Description | Frequency |
|-------|-------------|-------------|-----------|
| `/reachability_map` | `reachability_map_visualizer/WorkSpace` | Reachability map with RI values | 1 Hz |
| `/collision_voxels` | `sensor_msgs/PointCloud2` | Collision/obstacle voxels (red) | 1 Hz |

### Optional: Multiple Map Switching

To switch between different maps at runtime, publish to `/index`:

```bash
# Terminal 1: Run visualizer with multiple maps
ros2 run reachability_map_visualizer load_reachability_voxel \
  --ros-args \
  -p h5_path:=/path/to/maps_directory/map_collection.h5

# Terminal 2: Switch to map index 0
ros2 topic pub --once /index std_msgs/msg/Int16 "{data: 0}"

# Terminal 3: Switch to map index 1
ros2 topic pub --once /index std_msgs/msg/Int16 "{data: 1}"
```

---

## RViz2 Configuration

### 1. Reachability Map Display

**Add Display:**
1. Open RViz2: `rviz2`
2. Click **Add** → **By topic**
3. Select `/reachability_map` → `ReachMapDisplay`
   - *Or*: **Add** → **By display type** → `reachability_map_visualizer` → `ReachMapDisplay`

**Display Parameters:**

| Parameter | Type | Range | Description |
|-----------|------|-------|-------------|
| `Lowest Reachability Index` | int | 0-100 | Minimum RI to display |
| `Highest Reachability Index` | int | 0-100 | Maximum RI to display |
| `Discret → Axis` | enum | X/Y/Z/None | Slice axis for spatial filtering |
| `Discret → Min` | int | 0-grid_size | Minimum voxel index for slice |
| `Discret → Max` | int | 0-grid_size | Maximum voxel index for slice |
| `Color by Reachability` | bool | true/false | Enable RI-based coloring |
| `Show Shape` | bool | true/false | Display voxel spheres |
| `Shape Property → Size` | float | > 0.0 | Voxel size (default: resolution) |

**Color Scheme (RI-based):**
- **Blue** (RI ≥ 90): Excellent reachability
- **Cyan** (RI ≥ 50): Good reachability
- **Green** (RI ≥ 30): Average reachability
- **Yellow** (RI ≥ 5): Low reachability
- **Red** (RI < 5): Very low reachability

**Example: View Only High Reachability Zones**
```
Lowest Reachability Index: 70
Highest Reachability Index: 100
Discret → Axis: None
```
Result: Shows only voxels with RI between 70-100 (blue/cyan zones).

**Example: View Horizontal Slice**
```
Discret → Axis: Z
Discret → Min: 30  (0.6m at 0.02m resolution)
Discret → Max: 40  (0.8m at 0.02m resolution)
Lowest RI: 0
Highest RI: 100
```
Result: Shows voxels between 0.6m and 0.8m height only.

### 2. Collision Voxels Display

**Add Display:**
1. Click **Add** → **By topic**
2. Select `/collision_voxels` → **PointCloud2**

**Display Parameters:**

| Parameter | Value | Description |
|-----------|-------|-------------|
| `Style` | Cubes | Render as cubic voxels (recommended) |
| `Size (m)` | 0.02 | Voxel size (match resolution) |
| `Color Transformer` | RGB8 | Use RGB color from data |
| `Alpha` | 1.0 | Opacity (1.0 = fully opaque) |

**Alternative Styles:**
- `Points`: Fast rendering, less visual clarity
- `Spheres`: Smooth appearance, higher GPU load
- `Cubes`: Best for voxel visualization

### 3. Complete RViz2 Setup Example

**Minimal Configuration:**
```
Global Options:
  Fixed Frame: base_link

Displays:
  - ReachMapDisplay
      Topic: /reachability_map
      Lowest RI: 30
      Highest RI: 100

  - PointCloud2
      Topic: /collision_voxels
      Style: Cubes
      Size: 0.02
      Color Transformer: RGB8
```

**Save Configuration:**
- **File** → **Save Config As...** → `reachability_map_config.rviz`
- **Load on startup:** `rviz2 -d reachability_map_config.rviz`

---

## HDF5 File Format

### File Structure

The HDF5 files use a hierarchical structure with multiple groups and datasets:

```
HDF5_FILE.h5
│
├── /group/0/                        # First map (index 0)
│   ├── reachability_map (dataset)   # 3D or 4D float array
│   │   └── Attributes:
│   │       ├── voxel_size: float    # Resolution in meters (e.g., 0.02)
│   │       ├── origine_x: float     # Grid origin X coordinate
│   │       ├── origine_y: float     # Grid origin Y coordinate
│   │       ├── origine_z: float     # Grid origin Z coordinate
│   │       ├── voxel_grid_size_x: int  # Number of voxels in X
│   │       ├── voxel_grid_size_y: int  # Number of voxels in Y
│   │       └── voxel_grid_size_z: int  # Number of voxels in Z
│   │
│   └── voxel_grid (dataset)         # 3D or 4D float array (collision voxels)
│
├── /group/1/                        # Second map (index 1)
│   ├── reachability_map
│   └── voxel_grid
│
└── /group/N/                        # N-th map (index N)
    ├── reachability_map
    └── voxel_grid
```

### Dataset Details

#### 1. `reachability_map` Dataset

**Type:** 3D or 4D float array

**Dimensions:**
- **3D:** `[size_x, size_y, size_z]`
- **4D:** `[num_poses, size_x, size_y, size_z]` (only first slice used)

**Values:**
- Range: `0.0` to `1.0` (stored as normalized RI)
- `0.0` = unreachable voxel
- `1.0` = perfect reachability (100% RI)

**Attributes:**

| Attribute Name | Type | Description | Example |
|---------------|------|-------------|---------|
| `voxel_size` | float | Voxel resolution in meters | `0.02` (2cm) |
| `origine_x` | float | Grid origin X coordinate (m) | `-1.5` |
| `origine_y` | float | Grid origin Y coordinate (m) | `-1.5` |
| `origine_z` | float | Grid origin Z coordinate (m) | `0.0` |
| `voxel_grid_size_x` | int | Number of voxels in X axis | `152` |
| `voxel_grid_size_y` | int | Number of voxels in Y axis | `152` |
| `voxel_grid_size_z` | int | Number of voxels in Z axis | `152` |

**Coordinate Calculation:**
```
voxel_position_x = origine_x + (voxel_index_x * voxel_size)
voxel_position_y = origine_y + (voxel_index_y * voxel_size)
voxel_position_z = origine_z + (voxel_index_z * voxel_size)
```

**RI Value Conversion:**
```cpp
// HDF5 stores normalized values [0.0, 1.0]
// ROS message uses percentage [0, 100]
ri_percentage = hdf5_value * 100.0f
```

#### 2. `voxel_grid` Dataset

**Type:** 3D or 4D float array (same dimensions as `reachability_map`)

**Values:**
- `0.0` = free space (no collision)
- `≠ 0.0` = collision/obstacle voxel

**Purpose:** Represents collision geometry or obstacle voxels in the workspace.

### Example: Creating HDF5 File (Python)

```python
import h5py
import numpy as np

# Create HDF5 file
with h5py.File('reachability_map.h5', 'w') as f:
    # Create group for first map
    group = f.create_group('/group/0')

    # Create reachability map dataset
    size_x, size_y, size_z = 152, 152, 152
    reachability_data = np.random.rand(size_x, size_y, size_z).astype(np.float32)

    dset = group.create_dataset('reachability_map', data=reachability_data)

    # Add attributes
    dset.attrs['voxel_size'] = 0.02  # 2cm resolution
    dset.attrs['origine_x'] = -1.5   # -1.5m
    dset.attrs['origine_y'] = -1.5   # -1.5m
    dset.attrs['origine_z'] = 0.0    # ground level
    dset.attrs['voxel_grid_size_x'] = size_x
    dset.attrs['voxel_grid_size_y'] = size_y
    dset.attrs['voxel_grid_size_z'] = size_z

    # Create collision voxel grid
    collision_data = np.zeros((size_x, size_y, size_z), dtype=np.float32)
    # Mark some voxels as obstacles
    collision_data[50:60, 50:60, 0:10] = 1.0

    group.create_dataset('voxel_grid', data=collision_data)
```

### Example: Reading HDF5 File (C++)

```cpp
#include <hdf5/serial/hdf5.h>

// Open HDF5 file
hid_t file = H5Fopen("reachability_map.h5", H5F_ACC_RDONLY, H5P_DEFAULT);

// Open group (index 0)
hid_t group = H5Gopen(file, "/group/0", H5P_DEFAULT);

// Open reachability_map dataset
hid_t dataset = H5Dopen(group, "reachability_map", H5P_DEFAULT);

// Read voxel_size attribute
hid_t attr = H5Aopen(dataset, "voxel_size", H5P_DEFAULT);
float resolution;
H5Aread(attr, H5T_NATIVE_FLOAT, &resolution);
H5Aclose(attr);

// Read data
std::vector<float> data(size_x * size_y * size_z);
H5Dread(dataset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, data.data());

// Close handles
H5Dclose(dataset);
H5Gclose(group);
H5Fclose(file);
```

---

## Message Format

### 1. `WorkSpace.msg` (Reachability Map)

**Message Type:** `reachability_map_visualizer/msg/WorkSpace`

**Definition:**
```
std_msgs/Header header
float32 resolution
int32 size_x
int32 size_y
int32 size_z
geometry_msgs/Point origine
float32[] ri_values
```

**Field Descriptions:**

| Field | Type | Description |
|-------|------|-------------|
| `header` | Header | ROS message header with timestamp and frame_id |
| `resolution` | float32 | Voxel resolution in meters (e.g., 0.02) |
| `size_x` | int32 | Number of voxels in X axis |
| `size_y` | int32 | Number of voxels in Y axis |
| `size_z` | int32 | Number of voxels in Z axis |
| `origine` | Point | Grid origin coordinates (x, y, z) in meters |
| `ri_values` | float32[] | Dense RI array (size: size_x × size_y × size_z) |

**Array Organization:**

The `ri_values` array is stored in **row-major order**:

```cpp
// Index calculation:
index = x * size_y * size_z + y * size_z + z

// Access RI value for voxel (x, y, z):
float ri = ri_values[x * size_y * size_z + y * size_z + z];

// Voxel position in space:
position.x = origine.x + x * resolution;
position.y = origine.y + y * resolution;
position.z = origine.z + z * resolution;
```

**Example:**
```
header:
  stamp: {sec: 1234, nanosec: 567890}
  frame_id: "base_link"
resolution: 0.02
size_x: 152
size_y: 152
size_z: 152
origine:
  x: -1.52
  y: -1.52
  z: 0.0
ri_values: [0.0, 12.5, 45.3, 67.8, ..., 89.2]  # 3,511,808 values
```

**Memory Layout:**
```
Total voxels = size_x × size_y × size_z = 152 × 152 × 152 = 3,511,808
Array size = 3,511,808 × 4 bytes (float32) ≈ 14 MB
```

### 2. `PointCloud2` (Collision Voxels)

**Message Type:** `sensor_msgs/msg/PointCloud2`

**Published on:** `/collision_voxels`

**Fields:**
```
std_msgs/Header header
uint32 height
uint32 width
PointField[] fields
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense
```

**Point Fields:**

| Field Name | Datatype | Offset | Description |
|------------|----------|--------|-------------|
| `x` | FLOAT32 | 0 | Voxel X position (meters) |
| `y` | FLOAT32 | 4 | Voxel Y position (meters) |
| `z` | FLOAT32 | 8 | Voxel Z position (meters) |
| `rgb` | UINT32 | 12 | RGB color (red: 0xFF0000) |

**Point Cloud Properties:**
- `height`: 1 (unordered cloud)
- `width`: Number of occupied collision voxels
- `point_step`: 16 bytes per point
- `is_dense`: true (no invalid points)

**RGB Encoding:**
```cpp
// Collision voxels are encoded as RED
uint8_t r = 255, g = 0, b = 0;
uint32_t rgb = ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
// Result: 0x00FF0000 (red)
```

**Example:**
```
header:
  stamp: {sec: 1234, nanosec: 567890}
  frame_id: "base_link"
height: 1
width: 350000  # 350k occupied voxels
point_step: 16
is_dense: true
data: [x0, y0, z0, rgb0, x1, y1, z1, rgb1, ...]  # 5.6 MB for 350k points
```

---

## Performance Optimizations

The visualizer includes 4 levels of optimization for real-time performance:

### Optimization Levels

| Level | Name | Technique | Speedup | Status |
|-------|------|-----------|---------|--------|
| **0** | Base HDF5 | Direct read + SIMD vectorization | Baseline | ✅ Active |
| **1** | Multi-threading | OpenMP collapse(3) | 7x | ✅ Active |
| **2** | Sparse Grid | 2-pass parallel + cache | 20-50x | ✅ Active |
| **3** | GPU Shaders | Compute shaders OpenGL | 75-150x | ⚠️ Disabled* |
| **4** | Collision Voxels | PointCloud2 vs CUBE_LIST | ∞ | ✅ Active |

*Level 3 disabled due to RViz/Ogre OpenGL context conflicts

### Performance Benchmarks

**Configuration:** 152×152×152 grid (3.5M voxels), ~10% visible

| Component | Time/Frame | Frequency | Memory |
|-----------|-----------|-----------|--------|
| Reachability map (Level 2) | 3-8 ms | 120-330 Hz | 1-2 GB |
| Collision voxels (Level 4) | 15-30 ms | 30-60 Hz | 1-2 GB |
| **Total** | **~42 ms** | **~23 Hz** | **2-4 GB** |

**Goal:** 10 Hz → **Achieved with 130% margin** ✅

### OpenMP Configuration

**Automatic (Recommended):**
```bash
# Uses all available CPU cores automatically
ros2 run reachability_map_visualizer load_reachability_voxel ...
```

**Manual Thread Control:**
```bash
# Set number of threads (e.g., 8 cores)
export OMP_NUM_THREADS=8
ros2 run reachability_map_visualizer load_reachability_voxel ...
```

**Verify OpenMP Activity:**
```bash
# Monitor CPU usage - should see multiple cores active
htop
```

For detailed performance analysis, see [OPTIMIZATIONS.md](OPTIMIZATIONS.md).

---

## Troubleshooting

### Build Issues

**Error: `OpenMP not found`**
```bash
sudo apt-get install libomp-dev
colcon build --packages-select reachability_map_visualizer --cmake-clean-cache
```

**Error: `omp_get_max_threads was not declared`**
- Fixed in latest version
- Ensure you have `#include <omp.h>` in source files

### Runtime Issues

**No visualization in RViz:**
1. Check topic is published: `ros2 topic list | grep reachability`
2. Verify frame_id matches RViz Fixed Frame
3. Check display is enabled in RViz

**Slow performance / lag:**
1. Verify OpenMP is enabled: `colcon build ... | grep "OpenMP found"`
2. Check CPU usage with `htop` - should use multiple cores
3. Reduce grid size or increase RI filter threshold

**Collision voxels not visible:**
1. Verify topic: `ros2 topic echo /collision_voxels --once`
2. Check PointCloud2 display Color Transformer is set to **RGB8**
3. Ensure `Size (m)` matches voxel resolution (e.g., 0.02)

### HDF5 File Issues

**Error: `Failed to open file`**
- Verify file path is absolute
- Check file permissions: `chmod 644 map.h5`

**Error: `Dataset 'reachability_map' not found`**
- Verify HDF5 structure: `h5dump -H map.h5`
- Ensure group exists: `/group/0/reachability_map`

---

## Additional Resources

- **Technical Documentation:** [OPTIMIZATIONS.md](OPTIMIZATIONS.md)
- **Performance Guide (French):** [PERFORMANCE_GUIDE.md](PERFORMANCE_GUIDE.md)
- **Issues & Support:** [GitHub Issues](https://github.com/Lab-CORO/reachability_map_visualizer/issues)

---

## License

TODO: Add license information

## Citation

If you use this package in your research, please cite:

```
TODO: Add citation information
```

---

## Changelog

**v1.0** (2024-11-15):
- ✅ Level 1-4 optimizations implemented
- ✅ Real-time performance: 23 Hz for 3.5M voxels
- ✅ Memory optimized: 2-4 GB (down from 16 GB)
- ✅ PointCloud2 collision voxel rendering
- ✅ OpenMP parallelization with collapse(3)
- ✅ Sparse grid with intelligent caching
