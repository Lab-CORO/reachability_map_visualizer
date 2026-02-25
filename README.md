# reachability_map_visualizer

ROS2 package for visualizing robot arm reachability maps in RViz. It reads HDF5 reachability data and provides an interactive 3D visualization through a custom RViz plugin.

## Dependencies

- ROS2 (Humble or later)
- RViz2
- HDF5 C++ library (`libhdf5-dev`)

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select reachability_map_visualizer
source install/setup.bash
```

## Usage

### Run the node

```bash
ros2 run reachability_map_visualizer load_reachability_voxel \
  --ros-args \
  -p h5_path:=/path/to/map.h5 \
  -p frame_id:=base_link
```

### Open with RViz preset

```bash
rviz2 -d $(ros2 pkg prefix reachability_map_visualizer)/share/reachability_map_visualizer/rviz/reachability_map.rviz
```

### Add the plugin manually in RViz

1. Click **Add** in the Displays panel
2. Select **ReachabilityMap** under `reachability_map_visualizer`
3. Set the **Topic** to `/reachability_map`

### Switch to a different map index

```bash
ros2 topic pub --once /index std_msgs/msg/Int16 "data: 0"
```

## Node Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `h5_path` | string | `"/"` | Absolute path to the `.h5` reachability map file |
| `frame_id` | string | `"base_link"` | TF frame in which the map is published |

## Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/index` | `std_msgs/Int16` | Index of the map to load from the HDF5 file |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/reachability_map` | `reachability_map_visualizer/WorkSpace` | Workspace spheres with reachability indices |
| `/voxel_grid` | `visualization_msgs/Marker` | Collision voxel grid (cube list) |

## RViz Plugin Properties

### Display

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `Show Poses` | bool | `false` | Display arrows for each pose in the map |
| `Show Shape` | bool | `true` | Display spheres for each voxel |
| `Color by Reachability` | bool | `true` | Color spheres using a jet colormap (blue = high, red = low) |

### Shape

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `Shape` | enum | `Sphere` | Voxel shape: `Sphere`, `Cylinder`, `Cone`, or `Cube` |
| `Color` | RGB | Yellow | Uniform color (used when *Color by Reachability* is off) |
| `Alpha` | float | `1.0` | Transparency (0 = invisible, 1 = opaque) |
| `Size` | float | `0.05` m | Size of each shape |

### Filtering

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `Lowest RI` | int | `0` | Minimum reachability index to display (0–100) |
| `Highest RI` | int | `100` | Maximum reachability index to display (0–100) |

### Slicing

| Property | Type | Default | Description |
|----------|------|---------|-------------|
| `Axis` | enum | `None` | Slice axis: `None`, `X`, `Y`, or `Z` |
| `Min` | int | `0` | Start index along the selected slice axis |
| `Max` | int | `0` | End index along the selected slice axis |

## HDF5 Map Format

Map files must follow this structure:

```
/group/{index}/
  reachability_map   # 3D/4D float dataset (reachability indices)
  voxel_grid         # 3D/4D float dataset (collision voxels)
  Attributes: voxel_size, origine_y
```

Sample maps are available in the `maps/` directory.
