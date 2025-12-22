# RosBag Loader Module - Summary

## What Was Created

I've created a complete **rosbag_loader** module with the following structure:

```
/workspace/analysis/rosbag_loader/
├── __init__.py              # Module exports
├── loader.py                # Bag loading and inspection functions
├── structures.py            # Dataclass definitions
├── README.md                # Complete documentation
└── QUICKREF.py              # Quick reference guide
```

## Key Components

### 1. **Data Classes** (`structures.py`)
Strongly-typed dataclasses for all sensor data:

- **MocapPose** - Vicon motion capture poses
- **MocapAccel** - Acceleration from mocap system
- **AgirosState** - Kalman-smoothed state (better quality)
- **AgirosOdometry** - Odometry estimates (lower quality)
- **IMUData** - Raw Pixhawk IMU measurements
- **RadarPointCloud** - TI mmWave radar point clouds
- **RadarVelocity** - Radar velocity data
- **BagData** - Container for all loaded data with metadata

Each dataclass includes:
- Type-safe fields with numpy arrays
- `.to_dict()` method for DataFrame conversion
- Metadata fields (frame_id, timestamps)

### 2. **Loader Functions** (`loader.py`)

#### `inspect_bag_topics(bag_path)`
- Lists all topics in the rosbag
- Shows message types and counts
- Samples first message from each topic
- Useful for understanding bag contents before loading

#### `load_bag_topics(bag_path, verbose=True)`
- Loads all 7 topic types into dataclasses
- Handles ROS message parsing automatically
- Returns fully structured `BagData` object
- Verbose output shows progress

### 3. **Key Features**

✅ **Data Structure**
- All positions: [x, y, z] in meters
- All orientations: [qx, qy, qz, qw] quaternions
- All velocities: [vx, vy, vz] in m/s
- Timestamps: float seconds since epoch

✅ **Integration Ready**
- Easy conversion to pandas DataFrames
- NumPy arrays for computation
- Synchronized time bounds calculation
- Proper field organization for factor graphs

✅ **Quality Notes Included**
- MoCap Accel has known position jumps
- Agiros State is higher quality than Odometry
- Clear metadata about data sources

✅ **Synchronization Tools**
- `get_sync_time_bounds()` finds overlapping sensor ranges
- Easy filtering to synchronized subsets
- Index mapping for aligned measurements

## Usage Example

```python
from rosbag_loader import load_bag_topics

# Load the bag
bag_data = load_bag_topics('/workspace/rosbags/2025-12-17-16-02-22.bag')

# Access data
print(f"Loaded {len(bag_data.mocap_pose)} poses")
print(f"Loaded {len(bag_data.imu_data)} IMU samples")

# Convert to DataFrame for analysis
import pandas as pd
poses_df = pd.DataFrame([p.to_dict() for p in bag_data.mocap_pose])

# Find synchronized time range
t_min, t_max = bag_data.get_sync_time_bounds()

# Get summary
print(bag_data.summary())
```

## Example Notebook

Created **rosbag_loader_example.ipynb** with:
1. ✅ Imports and setup
2. ✅ Bag inspection
3. ✅ Data loading
4. ✅ Data access examples
5. ✅ DataFrame conversion
6. ✅ Trajectory visualization
7. ✅ IMU acceleration plots
8. ✅ Synchronization analysis
9. ✅ Factor graph usage example

## Topics Handled

| Topic | Type | Format | Quality |
|-------|------|--------|---------|
| `/mocap/angrybird2/pose` | PoseStamped | pos + ori | Raw Vicon |
| `/mocap/angrybird2/accel` | AccelStamped | acceleration | ⚠️ Has jumps |
| `/angrybird2/agiros_pilot/state` | Odometry | full state | ✅ Better |
| `/angrybird2/agiros_pilot/odometry` | Odometry | full state | Lower quality |
| `/angrybird2/imu` | Imu | accel + gyro | Raw Pixhawk |
| `/ti_mmwave/radar_scan_pcl_0` | PointCloud2 | points + intensity | Radar PCL |
| `/mmWaveDataHdl/RScanVelocity` | Custom | velocities | Radar velocity |

## Next Steps

The module is ready to use for:

1. **Data Visualization** - Convert to DataFrames and plot trajectories
2. **Analysis** - Compute statistics, correlations, error metrics
3. **Factor Graph** - Extract measurements for g2o/Ceres/GTSAM pipelines
4. **Preprocessing** - Filter, align, interpolate sensor data
5. **Validation** - Compare different sensor estimates

## Files Location

All files are in: `/workspace/analysis/rosbag_loader/`

- Use in notebooks with: `from rosbag_loader import load_bag_topics, BagData`
- Full docs: `rosbag_loader/README.md`
- Quick ref: `rosbag_loader/QUICKREF.py`
- Example: `analysis/notebooks/rosbag_loader_example.ipynb`

## Installation

If rosbag isn't installed:
```bash
pip install rosbag
```

The module dependencies:
- rosbag (for ROS bag format reading)
- numpy (for arrays)
- pandas (for DataFrames, optional but recommended)
