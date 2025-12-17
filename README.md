# IWR6843AOPEVM Radar Driver

A ROS1-compatible driver for the IWR6843AOPEVM radar sensor, configured for precise data capture.

## Quick Start

### USB Device Setup (One-Time)

The radar requires USB serial access. The Docker setup handles this automatically, but if you encounter issues:

**Option 1: Rebuild container (recommended)**
```bash
docker-compose -f docker/docker-compose.yml down
docker-compose -f docker/docker-compose.yml up -d --build
docker exec -it iwr6843-dev bash
```

**Option 2: Manual device setup (temporary)**
```bash
# Inside the container
/workspace/scripts/setup_usb_devices.sh
```

### Running the Radar

1. **Install Dev Containers extension in VS Code** (recommended)

2. **Enable GUI support** (required after reboot):
   ```bash
   ./scripts/enable_gui.sh
   ```

3. **Setup the TI driver submodule**:
   ```bash
   ./scripts/setup_submodule.sh
   ```

4. **Open in container**: Press `F1` → "Dev Containers: Reopen in Container"

5. **Build and run**:
   ```bash
   cd /workspace/mmwave_ti_ros/ros1_driver
   catkin_make
   source devel/setup.bash
   
   # Launch velocity publisher with RViz
   roslaunch ti_mmwave_rospkg 6843AOP_velocity_3d.launch
   
   # Or launch headless for rosbag recording
   roslaunch ti_mmwave_rospkg 6843AOP_velocity_3d_headless.launch
   ```

6. **Record data** (in another terminal):
   ```bash
   rosbag record /mmWaveDataHdl/RScanVelocity /tf /tf_static
   ```

See [Velocity Publisher README](mmwave_ti_ros/ros1_driver/src/ti_mmwave_rospkg/VELOCITY_PUBLISHER_README.md) for detailed documentation.

## Project Structure

```
radar-iwr6843-driver/
├── mmwave_ti_ros/          # TI driver submodule (the actual driver code)
├── docker/                 # Docker environment setup
├── scripts/                # Setup and helper scripts
├── config/                 # Radar configuration files
└── .devcontainer/          # VS Code dev container config
```

## Features

- Focus on exact timestamps for data capture
- Captures raw and sensor-processed data
- Supports recording via `rosbag record`
- Dockerized development environment with USB and GUI support
- Live code editing with hot-reload

## Original TI Information

- **Evaluation Board**: [IWR6843AOPEVM](https://www.ti.com/tool/de-de/IWR6843AOPEVM)
- **mmWave Demo Visualizer**: [Version 3.6.0](https://dev.ti.com/gallery/view/mmwave/mmWave_Demo_Visualizer/ver/3.6.0/)
- **Drivers**: [Radar Toolbox 2.20.00.05](https://dev.ti.com/tirex/explore/node?node=A__ANSECEN8pUpQyDw4PbR9XQ__radar_toolbox__1AslXXD__2.20.00.05)