# IWR6843AOPEVM Radar Driver

A ROS1-compatible driver for the IWR6843AOPEVM radar sensor, configured for precise data capture.

## Quick Start

1. **Install Dev Containers extension in VS Code** (recommended)

2. **Setup the TI driver submodule**:
   ```bash
   ./scripts/setup_submodule.sh
   ```

3. **Open in container**: Press `F1` → "Dev Containers: Reopen in Container"

4. **Build and run**:
   ```bash
   catkin_make
   source devel/setup.bash
   roslaunch ti_mmwave_rospkg 6843_multi_3d_0.launch
   ```

See [Development Guide](docs/DEVELOPMENT.md) for detailed setup and workflow.

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