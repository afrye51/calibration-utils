# calibration-utils

Tools and visualizations for intrinsic and extrinsic sensor calibration

## Tips and tricks

- `rqt` is a great tool for monitoring node logs and topic statuses over the network

## Installation

Install ROS2 Humble (or use docker if it's the only option)

```bash
mkdir ~/utils_ws
mkdir ~/utils_ws/src
cd ~/utils_ws/src
git clone https://github.com/afrye51/calibration-utils
cd ~/utils_ws
colcon build
```

`echo "source ~/utils_ws/install/setup.bash" >> ~/.bashrc`
