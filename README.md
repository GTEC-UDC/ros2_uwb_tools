# ROS 2 UWB Tools

This ROS 2 package contains nodes for working with Ultra-Wideband (UWB) devices.

## Dependencies

Some nodes need some extra messages definitions present in the [rosmsgs](https://github.com/GTEC-UDC/rosmsgs) repository.

## Nodes:

### 1. Pozyx Ranging Reader
Node for reading distance measurements from Pozyx UWB devices.

#### Pozyx Ranging Reader Configuration

The Pozyx Ranging Reader node interfaces with Pozyx UWB devices to perform ranging measurements between a tag and multiple anchors.

##### Parameters:

- `serial` (string, default: `/dev/ttyUSB0`): UART port of the Pozyx device
- `targetDeviceId` (string, default: `0x1234`): Target device ID of the Pozyx in hexadecimal format
- `debug_level` (int, default: `0`): Log verbosity level (0-3)

##### Dependencies:

This node requires anchor positions to be published on the `/gtec/toa/anchors` topic as a MarkerArray message before it can begin ranging measurements. Therefore, it's recommended to run the Anchor Publisher node first.

##### How to Use the Pozyx Ranging Reader:

1. Connect your Pozyx device to your computer via USB
2. Make sure the Anchor Publisher is running and publishing anchor positions
3. Launch the Pozyx Ranging Reader: 

```bash
# Using the launch file with default parameters
ros2 launch ros2_uwb_tools pozyx_ranging_reader.launch.py

# With custom parameters
ros2 launch ros2_uwb_tools pozyx_ranging_reader.launch.py serial:=/dev/ttyACM0 targetDeviceId:=0x6A23 debug_level:=1
```

### 2. Anchor Publisher
Node for publishing the position of UWB anchors in the environment.

#### Anchor Publisher Configuration

The Anchor Publisher requires a JSON configuration file that specifies the location of the UWB anchors.

```json
{
  "anchors": [
    {
      "id": "0x673A",
      "label": "Anchor1",
      "position": {
        "x": 0.0,
        "y": 0.0,
        "z": 2.5
      }
    },
    {
      "id": "0x6747",
      "label": "Anchor2",
      "position": {
        "x": 5.0,
        "y": 0.0,
        "z": 2.5
      }
    },
    ...
  ]
}
```

Where:
- `id`: Hexadecimal identifier of the UWB anchor
- `label`: Optional label to identify each anchor
- `position`: Object with the coordinates in meters (`x`, `y`, `z`)

#### How to Use the Anchor Publisher

To run the node, it is **mandatory** to provide the path to a JSON file with the anchor configuration:

```bash
# Using the launch file (recommended)
ros2 launch ros2_uwb_tools anchor_publisher.launch.py json_config_path:=/absolute/path/to/my/configuration.json

# Running the node directly
ros2 run ros2_uwb_tools anchor_publisher --ros-args -p json_config_path:=/absolute/path/to/my/configuration.json
```

If the path to the JSON file is not provided, the node will display an informative error message and close.

#### Example Configuration File

The package includes an example configuration file that you can use as a reference:

```bash
# First, find the installation path of the package
PACKAGE_PATH=$(ros2 pkg prefix ros2_uwb_tools)/share/ros2_uwb_tools

# Using the included example file
ros2 launch ros2_uwb_tools anchor_publisher.launch.py json_config_path:=${PACKAGE_PATH}/config/example_anchors.json
```

## Visualization

The anchors are published as visualization markers on the topic `/gtec/toa/anchors` and can be visualized in RViz2.

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select ros2_uwb_tools
source install/setup.bash
``` 