# toy_block_perception

ROS 2 Humble live RGB detector for one toy block in the validated fixed work area.

## Scope

This first version is intentionally limited to:

- one colored block
- fixed camera and work area
- 640×480 RGB stream
- diffuse lighting
- yellow / green / cyan / blue / purple

It does not publish robot commands.

## Topics

Subscribes:

- `/depth_cam/rgb/image_raw` (`sensor_msgs/msg/Image`)

Publishes:

- `/toy_block/detection` (`std_msgs/msg/String`, JSON)
- `/toy_block/debug_image` (`sensor_msgs/msg/Image`, `bgr8`)
- `/toy_block/mask` (`sensor_msgs/msg/Image`, `mono8`)

## Build

```bash
cd ~/ros2_ws/src
unzip toy_block_perception.zip
cd ~/ros2_ws
colcon build --packages-select toy_block_perception
source install/setup.bash
```

## Run

```bash
ros2 launch toy_block_perception toy_block_color_detector.launch.py
```

## Inspect

```bash
ros2 topic echo /toy_block/detection
```

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/toy_block/debug_image`.

## Example JSON

```json
{
  "status": "AUTO_OK",
  "classification_status": "CLASSIFIED",
  "color": "blue",
  "confidence": 0.95,
  "stable": true,
  "bbox_xywh": [273, 140, 79, 86],
  "centroid_px": [311.59, 182.78],
  "angle_deg": 2.1
}
```
