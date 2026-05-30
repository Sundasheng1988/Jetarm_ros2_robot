# Camera and Perception Startup Troubleshooting

## Starting the Camera

```bash
ros2 launch peripherals depth_camera.launch.py
```

## Checking the Image Stream

```bash
ros2 topic hz /depth_cam/rgb/image_raw
```

Expected rate: around 20–21 Hz.

## Starting the Perception Pipeline

```bash
ros2 launch app perception_bringup.launch.py
```

Note: `perception_bringup.launch.py` assumes the depth camera is already running.
Start the camera first, then start the perception bringup.

## Checking Topics

After launching, verify the perception pipeline with these topics:

```bash
ros2 topic list | grep world_model
```

Expected topics:

- `/world_model/roi_objects` — ROI detector output
- `/world_model/objects` — YOLO output
- `/world_model/perception_objects` — Fusion output
- `/world_model/stable_objects` — Stable tracker output

## GUI Note

ROI / YOLO GUI windows may freeze or show "not responding" on NoMachine / Gnome.
Use `ros2 topic` commands as the real health check for perception nodes.
