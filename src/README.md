# ROS 2 Workspace Source

## Official Flight Entry

Use this launch file for the warehouse inventory contest task:

```bash
ros2 launch my_launch demo1.launch.py
```

`demo1.launch.py` uses this aircraft's hardware adaptation:

- STM32 UART on `/dev/ttyS6`
- Bluetooth ground station on `/dev/ttyS3` at `115200`
- Single camera on `/dev/video0`
- Cargo laser on GPIO pin `10`
- STM32 mission frame `1=inventory`, `2=target`

The warehouse coordinates, route planning, target route and landing flow stay in
`activity_control_pkg`'s `warehouse_inventory_task_node`.

## Camera Debug

For camera-only debugging:

```bash
ros2 launch my_launch side_camera_debug.launch.py
```
