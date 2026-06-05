# opencv01

单摄像头二维码识别与视觉微调包。

## 节点

- `qr_decoder_single`：唯一的二维码识别节点，默认使用 `/dev/video0`，发布 `/qr/id`、`/qr/offset_norm`、`/qr/aligned`、`/qr/debug_image`。
- `qr_decoder`：兼容命令，和 `qr_decoder_single` 指向同一个单摄像头节点。
- `qr_fine_tune`：订阅 `/qr/offset_norm` 和 `/qr/aligned`，输出 `/qr/fine_offset_body_cm`，给航点状态机做视觉微调。

## 正式飞行

正式任务由 `my_launch` 启动：

```bash
ros2 launch my_launch demo1.launch.py
```

`demo1.launch.py` 会设置 `qr_task_active_required:=true`。二维码节点只有在航点状态机发布 `/qr_task_active=true` 时才会解码，所以普通飞行途中遇到二维码不会发布 `/qr/id`，也不会误触发后续流程。

正式任务中激光只听 `/qr/fire_laser`，航点状态机在到达目标点、视觉对准、记录/校验二维码后控制 pin10 点亮 1 秒。

## 单独测试摄像头和激光

```bash
ros2 run opencv01 qr_decoder_single --ros-args \
  -p camera_device:=/dev/video0 \
  -p enable_gui:=true \
  -p qr_task_active_required:=false \
  -p laser_pin:=10 \
  -p laser_duration_sec:=1.0 \
  -p standalone_laser_once:=true
```

单独测试时，节点会正常解码；当二维码进入对准窗口后，pin10 激光自动点亮一次，持续 1 秒，不会重复打。

如果要同时看视觉微调输出，另开一个终端：

```bash
ros2 run opencv01 qr_fine_tune
ros2 topic echo /qr/fine_offset_body_cm
```
