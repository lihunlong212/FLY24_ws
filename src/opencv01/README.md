# opencv01

单摄像头二维码识别与视觉微调包。

## 节点

- `qr_decoder_single`：唯一的二维码识别节点，默认使用 `/dev/video0`，发布 `/qr/id`、`/qr/offset_norm`、`/qr/aligned`、`/qr/debug_image`。
- `qr_decoder`：兼容命令，和 `qr_decoder_single` 指向同一个单摄像头节点。
- `qr_fine_tune`：订阅 `/qr/offset_norm` 和 `/qr/aligned`，输出 `/qr/fine_offset_body_cm`，给航点状态机做视觉微调。

本包只保留单摄像头识别链路。

## 单独测试摄像头

```bash
ros2 run opencv01 qr_decoder_single --ros-args \
  -p camera_device:=/dev/video0 \
  -p enable_gui:=true \
  -p laser_task_active_required:=false
```

如果要同时看视觉微调输出，另开一个终端：

```bash
ros2 run opencv01 qr_fine_tune
ros2 topic echo /qr/fine_offset_body_cm
```

## 比赛任务

比赛完整链路由 `my_launch` 启动：

```bash
ros2 launch my_launch demo1.launch.py
```

`demo1.launch.py` 会显式给二维码节点传入 `laser_pin:=10`，正式任务中激光由 `/qr/fire_laser` 控制。单独视觉测试默认不初始化 GPIO。
