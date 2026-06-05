# opencv01

Single-camera QR recognition and visual fine-tuning package.

## Nodes

- `qr_decoder_single`: the only QR recognition node. It defaults to `/dev/video0` and publishes `/qr/id`, `/qr/offset_norm`, `/qr/aligned`, and `/qr/debug_image`.
- `qr_decoder`: compatibility command that points to the same single-camera node.
- `qr_fine_tune`: subscribes to `/qr/offset_norm` and `/qr/aligned`, then publishes `/qr/fine_offset_body_cm` for route fine tuning.

## Flight Mode

Start the full mission stack with:

```bash
ros2 launch my_launch demo1.launch.py
```

`demo1.launch.py` sets `qr_task_active_required:=true`, so the QR node decodes only after the route state machine publishes `/qr_task_active=true`. During normal flight it will not publish `/qr/id`.

The laser is controlled by `/qr/fire_laser` in flight mode. After the drone reaches a QR target, aligns visually, and records or verifies the QR value, the route state machine turns pin10 on for 1 second.

## Standalone Camera And Laser Test

The laser backend uses the Orange Pi/WiringOP `gpio` command. Check it first:

```bash
gpio readall
```

Then run:

```bash
ros2 run opencv01 qr_decoder_single --ros-args \
  -p camera_device:=/dev/video0 \
  -p enable_gui:=true \
  -p qr_task_active_required:=false \
  -p laser_pin:=10 \
  -p laser_duration_sec:=1.0 \
  -p standalone_laser_once:=true
```

In standalone mode, after the QR code enters the alignment window, wPi pin10 turns on once for 1 second.

To view fine-tune output in another terminal:

```bash
ros2 run opencv01 qr_fine_tune
ros2 topic echo /qr/fine_offset_body_cm
```
