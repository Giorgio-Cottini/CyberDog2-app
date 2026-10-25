# CyberDog 2 PC controller

Local web app to drive the dog from Windows over the authenticated gRPC
channel. No app, no Xiaomi account.

## Run

```
powershell -File app\run.ps1
```

Then open http://127.0.0.1:8793/ . The dog must be on the PC hotspot at
`192.168.137.115` (set `DOG_IP` to override).

## What works (verified against the dog)

- **Posture** (one-shot `MOTION_CMD_REQUEST` 1004): Stand up (`111`),
  Lie down (`101`), Standing posture (`112`), Emergency stop (`0`).
- **Movement** (streamed `MOTION_SERVO_REQUEST` 1002, gated behind "Arm
  movement", OFF by default): W/S forward-back, A/D turn, Q/E strafe.
- **Audio**: mute/unmute actions (1014), volume (1012).
- **Telemetry**: battery %, posture, charging, temperature, polled over 1001.

## Kinematics (why movement is a stream, not a button)

Pose changes are single service calls (1004). Walking is different: the dog
expects a continuous servo stream on topic `motion_servo_cmd` (nameCode 1002):

- `cmd_type` 1 = Data frame, 2 = End frame (sent once on key release).
- `cmd_source` 0.
- `motion_id` 303 = WALK_USERTROT (user trot gait).
- `vel_des` = [x, y, yaw]; limits x,y <= 1.5 m/s, yaw <= 2.0 rad/s.
- `step_height` = [front, back] m, recommended [0.05, 0.05].

`server.py` streams Data frames at 50 Hz with gentle defaults (0.2 m/s,
0.4 rad/s), clamps to the limits, and sends one End frame when all keys are
released. A single 1004 cannot produce continuous walking.

Source for these values: Xiaomi `cyberdog_vp_abilityset/src/sdk/motion.cpp`
and `protocol/ros/msg/MotionID.msg`.

## Video: live snapshot view (works now)

The "Start live view" button shows what the dog sees at ~1 frame/sec. This is
done entirely over the forward channel, no cert and no root needed:

1. `START_LIVE_STREAM` (nameCode 4002, command 7) puts the camera in a capture
   state. Without this, `TAKE_PICTURE` returns `RESULT_INVALID_STATE` (5).
2. `TAKE_PICTURE` (4002 command 1) over `getFile` returns a full-resolution
   JPEG (4208x3120, ~3.8 MB) as a `FileChunk` stream.
3. The UI polls `/snapshot` and swaps the image. Refresh is ~1 fps because the
   gRPC camera path does not accept width/height, so frames are always full
   resolution.

Caveat: each `TAKE_PICTURE` writes the JPEG to `/home/mi/Camera/` on the dog,
and the gRPC path can't pass a filename to `DELETE_FILE`, so frames accumulate
on the dog while live view runs. Clear `/home/mi/Camera/` from the dog if it
fills up. Backend endpoints: `/snapshot` (GET), and `cam_start` / `cam_stop`
actions on `/cmd`.

## True live video (WebRTC): still blocked

Smooth real-time video would use WebRTC. The browser builds an offer and the
dog accepts it on the forward channel, but the dog returns its SDP answer only
over a **reverse** channel it opens back to the PC on `:8981`, verifying our
cert against the name **`cyberdog2.client`**. We only have `cyberdog2.server`
and no CA key to mint a client cert, so that handshake is rejected. The
snapshot view above is the working substitute. Set `DOG_REVERSE=1` only once a
`cyberdog2.client` cert exists in `certs/`.

## Safety

Movement is gated server-side (`safety_on`, default off). The page forces a
disarm on every load so a stale armed state cannot outlive a reload. Stand and
lie buttons move the dog immediately; the camera and telemetry do not.

## Files

- `server.py` - gRPC bridge + HTTP/SSE server.
- `ui.html` - fullscreen video + collapsible control panel.
- `run.ps1` - launcher.

Depends only on the vendored `pylibs` (grpcio) and the certs in `../certs`.
