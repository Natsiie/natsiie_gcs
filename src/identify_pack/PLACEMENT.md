# Placement Progress

`place_task` manages placement independently of `PickTask`. Numbered ring centers
are ordered R1/R2/R3; the configured camera view is left/middle/right. A task code
such as `156+231+516+312` maps each color to the corresponding ring in the same
three-digit group. Placement follows the color order, not the spatial ring order.

## Configuration And Startup

Edit `config/place_task.yaml`, rebuild with `colcon build --packages-select identify_pack`,
and source `install/setup.bash`. Run alongside the existing camera, image-time TF,
and single serial bridge:

```bash
ros2 launch identify_pack place_task.launch.py
ros2 topic echo /arm/place_status --qos-durability transient_local
```

`start_detectors:=false` reuses already running detectors. Both detectors publish
stamped `sensor_msgs/PointCloud` observations at `/ring_station/centers_pixel` and
`/stacked_ring/centers_pixel`. Their existing correction strings are preserved.
Ring IDs and image resolution accompany each point; stability and alignment are
separate channels. Failed detection publishes an empty observation. The placement
node requires fresh observations after stage entry and each completed command,
three stable rings, alignment by default, and a TF at the acquisition time.

`enabled` defaults to false. Heights are relative to `gimbal_base` in mm:

- `stacked_ring.plane_z_mm=-72.5` is the confirmed ground/ring plane. It stays at
  ground level when projecting the ring center, even when placing the second layer.
- `item_height_mm=60` gives the increment between layers.
- `first_release_z_mm` is the gripper control point Z when the first material
  rests on the ground and the gripper is about to open. Measure this separately:
  ground Z plus the vertical distance from material bottom to gripper control point.
  It is not automatically equal to ground Z, material top Z, or motor encoder units.
- Configure the ordinary-workstation plane and first release height separately.
  Set each required `height_configured=true` and `enabled=true` only after filling
  these values. Parameters are read at startup and cannot be changed during a command.

Default phase order is ordinary batch 1, stacking batch 1, ordinary batch 2,
stacking batch 2. The stacking area records the first batch on the ground and the
second batch on top, per ring. Ordinary placements do not change stacking counts.
For stacking-only operation, set `phase_order: [stacked_ring:1, stacked_ring:2]`.
Each phase requires an explicit start after the robot reaches the workstation and
the batch/material supply is ready. This node does not navigate, retrieve materials
from the ordinary workstation, or choose material-platform motor positions.

## ROS Commands And Confirmation

All `/arm/place_*` messages use `std_msgs/String` JSON. Get `task_id` from
`/arm/place_status`; it is a generated UUID, not the QR text. The node also recovers
the QR text from latched `/arm/task_status` when started after the camera has decoded it.

Start the next expected phase through `/arm/place_control`:

```json
{"action":"start","task_id":"<32-character task ID>","stage":"ring_station","batch":1}
```

Each `/arm/place_command` contains `task_id`, `command_id`, `stage`, `batch`,
`sequence`, `color`, `ring`, `layer`, `frame_id`, `x_mm`, `y_mm`, `z_mm`.
Only one command may be pending. Coordinates are locked when issuing it; further
images cannot change the target. The checkpoint is written before publishing.

After release and retreat, the actuator controller returns `/arm/place_result`:

```json
{"task_id":"<32-character task ID>","command_id":1,"success":true}
```

Only a matching successful result advances placement and stacking counts. Wrong
task/command IDs and duplicate completion feedback cannot advance progress. Failure,
transport failure, or a 30-second confirmation timeout retains the pending target
and enters `ERROR`. A late matching successful confirmation can still reconcile it.
After verifying the action status, retry the retained command through control:

```json
{"action":"retry","task_id":"<32-character task ID>","command_id":1}
```

Retry uses the same command ID and coordinates. The executor must deduplicate IDs.
Progress survives ROS restart in `place_progress.json`. An unconfirmed command
restores as `ERROR`, never an automatic resend. Restarting between placements
requires explicitly starting the unfinished phase again. Archive the checkpoint after a
completed run before a new task; do not reset it while an action is executing.

## Serial And STM32 Integration

The existing single serial bridge transports commands and completion feedback:

| Type | Direction | ASCII Payload |
|---|---|---|
| `0x0A` | ROS to STM32 | `task_id,command_id,stage,batch,sequence,color,ring,layer,x_mm,y_mm,z_mm` |
| `0x0B` | STM32 to ROS | `task_id,command_id,success` |

Wire stage is `1` for ordinary workstation, `2` for stacking; success is `0/1`.
The wire base frame is the serial bridge `target_frame`, normally `gimbal_base`.
Frames retain the existing FF/type/length/payload/XOR/FE format. Firmware receive
capacity is increased to 128 bytes, and oversized frames are rejected.

`Serial.c` now parses and retains the numbered target without driving motors.
The actuator state machine must call `Serial_TakePlaceCommand(&command)` when ready
to execute, then select the material, move to XYZ, release, retreat, and call
`Serial_CompletePlaceCommand(command.task_id, command.command_id, success)`.
Call both from the task that calls `Serial_ProcessFrame`, not from an ISR.
In-flight duplicates are ignored; successful duplicates repeat the result without
re-execution; failed commands may be retried. Deduplication lives in MCU RAM:
after an MCU power loss, reconcile physical placement before requesting a retry.

The existing `Kin_GotoXY` controls grabbing at a fixed Z and uses a ground-origin
coordinate convention. It is not connected to this placement mailbox. An actuator
adapter needs calibrated Z conversion and a release/retreat sequence before physical
placement can execute. No firmware has been flashed and no hardware movement tested.

## Validation

Tests cover ring-number mapping and projection, phase sequencing, both stacking
layers, wrong/duplicate feedback, failure/retry/timeout, restart recovery, stale
observations, checkpoint write failure, serial transport, and the real firmware
parser/deduplication/result frames compiled on the host with peripheral stubs.
