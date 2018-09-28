#!/bin/bash
rosservice call /robodk_post_processors/set_tool "tool_id: 0
tool_name: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"

rosservice call /robodk_post_processors/set_frame "frame_id: 0
frame_name: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"

rosservice call /robodk_post_processors/move_c "pose_1:
  position: {x: 50.0, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
joints_1: [0, 0, 0, 0, 0, 0]
conf_RLF_1: [0, 0, 0]
pose_2:
  position: {x: 0.0, y: 50.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
joints_2: [0, 0, 0, 0, 0, 0]
conf_RLF_2: [0, 0, 0]"

rosservice call /robodk_post_processors/set_speed_joints "deg_sec: 10.0"

rosservice call /robodk_post_processors/move_j "pose:
  position:
    x: -50.0
    y: 2.0
    z: 100.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
joints:
- 0
- 0
- 0
- 0
- 0
- 0
conf_RLF:
- 0
- 0
- 0"

rosservice call /robodk_post_processors/set_zone_data "zone_mm: 2.0"

rosservice call /robodk_post_processors/set_speed "mm_sec: 200.0"

rosservice call /robodk_post_processors/move_l "pose:
  position:
    x: 32.0
    y: -0.0
    z: -50.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
joints:
- 0
- 0
- 0
- 0
- 0
- 0
conf_RLF:
- 0
- 0
- 0"

rosservice call /robodk_post_processors/run_message "msg: 'Coucou Victor'"

rosservice call /robodk_post_processors/pause "seconds: 1.5"

rosservice call /robodk_post_processors/set_do "io_var: '5'
io_value: true"

rosservice call /robodk_post_processors/wait_di "io_var: '5'
io_value: true
timeout_ms: 0"

rosservice call /robodk_post_processors/run_code "code: 'MY_FUNC'
is_function_call: false"
