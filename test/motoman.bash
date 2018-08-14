#!/bin/bash
rosservice call /robodk_post_processors/prog_start "post_processor: 'Motoman'
program_name: 'VIC'
program_comment: ''"

rosservice call /robodk_post_processors/prog_start "post_processor: 'Fanuc_R30iA'
program_name: 'VIC'
program_comment: ''"

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

rosservice call /robodk_post_processors/run_message "msg: 'Coucou Victor'"

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

rosservice call /robodk_post_processors/pause "seconds: 1.5"

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

rosservice call /robodk_post_processors/prog_finish "program_name: 'VIC'"

rosservice call /robodk_post_processors/generate_native_code "save_file: true
file_saving_dir: '/home/victor'"
