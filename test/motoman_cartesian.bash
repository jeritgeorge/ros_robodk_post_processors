#!/bin/bash
rosservice call /robodk_post_processors/prog_start "post_processor: 'Motoman'
program_name: 'test_cartesian'
program_comment: ''"

rosservice call /robodk_post_processors/motoman/dont_use_mframe "value: false"

rosservice call /robodk_post_processors/set_frame "frame_id: 1
frame_name: 'user'
pose:
  position:
    x: 0.0
    y: 0.0
    z: 200.0
  orientation:
    x: 0.0
    y: 1.0
    z: 0.0
    w: 0.0"

rosservice call /robodk_post_processors/set_tool "tool_id: 1
tool_name: 'tool'
pose:
  position:
    x: 0.0
    y: 0.0
    z: 10.0
  orientation:
    x: 0.0
    y: 1.0
    z: 0.0
    w: 0.0"

rosservice call /robodk_post_processors/move_l "pose:
  position:
    x: 32.0
    y: -0.0
    z: -50.0
  orientation:
    x: 0.0
    y: 1.0
    z: 0.0
    w: 0.0
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

rosservice call /robodk_post_processors/move_l "pose:
  position:
    x: 32.0
    y: -0.0
    z: -50.0
  orientation:
    x: 0.0
    y: 1.0
    z: 0.0
    w: 0.0
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

rosservice call /robodk_post_processors/move_l "pose:
  position:
    x: 0.0
    y: -50.0
    z: 0.0
  orientation:
    x: 0.0
    y: 1.0
    z: 0.0
    w: 0.0
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

rosservice call /robodk_post_processors/move_l "pose:
  position:
    x: 20.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 1.0
    z: 0.0
    w: 0.0
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

rosservice call /robodk_post_processors/move_l "pose:
  position:
    x: 3.0
    y: 0.0
    z: 50.0
  orientation:
    x: 0.0
    y: 1.0
    z: 0.0
    w: 0.0
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

rosservice call /robodk_post_processors/prog_finish "program_name: 'test_cartesian'"

rosservice call /robodk_post_processors/prog_save "program_name: 'test_cartesian'
file_saving_dir: '/home/victor/'"
