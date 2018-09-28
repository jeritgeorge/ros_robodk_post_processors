#!/bin/bash
rosservice call /robodk_post_processors/prog_start "post_processor: 'Motoman'
program_name: 'test'
program_comment: ''"

./common_commands.bash

rosservice call /robodk_post_processors/motoman/macro "number: 1
mf: 3
args:
- 1
- 2"

rosservice call /robodk_post_processors/motoman/arcon "asf_file: 1"
rosservice call /robodk_post_processors/motoman/arcof "aef_file: 0"

./finish_program.bash
