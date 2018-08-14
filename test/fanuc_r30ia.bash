#!/bin/bash
rosservice call /robodk_post_processors/prog_start "post_processor: 'Fanuc_R30iA'
program_name: 'test'
program_comment: ''"

./test_pp.bash
