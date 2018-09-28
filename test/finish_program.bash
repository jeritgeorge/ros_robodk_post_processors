#!/bin/bash
rosservice call /robodk_post_processors/prog_finish "program_name: 'test'"

rosservice call /robodk_post_processors/prog_save "program_name: 'test'
file_saving_dir: '/home/victor/Téléchargements/RoboDK_pp/'"
