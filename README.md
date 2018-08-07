[![Institut Maupertuis logo](http://www.institutmaupertuis.fr/media/gabarit/logo.png)](http://www.institutmaupertuis.fr)

[![build status](https://gitlab.com/InstitutMaupertuis/ros_robodk_post_processors/badges/melodic/build.svg)](https://gitlab.com/InstitutMaupertuis/ros_robodk_post_processors/commits/melodic)

# Testing
Launch the service servers node
```bash
roscore &
rosrun ros_robodk_post_processors services.py
```

Then generate a Motoman program:
```bash
#!/bin/bash
rosservice call /robodk_post_processors/prog_start "post_processor: 'Motoman'
program_name: 'VIC'
program_comment: ''"
rosservice call /robodk_post_processors/run_message "msg: 'Coucou Victor'"
rosservice call /robodk_post_processors/move_j "{}"
rosservice call /robodk_post_processors/move_l "{}"
rosservice call /robodk_post_processors/prog_finish "program_name: 'VIC'"
rosservice call /robodk_post_processors/generate_native_code "save_file: true
file_saving_dir: '/home/victor'"
```
