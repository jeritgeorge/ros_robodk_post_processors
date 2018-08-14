[![Institut Maupertuis logo](http://www.institutmaupertuis.fr/media/gabarit/logo.png)](http://www.institutmaupertuis.fr)

[![build status](https://gitlab.com/InstitutMaupertuis/ros_robodk_post_processors/badges/melodic/build.svg)](https://gitlab.com/InstitutMaupertuis/ros_robodk_post_processors/commits/melodic)

# Initializing submodules
```
cd ros_robodk_post_processors
git submodule init
git submodule update
```

# Testing
Launch the service servers node
```bash
roscore &
rosrun ros_robodk_post_processors services.py
```

Example bash scripts to generate programs: [test](./test)
