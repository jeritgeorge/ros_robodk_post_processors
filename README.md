[![Institut Maupertuis logo](http://www.institutmaupertuis.fr/media/gabarit/logo.png)](http://www.institutmaupertuis.fr)

[![build status](https://gitlab.com/InstitutMaupertuis/ros_robodk_post_processors/badges/melodic/build.svg)](https://gitlab.com/InstitutMaupertuis/ros_robodk_post_processors/commits/melodic)

# Overview
This project allows to generate programs for industrial robots by using ROS services.
It is a superset of the [InstitutMaupertuis/robodk_postprocessors](https://github.com/InstitutMaupertuis/robodk_postprocessors) project.

# Dependencies

## rosdep
Install, initialize and update [rosdep](https://wiki.ros.org/rosdep).

# Compiling
Create a catkin workspace and clone the project:

```bash
mkdir -p catkin_workspace/src
cd catkin_workspace/src
git clone --recurse-submodules https://gitlab.com/InstitutMaupertuis/ros_robodk_post_processors.git
cd ..
```

## Resolve ROS dependencies
```bash
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

## Compile
```bash
catkin_make
```

# Testing
```bash
catkin_make run_tests
```
