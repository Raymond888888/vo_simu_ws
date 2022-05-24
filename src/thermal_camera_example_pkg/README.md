# thermal_gazebo_example

### <Raymond Lau ,2583701681@qq.com>,for test
## manual:

add the project pkg folder "thermal_camera_example_pkg" at ~/catkin_ws/src

shell do: 

`catkin_make`
`ign gazebo -r examples/worlds/car_robot.sdf  (ros2)` 

会报错: `[GUI] [Err] [Model.hh:73] Unable to unserialize sdf::Model` 等等   ，但是根据 https://github.com/osrf/mbzirc/wiki/Troubleshooting 所说，不影响运行。

`ign topic -e -t /keyboard/keypress   (ros2)` 




`roscore`
`rosrun thermal_camera_example_pkg thermal_camera_example` 




# 安装

- https://gazebosim.org/docs/fortress/install_ubuntu 通过apt安装ignitiongazebo

- git clone https://github.com/ctu-mrs/mrs_gazebo_extras_resources.git 

- 运行 .ci/build.sh 和.ci/install.sh脚本，安装功能包到工作空间

- git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git 安装后在gazebo_link_attacher_ws 运行，发布了 `/link_attacher_node/attach`    `/link_attacher_node/detach`   两个service







## 各种库的功能指南

https://github.com/gazebosim/ros_gz.git    ign gazebo for ros

## sources 
car_robot : https://www.ncnynl.com/archives/202201/4926.html


## 调试日志


2022.5.20
运行成功，实现了： @todo

2022.5.24
将功能包迁移到vio_thermal_simu_ws包里，运行状态为：

(noetic)roscore
(noetic)ign gazebo -r src/thermal_camera_example_pkg/examples/worlds/car_robot.sdf
(noetic)rosrun thermal_camera_example_pkg thermal_camera_example
(noetic)rosrun thermal_camera_example_pkg orb_cv
(noetic)rosrun teleop_twist_keyboard teleop_twist_keyboard.py

此状态开始时图像没有任何延迟，一段时间后开始有明显延迟，此bug待解决