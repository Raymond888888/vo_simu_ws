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
再次观察发现igintion gazebo 也会卡，怀疑是igintion gazebo 本身的问题，因为imshow的原始图像和处理后的图像一致，延迟问题应该是出在订阅igntopic的图像接收阶段

2022.5.27
实现了用ignitiongazebo运行自己定义的world，下一步只需要丰富世界元素即可。
roslaunch mm_gazebo ls_bringup.launch world_name:=`rospack find mm_gazebo`/worlds/depot/depot.world  model_pose:="-x 2 -y 2 -z 0.15" teleop_base:=false 
TODO：撼地者无法出现在ign里面，待解决

tudo:只需要解决用ign rosrun ros_ign_gazebo ign_gazebo 运行ls.xacro文件，就可以解决上述问题

2022.5.28
milestone:可以实现在新环境下，ls搭载热像仪实现图像传输(ppp1.png)
TODO： 
add imu
丰富环境
键盘移动

2022.5.28 17:00
done :键盘移动
TODO：
add imu
丰富环境
bug:摄像头和imu的collision不能撞到东西，否则会开始pitch轴滚转，停不下来




$ ign topic -e -t /keyboard/keypress
ign topic -e -t /keyboard/keypress
roslaunch mm_gazebo ls_bringup.launch world_name:=`rospack find mm_gazebo`/worlds/depot/depot.world  model_pose:="-x 2 -y 2 -z 0.15" teleop_base:=false 
