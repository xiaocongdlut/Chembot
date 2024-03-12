#! /bin/bash 
# 运行程序前确认机器人IP为：192.168.3.30
# 本机IP为：192.168.3.31 

# gnome-terminal -t "roscore" -x bash -c "source /opt/ros/melodic/setup.bash;roscore;"

sleep 1

# gnome-terminal -x bash -c "source ~/Mecbot_ws/devel/setup.bash; roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.3.30; exec bash;"


gnome-terminal -x bash -c "source ~/Mecbot_ws/devel/setup.bash; roslaunch mecbot_driver_pkg 04_mecbot_with_arm_setup.launch robot_ip:=192.168.3.30; exec bash;"

sleep 3

gnome-terminal -x bash -c "source ~/Mecbot_ws/devel/setup.bash; roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch; exec bash;"

sleep 3

 gnome-terminal -x bash -c "source ~/Mecbot_ws/devel/setup.bash; roslaunch ur5e_moveit_config moveit_rviz.launch config:=true;exec bash;"

sleep 3

# gnome-terminal -x bash -c "source ~/Mecbot_ws/devel/setup.bash;rosrun ur_robot_demo ur_3e_demo_loop; exec bash;"
# gnome-terminal -x bash -c "source ~/Mecbot_ws/devel/setup.bash;rosrun cmd_pkg keyboard_cmd; exec bash;"

