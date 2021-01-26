# Save Manipulability

## Getting Started

Save manipulability of Panda

```sh
roslaunch panda_moveit_config demo.launch
---
rosrun save_manipulability recorder.py
```

<!-- rosrun save_manipulability record.py _group_name:=panda_arm -->

"right_arm"
"left_arm"
Save manipulability of Socialrobot

```sh
# roslaunch social_robot_arm_moveit default.launch
# roslaunch social_robot_arm_moveit demo.launch

```

```sh
roscore
sudo service mongodb stop
vrep
roslaunch socialrobot_demo init-sim.launch sim_env:=demo_empty
roslaunch robocare_navigation social_robot_navigation.launch scan_topic:=base_scan
---
rosrun save_manipulability recorder.py
```

<!-- rviz_visual_tools -->

rosrun social_robot_arm_moveit execute_trajectory.py

Reference

```sh
roslaunch panda_moveit_config demo.launch
rosrun moveit_tutorials move_group_python_interface_tutorial.py
```

## Output CSV

<!-- # euler = tfm.euler_from_quaternion(quat, axes="rzxy")  # z, x, y -->

1st row

1. Base link name
2. End effector link name

2nd~ rows

1. X
2. Y
3. Z
4. Roll(X)
5. Pitch(Y)
6. Yaw(Z) = approach angle(Cr)
7. Manipulability

---

지금 저장한 포멧

1. X
2. Y
3. Z
4. approach angle(Cr)
5. Joint 1
6. Joint 2
7. Joint 3
8. Joint 4
9. Joint 5
10. Joint 6
11. Joint 7
12. manipulability
