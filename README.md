# Save Manipulability

## Getting Started

Save manipulability of Panda

```sh
roslaunch panda_moveit_config demo.launch
rosrun save_manipulability recorder.py
```

<!-- rosrun save_manipulability record.py _group_name:=panda_arm -->

"right_arm"
"left_arm"
Save manipulability of Socialrobot

```sh
roslaunch social_robot_arm_moveit demo.launch

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
