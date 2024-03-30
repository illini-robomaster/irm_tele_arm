## Usage:

To install:
Install ROS2 humble

To compile:
```
colcon build
source ./install/setup.bash
```

To run:
```
# Temporal Robotics Arm
ros2 launch moveit_servo demo_ros_api.launch.py

# Spacemouse to Twist converter
ros2 run irm_tele_arm_utils twist2twiststamped
or:
ros2 run moveit_servo servo_keyboard_input
```
