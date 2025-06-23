A simple drone simulator (Skydio X2) for ROS 2 in MuJoCo.

dependencies:
```sh
pip3 install --break-system-packages mujoco robot_descriptions
```

start:
```sh
ros2 run mujoco_drone_simulator drone_sim
```

publisher:
- `/mujoco_drone_simulator/imu` (`sensor_msgs/msg/Imu`)

subscriber:
- `/mujoco_drone_simulator/actuators` (`mavros_msgs/msg/ActuatorControl`)
