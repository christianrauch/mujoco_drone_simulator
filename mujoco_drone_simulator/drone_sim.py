#!/usr/bin/env python3
import mujoco
import mujoco.viewer
from robot_descriptions.loaders.mujoco import load_robot_description
import rclpy
import rclpy.node
from sensor_msgs.msg import Imu


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.node.Node("mujoco_drone_simulator")

    pub_imu = node.create_publisher(Imu, "~/imu", 1)

    model_name = "skydio_x2"
    m = load_robot_description(f"{model_name}_mj_description", variant="scene")

    sensor_names = dict()
    for i, t in enumerate(m.sensor_type):
        sensor_names[t] = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SENSOR, i)

    def imu_msg(model_data) -> Imu:
        imu_msg = Imu()
        imu_msg.header.stamp = rclpy.time.Time(seconds=d.time).to_msg()
        imu_msg.header.frame_id = model_name
        (imu_msg.orientation.w, imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z) = \
            model_data.sensor(sensor_names[mujoco.mjtSensor.mjSENS_FRAMEQUAT]).data
        (imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z) = \
            model_data.sensor(sensor_names[mujoco.mjtSensor.mjSENS_GYRO]).data
        (imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z) = \
            model_data.sensor(sensor_names[mujoco.mjtSensor.mjSENS_ACCELEROMETER]).data
        return imu_msg

    d = mujoco.MjData(m)

    try:
        with mujoco.viewer.launch_passive(m, d) as viewer:
            while viewer.is_running():
                mujoco.mj_step(m, d)
                viewer.sync()

                pub_imu.publish(imu_msg(d))

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
