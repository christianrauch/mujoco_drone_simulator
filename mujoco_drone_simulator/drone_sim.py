#!/usr/bin/env python3
import mujoco
import mujoco.viewer
from robot_descriptions.loaders.mujoco import load_robot_description
import rclpy
import rclpy.node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu
from mavros_msgs.msg import ActuatorControl
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import threading


def main(args=None):
    rclpy.init(args=args)

    exec = SingleThreadedExecutor()
    node = rclpy.node.Node("mujoco_drone_simulator")
    exec.add_node(node)

    exec_thread = threading.Thread(target=exec.spin, daemon=True)
    exec_thread.start()

    pub_imu = node.create_publisher(Imu, "~/imu", 1)
    tf_broadcaster = TransformBroadcaster(node)

    model_name = "skydio_x2"
    m = load_robot_description(f"{model_name}_mj_description", variant="scene")
    d = mujoco.MjData(m)


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

    def tf_msgs(m, d) -> TransformStamped:
        transforms = list()
        for i in range(m.nbody):
            body_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)
            if body_name:
                transform_stamped = TransformStamped()
                transform_stamped.header.stamp = rclpy.time.Time(seconds=d.time).to_msg()
                transform_stamped.header.frame_id = "world"
                transform_stamped.child_frame_id = "mujoco/"+body_name

                transform_stamped.transform.translation.x = d.xpos[i][0]
                transform_stamped.transform.translation.y = d.xpos[i][1]
                transform_stamped.transform.translation.z = d.xpos[i][2]
                transform_stamped.transform.rotation.w = d.xquat[i][0]
                transform_stamped.transform.rotation.x = d.xquat[i][1]
                transform_stamped.transform.rotation.y = d.xquat[i][2]
                transform_stamped.transform.rotation.z = d.xquat[i][3]
                transforms.append(transform_stamped)

        return transforms

    def on_actuators(msg):
        for i, v in enumerate(msg.controls):
            name = f"thrust{i+1}"
            id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if id >= 0:
                ctrlmin = m.actuator_ctrlrange[id][0]
                ctrlmax = m.actuator_ctrlrange[id][1]
                # clip to range [0, 1] and scale to min/max control range
                d.actuator(name).ctrl = ctrlmin + min(max(0,v),1) * (ctrlmax - ctrlmin)

    sub_pwm = node.create_subscription(ActuatorControl, "~/actuators", on_actuators, 1)

    rate = node.create_rate(500)

    try:
        with mujoco.viewer.launch_passive(m, d) as viewer:
            while viewer.is_running():
                mujoco.mj_step(m, d)
                viewer.sync()

                pub_imu.publish(imu_msg(d))
                tf_broadcaster.sendTransform(tf_msgs(m, d))
                rate.sleep()

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
