#!/usr/bin/env python
# encoding: utf-8

# public lib
import sys
import math
import random
from math import pi
from time import sleep
from Rosmaster_Lib import Rosmaster

# ros lib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField, JointState
from broverette_msgs.msg import Control 
from rclpy.clock import Clock


class Broverette_Driver(Node):
    def __init__(self, name):
        super().__init__(name)
        self.RA2DE = 180 / pi
        self.car = Rosmaster()
        self.car.set_car_type(5)

        # Get parameters
        self.declare_parameter('imu_link', 'imu_link')
        self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value

        self.declare_parameter('Prefix', "")
        self.Prefix = self.get_parameter('Prefix').get_parameter_value().string_value

        self.declare_parameter('xlinear_limit', 1.0)
        self.xlinear_limit = self.get_parameter('xlinear_limit').get_parameter_value().double_value

        self.declare_parameter('ylinear_limit', 1.0)
        self.ylinear_limit = self.get_parameter('ylinear_limit').get_parameter_value().double_value

        self.declare_parameter('angular_limit', 1.0)
        self.angular_limit = self.get_parameter('angular_limit').get_parameter_value().double_value

        # Create subscribers
        self.sub_cmd_vel = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 1)
        self.sub_control = self.create_subscription(Control, 'broverette_control', self.control_callback, 10)

        # Create publishers
        self.volPublisher = self.create_publisher(Float32, "voltage", 100)
        self.staPublisher = self.create_publisher(JointState, "joint_states", 100)
        self.velPublisher = self.create_publisher(Twist, "vel_raw", 50)
        self.imuPublisher = self.create_publisher(Imu, "imu/data_raw", 100)
        self.magPublisher = self.create_publisher(MagneticField, "imu/mag", 100)

        # Create timer
        self.timer = self.create_timer(0.1, self.pub_data)

        # Create and initialize variables
        self.car.create_receive_threading()

    # Callback function for velocity control
    def cmd_vel_callback(self, msg):
        if not isinstance(msg, Twist):
            return

        vx = msg.linear.x * 1.0
        vy = msg.linear.y * 1.0
        angular = msg.angular.z * 1.0
        self.car.set_car_motion(vx, vy, angular)


    def control_callback(self, msg):
        """
        Handle incoming control messages from the broverette topic (Control.msg).
        Map these messages to the Rosmaster's car motion control using set_motor and set_pwm_servo.
        """
        # Define constants
        max_speed = 100  # Max speed for forward and reverse [-100, 100]
        creep_speed = 0  # A small constant speed for creep
        throttle_threshold = creep_speed / max_speed  # Throttle input threshold for creep
        deceleration_factor = 0.98  # Factor to simulate coasting
        brake_deceleration = 10  # Speed reduction when brake is applied
        steering_min = 135  # Inverted: Servo angle for full right
        steering_max = 45   # Inverted: Servo angle for full left
        steering_center = 90  # Servo angle for straight steering

        # Set initial state
        speed = 0  # Speed for forward/reverse
        state = 0  # 0 = stop, 1 = forward, 2 = backward
        servo_angle = steering_center  # Default steering angle (center)

        # Log the current vx speed from get_motion_data()
        current_vx, current_vy, current_angular = self.car.get_motion_data()
        # self.get_logger().info(f"Current vx (forward speed): {current_vx}")

        # Apply brake regardless of throttle input
        if msg.brake > 0:
            speed = max(0, current_vx - brake_deceleration)  # Gradually reduce speed when brake is pressed
        elif msg.shift_gears == Control.FORWARD:
            # Forward movement
            state = 1  # Set car to forward
            if msg.throttle >= throttle_threshold:
                # Throttle controls forward speed only if above creep threshold
                speed = min(msg.throttle * max_speed, max_speed)
            else:
                # If no input, gradually slow down to creep speed
                speed = max(creep_speed, current_vx * deceleration_factor)

        elif msg.shift_gears == Control.REVERSE:
            # Reverse movement
            state = 2  # Set car to reverse
            if msg.throttle >= throttle_threshold:
                speed = -min(msg.throttle * max_speed, max_speed)  # Negative speed for reverse
            else:
                speed = -creep_speed  # Apply reverse creep speed

        # Handle steering based on the steering input from the wheel (map -1 to 1 to 135° to 45°)
        if -1 <= msg.steer <= 1:
            # Invert the mapping: -1 (right) -> 135° and 1 (left) -> 45°
            servo_angle = ((msg.steer + 1) / 2) * (steering_max - steering_min) + steering_min
        else:
            # Fallback to center if invalid steering input
            servo_angle = steering_center

        # Apply car movement using set_motor for motor 2 and 4 (motors 1 and 3 are 0)
        self.car.set_motor(0, int(speed), 0, int(speed))  # Speed for motor 2 and 4, others 0

        # Apply steering using servo control
        self.car.set_pwm_servo(1, int(servo_angle))  # Servo angle should be integer


    # Publish data
    def pub_data(self):
        time_stamp = Clock().now()
        imu = Imu()
        twist = Twist()
        battery = Float32()
        mag = MagneticField()
        state = JointState()
        state.header.stamp = time_stamp.to_msg()
        state.header.frame_id = "joint_states"

        if len(self.Prefix) == 0:
            state.name = ["back_right_joint", "back_left_joint", "front_left_steer_joint", "front_left_wheel_joint",
                          "front_right_steer_joint", "front_right_wheel_joint"]
        else:
            state.name = [self.Prefix + "back_right_joint", self.Prefix + "back_left_joint", self.Prefix + "front_left_steer_joint",
                          self.Prefix + "front_left_wheel_joint", self.Prefix + "front_right_steer_joint", self.Prefix + "front_right_wheel_joint"]

        battery.data = self.car.get_battery_voltage() * 1.0
        ax, ay, az = self.car.get_accelerometer_data()
        gx, gy, gz = self.car.get_gyroscope_data()
        mx, my, mz = self.car.get_magnetometer_data()
        vx, vy, angular = self.car.get_motion_data()

        # Publish IMU data
        imu.header.stamp = time_stamp.to_msg()
        imu.header.frame_id = self.imu_link
        imu.linear_acceleration.x = ax * 1.0
        imu.linear_acceleration.y = ay * 1.0
        imu.linear_acceleration.z = az * 1.0
        imu.angular_velocity.x = gx * 1.0
        imu.angular_velocity.y = gy * 1.0
        imu.angular_velocity.z = gz * 1.0

        mag.header.stamp = time_stamp.to_msg()
        mag.header.frame_id = self.imu_link
        mag.magnetic_field.x = mx * 1.0
        mag.magnetic_field.y = my * 1.0
        mag.magnetic_field.z = mz * 1.0

        # Publish velocity
        twist.linear.x = vx * 1.0
        twist.linear.y = vy * 1000 * 1.0
        twist.angular.z = angular * 1.0
        self.velPublisher.publish(twist)
        self.imuPublisher.publish(imu)
        self.magPublisher.publish(mag)
        self.volPublisher.publish(battery)

        steer_radis = vy * 1000.0 * pi / 180.0
        state.position = [0.0, 0.0, steer_radis, 0.0, steer_radis, 0.0]
        if not vx == angular == 0:
            i = random.uniform(-3.14, 3.14)
            state.position = [i, i, steer_radis, i, steer_radis, i]
        self.staPublisher.publish(state)


def main():
    rclpy.init()
    driver = Broverette_Driver('driver_node')
    rclpy.spin(driver)


if __name__ == '__main__':
    main()

