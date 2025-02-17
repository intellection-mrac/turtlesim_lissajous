#!/usr/bin/env python3
"""
lissajous_drawer.py

This ROS node commands turtlesim to draw a Lissajous curve defined by:
    x(t) = center_x + A * sin(a * t + delta)
    y(t) = center_y + B * sin(b * t)

Derivatives:
    dx/dt = A * a * cos(a * t + delta)
    dy/dt = B * b * cos(b * t)

The instantaneous linear velocity is computed as:
    v = sqrt((dx/dt)^2 + (dy/dt)^2)
and the desired heading is computed using:
    theta = arctan2(dy/dt, dx/dt)

A proportional controller adjusts the angular velocity so that the turtle
matches the tangent direction of the curve.

Author: intellection, o3-mini
References:
    - Lissajous, J. A. (1857). Mémoire sur l'étude optique des mouvements vibratoires.
    - Weisstein, Eric W. "Lissajous Curve", MathWorld.
"""

import rospy
from geometry_msgs.msg import Twist
import math

def lissajous_drawer():
    # Initialize the ROS node
    rospy.init_node('lissajous_drawer', anonymous=True)
    vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(30)  # 30 Hz update frequency for smooth control

    # Lissajous curve parameters
    A = 4.0              # Amplitude along the x-axis
    B = 4.0              # Amplitude along the y-axis
    a = 3.0              # Angular frequency for the x component
    b = 4.0              # Angular frequency for the y component
    delta = math.pi / 2  # Phase shift in radians

    # Duration for drawing (in seconds)
    duration = 300.0

    # Turtlesim grid center (typically an 11x11 grid)
    center_x = 5.5
    center_y = 5.5

    start_time = rospy.get_time()
    vel_msg = Twist()

    while not rospy.is_shutdown():
        t = rospy.get_time() - start_time
        if t > duration:
            break

        # Calculate the derivatives
        dx_dt = A * a * math.cos(a * t + delta)
        dy_dt = B * b * math.cos(b * t)

        # Compute the linear velocity (Euclidean norm)
        linear_velocity = math.sqrt(dx_dt**2 + dy_dt**2)

        # Determine the desired heading using arctan2 (to correctly handle quadrants)
        desired_heading = math.atan2(dy_dt, dx_dt)

        # Proportional controller for angular velocity
        k_angular = 4.0  # Controller gain
        angular_velocity = k_angular * desired_heading

        # Publish velocity commands (adjust linear velocity scaling as needed)
        vel_msg.linear.x = linear_velocity * 0.1
        vel_msg.angular.z = angular_velocity
        vel_pub.publish(vel_msg)

        rate.sleep()

    # Stop the turtle after drawing the curve
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    vel_pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        lissajous_drawer()
    except rospy.ROSInterruptException:
        pass
