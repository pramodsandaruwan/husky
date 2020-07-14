#!/usr/bin/env python

import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class odometry():
    def __init__(self):
        rospy.loginfo("Setting Up the Node...")
        
        rospy.init_node('odometry_publisher')

        #self._velocity_msg    = Float32MultiArray()
        self._odom_msg        = Odometry()
        self._gps_msg         = NavSatFix()

        #--- Create the odom array publisher
        self.odom_pub         = rospy.Publisher("scooby_velocity_controller/odom", Odometry, queue_size=50)
        self.gps_pub          = rospy.Publisher("gps/fix", NavSatFix, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

        rospy.loginfo("> Odom Publisher corrrectly initialized")

        #--- Create wheel velocity subscriber
        self.ros_sub_gps      = rospy.Subscriber("android/fix", NavSatFix, self.send_gps)
        self.ros_sub_wheel    = rospy.Subscriber("wheel_velocity", Float32MultiArray, self.callback)

        rospy.loginfo("> Odom Subscriber corrrectly initialized")

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0
        self.vy = 0
        self.vth = 0

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        rospy.loginfo("Initialization complete")

    def send_gps(self,gps_data):
        self._gps_msg = gps_data
        self._gps_msg.header.frame_id = 'gps_link'
        self.gps_pub .publish(self._gps_msg)

    def send_odom(self):
        # next, we'll publish the odometry message over ROS
        self._odom_msg.header.stamp = self.current_time
        self._odom_msg.header.frame_id = "odom"

        # set the position
        self._odom_msg.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*self.odom_quat))

        # set the velocity
        self._odom_msg.child_frame_id = "base_link"
        self._odom_msg.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

        # publish the message
        self.odom_pub.publish(self._odom_msg)

    def callback(self,value):
        self.vx  = (value.data[0] + value.data[1])/2.0
        self.vy  = 0.0
        self.vth = (value.data[1] - value.data[0])/value.data[2]

    @property
    def run(self):

        #--- Set the control rate
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            # compute odometry in a typical way given the velocities of the robot
            dt = (self.current_time - self.last_time).to_sec()
            delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
            delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
            delta_th = self.vth * dt

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            # since all odometry is 6DOF we'll need a quaternion created from yaw
            self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

            # first, we'll publish the transform over tf
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                self.odom_quat,
                self.current_time,
                "base_link",
                "odom"
            )

            self.send_odom()
            self.last_time = self.current_time
            rate.sleep()


if __name__ == "__main__":
    odometry_publisher     = odometry()
    odometry_publisher.run()

