#! /usr/bin/env python

import rospy

from math import degrees
from turtlebro_web.msg import WebTelemetry
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState

from tf.transformations import euler_from_quaternion

class Telemetry(object):

    def __init__(self):

        rospy.init_node('web_telemetry_node', log_level=rospy.INFO)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("Start Web Telemetry server")

        self.odom = Odometry()
        self.imu = Imu()
        self.bat = BatteryState()
        
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("/imu", Imu, self.imu_cb)
        rospy.Subscriber("/bat", BatteryState, self.bat_cb)

    def odom_cb(self,msg):
        self.odom = msg

    def imu_cb(self,msg):
        self.imu = msg

    def bat_cb(self,msg):
        self.bat = msg        
    

    def pub_telemetry(self):

        rate = rospy.Rate(rospy.get_param('~rate', 2))
        tele_pub = rospy.Publisher('/web_tele', WebTelemetry, queue_size=1)

        while not rospy.is_shutdown():

            tele = WebTelemetry()
            orientation = self.imu.orientation

            (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

            tele.odom_x = self.odom.pose.pose.position.x
            tele.odom_y = self.odom.pose.pose.position.y
            tele.odom_z = self.odom.pose.pose.position.z

            tele.imu_yaw = int(degrees(yaw))
            tele.imu_pitch = int(degrees(pitch))
            tele.imu_roll = int(degrees(roll))

            tele.bat_voltage = self.bat.voltage

            tele.odom_lin_speed = self.odom.twist.twist.linear.x
            tele.odom_ang_speed = int(degrees(self.odom.twist.twist.angular.z))

            tele_pub.publish(tele)

            rate.sleep()
    

    def shutdown(self):
        rospy.loginfo("Stopping the telemetry node...")   

if __name__ == '__main__':
    server = Telemetry()
    server.pub_telemetry()

    rospy.spin()         