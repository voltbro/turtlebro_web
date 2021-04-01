#! /usr/bin/env python
from __future__ import print_function

import sys
import rospy
import time

from sensor_msgs.msg import CompressedImage
from turtlebro_web.srv import GetImage, GetImageResponse

class ImagesServer(object):

    def __init__(self):
        rospy.loginfo("Run: Images Server")
        s = rospy.Service('get_images', GetImage, self.handle_request)

        # rospy.Subscriber("/front_camera/compressed", CompressedImage, self.odom_cb)

    def getImage(self):

        msg = rospy.wait_for_message("/front_camera/compressed", CompressedImage)
        return msg.data   

    def handle_request(self, req):

        file_data = self.getImage()

        file_name = "images_%d.jpg" % int(time.time())
        rospy.loginfo("Save file: %s", file_name)
        # file = open('/var/www/html/images/'+file_name, 'w')
        file = open('/opt/ros/melodic/share/turtlebro_web/web/static/images/'+file_name, 'w')
        file.write(file_data)
        file.close()

        return GetImageResponse(file_name)

    def shutdown(self):
        rospy.loginfo("Stopping the telemetry node...")     


if __name__ == '__main__':   

    rospy.init_node('image_server')
    client = ImagesServer()
    rospy.spin()
