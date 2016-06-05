#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

def callback(data):
    rospy.loginfo("%f %f %f",data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z)

def listener():
    #In ROS, nodes are uniquely named. If two nodes with teh same name are launched, the previous
    #one is kicked off. the anonymous=True flag means that rospy will choose a unique name for our 'listener'
    #node so that multiple listeners can run simultaneously

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/imu/data",Imu,callback)
    rospy.loginfo("Running")

    now=rospy.get_rostime()




    r=rospy.Rate(0.1)
    while not rospy.is_shutdown():
        lasttime=now
        now=rospy.get_rostime()
        rospy.loginfo("%i", now.secs-lasttime.secs)
        r.sleep()



    rospy.spin()

if __name__=='__main__':
    listener()
