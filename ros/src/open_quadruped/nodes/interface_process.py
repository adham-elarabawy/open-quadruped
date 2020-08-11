#!/usr/bin/env python
 
import rospy, time, math, cv2, sys
 
from std_msgs.msg import String, Float32, Image, LaserScan, Int32
 
varS=None
 
#define function/functions to provide the required functionality
def fnc_callback(msg):
    global varS
    varS==do_something(msg.data)
 
if __name__=='__main__':
    #Add here the name of the ROS. In ROS, names are unique named.
    rospy.init_node('interface_process')
    #subscribe to a topic using rospy.Subscriber class
    sub=rospy.Subscriber('controller', THE_TYPE_OF_THE_MESSAGE, fnc_callback)
    #publish messages to a topic using rospy.Publisher class
    pub=rospy.Publisher('joint_angles', THE_TYPE_OF_THE_MESSAGE, queue_size=1)
    rate=rospy.Rate(300) # puvlish rate: 300 hz
 
    while not rospy.is_shutdown():
        if varS<= var2:
            varP=something()
        else:
            varP=something()
 
        pub.publish(varP)
        rate.sleep()
