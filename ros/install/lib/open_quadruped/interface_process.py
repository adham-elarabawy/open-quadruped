#!/usr/bin/env python2
 
import rospy, time, math, cv2, sys
 
from std_msgs.msg import String, Float32, Int32
from sensor_msgs.msg import Joy 
from open_quadruped.msg import JointAngles

body_mode = 0
gait_mode = 1
mode=body_mode
 
#define function/functions to provide the required functionality
def fnc_callback(msg):
    global mode
    if msg.buttons[7] == 1:
        mode = gait_mode
    else:
        mode = body_mode
    rospy.loginfo(str(mode))
 
if __name__=='__main__':
    #Add here the name of the ROS. In ROS, names are unique named.
    rospy.init_node('interface_process', log_level=rospy.DEBUG)
    #subscribe to a topic using rospy.Subscriber class
    sub=rospy.Subscriber('joy', Joy, fnc_callback)
    rospy.spin()
    #publish messages to a topic using rospy.Publisher class
    pub=rospy.Publisher('joint_angles', JointAngles, queue_size=1)
    rate=rospy.Rate(300) # publish rate: 300 hz
 
    while not rospy.is_shutdown():
        if varS<= var2:
            varP=something()
        else:
            varP=something()
 
        pub.publish(varP)
        rate.sleep()
