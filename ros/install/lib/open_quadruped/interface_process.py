#!/usr/bin/env python2
 
import rospy, time, math, cv2, sys
 
from std_msgs.msg import String, Float32, Int32
from sensor_msgs.msg import Joy 
from open_quadruped.msg import JointAngles

from lib import leg_ik, body_ik

body_mode = 0
gait_mode = 1
mode=body_mode

leg_model = LegIKModel(109.868, 144.580, 11.369, 63.763)
body_model = BodyIKModel(76.655, 229.3, 130)

buttons = None
axes = None
 
# Controller callback
def controller_callback(msg):
    global mode, body_mode, gait_mode, buttons, axes
    if msg.buttons[7] == 1:
        mode = gait_mode
    else:
        mode = body_mode
    buttons = msg.buttons
    axes = msg.axes
#    rospy.loginfo(str(mode))
 
if __name__=='__main__':
    #Add here the name of the ROS. In ROS, names are unique named.
    rospy.init_node('interface_process', log_level=rospy.DEBUG)
    #subscribe to a topic using rospy.Subscriber class
    sub=rospy.Subscriber('joy', Joy, controller_callback)
    rospy.spin()
    #publish messages to a topic using rospy.Publisher class
    pub=rospy.Publisher('joint_angles', JointAngles, queue_size=1)
    rate=rospy.Rate(300) # publish rate: 300 hz
 
    while not rospy.is_shutdown():
        if mode == body_mode:
            val = 'in body mode'
        elif mode == gait_mode:
            val = 'in gait mode'
        rospy.loginfo(val)

        ja = JointAngles()
        ja.fl = [0,0,0]
        ja.fr = [1,1,1]
        ja.bl = [2,2,2]
        js.br = [3,3,3]
 
        pub.publish(ja)
        rate.sleep()
