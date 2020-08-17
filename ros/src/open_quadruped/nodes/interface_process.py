#!/usr/bin/env python
 
import rospy, time, math, cv2, sys, math
 
from std_msgs.msg import String, Float32, Int32
from sensor_msgs.msg import Joy 
from open_quadruped.msg import JointAngles

from lib import leg_ik, body_ik

# body ik params
yaw_limit = 15
pitch_limit = 15
roll_limit = 15

# mode settings
body_mode = 0
gait_mode = 1
mode=body_mode

# instantiate leg & body IK models
leg_model = leg_ik.LegIKModel(109.868, 144.580, 11.369, 63.763)
body_model = body_ik.BodyIKModel(76.655, 229.3, 130)

# setup joystick containers
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
 
if __name__=='__main__':

    # setup ROS node + pub/sub 
    rospy.init_node('interface_process', log_level=rospy.DEBUG)
    sub=rospy.Subscriber('joy', Joy, controller_callback)
    pub=rospy.Publisher('joint_angles', JointAngles, queue_size=10)
    rate=rospy.Rate(10) # publish rate: 10 hz
 
    while not rospy.is_shutdown():
        if not (axes is None or buttons is None):
            if mode == body_mode:
                debug = 'in body mode'

                # use joystick values + limits to figure out desired euler angle representation
                yaw = axes[2] * yaw_limit
                pitch = axes[5] * pitch_limit
                roll = axes[0] * roll_limit

                # use the body IK model to figure out the hip-to-foot vectors needed for the desired pose
                body_model.reset_pose()
                body_model.transform(math.radians(yaw), math.radians(pitch), math.radians(roll))
                htf_vecs = body_model.get_htf_vectors()

            elif mode == gait_mode:
                debug = 'in gait mode'
            rospy.loginfo(debug)

            # convert the hip-to-foot vectors into joint angles using leg IK model
            ja_m = leg_model.ja_from_htf_vecs(htf_vecs)

            # populate the JointAngles ros message
            ja = JointAngles()
            ja.fl = ja_m[0]
            ja.fr = ja_m[1]
            ja.bl = ja_m[2]
            ja.br = ja_m[3]
     
            #publishing JointAngles message to topic
            pub.publish(ja)
            rate.sleep()
