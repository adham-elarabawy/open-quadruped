#ifndef ROSSERIAL_HPP
#define ROSSERIAL_HPP

#define USE_TEENSY_HW_SERIAL

#include <ros.h>
#include <ros/time.h>
#include <Arduino.h>
#include <mini_ros/ContactData.h>
#include <mini_ros/IMUdata.h>
#include <mini_ros/JointAngles.h>
#include <SpotServo.hpp>
#include <IMU.hpp>

struct LegPose
{
    LegType legtype;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    LegJoints(const LegType & legtype_)
    {
        legtype = legtype_;
    }

    void FillLegJoint(const double & des_x, const double & y, const double & z)
    {
        x = des_x;
        y = des_y;
        z = des_z;
    }
};

namespace ros_srl
{

    class ROSSerial
    {
        // Node Handle
        ros::NodeHandle nh_;

        // Joint Angle Subscriber
        ros::Subscriber<mini_ros::JointAngles, ROSSerial> ja_sub_;
        // joint msg timer
        unsigned long prev_joints_time_;
        unsigned long prev_resetter_time_;
        // joint msgs
        LegType fl_leg = FL;
        LegType fr_leg = FR;
        LegType bl_leg = BL;
        LegType br_leg = BR;
        LegPose FL_ = LegPose(fl_leg);
        LegPose FR_ = LegPose(fr_leg);
        LegPose BL_ = LegPose(bl_leg);
        LegPose BR_ = LegPose(br_leg);

        void JointCommandCallback(const mini_ros::JointAngles& ja_msg)
        {
            prev_joints_time_ = micros();

            FL_.FillLegJoint(ja_msg.fls, ja_msg.fle, ja_msg.flw);
            FR_.FillLegJoint(ja_msg.frs, ja_msg.fre, ja_msg.frw);
            BL_.FillLegJoint(ja_msg.bls, ja_msg.ble, ja_msg.blw);
            BR_.FillLegJoint(ja_msg.brs, ja_msg.bre, ja_msg.brw);
        }

        public:
            ROSSerial():
                ja_sub_("spot/joints", &ROSSerial::JointCommandCallback, this)

            {
                nh_.initNode();
                nh_.getHardware()->setBaud(500000);

                nh_.subscribe(ja_sub_);


                nh_.loginfo("TEENSY LLC ROS CLIENT CONNECTED");
            }

            void returnJoints(LegPose & FL_ref,  LegPose & FR_ref, LegPose & BL_ref, LegPose & BR_ref)
            {
                FL_ref = FL_;
                FR_ref = FR_;
                BL_ref = BL_;
                BR_ref = BR_;        
            }

            void run()
            {
                nh_.spinOnce();
            }
     };
}

#endif
