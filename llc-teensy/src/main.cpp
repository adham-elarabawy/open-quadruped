#define ROSSERIAL_HPP
#define USE_TEENSY_HW_SERIAL

#include <Arduino.h>
#include "AdvServo.h"
#include "FootSensor.h"
#include "InverseKinematics.h"
#include "Util.h"
#include <Servo.h>
#include <ros.h>
#include <open_quadruped/JointAngles.h>

using namespace std;

String serialResponse = "";
bool ESTOPPED = false;
int max_speed = 500; // deg / sec

ros::NodeHandle nh;
AdvServo BR_Hip, BR_Shoulder, BR_Wrist, BL_Hip, BL_Shoulder, BL_Wrist, FR_Hip, FR_Shoulder, FR_Wrist, FL_Hip, FL_Shoulder, FL_Wrist;
FootSensor FL_sensor, FR_sensor, BL_sensor, BR_sensor;
AdvServo * hips[4] = {&FL_Hip, &FR_Hip, &BL_Hip, &BR_Hip};
AdvServo * shoulders[4] = {&FL_Shoulder, &FR_Shoulder, &BL_Shoulder, &BR_Shoulder};
AdvServo * wrists[4] = {&FL_Wrist, &FR_Wrist, &BL_Wrist, &BR_Wrist};
Util util;
InverseKinematics ik;

void detach_servos() {
  // HIPS
  FL_Hip.detach();
  FR_Hip.detach();
  BR_Hip.detach();
  BL_Hip.detach();

  // SHOULDER
  FL_Shoulder.detach();
  FR_Shoulder.detach();
  BR_Shoulder.detach();
  BL_Shoulder.detach();

  // WRIST
  FL_Wrist.detach();
  FR_Wrist.detach();
  BR_Wrist.detach();
  BL_Wrist.detach();
}

void update_servos(){
  // HIPS
  FL_Hip.update_clk();
  FR_Hip.update_clk();
  BR_Hip.update_clk();
  BL_Hip.update_clk();

  // SHOULDER
  FL_Shoulder.update_clk();
  FR_Shoulder.update_clk();
  BR_Shoulder.update_clk();
  BL_Shoulder.update_clk();

  // WRIST
  FL_Wrist.update_clk();
  FR_Wrist.update_clk();
  BR_Wrist.update_clk();
  BL_Wrist.update_clk();
}

void update_sensors() {
  FL_sensor.update_clk();
  FR_sensor.update_clk();
  BL_sensor.update_clk();
  BR_sensor.update_clk();
}

void setLegJointIDS() {
  // HIPS
  FL_Hip.setType(0, 0);
  FR_Hip.setType(1, 0);
  BL_Hip.setType(2, 0);
  BR_Hip.setType(3, 0);

  //SHOULDERS
  FL_Shoulder.setType(0, 1);
  FR_Shoulder.setType(1, 1);
  BL_Shoulder.setType(2, 1);
  BR_Shoulder.setType(3, 1);

  //WRISTS
  FL_Wrist.setType(0, 2);
  FR_Wrist.setType(1, 2);
  BL_Wrist.setType(2, 2);
  BR_Wrist.setType(3, 2);

}

void callback( const open_quadruped::JointAngles& joint_angles){
  float ja_h;
  float ja_s;
  float ja_w;
  Serial.println("Data Received: ");
  for(int leg = 0; leg < 4; leg++) {
    if (leg == 0) {
      ja_h = joint_angles.fl[0];
      ja_s = joint_angles.fl[1];
      ja_w = joint_angles.fl[2];
    } else if (leg == 1) {
      ja_h = joint_angles.fr[0];
      ja_s = joint_angles.fr[1];
      ja_w = joint_angles.fr[2];
    } else if (leg == 2) {
      ja_h = joint_angles.bl[0];
      ja_s = joint_angles.bl[1];
      ja_w = joint_angles.bl[2];
    } else {
      ja_h = joint_angles.br[0];
      ja_s = joint_angles.br[1];
      ja_w = joint_angles.br[2];
    }
    Serial.print(leg);
    Serial.print(": ");
    Serial.print(ja_h);
    Serial.print(", ");
    Serial.print(ja_s);
    Serial.print(", ");
    Serial.println(ja_w);
    double hip_angle = util.angleConversion(leg, 0, ja_h);
    double shoulder_angle = util.angleConversion(leg, 1, ja_s);
    double wrist_angle = util.angleConversion(leg, 2, ja_w);

    double h_dist = abs(hip_angle - (*hips[leg]).getPosition());
    double s_dist = abs(shoulder_angle - (*shoulders[leg]).getPosition());
    double w_dist = abs(wrist_angle - (*wrists[leg]).getPosition());

    double scaling_factor = util.max(h_dist, s_dist, w_dist);

    h_dist /= scaling_factor;
    s_dist /= scaling_factor;
    w_dist /= scaling_factor;

    (*hips[leg]).setPosition(hip_angle, max_speed * h_dist);
    (*shoulders[leg]).setPosition(shoulder_angle, max_speed * s_dist);
    (*wrists[leg]).setPosition(wrist_angle, max_speed * w_dist);
  }
}


ros::Subscriber<open_quadruped::JointAngles> sub("joint_angles", &callback);

void setup() {
  // Serial1.begin(57600);
  Serial.begin(500000);

  // HIPS
  FL_Hip.init(4, 135, 0);
  FR_Hip.init(11, 135, 0);
  BL_Hip.init(7, 135, 0);
  BR_Hip.init(8, 135, 0);

  //SHOULDERS
  FL_Shoulder.init(2, 180, 0);
  FR_Shoulder.init(13, 90, 0);
  BL_Shoulder.init(5, 180, 0); // +
  BR_Shoulder.init(10, 90, 0); // -

  //WRISTS
  FL_Wrist.init(3, 0, 0);
  FR_Wrist.init(12, 270, 0);
  BL_Wrist.init(6, 0, 0);
  BR_Wrist.init(9, 270, 0);

  FL_sensor.init(A9, 17);
  FR_sensor.init(A8, 16);
  BL_sensor.init(A7, 15);
  BR_sensor.init(A6, 14);

  setLegJointIDS();

  delay(1000);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  if(!ESTOPPED){
    update_servos();
  } else {
    detach_servos();
  }
  update_sensors();
}
