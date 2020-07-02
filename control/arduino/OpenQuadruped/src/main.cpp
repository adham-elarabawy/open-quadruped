#include <Arduino.h>
#include "AdvServo.h"
#include "FootSensor.h"
#include "InverseKinematics.h"
#include "Util.h"
#include <Servo.h>

using namespace std;

String serialResponse = "";
char msg0[] = "0,-999.9,-999.9,-999.9"; // general structure for normal position command
char msg1[] = "0,-999.9,-999.9,-999.9,-999.9,-999.9,-999.9"; // general structure for normal position/speed command
bool ESTOPPED = false;
int max_speed = 500; // deg / sec


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

void setup() {
  Serial1.begin(500000);

  ik.init(8.7, 59, 107, 130); // hip offset 0, hip_offset 1, shoulder length, wrist length

  // HIPS
  FL_Hip.init(4, 135, -2);
  FR_Hip.init(11, 135, 0);
  BL_Hip.init(7, 135, 0);
  BR_Hip.init(8, 135, -3);

  //SHOULDERS
  FL_Shoulder.init(2, 180, -3);
  FR_Shoulder.init(13, 90, -14);
  BL_Shoulder.init(5, 180, 5); // +
  BR_Shoulder.init(10, 90, 0); // -

  //WRISTS
  FL_Wrist.init(3, 0, 2);
  FR_Wrist.init(12, 270, -2);
  BL_Wrist.init(6, 0, 2);
  BR_Wrist.init(9, 270, -1);

  FL_sensor.init(A9, 17);
  FR_sensor.init(A8, 16);
  BL_sensor.init(A7, 15);
  BR_sensor.init(A6, 14);

  setLegJointIDS();

  delay(1000);
}

void loop() {
  if(!ESTOPPED){
    update_servos();
  } else {
    detach_servos();
  }
  update_sensors();
  if (Serial1.available()) {
    serialResponse = Serial1.readStringUntil('\r\n');
    // Convert from String Object to String.
    char buf[sizeof(msg0)];
    serialResponse.toCharArray(buf, sizeof(buf));
    char *ptr = buf;
    char *str;
    int index = 0;
    int leg = -1; //0=FL, 1=FR, 2=BL, 3=BR
    double x = -9999;
    double y = -9999;
    double z = -9999;
    while ((str = strtok_r(ptr, ",", &ptr)) != NULL) { // delimiter is the dash
      if(strcmp(str, "e") == 0 || strcmp(str, "E") == 0) {
        ESTOPPED = true;
      }

      if(index == 0) {
        util.upper(str);
        if(strcmp(str, "0") == 0){
          leg = 0;
        }
        if(strcmp(str, "1") == 0){
          leg = 1;
        }
        if(strcmp(str, "2") == 0){
          leg = 2;
        }
        if(strcmp(str, "3") == 0){
          leg = 3;
        }
      }
      if(index == 1) {
        x = atof(str);
      }
      if(index == 2) {
        y = atof(str);
      }
      if(index == 3) {
        z = atof(str);
      }
      index++;
    }

    //COMPLETE MESSAGE CHECK
    if(leg != -9999 || x != -9999 || y != -9999 || z != -9999){
      Serial.println("complete message");
      double *p;
      p = ik.run(leg, x, y, z);

      double temp_hip = util.toDegrees(*p);

      if(leg == 1 || leg == 2){
        temp_hip *= -1;
      }

      double hip_angle = util.angleConversion(leg, 0, temp_hip);
      double shoulder_angle = util.angleConversion(leg, 1, util.toDegrees(*(p+1)));
      double wrist_angle = util.angleConversion(leg, 2, util.toDegrees(*(p+2)));

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
}
