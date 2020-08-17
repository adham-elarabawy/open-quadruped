#include "AdvServo.h"
#include <Arduino.h>
#include <Servo.h>

void AdvServo::init(int servo_pin, double starting_angle, double tempOffset) {
  servo.attach(servo_pin, 500, 2500);
  off = tempOffset;
  pos = starting_angle + off;
  servo.write((starting_angle + off) * 180 / control_range);
  delay(1000);
  last_actuated = millis();
}

void AdvServo::setType(int tempLegNum, int tempJointNum) {
  leg = tempLegNum;
  joint = tempJointNum;
}

void AdvServo::setPosition(double tempPos, double tempSpeed) {
  tempPos = tempPos + off;
  if(tempPos > control_range) {
//    Serial.println("Attempted to set position out of control range. Capped at max.");
    tempPos = control_range;
  }

  if(tempPos < 0) {
//    Serial.println("Attempted to set position out of control range. Capped at min.");
    tempPos = 0;
  }

  goal = tempPos;
  spd = tempSpeed;
}

int AdvServo::getPosition() {
  return pos - off;
}

void AdvServo::update_clk() {
  if(millis() - last_actuated > wait_time){
    if(abs(pos - goal) > error_threshold) {
      int dir = 0;
      if(goal - pos > 0) {
        dir = 1;
      } else if(goal - pos < 0) {
        dir = -1;
      }
      pos += dir * wait_time / 1000 * spd;
      servo.write(pos * 180 / control_range);
      last_actuated = millis();
    }
  }
}

void AdvServo::detach() {
  servo.detach();
}
