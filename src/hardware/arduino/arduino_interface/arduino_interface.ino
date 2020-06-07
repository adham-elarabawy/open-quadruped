#include <Servo.h>

class AdvServo {
  private:
  Servo servo;
  double last_actuated;
  double error_threshold = 0.1;
  int control_range = 270;
  double wait_time = 1;
  
  public:
  double goal;
  double pos;
  double spd;
  void init(int servo_pin, double starting_angle);
  void setPosition(double tempPos, double tempSpeed);
  void update_clk();
};

void AdvServo::init(int servo_pin, double starting_angle) {
  servo.attach(servo_pin, 500, 2500);
  pos = starting_angle;
  servo.write(starting_angle * 180 / control_range);
  delay(1000);
  last_actuated = millis();
}

void AdvServo::setPosition(double tempPos, double tempSpeed) {
  goal = tempPos;
  spd = tempSpeed;
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

AdvServo BR_Hip, BR_Shoulder, BR_Wrist, BL_Hip, BL_Shoulder, BL_Wrist, FR_Hip, FR_Shoulder, FR_Wrist, FL_Hip, FL_Shoulder, FL_Wrist;
void setup() {
  Serial.begin(9600);

  // HIPS
  FL_Hip.init(4, 135);
  FR_Hip.init(11, 135);
  BR_Hip.init(8, 135);
  BL_Hip.init(7, 135);

  //SHOULDERS
  FL_Shoulder.init(2, 135);
  FR_Shoulder.init(13, 125);
  BR_Shoulder.init(10, 135);
  BL_Shoulder.init(5, 135);

  //WRISTS
  FL_Wrist.init(3, 45);
  FR_Wrist.init(12, 225);
  BR_Wrist.init(9, 225);
  BL_Wrist.init(6, 45);
}

void loop() {
  FL_Hip.update_clk();
  FR_Hip.update_clk();
  BR_Hip.update_clk();
  BL_Hip.update_clk();
}
