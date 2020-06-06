#include <Servo.h>

class AdvServo {
  private:
  Servo servo;
  double last_actuated;
  double error_threshold = 0.1;
  int control_range = 270;
  double wait_time = 15;
  
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
      pos += wait_time / 1000 * spd;
      servo.write(pos * 180 / control_range);
      last_actuated = millis();
    }
  }
}

#define num_servos 2
AdvServo servo1, servo2, servo3;
void setup() {
  Serial.begin(9600);
  servo1.init(2, 0);
  servo2.init(3, 0);
  servo3.init(13, 0);
  servo1.setPosition(270, 150);
  servo2.setPosition(270, 150);
  servo3.setPosition(270, 150);
}

void loop() {
  servo1.update_clk();
  servo2.update_clk();
  servo3.update_clk();
}
