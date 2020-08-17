#include <Servo.h>

class AdvServo {
  private:
  Servo servo;
  double last_actuated;
  double error_threshold = 0.5;
  int control_range = 270;
  double wait_time = 1;

  public:
  double goal;
  double pos;
  double spd;
  int off = 0;
  int leg; // 0: Front Left, 1: Front Right, 2: Back Left, 3: Back Right
  int joint; // 0: Hip, 1: Shoulder, 2: Wrist
  void init(int servo_pin, double starting_angle, double off);
  void setType(int tempLegNum, int tempJointNum);
  void setPosition(double tempPos, double tempSpeed);
  int getPosition();
  void update_clk();
  void detach();
};
