#include <Servo.h>

class AdvServo {
  private:
  Servo servo;
  double last_actuated;
  double error_threshold = 0.1;
  int control_range = 270;
  double wait_time = 10;
  double off = 0;
  
  public:
  double goal;
  double pos;
  double spd;
  void init(int servo_pin, double starting_angle, double off);
  void setPosition(double tempPos, double tempSpeed);
  void update_clk();
};

void AdvServo::init(int servo_pin, double starting_angle, double tempOffset) {
  servo.attach(servo_pin, 500, 2500);
  off = tempOffset;
  pos = starting_angle + off;
  servo.write((starting_angle + off) * 180 / control_range);
  delay(1000);
  last_actuated = millis();
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

String serialResponse = "";
char sz[] = "FL,H,999.9,999.9"; // general structure with max numbers allowed.
bool ESTOPPED = false;


void upper(char* s) {
  int index = 0;
  for(int i = 0; i < strlen(s); i++){
    s[i] = toupper(s[i]);
  }
}


AdvServo BR_Hip, BR_Shoulder, BR_Wrist, BL_Hip, BL_Shoulder, BL_Wrist, FR_Hip, FR_Shoulder, FR_Wrist, FL_Hip, FL_Shoulder, FL_Wrist;

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

double angleConversion(int leg, int joint, double angle) {
  if(joint == 0){
    if(leg == 3 || leg == 0) {
      angle = -angle;
    }

    if(leg == 1 || leg == 3) {
      angle = -angle;
    }
    angle = angle + 135;
  }
  
  if(joint == 1) {
    if(leg == 0 || leg == 2) {
      angle = 90 + angle;
    }
    if(leg == 1 || leg == 3) {
      angle = 180 - angle;
    }
  }
  
  if(joint == 2) {
    double weird_offset = 50;
    if(leg == 0 || leg == 2) {
      angle = angle - weird_offset;
    }
    if(leg == 1 || leg == 3) {
      angle = (270 + weird_offset) - angle;
    }
  }
  return angle;
}

void setup() {
 Serial1.begin(19200); // default: 9600
 
  // HIPS
  FL_Hip.init(4, 135, 0);
  FR_Hip.init(11, 135, 0);
  BR_Hip.init(8, 135, -4);
  BL_Hip.init(7, 135, 4);

  //SHOULDERS
  FL_Shoulder.init(2, 180, 0);
  FR_Shoulder.init(13, 90, -11);
  BR_Shoulder.init(10, 90, 0);
  BL_Shoulder.init(5, 180, 0);

  //WRISTS
  FL_Wrist.init(3, 0, 0);
  FR_Wrist.init(12, 270, 0);
  BR_Wrist.init(9, 270, 0);
  BL_Wrist.init(6, 0, 0);
}

void loop() {
  if(!ESTOPPED){
    update_servos();
  }
  if (Serial1.available()) {
    serialResponse = Serial1.readStringUntil('\r\n');
    // Convert from String Object to String.
    char buf[sizeof(sz)];
    serialResponse.toCharArray(buf, sizeof(buf));
    char *p = buf;
    char *str;
    int index = 0;
    int leg = -1; //0=FL, 1=FR, 2=BL, 3=BR
    int joint = -1; //0=Hip, 1=Shoulder, 2=Wrist
    double angle = -1;
    double spd = -1;
    while ((str = strtok_r(p, ",", &p)) != NULL) { // delimiter is the dash
      if(strcmp(str, "e") == 0 || strcmp(str, "E") == 0) {
        ESTOPPED = true;
//        Serial.println("ESTOPPED! Restart to resume.");
      }
      if(index == 0) {
        upper(str);
        if(strcmp(str, "FL") == 0){
          leg = 0;
        }
        if(strcmp(str, "FR") == 0){
          leg = 1;
        }
        if(strcmp(str, "BL") == 0){
          leg = 2;
        }
        if(strcmp(str, "BR") == 0){
          leg = 3;
        }
      }

      if(index == 1) {
        upper(str);
        if(strcmp(str, "H") == 0){
          joint = 0;
        }
        if(strcmp(str, "S") == 0){
          joint = 1;
        }
        if(strcmp(str, "W") == 0){
          joint = 2;
        }
      }

      if(index == 2) {
        angle = atof(str);
      }

      if(index == 3) {
        spd = atof(str);
      }
      index++;
    }

    //ERROR CHECK
    if(leg == -1 || joint == -1 || angle == -1 || spd == -1){
//      Serial.println("BIG ERROR -- INCOMPLETE DATA");
    }

    angle = angleConversion(leg, joint, angle);
    
    if(leg == 0) {
      if(joint == 0) {
        FL_Hip.setPosition(angle, spd);
      }
      if(joint == 1) {
        FL_Shoulder.setPosition(angle, spd);
      }
      if(joint == 2) {
        FL_Wrist.setPosition(angle, spd);
      }
    }

    if(leg == 1) {
      if(joint == 0) {
        FR_Hip.setPosition(angle, spd);
      }
      if(joint == 1) {
        FR_Shoulder.setPosition(angle, spd);
      }
      if(joint == 2) {
        FR_Wrist.setPosition(angle, spd);
      }
    }

    if(leg == 2) {
      if(joint == 0) {
        BL_Hip.setPosition(angle, spd);
      }
      if(joint == 1) {
        BL_Shoulder.setPosition(angle, spd);
      }
      if(joint == 2) {
        BL_Wrist.setPosition(angle, spd);
      }
    }

    if(leg == 3) {
      if(joint == 0) {
        BR_Hip.setPosition(angle, spd);
      }
      if(joint == 1) {
        BR_Shoulder.setPosition(angle, spd);
      }
      if(joint == 2) {
        BR_Wrist.setPosition(angle, spd);
      }
    }
    
  }
}
