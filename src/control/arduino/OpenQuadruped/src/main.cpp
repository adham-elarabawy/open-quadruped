#include <Arduino.h>
#include "AdvServo.h"
#include <Servo.h>

using namespace std;

String serialResponse = "";
char sz[] = "FL,H,999.9,999.9"; // general structure with max numbers allowed.
bool ESTOPPED = false;

int loopIndex = 0;
int motorIndex = 0;
int send_factor = 27;


void upper(char* s) {
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
    if(leg == 0 || leg == 1) {
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

int inverse_angleConversion(int leg, int joint, double angle) {
  if(joint == 0){
    if (leg == 0 || leg == 1) {
      angle = 135 - angle;
    }
    if (leg == 2 || leg == 3) {
      angle = angle - 135;
    }
  }

  if(joint == 1) {
    if(leg == 0 || leg == 2) {
      angle = angle - 90;
    }
    if(leg == 1 || leg == 3) {
      angle = 180 - angle;
    }
  }

  if(joint == 2) {
    double weird_offset = 50;
    if(leg == 0 || leg == 2) {
      angle = weird_offset + angle;
    }
    if(leg == 1 || leg == 3) {
      angle = (270 + weird_offset) - angle;
    }
  }
  return angle;
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

String getPosition(int id) {
  String positions[12] = {};

  String key = "0";
  positions[0] = key + "H" + String(inverse_angleConversion(FL_Hip.leg, FL_Hip.joint, FL_Hip.getPosition()));
  positions[1] = key + "S" + String(inverse_angleConversion(FL_Shoulder.leg, FL_Shoulder.joint, FL_Shoulder.getPosition()));
  positions[2] = key + "W" + String(inverse_angleConversion(FL_Wrist.leg, FL_Wrist.joint, FL_Wrist.getPosition()));

  key = "1";
  positions[3] = key + "H" + String(inverse_angleConversion(FR_Hip.leg, FR_Hip.joint, FR_Hip.getPosition()));
  positions[4] = key + "S" + String(inverse_angleConversion(FR_Shoulder.leg, FR_Shoulder.joint, FR_Shoulder.getPosition()));
  positions[5] = key + "W" + String(inverse_angleConversion(FR_Wrist.leg, FR_Wrist.joint, FR_Wrist.getPosition()));

  key = "2";
  positions[6] = key + "H" + String(inverse_angleConversion(BL_Hip.leg, BL_Hip.joint, BL_Hip.getPosition()));
  positions[7] = key + "S" + String(inverse_angleConversion(BL_Shoulder.leg, BL_Shoulder.joint, BL_Shoulder.getPosition()));
  positions[8] = key + "W" + String(inverse_angleConversion(BL_Wrist.leg, BL_Wrist.joint, BL_Wrist.getPosition()));

  key = "3";
  positions[9] = key + "H" + String(inverse_angleConversion(BR_Hip.leg, BR_Hip.joint, BR_Hip.getPosition()));
  positions[10] = key + "S" + String(inverse_angleConversion(BR_Shoulder.leg, BR_Shoulder.joint, BR_Shoulder.getPosition()));
  positions[11] = key + "W" + String(inverse_angleConversion(BR_Wrist.leg, BR_Wrist.joint, BR_Wrist.getPosition()));

  return positions[id];

}

void setup() {
  // Serial.begin(115200);
  Serial1.begin(256000); // default: 9600, 19200, 57600,115200

  // HIPS
  FL_Hip.init(4, 135, 4);
  FR_Hip.init(11, 135, -2);
  BL_Hip.init(7, 135, 4);
  BR_Hip.init(8, 135, -4);

  //SHOULDERS
  FL_Shoulder.init(2, 180, 0);
  FR_Shoulder.init(13, 90, -11);
  BL_Shoulder.init(5, 180, 0); // +
  BR_Shoulder.init(10, 90, 0); // -

  //WRISTS
  FL_Wrist.init(3, 0, 0);
  FR_Wrist.init(12, 270, -8);
  BL_Wrist.init(6, 0, 7);
  BR_Wrist.init(9, 270, -7);

  setLegJointIDS();

  for(int i = 0; i < 12; i++){
    Serial1.println(getPosition(i));
    delay(500);
  }

  delay(1000);
}

void loop() {
  if (loopIndex > send_factor) {
    loopIndex = 0;
  }
  if(motorIndex > 11) {
    motorIndex = 0;
  }
  if(!ESTOPPED){
    update_servos();
  }
  if (Serial1.available()) {
    if(loopIndex == send_factor){
      Serial1.println(getPosition(motorIndex));
      // Serial.println(getPosition(motorIndex));
      motorIndex++;
    }
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
  loopIndex++;
}
