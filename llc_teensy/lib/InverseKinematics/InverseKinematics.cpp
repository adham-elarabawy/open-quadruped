#include "InverseKinematics.h"
#include <Arduino.h>
#include<cmath>

void InverseKinematics::init(double hip_offset_0, double hip_offset_1, double temp_shoulder, double temp_wrist) {
  hip_offset[0] = hip_offset_0;
  hip_offset[1] = hip_offset_1;
  shoulder = temp_shoulder;
  wrist = temp_wrist;
}

double * InverseKinematics::run(int legIndex, double x, double y, double z) {
  static double angles[3];

  double h1 = sqrt(pow(hip_offset[0], 2) + pow(hip_offset[1], 2));
  double h2 = sqrt(pow(z, 2) + pow(y, 2));
  double alpha_0 = atan(y / z);
  double alpha_1 = atan(hip_offset[1] / hip_offset[0]);
  double alpha_2 = atan(hip_offset[0] / hip_offset[1]);
  double alpha_3 = asin(h1 * sin(alpha_2 + HALF_PI) / h2);
  double alpha_4 = PI - (alpha_3 + alpha_2 + HALF_PI);
  double alpha_5 = alpha_1 - alpha_4;
  double theta_h = alpha_0 - alpha_5;

  double r0 = h1 * sin(alpha_4) / sin(alpha_3);
  double h = sqrt(pow(r0, 2) + pow(x, 2));
  double phi = asin(x / h);
  double theta_s = acos((pow(h, 2) + pow(shoulder, 2) - pow(wrist, 2)) / (2 * h * shoulder)) - phi;
  double theta_w = acos((pow(wrist, 2) + pow(shoulder, 2) - pow(h, 2)) / (2 * wrist * shoulder));

  angles[0] = theta_h;
  angles[1] = theta_s;
  angles[2] = theta_w;

  if(legIndex < 2) {
    return angles;
  } else {
    angles[0] = -theta_h;
    return angles;
  }
}
