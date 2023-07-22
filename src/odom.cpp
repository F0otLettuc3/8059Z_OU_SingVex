#include "main.h"
#include "odom.hpp"
const double halfPi = 1.57079632679;
const double inchPerDeg = 0.01542746392;
const double radPerDeg = 0.0174533;
double angle = halfPi;
double prevL, prevR;
double posL, posR, bearing;
double X, Y;

void resetCoords(double x, double y){
  Motor frontLeft (frontLeftPort);
  Motor midLeft (midLeftPort);
  Motor backLeft (backLeftPort);
  Motor frontRight (frontRightPort);
  Motor midRight (midRightPort);
  Motor backRight (backRightPort);
  Imu imu(imuPort);

frontLeft.tare_position();
  midLeft.tare_position();
  backLeft.tare_position();
  frontRight.tare_position();
  midRight.tare_position();
  backRight.tare_position();
  imu.tare_rotation();
  prevL = 0;
  prevR = 0;

  targBearing = bearing;
  targL = 0;
  targR = 0;

  setCoords(x, y);
}
void setCoords(double x, double y){
  X = x;
  Y = y;
}

void odometry(void * ignore){
  Imu imu (imuPort);
  Motor frontLeft(frontLeftPort);
  Motor frontRight(frontRightPort);
  Controller master (E_CONTROLLER_MASTER);
  posL = 0, posR = 0, bearing = 0;
  while(true){
    if(imu.is_calibrating()){
      resetCoords(0, 0);
    }else {
      posL = frontLeft.get_position()*inchPerDeg;
      posR = frontRight.get_position()*inchPerDeg;
      bearing = imu.get_rotation();
      angle = halfPi - bearing * radPerDeg;

      double encdChangeL = posL-prevL;
      double encdChangeR = posR-prevR;

      double distance = (encdChangeL + encdChangeR)/2;
      X += distance*cos(angle);
      Y += distance*sin(angle);
      /** update prev variables */
      prevL = posL;
      prevR = posR;
    }
    printf("%.2f, %.2f, %.2f\n", X, Y, bearing);
    master.print(0,0,"%.2f, %.2f, %.2f", X, Y, bearing);
    delay(5);
  }
}
