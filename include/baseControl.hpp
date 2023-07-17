#ifndef _BASE_CONTROL_HPP_
#define _BASE_CONTROL_HPP_

extern double targBearing, targL, targR;
extern const double defaultMaxPower;
extern bool auton;
double abscap(double x, double abscap);
void waitBase(double cutoff);
void baseMove(double dist, double cutoff, double moveP, double moveD);
void baseMove(double dist, double cutoff);
void baseTurn(double angle, double cutoff, double turnP, double turnD);
void baseTurn(double angle, double cutoff);
void baseControl(void*ignore);
void printingSensors(void*ignore);
void powerBase(double power);
void changeMaxPower(double power);

#endif
