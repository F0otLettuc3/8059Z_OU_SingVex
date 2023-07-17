#ifndef _ODOM_HPP_
#define _ODOM_HPP_
extern double X, Y;
extern const double inchPerDeg;
extern double posL, posR, bearing;
void resetCoords(double x, double y);
void setCoords(double x, double y);
void odometry(void * ignore);
#endif
