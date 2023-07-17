#ifndef _MECHANICS_HPP_
#define _MECHANICS_HPP_
#define downTarg 8650
extern double catapultTarg;
extern bool catapultManual;
void catapultControl(void*ignore);
void setRollerSpeed(double speed);
void shootCatapult();
#endif
