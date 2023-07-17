#include "main.h"
#define catapultP 0
#define catapultD 0

Motor catapult(catapultPort);
Rotation catapultSensor(catapultSensorPort);
Controller master(E_CONTROLLER_MASTER);

double catapultTarg = downTarg, catapultPower = 0, prevError = 0;
bool catapultManual = false, catapultMoving;
void catapultControl(void*ignore){
  while(true){
    if(catapultManual==false){
      double catapultPosition = catapultSensor.get_angle();
      double catapultError = catapultTarg - catapultPosition;
      if((prevError-fabs(catapultError))>35000||fabs(catapultError)<50){catapultMoving = false;}
      else{catapultMoving=true;}
      if(catapultError<0){catapultError = 36000-fabs(catapultError);}
      if(catapultMoving==true){
        catapultPower = catapultError*catapultP + (catapultError-prevError)*catapultD;
        if(catapultPower>127){catapultPower=127;}
        else if(catapultPower<0){catapultPower = 0;}
        catapult.move(catapultPower);
      }
      else{catapult.move(0);}
      double time = millis();
      prevError = catapultError;
      //printf(":%.2f s, pos:%.2f, targ:%.2f, err:%.2f, power:%.2f, \n",time, catapultPosition,catapultTarg,catapultError,catapultPower);
      //printf("%s", catapultMoving ? "true\n" : "false\n");
    }
    delay(10);
  }
}

void setRollerSpeed(double speed){
  Motor roller(rollerPort);
  roller.move(speed);
}

void shootCatapult(){
  catapultTarg = 0;
  delay(500);
  catapultTarg = downTarg;
}
