#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "DifferentialController.hpp"

#define MAX_PWM 255

double cap(double in,double c){
  if(abs(in)>c)return in/abs(in)*c;
  else return in;
}

DifferentialController::DifferentialController(double dp,double di,double dd,double ap,double ai,double ad){
  distancePID = new PID(&din,&dtar,&dout,dp,di,dd);
  anglePID = new CuPID(&ain,&atar,&aout,ap,ai,ad);
}

unsigned int balance;
unsigned int reduc;
void DifferentialController::update(double x,double y, double a){
  double cd=sqrt(pow(tx-x,2)+pow(ty-y,2));
  double ca=atan2(ty-y,tx-x);
  //angle
  if(cd>1) atar=ca;
  else atar=ta;
  ain=a;
  anglePID->compute();

  //distance
  if(abs(atan(tx-x-ty+y)-a)>3.141591/2)cd*=-1;
  dtar=0;
  din=cd;
  distancePID->compute();

  if(abs(dout+aout)>MAX_PWM && abs(dout-aout)>MAX_PWM){
    reduc = MAX_PWM/(dout+aout);
    balance = dout/aout;
  }else{
    reduc = 1;
  }
}
void DifferentialController::setTarget(double x,double y, double a){
  tx=x;
  ty=y;
  ta=a;
}

int DifferentialController::getLeft(){
  if(reduc != 1){
    return max(-MAX_PWM, min(reduc*(dout*balance+aout*(1-balance)), MAX_PWM));
  }else{
    return max(-MAX_PWM, min(reduc*(dout+aout), MAX_PWM));
  }
}

int DifferentialController::getRight(){
  if(reduc != 1){
    return max(-MAX_PWM, min(reduc*(dout*balance-aout*(1-balance)), MAX_PWM));
  }else{
    return max(-MAX_PWM, min(reduc*(dout-aout), MAX_PWM));
  }
  // - aout
}
void DifferentialController::setFactors(double dp,double di,double dd,double ap,double ai,double ad){
  distancePID->set(dp,di,dd);
  anglePID->set(ap,ai,ad);
}
double DifferentialController::getFactor(int i){
  if(i==0) return distancePID->getP();
  if(i==1) return distancePID->getI();
  if(i==2) return distancePID->getD();
  if(i==3) return anglePID->getP();
  if(i==4) return anglePID->getI();
  if(i==5) return anglePID->getD();
  return 0;
}
void DifferentialController::reset(){
  distancePID->reset();
  anglePID->reset();
}
