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
void DifferentialController::update(double x,double y, double a){
  double cd=sqrt(pow(tx-x,2)+pow(ty-y,2));
  double ca=atan2(ty-y,tx-x);
  //angle
  if(cd>1) atar=ca;
  else atar=ta;
  ain=a;
  anglePID->compute();
  //distance
  dtar=0;
  din=cd;
  distancePID->compute(anglePID->isFacingFront());
}
void DifferentialController::setTarget(double x,double y, double a){
  tx=x;
  ty=y;
  ta=a;
}
unsigned int balance;
unsigned int reduc;
int DifferentialController::getLeft(){
  return max(-MAX_PWM, min(dout, MAX_PWM));
  // + aout
}
int DifferentialController::getRight(){
  return max(-MAX_PWM, min(dout, MAX_PWM));
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
