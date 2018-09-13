#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "stuPid.hpp"

PID::PID(double * iinput, double * isetpoint,double * ioutput,double ikp,double iki,double ikd){
  kp=ikp;
  ki=iki;
  kd=ikd;
  input=iinput;
  setpoint=isetpoint;
  output=ioutput;
  prevT=-1;
  prevE=0;
  integ=0;
}
void PID::compute(bool isFacingFront){
  double E=*setpoint-*input;
  long T=micros();
  double dt=T-prevT;
  if(prevT==-1) dt=0;
  integ+=E*dt;
  double deriv;
  if(dt==0)deriv=0;
  else deriv=(E-prevE)/dt;
  prevT=T;
  prevE=E;
  *output=kp*E+ki*integ+kd*deriv*(isFacingFront?1.0:-1.0);
}
void PID::set(double ikp,double iki,double ikd){
  kp=ikp;
  ki=iki;
  kd=ikd;
}
double PID::getP(){
  return kp;
}
double PID::getI(){
  return ki;
}
double PID::getD(){
  return kd;
}
void PID::reset(){
  prevT=-1;
  prevE=0;
  integ=0;
}
