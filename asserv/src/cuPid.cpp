#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "cuPid.hpp"

const double pi = 3.141592f;

double circstrain(double c){
  while(c<-pi)c+=2*pi;
  while(c>pi)c-=2*pi;
  return c;
}

CuPID::CuPID(double * iinput, double * isetpoint, double * ioutput,double ikp,double iki,double ikd){
  while(*iinput<pi)*iinput+=2*pi;
  while(*iinput>3*pi)*iinput-=2*pi;
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
void CuPID::compute(){
  while(*setpoint<pi)*setpoint+=2*pi;
  while(*setpoint>3*pi)*setpoint-=2*pi;
  double E;
  double E1=circstrain(*setpoint)-circstrain(*input);
  double E2=circstrain(*setpoint+pi)-circstrain(*input+pi);
  if(E2<E1)E=E2;
  else E=E1;
  long T=micros();
  double dt=T-prevT;
  if(prevT==-1) dt=0;
  integ+=E*dt;
  double deriv;
  if(dt==0)deriv=0;
  else deriv=(E-prevE)/dt;
  prevT=T;
  prevE=E;
  *output=kp*E+ki*integ+kd*deriv;
}
bool CuPID::isFacingFront(){
  return abs(*setpoint-*input)<pi;
}
void CuPID::set(double ikp,double iki,double ikd){
  kp=ikp;
  ki=iki;
  kd=ikd;
}
double CuPID::getP(){
  return kp;
}
double CuPID::getI(){
  return ki;
}
double CuPID::getD(){
  return kd;
}
void CuPID::reset(){
  prevT=-1;
  prevE=0;
  integ=0;
}
