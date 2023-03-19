#include "Nidec.h"

//globais 
int _pwmCh = 1;
//long _time;

Nidec::Nidec(int breakPin, int pwmPin, int directionPin){
    _break = breakPin;
    
    _direction = directionPin;
    _inverse = 1;
    pinMode(_break,OUTPUT);
    
    pinMode(_direction,OUTPUT);
    ledcSetup(_pwmCh, 20000, 8);
    ledcAttachPin(pwmPin, _pwmCh);
    _pwm = _pwmCh;
    _pwmCh++;
    digitalWrite(_break,HIGH);
    ledcWrite(_pwm, 255);
}

void Nidec::setBreak(bool b){
    digitalWrite(_break, !b);
}

void Nidec::setForce(int f){
  f*=_inverse;
  if(f<0) digitalWrite(_direction,HIGH);
  else digitalWrite(_direction,LOW);
  int nf = map(abs(f),0,255,245,0);
  ledcWrite(_pwm, nf);
  //Serial.println(f);
}

void Nidec::setInverse(bool i){
  if(i) _inverse = -1;
  else _inverse = 1;
}