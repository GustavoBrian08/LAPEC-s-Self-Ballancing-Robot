#ifndef _NIDEC_H
#define _NIDEC_H

#include <Arduino.h>

class Nidec {

    private:
      int _break;
      int _pwm;
      int _direction;
      int _inverse;

    public:
        Nidec(int breakPin, int pwmPin, int directionPin);
        void setBreak(bool b);
        void setForce(int force);
        void setInverse(bool inv);
};
#endif