#ifndef PIDCLASS_h
#define PIDCLASS_h

#include "Arduino.h"
#include "define.h"

class PID
{
public:
    PID(float xKp, float xKi, float xKd, float xint_time);
    float getCmd(float ref, float act, float maxcmd);
    void PIDinit(float ref, float act);
    void setPara(float xKp, float xKi, float xKd);

    float Kp;
    float Ki;
    float Kd;

private:
    float preError;
    float intError;
    float int_time;
    
    boolean init_done;
    
};

#endif