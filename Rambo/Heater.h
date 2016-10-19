#ifndef Heater_h
#define Heater_h

#include "Arduino.h"

class Heater{
  public:
    Heater(int heatPin, int thermPin, int beta, float r_zero, int sample_time, String id);
    void compute();
    float getCurrTemp();
    void setTunings(float Kp, float Ki, float Kd);

  private:
    int HEAT_PIN;
    int THERM_PIN;
  
    String ID;
  
    unsigned long lastTime;
    float input;
    float output;
    float setpoint;
    float I_term;
    float lastInput;
    float kp;
    float ki;
    float kd;
    int SAMPLE_TIME;
    const int OUT_MIN;
    const int OUT_MAX;
    int BETA;
    float R_ZERO;
    float R_INF;
  
    const int DIVIDE_RESIST;
    const float TO_VOLTS;
};

#endif
