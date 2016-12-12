#ifndef Heater_h
#define Heater_h

#include "Arduino.h"

class Heater{
  public:
    Heater(int heatPin, int thermPin, int atTempPin, int beta, float r_zero, int sample_time, String id);
    void compute();
    float getCurrTemp();
    void setTunings(float Kp, float Ki, float Kd);
    void setTargetTemp(float set_p);
    bool getAtTemp();
    float getTargetTemp();
    float getOutput();
    String message();

  private:
    int HEAT_PIN,
        THERM_PIN,
        AT_TEMP_PIN;


    String ID;

    unsigned long lastTime;
    float output;
    float targetTemp;
    float I_term;
    float lastTemp;
    bool atTemp;
    float kp;
    float ki;
    float kd;
    int SAMPLE_TIME;
    int OUT_MIN;
    int OUT_MAX;
    int BETA;
    float R_ZERO;
    float R_INF;

    int DIVIDE_RESIST;
    float TO_VOLTS;
};

#endif
