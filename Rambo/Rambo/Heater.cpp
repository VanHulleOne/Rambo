#include "Heater.h"

Heater::Heater(int heatPin, int thermPin, int beta, float r_zero, int sample_time, String id){
  HEAT_PIN = heatPin;
  THERM_PIN = thermPin;
  BETA = beta;
  R_ZERO = r_zero;
  SAMPLE_TIME = sample_time;
  ID = id;
  OUT_MIN = 0; // PWM off
  OUT_MAX = 255; // PWM fully on
  DIVIDE_RESIST = 4700;
  TO_VOLTS = 5.0/1024.0; // 5V board power / 1024 10bit range

  R_INF = R_ZERO*exp(1.0 * -BETA / 298.15); // 298K = 25C = reference temp

  pinMode(13, OUTPUT);
  pinMode(HEAT_PIN, OUTPUT);
  pinMode(THERM_PIN, INPUT);
  lastTime = 0;
}

void Heater::compute(){
  unsigned long now = millis();
  int timeChange = (now - lastTime);
  if(timeChange >= SAMPLE_TIME){
    input = getCurrTemp();
    /*Compute all the working error variables*/
    float error = targetTemp - input;
    I_term += (ki * error);
    if(I_term > OUT_MAX) I_term = OUT_MAX;
    else if(I_term < OUT_MIN) I_term = OUT_MIN;
    float dInput = (input - lastInput);

    /*Compute PID Output*/
    output = kp * error + I_term- kd * dInput;
    if(output > OUT_MAX) output = OUT_MAX;
    else if(output < OUT_MIN) output = OUT_MIN;

//    String out = "Output: " + String(Output);
//    Serial.println(out);
    analogWrite(HEAT_PIN, output);

    /*Remember some variables for next time*/
    lastInput = input;
    lastTime = now;
  }
}

float Heater::getCurrTemp(){
  float resistance = 1.0 * DIVIDE_RESIST / (5 / (analogRead(THERM_PIN)*TO_VOLTS) - 1); // 5 is 5V board
  return BETA / log(resistance/R_INF) - 273.15;
}

void Heater::setTunings(float Kp, float Ki, float Kd){
  kp = Kp;
  ki = Ki * SAMPLE_TIME;
  kd = Kd / SAMPLE_TIME;
}

void Heater::setTargetTemp(float set_p){
  targetTemp = set_p;
}
