#include "Heater.h"

int HEAT_PIN;  // The pin which turns on the heat
int THERM_PIN; // The pin for the thermistor 

String ID; // name of the Heater

unsigned long lastTime;
float input, output, setpoint;
float I_term, lastInput;
float kp, ki, kd;
int SAMPLE_TIME; // Time between samples in millis
const int OUT_MIN = 0; // PWM off
const int OUT_MAX = 255; // PWM fully on
int BETA;
float R_ZERO;
float R_INF;

const int DIVIDE_RESIST = 4700;
const float TO_VOLTS = 5.0/1024.0; // 5V board power / 1024 10bit range

Heater::Heater(int heatPin, int thermPin, int beta, float r_zero, int sample_time, String id){
  HEAT_PIN = heatPin;
  THERM_PIN = thermPin;
  BETA = beta;
  R_ZERO = r_zero;
  SAMPLE_TIME = sample_time;
  ID = id;

  R_INF = R_ZERO*exp(1.0 * -BETA / 298.15); // 298K = 25C = reference temp
  
  pinMode(HEAT_PIN, OUTPUT);
  pinMode(THERM_PIN, INPUT);
  lastTime = 0; 
}

void Heater::compute(){
  unsigned long now = millis();
  int timeChange = (now - lastTime);
  if(timeChange >= SAMPLE_TIME)
  {
    input = getCurrTemp();
    /*Compute all the working error variables*/
    float error = setpoint - input;
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

