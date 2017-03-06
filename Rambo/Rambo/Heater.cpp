/**
  Heater.cpp
  @author: Luke Van Hulle

  The Heater library contains the logic for running closed loop heater control.
  It takes in all the necessary pins for reading the temperature from a thermistor
  through a voltage divider and using PWM to control the heat level for the heater.
  The set temperature for the heater can be adjusted on the fly.

  TODO: Add logic for feed-forward temperature control
*/

#include "Heater.h"

/**
  The constructor for initializing a Heater object.

  @param heatPin - the pin which turns on the heater
  @param thermPin - thermistor pin
  @param atTempPin - output which tells robot we are at the target temperature
  @param beta     - the beta value for the thermistor
  @param r_zero   - the R0 resistance of the thermistor
  @param sample_time - How often the control law should be sampled
  @param id           - String name of heater, used for serial printing
*/
Heater::Heater(int heatPin, int thermPin, int atTempPin, int beta, float r_zero, int sample_time, String id){
  HEAT_PIN = heatPin;
  THERM_PIN = thermPin;
  AT_TEMP_PIN = atTempPin;
  BETA = beta;
  R_ZERO = r_zero;
  SAMPLE_TIME = sample_time;
  ID = id;
  OUT_MIN = 0; // PWM off
  OUT_MAX = 255; // PWM fully on
  DIVIDE_RESIST = 4700; // [Ohms] of the internal resitor used as a volatge dividor for the thermistor
  TO_VOLTS = 5.0/1024.0; // 5V board power / 1024 10bit range

  R_INF = R_ZERO*exp(1.0 * -BETA / 298.15); // 298K = 25C = reference temp

  pinMode(HEAT_PIN, OUTPUT);
  pinMode(THERM_PIN, INPUT);
  lastTime = 0;
}

/**
  compute() runs the control law for the heater
*/
void Heater::compute(){
  unsigned long now = millis();
  int timeChange = (now - lastTime);
  float currTemp = 0;
  if(timeChange >= SAMPLE_TIME){
    currTemp = getCurrTemp();
    /*Compute all the working error variables*/
    float error = targetTemp - currTemp;
    I_term += (ki * error);
    if(I_term > OUT_MAX) I_term = OUT_MAX;
    else if(I_term < OUT_MIN) I_term = OUT_MIN;
    float dInput = (currTemp - lastTemp);

    /*Compute PID Output*/
    output = kp * error + I_term- kd * dInput;
    if(output > OUT_MAX) output = OUT_MAX;
    else if(output < OUT_MIN) output = OUT_MIN;

    analogWrite(HEAT_PIN, output);

    if(targetTemp - currTemp < 5 && targetTemp != 0){
      atTemp = true;
      digitalWrite(AT_TEMP_PIN, HIGH);
    }
    else{
      atTemp = false;
      digitalWrite(AT_TEMP_PIN, LOW);
    }

    /*Remember some variables for next time*/
    lastTemp = currTemp;
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

float Heater::getTargetTemp(){
  return targetTemp;
}

float Heater::getOutput(){
  return output;
}

bool Heater::getAtTemp(){
  return atTemp;
}

String Heater::message(){
  String s = ID + " Targ Temo: ";
  s += String(targetTemp);
  s += "\tCurr Temp: ";
  s += String(lastTemp);
  s += "\tOutput: ";
  s += String(output);
  s += "\n";
  return s;
}
