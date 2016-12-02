#include <Arduino.h>

#include <SPI.h>
#include <math.h>
#include "Heater.h"

// E0 Main extruder
const int E0_enable = 26; // low == enabled
const int E0_step = 34;
const int E0_dir = 43;
const int E0_MS1 = 65;
const int E0_MS2 = 66;
const int E0_digipot_channel = 0;
const float E0_steps_per_mm = 38.197;
const int E0_heater_pin = 9;
const int E0_digipot_setting = 100;
const bool E0_EXTRUDE = 0;
const bool E0_RETRACT = 1;
const int E0_thermistor = 0;
const int BETA_NOZZLE = 4267; // Semitec 104GT-2 Thermistor
const long R_ZERO = 100000; // Resistance at 25C
const int E0_SAMPLE_TIME = 500; // milliseconds
const int LED_PIN = 13;
const int MAX_VELOCITY = 10430; // 0.3183 increments/cycle * 2^15 = 10430
const int MAX_ACCELERATION = 4; // 1.1406*10^-4 * 2^15 = 3.73 = 3
int E0_acceleration = 0;
int E0_velocity = 0;
signed int E0_position = 0;
int target_velocity = 0;
unsigned long motor_test_time = 0;
int micro_step_scale = 1; // The micro step scale can be 1, 2, 4, or 16 based on
                          // the stepper driver data sheet

// Nozzle Heater
Heater E0_heater(E0_heater_pin, E0_thermistor, BETA_NOZZLE, R_ZERO, E0_SAMPLE_TIME, "E0");

// Digipot
const int slave_select_pin = 38;

// Fans
const int small_fan = 5;
const int large_fan = 8;
unsigned long last_fan_time = 0;
const int FAN_SAMPLE_TIME = 2000;

// Bed Heater
const int bed_heater_pin = 3;
const int BETA_BED = 3950;    // Not sure this is correct
const int bed_thermistor = 56;
const int bed_sample_time = 1000; // milliseconds
Heater bed_heater(bed_heater_pin, bed_thermistor, BETA_BED, R_ZERO, bed_sample_time, "Bed");

// Stepper Motor
const int MIN_HIGH_PULSE = 200; // microseconds
const int MIN_LOW_PULSE = 5; //microseconds
// TODO: Add ramp information

// Inputs from robot
const int MAN_EXTRUDE = 84,
          HEAT_BED = 83,
          HEAT_NOZZLE = 82,
          PROG_FEED = 80,
          ALL_STOP = 79;

// Outputs to robot
const int BED_AT_TEMP = 71,
          NOZZLE_AT_TEMP = 72;

void setup() {
  // put your setup code here, to run once:
  pinMode(E0_enable, OUTPUT);
  pinMode(E0_step, OUTPUT);
  pinMode(E0_dir, OUTPUT);
  pinMode(E0_MS1, OUTPUT);
  pinMode(E0_MS2, OUTPUT);
  pinMode(slave_select_pin, OUTPUT);
  pinMode(small_fan, OUTPUT);
  pinMode(large_fan, OUTPUT);

  // Inputs from robot
  pinMode(MAN_EXTRUDE, INPUT);
  pinMode(HEAT_BED, INPUT);
  pinMode(HEAT_NOZZLE, INPUT);
  pinMode(PROG_FEED, INPUT);
  pinMode(ALL_STOP, INPUT);

  // Outputs to robot
  pinMode(BED_AT_TEMP, OUTPUT);
  pinMode(NOZZLE_AT_TEMP, OUTPUT);

  pinMode(LED_PIN, OUTPUT);

  Serial.begin(9600);

  /*
  The Rambo board has programmable potentiometers or 'digipots' for tuning
  each individual stepper motor. the following code handles that
  */
  SPI.begin();
  digitalWrite(slave_select_pin, LOW);
  SPI.transfer(E0_digipot_channel);
  SPI.transfer(E0_digipot_setting);
  SPI.end();
  digitalWrite(slave_select_pin, HIGH);

  digitalWrite(E0_MS1, LOW);
  digitalWrite(E0_MS2, LOW);
  digitalWrite(E0_dir, LOW);

  E0_heater.setTunings(50, 1, 9); // Initial Nozzle PID parameters
  E0_heater.setTargetTemp(100);
  bed_heater.setTunings(50, 0.5, 9); // Initial Bed PID Values
  bed_heater.setTargetTemp(35);

  // initialize timer 3 for the stepper motor interrupts
  noInterrupts();
  // clear current bit selections
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;

  OCR3A = 1600;     // compare match register 10kHz
  TCCR3B |= (1 << WGM12); // CTC mode
  TCCR3B |= (1 << CS10); // No prescaling
  TIMSK3 |= (1 << OCIE3A); // enable timer compare interrupt
  interrupts(); // enable global interupts
}


/**
scaleMicroStep() checks the current commanded velocity and determines if it
is within one of the available micro step ranges. Using micro steps enables
better lower speed performance and more reliable accelerations from zero.
The available micro steps are 1 (full speed) 2, 4, and 16.
*/
void scaleMicroStep(){
  int holdScale = MAX_VELOCITY/abs(E0_velocity);
  if(holdScale >= 16){
    micro_step_scale = 16;
    digitalWrite(E0_MS1, HIGH);
    digitalWrite(E0_MS2, HIGH);
  }
  else if (holdScale >= 4){
    micro_step_scale = 4;
    digitalWrite(E0_MS1, LOW);
    digitalWrite(E0_MS2, HIGH);
  }
  else if (holdScale >= 2){
    micro_step_scale = 2;
    digitalWrite(E0_MS1, HIGH);
    digitalWrite(E0_MS2, LOW);
  }
  else{
    micro_step_scale = 1;
    digitalWrite(E0_MS1, LOW);
    digitalWrite(E0_MS2, LOW);
  }
  delayMicroseconds(2);
}

ISR(TIMER3_COMPA_vect){
  noInterrupts();

  E0_velocity += E0_acceleration;
  if(E0_velocity > MAX_VELOCITY){
    E0_velocity = MAX_VELOCITY;
  }
  else if(E0_velocity < -MAX_VELOCITY){
    E0_velocity = -MAX_VELOCITY;
  }
  if(E0_velocity < 0){ // Retract filament
    digitalWrite(E0_dir, HIGH);
  }
  else{ // Extrude filament
    digitalWrite(E0_dir, LOW);
  }

  scaleMicroStep();

  E0_position += E0_velocity*micro_step_scale;

  if(SREG & 0b00001000){ // The third bit in SREG is the overflow flag. If we overflow
                    // Then we know we should increment the stepper

    E0_position -= 0x8000; // Subtract a 1 in 16bit math
    digitalWrite(E0_step, HIGH);
    delayMicroseconds(2);
    digitalWrite(E0_step, LOW);
  }
  if(target_velocity - E0_velocity > MAX_ACCELERATION){
    E0_acceleration = MAX_ACCELERATION;
  }
  else if(E0_velocity - target_velocity > MAX_ACCELERATION){
    E0_acceleration = -MAX_ACCELERATION;
  }
  else{
    E0_acceleration = 0;
  }
  interrupts();
}

void setFans(){
  /*
    Tests the current temperature of the nozzle and then turns on the
    fans if they are above their temperatures.
  */
  unsigned long now = millis();
  if(now - last_fan_time > FAN_SAMPLE_TIME){
    float currTemp = E0_heater.getCurrTemp();
    if (currTemp > 100) {
      analogWrite(small_fan, 255);
    }
    if(currTemp > 180){
      analogWrite(large_fan, 255);
    }
    last_fan_time = now;
  }
}

void testMotor(){
  noInterrupts();
  unsigned long ellapsed = millis() - motor_test_time;
  if((E0_velocity >= MAX_VELOCITY || E0_velocity <= -MAX_VELOCITY) && motor_test_time==0){
    motor_test_time = millis();
    // E0_acceleration *= -1;
  }
  else if(motor_test_time >0 && ellapsed < 1500){

  }
  else if(motor_test_time>0 && ellapsed >= 1500){
    motor_test_time = 0;
    E0_acceleration *= -1;
  }
  interrupts();
}

void loop() {
  if(digitalRead(HEAT_NOZZLE)){
    E0_heater.setTargetTemp(100);
    if(E0_heater.atTemp()){
      digitalWrite(NOZZLE_AT_TEMP, HIGH);
    }
    else{
      digitalWrite(NOZZLE_AT_TEMP, LOW);
    }
  }
  else{
    E0_heater.setTargetTemp(0);
  }
  if(digitalRead(MAN_EXTRUDE)){
    target_velocity = 100*2.086;
  }
  else{
    target_velocity = 0;
  }
  E0_heater.compute();
  bed_heater.compute();
  setFans();
}
