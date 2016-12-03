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
const int MAX_ACCELERATION = 4; // 1.2207*10^-4 * 2^15 = 4
const float VELOCITY_CONVERSION = 2.086;  // The desired speed in mm/min * EXTRDUER_CONVERSION
                                          // puts us into increment math
const int PROGRAM_FEED_RATE = 30 * VELOCITY_CONVERSION;
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

// betweenLayerRetract
const int retract_dist = 1810; // TODO: create constant for converting mm to dist
long num_interrupts = 0;
bool bet_layer_retract_done = true;
bool prev_bet_layer_retract = true;
bool  S_retract = 0,
      S_between_layer = 0,
      S_extrude = 0,
      S_wait = 0,
      S_printing = 0,
      S_manual_extrude = 0,
      S_ALL_STOP = 0,
      S0 = 1,
      D1 = 0,
      D2 = 0,
      D3 = 0;
int direction = 0;

String currState = "";

// Inputs from robot
const int MAN_EXTRUDE = 84,
          HEAT_BED = 83,
          HEAT_NOZZLE = 82,
          PROG_FEED = 81,
          BETWEEN_LAYER_RETRACT = 80,
          ALL_STOP = 79;

bool man_extrude,
    heat_bed,
    heat_nozzle,
    prog_feed,
    between_layer_retract,
    all_stop;

// Outputs to robot
const int BED_AT_TEMP = 71,
          NOZZLE_AT_TEMP = 72;

// Report
unsigned long last_report_time = 0;

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
  pinMode(MAN_EXTRUDE, INPUT_PULLUP);
  pinMode(HEAT_BED, INPUT_PULLUP);
  pinMode(HEAT_NOZZLE, INPUT_PULLUP);
  pinMode(PROG_FEED, INPUT_PULLUP);
  pinMode(BETWEEN_LAYER_RETRACT, INPUT_PULLUP);
  pinMode(ALL_STOP, INPUT_PULLUP);

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

  int velocity_error = target_velocity - E0_velocity;

  if(target_velocity == 0){ // If we are not supposed to be moving
    // Note: This code does not decelerate to 0 if target_velocity == 0
    E0_velocity = 0;
    E0_acceleration = 0;
  }
  else if(abs(velocity_error)/2 <= MAX_ACCELERATION){
    // If we are within 1/2 of an acceleration step of our target velocity then
    // stop accelerating/
    E0_acceleration = 0;
  }
  else if(velocity_error > 0){ // We need to speed up
    E0_acceleration = MAX_ACCELERATION;
  }
  else{ // We need to slow down
    E0_acceleration = -MAX_ACCELERATION;
  }

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
  num_interrupts += 1;
  interrupts();
}

void checkStates(){
  man_extrude = !digitalRead(MAN_EXTRUDE);
  heat_bed = !digitalRead(HEAT_BED);
  heat_nozzle = !digitalRead(HEAT_NOZZLE);
  prog_feed = !digitalRead(PROG_FEED);
  between_layer_retract = !digitalRead(BETWEEN_LAYER_RETRACT);
  all_stop = !digitalRead(ALL_STOP);

  S0 = (S0 || D1 || D2 || D3) && !(S_manual_extrude || S_printing);
  S_manual_extrude = (S_manual_extrude || (S0 && man_extrude)) && !D1;
  D1 = (D1 || (S_manual_extrude && !man_extrude)) && !S0;
  S_printing = (S_printing || (S0 && prog_feed) || (S_extrude && (num_interrupts >= retract_dist))) && !(D2 || S_retract);
  S_retract = (S_retract || (S_printing && between_layer_retract)) && !S_wait;
  S_wait = (S_wait || (S_retract && (num_interrupts >= retract_dist))) && !(S_extrude || D2);
  S_extrude = (S_extrude || (S_wait && !between_layer_retract)) && !S_printing;
  D2 = (D2 ||((S_wait || S_printing) && !prog_feed)) && !S0;
  D3 = (D3 || (S_ALL_STOP && !(prog_feed || heat_bed || heat_nozzle || between_layer_retract || all_stop))) && !S0;
  S_ALL_STOP = (S_ALL_STOP || all_stop) && !D3;
  if(S_ALL_STOP){
    S0  = 0;
    S_manual_extrude = 0;
    D1 = 0;
    S_printing = 0;
    S_retract = 0;
    S_wait = 0;
    S_extrude = 0;
    D2 = 0;
    D3 = 0;
    E0_heater.setTargetTemp(0);
    bed_heater.setTargetTemp(0);
    target_velocity = 0;
    currState = "ALL_STOP";
  }
  else{
    if(S0 && !(S_manual_extrude || S_printing)){
      target_velocity = 0;
      currState = "S0";
    }
    else if(S_manual_extrude && !D1){
      target_velocity = 100 * VELOCITY_CONVERSION; // Manual extrude speed
      currState = "Manul Extrude";
    }
    else if(S_printing && !(D2 || S_retract)){
      num_interrupts = 0;
      target_velocity = PROGRAM_FEED_RATE;
      currState = "Printing";
    }
    else if(S_retract && !S_wait){
      target_velocity = -MAX_VELOCITY;
      currState = "Retract";
    }
    else if(S_wait && !(S_extrude || D2)){
      num_interrupts = 0;
      target_velocity = 0;
      currState = "Wait";
    }
    else if(S_extrude && !S_printing){
      target_velocity = MAX_VELOCITY;
      currState = "Extrude";
    }

    if(heat_nozzle){
      E0_heater.setTargetTemp(190);
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
    if(heat_bed){
      bed_heater.setTargetTemp(50);
      if(bed_heater.atTemp()){
        digitalWrite(BED_AT_TEMP, HIGH);
      }
      else{
        digitalWrite(BED_AT_TEMP, LOW);
      }
    }
    else{
      bed_heater.setTargetTemp(0);
    }
  }
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

void report(){
  unsigned long now = millis();
  if(now - last_report_time > 1000){
    Serial.print("velocity target: ");
    Serial.println(target_velocity);
    Serial.print("Accel: ");
    Serial.print("Curr State: ");
    Serial.println();

    last_report_time = now;
  }
}

void loop() {
  E0_heater.compute();
  bed_heater.compute();
  checkStates();
  setFans();
  report();
}
