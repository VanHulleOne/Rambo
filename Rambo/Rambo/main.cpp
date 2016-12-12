#include <Arduino.h>

#include <SPI.h>
#include <math.h>
#include "Heater.h"

/**
  The following set of variables are set apart so they can be quickly and easily
  changed by the user. If you plan to change variables outside of this selection
  you must be sure you know what you're doing.
*/
const int _EXT_FEED_RATE = 37;  // The feed rate in mm/min at which you want
                                // the extruder to run.
                                // SR * layer_height * nozzle_dia * robot_travel_speed / filament_area

const float EX_CORRECTION_FACTOR = 1; // If the length of filament being extruded
                                      // is not correct you can adjust it here

const int _RETRACT_DIST = 3; // [mm] to retract filament at layer changes are far moves
const int NOZZLE_TEMP = 220; // Temperature in degrees C for the nozzle
const int BED_TEMP = 70; // Temperature in degrees C for the bed

/**
  This is the end of the simple adjustable parameters. Any variable change
  beyond this point should only be done if you know what you are doing.
*/

// E0 Main extruder
const int E0_enable = 26; // low == enabled
const int E0_step = 34;
const int E0_dir = 43;
const int E0_MS1 = 65;
const int E0_MS2 = 66;
const int E0_digipot_channel = 0;

const float STEPS_PER_MM = 51.833 * EX_CORRECTION_FACTOR; // mm of extrusion * STEPS_PER_MM gives you the
                                                        // the required number of steps to move that many mm.
const int INTERRUPT_RATE = 10000; //Hz for interrupt rate
// 1[mm/min] * STEPS_PER_MM[steps/min] * 1[min]/60[sec] * 1[sec]/INTERRUPT_RATE[interrupt] * 2^15[increments/step]
const float VELOCITY_CONVERSION = STEPS_PER_MM * (2 << 15) / (60 * INTERRUPT_RATE); // [increments/interrupt] for determining velocity

// const float E0_steps_per_mm = 38.197;
const int E0_heater_pin = 9;
const int E0_digipot_setting = 100;
const bool E0_EXTRUDE = 0;
const bool E0_RETRACT = 1;
const int E0_thermistor = 0;
const int BETA_NOZZLE = 4267; // Semitec 104GT-2 Thermistor
const long R_ZERO = 100000; // Resistance at 25C
const int E0_SAMPLE_TIME = 500; // milliseconds
const int LED_PIN = 13;
const int MINIMUM_VELOCITY = 7 * VELOCITY_CONVERSION; // Based on testing the motor does not perfrom
                                                      // well when moving slower than 7[mm/min]
const int MAX_VELOCITY = 5000 * VELOCITY_CONVERSION; // 5000[mm/min] is zooming. Shouldn't need more than this
const int MAX_ACCELERATION = 2; // An expereince driven max value. Relability goes down at 4

const int PROGRAM_FEED_RATE = _EXT_FEED_RATE * VELOCITY_CONVERSION;
const int MANUAL_EX_RATE = 75 * VELOCITY_CONVERSION; // 75[mm/min] has been a reliable number for me
int E0_acceleration = 0;
int E0_velocity = 0;
signed int E0_position = 0;
int target_velocity = 0;
unsigned long motor_test_time = 0;
int micro_step_scale = 1; // The micro step scale can be 1, 2, 4, or 16 based on
                          // the stepper driver data sheet

// Digipot
const int slave_select_pin = 38;

// Fans
const int small_fan = 5;
const int large_fan = 8;
unsigned long last_fan_time = 0;
const int FAN_SAMPLE_TIME = 2000; // milliseconds

// Bed Heater
const int bed_heater_pin = 3;
const int BETA_BED = 3950;    // Not sure this is correct
const int bed_thermistor = 1;
const int bed_sample_time = 1000; // milliseconds
Heater bed_heater(bed_heater_pin, bed_thermistor, BETA_BED, R_ZERO, bed_sample_time, "Bed");

// Nozzle Heater
Heater E0_heater(E0_heater_pin, E0_thermistor, BETA_NOZZLE, R_ZERO, E0_SAMPLE_TIME, "E0");

// betweenLayerRetract
const int RETRACT_DIST = _RETRACT_DIST*STEPS_PER_MM; // retract this many steps between layers
long num_steps = 0;
bool  S_retract = 0,
      S_between_layer = 0,
      S_prime = 0,
      S_wait = 0,
      S_auto = 0,
      S_manual_extrude = 0,
      S_program_extrude = 0,
      S_ALL_STOP = 0,
      S0 = 1,
      D1 = 0,
      D2 = 0,
      D3 = 0,
      D4 = 0;
int direction = 0;

String currState = "";

// Inputs from robot
const int AUTO_MODE = 85,
          MAN_EXTRUDE = 84,
          HEAT_BED = 83,
          HEAT_NOZZLE = 82,
          PROG_FEED = 81,
          BETWEEN_LAYER_RETRACT = 80,
          ALL_STOP = 79;

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
  pinMode(AUTO_MODE, INPUT_PULLUP);
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

  OCR3A = 16000000/INTERRUPT_RATE; // 1600;     // compare match register 10kHz
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

  if(abs(target_velocity) < MINIMUM_VELOCITY){ // If we are not supposed to be moving
    // Note: This code does not decelerate to 0 if target_velocity == 0
    E0_velocity = 0;
    E0_acceleration = 0;
  }
  else if(abs(velocity_error)*2 <= MAX_ACCELERATION){
    // If we are within 1/2 of an acceleration step of our target velocity then
    // stop accelerating and assign the target velocity
    E0_acceleration = 0;
    E0_velocity = target_velocity;
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
    num_steps += 1;
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

void checkStates(){
  bool  auto_mode = !digitalRead(AUTO_MODE),
        man_extrude = !digitalRead(MAN_EXTRUDE),
        heat_bed = !digitalRead(HEAT_BED),
        heat_nozzle = !digitalRead(HEAT_NOZZLE),
        prog_feed = !digitalRead(PROG_FEED),
        between_layer_retract = !digitalRead(BETWEEN_LAYER_RETRACT),
        all_stop = !digitalRead(ALL_STOP);

  S0 = (S0 || D1 || D2 || D3 ||(S_wait && !auto_mode)) && !(S_manual_extrude || S_auto);
  S_manual_extrude = (S_manual_extrude || (S0 && man_extrude)) && !D1;
  D1 = (D1 || (S_manual_extrude && !man_extrude)) && !S0;
  S_auto = (S_auto || D4 || (S0 && auto_mode && !between_layer_retract) || (S_prime && (num_steps >= RETRACT_DIST)))
            && !(D2 || S_retract || S_program_extrude);
  S_retract = (S_retract || (S_auto && between_layer_retract)) && !S_wait;
  S_wait = (S_wait || (S_retract && (num_steps >= RETRACT_DIST))) && !(S_prime || S0);
  S_prime = (S_prime || (S_wait && !between_layer_retract)) && !S_auto;
  S_program_extrude = (S_program_extrude || (S_auto && prog_feed)) && !D4;
  D2 = (D2 ||(S_auto && !auto_mode)) && !S0;
  D3 = (D3 || (S_ALL_STOP && !(prog_feed || heat_bed || heat_nozzle
            || between_layer_retract || all_stop || man_extrude || auto_mode))) && !S0;
  D4 = (D4 || (S_program_extrude && !prog_feed)) && !S_auto;
  S_ALL_STOP = (S_ALL_STOP || all_stop) && !D3;

  if(S_ALL_STOP){
    S0  = 0;
    S_manual_extrude = 0;
    D1 = 0;
    S_auto = 0;
    S_program_extrude = 0;
    S_retract = 0;
    S_wait = 0;
    S_prime = 0;
    D2 = 0;
    D3 = 0;
    D4 = 0;
    E0_heater.setTargetTemp(0);
    bed_heater.setTargetTemp(0);
    target_velocity = 0;
    currState = "ALL_STOP";
  }
  else{
    if(S0 && !(S_manual_extrude || S_auto)){
      target_velocity = 0;
      currState = "S0";
    }
    else if(S_manual_extrude && !D1){
      target_velocity = MANUAL_EX_RATE;
      currState = "Manul Extrude";
    }
    else if(S_auto){
      num_steps = 0;
      target_velocity = 0;
      currState = "Auto mode";
    }
    else if(S_retract && !S_wait){
      target_velocity = -MAX_VELOCITY;
      currState = "Retract";
    }
    else if(S_wait && !(S_prime || S0)){
      num_steps = 0;
      target_velocity = 0;
      currState = "Wait";
    }
    else if(S_prime && !S_auto){
      target_velocity = MAX_VELOCITY;
      currState = "Prime";
    }
    else if(S_program_extrude && !D4){
      target_velocity = PROGRAM_FEED_RATE;
      currState = "Program Extrude";
    }
// TODO: Move this code into Heater modules
    if(heat_nozzle){
      E0_heater.setTargetTemp(NOZZLE_TEMP);
      if(E0_heater.getAtTemp()){
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
      bed_heater.setTargetTemp(BED_TEMP);
      if(bed_heater.getAtTemp()){
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
    if (currTemp > 50) {
      analogWrite(small_fan, 255);
    }
    if(currTemp > 180){
      analogWrite(large_fan, 255);
    }
    last_fan_time = now;
  }
}

void report(){
  unsigned long now = millis();
  if(now - last_report_time > 1000){
    Serial.print("velocity target: ");
    Serial.print(target_velocity);
    Serial.print("\tE0_velocity: ");
    Serial.println(E0_velocity);
    Serial.print("Accel: ");
    Serial.println(E0_acceleration);
    Serial.print("Curr State: ");
    Serial.println(currState);
    Serial.print(E0_heater.message());
    Serial.println(bed_heater.message());

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
