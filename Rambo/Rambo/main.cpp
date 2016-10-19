#include <Arduino.h>

#include <SPI.h>
#include <math.h>
#include <Heater.h>

// E0 Main extruder
const int E0_enable = 26; // low == enabled
const int E0_step = 34;
const int E0_dir = 43;
const int E0_MS1 = 65;
const int E0_MS2 = 66;
const int E0_digipot_channel = 0;
const float E0_steps_per_mm = 38.197;
const int E0_heater_pin = 9;
const int E0_digipot_setting = 150;
const bool E0_EXTRUDE = 0;
const bool E0_RETRACT = 1;
const int E0_thermistor = 0;
const int BETA_NOZZLE = 4267; // Semitec 104GT-2 Thermistor
const long R_ZERO = 100000; // Resistance at 25C
const int E0_SAMPLE_TIME = 500; // milliseconds

Heater E0_heater(E0_heater_pin, E0_thermistor, BETA_NOZZLE, R_ZERO, E0_SAMPLE_TIME, "E0");

// Digipot
const int slave_select_pin = 38;

// Fans
const int small_fan = 5;
const int large_fan = 8;

// Bed Heater
const int bed_heater = 3;

// Stepper Motor
const int MIN_HIGH_PULSE = 200; // microseconds
const int MIN_LOW_PULSE = 5; //microseconds
// TODO: Add ramp information

// THERMISTOR

const int BETA_BED = 3950;    // Not sure this is correct

const float TO_VOLTS = 5.0/1024.0; // 5V board power / 1024 range

// PID working variables
unsigned long last_time;
float input, output, setpoint;
float I_term, last_input;
float kp, ki, kd;
const int sample_time = 1000; //milliseconds = 1 sec
const int OUT_MIN = 0; // PWM off
const int OUT_MAX = 255; // PWM fully on




void setup() {
  // put your setup code here, to run once:
  pinMode(E0_enable, OUTPUT);
  pinMode(E0_step, OUTPUT);
  pinMode(E0_dir, OUTPUT);
  pinMode(E0_MS1, OUTPUT);
  pinMode(E0_MS2, OUTPUT);
//  pinMode(E0_heater, OUTPUT);
  pinMode(slave_select_pin, OUTPUT);
  pinMode(small_fan, OUTPUT);
  pinMode(large_fan, OUTPUT);



  Serial.begin(9600);

  SPI.begin();
  digitalWrite(slave_select_pin, LOW);
  SPI.transfer(E0_digipot_channel);
  SPI.transfer(E0_digipot_setting);
  SPI.end();
  digitalWrite(slave_select_pin, HIGH);

  digitalWrite(E0_MS1, LOW);
  digitalWrite(E0_MS2, LOW);

  E0_heater.setTunings(50, 1, 9); // Initial PID parameters
}

void loop() {
  // put your main code here, to run repeatedly:

  E0_heater.compute();

}
