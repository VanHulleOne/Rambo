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

// THERMISTOR

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

  E0_heater.setTunings(50, 1, 9); // Initial Nozzle PID parameters
  E0_heater.setTargetTemp(100);
  bed_heater.setTunings(50, 0.5, 9); // Initial Bed PID Values
  bed_heater.setTargetTemp(35);

  // initialize timer 3 for the stepper motor interupts
  noInterrupts();
  // clear current bit selections
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;

  OCR3A = 1600;     // compare match register 10kHz
  TCCR3B |= (1 << WGM12); // CTC mode
  TCCR3B |= (1 << CS10); // No prescaling
  TIMSK3 |= (1 << OCIE3A); // enable timer compare interrupt
  interupts(); // enable global interupts



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

void runMotor(){
  unsinged long = micros();
  if(now - last_E0_time > )
}

void loop() {
  // put your main code here, to run repeatedly:

  E0_heater.compute();
  setFans();
}
