#include <Arduino.h>

#include <SPI.h>
#include <math.h>
#include "Heater.h"

/**
  ******************************************************************************************************
  The following group of variables are set apart so they can be quickly and easily
  changed by the user. If you plan to change variables outside of this selection
  you must be sure you know what you're doing.
*/
const int _EXT_FEED_RATE = 25;  // The feed rate in mm/min at which you want
                                // the extruder to run.
                                // SR * layer_height * nozzle_dia * robot_travel_speed / filament_area
                                // 0.98*0.3*0.5*(30*60)/(pi*3^2/4)=37
                                // 0.2 layer height = 25

const float EX_CORRECTION_FACTOR = 1; // If the length of filament being extruded
                                      // is not correct you can adjust it here.
                                      // A percentage change of this number will
                                      // create a coresponding percentage change
                                      // to the extrusion distances and feedrates

const int _RETRACT_DIST = 2; // [mm] to retract filament at layer changes and long moves
const int NOZZLE_TEMP = 220; // Temperature in degrees C for the nozzle
const int BED_TEMP = 70; // Temperature in degrees C for the bed

/**
  This is the end of the simple adjustable parameters. Any variable change
  beyond this point should only be done if you know what you are doing.

  ******************************************************************************************************
*/

/**
  *******************************************************************************************************
  In the next section are the pin assignments.
  Please see "RAMBo-1.1B-User-Manual.pdf" Appendix B for a complete listing of
  RAMBo pin assignments.
*/

const int LED_PIN = 13; // The on-board LED

// Inputs from robot
const int AUTO_MODE = 85,
          MAN_EXTRUDE = 84,
          HEAT_BED = 83,
          HEAT_NOZZLE = 82,
          PROG_FEED = 22,
          BETWEEN_LAYER_RETRACT = 80,
          ALL_STOP = 79,
          TRIPLE_RETRACT = 78;

// Outputs to robot
const int BED_AT_TEMP = 71,
          NOZZLE_AT_TEMP = 72;

// E0 Main extruder
const int E0_enable = 26; // low == enabled
const int E0_step = 34;
const int E0_dir = 43;
const int E0_MS1 = 65;
const int E0_MS2 = 66;

// E0 heater
const int E0_thermistor = 0;
const int E0_heater_pin = 9;

// Bed Heater
const int bed_thermistor = 1;
const int bed_heater_pin = 3;

// Digipot
const int slave_select_pin = 38;
const int E0_digipot_channel = 0; // This is not a pin assignment but seemed to fit well here

// Fans
const int small_fan = 5;
const int large_fan = 8;

/**
  End of pin assignments.

  *************************************************************************************************
*/


/**
  *************************************************************************************************
  The next section contains all of the constants and global variables used.
*/

// E0_Exruder Stepper Motor
const float STEPS_PER_MM = 51.833 * EX_CORRECTION_FACTOR; // mm of extrusion * STEPS_PER_MM gives you the
                                                          // the required number of steps to move that many mm.
const int INTERRUPT_RATE = 10000;                          //Hz for interrupt rate

/**
  Here is the equation which should be used to determing the velocity conversion number:
  Vel_Conv = 1[mm/min] * STEPS_PER_MM[steps/mm] * 1[min]/60[sec] * 1[sec]/INTERRUPT_RATE[interrupt] * 2^16[increments/step] = [increments/interrupt]
*/
// TODO: Turn the fromula into code so this value is not hard coded.
const float VELOCITY_CONVERSION = 5.662;              // [min*increments/(mm*interrupt)] for determining velocity
                                                      // This value x a velocity in mm/min puts the

const int MINIMUM_VELOCITY = 7 * VELOCITY_CONVERSION; // [increments/interrupt]
                                                      // Based on testing the motor does not perfrom
                                                      // well when moving slower than 7[mm/min]

const int MAX_VELOCITY = 2500 * VELOCITY_CONVERSION;  // [increments/interrupt]
                                                      // 2500[mm/min] is zooming. Shouldn't need more than this

const int MAX_ACCELERATION = 2;                       // [increments/interrupt^2]
                                                      // An experience driven max value.
                                                      // Too high of an acceleration causes the motor to lock up

const int PROGRAM_FEED_RATE = _EXT_FEED_RATE * VELOCITY_CONVERSION; // [increments/interrupt]
                                                                    // The speed the extruder should be moving
                                                                    // when in program feed is activated

const int MANUAL_EX_RATE = 75 * VELOCITY_CONVERSION;    // [increments/interrupt]
                                                        // 75[mm/min] has been a reliable number for me


int E0_acceleration = 0;            // [increments/interrupt^2] current extruder acceleration
int E0_velocity = 0;                // [increments/interrupt] current extruder velocity
signed int E0_position = 0;         // [increments] Used in the interrupt for controlling
                                    // when to signal a step. A step is signaled when this value
                                    // overflows.

int target_velocity = 0;            // [increments/interrupt] The commanded extruder velocity
int micro_step_scale = 1;           // The micro step scale can be 1, 2, 4, or 16 based on
                                    // the stepper driver data sheet

const int E0_digipot_setting = 100; // Controls the current sent to the stepper motor.
                                    // The valid range is 0-255. Too high and the motor
                                    // over heats. Too low and it doesn't have enough
                                    // torque to turn.

const bool E0_EXTRUDE = 1;          // Used to control the stepper motor driver direction pin
const bool E0_RETRACT = 0;

int RETRACT_DIST = _RETRACT_DIST*STEPS_PER_MM*16; // [steps] retract this many steps between layers
const float PRIME_DIST_FACTOR = 0.9; // prime this fraction of the retracted amount after between layer retract
long num_steps = 0;           // Number of steps we have moved. Used for retracting between layers

// Nozzle Heater
const int BETA_NOZZLE = 4267;       // Semitec 104GT-2 Thermistor
const long R_ZERO = 100000;         // Resistance at 25C
const int E0_SAMPLE_TIME = 500;     // [milliseconds] How often the temperature of the Nozzle
                                    // Should be sampled for its control loop

// Fans
unsigned long last_fan_time = 0;    // Used for keeping track of when the fans are sampled
const int FAN_SAMPLE_TIME = 2000;   // [milliseconds] How often the temperature should be checked
                                    // for determining if the fans should be on

// Bed Heater
const int BETA_BED = 3950;        // The beta value for the bed thermistor
const int bed_sample_time = 1000; // [milliseconds] How often the temperature of the bed
                                  // Should be sampled for its control loop

// Report to Serial
const int REPORT_TIME = 1000;       // [milliseconds] how often a report should be sent
unsigned long last_report_time = 0; // Used for keeping track of when a report should be made
/**
  The end of the general global variables section.

  ******************************************************************************************************
*/

/**
  *******************************************************************************************************
  The State Varaibles used in the state machine.
*/
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

String currState = ""; // Stores the label of the current state
/**
  End of the state variables.

  *****************************************************************************************************
*/

// Initialize the bed heater object
Heater bed_heater(bed_heater_pin, bed_thermistor, BED_AT_TEMP, BETA_BED, R_ZERO, bed_sample_time, "Bed");

// Nozzle Heater
Heater E0_heater(E0_heater_pin, E0_thermistor, NOZZLE_AT_TEMP, BETA_NOZZLE, R_ZERO, E0_SAMPLE_TIME, "E0");


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
  pinMode(TRIPLE_RETRACT, INPUT_PULLUP);


  // Outputs to robot
  pinMode(BED_AT_TEMP, OUTPUT);
  pinMode(NOZZLE_AT_TEMP, OUTPUT);


  // testing
  pinMode(14, OUTPUT); // Serial Ex 10 Used for testing timing
  pinMode(15, OUTPUT); // Serial Ex 9 Used for testing timing

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

  // Stepper driver pins
  digitalWrite(E0_MS1, LOW);
  digitalWrite(E0_MS2, LOW);
  digitalWrite(E0_dir, LOW);

  E0_heater.setTunings(50, 1, 9); // Initial Nozzle PID parameters
  bed_heater.setTunings(50, 0.5, 9); // Initial Bed PID Values

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
The available micro steps are 1 (full speed) 1/2, 1/4, and 1/16.

Please see "A4982-Datasheet Stepper Driver" pg 6 for the microstep truth table.
*/
void scaleMicroStep(){
  int scale = MAX_VELOCITY/abs(E0_velocity);
  if(scale >= 16){
    micro_step_scale = 16;
    digitalWrite(E0_MS1, HIGH);
    digitalWrite(E0_MS2, HIGH);
  }
  else if (scale >= 4){
    micro_step_scale = 4;
    digitalWrite(E0_MS1, LOW);
    digitalWrite(E0_MS2, HIGH);
  }
  else if (scale >= 2){
    micro_step_scale = 2;
    digitalWrite(E0_MS1, HIGH);
    digitalWrite(E0_MS2, LOW);
  }
  else{
    micro_step_scale = 1;
    digitalWrite(E0_MS1, LOW);
    digitalWrite(E0_MS2, LOW);
  }
  delayMicroseconds(2); // a short delay to ensure the digitalWrite commands have been set
}

/**
  This is the interrupt which drives the stepper motor.
*/
ISR(TIMER3_COMPA_vect){
  noInterrupts();
  // Find the velocity error so we know if we should be adjusting the speed
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

  // Make sure we are not over the max velocity
  if(E0_velocity > MAX_VELOCITY){
    E0_velocity = MAX_VELOCITY;
  }
  // Make sure we are not under the max negative velocity
  else if(E0_velocity < -MAX_VELOCITY){
    E0_velocity = -MAX_VELOCITY;
  }

  // Set the stepper driver direction pin based on our current velocity
  digitalWrite(E0_dir, E0_velocity > 0 ? E0_EXTRUDE : E0_RETRACT);

  // The velocity is now set so check if  we need to adjust the micro step
  scaleMicroStep();

  // If we are using micro steps then we need to scale our velocity to produce
  // steps at an appropriately faster rate (although each step doesn't move as far)
  E0_position += E0_velocity*micro_step_scale;

  if(SREG & 0b00001000){  // The third bit in SREG is the overflow flag. If we overflow
                          // then we know we should increment the stepper

    E0_position -= 0x8000; // Subtract a 1 in 16bit math

    // Trigger the step, wait to ensure it is read, and then shut off the trigger
    digitalWrite(E0_step, HIGH);
    delayMicroseconds(2);
    digitalWrite(E0_step, LOW);

    // Increment the number of steps so it can be used with between layer retract
    // Incement by 16/micro_step_scale to account for the extra steps which happen
    // when microstepping.
    if(S_retract){
      num_steps += 16/micro_step_scale;
      if(num_steps >= RETRACT_DIST){
        target_velocity = 0;
      }
    }
    else if(S_prime){
      num_steps -= 16/micro_step_scale;
      if(num_steps <= 0){
        target_velocity = 0;
      }
    }
  }
  interrupts();
}

/**
  checkStates() runs the state machine. It reads all of the inputs from the robot,
  runs the state machine logic, and then sets the outputs it controls.
*/
void checkStates(){
  // Read all of the inputs first. This saves time since we don't have to read
  // inputs multiple times and it helps prevent problems if inputs change in the middle
  // of the logic run.
  // We are using the internal pull-up resistors on the inputs which means they
  // read HIGH when there is no input and LOW when there is an input. To help
  // make the logic easier to read and write all of the inputs are inverted at read time.
  bool  auto_mode = !digitalRead(AUTO_MODE),
        man_extrude = !digitalRead(MAN_EXTRUDE),
        heat_bed = !digitalRead(HEAT_BED),
        heat_nozzle = !digitalRead(HEAT_NOZZLE),
        prog_feed = !digitalRead(PROG_FEED),
        between_layer_retract = !digitalRead(BETWEEN_LAYER_RETRACT),
        all_stop = !digitalRead(ALL_STOP),
        triple_retract = !digitalRead(TRIPLE_RETRACT);

  if(triple_retract){
    RETRACT_DIST = _RETRACT_DIST*_RETRACT_DIST*STEPS_PER_MM*16*3;
  }
  else{
    RETRACT_DIST = _RETRACT_DIST*_RETRACT_DIST*STEPS_PER_MM*16;
  }

  // The logic for the state machine. Please see the state transition diagram for a
  // clearer picture of how the machine should run.
  S0 = (S0 || D1 || D2 || D3 ||(S_wait && !auto_mode)) && !(S_manual_extrude || S_auto);
  S_manual_extrude = (S_manual_extrude || (S0 && man_extrude)) && !D1;
  D1 = (D1 || (S_manual_extrude && !man_extrude)) && !S0;
  S_prime = (S_prime || (S_wait && !between_layer_retract)) && !S_auto;
  S_wait = (S_wait || (S_retract && ((num_steps >= RETRACT_DIST))))
            && !(S_prime || S0);
  S_retract = (S_retract || (S_auto && between_layer_retract)) && !S_wait;
  S_auto = (S_auto || D4 || (S0 && auto_mode) || (S_prime && (num_steps <= 0)))
            && !(D2 || S_retract || S_program_extrude);
  S_program_extrude = (S_program_extrude || (S_auto && prog_feed)) && !D4;
  D2 = (D2 ||(S_auto && !auto_mode)) && !S0;
  D3 = (D3 || (S_ALL_STOP && !(prog_feed || heat_bed || heat_nozzle
            || between_layer_retract || all_stop || man_extrude || auto_mode))) && !S0;
  D4 = (D4 || (S_program_extrude && !prog_feed)) && !S_auto;
  S_ALL_STOP = (S_ALL_STOP || all_stop) && !D3;

  // To help prent coding logic error we handle the all stop state seperately
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
  // The rest of the outputs are handled inside this else block
  else{
    if(S0 && !(S_manual_extrude || S_auto)){
      digitalWrite(E0_enable, HIGH); // High means disable motor
      target_velocity = 0;
      currState = "S0";
    }
    else if(S_manual_extrude && !D1){
      digitalWrite(E0_enable, LOW); // LOW means enable motor
      target_velocity = MANUAL_EX_RATE;
      currState = "Manul Extrude";
    }
    // There was a problem where the BLR signal was shut off, primming extrusion started
    // And then the BLR signal turned back on before num_steps >= RETRACT_DIST.
    // When num_steps was finally equal to RETRACT_DIST the state machine would be in
    // state S_auto and S_retract at the same time, causing this else if block to be skipped
    // but we had not been in S_auto yet so num_steps was not getting reset to 0.
    // So if we are in S_auto we make sure to enter here by not including any additional
    // qualifiers.
    else if(S_auto && !S_retract){
      digitalWrite(E0_enable, LOW); // LOW means enable motor
      num_steps = 0;
      target_velocity = 0;
      currState = "Auto mode";
    }
    else if(S_retract && !S_wait){
      target_velocity = -MAX_VELOCITY;
      currState = "Retract";
    }
    else if(S_wait && !(S_prime || S0)){
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

    // If we are not in ALL_STOP and the appropriate inputs are set high
    // then set the temperatures.
    E0_heater.setTargetTemp(heat_nozzle ? NOZZLE_TEMP : 0);
    bed_heater.setTargetTemp(heat_bed ? BED_TEMP : 0);

  }
}

/**
  setFans() turns on the fans if the nozzle temperature is above a specified
  value. If only runs when the ellapsed time is greater than FAN_SAMPLE_TIME.
*/
void setFans(){
  // TODO: turn the test times into constants
  unsigned long now = millis();
  if(now - last_fan_time > FAN_SAMPLE_TIME){
    float currTemp = E0_heater.getCurrTemp();
    if (currTemp > 50) {
      analogWrite(small_fan, 255);
    }
    else{
      analogWrite(small_fan, 0);
    }
    if(currTemp > 100){
      analogWrite(large_fan, 255);
    }
    else{
      analogWrite(large_fan, 0);
    }
    last_fan_time = now;
  }
}

/**
  Send a String based report to the serial monitor at the specified interval.
*/
void report(){
  unsigned long now = millis();
  if(now - last_report_time > REPORT_TIME){
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
    // Serial.print("num_steps: ");
    // Serial.println(num_steps);
    // Serial.print("Micro Step: ");
    // Serial.println(micro_step_scale);
    last_report_time = now;

  }
}

/**
  loop() runs full speed and calls each of the included options very often.
  to prvent unneeded calculations methods called from within loop() should
  conatain an elapsed time check.
*/
void loop() {
  E0_heater.compute();
  bed_heater.compute();
  checkStates();
  setFans();
  report();
}
