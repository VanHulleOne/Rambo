#include <SPI.h>
#include <math.h>

// testing a stepper motor with a Pololu A4988 driver board or equivalent
// on an Uno the onboard led will flash with each step
// this version uses delay() to manage timing

//Y axis
byte directionPin = 49;
byte enablePin = 28;
byte stepPin = 36;
int numberOfSteps = 2000;
byte ledPin = 13;
int pulseHigh = 200;  // microseconds
int pulseLow = 5; // microseconds
byte MS1 = 69; //analog
byte MS2 = 39;
int slaveSelectPin = 38;
int yAxisDigiPot = 5;
float upTime = 0.5; //sec
int platTime = 4; //sec
int thermNozzle = 0;
int fan0 = 8;
int fan2 = 2;
int readySig = 70;
const int BETA = 4092; // 4092 is the Beta for TDK 100k Epcos Therm #8304
                        // 4267 is the Beta from the Semitec 104GT-2
const float R_ZERO = 100000.0; // Resistance at 25C
const float TO_VOLTS = 5.0/1024.0; // 5V board power / 1024 10bit range
const float R_INF = R_ZERO*exp(1.0*-BETA/298.0);

/* PID working variables*/
int heat0 = 9; // Heater for extruder 0
unsigned long lastTime;
float Input, Output, Setpoint;
float ITerm, lastInput;
float kp, ki, kd;
int SampleTime = 1000; //1 sec
int outMin = 0; // The min output value, 0 is fully off
int outMax = 255; // the max output value, 255 is the max for PWM

void setup() {
  pinMode(13, OUTPUT);
  pinMode(fan0, OUTPUT);
  pinMode(fan2, OUTPUT);
  pinMode(readySig, OUTPUT);
  pinMode(heat0, OUTPUT);
  digitalWrite(fan2, LOW);
  Setpoint = 100;

  digitalWrite(readySig, HIGH);
//  analogWrite(fan0, 255);
//  analogWrite(fan2, 0);
  Serial.begin(9600);

  SetTunings(50, 1, 9); // Sets the initial PID parameters

//  pinMode(slaveSelectPin, OUTPUT);
//  SPI.begin();
//  digitalWrite(slaveSelectPin, LOW);
//  SPI.transfer(yAxisDigiPot);
//  SPI.transfer(150);
//  digitalWrite(slaveSelectPin, HIGH);
//  Serial.begin(9600);
//  Serial.println("Starting StepperTest");
//  delay(2000);
//
//  pinMode(directionPin, OUTPUT);
//  pinMode(stepPin, OUTPUT);
//  pinMode(ledPin, OUTPUT);
//  pinMode(enablePin, OUTPUT);
//  pinMode(MS1, OUTPUT);
//  pinMode(MS2, OUTPUT);
//
//  digitalWrite(MS1, LOW);
//  digitalWrite(MS2, LOW);
//
//  digitalWrite(enablePin, LOW);
//  digitalWrite(ledPin, LOW);
//  digitalWrite(directionPin, HIGH);
//
////  int upSteps = 1000000.0*upTime/(pulseHigh+pulseLow);
//  float pulseHighDec = 0.5;// (550.0 - pulseHigh)/upSteps;
//  int upSteps = (550-pulseHigh)/pulseHighDec;
//  float pulseLowDec = (100.0 - pulseLow)/upSteps;
//  Serial.println(pulseLowDec);
//  for (int n = upSteps; n > 0; n--){
//    digitalWrite(stepPin, HIGH);
//    delayMicroseconds(pulseHigh+pulseHighDec*n);
//    digitalWrite(stepPin, LOW);
//
//    delayMicroseconds(pulseLow+pulseLowDec*n);
//
//    digitalWrite(ledPin, !digitalRead(ledPin));
//  }
//  long platSteps = 1000000*platTime/(pulseHigh+pulseLow);
//  for (long n = 0; n < platSteps; n++) {
//    digitalWrite(stepPin, HIGH);
//    delayMicroseconds(pulseHigh);
//    digitalWrite(stepPin, LOW);
//
//    delayMicroseconds(pulseLow);
//
//    digitalWrite(ledPin, !digitalRead(ledPin));
//  }
//
//  delay(3000);

}

void Compute()
{
  // if(!digitalRead(heatOnPin) return;
  unsigned long now = millis();
  int timeChange = (now - lastTime);
  if(timeChange>=SampleTime)
  {
    Input = getCurrTemp();
    /*Compute all the working error variables*/
    double error = Setpoint - Input;
    ITerm+= (ki * error);
    if(ITerm > outMax) ITerm = outMax;
    else if(ITerm < outMin) ITerm = outMin;
    float dInput = (Input - lastInput);
  
    /*Compute PID Output*/
    Output = kp * error + ITerm- kd * dInput;
    if(Output > outMax) Output = outMax;
    else if(Output < outMin) Output = outMin;

    String out = "Output: " + String(Output);
    Serial.println(out);
    analogWrite(heat0, Output);
  
    /*Remember some variables for next time*/
    lastInput = Input;
    lastTime = now;
  }
}

float getCurrTemp(){
  float resistance = 4700.0/(5/(analogRead(thermNozzle)*TO_VOLTS)-1);
  String tov = "To Volts: " + String(TO_VOLTS, 9);
  String res = "Resist: " + String(resistance);
  String rinf = "R_INF: " + String(R_INF, 9);
  Serial.println(res);
  float temp = BETA/log(resistance/R_INF)-273.15;
  String tempString = "Temp: " + String(temp);
  Serial.println(tempString);
  return temp;
}

void SetTunings(double Kp, double Ki, double Kd)
{ 
  double SampleTimeInSec = ((double)SampleTime)/1000;
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;
}

void loop() {
  Compute();
}
