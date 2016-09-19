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
const int BETA = 4267;
const float R_ZERO = 100000.0;
const float TO_VOLTS = 5.0/1024.0;
const float R_INF = R_ZERO*exp(1.0*-BETA/298.0);
void setup() {
  pinMode(13, OUTPUT);
  pinMode(fan0, OUTPUT);
  analogWrite(fan0, 0);
  Serial.begin(9600);

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

void loop() {
  digitalWrite(13, HIGH);
//  float resistance = 4700.0*5.0/(1.0*analogRead(thermNozzle)*TO_VOLTS)-4700;
  float resistance = 4700.0*1.0/(5/(1.0*analogRead(thermNozzle)*TO_VOLTS)-1);
  String tov = "To Volts: " + String(TO_VOLTS, 9);
  String res = "Resist: " + String(resistance);
  String rinf = "R_INF: " + String(R_INF, 9);
  Serial.println(tov);
  Serial.println(res);
  Serial.println(rinf);
  float temp = BETA/log(resistance/R_INF)-273.15;
  Serial.println(analogRead(thermNozzle));
  Serial.println(temp);
  delay(1000);
  digitalWrite(13, LOW);
  delay(50);
}
