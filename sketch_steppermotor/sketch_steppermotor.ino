//void setup() {
//  // put your setup code here, to run once:
//pinMode(13, OUTPUT);
//}
//
//void loop() {
//  // put your main code here, to run repeatedly:
//  digitalWrite(13,HIGH);
//  delay(100);
//  digitalWrite(13,LOW);
//  delay(100);
//
//}

// testing a stepper motor with a Pololu A4988 driver board or equivalent
// on an Uno the onboard led will flash with each step
// this version uses delay() to manage timing

byte directionPin = 49;
byte enablePin = 28;
byte stepPin = 36;
int numberOfSteps = 100;
byte ledPin = 13;
int pulseWidthMicros = 20;  // microseconds
int millisbetweenSteps = 15; // milliseconds
byte MS1 = 15; //analog
byte MS2 = 39;

void setup() {

//  Serial.begin(9600);
//  Serial.println("Starting StepperTest");

  delay(2000);

  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);

  analogWrite(MS1, 0);
  digitalWrite(MS2, LOW);

  digitalWrite(enablePin, LOW);
  digitalWrite(ledPin, LOW);
  digitalWrite(directionPin, HIGH);

  
  for (int n = 0; n < numberOfSteps; n++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseWidthMicros); // this line is probably unnecessary
    digitalWrite(stepPin, LOW);

    delay(millisbetweenSteps);

    digitalWrite(ledPin, !digitalRead(ledPin));
  }

  delay(3000);


  digitalWrite(directionPin, LOW);
  for (int n = 0; n < numberOfSteps; n++) {
    digitalWrite(stepPin, HIGH);
    // delayMicroseconds(pulseWidthMicros); // probably not needed
    digitalWrite(stepPin, LOW);

    delay(millisbetweenSteps);

    digitalWrite(ledPin, !digitalRead(ledPin));
  }
}

void loop() {
}
