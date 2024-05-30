#include <Servo.h>

const int greenButtonPin = 3;  // Green button connected to digital pin 3
const int greenLedPin = 7;     // Green LED connected to digital pin 7
const int redLedPin = 6;       // Red LED connected to digital pin 6
const int runningLedPin = 8;
const int servoPin = 4;        // Servo connected to digital pin 4

Servo myServo;
unsigned long moveStartTime = 0;
const unsigned long moveDuration = 1650;  // how long to raise and shut for
bool actionTriggered = false; // Variable to track if action has been triggered

void setup() {
  pinMode(greenButtonPin, INPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(runningLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  Serial.begin(9600);

  myServo.attach(servoPin);
}



void loop() {

  if (digitalRead(greenButtonPin) == HIGH && !actionTriggered) { // If green button is pressed
    actionTriggered = true;  
    performServoAction(greenLedPin, redLedPin, runningLedPin, moveDuration, myServo);
    actionTriggered = false;
  } 

}

void performServoAction(int greenLedPin, int redLedPin, int runningLedPin, int moveDuration, Servo &myServo) {
  digitalWrite(greenLedPin, HIGH);  // Turn on green LED
  digitalWrite(runningLedPin, HIGH);  
  unsigned long moveStartTime = millis();
  while (millis() - moveStartTime < moveDuration) {
    myServo.writeMicroseconds(1000); // Rotate servo forward
  }
  myServo.writeMicroseconds(1500);  // Stop servo
  digitalWrite(greenLedPin, LOW);    // Turn off green LED
  delay(3000);
  digitalWrite(redLedPin, HIGH);    // Turn on red LED
  moveStartTime = millis();
  while (millis() - moveStartTime < moveDuration) {
    myServo.writeMicroseconds(2000); // Rotate servo backward
  }
  myServo.writeMicroseconds(1500);  // Stop servo
  digitalWrite(redLedPin, LOW);     // Turn off red LED
  digitalWrite(runningLedPin, LOW); 
}
