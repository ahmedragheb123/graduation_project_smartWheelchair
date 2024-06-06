/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <PID_v1.h>

#define PPR 441.0

// Motor pins
const int LEFT_PWM_PIN = 5; // PWM pin for left motor speed control
const int LEFT_INT1_PIN = 6; // Control pin 1 for left motor
const int LEFT_INT2_PIN = 7; // Control pin 2 for left motor
const int RIGHT_PWM_PIN = 9; // PWM pin for right motor speed control
const int RIGHT_INT1_PIN = 8; // Control pin 1 for right motor
const int RIGHT_INT2_PIN = 4; // Control pin 2 for right motor




// Constants for encoder pins
const int encoderPinsRight[2] = {2, 10}; // Encoder pins for right motor
const int encoderPinsLeft[2] = {3, 11}; // Encoder pins for left motor




// Interrupt service routine for right motor encoder
void rightEncoder() {
  if (digitalRead(encoderPinsRight[0]) == digitalRead(encoderPinsRight[1])) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
}

// Interrupt service routine for left motor encoder
void leftEncoder() {
  if (digitalRead(encoderPinsLeft[0]) == digitalRead(encoderPinsLeft[1])) {
    leftEncoderCount--;
  } else {
    leftEncoderCount++;
  }
}


// Main loop function
void Encoder_check() {
  unsigned long currentMillis = millis();

  // Calculate RPM for right motor
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    rightRPM = ((rightEncoderCount / 2.0) / PPR) * 60;
    rightEncoderCount = 0;

    // Calculate RPM for left motor
    leftRPM = ((leftEncoderCount / 2.0) / PPR) * 60;
    leftEncoderCount = 0;
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_INT1_PIN, OUTPUT);
  pinMode(LEFT_INT2_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_INT1_PIN, OUTPUT);
  pinMode(RIGHT_INT2_PIN, OUTPUT);


  // Set encoder pins as inputs
  for (int i = 0; i < 2; i++) {
    pinMode(encoderPinsRight[i], INPUT_PULLUP);
    pinMode(encoderPinsLeft[i], INPUT_PULLUP);
  }

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(encoderPinsRight[0]), rightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinsLeft[0]), leftEncoder, CHANGE);

  Setpoint_left = 50;
  Setpoint_right = 50;

  //turn the PID on
 right_wheel.SetMode(AUTOMATIC);
 left_wheel.SetMode(AUTOMATIC);



}

void loop()
{
  Encoder_check();
  Input_right = rightRPM;
  Input_left = leftRPM;
 
  PID_PWM();
 //right_wheel.Compute();
 //left_wheel.Compute();

 analogWrite(RIGHT_PWM_PIN ,255);
 analogWrite(LEFT_PWM_PIN ,  255);
 digitalWrite(LEFT_INT1_PIN, HIGH);
 digitalWrite(LEFT_INT2_PIN, LOW);
 digitalWrite(RIGHT_INT1_PIN, HIGH);
 digitalWrite(RIGHT_INT2_PIN, LOW);

 Serial.print( rightRPM );
 Serial.print(" ");
 Serial.print(  leftRPM);
 Serial.println();

}


