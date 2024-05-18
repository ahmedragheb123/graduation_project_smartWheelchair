#include <Arduino.h>
#include <Ultrasonic.h>

#define TRIGGER_PIN 22
#define ECHO_PIN 18
#define TRIGGER_PIN_2 23  
#define ECHO_PIN_2 19
#define TRIGGER_PIN_3 24
#define ECHO_PIN_3 18
#define TRIGGER_PIN_4 25
#define ECHO_PIN_4 19

#define SPEED_OF_SOUND 343 // Speed of sound in air in m/s
const int numReadings = 13;
volatile unsigned long start_time;
volatile unsigned long echo_duration;
volatile unsigned long start_time_2;
volatile unsigned long echo_duration_2;
//volatile unsigned long start_time_3;
volatile unsigned long echo_duration_3;
//volatile unsigned long start_time_4;
volatile unsigned long echo_duration_4;
const int DISTANCE_THRESHOLD = 20;
unsigned int readings[numReadings];  // Array to store ultrasonic sensor readings
unsigned int index = 0;  // Index for circular buffer
unsigned int readings_2[numReadings];  // Array to store ultrasonic sensor readings
unsigned int index_2 = 0;  // Index for circular buffer
unsigned int readings_3[numReadings];  // Array to store ultrasonic sensor readings
unsigned int index_3 = 0;  // Index for circular buffer
unsigned int readings_4[numReadings];  // Array to store ultrasonic sensor readings
unsigned int index_4 = 0;  // Index for circular buffer



void ultrasonic_init(){
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(TRIGGER_PIN_2, OUTPUT);
  pinMode(TRIGGER_PIN_3, OUTPUT);
  pinMode(TRIGGER_PIN_4, OUTPUT);

  pinMode(ECHO_PIN, INPUT);
  pinMode(ECHO_PIN_2, INPUT);
  //pinMode(ECHO_PIN_3, INPUT);
  //pinMode(ECHO_PIN_4, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN_2), echoInterrupt_2, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(ECHO_PIN_3), echoInterrupt_3, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(ECHO_PIN_4), echoInterrupt_4, CHANGE);
  pinMode(26,OUTPUT);
  pinMode(27,OUTPUT);
  pinMode(28,OUTPUT);
  pinMode(29,OUTPUT);
  digitalWrite(26,1);
  digitalWrite(27,1);
  digitalWrite(28,1);
  digitalWrite(29,1);
}

bool measureDistanceForward(){
  digitalWrite(28,1);
  digitalWrite(29,1);
  digitalWrite(26,0);
  digitalWrite(27,0);
// Trigger ultrasonic sensor
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  // Wait for echo pulse to be measured
  //delay(100); // Adjust as needed for your sensor and environment
// Calculate distance
  unsigned long duration = echo_duration;
  float distance = (duration * SPEED_OF_SOUND) / (2.0 * 10000); // Convert to centimeters
  // Update readings array with new value
  readings[index] = distance;
  index = (index + 1) % numReadings;

  // Calculate median value from readings array
  unsigned int sorted[numReadings];
  memcpy(sorted, readings, sizeof(sorted));
  for (int i = 0; i < numReadings - 1; i++) {
    for (int j = 0; j < numReadings - i - 1; j++) {
      if (sorted[j] > sorted[j + 1]) {
        int temp = sorted[j];
        sorted[j] = sorted[j + 1];
        sorted[j + 1] = temp;
      }
    }
  }
  unsigned int median = sorted[numReadings / 2];


  // Trigger ultrasonic sensor
  digitalWrite(TRIGGER_PIN_2, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_2, LOW);
  // Wait for echo pulse to be measured
  //delay(100); // Adjust as needed for your sensor and environment
// Calculate distance
  unsigned long duration_2 = echo_duration_2;
  float distance_2 = (duration_2 * SPEED_OF_SOUND) / (2.0 * 10000); // Convert to centimeters
  // Update readings array with new value
  readings_2[index_2] = distance_2;
  index_2 = (index_2 + 1) % numReadings;

  // Calculate median value from readings array
  unsigned int sorted_2[numReadings];
  memcpy(sorted_2, readings_2, sizeof(sorted_2));
  for (int i = 0; i < numReadings - 1; i++) {
    for (int j = 0; j < numReadings - i - 1; j++) {
      if (sorted_2[j] > sorted_2[j + 1]) {
        int temp_2 = sorted_2[j];
        sorted_2[j] = sorted_2[j + 1];
        sorted_2[j + 1] = temp_2;
      }
    }
  }
  unsigned int median_2 = sorted_2[numReadings / 2];
  
  if(median < DISTANCE_THRESHOLD || median_2 < DISTANCE_THRESHOLD)
    return false; // turn off LED , turn off motor
    else if (median > DISTANCE_THRESHOLD + 5 && median_2 > DISTANCE_THRESHOLD+5)
    return true; // turn on LED , turn on motor
}



bool measureDistanceReverse(){
  digitalWrite(26,1);
  digitalWrite(27,1);
  digitalWrite(28,0);
  digitalWrite(29,0);
// Trigger ultrasonic sensor
  digitalWrite(TRIGGER_PIN_3, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN_3, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_3, LOW);
  // Wait for echo pulse to be measured
  //delay(100); // Adjust as needed for your sensor and environment
// Calculate distance
  unsigned long duration_3 = echo_duration;
  float distance_3 = (duration_3 * SPEED_OF_SOUND) / (2.0 * 10000); // Convert to centimeters
  // Update readings array with new value
  readings_3[index_3] = distance_3;
  index_3 = (index_3 + 1) % numReadings;

  // Calculate median value from readings array
  unsigned int sorted_3[numReadings];
  memcpy(sorted_3, readings_3, sizeof(sorted_3));
  for (int i = 0; i < numReadings - 1; i++) {
    for (int j = 0; j < numReadings - i - 1; j++) {
      if (sorted_3[j] > sorted_3[j + 1]) {
        int temp_3 = sorted_3[j];
        sorted_3[j] = sorted_3[j + 1];
        sorted_3[j + 1] = temp_3;
      }
    }
  }
  unsigned int median_3 = sorted_3[numReadings / 2];


  // Trigger ultrasonic sensor
  digitalWrite(TRIGGER_PIN_4, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN_4, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_4, LOW);
  // Wait for echo pulse to be measured
  //delay(100); // Adjust as needed for your sensor and environment
// Calculate distance
  unsigned long duration_4 = echo_duration_2;
  float distance_4 = (duration_4 * SPEED_OF_SOUND) / (2.0 * 10000); // Convert to centimeters
  // Update readings array with new value
  readings_4[index_4] = distance_4;
  index_4 = (index_4 + 1) % numReadings;

  // Calculate median value from readings array
  unsigned int sorted_4[numReadings];
  memcpy(sorted_4, readings_4, sizeof(sorted_4));
  for (int i = 0; i < numReadings - 1; i++) {
    for (int j = 0; j < numReadings - i - 1; j++) {
      if (sorted_4[j] > sorted_4[j + 1]) {
        int temp_4 = sorted_4[j];
        sorted_4[j] = sorted_4[j + 1];
        sorted_4[j + 1] = temp_4;
      }
    }
  }
  unsigned int median_4 = sorted_4[numReadings / 2];
  
  if(median_3 < DISTANCE_THRESHOLD || median_4 < DISTANCE_THRESHOLD)
    return false; // turn off LED , turn off motor
    else if (median_3 > DISTANCE_THRESHOLD + 5 && median_4 > DISTANCE_THRESHOLD+5)
    return true; // turn on LED , turn on motor

}

void echoInterrupt() {
  if (digitalRead(ECHO_PIN) == HIGH) {
    start_time = micros();
  } else {
    //if(digitalRead(8) == LOW && digitalRead(9) == LOW){ 
    echo_duration = micros() - start_time;
    //}
  /*  if(digitalRead(10) == LOW && digitalRead(11) == LOW) {
    echo_duration_3 = micros() - start_time;
    }*/
  }
}

void echoInterrupt_2() {
  if (digitalRead(ECHO_PIN_2) == HIGH) {
    start_time_2 = micros();
  } else {
    // if(digitalRead(8) == LOW && digitalRead(9) == LOW){ 
    echo_duration_2 = micros() - start_time_2;
    //}
    /*if(digitalRead(10) == LOW && digitalRead(11) == LOW) {
    echo_duration_4 = micros() - start_time_2;
    }*/
  }
}


/*void echoInterrupt_3() {
  if (digitalRead(ECHO_PIN_3) == HIGH) {
    start_time_3 = micros();
  } else {
    echo_duration_3 = micros() - start_time_3;
  }
}
*/
/*void echoInterrupt_4() {
  if (digitalRead(ECHO_PIN_4) == HIGH) {
    start_time_4 = micros();
  } else {
    echo_duration_4 = micros() - start_time_4;
  }
}
*/