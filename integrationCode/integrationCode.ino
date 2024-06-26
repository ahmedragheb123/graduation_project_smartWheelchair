#include <Arduino.h>
#include <mpu.h>
#include <encoder_real.h>
#include <Ultrasonic.h>


int direction1 = 0;
int direction2 = 0;
float rpm;

int flag_z = 0;

float rollkal = 0;
float pitchkal = 0;
float yawkal = 0;

void setup() {
  Serial.begin(9600);
  initialize_encoder();
  intitialize_mpu();
  ultrasonic_init();

  while (flag_z == 0) {
    MPU_UPDATE();
    yawkal = yaw();
    if (yawkal > 50) {
      flag_z = 1;
    }
  }
}

void loop() {

  MPU_UPDATE();

  float rollkal = roll();
  float pitchkal = pitch();
  float yawkal = yaw();
  Serial.print(rollkal);
  Serial.print(":");
  Serial.print(pitchkal);
  Serial.print(":");
  Serial.print(yawkal);
  float rollabs = abs(rollkal);
  float pitchabs = abs(pitchkal);
  Serial.println();
  rpm = rpmreq();
  //Serial.println(rpm);

  if (flag_z == 1) {
    if (rollabs < pitchabs) {


      // SETTING THE X DIRECTION RANGES OF THE HEADSET
      if (pitchkal > 0 && pitchkal < 90) {  //FORWARD DIRECTION
        if (measureDistanceForward()) {
          //digitalWrite(Buzzer, LOW);
          if ((direction1 == -1 && rpm > 20) || (direction2 == 1 && rpm > 20)) {
            //brakeTheMotors();
            pid_controller(0, direction1, 0, direction2);
            Serial.println("stop");
          } else {
            if (pitchkal > 15 && pitchkal < 25) {
              direction1 = 1;
              direction2 = -1;
              Serial.println("moving forward");
              pid_controller(15, direction1, 15, direction2);
            } else if (pitchkal > 25 && pitchkal < 40) {
              direction1 = 1;
              direction2 = -1;
              pid_controller(20, direction1, 20, direction2);
              Serial.println("moving forward2");
            } else if (pitchkal < 10 && pitchkal > -10) {
              brakeTheMotors();
              //  pid_controller(0, direction1, 0, direction2);
              Serial.println("stop");
            }
          }
        } else {
          //digitalWrite(Buzzer, HIGH);
          brakeTheMotors();
          //pid_controller(0, direction1, 0, direction2);
          Serial.println("stop");
        }
      }

      else if (pitchkal < 0 && pitchkal > -90) {  //BACKWARD DIRECTION
        if (measureDistanceReverse()) {
          //digitalWrite(Buzzer, LOW);
          if ((direction1 == 1 && rpm > 20) || (direction2 == -1 && rpm > 20)) {
            //brakeTheMotors();
            pid_controller(0, direction1, 0, direction2);
            Serial.println("stop");
          } else {
            if (pitchkal < -15 && pitchkal > -30) {
              direction1 = -1;
              direction2 = 1;
              pid_controller(15, direction1, 15, direction2);
              Serial.println("moving backward");
            } else if (pitchkal < -30 && pitchkal > -55) {
              direction1 = -1;
              direction2 = 1;
              pid_controller(20, direction1, 20, direction2);
              Serial.println("moving backward2");
            } else if (pitchkal < 10 && pitchkal > -10) {
              brakeTheMotors();
              // pid_controller(0, direction1, 0, direction2);
              Serial.println("stop");
            }
          }
        } else {
          //digitalWrite(Buzzer, HIGH);
          brakeTheMotors();
          // pid_controller(0, direction1, 0, direction2);
          Serial.println("stop");
        }
      }
    } else {

      //digitalWrite(Buzzer, LOW);

      // SETTING THE Y DIRECTION RANGES OF THE HEADSET
      if (rollkal < 0 && rollkal > -70) {  //LEFT DIRECTION
        if ((direction1 == -1 && rpm > 20) || (direction2 == -1 && rpm > 20)) {
          //brakeTheMotors();
          pid_controller(0, direction1, 0, direction2);
          Serial.println("stop");
        } else {
          if (rollkal < -15 && rollkal > -30) {
            direction1 = 1;
            direction2 = 1;
            pid_controller(10, direction1, 10, direction2);
            Serial.println("moving left");
          } else if (rollkal < -30 && rollkal > -55) {
            direction1 = 1;
            direction2 = 1;
            pid_controller(15, direction1, 15, direction2);
            Serial.println("moving left");
          } else if (rollkal < 10 && rollkal > -10) {
            brakeTheMotors();
            // pid_controller(0, direction1, 0, direction2);
            Serial.println("stop");
          }
        }
      } else if (rollkal > 0 && rollkal < 70) {  //RIGHT DIRECTION
        if ((direction1 == 1 && rpm > 20) || (direction2 == 1 && rpm > 20)) {
          //brakeTheMotors();
          pid_controller(0, direction1, 0, direction2);
          Serial.println("stop");
        } else {
          if (rollkal > 15 && rollkal < 30) {
            direction1 = -1;
            direction2 = -1;
            pid_controller(10, direction1, 10, direction2);
            Serial.println("moving right");
          } else if (rollkal > 30 && rollkal < 55) {
            direction1 = -1;
            direction2 = -1;
            pid_controller(15, direction1, 15, direction2);
            Serial.println("moving right");
          } else if (rollkal < 10 && rollkal > -10) {
            brakeTheMotors();
            // pid_controller(0, direction1, 0, direction2);
            Serial.println("stop");
          }
        }
      }
    }
  }
}
