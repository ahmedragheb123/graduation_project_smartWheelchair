#include <Arduino.h>
void initialize_encoder();
float calculating_rpm();
float calculating_rpm_2();
void pid_controller(double setpoint,int dir1, double setpoint_2, int dir2);
void setmotor(int dir,int pwmval,int pwm_pin,int in1);
void readencoder();
void readencoder_2();
float time();
float time_2();
float rpmreq();


