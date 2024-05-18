#include <Arduino.h>
#include <encoder_real.h>
#include <util/atomic.h>
#include <PID_v1.h>

double Setpoint, Input, Output;
double Setpoint_2, Input_2, Output_2;
double Kp=0.5, Ki=1.2, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID_2(&Input_2, &Output_2, &Setpoint_2, Kp, Ki, Kd, DIRECT);


unsigned long prev = 0;
const long interval = 100;


#define enca 2
#define encb 4
#define pwm  7
#define int1 8

#define enca_2 3
#define encb_2 14
#define pwm_2  9
#define int1_2 10


long prevt = 0;
int posprev = 0;
volatile int posi = 0;
float v1_global = 0;

volatile int posi_2 = 0;
int posprev_2 = 0;
float v1_global_2 = 0;

 
 long prevt_2 = 0;

void setmotor(int dir,int pwmval,int pwm_pin,int in1){
  
  analogWrite(pwm_pin,pwmval);
  if (dir==1){
    digitalWrite(in1,LOW);
    }
    else if (dir==-1){
    digitalWrite(in1,HIGH);
    }
  }

void readencoder(){
  int b = digitalRead(encb);
  int incr = 0;
  if (b<1){
    incr = 1;
    }
  else{
    incr = 1;
    }
    posi = posi + incr;
    //Serial.println(posi);
  }
  void readencoder_2(){
  int b_2 = digitalRead(encb_2);
  int incr_2 = 0;
  if (b_2<1){
    incr_2 = 1;
    }
  else{
    incr_2 = 1;
    }
    posi_2 = posi_2 + incr_2;
  }

void initialize_encoder(){
  pinMode(enca,INPUT_PULLUP);
  digitalWrite(enca, HIGH);

  pinMode(encb,INPUT);
  pinMode(pwm,OUTPUT);
  pinMode(int1,OUTPUT);

  pinMode(enca_2,INPUT_PULLUP);
  digitalWrite(enca_2, HIGH);

  pinMode(encb_2,INPUT);
  pinMode(pwm_2,OUTPUT);
  pinMode(int1_2,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(enca),readencoder,RISING);
  attachInterrupt(digitalPinToInterrupt(enca_2),readencoder_2,RISING);

    //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID_2.SetMode(AUTOMATIC);
  
}
float time(){
  long currt = micros();
  float deltat = ((float)(currt-prevt))/1.0e6;
  prevt = currt;
  return deltat;
}
float time_2(){
  long currt_2 = micros();
  float deltat_2 = ((float)(currt_2-prevt_2))/1.0e6;
  prevt_2 = currt_2;
  return deltat_2;
}
float calculating_rpm(){
  int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = posi;
  }
  
  float deltat_encoder = time();
  float velocity1 = (pos-posprev)/deltat_encoder;
  posprev = pos; 
  float v1 = velocity1/600*60.0;
  if(v1 > 120 || v1 < -120){
  }
  else{
    v1_global = v1;
  }

  return v1_global;
}

float calculating_rpm_2(){
  int pos_2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos_2 = posi_2;
  }
  
  float deltat_encoder_2 = time_2();
  float velocity1_2 = (pos_2-posprev_2)/deltat_encoder_2;
  posprev_2 = pos_2; 
  float v1_2 = velocity1_2/600*60.0;
  //Serial.println(v1_2);
  if(v1_2 > 120 || v1_2 < -120){
  }
  else{
    v1_global_2 = v1_2;
  }
  return v1_global_2;
}

void pid_controller(double setpoint,int dir1, double setpoint_2, int dir2){
   unsigned long cur = millis();
  if(cur - prev >= interval){
    prev = cur;
  Setpoint = setpoint;
  Setpoint_2 = setpoint_2;
    

  Input = (double)(calculating_rpm());
  myPID.Compute();
  setmotor(dir1,Output,5,6);

  Input_2 = (double)(calculating_rpm_2());
  myPID_2.Compute();
  setmotor(dir2,Output_2,9,8);
  }
}
float rpmreq(){
  return v1_global;
}