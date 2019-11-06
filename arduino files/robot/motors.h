

#include <math.h>

#define motorA_in1 23//22
#define motorA_in2 25//24
#define motorA_pwm 9//8

#define motorB_in1 26//25
#define motorB_in2 27//23
#define motorB_pwm 10//9

#define motorC_in1 22//-1//26
#define motorC_in2 24//-1//27
#define motorC_pwm 8//-1//10


int A_speed = 0 , B_speed = 0 , C_speed = 0;


const float pi = 3.14159267;
//float theta = 270.0;
//float mag = 0;


void motors_setup(){
  
  pinMode(motorA_in1,OUTPUT);
  pinMode(motorA_in2,OUTPUT);
  pinMode(motorB_in1,OUTPUT);
  pinMode(motorB_in2,OUTPUT);
  pinMode(motorC_in1,OUTPUT);
  pinMode(motorC_in2,OUTPUT);
}



void motorA_move(int speed){
  speed = constrain(speed,-1023,1023);
  if(speed>0){
    analogWrite(motorA_pwm,speed);
    digitalWrite(motorA_in2,HIGH);
    digitalWrite(motorA_in1,LOW);
  }else{
    analogWrite(motorA_pwm,-speed);
    digitalWrite(motorA_in1,HIGH);
    digitalWrite(motorA_in2,LOW);
  }
}

void motorB_move(int speed){
  speed = constrain(speed,-1023,1023);
  if(speed>0){
    analogWrite(motorB_pwm,speed);
    digitalWrite(motorB_in2,HIGH);
    digitalWrite(motorB_in1,LOW);
  }else{
    analogWrite(motorB_pwm,-speed);
    digitalWrite(motorB_in1,HIGH);
    digitalWrite(motorB_in2,LOW);
  }
}

void motorC_move(int speed){
  speed = constrain(speed,-1023,1023);
  if(speed>0){
    analogWrite(motorC_pwm,speed);
    digitalWrite(motorC_in2,HIGH);
    digitalWrite(motorC_in1,LOW);
  }else{
    analogWrite(motorC_pwm,-speed);
    digitalWrite(motorC_in1,HIGH);
    digitalWrite(motorC_in2,LOW);
  }
}
