#include "TB6612FNG.h"

Motors::Motors(int STBY, int PWMA, int AIN1, int AIN2, int PWMB, int BIN1, int BIN2){
  Motors::STBY = STBY;
  Motors::PWMA = PWMA;
  Motors::AIN1 = AIN1;
  Motors::AIN2 = AIN2;
  Motors::PWMB = PWMB;
  Motors::BIN1 = BIN1;
  Motors::BIN2 = BIN2;
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
}
void Motors::forward(int speed){
  move(1, speed, 1);
  move(2, speed, 1);
}
void Motors::reverse(int speed){
  move(1, speed, -1);
  move(2, speed, -1);
}
void Motors::right(int speed){
  move(1, 255, 1);
  move(2, 255, -1);
}
void Motors::left(int speed){
  move(1, 255, -1);
  move(2, 255, 1);
}
void Motors::stop(){
  digitalWrite(STBY, LOW);
  //digitalWrite(D4, LOW);
}
void Motors::move(int motor, int speed, int direction){
  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }
  if(motor == 1){
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }else{
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }
}
