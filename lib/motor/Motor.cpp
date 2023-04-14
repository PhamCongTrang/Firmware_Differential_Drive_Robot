#include "Arduino.h"
#include "Motor.h"

/*plus,minus,en_a,en_b*/
Motor::Motor(int plus, int minus,int pwm, int en_a, int en_b) {
  pinMode(plus,OUTPUT);
  pinMode(minus,OUTPUT);
  pinMode(pwm,OUTPUT);
  pinMode(en_a,INPUT_PULLUP);
  pinMode(en_b,INPUT_PULLUP);
  Motor::plus = plus;
  Motor::minus = minus;
  Motor::pwm = pwm;
  Motor::en_a = en_a;
  Motor::en_b = en_b;
  
}

void Motor::rotate(int value) {
  if(value>=0){
    //Max Voltage with 16V battery with 12V required
    //(12/16)*255 ~=190
//    Serial.println("called");
//    Serial.println(plus);
    int out = map(value, 0, 100, 0, 100);
    analogWrite(pwm, out);
    digitalWrite(plus, HIGH);
    digitalWrite(minus, LOW);
  }else{
    //Max Voltage with 16V battery with 12V required
    //(12/16)*255 ~=190
    int out = map(value, 0, -100, 0, 100);
    analogWrite(pwm, out);
    digitalWrite(plus, LOW);
    digitalWrite(minus, HIGH);
  }
}
