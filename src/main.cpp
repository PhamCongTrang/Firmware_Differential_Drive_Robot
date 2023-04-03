#include <ros.h>
#include <Arduino.h>

#define ENA 8
#define ENB 9
#define IN1 30
#define IN2 32
#define IN3 34
#define IN4 36

void setup()
{
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void loop()
{
    analogWrite(ENA, 160);
    analogWrite(ENB, 160);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN2, LOW);
    delay(100);
}