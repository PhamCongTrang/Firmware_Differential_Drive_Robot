#include <ros.h>
#include <Arduino.h>

#define ENA 8
#define ENB 9
#define IN1 30
#define IN2 32
#define IN3 34
#define IN4 36
#define A1 18
#define B1 19
#define A2 20
#define B2 21

float v; //angular.x (m/s)
int pret = 0; // Temp of time 
int cycle = 100; // cycle to read encoder & calculate PID (ms)
float vr_set, vl_set; // Speed left & right setting (m/s)
float vr_mea, vl_mea; // Speed left & right measuring (m/s)
int duty_left, duty_right; // Duty of PWM pulse. Range from -100 to 100 (%)
float Kp, Ki, Kd; // PID parameter
float P, I, D; // Value of Proportional Integral Differential


void setup()
{
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}
int calculate_vright(float v, float omega)
{
    
}
int calculate_vleft(float v, float omega)
{

}
int PID(float vr_set, float vr_mea)
{

}
int hash_PWM(int duty_left, int duty_right)
{

}
void loop()
{
    analogWrite(ENA, 160);
    analogWrite(ENB, 160);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN2, LOW);
}