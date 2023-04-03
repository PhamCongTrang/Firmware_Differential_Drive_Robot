#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Arduino.h>


#define ENA 8
#define ENB 9
#define IN1 30
#define IN2 32
#define IN3 34
#define IN4 36
#define A1 18 
#define BB1 19 
#define A2 20 
#define B2 21 

float v; //linear.x (m/s) subcribe
float omega; //angular.z(rad/s) subcribe
float vBack; //linear.x (m/s) publish
float omegaBack;//angular.z(rad/s) publish
unsigned long pret = 0; // Temp of time 
int cycle = 100; // cycle to read encoder & calculate PID (ms)
float vr_set, vl_set; // Speed left & right setting (m/s)
float vr_mea, vl_mea; // Speed left & right measuring (m/s)
int duty_left, duty_right; // Duty of PWM pulse. Range from -255 to 255;
float Kp = 80, Ki = 0, Kd = 0; // PID parameter
float P, I = 0, D; // Value of Proportional Integral Differential
float L = 0.235; // distance between 2 wheel (m)
float r_wheel = 0.05; // radian of wheel (m)
float pre_v_err = 0; // pre v error (m/s)
volatile long cnt_l = 0, cnt_r = 0;
volatile long pre_cnt_l = 0, pre_cnt_r = 0;
int ppr = 330; //pulse per revolution
void velReceived(const geometry_msgs::Twist &msgIn){
    v = msgIn.linear.x;
    omega = msgIn.angular.z;
}

geometry_msgs::Twist velBack;

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> subvel("/cmd_vel",&velReceived);
ros::Publisher pubvel("/velocity_publisher",&velBack);

void encoder_counter_right()
{
    if(digitalRead(BB1) == LOW) cnt_r++;
    else cnt_r--;
}
void encoder_counter_left()
{
    if(digitalRead(B2) == LOW) cnt_l++;
    else cnt_l --;
}

void setup()
{
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    pinMode(A1, INPUT_PULLUP);
    pinMode(B1, INPUT_PULLUP);
    pinMode(A2, INPUT_PULLUP);
    pinMode(B2, INPUT_PULLUP);

    nh.initNode();
    nh.subscribe(subvel);
    nh.advertise(pubvel);

    attachInterrupt(5, encoder_counter_left, RISING);
    attachInterrupt(3, encoder_counter_right, RISING);

}

int measure_speed(long int cnt, long int pre_cnt)
{
    return (cnt - pre_cnt)/ppr*2*3.1415*r_wheel/(cycle/1000);
}
int calculate_vright(float v, float omega)
{
    return v + L*omega/2;
}
int calculate_vleft(float v, float omega)
{
    return v - L*omega/2;
}
/*
* Return duty
* Use both left & right motor
*/
int PID(float v_set, float v_mea)
{
   int v_err = v_set - v_mea;
   P = Kp*v_err;
   I += Ki*v_err*(cycle/100);
   D = (v_err - pre_v_err)/(cycle/100);
   pre_v_err = v_err;
   int duty = P + I + D;
   if (duty > 255) duty = 255;
   if (duty < -255) duty = -255;
   return duty;
}
void hash_PWM(int duty_left, int duty_right)
{
    while ((millis() - pret) < cycle)
    {
        if(duty_left >= 0)
        {
            analogWrite(ENA, duty_left);
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
        }
        if(duty_left < 0)
        {
            analogWrite(ENA, -duty_left);
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
        }
        if(duty_right >= 0)
        {
            analogWrite(ENB, duty_right);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        }
        if(duty_right < 0)
        {
            analogWrite(ENB, -duty_right);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
        }
    }
    pret = millis();
}
void loop()
{
    nh.spinOnce();

    vr_set = calculate_vright(v, omega);
    vl_set = calculate_vleft(v, omega);

    vr_mea = measure_speed(cnt_r, pre_cnt_r);
    pre_cnt_r = cnt_r;
    vl_mea = measure_speed(cnt_l, pre_cnt_l);
    pre_cnt_l = cnt_l;

    duty_right += PID(vr_set, vr_mea);
    duty_left += PID(vl_set, vl_mea);

    hash_PWM(duty_left, duty_right);

    vBack = (vr_mea + vl_mea)/2;
    omegaBack = (vr_mea - vl_mea)/L;

    velBack.linear.x = vBack;
    velBack.angular.z = omegaBack;
    pubvel.publish(&velBack);
    delay(1);
}