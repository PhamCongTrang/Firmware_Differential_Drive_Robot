#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Arduino.h>

#define ENA 9
#define ENB 8

#define IN1 30
#define IN2 32
#define IN3 36
#define IN4 34

#define CA1 21
#define CA1I 2 //Interup 1
#define CB1 20
#define CA2 18
#define CA2I 5 //Interup 2
#define CB2 19

float v; //linear.x (m/s) subcribe
float omega; //angular.z(rad/s) subcribe
float vBack; //linear.x (m/s) publish
float omegaBack;//angular.z(rad/s) publish
unsigned long pret = 0; // Temp of time 
int cycle = 400; // cycle to read encoder & calculate PID (ms)
float vr_set, vl_set; // Speed left & right setting (m/s)
float vr_mea, vl_mea; // Speed left & right measuring (m/s)
int duty_left, duty_right; // Duty of PWM pulse. Range from -255 to 255;
float Kp = 12, Ki = 1, Kd = 4; // PID parameter
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
    if(digitalRead(CB2) == LOW) cnt_r++;
    else cnt_r--;
}
void encoder_counter_left()
{
    if(digitalRead(CB1) == LOW) cnt_l++;
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

    pinMode(18, INPUT_PULLUP);
    pinMode(19, INPUT_PULLUP);
    pinMode(20, INPUT_PULLUP);
    pinMode(21, INPUT_PULLUP);

    nh.initNode();
    nh.subscribe(subvel);
    nh.advertise(pubvel);

    attachInterrupt(CA1I, encoder_counter_left, RISING); 
    attachInterrupt(CA2I, encoder_counter_right, RISING); 

    Serial.begin(9600);
}

float measure_speed(long int cnt, long int pre_cnt)
{
    return (cnt - pre_cnt)/(ppr+0.0)*2*3.1415*r_wheel/(cycle/1000.0);
}
float calculate_vright(float v, float omega)
{
    return (v + L*omega/2);
}
float calculate_vleft(float v, float omega)
{
    return (v - L*omega/2);
}
/*
* Return duty
* Use both left & right motor
*/
int PID(float v_set, float v_mea)
{
   float v_err = v_set - v_mea;
   P = Kp*v_err;
   I += Ki*v_err*(cycle/1000);
   D = Kd*(v_err - pre_v_err)/(cycle/1000.0);
   pre_v_err = v_err;
   int duty = (int) (P + I + D);

   return duty;
}
void hash_PWM(int duty_left, int duty_right)
{
    //duty_left = (int) duty_left*1.2;
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
    //delay(cycle);
    pret = millis();
}
void loop()
{
    //nh.spinOnce();
    // delay(1);
    v = 2;
    omega = 0;
    Serial.print("cnt_l, cnt_r"); Serial.print(cnt_l); Serial.print("  "); Serial.println(cnt_r);

    vr_set = calculate_vright(v, omega);
    vl_set = calculate_vleft(v, omega);
    Serial.print("vl_set, vr_set"); Serial.print(vl_set); Serial.print("  "); Serial.println(vr_set);

    vr_mea = measure_speed(cnt_r, pre_cnt_r);
    pre_cnt_r = cnt_r;
    vl_mea = measure_speed(cnt_l, pre_cnt_l);
    pre_cnt_l = cnt_l;
    Serial.print("vl_mea, vr_mea"); Serial.print(vl_mea); Serial.print("  "); Serial.println(vr_mea); 
    
    duty_left += PID(vl_set, vl_mea);
    duty_right += PID(vr_set, vr_mea);
    // duty_left +=(int) 20*(vl_set - vl_mea);
    // duty_right +=(int) 20*(vr_set - vr_mea);

    if (duty_left > 100) duty_left = 100;
        if (duty_left < -100) duty_left = -100;
    if (duty_right > 100) duty_right = 100;
        if (duty_right < -100) duty_right = -100;

    Serial.print("duty_left, duty_right"); Serial.print(duty_left); Serial.print("  "); Serial.println( duty_right);
    Serial.print("Time calculate"); Serial.println(millis() - pret);
    hash_PWM(duty_left, duty_right);

    vBack = (vr_mea + vl_mea)/2;
    omegaBack = (vr_mea - vl_mea)/L;

    velBack.linear.x = vBack;
    velBack.angular.z = omegaBack;
    pubvel.publish(&velBack);
    Serial.println("-------------------");
    //delay(1);
}