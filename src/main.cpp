#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <Arduino.h>

#define ENA 8
#define ENB 9
#define IN1 30
#define IN2 32
#define IN3 34
#define IN4 36

#define BUTTON 8
#define LED 13

ros::NodeHandle node_handle;

std_msgs::String button_msg;
std_msgs::UInt16 led_msg;

void subscriberCallback(const std_msgs::UInt16 &led_msg)
{
    if (led_msg.data == 1)
    {
        digitalWrite(LED, HIGH);
    }
    else
    {
        digitalWrite(LED, LOW);
    }
}

ros::Publisher button_publisher("button_press", &button_msg);
ros::Subscriber<std_msgs::UInt16> led_subscriber("toggle_led", &subscriberCallback);

void setup()
{
    pinMode(LED, OUTPUT);
    pinMode(BUTTON, INPUT);

    node_handle.initNode();
    node_handle.advertise(button_publisher);
    node_handle.subscribe(led_subscriber);

    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
}

void loop()
{
    if (digitalRead(BUTTON) == HIGH)
    {
        button_msg.data = "Pressed";
    }
    else
    {
        button_msg.data = "NOT pressed";
    }

    button_publisher.publish(&button_msg);
    node_handle.spinOnce();

    analogWrite(ENA, 160);
    analogWrite(ENB, 160);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN2, LOW);
    delay(100);
}