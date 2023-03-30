#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

#define BUTTON 8
#define LED 13

ros::NodeHandle node_handle;

std_msgs::String button_msg;
std_msgs::UInt16 led_msg;

void subscriberCallback(const std_msgs::UInt16& led_msg) {
  if (led_msg.data  == 1) {
    digitalWrite(LED, HIGH); 
  } else {
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
}

void loop()
{ 
  if (digitalRead(BUTTON) == HIGH) {
    button_msg.data = "Pressed";
  } else {
    button_msg.data = "NOT pressed";
  }

  button_publisher.publish( &button_msg );
  node_handle.spinOnce();
  
  delay(100);
}