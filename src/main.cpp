//AGV Machine - Vinay Lanka

//Import Motor - Cytron SPG30E-30K
#include <Motor.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

ros::NodeHandle  nh;

#define LOOPTIME 10

Motor right(36,34,8,19,18);
// Motor left(30,32,9,21,20);
Motor left(30,32,9,2,3);

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

double left_kp = 4 , left_ki = 0 , left_kd = 0.0;             // modify for optimal performance
double right_kp = 3.85 , right_ki = 0 , right_kd = 0.0;

double right_input = 0, right_output = 0, right_setpoint = 0;
PID rightPID(&right_input, &right_output, &right_setpoint, right_kp, right_ki, right_kd, DIRECT);  

double left_input = 0, left_output = 0, left_setpoint = 0;
PID leftPID(&left_input, &left_output, &left_setpoint, left_kp, left_ki, left_kd, DIRECT);  

float demandx=0;
float demandz=0;

double demand_speed_left;
double demand_speed_right;

unsigned long currentMillis;
unsigned long prevMillis;

float encoder0Diff;
float encoder1Diff;

float encoder0Error;
float encoder1Error;

float encoder0Prev;
float encoder1Prev;

void cmd_vel_cb( const geometry_msgs::Twist& twist){
  demandx = twist.linear.x;
  demandz = twist.angular.z*2;
}

// ************** encoder 1 *********************


void change_left_a(){  

  // look for a low-to-high on channel A
  if (digitalRead(left.en_a) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(left.en_b) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(left.en_b) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
 
}

void change_left_b(){  

  // look for a low-to-high on channel B
  if (digitalRead(left.en_b) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(left.en_a) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(left.en_a) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  
}

// ************** encoder 2 *********************

void change_right_a(){  

  // look for a low-to-high on channel A
  if (digitalRead(right.en_a) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(right.en_b) == LOW) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(right.en_b) == HIGH) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
}

void change_right_b(){  

  // look for a low-to-high on channel B
  if (digitalRead(right.en_b) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(right.en_a) == HIGH) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(right.en_a) == LOW) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb );
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_act_right = 0;                    //Command speed for left wheel in m/s 

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
  Serial.begin(57600);
  
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetSampleTime(1);
  rightPID.SetOutputLimits(-100, 100);

  leftPID.SetMode(AUTOMATIC);
  leftPID.SetSampleTime(1);
  leftPID.SetOutputLimits(-100, 100);
  
//  Serial.println("Basic Encoder Test:");
  attachInterrupt(digitalPinToInterrupt(left.en_a),change_left_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left.en_b),change_left_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right.en_a),change_right_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right.en_b),change_right_b, CHANGE);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
    mpu6050.update();
    currentMillis = millis();
    if (currentMillis - prevMillis >= LOOPTIME){
        prevMillis = currentMillis;

    demandx = 0.5;
    demandz = 3.14/2;

    demand_speed_left = demandx - (demandz*0.1175);
    demand_speed_right = demandx + (demandz*0.1175);
  
    /*PID controller for speed control
      Base speed being 1 ms and the demand_speed variables controlling it at fractions of the base.
      The PID controller keeps trying to match the difference 
      in encoder counts to match with the required amount, hence controlling the speed. */
    encoder0Diff = encoder0Pos - encoder0Prev; // Get difference between ticks to compute speed
    encoder1Diff = encoder1Pos - encoder1Prev;
    
    speed_act_left = encoder0Diff/39.65;                    
    speed_act_right = encoder1Diff/39.65; 
  
    encoder0Error = (demand_speed_left*39.65)-encoder0Diff; // 3965 ticks in 1m = 39.65 ticks in 10ms, due to the 10 millis loop
    encoder1Error = (demand_speed_right*39.65)-encoder1Diff;
  
    encoder0Prev = encoder0Pos; // Saving values
    encoder1Prev = encoder1Pos;
  
    left_setpoint = demand_speed_left*39.65;  //Setting required speed as a mul/frac of 1 m/s
    right_setpoint = demand_speed_right*39.65;
  
    left_input = encoder0Diff;  //Input to PID controller is the current difference
    right_input = encoder1Diff;
    
    leftPID.Compute();
    left.rotate(left_output);
    rightPID.Compute();
    right.rotate(right_output);
    if(millis() % 10 == 0)
    {
      Serial.print("LEFT:");
      Serial.print(speed_act_left);
      Serial.print(",");
      Serial.print("RIGHT:");
      Serial.print(speed_act_right);
      Serial.print(",");
      Serial.print("OMEGA:");
      Serial.print((speed_act_right-speed_act_left)/0.235);
      Serial.print(",");
      Serial.print("OMEGAGRYSCOPE:");
      Serial.println(mpu6050.getGyroZ()/180*3.14);

      // Gia toc dai do bang acc
      // Serial.print("accX:");Serial.print(mpu6050.getAccX());Serial.print(",");
      // Serial.print("accY:");Serial.print(mpu6050.getAccY());Serial.print(",");
      // Serial.print("accZ:");Serial.println(mpu6050.getAccZ());
      // Van toc goc do bang gyro
      // Serial.print("gyroX:");Serial.print(mpu6050.getGyroX());Serial.print(",");
      // Serial.print("gyroY:");Serial.print(mpu6050.getGyroY());Serial.print(",");
      // Serial.print("gyroZ:");Serial.println(mpu6050.getGyroZ());
      // Goc lech truc so voi trong luc, do bang acc
      // Serial.print("accAngleX:");Serial.print(mpu6050.getAccAngleX());Serial.print(",");
      // Serial.print("accAngleY:");Serial.println(mpu6050.getAccAngleY());
      // Goc lech so voi phuong ban dau - co tich luy, do bang gry
      // Serial.print("gyroAngleX:");Serial.print(mpu6050.getGyroAngleX());Serial.print(",");
      // Serial.print("gyroAngleY:");Serial.print(mpu6050.getGyroAngleY());Serial.print(",");
      // Serial.print("gyroAngleZ:");Serial.println(mpu6050.getGyroAngleZ());
      // Goc lech so voi phuong ban dau co tich luy, acc + gyro
      // Serial.print("angleX:");Serial.print(mpu6050.getAngleX());Serial.print(",");
      // Serial.print("angleY:");Serial.print(mpu6050.getAngleY());Serial.print(",");
      // Serial.print("angleZ:");Serial.println(mpu6050.getAngleZ());
      
    }

//    Serial.print(encoder0Pos);
//    Serial.print(",");
//    Serial.println(encoder1Pos);
  }
  // publishSpeed(LOOPTIME);
  // nh.spinOnce();
}


//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  
//  nh.loginfo("Publishing odometry");
}


// ************** encoders interrupts **************
