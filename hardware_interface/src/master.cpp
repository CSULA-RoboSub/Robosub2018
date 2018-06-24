/*
This is a quick translation of the old arduino code from level_with_motors.ino to c++
hardware_interface is a much faster version of the arduino node.
Arduino cannot operate as a node that receives the amount of data we send

*/

#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <robosub/HControl.h>
#include <robosub/RControl.h>
#include <robosub/MControl.h>
#include <pathfinder_dvl/DVL.h>
#include <ez_async_data/Rotation.h>
#include <hardware_interface/MotorHorizontal.h>
#include <hardware_interface/MotorVertical.h>
#include <string>
#include <cmath>

using namespace std;

//CHANGED PWM_Motors TO PWM_Motors_depth SINCE THERE ARE 2 DIFFERENT PWM CALCULATIONS
//ONE IS FOR DEPTH AND THE OTHER IS USED FOR MOTORS TO ROTATE TO PROPER NEW LOCATION
// int PWM_Motors_Depth;
// float dutyCycl_depth;
float assignedDepth;
float feetDepth_read;

//initializations for IMU
float pitch, yaw, roll, heading;
// bool firstIMUReading;

int i;
int PWM_Motors_orient;
float rotatePowerMax = 120;
float abias[3] = {0, 0, 0}, gbias[3] = {0, 0, 0};
float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method
float temperature;
float dutyCycl_orient;
float assignedYaw;

//Initialize ROS node
//const float rotationUpperBound = 166.2;
//const float rotationLowerBound = -193.8;
const float rotationUpperBound = 179.99;
const float rotationLowerBound = -179.99;
const float topDepth = 0.5;
const float bottomDepth = 12;
const float motorMax = 1700;
const float motorMin = 1300;
int mControlDirection;
float mControlPower;
float rControlPower;
float mControlDistance;
float mControlRunningTime;
float mControlMode5Timer;
// float frontCamForwardDistance;
// float frontCamHorizontalDistance;
// float frontCamVerticalDistance;
// float bottomCamForwardDistance;
// float bottomCamHorizontalDistance;
// float bottomCamVerticalDistance;
float centerTimer;
float rotationTimer;
float rotationTime;
// float movementTimer;
float movementTime;
bool subIsReady;
bool isGoingUp;
bool isGoingDown;
bool isTurningRight;
bool isTurningLeft;
bool keepTurningRight;
bool keepTurningLeft;
bool rControlMode3;
bool rControlMode4;
bool mControlMode1;
bool mControlMode2;
bool mControlMode3;
bool mControlMode4;
bool mControlMode5;
int mControlMode5Direction;
bool keepMovingForward;
bool keepMovingRight;
bool keepMovingBackward;
bool keepMovingLeft;

//Testing-----------------------
float positionXPrev = 0;
float positionYPrev = 0;

float positionX = 0;
float positionY = 0;
float positionZ = 0;
float velocityX = 0;
float velocityY = 0;
float velocityZ = 0;

std_msgs::Float32 currentDepth;
robosub::HControl hControlStatus;
robosub::RControl rControlStatus;
robosub::MControl mControlStatus;
hardware_interface::MotorVertical mVertical;
hardware_interface::MotorHorizontal mHorizontal;

ros::Publisher hControlPublisher;     //int: state, float: depth
ros::Publisher rControlPublisher;   //int: state, float: rotation
ros::Publisher mControlPublisher;   //int: state, int: direction, float: distance
ros::Publisher currentDepthPublisher;           //float: depth
ros::Publisher mHorizontalPublisher;
ros::Publisher mVerticalPublisher;

ros::Subscriber currentDepthSubscriber;
ros::Subscriber hControlSubscriber;   //int: state, float: depth
ros::Subscriber rControlSubscriber; //int: state, float: rotation
ros::Subscriber mControlSubscriber;
ros::Subscriber rotationSubscriber;
ros::Subscriber dvlSubscriber;

//depth control variables
// int pwm_submerge = 200;
// int pwm_emerge = 150;
float hControlPower;
//thrusters off value does not change
const float base_thrust = 1500;

//base thrust variables to be offset by base_thrust
// int base_thrust_1 = base_thrust;
// int base_thrust_2 = base_thrust;
// int base_thrust_3 = base_thrust;
// int base_thrust_4 = base_thrust;

//time variables
double elapsedTime, timeCur, timePrev, loopTime, loopTimePrev;
const int loopInterval = 20;

//the variable error will store the difference between the real_value_angle form IMU and desired_angle of 0 degrees. 
// float PID_pitch, PID_roll, pwmThruster_2, pwmThruster_1, pwmThruster_3, pwmThruster_4, error_roll, prev_error_roll = 0,error_pitch, prev_error_pitch = 0;
float PID_pitch, PID_roll, PID_depth, pwmThruster_1, pwmThruster_2, pwmThruster_3, pwmThruster_4, 
error_roll, prev_error_roll=0, error_pitch, prev_error_pitch=0, error_depth, prev_error_depth=0;

//////////////PID_pitch variables////////////////////
float pid_p_pitch=0;
float pid_d_pitch=0;
float pid_i_pitch=0;
/////////////////PID_pitch constants/////////////////
double kp_pitch=2;//11;//3.55;//3.55
double kd_pitch=0.7;//0.75;//2.05;//2.05
double ki_pitch=0.0003;//0.003
///////////////////////////////////////////////

//////////////PID_roll variables////////////////////
float pid_p_roll=0;
float pid_d_roll=0;
float pid_i_roll=0;
/////////////////PID_roll constants/////////////////
double kp_roll=2;//11;//3.55;//3.55
double kd_roll=0.7;//0.75;//2.05;//2.05
double ki_roll=0.0003;//0.003
///////////////////////////////////////////////

//////////////PID_depth variables////////////////////
float pid_p_depth=0;
float pid_d_depth=0;
float pid_i_depth=0;
/////////////////PID_depth constants/////////////////
double kp_depth=500;//11;//3.55;//3.55
double kd_depth=4.1;//0.75;//2.05;//2.05
double ki_depth=0.003;//0.003
///////////////////////////////////////////////

//double thrust=base_thrust; //initial value of thrust to the thrusters
float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady
//// threshold for going_up, going_down and hoover
float threshold = 0.1;
///////////////////////////////////////////////

float degreeToTurn();
void rotateRightDynamically();
void rotateLeftDynamically();

//returns time in milliseconds
double millis(){
  return ros::WallTime::now().toNSec()*1e-6;
}

//pass in -1 to ignore that motor
void publishMVertical(const float t1, const float t2, const float t3, const float t4){
  if(t1 > 0)
    mVertical.t1 = t1;
  if(t2 > 0)
    mVertical.t2 = t2;
  if(t3 > 0)
    mVertical.t3 = t3;
  if(t4 > 0)
    mVertical.t4 = t4;

}
//pass in -1 to ignore that motor
void publishMHorizontal(const float t5, const float t6, const float t7, const float t8){
  if(t5 > 0)
    mHorizontal.t5 = t5;
  if(t6 > 0)
    mHorizontal.t6 = t6;
  if(t7 > 0)
    mHorizontal.t7 = t7;
  if(t8 > 0)
    mHorizontal.t8 = t8;

}

void publishMotors(){
  mVerticalPublisher.publish(mVertical);
  mHorizontalPublisher.publish(mHorizontal);
}

void rotationCallback(const ez_async_data::Rotation& rotation){
  yaw = rotation.yaw;
  roll = -rotation.roll;
  pitch = -rotation.pitch;
}

void dvlCallback(const pathfinder_dvl::DVL& dvl_status){
  positionX = dvl_status.xpos;
  positionY = dvl_status.ypos;
  positionZ = dvl_status.zpos;

  velocityX = dvl_status.xvel;
  velocityY = dvl_status.yvel;
  velocityZ = dvl_status.zvel;
}

void hControlCallback(const robosub::HControl& hControl) {
  int hState = hControl.state;
  float hDepth = hControl.depth;
  float depth = hControl.depth;
  // char depthChar[6];
  // dtostrf(depth, 4, 2, depthChar);
  hControlPower = hControl.power;

  if(hControl.state == 0){
    if(!isGoingUp && !isGoingDown){
      if(depth == -1 || depth + assignedDepth >= bottomDepth)
        assignedDepth = bottomDepth;
      else
        assignedDepth = assignedDepth + depth;
      isGoingDown = true;
      ROS_INFO("Going down...");
      // ROS_INFO(depthChar);
      // ROS_INFO("ft...(-1 means infinite)\n");
    }else
      ROS_INFO("Sub is still running. Command abort.");
  }
  else if(hControl.state == 1){
    if(isGoingUp || isGoingDown){
      assignedDepth = feetDepth_read;
      isGoingUp = false;
      isGoingDown = false;
      ROS_INFO("Height control is now cancelled\n");
    }
    // ROS_INFO();
    // ROS_INFO("assignedDepth:");
    // ROS_INFO(assignedDepthChar);
    // ROS_INFO("feetDepth_read:");
    // ROS_INFO(feetDepth_readChar);
  }
  else if(hControl.state == 2){
    if(!isGoingUp && !isGoingDown){
      if(depth == -1 || depth >= assignedDepth - topDepth)
        assignedDepth = topDepth;
      else
        assignedDepth = assignedDepth - depth;
      isGoingUp = true;
      ROS_INFO("Going up...");
      // ROS_INFO(depthChar);
      // ROS_INFO("ft...(-1 means infinite)\n");
    }else
      ROS_INFO("Sub is still running. Command abort.");
  }
  else if(hControl.state == 4){
    assignedYaw = yaw;
    assignedDepth = feetDepth_read;
    if(!subIsReady){
      subIsReady = true;
      ROS_INFO("Motors unlocked.");
    }
    //assignedDepth = 0.1;
  }
  else if(hControl.state == 5){
    if(subIsReady){
      subIsReady = false;
      ROS_INFO("Motors are locked.");

    }
    assignedDepth = feetDepth_read;
//    assignedDepth = 0.2;
    subIsReady = false;
    // T1.writeMicroseconds(base_thrust);
    // T2.writeMicroseconds(base_thrust);
    // T3.writeMicroseconds(base_thrust);
    // T4.writeMicroseconds(base_thrust);
    // T5.writeMicroseconds(base_thrust);
    // T6.writeMicroseconds(base_thrust);
    // T7.writeMicroseconds(base_thrust);
    // T8.writeMicroseconds(base_thrust);
    publishMVertical(base_thrust,base_thrust,base_thrust,base_thrust);
    publishMHorizontal(base_thrust,base_thrust,base_thrust,base_thrust);
  }
  hControlStatus.state = hState;
  hControlStatus.depth = hDepth;
  hControlStatus.power = hControlPower;
  hControlPublisher.publish(hControlStatus);

}


void rControlCallback(const robosub::RControl& rControl){

  float rotation = rControl.rotation;
  // char rotationChar[11];
  // dtostrf(rotation, 4, 2, rotationChar);
  rControlPower = rControl.power;

  if(rControl.state == 0){
    if(!isTurningRight && !isTurningLeft && !rControlMode3 && !rControlMode4){
      if(rotation == -1) {
        keepTurningLeft = true;
        //Testing---------------------------------------------------
        rotationTimer = 0;
      }
      else{
        if (yaw - rotation <= rotationLowerBound)
          assignedYaw = yaw - rotation + 360;
        else
          assignedYaw = yaw - rotation;
      }
      isTurningLeft = true;
      ROS_INFO("Turning left...");
      // ROS_INFO(rotationChar);
      // ROS_INFO("degree...(-1 means infinite)\n");
    }else
      ROS_INFO("Sub is still rotating. Command abort.");
  }
  else if(rControl.state == 1){
    if(isTurningRight || isTurningLeft || keepTurningRight || keepTurningLeft || rControlMode3 || rControlMode4){
      isTurningRight = false;
      isTurningLeft = false;
      keepTurningRight = false;
      keepTurningLeft = false;
      rControlMode3 = false;
      rControlMode4 = false;
      assignedYaw = yaw;
      ROS_INFO("Rotation control is now cancelled\n");
      rControlStatus.state = 1;
      rControlStatus.rotation = 0;
      rControlStatus.power = rControlPower;
      rControlPublisher.publish(rControlStatus);
    }
  }
  else if(rControl.state == 2){
    if(!isTurningRight && !isTurningLeft && !rControlMode3 && !rControlMode4){
      if(rotation == -1) {
        keepTurningRight = true;
        //Testing---------------------------------------------------------------
        rotationTimer = 0;
      }
      else{
        if (yaw + rotation > rotationUpperBound)
          assignedYaw = yaw + rotation - 360;
        else
          assignedYaw = yaw + rotation;
      }
      isTurningRight = true;
      ROS_INFO("Turning right...");
      // ROS_INFO(rotationChar);
      // ROS_INFO("degree...(-1 means infinite)\n");
    }else
      ROS_INFO("Sub is still rotating.Command abort.");
  }
  else if(rControl.state == 3){
    if(!isTurningRight && !isTurningLeft && !rControlMode3 && !rControlMode4){
      rControlMode3 = true;
      rotationTime = 3;
      rotationTimer = 0;
      ROS_INFO("Rotate with front camera horizontal distance.\n");
    }
    else
      ROS_INFO("Sub is still rotating.Command abort.");
  }
  else if(rControl.state == 4){
    if(!isTurningRight && !isTurningLeft && !rControlMode3 && !rControlMode4){
      rControlMode4 = true;
      ROS_INFO("Keep rotating with front camera horizontal distance.\n");
    }
    else
      ROS_INFO("Sub is still rotating.Command abort.");
    
  }
  // rControlStatus.state = rControl.state;
  // rControlStatus.rotation = rControl.rotation;
  // rControlStatus.power = rControlPower;
  // rControlPublisher.publish(rControlStatus);

}

// MControl
// state => 0: off, 1: on with power, 2: on with distance, 3: centered with front cam, 4: centered with bottom cam
// direction => 0: none, 1: forward, 2: right, 3: backward, 4: left
// power => 0: none, x: x power add to the motors
// distance => 0: none, x: x units away from the object
void mControlCallback(const robosub::MControl& mControl){
  float power = mControl.power;
  float distance = mControl.distance;
  float mode5Time = mControl.runningTime;

  string directionStr;
  // char powerChar[11];
  // char distanceChar[11];
  // char timeChar[11];
  // dtostrf(power, 4, 2, powerChar);
  // dtostrf(distance, 4, 2, distanceChar);
  // dtostrf(mode5Time, 4, 2, timeChar);


  if(mControl.state == 0){
    if(mControlMode1 || mControlMode2|| mControlMode3 || mControlMode4 || mControlMode5){
      // T6.writeMicroseconds(base_thrust);
      // T8.writeMicroseconds(base_thrust);
      // T5.writeMicroseconds(base_thrust);
      // T7.writeMicroseconds(base_thrust);
      publishMHorizontal(base_thrust,base_thrust,base_thrust,base_thrust);
      mControlMode1 = false;
      mControlMode2 = false;
      mControlMode3 = false;
      mControlMode4 = false;
      mControlMode5 = false;
      keepMovingForward = false;
      keepMovingRight = false;
      keepMovingBackward = false;
      keepMovingLeft = false;
      ROS_INFO("Movement control is now cancelled\n");
    }
    mControlDirection = 0;
    mControlPower = 0;
    mControlDistance = 0;
  }
  else if(mControl.state == 1){
    if(mControlMode1 || mControlMode2|| mControlMode3 || mControlMode4 || mControlMode5)
      ROS_INFO("Sub is still moving. Command abort.");
    else if(mControl.mDirection != 1 && mControl.mDirection != 2 && mControl.mDirection != 3 && mControl.mDirection != 4)
      ROS_INFO("Invalid direction with state 1. Please check the program and try again.\n");
    else if(mControl.power == 0)
      ROS_INFO("Invalid power with state 1. Please check the program and try again.");
    else{
      if(mControl.mDirection == 1){
        keepMovingForward = true;
        directionStr = "forward";
      }
      else if(mControl.mDirection == 2){
        keepMovingRight = true;
        directionStr = "right";
      }
      else if(mControl.mDirection == 3){
        keepMovingBackward = true;
        directionStr = "backward";
      }
      else if(mControl.mDirection == 4){
        keepMovingLeft = true;
        directionStr = "left";
      }

      directionStr = "Moving " + directionStr + " with power...";
      ROS_INFO("%s", directionStr.c_str());
      // ROS_INFO(powerChar);
      // ROS_INFO("...\n");

      //Testing -----------------------------------------------------------
      // movementTimer = 0;
      mControlMode1 = true;
      mControlDirection = mControl.mDirection;
      mControlPower = mControl.power;
      mControlDistance = 0;
    }
  }
  else if(mControl.state == 2){
    if(mControlMode1 || mControlMode2|| mControlMode3 || mControlMode4 || mControlMode5)
      ROS_INFO("Sub is still moving. Command abort.");
    // else if(mControl.mDirection != 1)
    //   ROS_INFO("Invalid direction with state 2. Please check the program and try again.\n");
    else{
      if(mControl.mDirection == 1){
        directionStr = "forward";
      }
      else if(mControl.mDirection == 2){
        directionStr = "right";
      }
      else if(mControl.mDirection == 3){
        directionStr = "backward";
      }
      else if(mControl.mDirection == 4){
        directionStr = "left";
      }
      // ROS_INFO("Adjusting distance to...");
      // ROS_INFO(distanceChar);
      // ROS_INFO("away from the target.../n");
      mControlMode2 = true;
      mControlPower = mControl.power;
      mControlDirection = mControl.mDirection;
      mControlDistance = mControl.distance;
      positionXPrev = positionX;
      positionYPrev = positionY;
      cout << "going " << directionStr << " " << mControlDistance << " meters."<< endl;
    }
  }
  else if(mControl.state == 3){
    if(mControlMode1 || mControlMode2|| mControlMode3 || mControlMode4 || mControlMode5)
      ROS_INFO("Sub is still moving. Command abort.");
    else{
      ROS_INFO("Centering the sub with target from the front camera...\n");

      mControlMode3 = true;
      mControlPower = 0;
      mControlDistance = 0;
    }
  }
  else if(mControl.state == 4){
    if(mControlMode1 || mControlMode2 || mControlMode3 || mControlMode4 || mControlMode5)
      ROS_INFO("Sub is still moving. Command abort.");
    else{
      ROS_INFO("Centering the sub with target from the bottom camera...\n");

      mControlMode4 = true;
      mControlPower = 0;
      mControlDistance = 0;
    }
  }
  else if(mControl.state == 5){
    if(mControlMode1 || mControlMode2|| mControlMode3 || mControlMode4 || mControlMode5)
      ROS_INFO("Sub is still moving. Command abort.");
    else if(mControl.mDirection != 1 && mControl.mDirection != 2 && mControl.mDirection != 3 && mControl.mDirection != 4)
      ROS_INFO("Invalid direction with state 5. Please check the program and try again.\n");
    else if(mControl.power == 0)
      ROS_INFO("Invalid power with state 5. Please check the program and try again.");
    else{
      if(mControl.mDirection == 1){
        directionStr = "forward";
      }
      else if(mControl.mDirection == 2){
        directionStr = "right";
      }
      else if(mControl.mDirection == 3){
        directionStr = "backward";
      }
      else if(mControl.mDirection == 4){
        directionStr = "left";
      }

      directionStr = "Moving " + directionStr + " with power...";
      ROS_INFO("%s", directionStr.c_str());
      // ROS_INFO(powerChar);
      // ROS_INFO("for...");
      // ROS_INFO(timeChar);
      // ROS_INFO("seconds...\n");

      mControlMode5 = true;
      mControlMode5Timer = 0;
      mControlDirection = mControl.mDirection;
      mControlPower = mControl.power;
      mControlRunningTime = mControl.runningTime;
      mControlMode5Timer = loopTime;
      mControlDistance = 0;
    }
  }
  mControlStatus.state = mControl.state;
  mControlStatus.mDirection = mControl.mDirection;
  mControlStatus.power = mControl.power;
  mControlStatus.distance = mControl.distance;
  mControlStatus.runningTime = mControl.runningTime;
  mControlPublisher.publish(mControlStatus);

}

//roll left is negative roll right is positive (roll currently inverted of this)
//pitch backward is positive pitch forward is negative
void heightControl(){
  if(!subIsReady){ return; }

  timePrev = timeCur;  // the previous time is stored before the actual time read
  timeCur = millis();  // actual time read
  elapsedTime = (timeCur - timePrev) /base_thrust;      //base_thrust; 
  
  /*///////////////////////////P I Ds///////////////////////////////////*/
  
  error_pitch = pitch - desired_angle; 
  error_roll = roll - desired_angle;
  error_depth = feetDepth_read - assignedDepth;
  
  pid_p_pitch = kp_pitch*error_pitch;
  pid_p_roll = kp_roll*error_roll;
  pid_p_depth = kp_depth*error_depth;
  
  if(-1 < error_pitch && error_pitch < 1){
    pid_i_pitch = pid_i_pitch+(ki_pitch*error_pitch);  
  }
  
  if(-1 < error_roll && error_roll < 1){
    pid_i_roll = pid_i_roll+(ki_roll*error_roll);
  } 

  if(-0.5 < error_depth && error_depth < 0.5)
  {
    pid_i_depth = pid_i_depth+(ki_depth*error_depth);
  } 
  
  pid_d_pitch = kd_pitch*((error_pitch - prev_error_pitch)/elapsedTime);
  pid_d_roll = kd_roll*((error_roll - prev_error_roll)/elapsedTime);
  pid_d_depth = kd_depth*((error_depth - prev_error_depth)/elapsedTime);
  
  PID_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch;
  PID_roll = pid_p_roll + pid_i_roll + pid_d_roll;
  PID_depth = pid_p_depth + pid_i_depth + pid_d_depth;

  //max pid adjustment is full 800 which is the range of motor power
  if(PID_pitch < -800){PID_pitch=-800;}
  if(PID_pitch > 800){PID_pitch=800;}
  if(PID_roll < -800){PID_roll=-800;}
  if(PID_roll > 800){PID_roll=800;}

  //depth max speed set to pwm_submerge and pwm_emerge
  //negative number for submerge
  if(PID_depth < -hControlPower){PID_depth = -hControlPower;}
  //positive for emerge
  if(PID_depth > hControlPower){PID_depth = hControlPower;}
  
  //Emerging and submerging thruster pwm requirments:
  /*Submerging: pwmThruster_1 > base_thrust 
                pwmThruster_2 < base_thrust
                pwmThruster_3 > base_thrust 
                pwmThruster_4 < base_thrust

    Emerging:   pwmThruster_1 < base_thrust 
                pwmThruster_2 > base_thrust
                pwmThruster_3 < base_thrust 
                pwmThruster_4 > base_thrust         
  */

  //emerging                                           
  if (assignedDepth < (feetDepth_read - threshold )){
    pwmThruster_1 = base_thrust - PID_pitch - PID_roll - PID_depth;
    pwmThruster_2 = base_thrust + PID_pitch - PID_roll + PID_depth;
    pwmThruster_3 = base_thrust - PID_pitch + PID_roll - PID_depth;
    pwmThruster_4 = base_thrust + PID_pitch + PID_roll + PID_depth;
  }

  //submerging
  else if (assignedDepth > (feetDepth_read + threshold)){
    pwmThruster_1 = base_thrust - PID_pitch - PID_roll - PID_depth;
    pwmThruster_2 = base_thrust + PID_pitch - PID_roll + PID_depth;
    pwmThruster_3 = base_thrust - PID_pitch + PID_roll - PID_depth;
    pwmThruster_4 = base_thrust + PID_pitch + PID_roll + PID_depth;
  }
  
  //////Stabilization sum
  else{
    if(isGoingUp || isGoingDown){
      isGoingUp = false;
      isGoingDown = false;
      ROS_INFO("Assigned depth reached.\n");
    }

    pwmThruster_1 = base_thrust - PID_pitch - PID_roll;
    pwmThruster_2 = base_thrust + PID_pitch - PID_roll;
    pwmThruster_3 = base_thrust - PID_pitch + PID_roll;
    pwmThruster_4 = base_thrust + PID_pitch + PID_roll;

    hControlStatus.state = 1;
    hControlStatus.depth = feetDepth_read;
    hControlStatus.power = hControlPower;
    hControlPublisher.publish(hControlStatus);
  }

  ///////////Thruster power buffer//////////////
  //Thruster_1
  if(pwmThruster_1 < motorMin){pwmThruster_1= motorMin;}
  if(pwmThruster_1 > motorMax){pwmThruster_1=motorMax;}
  //Thruster_2
  if(pwmThruster_2 < motorMin){pwmThruster_2= motorMin;}
  if(pwmThruster_2 > motorMax){pwmThruster_2=motorMax;}
  //Thruster_3
  if(pwmThruster_3 < motorMin){pwmThruster_3= motorMin;}
  if(pwmThruster_3 > motorMax){pwmThruster_3=motorMax;}
  //Thruster_4
  if(pwmThruster_4 < motorMin){pwmThruster_4= motorMin;}
  if(pwmThruster_4 > motorMax){pwmThruster_4=motorMax;}
          
  prev_error_pitch = error_pitch;
  prev_error_roll = error_roll;
  prev_error_depth = error_depth;
  
  // char temp[8];
  // dtostrf(pwmThruster_1, 8, 0, temp);
  // ROS_INFO(temp);

  /*Create the PWM pulses with the calculated width for each pulse*/
  // T1.writeMicroseconds(pwmThruster_1);
  // T2.writeMicroseconds(pwmThruster_2);
  // T3.writeMicroseconds(pwmThruster_3);
  // T4.writeMicroseconds(pwmThruster_4);
  publishMVertical(pwmThruster_1, pwmThruster_2, pwmThruster_3, pwmThruster_4);
}

//read rotation is from -193.8 to 166.2
void rotationControl(){

  float delta = degreeToTurn();
  float rotationError = 0.3;
  int fixedPower = rControlPower;
  if(fixedPower > rotatePowerMax) fixedPower = rotatePowerMax;

  if(keepTurningLeft){
    // //Turn on left rotation motor with fixed power
    // T5.writeMicroseconds(base_thrust + fixedPower);
    // T7.writeMicroseconds(base_thrust - fixedPower);
    if( ((mControlMode5 || mControlMode1 || mControlMode2) && (mControlDirection == 2 || mControlDirection == 4)) || keepMovingRight || keepMovingLeft){
      // T6.writeMicroseconds(base_thrust + fixedPower);
      // T8.writeMicroseconds(base_thrust + fixedPower);
      publishMHorizontal(-1, base_thrust + fixedPower, -1, base_thrust + fixedPower);
    }
    else{
      // T5.writeMicroseconds(base_thrust - fixedPower);
      // T7.writeMicroseconds(base_thrust + fixedPower);
      publishMHorizontal(base_thrust - fixedPower, -1, base_thrust + fixedPower, -1);
    }
    assignedYaw = yaw;

    //Testing----------------------------
//    rotationTimer += 0.01;
//    if(rotationTimer > rotationTime)
//      keepTurningLeft = false;
//    yaw += 0.05;
//    if(yaw > rotationUpperBound)
//      yaw -= 360;
  }
  else if(keepTurningRight){
    //Turn on right rotation motor with fixed power
    // T5.writeMicroseconds(base_thrust - fixedPower);
    // T7.writeMicroseconds(base_thrust + fixedPower);
    if(((mControlMode5 || mControlMode1 || mControlMode2) && (mControlDirection == 2 || mControlDirection == 4)) || keepMovingRight || keepMovingLeft){
      // T6.writeMicroseconds(base_thrust - fixedPower);
      // T8.writeMicroseconds(base_thrust - fixedPower);
      publishMHorizontal(-1, base_thrust - fixedPower, -1, base_thrust - fixedPower);
    }
    else{
      // T5.writeMicroseconds(base_thrust + fixedPower);
      // T7.writeMicroseconds(base_thrust - fixedPower);
      publishMHorizontal(base_thrust + fixedPower, -1, base_thrust - fixedPower, -1);
    }
    assignedYaw = yaw;
    //Testing----------------------------
//    rotationTimer += 0.01;
//    if(rotationTimer > rotationTime)
//      keepTurningRight = false;
//    yaw -= 0.05;
//    if(yaw < rotationLowerBound)
//      yaw +=360;
  }
  // AutoRotation to the assignedYaw
  else if(delta > rotationError){
    // cout << "in rotationError delta: " << delta << " assignedYaw: " << assignedYaw << " yaw: " << yaw << endl;
    if(isTurningRight){
      // ROS_INFO("isTurningRight");
      // cout << "isTurningRight" << endl;
      rotateRightDynamically();
    }
    else if(isTurningLeft){
      // ROS_INFO("isTurningLeft");
      // cout << "isTurningLeft" << endl;
      rotateLeftDynamically();
    }
    else if(yaw + delta > rotationUpperBound){
      // ROS_INFO("yaw + delta > rotationUpperBound");
      if(yaw - delta == assignedYaw){
        // ROS_INFO("aw - delta == assignedYaw");
        // cout << "aw - delta == assignedYaw" << endl;
        rotateLeftDynamically();
      }else{
        // ROS_INFO("aw - delta == assignedYaw else");
        // cout << "aw - delta == assignedYaw else" << endl;
        rotateRightDynamically();
      }
    }
    else if(yaw - delta < rotationLowerBound){
      // ROS_INFO("yaw - delta < rotationUpperBound");

      // cout << "yaw - delta < rotationUpperBound" << endl;
      if(yaw + delta == assignedYaw){
        // ROS_INFO("yaw + delta == assignedYaw");
        // cout << "yaw + delta == assignedYaw" << endl;
        rotateRightDynamically();
      }
      else{
        // ROS_INFO("yaw + delta == assignedYaw else");
        // cout << "yaw + delta == assignedYaw else" << endl;
        rotateLeftDynamically();
      }
    }
    else if(yaw < assignedYaw){
      // ROS_INFO("yaw < assignedYaw");
      // cout << "yaw < assignedYaw" << endl;
      rotateRightDynamically();
    }
    else if(yaw > assignedYaw){
      // ROS_INFO("yaw < assignedYaw else");
      // cout << "yaw < assignedYaw else" << endl;
      rotateLeftDynamically();
    }
  }
  //No rotation
  else if(!keepTurningRight && !keepTurningLeft && !rControlMode3 && !rControlMode4 && delta <= rotationError){
    // cout << "in no rotation" << endl;
    if(isTurningRight || isTurningLeft){
      isTurningRight = false;
      isTurningLeft = false;
      ROS_INFO("Assigned rotation reached.\n");
      rControlStatus.state = 1;
      rControlStatus.rotation = 0;
      rControlStatus.power = rControlPower;
      rControlPublisher.publish(rControlStatus);
    }
    // T5.writeMicroseconds(base_thrust);
    // T7.writeMicroseconds(base_thrust);
    // publishMHorizontal(base_thrust, -1, base_thrust, -1);
    if(((mControlMode5 || mControlMode1 || mControlMode2) && (mControlDirection == 2 || mControlDirection == 4)) || keepMovingRight || keepMovingLeft){
    // T6.writeMicroseconds(base_thrust + rotatePower);
    // T8.writeMicroseconds(base_thrust + rotatePower);
      // cout << "base_thrust 6 and 8" << endl;
      publishMHorizontal(-1, base_thrust, -1, base_thrust);
    }
    else{
      // T5.writeMicroseconds(base_thrust - rotatePower);
      // T7.writeMicroseconds(base_thrust + rotatePower);
      // cout << "base_thrust 5 and 7" << endl;
      publishMHorizontal(base_thrust, -1, base_thrust, -1);
    }
  }

}


void movementControl(){
  float mControlPowerTemp = mControlPower;
  if(mControlPowerTemp > 200) mControlPowerTemp = 200;

  if(mControlMode1){
    if(keepMovingForward){
      // T6.writeMicroseconds(base_thrust + mControlPowerTemp);
      // T8.writeMicroseconds(base_thrust - mControlPowerTemp);
      publishMHorizontal(-1, base_thrust + mControlPowerTemp, -1, base_thrust - mControlPowerTemp);
      //Testing-------------------
      // positionY += 0.05;
      //ROS_INFO("moving forward...");
    }
    else if(keepMovingRight){
      // T5.writeMicroseconds(base_thrust + mControlPowerTemp);
      // T7.writeMicroseconds(base_thrust + mControlPowerTemp);
      publishMHorizontal(base_thrust + mControlPowerTemp, -1, base_thrust + mControlPowerTemp, -1);
      //Testing-------------------
      // positionX += 0.05;
      //ROS_INFO("moving right...");
    }
    else if(keepMovingBackward){
      // T6.writeMicroseconds(base_thrust - mControlPowerTemp);
      // T8.writeMicroseconds(base_thrust + mControlPowerTemp);
      publishMHorizontal(-1, base_thrust - mControlPowerTemp, -1, base_thrust + mControlPowerTemp);
      //Testing-------------------
      // positionY -= 0.05;
      //ROS_INFO("moving backward...");
    }
    else if(keepMovingLeft){
      // T5.writeMicroseconds(base_thrust - mControlPowerTemp);
      // T7.writeMicroseconds(base_thrust - mControlPowerTemp);
      publishMHorizontal(base_thrust - mControlPowerTemp, -1, base_thrust - mControlPowerTemp, -1);
      //Testing-------------------
      // positionX -= 0.05;
      //ROS_INFO("moving left...");
    }
  }
  else if(mControlMode2){
    // cout << "in mControlMode2" << endl;
    if(mControlDirection == 1){
      // T6.writeMicroseconds(base_thrust + mControlPowerTemp);
      // T8.writeMicroseconds(base_thrust - mControlPowerTemp);
      // cout << "in mControlMode2 direction 1" << endl;
      publishMHorizontal(-1, base_thrust + mControlPowerTemp, -1, base_thrust - mControlPowerTemp);
      
      // ROS_INFO("moving forward...");
    }
    //right
    else if(mControlDirection == 2){
      // T5.writeMicroseconds(base_thrust + mControlPowerTemp);
      // T7.writeMicroseconds(base_thrust + mControlPowerTemp);
      publishMHorizontal(base_thrust + mControlPowerTemp, -1, base_thrust + mControlPowerTemp, -1);
      
      // ROS_INFO("moving right...");
    }
    //backward
    else if(mControlDirection == 3){
      // T6.writeMicroseconds(base_thrust - mControlPowerTemp);
      // T8.writeMicroseconds(base_thrust + mControlPowerTemp);
      publishMHorizontal(-1, base_thrust - mControlPowerTemp, -1, base_thrust + mControlPowerTemp);
      
      // ROS_INFO("moving backward...");
    }
    //left
    else if(mControlDirection == 4){
      // T5.writeMicroseconds(base_thrust - mControlPowerTemp);
      // T7.writeMicroseconds(base_thrust - mControlPowerTemp);
      publishMHorizontal(base_thrust - mControlPowerTemp, -1, base_thrust - mControlPowerTemp, -1);
      
      // ROS_INFO("moving left...");
    }
    double calculatedDistance = 0;
    float l1 = max(positionX, positionXPrev) - min(positionX, positionXPrev);
    float l2 = max(positionY, positionYPrev) - min(positionY, positionYPrev);
    calculatedDistance = sqrt(l1*l1 + l2*l2);
    // cout << "calculatedDistance: " << calculatedDistance << " mControlDistance: " << mControlDistance << endl;
    // cout << "mControlPower: " << mControlPower << endl;
    
    if(calculatedDistance >= mControlDistance){
      // T6.writeMicroseconds(base_thrust);
      // T8.writeMicroseconds(base_thrust);
      // T5.writeMicroseconds(base_thrust);
      // T7.writeMicroseconds(base_thrust);
      publishMHorizontal(base_thrust, base_thrust, base_thrust, base_thrust);
      mControlMode2 = false;
      ROS_INFO("Mode 2 finished.\n");
    }
  }
  else if(mControlMode5){
    //forward
    if(mControlDirection == 1){
      // T6.writeMicroseconds(base_thrust + mControlPowerTemp);
      // T8.writeMicroseconds(base_thrust - mControlPowerTemp);
      publishMHorizontal(-1, base_thrust + mControlPowerTemp, -1, base_thrust - mControlPowerTemp);
      
      //Testing-------------------
      // positionY += 0.05;
      ROS_INFO("moving forward...");
    }
    //right
    else if(mControlDirection == 2){
      // T5.writeMicroseconds(base_thrust + mControlPowerTemp);
      // T7.writeMicroseconds(base_thrust + mControlPowerTemp);
      publishMHorizontal(base_thrust + mControlPowerTemp, -1, base_thrust + mControlPowerTemp, -1);
      //Testing-------------------
      // positionX += 0.05;
      ROS_INFO("moving right...");
    }
    //backward
    else if(mControlDirection == 3){
      // T6.writeMicroseconds(base_thrust - mControlPowerTemp);
      // T8.writeMicroseconds(base_thrust + mControlPowerTemp);
      publishMHorizontal(-1, base_thrust - mControlPowerTemp, -1, base_thrust + mControlPowerTemp);
      //Testing-------------------
      // positionY -= 0.05;
      ROS_INFO("moving backward...");
    }
    //left
    else if(mControlDirection == 4){
      // T5.writeMicroseconds(base_thrust - mControlPowerTemp);
      // T7.writeMicroseconds(base_thrust - mControlPowerTemp);
      publishMHorizontal(base_thrust - mControlPowerTemp, -1, base_thrust - mControlPowerTemp, -1);
      //Testing-------------------
      // positionX -= 0.05;
      ROS_INFO("moving left...");
    }
    // mControlMode5Timer += 0.05;
    // char timerChar[11];
    // dtostrf(mControlMode5Timer, 4, 2, timerChar);
    // ROS_INFO(timerChar);
    if((loopTime - mControlMode5Timer) >= (mControlRunningTime*1000)){
      // T6.writeMicroseconds(base_thrust);
      // T8.writeMicroseconds(base_thrust);
      // T5.writeMicroseconds(base_thrust);
      // T7.writeMicroseconds(base_thrust);
      publishMHorizontal(base_thrust, base_thrust, base_thrust, base_thrust);
      mControlMode5 = false;
      ROS_INFO("Mode 5 finished.\n");
    }
  }
  else{
    centerTimer = 0;
    // movementTimer = 0;
    mControlMode5Timer = 0;
    mControlDirection = 0;
    mControlRunningTime = 0;
    mControlPower = 0;
    mControlDistance = 0;
    mControlStatus.state = 0;
    mControlStatus.mDirection = 0;
    mControlStatus.power = 0;
    mControlStatus.distance = 0;
    mControlStatus.runningTime = 0;
    mControlPublisher.publish(mControlStatus);
  }

}

//&&&&&&&&&&&&&&&&&&&&&&&&&&
//NOTE: I FORGOT WHAT IS THE ROTATION ON THE MOTORS IN THE BOT TO SEE WHAT DIRECTION OF MOVEMENT
//WHEN I RETURN TO THE ROOM I CAN FIND OUT AND EDIT THIS CODE.
//  if (yaw < assignedYaw){
//    //turn on motos to go down
//    T5.writeMicroseconds(base_thrust + PWM_Motors);
//    T7.writeMicroseconds(base_thrust - PWM_Motors);
//  }
//  else if (yaw > assignedYaw){
//    //turn on motors to go up
//    T5.writeMicroseconds(base_thrust - PWM_Motors);
//    T7.writeMicroseconds(base_thrust + PWM_Motors);
//  }
void rotateLeftDynamically(){
  float rotatePower = PWM_Motors_orient * 5.0;

  if(rotatePower > rControlPower && isTurningLeft) rotatePower = rControlPower;
  if(rotatePower > rotatePowerMax) rotatePower = rotatePowerMax;
  if(((mControlMode5 || mControlMode1 || mControlMode2) && (mControlDirection == 2 || mControlDirection == 4)) || keepMovingRight || keepMovingLeft){
    // T6.writeMicroseconds(base_thrust + rotatePower);
    // T8.writeMicroseconds(base_thrust + rotatePower);
    publishMHorizontal(-1, base_thrust + rotatePower, -1, base_thrust + rotatePower);
  }
  else{
    // T5.writeMicroseconds(base_thrust - rotatePower);
    // T7.writeMicroseconds(base_thrust + rotatePower);
    publishMHorizontal(base_thrust - rotatePower, -1, base_thrust + rotatePower, -1);
  }
  // ROS_INFO("rotate left");
  //Testing----------------------------
  //yaw += 1;
  //if(yaw > rotationUpperBound) yaw -= 360;

}

void rotateRightDynamically(){
  float rotatePower = PWM_Motors_orient * 5.0;

  if(rotatePower > rControlPower && isTurningRight) rotatePower = rControlPower;
  if(rotatePower > rotatePowerMax) rotatePower = rotatePowerMax;
  if(((mControlMode5 || mControlMode1 || mControlMode2) && (mControlDirection == 2 || mControlDirection == 4)) || keepMovingRight || keepMovingLeft){
    // T6.writeMicroseconds(base_thrust - rotatePower);
    // T8.writeMicroseconds(base_thrust - rotatePower);
    publishMHorizontal(-1, base_thrust - rotatePower, -1, base_thrust - rotatePower);
  }
  else{
    // T5.writeMicroseconds(base_thrust + rotatePower);
    // T7.writeMicroseconds(base_thrust - rotatePower);
    publishMHorizontal(base_thrust + rotatePower, -1, base_thrust - rotatePower, -1);
  }
  // ROS_INFO("rotate right");
  //Testing----------------------------
  //yaw -= 1;
  //if(yaw < rotationLowerBound) yaw +=360;

}

//Return 0 to 180
float degreeToTurn(){
  float difference = max(yaw, assignedYaw) - min(yaw, assignedYaw);
  if (difference > 180) return 360 - difference;
  else return difference;
}


void currentDepthCallback(const std_msgs::Float32& currentDepth){
  feetDepth_read = currentDepth.data;
}


void setup() {

  //setup motor messages
  mVertical.t1 = base_thrust;
  mVertical.t2 = base_thrust;
  mVertical.t3 = base_thrust;
  mVertical.t4 = base_thrust;
  mHorizontal.t5 = base_thrust;
  mHorizontal.t6 = base_thrust;
  mHorizontal.t7 = base_thrust;
  mHorizontal.t8 = base_thrust;

  //Initialize ROS variable
  mControlDirection = 0;
  mControlPower = 0;
  rControlPower = 0;
  hControlPower = 0;
  mControlDistance = 0;
  mControlRunningTime = 0;
  mControlMode5Timer = 0;
  // frontCamForwardDistance = 0;
  // frontCamHorizontalDistance = 0;
  // frontCamVerticalDistance = 0;
  // bottomCamForwardDistance = 0;
  // bottomCamHorizontalDistance = 0;
  // bottomCamVerticalDistance = 0;
  centerTimer = 0;
  rotationTimer = 0;
  rotationTime = 10;
  // movementTimer = 0;
  movementTime = 10;
  subIsReady = false;
  isGoingUp = false;
  isGoingDown = false;
  isTurningRight = false;
  isTurningLeft = false;
  keepTurningRight = false;
  keepTurningLeft = false;
  rControlMode3 = false;
  rControlMode4 = false;
  mControlMode1 = false;
  mControlMode2 = false;
  mControlMode3 = false;
  mControlMode4 = false;
  mControlMode5 = false;
  keepMovingForward = false;
  keepMovingRight = false;
  keepMovingBackward = false;
  keepMovingLeft = false;

  //Testing------------------
  feetDepth_read = 0;
  roll = 999;
  pitch = 999;
  yaw = 999;
  positionXPrev = 0;
  positionYPrev = 0;

  positionX = 0;
  positionY = 0;
  positionZ = 0;
  velocityX = 0;
  velocityY = 0;
  velocityZ = 0;

  assignedDepth = topDepth;
  currentDepth.data = feetDepth_read;

//  assignedDepth = 0.2;
  //assignedYaw = -191.5;
  //subIsReady = true;
  // assignedYaw = 64.6;
//  currentRotation.data = yaw;

  hControlStatus.state = 1;
  hControlStatus.depth = 0;
  rControlStatus.state = 1;
  rControlStatus.rotation = 0;

  // MControl
  // state => 0: off, 1: on with power, 2: on with distance, 3: centered with front cam, 4: centered with bottom cam
  // direction => 0: none, 1: forward, 2: right, 3: backward, 4: left
  // power => 0: none, x: x power add to the motors
  // distance => 0: none, x: x units away from the object
  // runningTime => 0: none, x: x seconds for the motors to run
  mControlStatus.state = 0;
  mControlStatus.mDirection = 0;
  mControlStatus.power = 0;
  mControlStatus.distance = 0;
  mControlStatus.runningTime = 0;

  timeCur = millis();
  loopTime = timeCur;
  loopTimePrev = loopTime;

  ROS_INFO("Sub is staying. Waiting to receive data from master...\n");
}

void loop() {
  ros::spinOnce();
  loopTime = millis();  // actual time read
  //  gettingRawData();

  if((loopTime-loopTimePrev) > loopInterval) {
    loopTimePrev = loopTime;
    //Rotation
    //duty cycle and PWM calculation for orientation
    dutyCycl_orient = degreeToTurn() / 180.0;
    PWM_Motors_orient = dutyCycl_orient * 400; //Maximum is 200


    /////////////////////////////////////////////////////////////////////////////////////////////
    // DEBUG
    // assignedYaw = yaw;
    // cout << "assignedYaw: " << assignedYaw << endl;
    
    /////////////////////////////////////////////////////////////////////////////////////////////
    
    if(subIsReady){
      ros::spinOnce();
      heightControl();
      movementControl();
      rotationControl();
      ros::spinOnce();
      publishMotors();
    }
  }
}

int main(int argc, char **argv){

  ros::init(argc, argv, "hardware_interface");
  ros::NodeHandle nh;
  
  hControlPublisher = nh.advertise<robosub::HControl>("height_control_status", 100);
  rControlPublisher = nh.advertise<robosub::RControl>("rotation_control_status", 100);
  mControlPublisher = nh.advertise<robosub::MControl>("movement_control_status", 100);
  mVerticalPublisher = nh.advertise<hardware_interface::MotorVertical>("motor_vertical", 1);
  mHorizontalPublisher = nh.advertise<hardware_interface::MotorHorizontal>("motor_horizontal", 1);

  currentDepthSubscriber = nh.subscribe("current_depth", 100, currentDepthCallback);
  hControlSubscriber = nh.subscribe("height_control", 100, hControlCallback);
  rControlSubscriber = nh.subscribe("rotation_control", 100, rControlCallback);
  mControlSubscriber = nh.subscribe("movement_control", 100, mControlCallback);
  rotationSubscriber = nh.subscribe("current_rotation", 1, rotationCallback);
  dvlSubscriber = nh.subscribe("dvl_status", 1, dvlCallback);

  setup();
  while(ros::ok()){
    loop();
  }

  return 0;
}