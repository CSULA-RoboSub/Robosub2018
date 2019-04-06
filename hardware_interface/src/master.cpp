#include <string>
#include <cmath>
#include <algorithm>
#include <iostream>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <robosub/HControl.h>
#include <robosub/RControl.h>
#include <robosub/MControl.h>
#include <pathfinder_dvl/DVL.h>
#include <ez_async_data/Rotation.h>
#include <hardware_interface/MotorHorizontal.h>
#include <hardware_interface/MotorVertical.h>
#include "include/DVLHelper.h"

using namespace std;

bool subIsReady;
double assignedDepth;
double feetDepth_read;
//IMU initializing
double pitch, yaw, roll;

double assignedYaw;

double rotatePowerMax = 120;
const double rotationUpperBound = 180;
const double rotationLowerBound = -180;
const double rotationMultiplier = 4.5;
const double rotationMinOffset = 35;
const double topDepth = 0.5;
const double bottomDepth = 20;
const double motorMax = 1700;
const double motorMin = 1300;
const double motorRange = 200;

int mControlDirection;
double mControlPower;
double rControlPower;
double hControlPower;
double mControlDistance;
double mControlRunningTime;
double mControlMode3Timer; //time variable for timed movement


//Movement booleans - deleted infinite rotation booleans for time being
bool isGoingUp;
bool isGoingDown;
bool isTurningRight;
bool isTurningLeft;

//bool flags for control modes
bool mControlMode1 = false; //Power
bool mControlMode2 = false; //Distance
bool mControlMode3 = false; //Time

//bool flags for infinite movement
bool keepMovingForward;
bool keepMovingRight;
bool keepMovingBackward;
bool keepMovingLeft;

//Used when calculating distance to travel when on mControlMode3
double positionXPrev = 0;
double positionYPrev = 0;

//DVL values
double positionX = 0;
double positionY = 0;
double positionZ = 0;
double velocityX = 0;
double velocityY = 0;
double velocityZ = 0;

//thrusters off value does not change
const double base_thrust = 1500;

//time variables
double elapsedTime, timeCur, timePrev, loopTime, loopTimePrev;
const int loopInterval = 20;

//the variable error will store the difference between the real_value_angle form IMU and desired_angle of 0 degrees.
// double PID_pitch, PID_roll, pwmThruster_2, pwmThruster_1, pwmThruster_3, pwmThruster_4, error_roll, prev_error_roll = 0,error_pitch, prev_error_pitch = 0;
double PID_pitch, PID_roll, PID_depth, pwmThruster_1, pwmThruster_2, pwmThruster_3, pwmThruster_4,
error_roll, prev_error_roll=0, error_pitch, prev_error_pitch=0, error_depth, prev_error_depth=0;

//////////////PID_pitch variables////////////////////
double pid_p_pitch=0;
double pid_d_pitch=0;
double pid_i_pitch=0;
/////////////////PID_pitch constants/////////////////
const double kp_pitch=2;//11;//3.55;//3.55
const double kd_pitch=0.7;//0.75;//2.05;//2.05
const double ki_pitch=0.0003;//0.003
///////////////////////////////////////////////

//////////////PID_roll variables////////////////////
double pid_p_roll=0;
double pid_d_roll=0;
double pid_i_roll=0;

/////////////////PID_roll constants/////////////////
const double kp_roll=2;//11;//3.55;//3.55
const double kd_roll=0.7;//0.75;//2.05;//2.05
const double ki_roll=0.0003;//0.003
///////////////////////////////////////////////

//////////////PID_depth variables////////////////////
double pid_p_depth=0;
double pid_d_depth=0;
double pid_i_depth=0;
/////////////////PID_depth constants/////////////////
const double kp_depth=300;//11;//3.55;//3.55
const double kd_depth=0.7;//0.75;//2.05;//2.05
const double ki_depth=0.003;//0.003

const double desired_angle = 0; //This is the angle in which we
//balance to stay steady
//// threshold for going_up, going_down and hover
const double heightThreshold = 0.3;
const double defaultHeightPower = 100;
///////////////////////////////////////////////

/////////////////PID_heading constants and variables/////////////////
const double kp_heading=11; //11;//3.55;//3.55
const double kd_heading=0.4; //0.75;//2.05;//2.05
const double ki_heading=0.008;

double pid_i_heading = 0;
double prev_error_heading = 0;
double rotation_ki_threshold = 5;
double rotationThreshold = 3;
///////////////////////////////////////////////

/////////////////PID_positionX constants and variables/////////////////
// const double kp_positionX=9;//11;//3.55;//3.55
const double kd_positionX=0.7;//0.75;//2.05;//2.05
const double ki_positionX=0.003;//0.003

double pid_i_positionX = 0;
double rightLeftDistancePrev = 0;
///////////////////////////////////////////////

/////////////////PID_positionY constants and variables/////////////////
// const double kp_positionY=9;//11;//3.55;//3.55
const double kd_positionY=0.7;//0.75;//2.05;//2.05
const double ki_positionY=0.003;//0.003

double pid_i_positionY = 0;
double forwardBackwardDistancePrev = 0;
///////////////////////////////////////////////

//position keeping variables
double keepPositionX = 0;
double keepPositionY = 0;
double positionControlMOffset = 0;
const double keepPositionForwardThreshold = 0.9; //meters
const double keepPositionRightThreshold = 0.5; //meters
const double keepPositionMaxPowerForwardBackward = 200;
const double keepPositionMaxPowerRightLeft = 200;
const double keepCourseRightMult = 0.028;
const double keepCourseForwardMult = 0.09;


std_msgs::Float32 currentDepth;
//std_msgs::Float32 dvlHeading;
robosub::HControl hControlStatus;
robosub::RControl rControlStatus;
robosub::MControl mControlStatus;
std_msgs::Int8 ledStatus;

hardware_interface::MotorVertical mVertical;
hardware_interface::MotorHorizontal mHorizontal;
//msg container used to offset mHorizontal for position keeping
double mHorizontalOffsetT5 = 0, mHorizontalOffsetT6 = 0, mHorizontalOffsetT7 = 0, mHorizontalOffsetT8 = 0;
hardware_interface::MotorHorizontal mHorizontalTotal;


ros::Publisher hControlPublisher;     //int: state, double: depth
ros::Publisher rControlPublisher;   //int: state, double: rotation
ros::Publisher mControlPublisher;   //int: state, int: direction, double: distance
ros::Publisher currentDepthPublisher;           //double: depth
ros::Publisher ledPublisher;        //int: state
ros::Publisher mHorizontalPublisher;
ros::Publisher mVerticalPublisher;


ros::Subscriber currentDepthSubscriber;
ros::Subscriber hControlSubscriber;   //int: state, double: depth
ros::Subscriber rControlSubscriber; //int: state, double: rotation
ros::Subscriber mControlSubscriber;
ros::Subscriber rotationSubscriber;
ros::Subscriber dvlSubscriber;

void resetAll(){
  mControlDirection = 0;
  mControlPower = 0;
  rControlPower = 0;
  hControlPower = 0;
  mControlDistance = 0;
  mControlRunningTime = 0;
  mControlMode3Timer = 0;
  subIsReady = false;
  isGoingUp = false;
  isGoingDown = false;
  isTurningRight = false;
  isTurningLeft = false;
  mControlMode1 = false;
  mControlMode2 = false;
  mControlMode3 = false;
  keepMovingForward = false;
  keepMovingRight = false;
  keepMovingBackward = false;
  keepMovingLeft = false;
  positionControlMOffset = 0;
}

double millis(){
  return ros::WallTime::now().toNSec()/1000000;
}
bool isDoingMovement(){
  return(mControlMode1 || mControlMode2 || mControlMode3);
}
bool isDoingStrafeMovement(){
  return ((isDoingMovement() && (mControlDirection == 2 || mControlDirection == 4)) || keepMovingRight || keepMovingLeft);
}

void setMVertical(const double t1, const double t2, const double t3, const double t4){
  if(t1 > 0)
  mVertical.t1 = (int) t1;
  if(t2 > 0)
  mVertical.t2 = (int) t2;
  if(t3 > 0)
  mVertical.t3 = (int) t3;
  if(t4 > 0)
  mVertical.t4 = (int) t4;
}
void setMHorizontal(const double t5, const double t6, const double t7, const double t8){
  if(t5 > 0)
  mHorizontal.t5 = (int) t5;
  if(t6 > 0)
  mHorizontal.t6 = (int) t6;
  if(t7 > 0)
  mHorizontal.t7 = (int) t7;
  if(t8 > 0)
  mHorizontal.t8 = (int) t8;
}
void setMHorizontalOffset(const double t5, const double t6, const double t7, const double t8){
  // cout << "setMHorizontalOffset: " << t5 << " " << t6 << " " << t7 << " " << t8 << endl;
  mHorizontalOffsetT5 = t5;
  mHorizontalOffsetT6 = t6;
  mHorizontalOffsetT7 = t7;
  mHorizontalOffsetT8 = t8;
}

void publishMotors(){
  mHorizontalTotal.t5 = mHorizontal.t5 + mHorizontalOffsetT5;
  mHorizontalTotal.t6 = mHorizontal.t6 + mHorizontalOffsetT6;
  mHorizontalTotal.t7 = mHorizontal.t7 + mHorizontalOffsetT7;
  mHorizontalTotal.t8 = mHorizontal.t8 + mHorizontalOffsetT8;
  mVerticalPublisher.publish(mVertical);
  mHorizontalPublisher.publish(mHorizontalTotal);
}

void motorsOff(){
  setMHorizontal(base_thrust, base_thrust, base_thrust, base_thrust);
  setMVertical(base_thrust, base_thrust, base_thrust, base_thrust);
  setMHorizontalOffset(0, 0, 0, 0);
}

double motorPowerCap(const double power){
  if(power > motorRange){
    return motorRange;
  }
  else if(power < -motorRange){
    return -motorRange;
  }
  else power;
}

//-----------------------------------------------------------------------
//------------------- position keeping helper functions -----------------
//-----------------------------------------------------------------------
//sets position keeping to current position
void setKeepPosition(){
  keepPositionX = positionX;
  keepPositionY = positionY;
}

//pid for forward axis
double positionForwardPID(const double error_positionY){
  int pid_d_positionY = 0;
  if(-keepPositionForwardThreshold < error_positionY && error_positionY < keepPositionForwardThreshold){
    pid_i_positionY = pid_i_positionY+(ki_positionY*error_positionY);
  }
  else{
    pid_i_positionY = 0;
  }

  pid_d_positionY = kd_positionY*((error_positionY - forwardBackwardDistancePrev)/elapsedTime);
  return pid_d_positionY + pid_i_positionY;
}

//pid for right axis
double positionRightPID(const double error_positionX){
  int pid_d_positionX = 0;
  if(-keepPositionRightThreshold < error_positionX && error_positionX < keepPositionRightThreshold){
    pid_i_positionX = pid_i_positionX+(ki_positionX*error_positionX);
  }
  else{
    pid_i_positionX = 0;
  }
  pid_d_positionX = kd_positionX*((error_positionX - rightLeftDistancePrev)/elapsedTime);
  return pid_d_positionX + pid_i_positionX;
}

//-----------------------------------------------------------------------
//------------------- movement helper functions -------------------------
//-----------------------------------------------------------------------

void movementControlFinish(){
  mControlMode1 = false;
  mControlMode2 = false;
  mControlMode3 = false;

  setMHorizontal(base_thrust, base_thrust, base_thrust, base_thrust);
  setKeepPosition();

  mControlMode3Timer = 0;
  mControlDirection = 0;
  mControlRunningTime = 0;
  mControlPower = 0;
  mControlStatus.state = 0;
  mControlStatus.state = 0;
  mControlStatus.mDirection = mControlDirection;
  mControlStatus.power = 0;
  mControlStatus.distance = mControlDistance;
  mControlStatus.runningTime = 0;
  mControlDistance = 0;
  mControlPublisher.publish(mControlStatus);

}

//-----------------------------------------------------------------------
//---------------------- rotation helper functions ----------------------
//-----------------------------------------------------------------------

//Return 0-180
double degreeToTurn(){
  double difference = max(yaw,assignedYaw) - min(yaw,assignedYaw);
  if(difference > rotationUpperBound){
    return 360 - difference;
  }
  else{
    return difference;
  }
}
//helper function to set power for rotation
//rotatePower must be negative to rotate left
//pid calculations for rotation power
double rotationPID(double error_heading){

  if(-rotation_ki_threshold <= error_heading && error_heading <= rotation_ki_threshold){
    pid_i_heading = pid_i_heading+(ki_heading*error_heading);
  }
  else{
    pid_i_heading = 0;
  }
  double pid_p_heading = kp_heading*error_heading;
  double pid_d_heading = kd_heading*((error_heading - prev_error_heading)/elapsedTime);

  return pid_p_heading + pid_i_heading + pid_d_heading;
}

void setRotationPower(double rotatePower){
  if(rotatePower > rControlPower && rControlPower > 0){
    rotatePower = rControlPower;
  }
  if(rotatePower < -rControlPower && -rControlPower < 0){
    rotatePower = -rControlPower;
  }
  rotatePower = motorPowerCap(rotatePower);
  setMHorizontal(base_thrust + rotatePower, -1, base_thrust - rotatePower, -1);
}

//dynamic rotations left/right
//error_heading comes from degreeToTurn() which is always positive
void rotateLeftDynamically(const double& error_heading){
  // cout << "rotate left dyn" << endl;
  ledStatus.data = 6;
  double rotatePower = rotationPID(-error_heading); //negative error for left
  setRotationPower(rotatePower);
}

void rotateRightDynamically(const double& error_heading){
  // cout << "rotate right dyn" << endl;
  ledStatus.data = 7;
  double rotatePower = rotationPID(error_heading); //positive error for right
  setRotationPower(rotatePower);
}

//-----------------------------------------------------------------------
//------------------------ ros callbacks --------------------------------
//-----------------------------------------------------------------------
void currentDepthCallback(const std_msgs::Float32& currentDepth){
  feetDepth_read = currentDepth.data;
}

void dvlCallback(const pathfinder_dvl::DVL& dvl_status){
  positionX = dvl_status.xpos;
  positionY = dvl_status.ypos;
  positionZ = dvl_status.zpos;

  velocityX = dvl_status.xvel;
  velocityY = dvl_status.yvel;
  velocityZ = dvl_status.zvel;
}

void rotationCallback(const ez_async_data::Rotation& rotation){
  yaw = rotation.yaw;
  roll = rotation.roll;
  pitch = rotation.pitch;
}

void hControlCallback(const robosub::HControl& hControl){
  double depth = hControl.depth;
  int hState = hControl.state;

  switch(hState){
    case 0:
    if(!isGoingUp && !isGoingDown){
      isGoingDown = true;
      ROS_INFO("Going down...");
      ledStatus.data = 3;

    }else{
      ROS_INFO("Sub is still running.");
    }

    if(depth == -1 || assignedDepth + depth >= bottomDepth){
      assignedDepth = bottomDepth;
    }
    else {
      assignedDepth = depth + assignedDepth;
    }
    break;

    case 1:
    if(isGoingUp || isGoingDown){
      isGoingUp = false;
      isGoingDown = false;
      ROS_INFO("Height control is now cancelled.");
      assignedDepth = feetDepth_read + 0.1;
    }
    break;

    case 2:
    if(!isGoingDown && !isGoingUp){
      isGoingUp = true;
      ROS_INFO("Going up...");
      ledStatus.data = 4;
    }
    else{
      ROS_INFO("Sub is still running.");
    }

    if(depth == -1 || depth >= assignedDepth - topDepth){
      assignedDepth = topDepth;
    }
    else {
      assignedDepth = assignedDepth - depth;
    }
    break;

    case 4:
    assignedYaw = yaw; //First time assignedYaw is assigned a value
    assignedDepth = topDepth;
    setKeepPosition();
    setMHorizontalOffset(0, 0, 0, 0);
    if(!subIsReady){
      ROS_INFO("Motors Unlocked");
    }
    subIsReady = true;
    break;

    case 5:
    if(subIsReady){
      ROS_INFO("Motors are locked");
    }
    assignedDepth = topDepth;
    subIsReady = false;
    setMVertical(base_thrust,base_thrust,base_thrust,base_thrust);
    setMHorizontal(base_thrust,base_thrust,base_thrust,base_thrust);
    setMHorizontalOffset(0, 0, 0, 0);
    resetAll();
    break;
  }

  hControlPower = hControl.power;
  hControlStatus.state = hControl.state;
  hControlStatus.depth = assignedDepth;
  hControlStatus.power = hControlPower;
  ledPublisher.publish(ledStatus);
  hControlPublisher.publish(hControlStatus);
}

void rControlCallback(const robosub::RControl& rControl){
  if (rControl.rotation > 180){
    ROS_INFO("Error rotation must be less or equal to 180 degrees");
    return;
  }
  int rState = rControl.state;
  double rotation = rControl.rotation;
  rControlStatus.state = rControl.state;
  rControlStatus.rotation = rControl.rotation;
  rControlStatus.power = rControlPower;

  switch (rState) {

    case 0:
    if(!isTurningLeft && !isTurningRight){
      if(yaw - rotation < rotationLowerBound){
        assignedYaw = yaw - rotation + 360;
      }
      else {
        assignedYaw = yaw - rotation;
      }

      isTurningLeft = true;
      ROS_INFO("Turning left...");
    }
    else{
      ROS_INFO("Sub is still rotating, ignoring command");
    }
    break;

    case 1:
    if(isTurningRight || isTurningLeft){
      ROS_INFO("Rotation control is now cancelled\n");
      rControlStatus.state = 1;
      rControlStatus.rotation = 0;
      rControlStatus.power = rControlPower;
      rControlPublisher.publish(rControlStatus);
      assignedYaw = yaw;
    }
    isTurningRight = false;
    isTurningLeft = false;
    break;

    case 2:
    if(!isTurningRight && !isTurningLeft){
      if (yaw + rotation > rotationUpperBound){
        assignedYaw = yaw + rotation - 360;
      }
      else {
        assignedYaw = yaw + rotation;
      }
      isTurningRight = true;
      ROS_INFO("Turning right...");
    }
    else {
      ROS_INFO("Sub is still rotating, ignoring command");
    }
    break;
  }
  rControlPower = rControl.power;
}

void mControlCallback(const robosub::MControl& mControl){
  double power = mControl.power;
  double distance = mControl.distance;
  double mode3Time = mControl.runningTime;
  string directionStr;
  int mState = mControl.state;

  mControlStatus.state = mControl.state;
  mControlStatus.mDirection = mControl.mDirection;
  mControlStatus.power = mControl.power;
  mControlStatus.distance = mControl.distance;
  mControlStatus.runningTime = mControl.runningTime;

  switch(mState){
    case 0:
    if(isDoingMovement()){
      mControlMode1 = false;
      mControlMode2 = false;
      mControlMode3 = false;
      keepMovingForward = false;
      keepMovingRight = false;
      keepMovingBackward = false;
      keepMovingLeft = false;
      positionControlMOffset = 0;

      mControlDirection = 0;
      mControlPower = 0;
      mControlDistance = 0;

      setMHorizontal(base_thrust,base_thrust,base_thrust,base_thrust);
      setKeepPosition();

      ROS_INFO("Movement control is now cancelled\n");
      mControlPublisher.publish(mControlStatus);
    }
    break;

    case 1: //Power
    if(isDoingMovement()){
      ROS_INFO("Sub is still moving. Command abort.");
    }
    else if(mControl.mDirection < 1 || mControl.mDirection > 4){
      ROS_INFO("Invalid direction with state 1. Please check the program and try again.\n");
    }
    else{
      switch(mControl.mDirection){
        case 1:
        keepMovingForward = true;
        directionStr = "forward";
        ledStatus.data = 1;
        break;

        case 2:
        keepMovingRight = true;
        directionStr = "right";
        ledStatus.data = 9;
        break;

        case 3:
        keepMovingBackward = true;
        directionStr = "backward";
        ledStatus.data = 2;
        break;

        case 4:
        keepMovingLeft = true;
        directionStr = "left";
        ledStatus.data = 8;
        break;
      }

      mControlMode1 = true;
      mControlDirection = mControl.mDirection;
      mControlDistance = 0;
      directionStr = "Moving " + directionStr + " until cancelled";
      ROS_INFO("%s", directionStr.c_str());
    }
    break;

    case 2://distance
    if(isDoingMovement()){
      ROS_INFO("Sub is still moving. Command abort.");
    }
    else{
      switch(mControl.mDirection){
        case 1:
        keepMovingForward = true;
        directionStr = "forward";
        ledStatus.data = 1;
        break;

        case 2:
        keepMovingRight = true;
        directionStr = "right";
        ledStatus.data = 9;
        break;

        case 3:
        keepMovingBackward = true;
        directionStr = "backward";
        ledStatus.data = 2;
        break;

        case 4:
        keepMovingLeft = true;
        directionStr = "left";
        ledStatus.data = 8;
        break;
      }

      mControlMode2 = true;
      mControlDirection = mControl.mDirection;
      mControlDistance = mControl.distance;
      positionXPrev = positionX;
      positionYPrev = positionY;
      cout << "going " << directionStr << " " << mControlDistance << " meters."<< endl;
    }
    break;

    case 3: //Time
    if(isDoingMovement()){
      ROS_INFO("Sub is still moving. Command abort.");
    }
    else{
      switch(mControl.mDirection){
        case 1:
        directionStr = "forward";
        ledStatus.data = 1;
        break;

        case 2:
        directionStr = "right";
        ledStatus.data = 9;
        break;

        case 3:
        directionStr = "backward";
        ledStatus.data = 2;
        break;

        case 4:
        directionStr = "left";
        ledStatus.data = 8;
        break;
      }
      mControlMode3 = true;
      mControlDirection = mControl.mDirection;
      mControlRunningTime = mControl.runningTime * 1000;
      mControlMode3Timer = loopTime;
      mControlDistance = 0;
      directionStr = "Moving " + directionStr + " with time...";
      ROS_INFO("%s", directionStr.c_str());
    }
    break;
  }
  mControlPower = mControl.power;
}


//----------------------------------------------------------------------------
//------------------------------ control functions ---------------------------
//----------------------------------------------------------------------------


//read rotation is from -179.999 to 179.999
//handles rotation and heading keeping
void rotationControl(){

  double delta = degreeToTurn();

  if(rControlPower > rotatePowerMax){
    rControlPower = rotatePowerMax;
  }

  if(delta <= rotationThreshold){
    if(isTurningRight || isTurningLeft){
      isTurningRight = false;
      isTurningLeft = false;
      ROS_INFO("Assigned rotation reached.\n");
      rControlStatus.state = 1;
      rControlStatus.rotation = 0;
      rControlStatus.power = rControlPower;
      rControlPublisher.publish(rControlStatus);
    }
  }
  else if(yaw + delta > rotationUpperBound && (yaw + -assignedYaw) > 180){
    if(isTurningLeft){
      rotateLeftDynamically(delta);
    }
    else{
      rotateRightDynamically(delta);
    }
  }
  else if(yaw - delta < rotationLowerBound && (-yaw + assignedYaw) > 180){
    if(isTurningRight){
      rotateRightDynamically(delta);
    }
    else{
      rotateLeftDynamically(delta);
    }
  }
  else if(yaw < assignedYaw){
    rotateRightDynamically(delta);
  }
  else if(yaw > assignedYaw){
    rotateLeftDynamically(delta);
  }
  else{
    setMHorizontal(base_thrust,-1,base_thrust,-1);
  }
}


//depth and stabilization control function
//roll left is negative roll right is positive (roll currently inverted of this)
//pitch backward is positive pitch forward is negative
void heightControl(){
  if(!subIsReady){
    return;
  }

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

  if(-heightThreshold < error_depth && error_depth < heightThreshold)
  {
    pid_i_depth = pid_i_depth+(ki_depth*error_depth);
  } else {
    pid_i_depth = 0;
  }

  pid_d_pitch = kd_pitch*((error_pitch - prev_error_pitch)/elapsedTime);
  pid_d_roll = kd_roll*((error_roll - prev_error_roll)/elapsedTime);
  pid_d_depth = kd_depth*((error_depth - prev_error_depth)/elapsedTime);

  PID_pitch = pid_p_pitch + pid_i_pitch + pid_d_pitch;
  PID_roll = pid_p_roll + pid_i_roll + pid_d_roll;
  PID_depth = pid_p_depth + pid_i_depth + pid_d_depth;

  //max pid adjustment is full 800 which is the range of motor power
  if(PID_pitch < -800){
    PID_pitch=-800;
  }
  if(PID_pitch > 800){
    PID_pitch=800;
  }
  if(PID_roll < -800){
    PID_roll=-800;
  }
  if(PID_roll > 800){
    PID_roll=800;
  }

  //depth max speed set to defaultHeightPower if no power given
  if(hControlPower == 0){
    hControlPower = defaultHeightPower;
  }
  //negative number for submerge
  if(PID_depth < -hControlPower && hControlPower != 0){
    PID_depth = -hControlPower;
  }
  //positive for emerge
  if(PID_depth > hControlPower && hControlPower != 0){
    PID_depth = hControlPower;
  }


  if((feetDepth_read - heightThreshold) < assignedDepth && assignedDepth < (feetDepth_read + heightThreshold)){
    if(isGoingUp || isGoingDown){
      isGoingUp = false;
      isGoingDown = false;
      ROS_INFO("Assigned depth reached.\n");
      hControlStatus.state = 1;
      hControlStatus.depth = feetDepth_read;
      hControlStatus.power = hControlPower;
      hControlPublisher.publish(hControlStatus);
    }
  }

  pwmThruster_1 = 0 + PID_pitch + PID_roll - PID_depth;
  pwmThruster_2 = 0 - PID_pitch + PID_roll + PID_depth;
  pwmThruster_3 = 0 - PID_pitch - PID_roll - PID_depth;
  pwmThruster_4 = 0 + PID_pitch - PID_roll + PID_depth;

  ///////////Thruster power buffer//////////////
  pwmThruster_1 = motorPowerCap(pwmThruster_1);
  pwmThruster_2 = motorPowerCap(pwmThruster_2);
  pwmThruster_3 = motorPowerCap(pwmThruster_3);
  pwmThruster_4 = motorPowerCap(pwmThruster_4);

  prev_error_pitch = error_pitch;
  prev_error_roll = error_roll;
  prev_error_depth = error_depth;

  /*Create the PWM pulses with the calculated width for each pulse*/
  setMVertical(base_thrust + pwmThruster_1, base_thrust + pwmThruster_2, base_thrust + pwmThruster_3, base_thrust + pwmThruster_4);
}


//handles forward, backward, left strafe, and right strafe movement
void movementControl(){
  double mControlPowerTemp = mControlPower;
  if (mControlPowerTemp > motorRange){
    mControlPowerTemp = motorRange;
  }
  if(mControlMode1){
    if(keepMovingForward){
      setMHorizontal(-1, base_thrust + mControlPowerTemp, -1, base_thrust - mControlPowerTemp);
    }
    else if(keepMovingRight){
      setMHorizontal(base_thrust + mControlPowerTemp, -1, base_thrust + mControlPowerTemp, -1);
    }
    else if(keepMovingBackward){
      setMHorizontal(-1, base_thrust - mControlPowerTemp, -1, base_thrust + mControlPowerTemp);
    }
    else if(keepMovingLeft){
      setMHorizontal(base_thrust - mControlPowerTemp, -1, base_thrust - mControlPowerTemp, -1);
    }
    else{
      movementControlFinish();
    }
  }
  else if(mControlMode2){
    if(mControlDirection == 1){
      setMHorizontal(-1, base_thrust + mControlPowerTemp, -1, base_thrust - mControlPowerTemp);
    }
    else if(mControlDirection == 2){
      setMHorizontal(base_thrust + mControlPowerTemp, -1, base_thrust + mControlPowerTemp, -1);
    }
    else if(mControlDirection == 3){
      setMHorizontal(-1, base_thrust - mControlPowerTemp, -1, base_thrust + mControlPowerTemp);
    }
    else if(mControlDirection == 4){
      setMHorizontal(base_thrust - mControlPowerTemp, -1, base_thrust - mControlPowerTemp, -1);
    }
    else{
      movementControlFinish();
    }
    double l1 = max(positionX, positionXPrev) - min(positionX, positionXPrev);
    double l2 = max(positionY, positionYPrev) - min(positionY, positionYPrev);
    double calculatedDistance = sqrt(l1*l1 + l2*l2);
    if(calculatedDistance >= mControlDistance){
      ROS_INFO("Mode 2 finished.\n");
      movementControlFinish();
    }
  }
  else if(mControlMode3){
    if(mControlDirection == 1){
      setMHorizontal(-1, base_thrust + mControlPowerTemp, -1, base_thrust - mControlPowerTemp);
    }
    else if(mControlDirection == 2){
      setMHorizontal(base_thrust + mControlPowerTemp, -1, base_thrust + mControlPowerTemp, -1);
    }
    else if(mControlDirection == 3){
      setMHorizontal(-1, base_thrust - mControlPowerTemp, -1, base_thrust + mControlPowerTemp);
    }
    else if(mControlDirection == 4){
      setMHorizontal(base_thrust - mControlPowerTemp, -1, base_thrust - mControlPowerTemp, -1);
    }
    else{
      movementControlFinish();
    }
    if((loopTime - mControlMode3Timer) > (mControlRunningTime)){
      // mControlMode3 = false;
      ROS_INFO("Mode 3 finished.\n");
      movementControlFinish();
    }
  }
}

// keeps sub in current position
void positionControl(){
  if(isDoingMovement()){

    double keepCoursePower = 0;
    if(isDoingStrafeMovement()){
      positionControlMOffset += (-velocityY)*keepCourseRightMult;
      keepCoursePower = motorPowerCap(positionControlMOffset);
      setMHorizontalOffset(0, keepCoursePower, 0, -keepCoursePower);
    }
    else{
      positionControlMOffset += (-velocityX)*keepCourseForwardMult;
      keepCoursePower = motorPowerCap(positionControlMOffset);
      setMHorizontalOffset(keepCoursePower, 0, keepCoursePower, 0);
    }
  }
  else{
    positionControlMOffset = 0; //rezero position keeping for movementControl
    double pDirection = DVLHelper::getDirection(positionX, positionY, keepPositionX, keepPositionY, yaw);
    double pDistance = DVLHelper::getDistance(positionX, positionY, keepPositionX, keepPositionY);
    double forwardBackwardDistance, rightLeftDistance;
    double forwardBackwardOffset = 0, rightLeftOffset = 0;
    if(pDirection >= 0){
      //on right side of sub rightleftoffset must be positive
      if(pDirection < 90){
        //move forward positive forwardbackwardoffset
        forwardBackwardDistance = abs(cos(DVLHelper::deg2rad(abs(pDirection))) * pDistance);
        forwardBackwardOffset = (forwardBackwardDistance/keepPositionForwardThreshold) * keepPositionMaxPowerForwardBackward + positionForwardPID(forwardBackwardDistance);
        rightLeftDistance = abs(sin(DVLHelper::deg2rad(abs(pDirection))) * pDistance);
        rightLeftOffset = (rightLeftDistance/keepPositionRightThreshold) * keepPositionMaxPowerRightLeft + positionRightPID(rightLeftDistance);
      }
      else{
        //move backward negative forwardbackwardoffset
        forwardBackwardDistance = abs(sin(DVLHelper::deg2rad(abs(pDirection) - 90)) * pDistance);
        forwardBackwardOffset = -((forwardBackwardDistance/keepPositionForwardThreshold) * keepPositionMaxPowerForwardBackward + positionForwardPID(forwardBackwardDistance));
        rightLeftDistance = abs(cos(DVLHelper::deg2rad(abs(pDirection) - 90)) * pDistance);
        rightLeftOffset = (rightLeftDistance/keepPositionRightThreshold) * keepPositionMaxPowerRightLeft + positionRightPID(rightLeftDistance);
      }
    }
    else{
      //on left side of sub rightleftoffset must be negative
      if(pDirection > -90){
        //move forward positive forwardbackwardoffset
        forwardBackwardDistance = abs(cos(DVLHelper::deg2rad(abs(pDirection))) * pDistance);
        forwardBackwardOffset = (forwardBackwardDistance/keepPositionForwardThreshold) * keepPositionMaxPowerForwardBackward + positionForwardPID(forwardBackwardDistance);
        rightLeftDistance = abs(sin(DVLHelper::deg2rad(abs(pDirection))) * pDistance);
        rightLeftOffset = -((rightLeftDistance/keepPositionRightThreshold) * keepPositionMaxPowerRightLeft + positionRightPID(rightLeftDistance));
      }
      else{
        //move backward negative forwardbackwardoffset
        forwardBackwardDistance = abs(sin(DVLHelper::deg2rad(abs(pDirection) - 90)) * pDistance);
        forwardBackwardOffset = -(forwardBackwardDistance/keepPositionForwardThreshold * keepPositionMaxPowerForwardBackward + positionForwardPID(forwardBackwardDistance));
        rightLeftDistance = abs(cos(DVLHelper::deg2rad(abs(pDirection) - 90)) * pDistance);
        rightLeftOffset = -((rightLeftDistance/keepPositionRightThreshold) * keepPositionMaxPowerRightLeft + positionRightPID(rightLeftDistance));
      }
    }
    // cout << "forwardBackwardDistance: " << forwardBackwardDistance << " rightLeftDistance: " << rightLeftDistance << endl;
    //keep prev distance for pid
    forwardBackwardDistancePrev = forwardBackwardDistance;
    rightLeftDistancePrev = rightLeftDistance;
    //cap the offset amount
    if(forwardBackwardOffset > keepPositionMaxPowerForwardBackward){
      forwardBackwardOffset = keepPositionMaxPowerForwardBackward;
    }
    else if(forwardBackwardOffset < -keepPositionMaxPowerForwardBackward){
      forwardBackwardOffset = -keepPositionMaxPowerForwardBackward;
    }
    if(rightLeftOffset > keepPositionMaxPowerRightLeft){
      rightLeftOffset = keepPositionMaxPowerRightLeft;
    }
    else if(rightLeftOffset < -keepPositionMaxPowerRightLeft){
      rightLeftOffset = -keepPositionMaxPowerRightLeft;
    }

    setMHorizontalOffset(rightLeftOffset, forwardBackwardOffset, rightLeftOffset, -forwardBackwardOffset);
  }
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

  //initialize offset for position keeping
  mHorizontalOffsetT5 = 0;
  mHorizontalOffsetT6 = 0;
  mHorizontalOffsetT7 = 0;
  mHorizontalOffsetT8 = 0;

  //Initialize variables
  resetAll();
  loopTime = millis();
  timePrev = loopTime;
  loopTimePrev = loopTime;
  elapsedTime = 0;
  ROS_INFO("Sub is staying. Waiting to receive data from master...\n");
}
void loop() {
  loopTime = millis();  // update time

  if(subIsReady){
    movementControl();
  } else {
    motorsOff();
  }

  if((loopTime-loopTimePrev) > loopInterval) {
    loopTimePrev = loopTime;

    if(subIsReady){
      elapsedTime = (loopTime - timePrev) /1500; //1500;
      timePrev = loopTime; //update to current time after done with previous time
      heightControl();
      rotationControl();
      positionControl();
    } else {
      motorsOff();
    }
    publishMotors();
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
  ledPublisher = nh.advertise<std_msgs::Int8>("led",1);

  currentDepthSubscriber = nh.subscribe("current_depth", 1, currentDepthCallback);
  hControlSubscriber = nh.subscribe("height_control", 100, hControlCallback);
  rControlSubscriber = nh.subscribe("rotation_control", 100, rControlCallback);
  mControlSubscriber = nh.subscribe("movement_control", 100, mControlCallback);
  rotationSubscriber = nh.subscribe("current_rotation", 1, rotationCallback);
  dvlSubscriber = nh.subscribe("dvl_status", 1, dvlCallback);
  //  dvlHeadingSubscriber = nh.subscribe("dvl_heading", 1, dvlHeadingCallback);

  //set number of threads to run callbacks
  ros::AsyncSpinner spinner(1);
  spinner.start();
  setup();
  while(ros::ok()){
    loop();
  }

  return 0;
}
