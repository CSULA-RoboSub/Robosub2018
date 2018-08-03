#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <hardware_interface/MotorHorizontal.h>
#include <hardware_interface/MotorVertical.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <Wire.h>
#include "MS5837.h"
#include "SFE_LSM9DS0.h"
#include "fontv1.h";

#define GyroMeasError PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

#define LSM9DS0_XM  0x1D
#define LSM9DS0_G   0x6B
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
//-------------------------------------------------------------------------------
//pin section
//LCD Display
#define RST 48
#define CE  47
#define DC  46
#define DIN 45
#define CLK 44

//reed switch pin
const int REED = 13;

const byte INT1XM = 53; // INT1XM tells us when accel data is ready
const byte INT2XM = 51; // INT2XM tells us when mag data is ready
const byte DRDYG  = 49; // DRDYG  tells us when gyro data is ready

//end pin section
//--------------------------------------------------------------------------------

//Initialization of the Servo's for Blue Robotics Motors
Servo T1;     //
Servo T2;     //
Servo T3;     //
Servo T4;     //
Servo T5;     //
Servo T6;     //
Servo T7;     //
Servo T8;     //

MS5837 sensor;

//variable for LCD 
char string[8];

//CHANGED PWM_Motors TO PWM_Motors_depth SINCE THERE ARE 2 DIFFERENT PWM CALCULATIONS
//ONE IS FOR DEPTH AND THE OTHER IS USED FOR MOTORS TO ROTATE TO PROPER NEW LOCATION
// int PWM_Motors_Depth;
// float dutyCycl_depth;
float assignedDepth;
float feetDepth_read;

int reedVal = HIGH;  //reed switch value

ros::NodeHandle nh;
std_msgs::Float32 currentDepth;
ros::Publisher currentDepthPublisher("current_depth", &currentDepth);           //float: depth

void mVerticalCallback(const hardware_interface::MotorVertical& mVertical);
void mHorizontalCallback(const hardware_interface::MotorHorizontal& mHorizontal);

ros::Subscriber<hardware_interface::MotorVertical> mVerticalSubscriber("motor_vertical", &mVerticalCallback);
ros::Subscriber<hardware_interface::MotorHorizontal> mHorizontalSubscriber("motor_horizontal", &mHorizontalCallback);

float t1Power, t2Power, t3Power, t4Power, t5Power, t6Power, t7Power, t8Power;
const float motorBase = 1500;
const float motorMin = 1300;
const float motorMax = 1700;

//time variables
unsigned long loopTime, loopTimePrev;
const unsigned int loopInterval = 10;

void setup() {

  //For servo motors on Pelican, pins 2-5 are for motors 1-4. PWM on these motors are 1100-1499 (counter
  //clockwise direction) and 1501-1900 (clockwise direction). Note that in code I use pins 6-8, this was used
  //for testing with leds.
  pinMode(INT1XM, INPUT);
  pinMode(INT2XM, INPUT);
  pinMode(DRDYG,  INPUT);

  //motor pins 
  pinMode(2, OUTPUT); //1 on motor
  pinMode(3, OUTPUT); //2 on motor
  pinMode(4, OUTPUT); //3 on motor
  pinMode(5, OUTPUT); //4 on motor
  pinMode(6, OUTPUT); //5 on motor
  pinMode(7, OUTPUT); //6 on motor
  pinMode(8, OUTPUT); //7 on motor
  pinMode(9, OUTPUT); //8 on motor

  //Pins for LCD Display (NOKIA 5110)
  pinMode(RST, OUTPUT);
  pinMode(CE, OUTPUT);
  pinMode(DC, OUTPUT);
  pinMode(DIN, OUTPUT);
  pinMode(CLK, OUTPUT);
  digitalWrite(RST, LOW);
  digitalWrite(RST, HIGH);

  //Reed switch pin
  pinMode(REED, INPUT);

  //Initialization of LCD 
 //  LcdWriteCmd(0x21);                    //LCD extended commands
 //  LcdWriteCmd(0xB8);                    //set LCD VOp (contrast)
 //  LcdWriteCmd(0x04);                    //set emp coefficent
 //  LcdWriteCmd(0x14);                    //LCD bias mode 1:30
 //  LcdWriteCmd(0x20);                    //LCD basic commands
 //  LcdWriteCmd(0x0C);                    

 //  for (int i = 0; i < 504; i++) {
 //    LcdWriteData(0x00);
 //  }

 // LcdXY(0,0);
 // LcdWriteString("Roll: ");
 // LcdXY(0,2);
 // LcdWriteString("Pitch: ");
 // LcdXY(0,4);
 // LcdWriteString("Yaw: ");

  t1Power = motorBase; 
  t2Power = motorBase;
  t3Power = motorBase;
  t4Power = motorBase;
  t5Power = motorBase;
  t6Power = motorBase;
  t7Power = motorBase;
  t8Power = motorBase;
 //Pelican Motors activation of motors (initialization of pins to servo motors)
  T1.attach(2); //right front servo
  T1.writeMicroseconds(motorBase);
  T2.attach(3); //right back servo
  T2.writeMicroseconds(motorBase);
  T3.attach(4); //back left servo
  T3.writeMicroseconds(motorBase);
  T4.attach(5); //front left servo
  T4.writeMicroseconds(motorBase);
  T5.attach(6); //front left servo
  T5.writeMicroseconds(motorBase);
  T6.attach(7); //front left servo
  T6.writeMicroseconds(motorBase);
  T7.attach(8); //front left servo
  T7.writeMicroseconds(motorBase);
  T8.attach(9); //front left servo
  T8.writeMicroseconds(motorBase);

  delay(1000);


  Wire.begin();
  // Serial.begin(57600);
  // Serial.println("Serial 57600");

  //Initialize ROS variable
  feetDepth_read = 0;
  currentDepth.data = feetDepth_read;

//  assignedDepth = 0.2;
  //assignedYaw = -191.5;
  //subIsReady = true;
  // assignedYaw = 64.6;
//  currentRotation.data = yaw;

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(mVerticalSubscriber);
  nh.subscribe(mHorizontalSubscriber);
  nh.advertise(currentDepthPublisher);
//  nh.advertise(currentRotationPublisher);

//  initializeIMU();
//
  sensor.init();
  sensor.setFluidDensity(997); // kg/m^3 (997 freshwater, 1029 for seawater)

  loopTime = millis();
  loopTimePrev = loopTime;
//  nh.loginfo("Data is ready.");

  delay(1000);
  nh.loginfo("Sub is staying. Waiting to receive data from master...\n");

}

void loop() {
  nh.spinOnce();
  loopTime = millis();  // actual time read
  //  gettingRawData();

  if((loopTime-loopTimePrev) > loopInterval) {
    loopTimePrev = loopTime;
    sensor.read();

    //Set the display outputs for roll, pitch, and yaw
    // LcdXY(40, 0);
    // LcdWriteString(dtostrf(roll, 5, 2, string));
    // LcdXY(40, 2);
    // LcdWriteString(dtostrf(pitch, 5, 2, string));
    // LcdXY(40, 4);
    // LcdWriteString(dtostrf(yaw, 5, 2, string));

    //read reed switch
    // reedVal = digitalRead(REED);
    reedVal = LOW;
    
    // nh.loginfo("--------------------------");
    // nh.loginfo(HIGH);
    // nh.loginfo(LOW);
    // nh.loginfo(reedVal);
    // char reedChar[4];
    // dtostrf(reedVal, 4, 0, reedChar);
    // nh.loginfo(reedChar);
    // if(reedVal == LOW){

    //   nh.loginfo("reedVal == LOW");
    // } else if (reedVal == HIGH) {

    //   nh.loginfo("reedVal == HIGH");
    // }
    //Depth
    //Testing----------------------
    feetDepth_read =  sensor.depth() * 3.28 + 1.8;                                   //1 meter = 3.28 feet
    // dutyCycl_depth = (abs(assignedDepth - feetDepth_read)/ 13.0);              //function to get a percentage of assigned height to the feet read
    // PWM_Motors_Depth = dutyCycl_depth * 400;                                   //PWM for motors are between 1500 - 1900; difference is 400
    nh.spinOnce();
    if(reedVal == LOW){
      writeMotors();
    }else{
      killSwitch();
    }
    nh.spinOnce();

    //Update and publish current data to master
    currentDepth.data = feetDepth_read;
    currentDepthPublisher.publish(&currentDepth);
    nh.spinOnce();
  }
}

float motorPowerRangeProtection(float power){
  if(power < motorMin)
    return motorMin;
  else if(power > motorMax)
    return motorMax;

  return power;
}

void mVerticalCallback(const hardware_interface::MotorVertical& mVertical){
   // nh.loginfo("adjust vertical power");
  t1Power = motorPowerRangeProtection(mVertical.t1);
  t2Power = motorPowerRangeProtection(mVertical.t2);
  t3Power = motorPowerRangeProtection(mVertical.t3);
  t4Power = motorPowerRangeProtection(mVertical.t4);
}

void mHorizontalCallback(const hardware_interface::MotorHorizontal& mHorizontal){
   // nh.loginfo("adjust horizontal power");
  t5Power = motorPowerRangeProtection(mHorizontal.t5);
  t6Power = motorPowerRangeProtection(mHorizontal.t6);
  t7Power = motorPowerRangeProtection(mHorizontal.t7);
  t8Power = motorPowerRangeProtection(mHorizontal.t8);
}

void writeMotors(){
  // nh.loginfo("writeMotors");
  T1.writeMicroseconds(t1Power);
  T2.writeMicroseconds(t2Power);
  T3.writeMicroseconds(t3Power);
  T4.writeMicroseconds(t4Power);
  T5.writeMicroseconds(t5Power);
  T6.writeMicroseconds(t6Power);
  T7.writeMicroseconds(t7Power);

  //offset to fix uneven thrust
  if(t8Power == motorBase)
    T8.writeMicroseconds(t8Power);  
  else
    T8.writeMicroseconds(t8Power-10);
}
//reed switch helper function
void killSwitch(){
  T1.writeMicroseconds(motorBase);
  T2.writeMicroseconds(motorBase);
  T3.writeMicroseconds(motorBase);
  T4.writeMicroseconds(motorBase);
  T5.writeMicroseconds(motorBase);
  T6.writeMicroseconds(motorBase);
  T7.writeMicroseconds(motorBase);
  T8.writeMicroseconds(motorBase);
}

// void LcdWriteString(char *characters){
//   while(*characters) LcdWriteCharacter(*characters++);
// }

// void LcdWriteCharacter(char character){
//   for (int i = 0; i < 5; i++) 
//   {
//     LcdWriteData(ASCII[character - 0x20][i]);
//   }
//   LcdWriteData(0x00);
//   }

// void LcdWriteData(byte dat)
// {
//   digitalWrite(DC, HIGH);
//   digitalWrite(CE, LOW);
//   shiftOut(DIN, CLK, MSBFIRST, dat);
//   digitalWrite(CE, HIGH);
// }

// void LcdWriteCmd(byte cmd)
// {
//   digitalWrite(DC, LOW);                 //DC pin is low for commands
//   digitalWrite(CE, LOW);                                
//   shiftOut(DIN, CLK, MSBFIRST, cmd);     //transmit serial data
//   digitalWrite(CE, HIGH);
// }

// void LcdXY(int x, int y){
//   LcdWriteCmd(0x80 | x);
//   LcdWriteCmd(0x40 | y);
// }

