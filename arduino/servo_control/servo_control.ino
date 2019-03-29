#include <ros.h>
#include <std_msgs/Int8.h>
#include <robosub/Torpedo.h>
#include <robosub/Dropper.h>
#include <SparkFun_Tlc5940.h>
#include <avr/power.h>
#include <tlc_servos.h>

//Tlc5940 Tlc;/

ros::NodeHandle nh;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(64, 6, NEO_GRB + NEO_KHZ800);
int DROPPER_BOTTOM_SERVO_PIN = 0;
int DROPPER_TOP_SERVO_PIN = 1;
int TORPEDO_LEFT_SERVO_PIN = 2;
int TORPEDO_RIGHT_SERVO = 3;

void torpedo_cb(const robosub::Torpedo& msg) {
  /*
  - state: 0 = prime, 1 = fire
  - torpedo_number: 0 = left, 1 = right
  */

  //prime
  if(msg.state == 0){
    //left
    if(msg.torpedo_number == 0){
      tlc_setServo(TORPEDO_LEFT_SERVO_PIN, 0);
      Tlc.update();
      delay(300);
    }
    //right
    else if(msg.torpedo_number == 1){
      tlc_setServo(TORPEDO_RIGHT_SERVO, 0);
      Tlc.update();
      delay(300);
    }
  }

  //fire
  if(msg.state == 1){
    //left
    if(msg.torpedo_number == 0){
      tlc_setServo(TORPEDO_RIGHT_SERVO, 0);
      Tlc.update();
      delay(300);
    }
    //right
    else if(msg.torpedo_number == 1){
      tlc_setServo(TORPEDO_RIGHT_SERVO, 50);
      Tlc.update();
      delay(300);
    }
  }
}

void dropper_cb(const robosub::Dropper& msg) {
  /*
  - state: 0 = close both gates, 1 = open both gates, 2 = drop one ball
  */

  //close both gates
  if(msg.state == 0){
    tlc_setServo(DROPPER_BOTTOM_SERVO_PIN,0);
    Tlc.update();
    delay(30);
    tlc_setServo(DROPPER_TOP_SERVO_PIN,0);
    Tlc.update();
  }
  //open both gates
  else if(msg.state == 1){
    tlc_setServo(DROPPER_BOTTOM_SERVO_PIN,250);
    int test = Tlc.update();
    Serial.println(test);
    delay(30);
    tlc_setServo(DROPPER_TOP_SERVO_PIN,250);
    Tlc.update();
  }
  //drop one ball
  else if(msg.state == 2){
    tlc_setServo(DROPPER_BOTTOM_SERVO_PIN,0);
    Tlc.update();
    delay(3000);
    tlc_setServo(DROPPER_BOTTOM_SERVO_PIN,250);
    Tlc.update();
    delay(3000);
    tlc_setServo(DROPPER_TOP_SERVO_PIN,0);
    Tlc.update();
    delay(3000);
    tlc_setServo(DROPPER_TOP_SERVO_PIN,250);
    Tlc.update();
    delay(3000);
  }
}

ros::Subscriber<robosub::Torpedo> torpedo_subscriber("torpedo", &torpedo_cb);
ros::Subscriber<robosub::Dropper> dropper_subscriber("dropper", &dropper_cb);


void setup() {

  tlc_initServos();
  Serial.begin(9600);
  nh.initNode();
  nh.subscribe(torpedo_subscriber);
  nh.subscribe(dropper_subscriber);
  delay(1000)

}

void loop() {
  nh.spinOnce();
}
// publish servo
