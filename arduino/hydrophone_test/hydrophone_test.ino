#include <ros.h>
#include <robosub/Hydrophone.h>
#include <SoftwareSerial.h>

ros::NodeHandle nh;
SoftwareSerial mySerial(10, 11); //rx, tx

robosub::Hydrophone hStatus;

ros::Publisher hStatusPublisher("hydrophone_status", &hStatus);

void resetStatus();

const int numHydrophone = 4;
bool readStatus[4];
uint16_t times1[4];
uint16_t times2[4];
uint16_t times3[4];
uint16_t times4[4];
char type;

unsigned long timer;
unsigned long init_timer;

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(hStatusPublisher);

  init_timer = 0;

  //  Serial.begin(115200);
  mySerial.begin(115200);

  resetStatus();

  //dummy 4th hydrophone
  for (int i; i < 4; i++) {
    times1[i] = 0;
    times2[i] = 0;
    times3[i] = 0;
    times4[i] = 0;
  }

  delay(1000);
  nh.loginfo("setup complete.");
}

void loop() {
  timer = millis();
  if (init_timer){
    if(timer-init_timer > 20){
      resetStatus();
    }
  }
  
  if (mySerial.available()) {
    if(!init_timer){
      init_timer = timer;
    }
    
    type = mySerial.read();
    //    nh.loginfo("available");
    if (type == 'a') {
      times1[0] = mySerial.read();
      times1[1] = mySerial.read();
      times1[2] = mySerial.read();
      times1[3] = mySerial.read();
      readStatus[0] = true;
      // nh.loginfo("type a");
    } else if (type == 'b') {
      times2[0] = mySerial.read();
      times2[1] = mySerial.read();
      times2[2] = mySerial.read();
      times2[3] = mySerial.read();
      readStatus[1] = true;
      // nh.loginfo("type b");
    }
    else if (type == 'c') {
      times3[0] = mySerial.read();
      times3[1] = mySerial.read();
      times3[2] = mySerial.read();
      times3[3] = mySerial.read();
      readStatus[2] = true;
      // nh.loginfo("type c");
    }
    else if (type == 'd') {
      times4[0] = mySerial.read();
      times4[1] = mySerial.read();
      times4[2] = mySerial.read();
      times4[3] = mySerial.read();
      readStatus[3] = true;
      // nh.loginfo("type d");
    }
  }

  int check = 0;
  for (int i = 0; i < 4; i++) {
    if (readStatus[i] == true) {
      check++;
    }
  }

  if (check >= numHydrophone) {
    resetStatus();

    for (int i = 0; i < 4; i++) {
      hStatus.times1[i] = times1[i];
      hStatus.times2[i] = times2[i];
      hStatus.times3[i] = times3[i];
      hStatus.times4[i] = times4[i];
    }
//    nh.loginfo("publish");
    hStatusPublisher.publish(&hStatus);
  }

  nh.spinOnce();
  //  delay(10);
}

void resetStatus() {
  init_timer = 0;
  for (int i = 0; i < numHydrophone; i++) {
    readStatus[i] = false;
  }
}
