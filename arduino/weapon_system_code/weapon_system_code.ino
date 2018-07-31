#include <SparkFun_Tlc5940.h>
#include <tlc_servos.h>

//Tlc5940 Tlc;/

int DropperBottomServo = 0;
int DropperTopServo = 1;
int TorpedoLeftServo = 2;
int TorpedoRightServo = 3;

int usercommand = 0;

void setup() {
  tlc_initServos();
  Serial.begin(9600);
}
 
void loop() {
  Serial.println("Enter Dropper or Torpedo Commands: ");
  Serial.println("1 = Opens Both Dropper gates");
  Serial.println("2 = Closes Both Dropper gates");
  Serial.println("3 = Drop one ball");
  Serial.println("4 = Prime Left Torpedo");
  Serial.println("5 = Fire Left Torpedo");
  Serial.println("6 = Prime Right Torpedo");
  Serial.println("7 = Fire Right Torpedo");
  while(Serial.available()==0){
  }
  usercommand = Serial.parseInt();
  Serial.println(usercommand);
  if (usercommand == 1) {
      Serial.println("in 1");
      tlc_setServo(DropperBottomServo,250);
      int test = Tlc.update();
      Serial.println(test);
      delay(30);
      tlc_setServo(DropperTopServo,250);
      Tlc.update();
    }
  else if (usercommand == 2) {
      tlc_setServo(DropperBottomServo,0);
      Tlc.update();
      delay(30);
      tlc_setServo(DropperTopServo,0);
      Tlc.update();
    }
  else if (usercommand == 3) {
      tlc_setServo(DropperBottomServo,0);
      Tlc.update();
      delay(3000);
      tlc_setServo(DropperBottomServo,250);
      Tlc.update(); 
      delay(3000);
      tlc_setServo(DropperTopServo,0);
      Tlc.update();
      delay(3000);
      tlc_setServo(DropperTopServo,250);
      Tlc.update();
      delay(3000);
    }
  else if(usercommand == 4){
      tlc_setServo(TorpedoLeftServo, 0);
      Tlc.update();
      delay(300);
    }
  else if(usercommand == 5){
      tlc_setServo(TorpedoLeftServo, 50);
      Tlc.update();
      delay(300);
    }
  else if(usercommand == 6){
      tlc_setServo(TorpedoRightServo, 0);
      Tlc.update();
      delay(300);
    }
  else if(usercommand == 7){
      tlc_setServo(TorpedoRightServo, 50);
      Tlc.update();
      delay(300);
    }
}
