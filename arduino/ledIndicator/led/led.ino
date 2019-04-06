//****************************************************************************************************************************//
/////////////////////////California State University Los Angeles - Autonomous Underwater Vehicle project////////////////////////
//////////////////////////////////AUVCalStateLa Neopixel sketch for AUV motion/action states.///////////////////////////////////
/////////////////////////////////////////////June 28, 2018 - by J.Diego Santillan///////////////////////////////////////////////
//****************************************************************************************************************************//
#include <ros.h>
#include <std_msgs/Int8.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#define FrontPin 6
#define BackPin 9

Adafruit_NeoPixel FrontStrip = Adafruit_NeoPixel(18, FrontPin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel BackStrip = Adafruit_NeoPixel(18, BackPin, NEO_GRB + NEO_KHZ800);

ros::NodeHandle nh;

void messageCb(const std_msgs::Int8& led_msg){

  switch (led_msg.data){

    case 0:// Stand by
    //Front is solid - Green
    //Back is solid - Green
    idle(FrontStrip.Color(0, 128, 0), 50);
    break;

    case 1: //forward
    //Front is solid - Green
    //Back is rotating - Green
    forward(FrontStrip.Color(0, 128, 0), 50);
    break;

    case 2: //reverse
    //Front is solid - Green
    //Back is rotating - White
    backward(FrontStrip.Color(0, 128, 0),FrontStrip.Color(255,255,255), 50);
    break;

    case 3://submerging
    //Front is bottom half - Blue
    //Back is bottom half - Blue
    submerging(FrontStrip.Color(0, 0, 255), 50);
    break;

    case 4: //emerging
    //Front is top half - Blue
    //Back is top half - Blue
    emerging(FrontStrip.Color(0, 0, 255), 50);
    break;

    //  case 5: //Desired Depth reached
    //  theaterChaseRainbow(50); //Rainbow
    //  break;
    //
    case 6: //turn left
    //Front is divided - Left 1/2 Orange Right 1/2 Green
    //Back is divided - Left 1/2 Green Right 1/2 Orange
    turningLeft(FrontStrip.Color(0,128, 0),FrontStrip.Color(255, 165,0), 50);
    break;

    case 7: //turn right
    //Front is divided - Left 1/2 Green Right 1/2 Orange
    //Back is divided - Left 1/2 Orange Right 1/2 Green
    turningRight(FrontStrip.Color(0,128, 0),FrontStrip.Color(255, 165,0), 50);
    break;

    case 8: //strafing left
    //Front is left half - Green
    //Back is left half - Orange
    strafeLeft(FrontStrip.Color(0,128, 0),FrontStrip.Color(255, 165,0), 50);
    break;

    case 9: //strafing right
    //Front is right half - Orange
    //Back is right half - Green
    strafeRight(FrontStrip.Color(0,128, 0),FrontStrip.Color(255, 165,0), 50);
    break;

    case 10://Computer Vision search
    //Front is solid - Magenta
    //Back is solid - Yellow
    computerVisionSearch(FrontStrip.Color(255, 0, 255), FrontStrip.Color(255, 255, 0),50); //Flashing Fuchsia
    break;

    case 11://Computer Vision target found
    //Front is solid - Blue
    //Back is solid - Yellow
    cvFoundTask(FrontStrip.Color(0, 128, 0), 50); //Solid Fuchsia
    break;
    //
    //  case 12://Hydrophones search
    //  theaterChase(strip.Color(51, 255, 255), 50); //Flashing Aqua
    //  break;
    //
    //  case 13://Hydrophone target found
    //  colorWipe(strip.Color(255, 0, 255), 50); //Solid Aqua
    //  break;

    case 14://Torpedo fired
    //Front is solid - Red
    //Back is solid - Yellow
    torpedos(FrontStrip.Color(255, 0, 0),FrontStrip.Color(255, 255, 0), 50);
    break;

    case 15://Mechanical Arm engaged
    //Front is solid - Blue
    //Back is solid - Yellow
    roboticArm(FrontStrip.Color(0, 0, 255),FrontStrip.Color(255, 255, 0), 50);
    break;

    case 16://Starting a Task
    //Front is solid - Yellow
    //Back is solid - Yellow
    startingTask(FrontStrip.Color(255,255,0),50);
    break;

    case 17://Sub is performing the task
    //Front is rotating - Rainbow
    //Back is rotating - Rainbow
    performingTask(20);
    break;


    case 18://Using Marker Dropper
    //Front is solid - Yellow
    //Back is Offset - Yellow
    markerDropper(FrontStrip.Color(255, 255, 0), 50);
    break;
  }
}


void emerging(uint32_t color, uint8_t wait){
  for(uint16_t i=0; i<FrontStrip.numPixels(); i++) {
    FrontStrip.setPixelColor(i, color);
    BackStrip.setPixelColor(i,color);
    if (i < 10 || i > 30){
      FrontStrip.setPixelColor(i, 0);
      BackStrip.setPixelColor(i, 0);
    }
  }
  //strip.show();
  //delay(wait);
}

void submerging(uint32_t color, uint8_t wait){

  for(uint16_t i=0; i<FrontStrip.numPixels(); i++) {
    FrontStrip.setPixelColor(i, color);
    BackStrip.setPixelColor(i,color);
    if (i > 10 || i < 30){
      FrontStrip.setPixelColor(i, 0);
      BackStrip.setPixelColor(i, 0);
    }
  }
  //   strip.show();
  //   delay(wait);
}

void strafeLeft(uint32_t firstColor, uint32_t secondColor,  uint8_t wait){
  for(uint16_t i=0; i<FrontStrip.numPixels(); i++) {
    FrontStrip.setPixelColor(i, secondColor);
    BackStrip.setPixelColor(i, secondColor);
    if (i > (FrontStrip.numPixels() / 2)){
      FrontStrip.setPixelColor(i, firstColor);
      BackStrip.setPixelColor(i, firstColor);
    }
  }
  //
  // for(uint16_t i=0; i<FrontStrip.numPixels(); i++) {
  //    FrontStrip.setPixelColor(i, color);
  //    BackStrip.setPixelColor(i,color);
  //  if (i > (FrontStrip.numPixels() / 2)){
  //    FrontStrip.setPixelColor(i, 0);
  //    BackStrip.setPixelColor(i, 0);
  //  }
  //  }
  //   strip.show();
  //   delay(wait);
}

void strafeRight(uint32_t firstColor,uint32_t secondColor, uint8_t wait){
  for(uint16_t i=0; i<FrontStrip.numPixels(); i++) {
    FrontStrip.setPixelColor(i, firstColor);
    BackStrip.setPixelColor(i, firstColor);
    if (i > (FrontStrip.numPixels() / 2)){
      FrontStrip.setPixelColor(i, secondColor);
      BackStrip.setPixelColor(i, secondColor);
    }
  }
  // for(uint16_t i=FrontStrip.numPixels(); i<0; i--) {
  //    FrontStrip.setPixelColor(i, color);
  //    BackStrip.setPixelColor(i,color);
  //  if (i < (FrontStrip.numPixels() / 2)){
  //    FrontStrip.setPixelColor(i, color);
  //    BackStrip.setPixelColor(i,color);
  //  }
  //  }
  //   strip.show();
  //   delay(wait);
}

void performingTask(uint8_t wait) {

  for(int j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(uint16_t i=0; i< FrontStrip.numPixels(); i++) {
      FrontStrip.setPixelColor(i, Wheel(((i * 256 / FrontStrip.numPixels()) + j) & 255));
      BackStrip.setPixelColor(i, Wheel(((i * 256 / BackStrip.numPixels()) + j) & 255));
    }
    FrontStrip.show();
    BackStrip.show();
    delay(wait);
  }
}


void forward(uint32_t color, uint8_t wait){

  for(uint16_t i=20; i<FrontStrip.numPixels(); i++) {
    FrontStrip.setPixelColor(i, color);

    for (int j=0; j<10; j++) {  //do 10 cycles of chasing
      for (int q=0; q < 3; q++) {
        for (uint16_t i=0; i < BackStrip.numPixels(); i=i+3) {
          BackStrip.setPixelColor(i+q, color);    //turn every third pixel on
        }

        BackStrip.show();

        delay(wait);

        for (uint16_t i=0; i < BackStrip.numPixels(); i=i+3) {
          BackStrip.setPixelColor(i+q, 0);        //turn every third pixel off
        }
      }
    }
  }
}

void backward(uint32_t frontColor, uint32_t backColor, uint8_t wait){
  for(uint16_t i=0; i<FrontStrip.numPixels(); i++) {
    FrontStrip.setPixelColor(i, frontColor);
    if (i > (FrontStrip.numPixels() / 2)){
      FrontStrip.setPixelColor(i, frontColor);
    }
  }
  for(uint16_t i=0; i<BackStrip.numPixels(); i++) {
    BackStrip.setPixelColor(i, backColor);
    BackStrip.show();
    delay(wait);
  }
}

void turningLeft(uint32_t firstColor, uint32_t secondColor, uint8_t wait){
  for(uint16_t i=0; i<FrontStrip.numPixels(); i++) {
    FrontStrip.setPixelColor(i, secondColor);
    BackStrip.setPixelColor(i, firstColor);
    if (i > (FrontStrip.numPixels() / 2)){
      FrontStrip.setPixelColor(i, firstColor);
      BackStrip.setPixelColor(i, secondColor);
    }
  }
}

void turningRight(uint32_t firstColor, uint32_t secondColor, uint8_t wait){
  for(uint16_t i=0; i<FrontStrip.numPixels(); i++) {
    FrontStrip.setPixelColor(i, firstColor);
    BackStrip.setPixelColor(i, secondColor);
    if (i > (FrontStrip.numPixels() / 2)){
      FrontStrip.setPixelColor(i, secondColor);
      BackStrip.setPixelColor(i, firstColor);
    }
  }
}

void startingTask(uint32_t color, uint8_t wait){
  for(uint16_t i=0; i<FrontStrip.numPixels(); i++) {
    FrontStrip.setPixelColor(i, color);
    BackStrip.setPixelColor(i,color);
  }
}

void idle(uint32_t color,uint8_t wait){
  for(uint16_t i=0; i<FrontStrip.numPixels(); i++) {
    FrontStrip.setPixelColor(i, color);
    BackStrip.setPixelColor(i, color);
  }
}


void computerVisionSearch(uint32_t firstColor, uint32_t secondColor,uint8_t wait){
  for(uint16_t i=0; i<FrontStrip.numPixels(); i++) {
    FrontStrip.setPixelColor(i, firstColor);
    BackStrip.setPixelColor(i, secondColor);
  }
}

void cvFoundTask(uint32_t color, uint8_t wait){
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < BackStrip.numPixels(); i=i+3) {
        FrontStrip.setPixelColor(i+q,color);
        BackStrip.setPixelColor(i+q, color);    //turn every third pixel on
      }
      FrontStrip.show();
      BackStrip.show();
      delay(wait);

      for (uint16_t i=0; i < BackStrip.numPixels(); i=i+3) {
        FrontStrip.setPixelColor(i+q,0);
        BackStrip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }


}

uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
    return FrontStrip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
    return FrontStrip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return FrontStrip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

void roboticArm(uint32_t firstColor, uint32_t secondColor, uint8_t wait){
  for(uint16_t i=0; i<FrontStrip.numPixels(); i++) {
    FrontStrip.setPixelColor(i, firstColor);
    BackStrip.setPixelColor(i, secondColor);
  }
}

void torpedos(uint32_t firstColor, uint32_t secondColor, uint8_t wait){
  for(uint16_t i=0; i<FrontStrip.numPixels(); i++) {
    FrontStrip.setPixelColor(i, firstColor);
    BackStrip.setPixelColor(i, secondColor);
  }
}

void markerDropper(uint32_t color, uint8_t wait){
  for(uint16_t i=0; i<FrontStrip.numPixels(); i++) {
    FrontStrip.setPixelColor(i, color);
  }

  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < BackStrip.numPixels(); i=i+3) {
        BackStrip.setPixelColor(i+q, color);    //turn every third pixel on
      }

      BackStrip.show();

      delay(wait);

      for (uint16_t i=0; i < BackStrip.numPixels(); i=i+3) {
        BackStrip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}


ros::Subscriber<std_msgs::Int8> sub("led", &messageCb);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  FrontStrip.begin();
  BackStrip.begin();
  FrontStrip.show(); // Initialize all pixels to 'off'
  BackStrip.show();
}

void loop() {
  nh.spinOnce();
  delay(1);
}
