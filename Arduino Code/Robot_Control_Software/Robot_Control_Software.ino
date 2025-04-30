#include <stdint.h>

#include "Setup.h"
#include "Color.h"
#include "Motor.h"

////
//Color Sensor Global Variables
////

//Minimum pulse width values read during calibration (mapped to 255)
uint16_t minRedPW[] =    {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
uint16_t minGreenPW[] =  {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
uint16_t minBluePW[] =   {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};

//Maximum pulse width values read during calibration (mapped to 0)
uint16_t maxRedPW[] =    {0, 0, 0, 0};
uint16_t maxGreenPW[] =  {0, 0, 0, 0};
uint16_t maxBluePW[] =   {0, 0, 0, 0};

//Stores current pulse width read on each color sensor for red, green, and blue
uint16_t redFreq[NUM_SENSORS];
uint16_t greenFreq[NUM_SENSORS];
uint16_t blueFreq[NUM_SENSORS];

//Data type for color readings
struct RGB {
  uint16_t r;
  uint16_t g;
  uint16_t b;
};

//Stores current read color values for each sensor
struct RGB color[NUM_SENSORS];

//    A     R     M     L

//  [ r     r     r     r ]
//  [ g     g     g     g ]
//  [ b  ,  b  ,  b  ,  b ]

//State variables
enum STATE {
  SEARCH,
  FOLLOW,
  CALIBRATE_MIN,
  CALIBRATE_MAX,
  OBSTACLE_AVOID
};

//Initialization of Global Motor Pointers
Motor* R1 = nullptr;
Motor* R2 = nullptr;
Motor* L1 = nullptr;
Motor* L2 = nullptr;

/////////////////////////////////
//          main()
/////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  pinSetup();
  freqScalingSetup();
  //Initializing motors at global pointer locations
  R1 = new Motor(ENA_R, IN1_R, IN2_R);
  R2 = new Motor(ENB_R, IN3_R, IN4_R);
  L1 = new Motor(ENA_L, IN1_L, IN2_L);
  L2 = new Motor(ENB_L, IN3_L, IN4_L);
}

void loop() {
  // put your main code here, to run repeatedly:

}

//////////////////////////////////
/////COLOR SENSOR FUNCTIONS///////
//////////////////////////////////

////
//Reads the frequencies from each color sensor then sets each color struct accordingly
////
void readColorSensors() {
  uint16_t* redFreq = getRedPW();
  uint16_t* greenFreq = getGreenPW();
  uint16_t* blueFreq = getBluePW();

  for(uint8_t i = 0; i < NUM_SENSORS; i++) {
    color[i].r = map(redFreq[i], minRedPW[i], maxRedPW[i], 255, 0);
    color[i].g = map(greenFreq[i], minGreenPW[i], maxGreenPW[i], 255, 0);
    color[i].b = map(blueFreq[i], minBluePW[i], maxBluePW[i], 255, 0);
  }
}
////
//Only reads sensor once-- must be run multiple times to get better calibration.
////          ***Should run when looking at white***

void calibrateSensorsMin() {
  uint16_t* readRedPW = getRedPW();
  uint16_t* readGreenPW = getGreenPW();
  uint16_t* readBluePW = getBluePW();
  
  //check for each color sensor whether the new reading is older
  for(uint8_t i = 0; i < NUM_SENSORS; i++) 
  {
    if(readRedPW[i] < minRedPW[i]) {
      minRedPW[i] = readRedPW[i];
    }
    if(readGreenPW[i] < minGreenPW[i]) {
      minGreenPW[i] = readGreenPW[i];
    }
    if(readBluePW[i] < minBluePW[i]) {
      minBluePW[i] = readBluePW[i];
    }
  }

  //DELAY??
}
////
//Only reads sensor once-- must be run multiple times to get better calibration.
////          ***Should run while looking at black***

void calibrateSensorsMax() {
  uint16_t* readRedPW = getRedPW();
  uint16_t* readGreenPW = getGreenPW();
  uint16_t* readBluePW = getBluePW();
  
  //check for each color sensor whether the new reading is older
  for(uint8_t i = 0; i < NUM_SENSORS; i++) 
  {
    if(readRedPW[i] > maxRedPW[i]) {
      maxRedPW[i] = readRedPW[i];
    }
    if(readGreenPW[i] > maxGreenPW[i]) {
      maxGreenPW[i] = readGreenPW[i];
    }
    if(readBluePW[i] > maxBluePW[i]) {
      maxBluePW[i] = readBluePW[i];
    }
  }  

  //DELAY??
}
///////////////////////////
//////MOTOR FUNCTIONS//////
///////////////////////////

void moveForward(uint8_t pwm) {
  R1->rotateCW(pwm);
  R2->rotateCW(pwm);
  L1->rotateCCW(pwm);
  L2->rotateCCW(pwm);
}

void moveBackward(uint8_t pwm) {
  R1->rotateCCW(pwm);
  R2->rotateCCW(pwm);
  L1->rotateCW(pwm);
  L2->rotateCW(pwm);
}

void Stop() {
  R1->stop();
  R2->stop();
  L1->stop();
  L2->stop();
}
