#include "Color.h"
#include "Setup.h"
#include <stdint.h>

#define TEST_BUTTON 12

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

char sensors[] = {'A','R','M','L'};
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



enum Mode {
  CAL_MIN,
  CAL_MAX,
  READ
}; 
enum Mode MODE = CAL_MIN;

void setup() {
  Serial.begin(9600);
  pinMode(12, INPUT);
}

void loop() {
  //Calibrate color sensors
  switch(MODE) {
    case CAL_MIN:
      Serial.println(MODE);
      if(digitalRead(TEST_BUTTON)) {
        MODE = CAL_MAX;
      }
      calibrateSensorsMin();
      for(int i = 0; i < 4; i++) {
        Serial.print(sensors[i]); Serial.print(minRedPW[i]); Serial.print(" "); Serial.print(minGreenPW[i]); Serial.print(" "); Serial.println(minBluePW[i]);
      }
      delay(500);
    break;
    case CAL_MAX:
      if(digitalRead(TEST_BUTTON)) {
        MODE = READ;
      }
      Serial.println(MODE);
      calibrateSensorsMax();
      for(int i = 0; i < 4; i++) {
        Serial.print(sensors[i]); Serial.print(maxRedPW[i]); Serial.print(" "); Serial.print(maxGreenPW[i]); Serial.print(" "); Serial.println(maxBluePW[i]);
      }
      delay(500);
    break;
    case READ:
      Serial.println(MODE);
      if(digitalRead(TEST_BUTTON)) {
        MODE = CAL_MIN;
      }
      readColorSensors();
      delay(500);
    break;
    default:
    break;
  }
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