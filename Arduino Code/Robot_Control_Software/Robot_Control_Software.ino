#include <Servo.h>
#include <stdint.h>

#include "Setup.h"
#include "Color.h"
#include "Motor.h"

////
//Color Sensor Global Variables
////

//Minimum pulse width values read during calibration (mapped to 255)
uint16_t minRedPW[] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
uint16_t minGreenPW[] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
uint16_t minBluePW[] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};

//Maximum pulse width values read during calibration (mapped to 0)
uint16_t maxRedPW[] = {0, 0, 0, 0};
uint16_t maxGreenPW[] = {0, 0, 0, 0};
uint16_t maxBluePW[] = {0, 0, 0, 0};

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

//Will store previously read color values up to a length of 10


//    A     R     M     L

//  [ r     r     r     r ]
//  [ g     g     g     g ]
//  [ b  ,  b  ,  b  ,  b ]


//State variables
enum STATE {
  IDLE,
  CALIBRATE_MIN,
  CALIBRATE_MAX,
  FIND_PEDESTAL,
  PICKUP,
  SEARCH,
  FOLLOW,
  PLACE,
  OBSTACLE_AVOID
};

enum COLOR {
  BLACK,
  WHITE,
  RED,
  GREEN,
  BLUE,
  OTHER
};

enum DIRECTION {
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT
};

DIRECTION Facing = BACKWARD;

COLOR armReadings[] = {OTHER};
COLOR leftReadings[] = {OTHER};
COLOR midReadings[] = {OTHER};
COLOR rightReadings[] = {OTHER};

COLOR mode = OTHER;
STATE Robot_State = IDLE;

//Interrupt flags

uint8_t gCalFlag = 0;
uint8_t gPnpFlag = 0;
uint8_t gObsFlag = 0;
//Debounce delay time
uint8_t debounce = 50;
int size = 0;

//Ultrasonic Sensor Logic Flags and Variables

uint8_t gObstacleFoundFlag = 0;
float minDistance = 0.6; //cm
float boxDistance = 0.01; //cm

//Initialization of Global Motor Pointers and Servos
Motor* R1 = nullptr;
Motor* R2 = nullptr;
Motor* L1 = nullptr;
Motor* L2 = nullptr;

//Servo Init and Global Variables
Servo Gripper;
Servo Arm;


uint16_t gripperTolerance = 5;
uint16_t armTolerance = 10;
uint16_t currentArmPos;
uint16_t currentGripperPos;

uint16_t lastGripperCommand = 0;

uint16_t gripperClosed = 180;
uint16_t gripperOpen = 250;

uint16_t gripperLargeBox = 210;
uint16_t gripperSmallBox = 190; // ##TODO figure out these values.

uint16_t armUp = 100;
uint16_t armFront = 200; //Technically 200 but worried about wires
uint16_t armBack = 0;

uint16_t armStep = 10;
uint16_t timeStep = 40;
//Servo Flags
uint8_t levelWithBox = 0;
uint8_t largeBoxPos = 0;
uint8_t smallBoxPos = 0;
uint8_t boxGrabbed = 0;
uint8_t largeBox = 0;
uint8_t smallBox = 0;
uint8_t servoError = 0;

uint16_t idleGripperPos = gripperOpen;
uint16_t idleArmPos = armUp;

//PWM speeds to pass to motor functions
uint8_t slow = 63;
uint8_t med = 127;
uint8_t fast = 191;
uint8_t fastest = 255;

int count = 0;
float distance;
/////////////////////////////////
//          main()             //
/////////////////////////////////
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinSetup();
  freqScalingSetup();

  //Initializing motors at global pointer locations and initializing Servo objects at data pins

  R1 = new Motor(ENB_R, IN3_R, IN4_R);
  R2 = new Motor(ENA_R, IN1_R, IN2_R);
  L1 = new Motor(ENA_L, IN1_L, IN2_L);
  L2 = new Motor(ENB_L, IN3_L, IN4_L);
  
  Gripper.attach(DATA_GRIP);
  Arm.attach(DATA_ARM);

  //Interrupt Routines
  //Each trigger on falling edge of button being pressed (all 3 are set up with a pullup resistor, see pinSetup())
  attachInterrupt(digitalPinToInterrupt(CAL_BUTTON), calibrateButtonPressed,FALLING);
  attachInterrupt(digitalPinToInterrupt(PNP_BUTTON), pnpButtonPressed, FALLING);
  attachInterrupt(digitalPinToInterrupt(OBS_BUTTON), obsButtonPressed, FALLING);
}

void loop() {
  switch(Robot_State) 
  {
    case IDLE:
      Serial.println("Idle");

      //
      //Arm/Gripper Idle Loop
      // 
      currentArmPos = Arm.read();
      currentGripperPos = Gripper.read();


      if(abs(currentArmPos - toServoPos(idleArmPos)) > armTolerance) 
      {
        Arm.write(toServoPos(idleArmPos));
      }
      if(abs(currentGripperPos - toServoPos(idleGripperPos)) > gripperTolerance) 
      {
        Gripper.write(toServoPos(idleGripperPos));
      }

      //
      //Color Sensor Reading
      //
      readColorSensors();
      size = sizeof(midReadings)/sizeof(midReadings[0]);
      mode = calculateMode(midReadings, size);
      
      for(int i = 0; i < NUM_SENSORS; i++) {
        Serial.println(enumToString(returnColor(color[i])));
      }



      //
      //Ultrasonic Sensor reading
      //

      distance = ultraPing(1);
      Serial.println(distance);
      distance = ultraPing(2);
      Serial.println(distance);


      //
      //OUT FLAGS
      //
      if(gCalFlag) {
        delay(debounce);
        Robot_State = CALIBRATE_MIN;
        gCalFlag = 0;
      }
      if(gPnpFlag) {
        delay(debounce);
        Robot_State = FIND_PEDESTAL;
        gPnpFlag = 0;
      }
      if(gObsFlag) {
        delay(debounce);
        Robot_State = OBSTACLE_AVOID;
        gObsFlag = 0;
      }
      break;
    case CALIBRATE_MIN:
      for(int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(minRedPW[i]); Serial.println();
        Serial.print(minGreenPW[i]); Serial.println();
        Serial.print(minBluePW[i]); Serial.println();
      }

      calibrateSensorsMin();

      //OUT FLAGS
      if(gCalFlag) {
        delay(debounce);
        Robot_State = CALIBRATE_MAX;
        gCalFlag = 0;
        count = 0;
      }    
      break;
    case CALIBRATE_MAX:
      for(int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(maxRedPW[i]); Serial.println();
        Serial.print(maxGreenPW[i]); Serial.println();
        Serial.print(maxBluePW[i]); Serial.println();
      }

      calibrateSensorsMax();

      //OUT FLAGS
      if(gCalFlag) {
        delay(debounce);
        Robot_State = IDLE;
        gCalFlag = 0;
        count = 0;
      } 
      break;
    case FIND_PEDESTAL:
      Serial.println("FIND_PEDESTAL");
      if(!gObstacleFoundFlag) 
      {
        moveForward(med);
        distance = ultraPing(1);
        Serial.print(distance);
        if(distance <= minDistance) 
        {
          Stop();
          gObstacleFoundFlag = 1;
          Serial.print("Obstacle Found");
        }
      } 
      else 
      {
        Robot_State = PICKUP;
        gObstacleFoundFlag = 0;
      }
      break;
    case PICKUP:
      Serial.println("PICKUP");
      if(servoError) {
        digitalWrite(RED_LED, HIGH);
      }
      else
      {
        if(!levelWithBox) 
        {
          //Arm -> Front
          for(uint16_t pos = armUp; pos < armFront; pos+=armStep) 
          {
            Serial.println("step completed");
            Arm.write(toServoPos(pos));
            delay(timeStep);
          }
          //wait for motors to get there
          delay(200);

          //#DEBUG
          Serial.println(Arm.read());
          Serial.println(Arm.read()-toServoPos(armFront));
          Serial.println(armTolerance);
          //#DEBUG

          //Check if arm is level with the box
          if(abs(Arm.read() - toServoPos(armFront)) < armTolerance)
          {
            levelWithBox = 1;
          } else {
            servoError = 1;
          }
          Serial.println(levelWithBox);
        } else if(levelWithBox && !boxGrabbed)
        {
          if(!smallBox) {
            Gripper.write(toServoPos(gripperLargeBox));
          } else {
            Gripper.write(toServoPos(gripperSmallBox));
          }

          //wait for motor to get there
          delay(200);

          //Arm -> Up
          for(uint16_t pos = armFront; pos > armUp; pos-=armStep) 
          {
            Serial.println("step completed");
            Arm.write(toServoPos(pos));
            delay(timeStep);
          }
          //wait for motor to get there
          delay(200);
          
          if(ultraPing(2) < boxDistance) 
          {
            if(smallBox) {
              largeBox = 0;
              boxGrabbed = 1;
            }
            else {
              largeBox = 1;
              boxGrabbed = 1;
            }

          } else {
            
            levelWithBox = 0;
            smallBox = 1;
          }

        }
      }
      if(boxGrabbed) 
      {
        Robot_State = FOLLOW;
      }
      break;
    case FOLLOW:
      readColorSensors();
      if(Facing == BACKWARD) {
        turnCCW(med);
        delay(5000);
        Stop();
        Facing = FORWARD;
      }
      Serial.println(enumToString(returnColor(color[0])));
      
      break;
    case PLACE:
      break;
    case OBSTACLE_AVOID:
      Serial.println("OBSTACLE_AVOID");
      delay(200);
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

COLOR returnColor(struct RGB c) {
  if(c.r < 50 && c.g < 50 && c.b < 50) {
    return BLACK;
  }
  else if(c.r>200 && c.g>200 && c.b>200) {
    return WHITE;
  }
  else if(c.r > c.g && c.r > c.b) {
    return RED;
  }
  else if(c.g > c.r && c.g > c.b) {
    return GREEN;
  }
  else if(c.b > c.r && c.b > c.g) {
    return BLUE;
  }
  else {
    return OTHER;
  }
}

COLOR calculateMode(const COLOR* colors, int size) {
  int counts[6] = {0};  // One slot per COLOR

  // Count each color's frequency
  for (int i = 0; i < size; i++) {
    if (colors[i] >= BLACK && colors[i] <= OTHER) {
      counts[colors[i]]++;
    }
  }

  // Find the color with the highest count
  int maxCount = 0;
  COLOR mode = OTHER;

  for (int i = 0; i < 6; i++) {
    if (counts[i] > maxCount) {
      maxCount = counts[i];
      mode = (COLOR)i;
    }
  }
  return mode;
}

char* enumToString(COLOR color) 
{
  if(color == BLACK) return "BLACK";
  if(color == WHITE) return "WHITE";
  if(color == RED) return "RED";
  if(color == GREEN) return "GREEN";
  if(color == BLUE) return "BLUE";
  if(color == OTHER) return "OTHER";
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



/////////////////////////////////
//////MISC SENSOR FUNCTIONS//////
/////////////////////////////////

//WARNING: USES DELAY (MAY INTERFERE WITH INTERRUPTS)
float ultraPing(int sensor) 
{
  if(sensor == 1)
  {
    digitalWrite(TRIGGER_1, HIGH);
    delay(10);
    digitalWrite(TRIGGER_1, LOW);

    float duration = pulseIn(ECHO_1, HIGH);

    float distance = (duration/2) * 343e-5; //calculates distance in cm

    return distance;
  }
  if(sensor == 2)
  {
    digitalWrite(TRIGGER_2, HIGH);
    delay(10);
    digitalWrite(TRIGGER_2, LOW);

    float duration = pulseIn(ECHO_2, HIGH);

    float distance = (duration/2) * 343e-7; //calculates distance in cm

    return distance;
  }

}
//////MOTOR FUNCTIONS//////
///////////////////////////

//pwm = 0-255
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
void moveLeft(uint8_t pwm) {
  R1->rotateCW(pwm);
  L1->rotateCW(pwm);
  R2->rotateCCW(pwm);
  L2->rotateCCW(pwm);
}
void moveRight(uint8_t pwm) {
  R1->rotateCCW(pwm);  
  L1->rotateCCW(pwm);
  R2->rotateCW(pwm);
  L2->rotateCW(pwm);
}
void moveFR(uint8_t pwm) {
  L1->rotateCCW(pwm);
  R2->rotateCW(pwm);
  R1->stop();
  L2->stop();
}
void moveFL(uint8_t pwm) {
  R1->rotateCW(pwm);
  L2->rotateCCW(pwm);
  R2->stop();
  L1->stop();
}
void moveBL(uint8_t pwm) {
  L1->rotateCW(pwm);
  R2->rotateCCW(pwm);
  R1->stop();
  L2->stop();
}
void moveBR(uint8_t pwm) {
  R1->rotateCCW(pwm);
  L2->rotateCW(pwm);
  R2->stop();
  L1->stop();
}
void turnCW(uint8_t pwm) {
  R1->rotateCCW(pwm);
  L1->rotateCCW(pwm);
  R2->rotateCCW(pwm);
  L2->rotateCCW(pwm);
}
void turnCCW(uint8_t pwm) {
  R1->rotateCW(pwm);
  L1->rotateCW(pwm);
  R2->rotateCW(pwm);
  L2->rotateCW(pwm);
}
void Stop() {
  R1->stop();
  R2->stop();
  L1->stop();
  L2->stop();
}


//Servo Functions

//param pos-- range 0-300 (actual position of the motor)
uint16_t toServoPos(uint16_t pos) {
  return map(pos, 0, 300, 0, 180);
}
//Predefined Movesets (rely on above functions)
void squareDanceCW(uint8_t pwm) {
  moveForward(pwm);
  delay(2000);
  Stop();
  delay(1000);
  moveRight(pwm);
  delay(2000);
  Stop();
  delay(1000);
  moveBackward(pwm);
  delay(2000);
  Stop();
  delay(1000);
  moveLeft(pwm);
  delay(2000);
  Stop();
  delay(1000);
}

void diamondDanceCCW(uint8_t pwm) {
  moveFL(pwm);
  delay(2000);
  Stop();
  delay(1000);
  moveBL(pwm);
  delay(2000);
  Stop();
  delay(1000);  
  moveBR(pwm);
  delay(2000);
  Stop();
  delay(1000);
  moveFR(pwm);
  delay(2000);
  Stop();
  delay(1000);
}


///////////////////////////////////
////////INTERRUPT CALLBACKS////////
///////////////////////////////////

void calibrateButtonPressed() {
  gCalFlag = 1;
}
void pnpButtonPressed() {
  gPnpFlag = 1;
}
void obsButtonPressed() {
  gObsFlag = 1;
}
