#include <Servo.h>


#include "Setup.h"
Servo Gripper;
Servo Arm;




uint16_t idleGripperPos = 180;
uint16_t idleArmPos = 180;
uint16_t gripperTolerance = 5;
uint16_t armTolerance = 5;

uint16_t lastGripperCommand = idleGripperPos;
uint16_t lastArmCommand = idleArmPos;


void setup() {
  pinSetup();
  Gripper.attach(DATA_GRIP);
  Arm.attach(DATA_ARM);
}

void loop() {
  int currentArmPos = Arm.read();
  int currentGripperPos = Gripper.read();

  if (abs(currentArmPos - idleArmPos) > armTolerance) {
    if (lastArmCommand != idleArmPos) {
      Arm.write(idleArmPos);
      lastArmCommand = idleArmPos;
    }
  }

  if (abs(currentGripperPos - idleGripperPos) > gripperTolerance) {
    if (lastGripperCommand != idleGripperPos) {
      Gripper.write(idleGripperPos);
      lastGripperCommand = idleGripperPos;
    }
  }
}