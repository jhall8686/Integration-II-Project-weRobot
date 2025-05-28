#include "Setup.h"

#include <Arduino.h>
#include <stdint.h>

void pinSetup() 
{
    //Color Sensor Pin Setup
    pinMode(A_OUT, INPUT);
    pinMode(R_OUT, INPUT);
    pinMode(M_OUT, INPUT);
    pinMode(L_OUT, INPUT);
  
    pinMode(S3, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S0, OUTPUT);
  
    //Ultrasonic Sensor Pin Setup
    pinMode(ECHO, INPUT);
    pinMode(TRIGGER, OUTPUT);
    
    //Drive Motor Pin Setup
    pinMode(IN4_L, OUTPUT);
    pinMode(IN3_L, OUTPUT);
    pinMode(IN2_L, OUTPUT);
    pinMode(IN1_L, OUTPUT);
    pinMode(ENA_L, OUTPUT);
    pinMode(ENB_L, OUTPUT);
  
    pinMode(IN4_R, OUTPUT);
    pinMode(IN3_R, OUTPUT);
    pinMode(IN2_R, OUTPUT);
    pinMode(IN1_R, OUTPUT);
    pinMode(ENA_R, OUTPUT);
    pinMode(ENB_R, OUTPUT);
  
    //Arm Servo Pin Setup
    pinMode(DATA_GRIP, OUTPUT);
    pinMode(DATA_ARM, OUTPUT);

    //Button Pin Setup
    pinMode(CAL_BUTTON, INPUT_PULLUP);
    pinMode(PNP_BUTTON, INPUT_PULLUP);
    pinMode(OBS_BUTTON, INPUT_PULLUP);
}

void freqScalingSetup() {
  //Set frequency Scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}
