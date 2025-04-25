#ifndef MACROS_H
#define MACROS_H

//Color Sensor Pins
#define A_OUT 29
#define R_OUT 30
#define M_OUT 31
#define L_OUT 32
#define S3 36
#define S2 35
#define S1 34
#define S0 33

#define ARM 0
#define RIGHT 1
#define MIDDLE 2
#define LEFT 3

#define NUM_SENSORS 4

//Ultrasonic Sensor Pins
#define ECHO 38
#define TRIG 37

//Drive Motor Pins
#define IN4_L 21
#define IN3_L 20
#define IN2_L 19
#define IN1_L 18
#define ENB_L 17
#define ENA_L 16

#define IN4_R 4
#define IN3_R 5
#define IN2_R 6
#define IN1_R 7
#define ENB_R 8
#define ENA_R 9

//Arm Servo Pins
#define DATA_GRIP 15
#define DATA_ARM 14

//User Functions

void pinSetup();
void freqScalingSetup();

#endif