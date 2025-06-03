#ifndef SETUP_H
#define SETUP_H

//Color Sensor Pins
#define A_OUT 29
#define R_OUT 30
#define M_OUT 31
#define L_OUT 32
#define S3 36
#define S2 35
#define S1 34
#define S0 33

#define NUM_SENSORS 4

//User Buttons
#define PNP_BUTTON 22
#define CAL_BUTTON 21
#define OBS_BUTTON 20

//Ultrasonic Sensor Pins
#define ECHO_1 38
#define TRIGGER_1 37

#define ECHO_2 36
#define TRIGGER_2 35

//Drive Motor Pins
#define IN4_L 19
#define IN3_L 18
#define IN2_L 17
#define IN1_L 16
#define ENB_L 15
#define ENA_L 14

#define IN4_R 6
#define IN3_R 7
#define IN2_R 8
#define IN1_R 9
#define ENB_R 10
#define ENA_R 11

//Arm Servo Pins
#define DATA_GRIP 41
#define DATA_ARM 40

//LED Pins

#define GREEN_LED 4
#define RED_LED 3
//User Functions
//serial.print("silly Goose!!!!!")
void pinSetup();
void freqScalingSetup();

#endif