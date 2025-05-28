//////////////////////////////////////////////////////////////////////////////////////
// INTEGRATION LAB-03:  VehicleTestProgram                                          //
//                                                                                  //
// This program is an open-loop control program that will test the construction and //
// wiring of a 2WDD (2-Wheel Differential Drive) vehicle.  By running the program,  //
// the vehicle should move forward, rotate to the right, rotate to the left, and    //
// move backward.  The vehicle should begin and end at the same place.              //
//////////////////////////////////////////////////////////////////////////////////////

//L298 Motor Controller Pin Assignment
// Motor A pin connections
int enA = 9;
int in1 = 8;
int in2 = 7;

// Motor B pin connections
int enB = 3;
int in3 = 4;
int in4 = 5;

void setup() {
  // Set all motor controller pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Initial state - Turn off all the motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void loop() {

  // Set PWM value for 60% of max speed - For PWM values varies between 0 (no speed) and 255 (full speed)
  analogWrite(enA, 153);
  analogWrite(enB, 153);

  // Rotate left and right wheels in clockwise (CW) direction - Move forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  delay(2000); // move vehicle forward for 2 seconds

  // Set PWM value for 35% of max speed - For PWM values varies between 0 (no speed) and 255 (full speed)
  analogWrite(enA, 89);
  analogWrite(enB, 89);

  // Rotate left wheel clockwise (CW) and right wheel in anti-clockwise direction (CCW) - Spin clockwise/Rotate right
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  delay(1000); // rotate vehicle to the right for 1 second

  // Set PWM value for 35% of max speed - For PWM values varies between 0 (no speed) and 255 (full speed)
  analogWrite(enA, 89);
  analogWrite(enB, 89);

  // Rotate left wheel anti-clockwise (CCW) and right wheel in clockwise direction (CW) - Spin anti-clockwise/Rotate left
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  delay(1000); // rotate vehicle to the left for 1 second

  // Set PWM value for 60% of max speed - For PWM values varies between 0 (no speed) and 255 (full speed)
  analogWrite(enA, 153);
  analogWrite(enB, 153);

  // Rotate left and right wheels in anti-clockwise (CCW) direction - Move backward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  
  delay(2000); // move vehicle backward for 2 seconds
}