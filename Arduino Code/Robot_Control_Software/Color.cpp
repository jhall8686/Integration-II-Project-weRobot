#include "Setup.h"
#include "Color.h"
#include <stdint.h>
#include <Arduino.h>
// Returns an array with red PW values for ARML
int* getRedPW() {
  static int PW[NUM_SENSORS];

  // Set color sensor filter to only detect Red color
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);

  // Read the output Pulse Widths
  PW[0] = pulseIn(A_OUT, LOW);
  PW[1] = pulseIn(R_OUT, LOW);
  PW[2] = pulseIn(M_OUT, LOW);
  PW[3] = pulseIn(L_OUT, LOW);

  return PW;
}

// Returns an array with green PW values for ARML
int* getGreenPW() {
  static int PW[NUM_SENSORS]; 

  // Set color sensor filter to only detect green color
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);

  // Read the output Pulse Widths
  PW[0] = pulseIn(A_OUT, LOW);
  PW[1] = pulseIn(R_OUT, LOW);
  PW[2] = pulseIn(M_OUT, LOW);
  PW[3] = pulseIn(L_OUT, LOW);

  return PW;
}

// Returns an array with blue PW values for ARML
int* getBluePW() {
  static int
   PW[NUM_SENSORS]; 

  // Set color sensor filter to only detect blue color
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);

  // Read the output Pulse Widths
  PW[0] = pulseIn(A_OUT, LOW);
  PW[1] = pulseIn(R_OUT, LOW);
  PW[2] = pulseIn(M_OUT, LOW);
  PW[3] = pulseIn(L_OUT, LOW);

  return PW;
}

