#include "Setup.h"
#include "Color.h"



char* sensors[] = {"A: ","R: ","M: ","L: "};

void setup() {
  pinSetup();
  freqScalingSetup();
  Serial.begin(9600);
}

void loop() {
  uint16_t* redPW = getRedPW();
  Serial.println("RedPW");
  uint16_t* greenPW = getGreenPW();
  Serial.println("GreenPW");
  uint16_t* bluePW = getBluePW();
  Serial.println("BluePW");

  for(int i = 0; i < 4; i++) {
    Serial.print(sensors[i]);
    Serial.println(redPW[i]);
    Serial.println(greenPW[i]);
    Serial.println(bluePW[i]);
  }
  delay(500);

}

