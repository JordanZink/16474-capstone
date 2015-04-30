
#ifndef ___TOURMAX__PNEUMATICS__H___
#define ___TOURMAX__PNEUMATICS__H___

#include <Wire.h>
#import "LPS331.h"

class Pneumatics {

private:

  int valvePin;
  float targetMillibars;
  unsigned long nextToggle;
  bool curState;
  
  LPS331 pressureSensor;
  
  static inline float convertPsiToMillibars(float psi) {
    //http://www.srh.noaa.gov/images/epz/wxcalc/pressureConversion.pdf
    return psi * 68.9476f;
  }

public:

  Pneumatics(int valvePinIn, float targetPsi) {
    valvePin = valvePinIn;
    targetMillibars = convertPsiToMillibars(targetPsi);
    nextToggle = 0;
    curState = false;
  }
  
  void setupThing() {
    pinMode(valvePin, OUTPUT);
    Wire.begin();
    pressureSensor.init();
    pressureSensor.enableDefault();
  }
  
  void onTick() {
    if (millis() > nextToggle) {
      curState = !curState;
      //digitalWrite(valvePin, (curState ? HIGH : LOW));
      nextToggle = millis() + 1000;
    }
  }
  
};

#endif /* ___TOURMAX__PNEUMATICS__H___ */

