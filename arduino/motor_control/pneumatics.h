
#ifndef ___TOURMAX__PNEUMATICS__H___
#define ___TOURMAX__PNEUMATICS__H___

class Pneumatics {
  
private:

  int sensorPin;
  int valvePin;

public:

  Pneumatics(int sensorPinIn, int valvePinIn) {
    sensorPin = sensorPinIn;
    valvePin = valvePinIn;
  }
  
  void setupThing() {
    //nothing right now
  }
  
  void onTick() {
    //nothing right now
  }
  
};

#endif /* ___TOURMAX__PNEUMATICS__H___ */

