
#ifndef ___TOURMAX__KIWI__DRIVE__H___
#define ___TOURMAX__KIWI__DRIVE__H___

#import "Arduino.h"
#import "vector.h"
#import "movement_control.h"
#import "smoothed_values.h"

//red=330,80
//orange=445,190
//green=515,250

static const int WHEEL_POWER_FULL_REVERSE = 190 - (190 - 80) / 5; //80; //330;
static const int WHEEL_POWER_NO_MOVEMENT = 190; //445;
static const int WHEEL_POWER_FULL_FORWARD = 190 + (250 - 190) / 5; //250; //515;

static const int NUM_VALUES_FOR_WHEEL_SMOOTHED_VALUES = 21;

class KiwiDrive {

private:

  int northPin;
  int southWestPin;
  int southEastPin;
  
  SmoothedValues wheelNorthSmoothed;
  SmoothedValues wheelSouthWestSmoothed;
  SmoothedValues wheelSouthEastSmoothed;

  int convertValueInLinearRangeToMotorPower(int val, int rangeMin, int rangeMax) {
    int rangeMiddle = (rangeMin + rangeMax) / 2;
    if (val < rangeMiddle) {
      //reverse
      return map(val, rangeMin, rangeMiddle, WHEEL_POWER_FULL_REVERSE, WHEEL_POWER_NO_MOVEMENT);
    } 
    else if (val > rangeMiddle) {
      //forward
      return map(val, rangeMiddle, rangeMax, WHEEL_POWER_NO_MOVEMENT, WHEEL_POWER_FULL_FORWARD);
    } 
    else {
      //in the middle, no motion
      return WHEEL_POWER_NO_MOVEMENT;
    }
  }

  //http://stackoverflow.com/questions/3748037/how-to-control-a-kiwi-drive-robot

  //assumes the vector has a magnitude of 1 for full power
  void convertMotorVectorToWheelPower(MovementControl &movementControl,
                                      int &north, int &southWest, int &southEast) {
    //const int actualWheelValueMax = WHEEL_VALUE_MAX; // ((int) (WHEEL_VALUE_MAX / ((sqrt(3) + 1) / 2)));
    Vector xyVector = movementControl.xyVector;
    float rotation = movementControl.rotation;
    if (xyVector.getMagnitude() > 1.0f) {
      xyVector.normalize();
    }
    const int symmetricRangeForCalc = 256;
    int xForWheels, yForWheels;
    xyVector.mult(symmetricRangeForCalc);
    xyVector.getComponentsInt(xForWheels, yForWheels);
    north = xForWheels;
    int xComponentForSouth = -xForWheels / 2;
    int yComponentForSouth = -((int) (sqrt(3) / 2 * yForWheels));
    southWest = xComponentForSouth - yComponentForSouth;
    southEast = xComponentForSouth + yComponentForSouth;
    //for whatever reason, everything needs to be negated...
    north *= -1;
    southWest *= -1;
    southEast *= -1;
    //now, get them in the right range
    north = convertValueInLinearRangeToMotorPower(north, -symmetricRangeForCalc, symmetricRangeForCalc);
    southWest = convertValueInLinearRangeToMotorPower(southWest, -symmetricRangeForCalc, symmetricRangeForCalc);
    southEast = convertValueInLinearRangeToMotorPower(southEast, -symmetricRangeForCalc, symmetricRangeForCalc);
    //THEN, APPLY ROTATION...I GUESS???
    int rotationPower = ((int) (-10 * rotation));
    north += rotationPower;
    southWest += rotationPower;
    southEast += rotationPower;
  }

  void sendWheelPowers(int north, int southWest, int southEast) {
    wheelNorthSmoothed.addValue(north);
    wheelSouthWestSmoothed.addValue(southWest);
    wheelSouthEastSmoothed.addValue(southEast);
    int wheelNorth = wheelNorthSmoothed.getSmoothedValue();
    int wheelSouthWest = wheelSouthWestSmoothed.getSmoothedValue();
    int wheelSouthEast = wheelSouthEastSmoothed.getSmoothedValue();
    analogWrite(northPin, wheelNorth);
    analogWrite(southWestPin, wheelSouthWest);
    analogWrite(southEastPin, wheelSouthEast);
  }

public:

  KiwiDrive(int northPinIn, int southWestPinIn, int southEastPinIn) {
    northPin = northPinIn;
    southWestPin = southWestPinIn;
    southEastPin = southEastPinIn;
    wheelNorthSmoothed = SmoothedValues(NUM_VALUES_FOR_WHEEL_SMOOTHED_VALUES, WHEEL_POWER_NO_MOVEMENT);
    wheelSouthWestSmoothed = SmoothedValues(NUM_VALUES_FOR_WHEEL_SMOOTHED_VALUES, WHEEL_POWER_NO_MOVEMENT);
    wheelSouthEastSmoothed = SmoothedValues(NUM_VALUES_FOR_WHEEL_SMOOTHED_VALUES, WHEEL_POWER_NO_MOVEMENT);
  }

  void setup() {
    pinMode(northPin, OUTPUT);
    pinMode(southWestPin, OUTPUT);
    pinMode(southEastPin, OUTPUT);
  }

  void applyMovementControl(MovementControl &movementControl) {
    int north, southWest, southEast;
    convertMotorVectorToWheelPower(movementControl, north, southWest, southEast);
    sendWheelPowers(north, southWest, southEast);
  }

};

#endif /* ___TOURMAX__KIWI__DRIVE__H___ */


