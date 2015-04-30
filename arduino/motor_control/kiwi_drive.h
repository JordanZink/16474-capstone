
#ifndef ___TOURMAX__KIWI__DRIVE__H___
#define ___TOURMAX__KIWI__DRIVE__H___

#import "Arduino.h"
#import "vector.h"
#import "movement_control.h"
#import "smoothed_values.h"
#import "assert.h"

// BEGIN HACK

static int bigGlobalKiwiCounter = 0;
static const int KIWI_PRINT_PERIOD = 60;

// END HACK

//red=330,80
//orange=445,190
//green=515,250

static const int WHEEL_POWER_ABSOLUTE_FULL_REVERSE = 80; //80; //330;
static const int WHEEL_POWER_ABSOLUTE_NO_MOVEMENT = 190; //445;
static const int WHEEL_POWER_ABSOLUTE_FULL_FORWARD = 250; //250; //515;
static const float PERCENT_FULL_POWER = 0.4f;
static const float PERCENT_ROTATION_RELATIVE_TO_LINEAR_POWER = 0.55f;
static const float MOTOR_BIAS_CORRECTION = 0.70f;

static inline int applyPercentPower(int fullPower, int noPower, float percent, bool applyWheelBias) {
  float diff = (float) (fullPower - noPower);
  if (applyWheelBias == true) {
    diff *= MOTOR_BIAS_CORRECTION;
  }
  int diffLimited = (int) (diff * percent);
  return noPower + diffLimited;
}

static const int WHEEL_POWER_FULL_REVERSE = applyPercentPower(WHEEL_POWER_ABSOLUTE_FULL_REVERSE, WHEEL_POWER_ABSOLUTE_NO_MOVEMENT, PERCENT_FULL_POWER, true);
static const int WHEEL_POWER_NO_MOVEMENT = WHEEL_POWER_ABSOLUTE_NO_MOVEMENT;
static const int WHEEL_POWER_FULL_FORWARD = applyPercentPower(WHEEL_POWER_ABSOLUTE_FULL_FORWARD, WHEEL_POWER_ABSOLUTE_NO_MOVEMENT, PERCENT_FULL_POWER, false);

class KiwiDrive {

private:

  int northPin;
  int southWestPin;
  int southEastPin;
  
  SmoothedValues wheelNorthSmoothed;
  SmoothedValues wheelSouthWestSmoothed;
  SmoothedValues wheelSouthEastSmoothed;
  
  int timeToStartWheels;
  bool shouldPowerWheels;

  static int convertValueInLinearRangeToMotorPower(int val, int rangeMin, int rangeMax) {
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
  void convertMotorVectorToWheelPower(const MovementControl* movementControl,
                                      int &north, int &southWest, int &southEast) {
    //const int actualWheelValueMax = WHEEL_VALUE_MAX; // ((int) (WHEEL_VALUE_MAX / ((sqrt(3) + 1) / 2)));
    Vector xyVector = movementControl->xyVector;
    float rotation = movementControl->rotation;
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
    //apply rotation
    int maxRotationPower = (int) (symmetricRangeForCalc * PERCENT_ROTATION_RELATIVE_TO_LINEAR_POWER);
    const int symmetricRangeWithRotation = symmetricRangeForCalc + maxRotationPower;
    int rotationPower = -((int) (rotation * maxRotationPower));
    north += rotationPower;
    southWest += rotationPower;
    southEast += rotationPower;
    //now, get them in the right range
    north = convertValueInLinearRangeToMotorPower(north, -symmetricRangeWithRotation, symmetricRangeWithRotation);
    southWest = convertValueInLinearRangeToMotorPower(southWest, -symmetricRangeWithRotation, symmetricRangeWithRotation);
    southEast = convertValueInLinearRangeToMotorPower(southEast, -symmetricRangeWithRotation, symmetricRangeWithRotation);
    /*
    //THEN, APPLY ROTATION...I GUESS???
    int maxRot = -5;
    if (rotation > 0) {maxRot = -8;}
    int rotationPower = ((int) (maxRot * rotation));
    */
  }

  void sendWheelPowers(int north, int southWest, int southEast) {
    //north = convertValueInLinearRangeToMotorPower(25, -256, 256);
    //southWest = convertValueInLinearRangeToMotorPower(25, -256, 256);
    //southEast = convertValueInLinearRangeToMotorPower(25, -256, 256);
    wheelNorthSmoothed.addValue(north);
    wheelSouthWestSmoothed.addValue(southWest);
    wheelSouthEastSmoothed.addValue(southEast);
    int wheelNorth = wheelNorthSmoothed.getSmoothedValue();
    int wheelSouthWest = wheelSouthWestSmoothed.getSmoothedValue();
    int wheelSouthEast = wheelSouthEastSmoothed.getSmoothedValue();
    if (shouldPowerWheels) {
      /**/
      analogWrite(northPin, wheelNorth);
      analogWrite(southWestPin, wheelSouthWest);
      analogWrite(southEastPin, wheelSouthEast);
      /**/
      /*
      bigGlobalKiwiCounter = (bigGlobalKiwiCounter + 1) % KIWI_PRINT_PERIOD;
      if (bigGlobalKiwiCounter == 0) {
        char buf[32];
        int numChars = sprintf(buf, "m %d %d %d\n\0", wheelNorth, wheelSouthWest, wheelSouthEast);
        assert(numChars <= 32);
        Serial.print(buf);
      }
      */
    }
  }

public:

  KiwiDrive(int northPinIn, int southWestPinIn, int southEastPinIn) {
    northPin = northPinIn;
    southWestPin = southWestPinIn;
    southEastPin = southEastPinIn;
    wheelNorthSmoothed = SmoothedValues(WHEEL_POWER_NO_MOVEMENT);
    wheelSouthWestSmoothed = SmoothedValues(WHEEL_POWER_NO_MOVEMENT);
    wheelSouthEastSmoothed = SmoothedValues(WHEEL_POWER_NO_MOVEMENT);
    timeToStartWheels = millis() + 1000;
    shouldPowerWheels = false;
  }

  void setupThing() {
    pinMode(northPin, OUTPUT);
    pinMode(southWestPin, OUTPUT);
    pinMode(southEastPin, OUTPUT);
  }

  void applyMovementControl(const MovementControl* movementControl) {
    if (shouldPowerWheels == false && millis() > timeToStartWheels) {
      shouldPowerWheels = true;
    }
    int north, southWest, southEast;
    convertMotorVectorToWheelPower(movementControl, north, southWest, southEast);
    sendWheelPowers(north, southWest, southEast);
  }

};

#endif /* ___TOURMAX__KIWI__DRIVE__H___ */


