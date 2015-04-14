
#ifndef ___TOURMAX__JOYSTICK__H___
#define ___TOURMAX__JOYSTICK__H___

#import "Arduino.h"
#import "assert.h"
#import "vector.h"
#import "movement_control.h"

static const int JOYSTICK_VALUE_MIN = 0;
static const int JOYSTICK_VALUE_MAX = 1023;

//100 implies the entire thing is dead
static const int JOYSTICK_DEAD_ZONE_PERCENT = 10;

enum JoystickInputType {
  JOYSTICK_INPUT_NO_ROTATION,
  JOYSTICK_INPUT_X_FOR_ROTATION
};

class Joystick {
  
private:

  int xPin;
  int yPin;
  
  SmoothedValues joystickXSmoothed;
  SmoothedValues joystickYSmoothed;
  
  JoystickInputType inputType;
  
  static long applyDeadZone(long x, int lo, int hi, long deadValue) {
    if (lo < x && x < hi) {
      return deadValue;
    } else {
      return x;
    }
  }

public:

  Joystick(int xPinIn, int yPinIn, JoystickInputType inputTypeIn) {
    xPin = xPinIn;
    yPin = yPinIn;
    joystickXSmoothed = SmoothedValues(0);
    joystickYSmoothed = SmoothedValues(0);
    inputType = inputTypeIn;
  }
  
  void setupThing() {
    //nothing needed to set up for now
  }
  
  void changeInputType(JoystickInputType newInputType) {
    inputType = newInputType;
  }
  
  void readToMovementControl(MovementControl* movementControl) {
    int rawX = analogRead(xPin);
    int rawY = analogRead(yPin);
    joystickXSmoothed.addValue(rawX);
    joystickYSmoothed.addValue(rawY);
    long x = joystickXSmoothed.getSmoothedValue();
    long y = joystickYSmoothed.getSmoothedValue();
    const int symmetricRangeForDeadzone = 256;
    x = map(x, JOYSTICK_VALUE_MIN, JOYSTICK_VALUE_MAX, -symmetricRangeForDeadzone, symmetricRangeForDeadzone);
    y = map(y, JOYSTICK_VALUE_MIN, JOYSTICK_VALUE_MAX, -symmetricRangeForDeadzone, symmetricRangeForDeadzone);
    x = (pow(x, 2)  / symmetricRangeForDeadzone) * (x < 0 ? -1 : 1);
    y = (pow(y, 2) / symmetricRangeForDeadzone) * (y < 0 ? -1 : 1);
    //flip x (cuz why not)
    x *= -1;
    //flip y (cuz why not)
    y *= -1;
    //apply deadzone
    const int deadZone = symmetricRangeForDeadzone * JOYSTICK_DEAD_ZONE_PERCENT / 100;
    x = applyDeadZone(x, -deadZone, deadZone, 0);
    y = applyDeadZone(y, -deadZone, deadZone, 0);
    //scale if magnitude is too big
    long dist = sqrt(x * x + y * y);
    long maxMagnitude = symmetricRangeForDeadzone;
    if (dist > maxMagnitude) {
      x = x * maxMagnitude / dist;
      y = y * maxMagnitude / dist;
    }
    switch (inputType) {
      case JOYSTICK_INPUT_NO_ROTATION:
        movementControl->xyVector = Vector(x, y, -symmetricRangeForDeadzone, symmetricRangeForDeadzone);
        movementControl->rotation = 0.0f;
        break;
      case JOYSTICK_INPUT_X_FOR_ROTATION:
        movementControl->xyVector = Vector(0, y, -symmetricRangeForDeadzone, symmetricRangeForDeadzone);
        movementControl->rotation = ((float) x) / symmetricRangeForDeadzone;
        break;
      default:
        assert (false);
    }
  }
  
};

#endif /* ___TOURMAX__JOYSTICK__H___ */

