
#ifndef ___TOURMAX__JOYSTICK__INPUT__H___
#define ___TOURMAX__JOYSTICK__INPUT__H___

#import "Arduino.h"
#import "buffer.h"

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class JoystickInput {

public:

  virtual void setupThing() = 0;
  virtual bool getCurrentXY(int &x, int &y) = 0;
  
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class DirectJoystickInput : public JoystickInput {
  
private:

  int xPin, yPin;

public:

  DirectJoystickInput(int xPinIn, int yPinIn) {
    xPin = xPinIn;
    yPin = yPinIn;
  }
  
  void setupThing() {
    //nothing needed right now...
  }
  
  bool getCurrentXY(int &x, int &y) {
    x = analogRead(xPin);
    y = analogRead(yPin);
    return true;
  }
  
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static const int WIRELESS_JOYSTICK_HEADER_BYTE = 127;

static const int WIRELESS_JOYSTICK_DEAD_VALUE = 512;

static const int WIRELESS_JOYSTICK_TIMEOUT_MSEC = 500;

class WirelessJoystickInput : public JoystickInput {
  
private:

  Buffer rawValues;
  
  int lastX;
  int lastY;
  
  int nextTimeoutTime;

  void handleTick() {
    while (Serial1.available()) {
      rawValues.put(Serial1.read());
    }
  }
  
  void processBuffer() {
    while (rawValues.getSize() >= 3) {
      int head;
      do {
        rawValues.get(head);
      } while (head != WIRELESS_JOYSTICK_HEADER_BYTE && rawValues.getSize() >= 3);
      if (head == WIRELESS_JOYSTICK_HEADER_BYTE) {
        int rawX, rawY;
        rawValues.get(rawX);
        rawValues.get(rawY);
        lastX = rawX * 4;
        lastY = rawY * 4;
        nextTimeoutTime = millis() + WIRELESS_JOYSTICK_TIMEOUT_MSEC;
      }
    }
  }
  
  void checkForTimeout() {
    bool hasTrippedTimeout = nextTimeoutTime < millis();
    if (hasTrippedTimeout) {
      lastX = WIRELESS_JOYSTICK_DEAD_VALUE;
      lastY = WIRELESS_JOYSTICK_DEAD_VALUE;
    }
  }

public:

  WirelessJoystickInput() {
    lastX = WIRELESS_JOYSTICK_DEAD_VALUE;
    lastY = WIRELESS_JOYSTICK_DEAD_VALUE;
    nextTimeoutTime = 0;
  }
  
  void setupThing() {
    Serial1.begin(9600);
  }
  
  bool getCurrentXY(int &x, int &y) {
    handleTick();
    processBuffer();
    checkForTimeout();
    x = lastX;
    y = lastY;
    return true;
  }
  
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#endif /* ___TOURMAX__JOYSTICK__INPUT__H___ */

