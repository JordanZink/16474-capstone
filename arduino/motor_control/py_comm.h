
#ifndef ___TOURMAX__PY__COMM__H___
#define ___TOURMAX__PY__COMM__H___

#import "Arduino.h"
#import "vector.h"
#import "movement_control.h"
#import "buffer.h"

class PyComm {
  
private:

  static const int ARDUINO_ACKNOWLEDGE_SERIAL_CODE = 201;
  static const int ARDUINO_ERROR_SERIAL_CODE = 212;
  static const int MOTOR_CONTROL_SERIAL_CODE = 199;
  static const int MOTOR_CONTROL_NUM_BYTES = 5;
  static const int MAGIC_HEADER = 255;

  Buffer rawBuffer;
  
  float lastX;
  float lastY;
  float lastRotation;
  unsigned long lastTimeout;
  
  bool eatRawBufferUntilHeader() {
    int curByte;
    while (rawBuffer.peek(curByte) == true) {
      if (curByte == MAGIC_HEADER) {
        return true;
      }
      rawBuffer.drop();
    }
    return false;
  }
  
  void processMotorCommand() {
    rawBuffer.drop();
    int xRaw, yRaw, rotationRaw, timeoutRaw;
    rawBuffer.get(xRaw);
    rawBuffer.get(yRaw);
    rawBuffer.get(rotationRaw);
    rawBuffer.get(timeoutRaw);
    lastX = (((float) xRaw) - 127) / 128;
    lastY = (((float) yRaw) - 127) / 128;
    lastRotation = (((float) rotationRaw) - 127) / 128;
    lastTimeout = millis() + (timeoutRaw * 100);
  }
  
  void checkRawForCommand() {
    if (eatRawBufferUntilHeader() == false) {return;}
    int code;
    if (rawBuffer.peek_forward(code, 1) == false) {return;}
    switch (code) {
      case MOTOR_CONTROL_SERIAL_CODE:
        if (rawBuffer.getSize() < MOTOR_CONTROL_NUM_BYTES + 1) {return;}
        Serial.write(ARDUINO_ACKNOWLEDGE_SERIAL_CODE);
        Serial.flush();
        rawBuffer.drop();
        processMotorCommand();
        return;
      
      default:
        Serial.write(ARDUINO_ERROR_SERIAL_CODE);
        Serial.flush();
        rawBuffer.drop();
    }
  }

public:

  PyComm() {
    lastX = 0.0f;
    lastY = 0.0f;
    lastRotation = 0.0f;
    lastTimeout = 0;
  }

  void setupThing() {
    Serial.begin(9600);
  }
  
  void onTick() {
    while (Serial.available() > 0 && rawBuffer.isFull() == false) {
      rawBuffer.put(Serial.read());
      Serial.flush();
    }
    checkRawForCommand();
  }
  
  bool getMovementControl(MovementControl &movementControl, unsigned long &nextTimeout) {
    movementControl.xyVector = Vector(lastX, lastY);
    movementControl.rotation = lastRotation;
    nextTimeout = lastTimeout;
  }
  
};

#endif /* ___TOURMAX__PY__COMM__H___ */
