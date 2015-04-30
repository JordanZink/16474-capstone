
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
  
  float x;
  float y;
  float rotation;
  unsigned long nextTimeout;
  
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
    x = (((float) xRaw) - 127) / 128;
    y = (((float) yRaw) - 127) / 128;
    rotation = (((float) rotationRaw) - 127) / 128;
    nextTimeout = millis() + (timeoutRaw * 100);
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
    x = 0.0f;
    y = 0.0f;
    rotation = 0.0f;
    nextTimeout = 0;
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
  
  void readToMovementControl(MovementControl &movementControl) {
    bool hasTimedout = nextTimeout < millis();
    if (hasTimedout == false) {
      movementControl.xyVector = Vector(x, y);
      movementControl.rotation = rotation;
    } else {
      movementControl.xyVector = Vector(0.0f, 0.0f);
      movementControl.rotation = 0.0f;
    }
  }
  
};

#endif /* ___TOURMAX__PY__COMM__H___ */
