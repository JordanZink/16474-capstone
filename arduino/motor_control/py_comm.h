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

  Buffer rawBuffer;
  Buffer consumeBuffer;
  
  void checkRawForCommand() {
    int code;
    if (rawBuffer.peek(code) == false) {return;}
    switch (code) {
      case MOTOR_CONTROL_SERIAL_CODE:
        if (rawBuffer.getSize() < MOTOR_CONTROL_NUM_BYTES) {return;}
        if (consumeBuffer.canAcceptNumElements(MOTOR_CONTROL_NUM_BYTES) == false) {
          Serial.write(ARDUINO_ERROR_SERIAL_CODE);
          for (int i = 0; i < MOTOR_CONTROL_NUM_BYTES; i++) {
            rawBuffer.drop();
          }
          return;
        }
        Serial.write(ARDUINO_ACKNOWLEDGE_SERIAL_CODE);
        for (int i = 0; i < MOTOR_CONTROL_NUM_BYTES; i++) {
          int v;
          rawBuffer.get(v);
          consumeBuffer.put(v);
        }
        return;
        
      default:
        Serial.write(ARDUINO_ERROR_SERIAL_CODE);
        rawBuffer.drop();
    }
  }

public:

  PyComm() {
    //nothing right now
  }

  void setupThing() {
    Serial.begin(9600);
  }
  
  void onTick() {
    while (Serial.available() > 0 && rawBuffer.isFull() == false) {
      rawBuffer.put(Serial.read());
    }
    checkRawForCommand();
  }
  
  bool getMovementControl(MovementControl &movementControl, int &nextTimeout) {
    int code;
    consumeBuffer.peek(code);
    if (consumeBuffer.isEmpty() || code != MOTOR_CONTROL_SERIAL_CODE) {
      return false;
    }
    consumeBuffer.drop();
    int xRaw, yRaw, rotationRaw, timeoutRaw;
    consumeBuffer.get(xRaw);
    consumeBuffer.get(yRaw);
    consumeBuffer.get(rotationRaw);
    consumeBuffer.get(timeoutRaw);
    float x = (((float) xRaw) - 127) / 128;
    float y = (((float) yRaw) - 127) / 128;
    float rotation = (((float) rotationRaw) - 127) / 128;
    movementControl.xyVector = Vector(x, y);
    movementControl.rotation = rotation;
    nextTimeout = millis() + (timeoutRaw * 100);
    return true;
  }
  
};

#endif /* ___TOURMAX__PY__COMM__H___ */
