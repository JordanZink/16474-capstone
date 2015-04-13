#ifndef ___TOURMAX__PY__COMM__H___
#define ___TOURMAX__PY__COMM__H___

#import "Arduino.h"
#import "vector.h"
#import "movement_control.h"

class PyComm {
  
private:

  static const int ARDUINO_ACKNOWLEDGE_SERIAL_CODE = 201;
  static const int ARDUINO_ERROR_SERIAL_CODE = 212;
  static const int MOTOR_CONTROL_SERIAL_CODE = 199;
  static const int MOTOR_CONTROL_NUM_BYTES = 5;

  static const int BUFFER_SIZE = 64;
  
  int rawBuffer[BUFFER_SIZE];
  int consumeBuffer[BUFFER_SIZE];
  int rawI;
  int consumeI;
  
  static void dropBuffer(int* buffer, int curI, int numToDrop) {
    for (int i = numToDrop; i < curI; i++) {
      buffer[i - numToDrop] = buffer[i];
    }
  }
  
  void checkRawForCommand() {
    if (rawI == 0) {return;}
    switch (rawBuffer[0]) {
      case MOTOR_CONTROL_SERIAL_CODE:
        if (rawI < MOTOR_CONTROL_NUM_BYTES) {return;}
        if (consumeI + MOTOR_CONTROL_NUM_BYTES >= BUFFER_SIZE) {
          Serial.write(ARDUINO_ERROR_SERIAL_CODE);
          return;
        }
        Serial.write(ARDUINO_ACKNOWLEDGE_SERIAL_CODE);
        for (int i = 0; i < MOTOR_CONTROL_NUM_BYTES; i++) {
          consumeBuffer[consumeI + i] = rawBuffer[i];
        }
        dropBuffer(rawBuffer, rawI, MOTOR_CONTROL_NUM_BYTES);
        rawI -= MOTOR_CONTROL_NUM_BYTES;
        return;
        
      default:
        Serial.write(ARDUINO_ERROR_SERIAL_CODE);
        dropBuffer(rawBuffer, rawI, 1);
        rawI -= 1;
    }
  }

public:

  PyComm() {
    rawI = 0;
    consumeI = 0;
  }

  void setup() {
    Serial.begin(9600);
  }
  
  void onTick() {
    while (Serial.available() > 0 && rawI < BUFFER_SIZE) {
      rawBuffer[rawI] = Serial.read();
      rawI++;
    }
    checkRawForCommand();
  }
  
  bool getMovementControl(MovementControl &movementControl, int &nextTimeout) {
    if (consumeBuffer[0] != MOTOR_CONTROL_SERIAL_CODE) {
      return false;
    }
    int xRaw = consumeBuffer[1];
    int yRaw = consumeBuffer[2];
    int rotationRaw = consumeBuffer[3];
    int timeoutRaw = consumeBuffer[4];
    dropBuffer(consumeBuffer, consumeI, MOTOR_CONTROL_NUM_BYTES);
    consumeI -= MOTOR_CONTROL_NUM_BYTES;
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
