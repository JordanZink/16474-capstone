////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#import "vector.h"
#import "movement_control.h"
#import "smoothed_values.h"
#import "joystick.h"
#import "kiwi_drive.h"

/*

the orientation of the kiwi drive is such that the side opposite the battery is
the front. the the wheel on that side is the north wheel. in clockwise order
from above, the wheels are north, south west, and south east.

the y axis is away from the robot in the direction of north.
the x axis goes away from the robot to the west, not east.

*/

static const int PIN_WHEEL_NORTH = 3;
static const int PIN_WHEEL_SOUTH_WEST = 6;
static const int PIN_WHEEL_SOUTH_EAST = 5;

static const int PIN_JOYSTICK_X = 0;
static const int PIN_JOYSTICK_Y = 1;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/*

the north ir sensor should be the 0th index, with the rest going in clockwise order

*/

static const int NUM_IR_SENSORS = 6;

static const int IR_PINS[NUM_IR_SENSORS] = {2, 3, 4, 5, 6, 7};

//these are the vectors the repulsion will be applied in
const static Vector IR_VECTORS[NUM_IR_SENSORS] = {
  Vector(0, -1),
  Vector(sqrt(3) / 2.0f, -0.5f),
  Vector(sqrt(3) / 2.0f, 0.5f),
  Vector(0, 1),
  Vector(-sqrt(3) / 2.0f, 0.5f),
  Vector(-sqrt(3) / 2.0f, -0.5f),
};

static const int IR_VALUE_MIN = 50;
static const int IR_VALUE_MAX = 550;

//where 1 means full power
static const double MAX_IR_REPULSION = 0.85;

static const int NUM_VALUES_FOR_IR_SMOOTHED_VALUES = 50;

static SmoothedValues irSmoothedValues[NUM_IR_SENSORS] = {
  SmoothedValues(NUM_VALUES_FOR_IR_SMOOTHED_VALUES, IR_VALUE_MAX),
  SmoothedValues(NUM_VALUES_FOR_IR_SMOOTHED_VALUES, IR_VALUE_MAX),
  SmoothedValues(NUM_VALUES_FOR_IR_SMOOTHED_VALUES, IR_VALUE_MAX),
  SmoothedValues(NUM_VALUES_FOR_IR_SMOOTHED_VALUES, IR_VALUE_MAX),
  SmoothedValues(NUM_VALUES_FOR_IR_SMOOTHED_VALUES, IR_VALUE_MAX),
  SmoothedValues(NUM_VALUES_FOR_IR_SMOOTHED_VALUES, IR_VALUE_MAX),
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static KiwiDrive kiwiDrive(PIN_WHEEL_NORTH, PIN_WHEEL_SOUTH_WEST, PIN_WHEEL_SOUTH_EAST);
static Joystick joystick(PIN_JOYSTICK_X, PIN_JOYSTICK_Y);

void setup() {
  kiwiDrive.setup();
  joystick.setup();
  Serial.begin(9600);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


Vector getIrVector() {
  Vector resultVector(0.0f, 0.0f);
  /*
  for (int i = 0; i < NUM_IR_SENSORS; i++) {
    if (i == 3) {continue;} //because that sensor is broken right now
    int rawVal = analogRead(IR_PINS[i]);
    irSmoothedValues[i].addValue(rawVal);
    int val = irSmoothedValues[i].getSmoothedValue();
    val = constrain(val, IR_VALUE_MIN, IR_VALUE_MAX);
    float mag = ((pow(val, 2) - pow(IR_VALUE_MIN, 2)) / pow(IR_VALUE_MAX - IR_VALUE_MIN, 2)) * MAX_IR_REPULSION; //quadratic
    //float mag = ((((float) val) - IR_VALUE_MIN) / (IR_VALUE_MAX - IR_VALUE_MIN)) * MAX_IR_REPULSION; //linear
    Vector curVector(IR_VECTORS[i]);
    curVector.normalize();
    curVector.mult(mag);
    resultVector.add(curVector);
  }
  */
  return resultVector;
}

void joystickControlLoop() {
  MovementControl movementControl;
  //struct WheelPower wp;

  joystick.readToMovementControl(movementControl);
  Vector irVector = getIrVector();
  movementControl.xyVector.add(irVector);
  if (movementControl.xyVector.getMagnitude() > 1.0f) {
    movementControl.xyVector.normalize();
  }
  //convertMotorVectorToWheelPower(movementControl, wp);
  //sendWheelPower(wp);
  kiwiDrive.applyMovementControl(movementControl);

  delay(2);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static const int ARDUINO_ACKNOWLEDGE_SERIAL_CODE = 201;
static const int ARDUINO_ERROR_SERIAL_CODE = 212;

static const int MOTOR_CONTROL_SERIAL_CODE = 199;

static int nextTimeoutTime = 0;
static MovementControl lastCommandedMovementControl;
static bool useIr = true;

int readIntBlocking() {
  while (Serial.available() == 0) {}
  return Serial.read();
}

void receiveMotorControlOverSerial(MovementControl &movementControl, int &nextTimeout) {
  //for right now, it will just receive two bytes, one for x, one for y, and one for rotation. then the values are mapped to [-1, 1] floats
  int xRaw = readIntBlocking();
  int yRaw = readIntBlocking();
  int rotationRaw = readIntBlocking();
  int timeoutRaw = readIntBlocking();
  Serial.write(ARDUINO_ACKNOWLEDGE_SERIAL_CODE);
  float x = (((float) xRaw) - 127) / 128;
  float y = (((float) yRaw) - 127) / 128;
  float rotation = (((float) rotationRaw) - 127) / 128;
  movementControl.xyVector = Vector(x, y);
  movementControl.rotation = rotation;
  nextTimeout = millis() + (timeoutRaw * 100);
}

void applyLastCommandedMovementControl() {
  MovementControl movementControl;
  movementControl.xyVector = Vector(lastCommandedMovementControl.xyVector);
  movementControl.rotation = lastCommandedMovementControl.rotation;
  if (useIr == true) {
    Vector irVector = getIrVector();
    movementControl.xyVector.add(irVector);
  }
  if (movementControl.xyVector.getMagnitude() > 1.0f) {
    movementControl.xyVector.normalize();
  }
  //struct WheelPower wp;
  //convertMotorVectorToWheelPower(movementControl, wp);
  //sendWheelPower(wp);
  kiwiDrive.applyMovementControl(movementControl);
}

void stopMotion() {
  MovementControl movementControl;
  movementControl.xyVector = Vector(0.0f, 0.0f);
  movementControl.rotation = 0.0f;
  //struct WheelPower wp;
  //convertMotorVectorToWheelPower(movementControl, wp);
  //sendWheelPower(wp);
  kiwiDrive.applyMovementControl(movementControl);
}

void raspiControlLoop() {
  if (Serial.available() > 0) {
    //there are things to be read; process it
    int commandCode = Serial.read();
    switch (commandCode) {
      case MOTOR_CONTROL_SERIAL_CODE:
        receiveMotorControlOverSerial(lastCommandedMovementControl, nextTimeoutTime);
        break;
      //more would go here
      default:
        Serial.write(ARDUINO_ERROR_SERIAL_CODE);
        break;
    }
  }
  
  bool hasTrippedWatchdog = nextTimeoutTime < millis();
  if (hasTrippedWatchdog == false) {
    applyLastCommandedMovementControl();
  } else {
    stopMotion();
  }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void loop() {
  //joystickControlLoop();
  raspiControlLoop();
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
