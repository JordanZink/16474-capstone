////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#import "vector.h"
#import "smoothed_values.h"

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

/*

currently assuming the interval represents both negative and positive. so
halfway between min and max indicates no movement, below that is negative,
and above that is positive

*/

//red=330,80
//orange=445,190
//green=515,250

//static const int WHEEL_VALUE_MIN = 256;//128 - 128 / 4;
//static const int WHEEL_VALUE_MAX = 512;//(255 - 128) / 4 + 128;
static const int WHEEL_POWER_FULL_REVERSE = 190 - (190 - 80) / 5; //80; //330;
static const int WHEEL_POWER_NO_MOVEMENT = 190; //445;
static const int WHEEL_POWER_FULL_FORWARD = 190 + (250 - 190) / 5; //250; //515;

static const int JOYSTICK_VALUE_MIN = 0;
static const int JOYSTICK_VALUE_MAX = 1023;

//100 implies the entire thing is dead
static const int JOYSTICK_DEAD_ZONE_PERCENT = 10;

static const int NUM_VALUES_FOR_WHEEL_SMOOTHED_VALUES = 21;

static SmoothedValues wheelNorthSmoothed(NUM_VALUES_FOR_WHEEL_SMOOTHED_VALUES, WHEEL_POWER_NO_MOVEMENT);
static SmoothedValues wheelSouthWestSmoothed(NUM_VALUES_FOR_WHEEL_SMOOTHED_VALUES, WHEEL_POWER_NO_MOVEMENT);
static SmoothedValues wheelSouthEastSmoothed(NUM_VALUES_FOR_WHEEL_SMOOTHED_VALUES, WHEEL_POWER_NO_MOVEMENT);

static const int NUM_VALUES_FOR_JOYSTICK_SMOOTHED_VALUES = 39;

static SmoothedValues joystickXSmoothed(NUM_VALUES_FOR_JOYSTICK_SMOOTHED_VALUES, 0);
static SmoothedValues joystickYSmoothed(NUM_VALUES_FOR_JOYSTICK_SMOOTHED_VALUES, 0);

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

void setup() {
  pinMode(PIN_WHEEL_NORTH, OUTPUT);
  pinMode(PIN_WHEEL_SOUTH_WEST, OUTPUT);
  pinMode(PIN_WHEEL_SOUTH_EAST, OUTPUT);
  Serial.begin(9600);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

struct WheelPower {
  int north;
  int southWest;
  int southEast;
};

struct MovementControl {
  Vector xyVector;
  float rotation;
};

long applyDeadZone(long x, int lo, int hi, long deadValue) {
  if (lo < x && x < hi) {
    return deadValue;
  } else {
    return x;
  }
}

void readJoystick(struct MovementControl &movementControl) {
  int rawX = analogRead(PIN_JOYSTICK_X);
  int rawY = analogRead(PIN_JOYSTICK_Y);
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
  /*
  movementControl.xyVector = Vector(x, y, -symmetricRangeForDeadzone, symmetricRangeForDeadzone);
  movementControl.rotation = 0.0f;
  */
  movementControl.xyVector = Vector(0, y, -symmetricRangeForDeadzone, symmetricRangeForDeadzone);
  movementControl.rotation = ((float) x) / symmetricRangeForDeadzone;
}

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

int convertValueInLinearRangeToMotorPower(int val, int rangeMin, int rangeMax) {
  int rangeMiddle = (rangeMin + rangeMax) / 2;
  if (val < rangeMiddle) {
    //reverse
    return map(val, rangeMin, rangeMiddle, WHEEL_POWER_FULL_REVERSE, WHEEL_POWER_NO_MOVEMENT);
  } else if (val > rangeMiddle) {
    //forward
    return map(val, rangeMiddle, rangeMax, WHEEL_POWER_NO_MOVEMENT, WHEEL_POWER_FULL_FORWARD);
  } else {
    //in the middle, no motion
    return WHEEL_POWER_NO_MOVEMENT;
  }
}

//http://stackoverflow.com/questions/3748037/how-to-control-a-kiwi-drive-robot

//assumes the vector has a magnitude of 1 for full power
void convertMotorVectorToWheelPower(struct MovementControl &movementControl,
                                    struct WheelPower &wp) {
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
  wp.north = xForWheels;
  int xComponentForSouth = -xForWheels / 2;
  int yComponentForSouth = -((int) (sqrt(3) / 2 * yForWheels));
  wp.southWest = xComponentForSouth - yComponentForSouth;
  wp.southEast = xComponentForSouth + yComponentForSouth;
  //for whatever reason, everything needs to be negated...
  wp.north *= -1;
  wp.southWest *= -1;
  wp.southEast *= -1;
  //now, get them in the right range
  wp.north = convertValueInLinearRangeToMotorPower(wp.north, -symmetricRangeForCalc, symmetricRangeForCalc);
  wp.southWest = convertValueInLinearRangeToMotorPower(wp.southWest, -symmetricRangeForCalc, symmetricRangeForCalc);
  wp.southEast = convertValueInLinearRangeToMotorPower(wp.southEast, -symmetricRangeForCalc, symmetricRangeForCalc);
  //THEN, APPLY ROTATION...I GUESS???
  int rotationPower = ((int) (-10 * rotation));
  wp.north += rotationPower;
  wp.southWest += rotationPower;
  wp.southEast += rotationPower;
}

void sendWheelPower(struct WheelPower &wp) {
  wheelNorthSmoothed.addValue(wp.north);
  wheelSouthWestSmoothed.addValue(wp.southWest);
  wheelSouthEastSmoothed.addValue(wp.southEast);
  int wheelNorth = wheelNorthSmoothed.getSmoothedValue();
  int wheelSouthWest = wheelSouthWestSmoothed.getSmoothedValue();
  int wheelSouthEast = wheelSouthEastSmoothed.getSmoothedValue();
  analogWrite(PIN_WHEEL_NORTH, wheelNorth);
  analogWrite(PIN_WHEEL_SOUTH_WEST, wheelSouthWest);
  analogWrite(PIN_WHEEL_SOUTH_EAST, wheelSouthEast);
}

void joystickControlLoop() {
  struct MovementControl movementControl;
  struct WheelPower wp;

  readJoystick(movementControl);
  Vector irVector = getIrVector();
  movementControl.xyVector.add(irVector);
  if (movementControl.xyVector.getMagnitude() > 1.0f) {
    movementControl.xyVector.normalize();
  }
  convertMotorVectorToWheelPower(movementControl, wp);
  sendWheelPower(wp);

  delay(2);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static const int ARDUINO_ACKNOWLEDGE_SERIAL_CODE = 201;
static const int ARDUINO_ERROR_SERIAL_CODE = 212;

static const int MOTOR_CONTROL_SERIAL_CODE = 199;

static struct MovementControl lastCommandedMovementControl;
static bool useIr = true;

int readIntBlocking() {
  while (Serial.available() == 0) {}
  return Serial.read();
}

void receiveMotorControlOverSerial(struct MovementControl &movementControl) {
  //for right now, it will just receive two bytes, one for x, one for y, and one for rotation. then the values are mapped to [-1, 1] floats
  int xRaw = readIntBlocking();
  int yRaw = readIntBlocking();
  int rotationRaw = readIntBlocking();
  Serial.write(ARDUINO_ACKNOWLEDGE_SERIAL_CODE);
  float x = (((float) xRaw) - 127) / 128;
  float y = (((float) yRaw) - 127) / 128;
  float rotation = (((float) rotationRaw) - 127) / 128;
  movementControl.xyVector = Vector(x, y);
  movementControl.rotation = rotation;
}

void raspiControlLoop() {
  if (Serial.available() > 0) {
    //there are things to be read; process it
    int commandCode = Serial.read();
    switch (commandCode) {
      case MOTOR_CONTROL_SERIAL_CODE:
        {
        receiveMotorControlOverSerial(lastCommandedMovementControl);
        }
        break;
      //more would go here
      default:
        Serial.write(ARDUINO_ERROR_SERIAL_CODE);
        break;
    }
  } else {
    //nothing new to be read, but need to check if there is anything that should be done

    //watchdogs and such would go here; for now just move
    struct MovementControl movementControl;
    movementControl.xyVector = Vector(lastCommandedMovementControl.xyVector);
    movementControl.rotation = lastCommandedMovementControl.rotation;
    if (useIr == true) {
      Vector irVector = getIrVector();
      movementControl.xyVector.add(irVector);
    }
    if (movementControl.xyVector.getMagnitude() > 1.0f) {
      movementControl.xyVector.normalize();
    }
    struct WheelPower wp;
    convertMotorVectorToWheelPower(movementControl, wp);
    sendWheelPower(wp);
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
