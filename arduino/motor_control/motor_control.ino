////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <LPS331.h>
#import "vector.h"
#import "movement_control.h"
#import "smoothed_values.h"
#import "joystick.h"
#import "kiwi_drive.h"
#import "ir_array.h"
#import "py_comm.h"
#import "joystick_input.h"
#import "pneumatics.h"

/*

the orientation of the kiwi drive is such that the side opposite the battery is
the front. the the wheel on that side is the north wheel. in clockwise order
from above, the wheels are north, south west, and south east.

the y axis is away from the robot in the direction of north.
the x axis goes away from the robot to the west, not east.

*/

static const int PIN_WHEEL_NORTH = 6;
static const int PIN_WHEEL_SOUTH_WEST = 7;
static const int PIN_WHEEL_SOUTH_EAST = 5;

static const int PIN_JOYSTICK_X = 1;  //purple
static const int PIN_JOYSTICK_Y = 0;  //white

static const int PNEUMATIC_VALVE_PIN = 12;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/*

the north ir sensor should be the 0th index, with the rest going in clockwise order

*/

static const int NUM_IR_SENSORS = 6;

static const int IR_PINS[NUM_IR_SENSORS] = {3, 4, 7, 6, 5, 2}; //{2, 3, 4, 5, 6, 7};

static const bool IR_ENABLED[NUM_IR_SENSORS] = {true, true, true, true, true, true};

//these are the directions the ir sensors point in
const static Vector IR_VECTORS[NUM_IR_SENSORS] = {
  Vector(0, 1),
  Vector(sqrt(3) / 2.0f, 0.5f),
  Vector(sqrt(3) / 2.0f, -0.5f),
  Vector(0, -1),
  Vector(-sqrt(3) / 2.0f, -0.5f),
  Vector(-sqrt(3) / 2.0f, 0.5f),
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static KiwiDrive kiwiDrive(PIN_WHEEL_NORTH, PIN_WHEEL_SOUTH_WEST, PIN_WHEEL_SOUTH_EAST);
//static ZeroJoystickInput joystickInput;
static DirectJoystickInput joystickInput(PIN_JOYSTICK_X, PIN_JOYSTICK_Y);
//static WirelessJoystickInput joystickInput;
static Joystick joystick(&joystickInput, JOYSTICK_INPUT_X_FOR_ROTATION);
static IrArray irSensors(NUM_IR_SENSORS, IR_PINS, IR_VECTORS, IR_ENABLED, false);
static PyComm pyComm;
static Pneumatics pneumatics(PNEUMATIC_VALVE_PIN, 2.0f);

void setup() {
  kiwiDrive.setupThing();
  joystick.setupThing();
  irSensors.setupThing();
  pyComm.setupThing();
  pneumatics.setupThing();
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void onTick() {
  pyComm.onTick();
  pneumatics.onTick();
}

void mainControlLoop() {
  onTick();
  
  MovementControl movementControl, joystickMovementControl, pyCommMovementControl;
  joystick.readToMovementControl(joystickMovementControl);
  pyComm.readToMovementControl(pyCommMovementControl);
  mergeMovementControls(movementControl, joystickMovementControl, pyCommMovementControl);
  
  irSensors.applyToMovementControl(movementControl);
  getMovementControlInValidRange(movementControl);
  
  kiwiDrive.applyMovementControl(movementControl);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void loop() {
  mainControlLoop();
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
