////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

/*

the orientation of the kiwi drive is such that the side opposite the battery is
the front. the the wheel on that side is the north wheel. in clockwise order
from above, the wheels are north, south west, and south east.

*/

static const int PIN_JOYSTICK_X = 1;
static const int PIN_JOYSTICK_Y = 0;

static const int PIN_WHEEL_NORTH = 3;
static const int PIN_WHEEL_SOUTH_WEST = 5;
static const int PIN_WHEEL_SOUTH_EAST = 6;

/*

currently assuming the interval represents both negative and positive. so
halfway between min and max indicates no movement, below that is negative,
and above that is positive

*/

static const int WHEEL_VALUE_MIN = 0;//128 - 128 / 4;
static const int WHEEL_VALUE_MAX = 255;//(255 - 128) / 4 + 128;

static const int JOYSTICK_VALUE_MIN = 0;
static const int JOYSTICK_VALUE_MAX = 1023;

static const int JOYSTICK_DEAD_ZONE_MIN = 450;
static const int JOYSTICK_DEAD_ZONE_MAX = 550;

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

struct JoystickReading {
  int x;
  int y;
};

int applyDeadZone(int x, int lo, int hi) {
  if (lo < x && x < hi) {
    return 512; //todo get rid of const
  } else {
    return x;
  }
}

void readJoystick(struct JoystickReading* jr) {
  jr->x = analogRead(PIN_JOYSTICK_X);
  jr->y = analogRead(PIN_JOYSTICK_Y);
  jr->x = applyDeadZone(jr->x, JOYSTICK_DEAD_ZONE_MIN, JOYSTICK_DEAD_ZONE_MAX);
  jr->y = applyDeadZone(jr->y, JOYSTICK_DEAD_ZONE_MIN, JOYSTICK_DEAD_ZONE_MAX);
  /*
  Serial.println("test2");
  Serial.println(jr->x);
  Serial.println(jr->y);
  */
}

void sendPower(int power) {
  analogWrite(PIN_WHEEL_NORTH, power);
  analogWrite(PIN_WHEEL_SOUTH_WEST, power);
  analogWrite(PIN_WHEEL_SOUTH_EAST, power);
}

void runLinearRange(int startPower, int endPower, int stp, int delayTime) {
  int mult = (startPower < endPower ? 1 : -1);
  for (int power = startPower; power * mult < endPower * mult; power += stp * mult) {
    sendPower(power);
    Serial.println(power);
    delay(delayTime);
  }
  sendPower(endPower);
  Serial.println(endPower);
  delay(delayTime);
}


const int maxPower = 450;
const int minPower = 300;
const int neutralPower = (maxPower + minPower) / 2;

void runCalibration() {
  const int stp = 8;
  const int delayTime = 30;
  const int delayBetweenRanges = 500;
  runLinearRange(neutralPower, maxPower, stp, delayTime);
  delay(delayBetweenRanges);
  runLinearRange(maxPower, neutralPower, stp, delayTime);
  delay(delayBetweenRanges);
  runLinearRange(neutralPower, minPower, stp, delayTime);
  delay(delayBetweenRanges);
  runLinearRange(minPower, neutralPower, stp, delayTime);
}

void runCalibration2() {
  for (int power = 0; power < 300; power++) {
    sendPower(power);
    Serial.println(power);
    delay(100);
  }
}

void loop() {
  struct JoystickReading jr;
  
  readJoystick(&jr);
  if (jr.x < 100) {
    //runCalibration();
    runCalibration2();
  }
  sendPower(neutralPower);
  delay(100);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
