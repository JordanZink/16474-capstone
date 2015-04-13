################################################################################
## arduinoComm.py      #########################################################
## Jordan Zink + jzink #########################################################
## 16-474 S15          #########################################################
################################################################################

import serial
import time

ARDUINO_ACKNOWLEDGE_SERIAL_CODE = 201
ARDUINO_ERROR_SERIAL_CODE = 212

MOTOR_CONTROL_SERIAL_CODE = 199

class ArduinoComm(object):
    def __init__(self, serialName, serialBaud):
        self.serial = serial.Serial(serialName, serialBaud)
        time.sleep(1)

    @staticmethod
    def signedFloatToByte(f):
        f = max(min(f, 1.0), -1.0)
        b = int(round((f * 128))) + 127
        b = max(min(b, 255), 0)
        return b

    def motorVector(self, x, y, rotation, timeout):
        xByte = ArduinoComm.signedFloatToByte(x)
        yByte = ArduinoComm.signedFloatToByte(y)
        rotationByte = ArduinoComm.signedFloatToByte(rotation)
        timeoutByte = min(max(int(math.ceil(timeout / 100)), 0), 255)
        self.serial.write(chr(MOTOR_CONTROL_SERIAL_CODE))
        self.serial.write(chr(xByte))
        self.serial.write(chr(yByte))
        self.serial.write(chr(rotationByte))
        self.serial.write(chr(timeoutByte))
        self.serial.flush()
        response = self.serial.read()
        return response == chr(ARDUINO_ACKNOWLEDGE_SERIAL_CODE)

    def disableIrAvoidance(self):
        raise NotImplementedError("does not support disable ir avoidance yet")

    def enableIrAvoidance(self):
        raise NotImplementedError("does not support enable ir avoidance yet")

################################################################################
################################################################################
################################################################################
