################################################################################
## arduinoComm.py      #########################################################
## Jordan Zink + jzink #########################################################
## 16-474 S15          #########################################################
################################################################################

import serial
import time

MAGIC_COMMAND_HEADER = 255

ARDUINO_ACKNOWLEDGE_SERIAL_CODE = 201
ARDUINO_ERROR_SERIAL_CODE = 212

MOTOR_CONTROL_SERIAL_CODE = 199

DEFAULT_TIMEOUT = 0.25 #seconds

class ArduinoComm(object):
    def __init__(self, serialName, serialBaud):
        self.serial = serial.Serial(serialName, serialBaud, timeout=DEFAULT_TIMEOUT)
        time.sleep(1)

    @staticmethod
    def signedFloatToByte(f):
        f = max(min(f, 1.0), -1.0)
        b = int(round((f * 128))) + 127
        b = max(min(b, 255), 0)
        return b
        
    def sendHeaderByte():
        self.serial.write(chr(MAGIC_COMMAND_HEADER))
        
    def sendNormalByte(b):
        if b == MAGIC_COMMAND_HEADER:
            b -= 1
        self.serial.write(chr(b))

    def motorVector(self, x, y, rotation, timeout):
        xByte = ArduinoComm.signedFloatToByte(x)
        yByte = ArduinoComm.signedFloatToByte(y)
        rotationByte = ArduinoComm.signedFloatToByte(rotation)
        timeoutByte = min(max(int(math.ceil(timeout / 100)), 0), 255)
        self.sendHeaderByte()
        self.sendNormalByte(MOTOR_CONTROL_SERIAL_CODE)
        self.sendNormalByte(xByte)
        self.sendNormalByte(yByte)
        self.sendNormalByte(rotationByte)
        self.sendNormalByte(timeoutByte)
        self.serial.flush()
        response = self.serial.read()
        if len(response) == 0:
            return None
        else:
            return response == chr(ARDUINO_ACKNOWLEDGE_SERIAL_CODE)

    def disableIrAvoidance(self):
        raise NotImplementedError("does not support disable ir avoidance yet")

    def enableIrAvoidance(self):
        raise NotImplementedError("does not support enable ir avoidance yet")

################################################################################
################################################################################
################################################################################
