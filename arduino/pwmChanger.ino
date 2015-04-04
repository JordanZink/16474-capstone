int val = 127;
int pwm = 127;
int motor = 9;

void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR1A = TCCR1A & 0b11111000 | mode;
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(motor,OUTPUT);
  //setPwmFrequency(9,8);
}

void loop()
{
  if(Serial.available())
  {
    val = 0;
    while(Serial.available())
      val = val*10+int(Serial.read())-48;
  }
  val = constrain(val,0,255);
  pwm = map(val,0,255,100,254); //Wasp Driver only reads 100-254
  
  Serial.print(val); Serial.print("\t");
  Serial.println(pwm);
  analogWrite(motor,pwm);
  
  delay(10);
}
