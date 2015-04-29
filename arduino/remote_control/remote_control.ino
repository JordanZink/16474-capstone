
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10,11);

static const int X_PIN = 0;
static const int Y_PIN = 1;

void setup(){
  Serial.begin(9600);
  mySerial.begin(9600);
}

static const int HEADER_BYTE = 127;
//static const int RESP_BEGIN = 121;
//static const int SUCCESS = 122;

void loop(){
  int rawX = analogRead(X_PIN);
  int rawY = analogRead(Y_PIN);
  int xToSend = rawX / 4;
  int yToSend = rawY / 4;
  if (xToSend == HEADER_BYTE) {xToSend -= 1;}
  if (yToSend == HEADER_BYTE) {yToSend -= 1;}
  mySerial.write(HEADER_BYTE);
  mySerial.write(xToSend);
  mySerial.write(yToSend);
  delay(100);
  /*
  mySerial.write(REQ_NUM);
  while (mySerial.available() == 0) {}
  int resp = mySerial.read();  
  int x = mySerial.read();  
  int y = mySerial.read();
  mySerial.write(SUCCESS);
  Serial.println(x);
  Serial.println(y);
  delay(500);
  */
  /*
  if (gotEcho == true) {
    fooey = (fooey + 1) % 200;
    mySerial.write(fooey);
    gotEcho = false;
  }
  
  if (mySerial.available() > 0) {
    Serial.println(mySerial.read());
    gotEcho = true;
  }
  */
  /*
  if(mySerial.available()){
    sdata = mySerial.read();
    Serial.println(sdata);
    mySerial.print(sdata);
  }
  */
  /*mySerial.println('a');
  mySerial.println('b');
  delay(100);*/
}
  
