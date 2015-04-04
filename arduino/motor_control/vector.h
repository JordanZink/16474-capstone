
#include "math.h"

static inline int mapFloat(float val, float loIn, float hiIn, int loOut, int hiOut) {
  float inRange = hiIn - loIn;
  int outRange = hiOut - loOut;
  return (int) (loOut + (((val - loIn) / inRange) * outRange));
}

class Vector {
  
private:
  float x;
  float y;
  
public:
  
  Vector(float xIn, float yIn) {
    x = xIn;
    y = yIn;
  }
  
  Vector(int xIn, int yIn, int rangeLo, int rangeHi) {
    int range = rangeHi - rangeLo;
    x = ((float) (xIn - rangeLo)) * 2.0f / range - 1.0f;
    y = ((float) (yIn - rangeLo)) * 2.0f / range - 1.0f;
  }
  
  Vector(const Vector &vIn) {
    x = vIn.x;
    y = vIn.y;
  }
  
  void getComponentsFloat(float &xIn, float &yIn) {
    xIn = x;
    yIn = y;
  }
  
  void getComponentsInt(int &xIn, int &yIn) {
    xIn = (int) (x);
    yIn = (int) (y);
  }
  
  float getMagnitude() {
    return sqrt((x * x) + (y * y));
  }  
  
  void setMagnitude(float desiredMag) {
    float curMag = getMagnitude();
    x = x / curMag * desiredMag;
    y = y / curMag * desiredMag;
  }
  
  void normalize() {
    setMagnitude(1.0f);
  }
  
  void mult(float multiplier) {
    x *= multiplier;
    y *= multiplier;
  }
  
  void add(Vector other) {
    x += other.x;
    y += other.y;
  }
  
};

