
#ifndef ___TOURMAX__IR__ARRAY__H___
#define ___TOURMAX__IR__ARRAY__H___

#import "Arduino.h"
#import "vector.h"
#import "movement_control.h"
#import "smoothed_values.h"

static const int IR_VALUE_MIN = 50;
static const int IR_VALUE_MAX = 550;

//where 1 means full power
static const double MAX_IR_REPULSION = 0.85;

class IrArray {
 
private:

  struct SensorInfo {
    int pin;
    Vector dir;
    SmoothedValues smoothedValues;
    float weight;
  };

  int numSensors;
  struct SensorInfo* sensorInfos;
  
  static inline float constrainFloat(float v, float minV, float maxV) {
    if (v < minV) {
      return minV;
    } else if (v > maxV) {
      return maxV;
    } else {
      return v;
    }
  }
  
public:

  IrArray(const int numSensorsIn, const int* sensorPinsIn, const Vector* sensorDirectionsIn) {
    numSensors = numSensorsIn;
    sensorInfos = new struct SensorInfo[numSensorsIn];
    for (int i = 0; i < numSensors; i++) {
      sensorInfos[i].pin = sensorPinsIn[i];
      sensorInfos[i].dir = sensorDirectionsIn[i];
      sensorInfos[i].smoothedValues = SmoothedValues(IR_VALUE_MAX);
      sensorInfos[i].weight = 1.0f;
    }
  }
  
  ~IrArray() {
    delete sensorInfos;
  }
  
  void setupThing() {
    //nothing to do right now
  }
  
  Vector getRepulsionVector() {
    Vector resultVector(0.0f, 0.0f);
    for (int i = 0; i < numSensors; i++) {
      struct SensorInfo* info = &(sensorInfos[i]);
      int rawVal = analogRead(info->pin);
      info->smoothedValues.addValue(rawVal);
      int val = info->smoothedValues.getSmoothedValue();
      val = constrain(val, IR_VALUE_MIN, IR_VALUE_MAX);
      float mag = ((pow(val, 2) - pow(IR_VALUE_MIN, 2)) / pow(IR_VALUE_MAX - IR_VALUE_MIN, 2)) * MAX_IR_REPULSION; //quadratic
      //float mag = ((((float) val) - IR_VALUE_MIN) / (IR_VALUE_MAX - IR_VALUE_MIN)) * MAX_IR_REPULSION; //linear
      mag *= info->weight;
      Vector curVector(info->dir);
      curVector.normalize();
      curVector.mult(-mag);
      resultVector.add(curVector);
    }
    return resultVector;
  }
  
  float getSensorWeight(int sensorI) {
    return sensorInfos[sensorI].weight;
  }
  
  void setSensorWeight(int sensorI, float newWeight) {
    sensorInfos[sensorI].weight = constrainFloat(newWeight, 0.0f, 1.0f);
  }
  
};

#endif /* ___TOURMAX__IR__ARRAY__H___ */

