
#ifndef ___TOURMAX__SMOOTHED__VALUES__H___
#define ___TOURMAX__SMOOTHED__VALUES__H___

class SmoothedValues {

private:

  static const int NUM_THINGS = 47;

  int values[NUM_THINGS];
  int curI;

public:

  SmoothedValues() {
    curI = 0;
  }
  
  SmoothedValues(int defaultVal) {
    for (int i = 0; i < NUM_THINGS; i++) {
      values[i] = defaultVal;
    }
    curI = 0;
  }
  
  ~SmoothedValues() {
    //nothiing right now...
  }
  
  void addValue(int val) {
    values[curI] = val;
    curI = (curI + 1) % NUM_THINGS;
  }
  
  int getSmoothedValue() {
    long sum = 0;
    int minVal = values[0];
    int maxVal = values[0];
    for (int i = 0; i < NUM_THINGS; i++) {
      int val = values[i];
      sum += val;
      minVal = (val < minVal ? val : minVal);
      maxVal = (val > maxVal ? val : maxVal);
    }
    sum -= (minVal + maxVal);
    return sum / (NUM_THINGS - 2);
  }
  
};

#endif /* ___TOURMAX__SMOOTHED__VALUES__H___ */

