
#ifndef ___TOURMAX__SMOOTHED__VALUES__H___
#define ___TOURMAX__SMOOTHED__VALUES__H___

class SmoothedValues {

private:

  int* values;
  int n;
  int curI;

public:

  SmoothedValues() {
    n = 0;
    values = 0;
    curI = 0;
  }
  
  SmoothedValues(int nIn, int defaultVal) {
    //must be at least 3
    if (nIn < 3) {
      nIn = 3;
    }
    n = nIn;
    values = new int[n];
    for (int i = 0; i < n; i++) {
      values[i] = defaultVal;
    }
    curI = 0;
  }
  
  ~SmoothedValues() {
    if (values != 0) {
      delete values;
    }
  }
  
  void addValue(int val) {
    values[curI] = val;
    curI = (curI + 1) % n;
  }
  
  int getSmoothedValue() {
    long sum = 0;
    int minVal = values[0];
    int maxVal = values[0];
    for (int i = 0; i < n; i++) {
      int val = values[i];
      sum += val;
      minVal = (val < minVal ? val : minVal);
      maxVal = (val > maxVal ? val : maxVal);
    }
    sum -= (minVal + maxVal);
    return sum / (n - 2);
  }
  
};

#endif /* ___TOURMAX__SMOOTHED__VALUES__H___ */

