
#ifndef ___TOURMAX__BUFFER__H___
#define ___TOURMAX__BUFFER__H___

class Buffer {
  
private:
  
  static const int BUFFER_SIZE = 64;
  
  int buffer[BUFFER_SIZE];
  int frontI;
  int backI;
  
  static inline int incI(int i) {
    return (i + 1) % BUFFER_SIZE;
  }
  
public:

  Buffer() {
    frontI = 0;
    backI = 0;
  }
  
  bool isEmpty() {
    return frontI == backI;
  }
  
  bool isFull() {
    return incI(frontI) == backI;
  }
  
  bool put(int v) {
    if (isFull()) {
      return false;
    }
    buffer[frontI] = v;
    frontI = incI(frontI);
    return true;
  }
  
  int getSize() {
    return (frontI - backI + BUFFER_SIZE) % BUFFER_SIZE;
  }
  
  bool peek_forward(int &v, int i) {
    if (i >= getSize()) {
      return false;
    } else {
      v = buffer[(backI + i) % BUFFER_SIZE];
      return true;
    }
  }
  
  bool peek(int &v) {
    return peek_forward(v, 0);
    /*
    if (isEmpty()) {
      return false;
    } else {
      v = buffer[backI];
      return true;
    }
    */
  }
  
  bool get(int &v) {
    if (isEmpty()) {
      return false;
    } else {
      v = buffer[backI];
      backI = incI(backI);
      return true;
    }
  }
  
  bool drop() {
    if (isEmpty()) {
      return false;
    } else {
      backI = incI(backI);
      return true;
    }
  }
  
  bool canAcceptNumElements(int num) {
    return getSize() + num < BUFFER_SIZE;
  }
  
};

#endif /* ___TOURMAX__BUFFER__H___ */

