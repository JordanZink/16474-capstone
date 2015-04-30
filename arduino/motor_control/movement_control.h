
#ifndef ___TOURMAX__MOVEMENT__CONTROL__H___
#define ___TOURMAX__MOVEMENT__CONTROL__H___

#import "vector.h"

typedef struct {
  Vector xyVector;
  float rotation;
} MovementControl;

static const MovementControl ZERO_MOVEMENT_CONTROL = {
  Vector(0.0f, 0.0f),
  0.0f
};

void mergeMovementControls(MovementControl &merged, MovementControl &a, MovementControl &b) {
  merged.xyVector = a.xyVector;
  merged.xyVector.add(b.xyVector);
  merged.rotation = a.rotation + b.rotation;
}

void getMovementControlInValidRange(MovementControl &movementControl) {
  if (movementControl.xyVector.getMagnitude() > 1.0f) {
    movementControl.xyVector.normalize();
  }
  if (movementControl.rotation < -1.0f) {
    movementControl.rotation = -1.0f;
  } else if (movementControl.rotation > 1.0f) {
    movementControl.rotation = 1.0f;
  }
}

#endif /* ___TOURMAX__MOVEMENT__CONTROL__H___ */
