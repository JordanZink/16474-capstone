
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

#endif /* ___TOURMAX__MOVEMENT__CONTROL__H___ */
