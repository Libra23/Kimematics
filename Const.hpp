#ifndef _CONST_H
#define _CONST_H

#define NUM_JOINT 3
#define DOF 3

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886
#endif

enum Axis {
  X,
  Y,
  Z,
  XYZ
};

#endif