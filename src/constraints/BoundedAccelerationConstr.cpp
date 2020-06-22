#include "BoundedAccelerationConstr.h"



BoundedAccelerationConstr::BoundedAccelerationConstr(unsigned int rIndex,
                                                     double maxAccTransXY,
                                                     double maxAccRotZ)
: mc_solver::GenInequalityConstraintRobot(rIndex),
  maxAccTransXY_(maxAccTransXY), maxAccRotZ_(maxAccRotZ)
  {
    A_.resize(3, 6);
    A_.setZero();
    A_(0, 2) = 1.0;
    A_(1, 3) = 1.0;
    A_(2, 4) = 1.0;

    L_.resize(3);
    L_.setZero();
    L_(0) = -maxAccRotZ;
    L_(1) = -maxAccTransXY;
    L_(2) = -maxAccTransXY;

    U_ = -L_.eval();
  }
