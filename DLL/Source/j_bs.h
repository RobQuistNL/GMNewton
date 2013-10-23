#include <cstddef>
#include "newton.h"
#include "dll.h"
#include "recast.h"


export double GmnConstraintCreateBall(double dWorld, double px, double py,  double pz, double dChildBody, double dParentBody);
export double GmnBallGetJointForce(double dJoint, double axis);
export double GmnDestroyJoint(double dWorld, double dJoint);
export double GmnJointSetCollisionState(double dJoint, double state);
