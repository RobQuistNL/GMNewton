#include <cstddef>
#include "newton.h"

//double-world
double recastWorldDouble(NewtonWorld* nWorld);
NewtonWorld* recastDoubleWorld(double dWorld);

//double-body
double recastBodyDouble(NewtonBody* nBody);
NewtonBody* recastDoubleBody(double dBody);

//double-collision
double recastCollisionDouble(NewtonCollision* nCollision);
NewtonCollision* recastDoubleCollision(double dCollision);

//double-joint
double recastJointDouble(NewtonJoint* nJoint);
NewtonJoint* recastDoubleJoint(double dJoint);
