#include "recast.h"

//****BEGIN RECASTING FUNCTIONS****//

//double-world
double recastWorldDouble(NewtonWorld* nWorld){
   return( (double)reinterpret_cast<int>(nWorld) );
}

NewtonWorld* recastDoubleWorld(double dWorld){
   return( reinterpret_cast<NewtonWorld*>((int)dWorld) );
}

//double-body
double recastBodyDouble(NewtonBody* nBody){
   return( (double)reinterpret_cast<int>(nBody) );
}

NewtonBody* recastDoubleBody(double dBody){
   return( reinterpret_cast<NewtonBody*>((int)dBody) );
}

//double-collision
double recastCollisionDouble(NewtonCollision* nCollision){
   return( (double)reinterpret_cast<int>(nCollision) );
}

NewtonCollision* recastDoubleCollision(double dCollision){
   return( reinterpret_cast<NewtonCollision*>((int)dCollision) );
}

//double-joint
double recastJointDouble(NewtonJoint* nJoint){
   return( (double)reinterpret_cast<int>(nJoint) );
}

NewtonJoint* recastDoubleJoint(double dJoint){
   return( reinterpret_cast<NewtonJoint*>((int)dJoint) );
}

//****END RECASTING FUNCTIONS****//
