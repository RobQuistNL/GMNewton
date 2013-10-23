#include "j_bs.h"
#include "joint_destroyer.h"
#include "dVector.h"

Joint_List mlist(10);

export double GmnConstraintCreateBall(double dWorld, double px, double py, double pz, double dChildBody, double dParentBody){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonBody* nChildBody = recastDoubleBody(dChildBody);
   NewtonBody* nParentBody;
   if(dParentBody==0){
      nParentBody = NULL;
   }else{
      nParentBody = recastDoubleBody(dParentBody);
   }

   dVector pivot (px,py,pz);

   NewtonJoint* nJoint = NewtonConstraintCreateBall(nWorld, &pivot.m_x, nChildBody, nParentBody);
   double jnt = recastJointDouble(nJoint);
   mlist.AddElement((int)jnt);
   return(recastJointDouble(nJoint));
}

export double GmnBallGetJointForce(double dJoint, double axis){
   NewtonJoint* nJoint = recastDoubleJoint(dJoint);
   float force[3];

   NewtonBallGetJointForce(nJoint, force);

   switch((int)axis){
      case 0:
         return((double) force[0]);
         break;
      case 1:
         return((double) force[1]);
         break;
      case 2:
         return((double) force[2]);
         break;
   }
   return (1);
}

export double GmnBallGetJointForceMag(double dJoint){
   NewtonJoint* nJoint = recastDoubleJoint(dJoint);
   float force[3];

   NewtonBallGetJointForce(nJoint, force);

   return ( sqrtf(force[0]*force[0] + force[1]*force[1] + force[2]*force[2]) );
}

export double GmnDestroyJoint(double dWorld, double dJoint){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonJoint* nJoint = recastDoubleJoint(dJoint);

   NewtonDestroyJoint(nWorld, nJoint);
   return (1);
}

export double GmnJointSetCollisionState(double dJoint, double state){
   NewtonJoint* nJoint = recastDoubleJoint(dJoint);

   NewtonJointSetCollisionState(nJoint, (int)state);
   return (1);
}

export double GmnJointsBreak(double world, double force){
    int size = mlist.CountElements();
    //Joint_List temp(size);
    int mag;
    int jnt;
    for(int i=0; i<size; i++){
        jnt = mlist.FindElementValue(i);
        mag = (int)GmnBallGetJointForceMag((double)jnt);
        if(mag>=force){
            //temp.AddElement((int)jnt);
            mlist.RemoveElement(i);
            GmnDestroyJoint(world, (double)jnt);
            i-=1;
            size-=1;
        }
    }
}
