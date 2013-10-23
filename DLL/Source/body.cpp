#include "body.h"
#include "dVector.h"

void  PhysicsApplyForceAndTorque (const NewtonBody* body)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	NewtonBodyGetMassMatrix (body, &mass, &Ixx, &Iyy, &Izz);
	dVector force (0.0f, 0.0f, -mass * 9.8f*16);
	NewtonBodySetForce (body, &force.m_x);
}



export double GmnCreateBody(double dWorld, double dCollision){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonCollision* nCollision = recastDoubleCollision(dCollision);

   NewtonBody* nBody = NewtonCreateBody(nWorld, nCollision, 0.0f);

   NewtonBodySetForceAndTorqueCallback (nBody, PhysicsApplyForceAndTorque);

   return( recastBodyDouble(nBody) );
}

export double GmnDestroyBody(double dWorld, double dBody){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonBody* nBody = recastDoubleBody(dBody);

   NewtonDestroyBody(nWorld, nBody);
   return (1);
}

//NewtonBodySetUserData().

//NewtonBodyGetUserData().

export double GmnBodyGetWorld(double dBody){
   NewtonBody* nBody = recastDoubleBody(dBody);
   NewtonWorld* nWorld = NewtonBodyGetWorld(nBody);

   return( recastWorldDouble(nWorld) );
}

//NewtonBodySetTransformCallback().

//NewtonBodySetAutoactiveCallback().NewtonBodySetUserData

//***Need this for gravity*** NewtonBodySetForceAndTorqueCallback().

//NewtonBodyGetForceAndTorqueCallback().

//NewtonBodySetDestructorCallback().

export double GmnBodySetMassMatrix(double dBody, double mass, double Ixx, double Iyy, double Izz){
   NewtonBody* nBody = recastDoubleBody(dBody);
   //Ixx, Iyy, Izz maybe 150 for starters?   I don't fully understand the inertial values yet...
   NewtonBodySetMassMatrix(nBody,(int)mass, (int)Ixx, (int)Iyy, (int)Izz);
   return (1);
}

//NewtonBodyGetMassMatrix().

//NewtonBodyGetInvMass().

//***Priority!*** NewtonBodySetMatrix().

//NewtonBodySetMatrixRecursive().

/*************************
BEGIN: A few extra simple interfacing functions
*************************/

export double GmnBodyGetPosition(double dBody, double axis){
   NewtonBody* nBody = recastDoubleBody(dBody);

   dMatrix matrix;
   NewtonBodyGetMatrix(nBody, &matrix[0][0]);

   switch((int) axis){
      case 0:
         return((double) matrix.m_posit.m_x);
         break;
      case 1:
         return((double) matrix.m_posit.m_y);
         break;
      case 2:
         return((double) matrix.m_posit.m_z);
         break;
      default:
         return(0);
         break;
   }
}

export double GmnBodyGetRotation(double dBody, double axis){
   NewtonBody* nBody = recastDoubleBody(dBody);

   dFloat angles[3];
   dFloat matrix[16];
   NewtonBodyGetMatrix(nBody, matrix);

   NewtonGetEulerAngle(matrix,angles);
   if((axis<=2)&&(axis>=0)){
      return((double) -angles[(int)axis]);
   }else{
      return 0;
   }
}

export double GmnBodySetPosition(double dBody, double x, double y, double z){
   NewtonBody* nBody = recastDoubleBody(dBody);

   dMatrix matrix;
   NewtonBodyGetMatrix(nBody, &matrix[0][0]);
   matrix.m_posit.m_x=x;
   matrix.m_posit.m_y=y;
   matrix.m_posit.m_z=z;
   NewtonBodySetMatrix (nBody, &matrix[0][0]);
   return(1);
}

export double GmnBodySetRotation(double dBody, double xrot, double yrot, double zrot){
   NewtonBody* nBody = recastDoubleBody(dBody);

   dMatrix matrix, newmatrix;
   NewtonBodyGetMatrix(nBody, &matrix[0][0]);
   newmatrix=GetIdentityMatrix();
   for(int asd=0;asd<4;asd++){
      newmatrix[3][asd]=matrix[3][asd];
   }

   //set the transformation matrix
   matrix = newmatrix * dgPitchMatrix(xrot * 3.1416f / 180.0f);
   matrix = matrix * dgRollMatrix(zrot * 3.1416f / 180.0f);
   matrix = matrix * dgYawMatrix(yrot * 3.1416f / 180.0f);
   NewtonBodySetMatrix (nBody, &matrix[0][0]);
   return(1);
}

/*************************
END: A few extra simple interfacing functions
*************************/
