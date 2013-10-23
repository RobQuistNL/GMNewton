#include "c_col.h"


export double GmnCreateNull(double dWorld){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonCollision* nCollision = NewtonCreateNull(nWorld);
   return( recastCollisionDouble(nCollision) );
}

export double GmnCreateBox(double dWorld, double dx, double dy, double dz, double offset_x, double offset_y, double offset_z){
    NewtonWorld* nWorld = recastDoubleWorld(dWorld);

/*
   dMatrix matrix(GetIdentityMatrix());//Create the offset matrix
   matrix.m_posit.m_x = offset_x;//Apply offsets to matrix
   matrix.m_posit.m_y = offset_y;
   matrix.m_posit.m_z = offset_z;
  */
    dFloat matrix[16];
    NewtonCollision* nCollision = NewtonCreateBox (nWorld,dx,dy,dz,NULL,matrix);//&matrix[0][0]);

    return( recastCollisionDouble(nCollision) );
}

export double GmnCreateSphere(double dWorld, double radiusX, double radiusY, double radiusZ, double offset_x, double offset_y, double offset_z){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);

   dMatrix matrix(GetIdentityMatrix());//Create the offset matrix
   matrix.m_posit.m_x = offset_x;//Apply offsets to matrix
   matrix.m_posit.m_y = offset_y;
   matrix.m_posit.m_z = offset_z;

   NewtonCollision* nCollision = NewtonCreateSphere (nWorld, radiusX, radiusY, radiusZ,0,&matrix[0][0]);
   return( recastCollisionDouble(nCollision) );
}

export double GmnCreateCone(double dWorld, double radius, double height, double offset_x, double offset_y, double offset_z){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);

   dMatrix matrix(GetIdentityMatrix());//Create the offset matrix
   matrix.m_posit.m_x = offset_x;//Apply offsets to matrix
   matrix.m_posit.m_y = offset_y;
   matrix.m_posit.m_z = offset_z;

   NewtonCollision* nCollision = NewtonCreateCone (nWorld, radius, height,0,&matrix[0][0]);
   return( recastCollisionDouble(nCollision) );
}

export double GmnCreateCapsule(double dWorld, double radius, double height, double offset_x, double offset_y, double offset_z){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);

   dMatrix matrix(GetIdentityMatrix());//Create the offset matrix
   matrix.m_posit.m_x = offset_x;//Apply offsets to matrix
   matrix.m_posit.m_y = offset_y;
   matrix.m_posit.m_z = offset_z;

   NewtonCollision* nCollision = NewtonCreateCapsule (nWorld, radius, height,0,&matrix[0][0]);
   return( recastCollisionDouble(nCollision) );
}

export double GmnCreateCylinder(double dWorld, double radius, double height, double offset_x, double offset_y, double offset_z){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);

   dMatrix matrix(GetIdentityMatrix());//Create the offset matrix
   matrix.m_posit.m_x = offset_x;//Apply offsets to matrix
   matrix.m_posit.m_y = offset_y;
   matrix.m_posit.m_z = offset_z;

   NewtonCollision* nCollision = NewtonCreateCylinder (nWorld, radius, height,0,&matrix[0][0]);
   return( recastCollisionDouble(nCollision) );
}

export double GmnCreateChamferCylinder(double dWorld, double radius, double height, double offset_x, double offset_y, double offset_z){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);

   dMatrix matrix(GetIdentityMatrix());//Create the offset matrix
   matrix.m_posit.m_x = offset_x;//Apply offsets to matrix
   matrix.m_posit.m_y = offset_y;
   matrix.m_posit.m_z = offset_z;

   NewtonCollision* nCollision = NewtonCreateChamferCylinder (nWorld, radius, height,0,&matrix[0][0]);
   return( recastCollisionDouble(nCollision) );
}


/*  Make a list for creating convex hulls....
export double GmnVertBufferAdd(double x, double y, double z){//Add a point to the vertex buffer
   vert_buffer.push_back((int) x);
   vert_buffer.push_back((int) y);
   vert_buffer.push_back((int) z);
   return(vert_buffer.size());
}

export double GmnVertBufferClear(){//Clear the vertex buffer
   vert_buffer.clear();
   return(0);
}
*/

//NewtonCreateConvexHull().  Just doing simple primitives for now...

//NewtonCreateConvexHullModifier().

//NewtonConvexHullModifierGetMatrix().

//NewtonConvexHullModifierSetMatrix().

//NewtonConvexCollisionSetUserID().

//NewtonConvexCollisionGetUserID().

//NewtonConvexCollisionCalculateVolume().

//NewtonConvexCollisionCalculateInertialMatrix().
