#include "t_col.h"

export double GmnCreateTreeCollision(double dWorld){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonCollision* nCollision = NewtonCreateTreeCollision(nWorld, NULL);
   return( recastCollisionDouble(nCollision) );
}

export double GmnTreeCollisionBeginBuild(double dCollision){
   NewtonCollision* nCollision = recastDoubleCollision(dCollision);
   NewtonTreeCollisionBeginBuild(nCollision);
   return(1);
}

export double GmnTreeCollisionAddFace(double dCollision, double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3, double id){
   NewtonCollision* nCollision = recastDoubleCollision(dCollision);

   float *verts = new float[9];
   verts[0]=x1;
   verts[1]=y1;
   verts[2]=z1;
   verts[3]=x2;
   verts[4]=y2;
   verts[5]=z2;
   verts[6]=x3;
   verts[7]=y3;
   verts[8]=z3;


   NewtonTreeCollisionAddFace (nCollision, 3, verts, sizeof(float)*3, (int) id);//Create collision shape
   //delete [] verts;//Do I need this?  It was commented out in old version...
   return (1);
}

export double GmnTreeCollisionEndBuild(double dCollision, double optimize){
   NewtonCollision* nCollision = recastDoubleCollision(dCollision);
   NewtonTreeCollisionEndBuild(nCollision, (int)optimize);
   return(1);
}

//NewtonTreeCollisionSerialize().

//NewtonCreateTreeCollisionFromSerialization().

//NewtonTreeCollisionGetFaceAtribute().

//NewtonTreeCollisionSetFaceAtribute().
