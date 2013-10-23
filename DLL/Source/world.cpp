#include "world.h"

void* physicsAlloc(int size)
{

    return new char[size];

}

void physicsFree(void *pointer, int size)
{

	char *tmp;
	tmp = (char*) pointer;
	delete[] tmp;

}

export double GmnCreate(){  // Create an instance of the Newton world.
   double dWorld;
   NewtonWorld* nWorld = NewtonCreate(physicsAlloc,physicsFree);
   return(recastWorldDouble(nWorld));

}

export double GmnSetGravity(double xForce, double yForce, double zForce){
   return 0; //Not yet implementedt -
}

export double GmnDestroy(NewtonWorld* nWorld){  // Destroy an instance of the Newton world.
   NewtonDestroy(nWorld);
   return (1);
}

export double GmnSetPlatformArchitecture(double dWorld, double mode){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonSetPlatformArchitecture(nWorld, (int)mode);
   return( 1 );
}

export double GmnSetSolverModel(double dWorld, double model){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonSetSolverModel(nWorld, (int)model);
   return(1);
}

export double GmnSetFrictionModel(double dWorld, double model){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonSetFrictionModel(nWorld, (int)model);
   return(1);
}

//have not yet added declarations in world.h from here down

export double GmnUpdate(double dWorld, double timestep){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);

   ClearCollisionRecord();

   NewtonUpdate(nWorld, timestep);
   return (1);
}

export double GmnSetMinimumFrameRate(double dWorld, double framerate){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonSetMinimumFrameRate(nWorld, framerate);
   return (1);
}

export double GmnGetTimeStep(double dWorld){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   return( (double)NewtonGetTimeStep(nWorld) );
}

export double GmnDestroyAllBodies(double dWorld){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   return (1);
}

export double GmnSetWorldSize(double dWorld, double x1, double y1, double z1, double x2, double y2, double z2){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   dFloat p1[]={x1,y1,z1};
   dFloat p2[]={x2,y2,z2};
   NewtonSetWorldSize(nWorld, p1, p2);
   return (1);
}

//NewtonSetBodyLeaveWorldEvent.  Not yet implementing callbacks...

export double GmnWorldFreezeBody(double dWorld, double dBody){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonBody* nBody = recastDoubleBody(dBody);
   NewtonWorldFreezeBody(nWorld, nBody);
   return (1);
}

export double GmnWorldUnfreezeBody(double dWorld, double dBody){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonBody* nBody = recastDoubleBody(dBody);
   NewtonWorldUnfreezeBody(nWorld, nBody);
   return (1);
}

//NewtonWorldForEachBodyDo().

//NewtonWorldForEachBodyInAABBDo().

//NewtonGetVersion() not implimented or disabled?

//NewtonWorldSetUserData()

//NewtonWorldRayCast().
