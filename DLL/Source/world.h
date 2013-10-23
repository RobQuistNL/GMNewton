#include <cstddef>
#include "newton.h"
#include "dll.h"
#include "recast.h"
#include "materials.h"

export double GmnCreate();
export double GmnDestroy(NewtonWorld* nWorld);
export double GmnSetPlatformArchitecture(double dWorld, double mode);
export double GmnSetSolverModel(double dWorld, double model);
export double GmnSetFrictionModel(double dWorld, double model);
export double GmnUpdate(double dWorld, double timestep);
export double GmnSetMinimumFrameRate(double dWorld, double framerate);
export double GmnGetTimeStep(double dWorld);
export double GmnDestroyAllBodies(double dWorld);
export double GmnSetGravity(double xForce, double yForce, double zForce);
export double GmnSetWorldSize(double dWorld, double x1, double y1, double z1, double x2, double y2, double z2);
export double GmnWorldFreezeBody(double dWorld, double dBody);
export double GmnWorldUnfreezeBody(double dWorld, double dBody);
