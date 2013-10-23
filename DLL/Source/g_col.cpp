#include "g_col.h"

export double GmnReleaseCollision(double dWorld, double dCollision){
   NewtonWorld* nWorld = recastDoubleWorld(dWorld);
   NewtonCollision* nCollision = recastDoubleCollision(dCollision);

   NewtonReleaseCollision(nWorld, nCollision);
   return (1);
}
