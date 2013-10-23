#include <cstddef>
#include <stdafx.h>
#include "newton.h"
#include "dll.h"
#include "recast.h"

export double GmnCreateTreeCollision(double dWorld);
export double GmnTreeCollisionBeginBuild(double dCollision);
export double GmnTreeCollisionAddFace(double dCollision, double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3, double id);
export double GmnTreeCollisionEndBuild(double dCollision, double optimize);
