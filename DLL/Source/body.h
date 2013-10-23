#ifndef _BODY_H_
#define _BODY_H_

#include <cstddef>
#include "dVector.h"
#include "dMatrix.h"
#include "dMathDefines.h"
#include "dStdAfxMath.h"
#include "dll.h"
#include "recast.h"

export double GmnCreateBody(double dWorld, double dCollision);
export double GmnDestroyBody(double dWorld, double dBody);
export double GmnBodyGetWorld(double dBody);
export double GmnBodySetMassMatrix(double dBody, double mass, double Ixx, double Iyy, double Izz);

//BEGIN: Extra simple interface functions
export double GmnBodyGetPosition(double dBody, double axis);
export double GmnBodyGetRotation(double dBody, double axis);
export double GmnBodySetPosition(double dBody, double x, double y, double z);
export double GmnBodySetRotation(double dBody, double xrot, double yrot, double zrot);
//END: Extra simple interface functions




struct BODYDAT{
    BODYDAT();
    int linked;
    int object_id;
    int general_id;
};
BODYDAT::BODYDAT(){
    linked=0;
    object_id=0;
    general_id=0;
}

#endif /* _BODY_H_ */
