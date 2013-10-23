#include <cstddef>
#include <stdafx.h>
#include "newton.h"
#include "dll.h"
#include "recast.h"

export double GmnCreateNull(double dWorld);
export double GmnCreateBox(double dWorld, double dx, double dy, double dz, double offset_x, double offset_y, double offset_z);
export double GmnCreateSphere(double dWorld, double radiusX, double radiusY, double radiusZ, double offset_x, double offset_y, double offset_z);
export double GmnCreateCone(double dWorld, double radius, double height, double offset_x, double offset_y, double offset_z);
export double GmnCreateCapsule(double dWorld, double radius, double height, double offset_x, double offset_y, double offset_z);
export double GmnCreateCylinder(double dWorld, double radius, double height, double offset_x, double offset_y, double offset_z);
export double GmnCreateChamferCylinder(double dWorld, double radius, double height, double offset_x, double offset_y, double offset_z);
