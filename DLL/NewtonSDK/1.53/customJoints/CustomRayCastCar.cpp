// CustomRayCastCar.cpp: implementation of the CustomRayCastCar class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
#include "CustomRayCastCar.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CustomRayCastCar::CustomRayCastCar(int maxTireCount, NewtonBody* carBody)
	:NewtonCustomJoint(3 * maxTireCount, carBody, NULL)
{

	
}

CustomRayCastCar::~CustomRayCastCar()
{

}



void CustomRayCastCar::SubmitConstrainst()
{
}