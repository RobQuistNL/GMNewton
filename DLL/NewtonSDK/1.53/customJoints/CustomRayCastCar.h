// CustomRayCastCar.h: interface for the CustomRayCastCar class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CUSTOMRAYCASTCAR_H__7EF45551_AEA8_4ABB_B344_ED947046A874__INCLUDED_)
#define AFX_CUSTOMRAYCASTCAR_H__7EF45551_AEA8_4ABB_B344_ED947046A874__INCLUDED_


#include "NewtonCustomJoint.h"

class CustomRayCastCar : public NewtonCustomJoint  
{
	public:
	CustomRayCastCar(int maxTireCount, NewtonBody* carBody);
	virtual ~CustomRayCastCar();


	protected:
	virtual void SubmitConstrainst ();

};



#endif 
