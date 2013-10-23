//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple demo list vector class with iterators
//********************************************************************

// CustomCorkScrew.h: interface for the CustomCorkScrew class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CUSTOMCORKSCREW_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_)
#define AFX_CUSTOMCORKSCREW_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_

#include "NewtonCustomJoint.h"

class CustomCorkScrew: public NewtonCustomJoint  
{
	public:
	CustomCorkScrew(const dVector& pivot, const dVector& pin, NewtonBody* child, NewtonBody* parent = NULL);
	virtual ~CustomCorkScrew();

	void EnableLimits(bool state);
	void SetLimis(dFloat minAngle, dFloat maxAngle);

	protected:
	virtual void SubmitConstrainst ();
	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;

	bool m_limitsOn;
	dFloat m_minDist;
	dFloat m_maxDist;

	bool m_angularmotorOn;
	dFloat m_angularDamp;
	dFloat m_angularAccel;
	
};

#endif // !defined(AFX_CUSTOMCORKSCREW_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_)
