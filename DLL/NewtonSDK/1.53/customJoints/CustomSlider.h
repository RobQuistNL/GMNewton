//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple demo list vector class with iterators
//********************************************************************

// CustomSlider.h: interface for the CustomSlider class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CUSTOMSLIDER_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_)
#define AFX_CUSTOMSLIDER_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_

#include "NewtonCustomJoint.h"

class CustomSlider: public NewtonCustomJoint  
{
	public:
	CustomSlider(const dVector& pivot, const dVector& pin, NewtonBody* child, NewtonBody* parent = NULL);
	virtual ~CustomSlider();

	void EnableLimits(bool state);
	void SetLimis(dFloat mindist, dFloat maxdist);

	protected:
	virtual void SubmitConstrainst ();
	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;

	bool m_limitsOn;
	dFloat m_minDist;
	dFloat m_maxDist;
};

#endif // !defined(AFX_CUSTOMSLIDER_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_)
