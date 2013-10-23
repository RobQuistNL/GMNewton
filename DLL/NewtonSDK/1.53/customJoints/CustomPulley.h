//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple demo list vector class with iterators
//********************************************************************


// CustomPulley.h: interface for the CustomPulley class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CustomPulley_H__B631F556_468B_4331_B7D7_INCLUDED_)
#define AFX_CustomPulley_H__B631F556_468B_4331_B7D7_INCLUDED_

#include "NewtonCustomJoint.h"

// this joint is for used in conjustion with Slider of other linear joints
// is is usefull for stablishing synchronization between the rel;ative position 
// or relative linear velocity of two sliding bodies according to the law of pulley
// velErro = -(v0 + n * v1)
// where v0 and v1 are the linear velocity
// n is the number of turn on the pulley, in the case of this joint N coudl
// be a velue with fraction, as this joint is a generalization of teh pulley idea.
class CustomPulley: public NewtonCustomJoint  
{
	public:
	CustomPulley(dFloat pulleyRatio, 
			   const dVector& childPin, const dVector& parentPin, 
			   NewtonBody* parenPin, NewtonBody* parent);
	virtual ~CustomPulley();


	protected:
	virtual void SubmitConstrainst ();

	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;

	dFloat m_gearRatio;
};

#endif 
