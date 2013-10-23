//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple demo list vector class with iterators
//********************************************************************

// CustomUniversal.h: interface for the CustomUniversal class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CUSTOMUNIVERSAL_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_)
#define AFX_CUSTOMUNIVERSAL_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_

#include "NewtonCustomJoint.h"

class CustomUniversal: public NewtonCustomJoint  
{
	public:
	CustomUniversal(const dVector& pivot, const dVector& pin0, const dVector& pin1, NewtonBody* child, NewtonBody* parent = NULL);
	virtual ~CustomUniversal();

	void EnableLimit_0(bool state);
	void EnableLimit_1(bool state);
	void SetLimis_0(dFloat minAngle, dFloat maxAngle);
	void SetLimis_1(dFloat minAngle, dFloat maxAngle);

	void EnableMotor_0(bool state);
	void EnableMotor_1(bool state);


	protected:
	virtual void SubmitConstrainst ();
	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;

	bool m_limit_0_On;
	bool m_limit_1_On;
	dFloat m_minAngle_0;
	dFloat m_maxAngle_0;
	dFloat m_minAngle_1;
	dFloat m_maxAngle_1;


	bool m_angularmotor_0_On;
	bool m_angularmotor_1_On;
	dFloat m_angularDamp_0;
	dFloat m_angularAccel_0;

	dFloat m_angularDamp_1;
	dFloat m_angularAccel_1;
	
};

#endif // !defined(AFX_CUSTOMUNIVERSAL_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_)
