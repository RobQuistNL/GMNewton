//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple demo list vector class with iterators
//********************************************************************


// CustomPoweredBallAndSocket.h: interface for the CustomPoweredBallAndSocket class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CustomPoweredBallAndSocket_H__468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_)
#define AFX_CustomPoweredBallAndSocket_H__468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_

#include "NewtonCustomJoint.h"

// this joint is very similar to the ball and sucked plus it had the property that
// the its three rotationals degree of freedoms can by controlled by the applycation
// this joint could be used for controlling the limbs of organics articulated bodies
// like creatures and humanoids 
class CustomPoweredBallAndSocket: public NewtonCustomJoint  
{
	public:
	CustomPoweredBallAndSocket(const dVector& pivot, NewtonBody* child, NewtonBody* parent = NULL);
	virtual ~CustomPoweredBallAndSocket();

	void SetChildOrientation (const dMatrix& matrix);

	protected:
	virtual void SubmitConstrainst ();

	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;

	dMatrix m_targetMatrix;
};

#endif // !defined(AFX_CustomPoweredBallAndSocket_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_)
