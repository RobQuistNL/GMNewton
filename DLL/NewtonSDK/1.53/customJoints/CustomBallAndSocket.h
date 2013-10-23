//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple demo list vector class with iterators
//********************************************************************


// CustomBallAndSocket.h: interface for the CustomBallAndSocket class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CUSTOMBALLANDSOCKET_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_)
#define AFX_CUSTOMBALLANDSOCKET_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_

#include "NewtonCustomJoint.h"

class CustomBallAndSocket: public NewtonCustomJoint  
{
	public:
	CustomBallAndSocket(const dVector& pivot, NewtonBody* child, NewtonBody* parent = NULL);
	virtual ~CustomBallAndSocket();


	protected:
	virtual void SubmitConstrainst ();

	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;
};

#endif // !defined(AFX_CUSTOMBALLANDSOCKET_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_)
