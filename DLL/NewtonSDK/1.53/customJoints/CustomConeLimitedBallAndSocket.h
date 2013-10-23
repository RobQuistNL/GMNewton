//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple demo list vector class with iterators
//********************************************************************

// CustomConeLimitedBallAndSocket.h: interface for the CustomConeLimitedBallAndSocket class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CUSTOMCONELIMITEDBALLANDSOCKET_H__CAAB4770_1FD9_497D_9CE7_F2AD5C3EF678__INCLUDED_)
#define AFX_CUSTOMCONELIMITEDBALLANDSOCKET_H__CAAB4770_1FD9_497D_9CE7_F2AD5C3EF678__INCLUDED_

#include "CustomBallAndSocket.h"

class CustomConeLimitedBallAndSocket: public CustomBallAndSocket  
{
	public:
	CustomConeLimitedBallAndSocket(dFloat twistAngle, dFloat coneAngle, const dVector& coneDir, 
								   const dVector& pivot, NewtonBody* child, NewtonBody* parent);
	virtual ~CustomConeLimitedBallAndSocket();

	protected:
	virtual void SubmitConstrainst ();

	dFloat m_coneAngle; 
	dFloat m_twistAngle; 
	dFloat m_cosConeAngle; 
};

#endif 
