/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// CustomBallAndSocket.h: interface for the CustomBallAndSocket class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CUSTOMBALLANDSOCKET_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_)
#define AFX_CUSTOMBALLANDSOCKET_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_

#include "NewtonCustomJoint.h"
class JOINTLIBRARY_API CustomBallAndSocket: public NewtonCustomJoint  
{
	public:
	CustomBallAndSocket(const dMatrix& pinsAndPivoFrame, const NewtonBody* child, const NewtonBody* parent = NULL);
	virtual ~CustomBallAndSocket();

	protected:
	virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	virtual void GetInfo (NewtonJointRecord* info) const;

	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;
};


// similar to the ball and socked 
// plus it has the ability to set joint linmits
class JOINTLIBRARY_API CustomLimitBallAndSocket: public CustomBallAndSocket  
{
	public:
	CustomLimitBallAndSocket(const dMatrix& pinsAndPivoFrame, const NewtonBody* child, const NewtonBody* parent = NULL);
	virtual ~CustomLimitBallAndSocket();

//	void SetChildOrientation (const dMatrix& matrix);

	void SetConeAngle (dFloat angle);
	void SetTwistAngle (dFloat minAngle, dFloat maxAngle);

	protected:
	virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	virtual void GetInfo (NewtonJointRecord* info) const;

	dFloat m_minTwistAngle;
	dFloat m_maxTwistAngle;

	dFloat m_coneAngleCos;
	dFloat m_coneAngleSin;
	dFloat m_coneAngleHalfCos;
	dFloat m_coneAngleHalfSin;
};





#endif // !defined(AFX_CUSTOMBALLANDSOCKET_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_)

