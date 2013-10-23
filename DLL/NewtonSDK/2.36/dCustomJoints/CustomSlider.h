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



// CustomSlider.h: interface for the CustomSlider class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CUSTOMSLIDER_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_)
#define AFX_CUSTOMSLIDER_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_

#include "NewtonCustomJoint.h"

class JOINTLIBRARY_API CustomSlider: public NewtonCustomJoint  
{
	public:
	CustomSlider (const dMatrix& pinsAndPivoFrame, const NewtonBody* child, const NewtonBody* parent = NULL);
	virtual ~CustomSlider();

	void EnableLimits(bool state);
	void SetLimis(dFloat mindist, dFloat maxdist);

	bool JoinHitLimit () const ;
	
	protected:
	virtual void SubmitConstraints (dFloat timestep, int threadIndex);
	virtual void GetInfo (NewtonJointRecord* info) const;
	dMatrix m_localMatrix0;
	dMatrix m_localMatrix1;

	bool m_limitsOn;
	bool m_hitLimitOnLastUpdate;
	dFloat m_minDist;
	dFloat m_maxDist;
};

#endif // !defined(AFX_CUSTOMSLIDER_H__B631F556_468B_4331_B7D7_F85ECF3E9ADE__INCLUDED_)

