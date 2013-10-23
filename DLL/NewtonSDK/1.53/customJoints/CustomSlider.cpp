//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple demo list vector class with iterators
//********************************************************************

// CustomSlider.cpp: implementation of the CustomSlider class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
#include "CustomSlider.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define MIN_JOINT_PIN_LENGTH	50.0f

CustomSlider::CustomSlider(const dVector& pivot, const dVector& pin, NewtonBody* child, NewtonBody* parent)
	:NewtonCustomJoint(6, child, parent)
{
	m_limitsOn = false;
	m_minDist = -1.0f;
	m_maxDist =  1.0f;

	// calculate the two local matrix of the pivot point
	CalculateLocalMatrix (pivot, pin, m_localMatrix0, m_localMatrix1);
}

CustomSlider::~CustomSlider()
{
}


void CustomSlider::EnableLimits(bool state)
{
	m_limitsOn = state;
}

void CustomSlider::SetLimis(dFloat minDist, dFloat maxDist)
{
//	_ASSERTE (minDist < 0.0f);
//	_ASSERTE (maxDist > 0.0f);

	m_minDist = minDist;
	m_maxDist = maxDist;
}


void CustomSlider::SubmitConstrainst ()
{
	dFloat dist;
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

	// Restrict the movement on the pivot point along all tree orthonormal direction
	dVector p0 (matrix0.m_posit);
	dVector p1 (matrix1.m_posit + matrix1.m_front.Scale ((p0 - matrix1.m_posit) % matrix1.m_front));


	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix0.m_up[0]);
	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix0.m_right[0]);
	
	// get a point along the ping axis at some reasonable large distance from the pivot
	dVector q0 (p0 + matrix0.m_front.Scale(MIN_JOINT_PIN_LENGTH));
	dVector q1 (p1 + matrix1.m_front.Scale(MIN_JOINT_PIN_LENGTH));

	// two constraints row perpendicular to the pin
 	NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix0.m_up[0]);
	NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix0.m_right[0]);

	// get a point along the ping axis at some reasonable large distance from the pivot
	dVector r0 (p0 + matrix0.m_up.Scale(MIN_JOINT_PIN_LENGTH));
	dVector r1 (p1 + matrix1.m_up.Scale(MIN_JOINT_PIN_LENGTH));

	// one constraint row perpendicular to the pin
 	NewtonUserJointAddLinearRow (m_joint, &r0[0], &r1[0], &matrix0.m_right[0]);

	// if limit are enable ...
	if (m_limitsOn) {
		dist = (matrix0.m_posit - matrix1.m_posit) % matrix0.m_front;
		if (dist < m_minDist) {
			// get a point along the up vector and set a constraint  
			NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix0.m_posit[0], &matrix0.m_front[0]);
			// allow the object to return but not to kick going forward
			NewtonUserJointSetRowMinimumFriction (m_joint, 0.0f);
			
		} else if (dist > m_maxDist) {
			// get a point along the up vector and set a constraint  
			NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix0.m_posit[0], &matrix0.m_front[0]);
			// allow the object to return but not to kick going forward
			NewtonUserJointSetRowMaximumFriction (m_joint, 0.0f);
		}
	}
 }
