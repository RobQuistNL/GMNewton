//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple demo list vector class with iterators
//********************************************************************

// CustomBallAndSocket.cpp: implementation of the CustomBallAndSocket class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
#include "CustomBallAndSocket.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////



CustomBallAndSocket::CustomBallAndSocket(const dVector& pivot, NewtonBody* child, NewtonBody* parent)
	:NewtonCustomJoint(5, child, parent)
{
	dMatrix matrix;

	// use as primary pin vector the line that goes from the pivot to the origin of body zero
	// this eliminates some offset unwanted torques 
	NewtonBodyGetMatrix (m_body0, &matrix[0][0]);

	dVector pin (matrix.m_posit - pivot); 
	if ((pin % pin) < 1.0e-3f) {
		// the the pivot is already at the origin then any pin will do
		pin = matrix.m_front;
	}

	// calculate the two local matrix of the pivot point
	CalculateLocalMatrix (pivot, pin, m_localMatrix0, m_localMatrix1);
}

CustomBallAndSocket::~CustomBallAndSocket()
{
}


void CustomBallAndSocket::SubmitConstrainst ()
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

	// Restrict the movement on the pivot point along all tree orthonormal direction
	NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_front[0]);
	NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_up[0]);
	NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_right[0]);
}