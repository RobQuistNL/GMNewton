//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple demo list vector class with iterators
//********************************************************************

// CustomPoweredBallAndSocket.cpp: implementation of the CustomPoweredBallAndSocket class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
#include "CustomPoweredBallAndSocket.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////



CustomPoweredBallAndSocket::CustomPoweredBallAndSocket(const dVector& pivot, NewtonBody* child, NewtonBody* parent)
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

	// set the target matrix to point to the child matrix
	m_targetMatrix = m_localMatrix0;
}

CustomPoweredBallAndSocket::~CustomPoweredBallAndSocket()
{
}


void CustomPoweredBallAndSocket::SetChildOrientation (const dMatrix& matrix)
{

}

void CustomPoweredBallAndSocket::SubmitConstrainst ()
{
	// this is still work in progress.
	// if anybody is integeted on this joint let us know
/*
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian's direction vectors, in global space. 
	CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

	// Restrict the movement on the pivot point along all tree orthonormal direction
	NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_front[0]);
	NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_up[0]);
	NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_right[0]);


	// now we need to determine the direction of the DOG to align m_localMatrix0 with m_targetMatrix
	// step 1
	// first will will take m_targetMatrix to global space
	dMatrix dommy;
	dMatrix localMatrix;
	CalculateGlobalMatrix (m_targetMatrix, m_targetMatrix, localMatrix, dommy);

    // now we will calculate the rotation angle to align between the front axis
	// of localMatrix and matrix0;
	// this is done by taking the cross product between the tow axis
	// if the magnitude of the dot product is sufficiently large, these means that 
    // that direction of the cross product define a plane about and and angular rotation
	// of matrix0.m_front will align up with localMatrix.m_front.
	// angle is the arch sign of the of the magnitude of the cross product.

	dFloat mag2;
	dFloat angle;
//	dFloat accel;
//	dFloat time;
//	time = NewtonGetTimeStep (m_world);

	dVector axis (localMatrix.m_front * matrix0.m_front);
	mag2 = axis % axis;
	if (mag2 < 1.0e-4f) {
		// we have a non zero error, get axis
		axis = axis.Scale (1.0f / sqrtf (axis % axis));
		angle = asinf (mag2);

		// the asinf will only report and angle between 0 and 90 degree
		// we need to check if the angle is larger than 90, by checking 
		// if the projection of localMatrix.m_front over matrix0.m_front
		// is negative
		if ((localMatrix.m_front % matrix0.m_front) < 0.0f) {
			angle = 3.1416f - angle;
		}

		// we can not really do a extremes rotation so will will clam teh anglu to not be more
		// than 30 degree in each step
		angle = min (angle, 30.0f * (3.1416f / 180.0f));

		// now we are ready to add an angular row
		NewtonUserJointAddAngularRow (m_joint, -angle, &axis[0]);

		// we need to add another rotation

		// this work is incomplete		


	}
	// this work is incomplete		
*/	
}