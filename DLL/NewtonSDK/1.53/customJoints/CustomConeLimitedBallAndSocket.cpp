//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple demo list vector class with iterators
//********************************************************************


// CustomConeLimitedBallAndSocket.cpp: implementation of the CustomConeLimitedBallAndSocket class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
#include "CustomConeLimitedBallAndSocket.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


#define MIN_JOINT_PIN_LENGTH	50.0f

CustomConeLimitedBallAndSocket::CustomConeLimitedBallAndSocket(
   dFloat twistAngle, 
   dFloat coneAngle, 
   const dVector& coneDir, 
   const dVector& pivot, 
   NewtonBody* child, 
   NewtonBody* parent)
	:CustomBallAndSocket(pivot, child, parent)
{
	m_coneAngle = coneAngle; 
	m_coneAngle = twistAngle;

	m_cosConeAngle = dCos (m_coneAngle);

	// Recalculate local matrices so that the front vector align with the cone pin
	CalculateLocalMatrix (pivot, coneDir, m_localMatrix0, m_localMatrix1);
}

CustomConeLimitedBallAndSocket::~CustomConeLimitedBallAndSocket()
{

}


void CustomConeLimitedBallAndSocket::SubmitConstrainst ()
{
	dFloat coneCos;
	dMatrix matrix0;
	dMatrix matrix1;

	// add the tree rows to keep the pivot in place
	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

	// Restrict the movement on the pivot point along all tree orthonormal direction
	NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_front[0]);
	NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_up[0]);
	NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_right[0]);

	// ///////////////////////////////////////////////////////////////////
	//
	// add a row to keep the child body inside the cone limit
	//
	// The child is inside the cone if the dCos of the angle between the pin and 
	coneCos = matrix0.m_front % matrix1.m_front;
	if (coneCos < m_cosConeAngle) {

		// the child body has violated the cone limit we need to stop it from keep moving 
		// for that we are going to pick a point along the the child body front vector
 		dVector p0 (matrix0.m_posit + matrix0.m_front.Scale(MIN_JOINT_PIN_LENGTH));
		dVector p1 (matrix1.m_posit + matrix1.m_front.Scale(MIN_JOINT_PIN_LENGTH));

		// get a vectors perpendicular to the plane of motion
		dVector lateralDir (matrix0.m_front * matrix1.m_front);

		// note this could fail if the angle between matrix0.m_front and matrix1.m_front is 90 degree
		dVector unitLateralDir = lateralDir.Scale (1.0f / dSqrt (lateralDir % lateralDir));

		// now we will add a constraint row along the lateral direction, 
		// this will add stability as it will prevent the child body from moving sideways
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p0[0], &unitLateralDir[0]);

		// calculate the unit vector tangent to the trajectory
		dVector tangentDir (unitLateralDir * matrix0.m_front);

		p1 = p0 + (p1 - p0).Scale (0.3f);
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &tangentDir[0]);

		//we need to allow the body to mo in opposite direction to the penetration
		//that can be done by setting the min friction to zero
		NewtonUserJointSetRowMinimumFriction (m_joint, 0.0f);
	}
}
