//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple demo list vector class with iterators
//********************************************************************

// CustomUniversal.cpp: implementation of the CustomUniversal class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
#include "CustomUniversal.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define MIN_JOINT_PIN_LENGTH	50.0f

CustomUniversal::CustomUniversal(const dVector& pivot, const dVector& pin0, const dVector& pin1, NewtonBody* child, NewtonBody* parent)
	:NewtonCustomJoint(6, child, parent)
{
	dMatrix matrix0;

	m_limit_0_On = true;
	m_limit_1_On = true;

	m_angularmotor_0_On = true;
	m_angularmotor_1_On = false; 


	m_minAngle_0 = -45.0f * 3.1416f / 180.0f;
	m_maxAngle_0 =  45.0f * 3.1416f / 180.0f;

	m_minAngle_1 = -45.0f * 3.1416f / 180.0f;
	m_maxAngle_1 =  45.0f * 3.1416f / 180.0f;


	m_angularDamp_0 = 0.5f;
	m_angularAccel_0 = -4.0f;

	m_angularDamp_1 = 0.3f;
	m_angularAccel_1 = -4.0f;



	// Get the global matrices of each rigid body.
	NewtonBodyGetMatrix(m_body0, &matrix0[0][0]);

	dMatrix matrix1 (GetIdentityMatrix());
	if (m_body1) {
		NewtonBodyGetMatrix(m_body1, &matrix1[0][0]);
	}

	// create a global matrix at the pivot point with front vector aligned to the pin vector
	dMatrix pinAndPivotMatrix;
	pinAndPivotMatrix.m_front = pin0.Scale (1.0f / dSqrt (pin0 % pin0));
	pinAndPivotMatrix.m_right = pin0 * pin1;
	pinAndPivotMatrix.m_right = pinAndPivotMatrix.m_right.Scale (1.0f / dSqrt (pinAndPivotMatrix.m_right % pinAndPivotMatrix.m_right));
	pinAndPivotMatrix.m_up = pinAndPivotMatrix.m_right * pinAndPivotMatrix.m_front;

	pinAndPivotMatrix.m_posit = pivot;

	// calculate the relative matrix of the pin and pivot on each body
 	m_localMatrix0 = pinAndPivotMatrix * matrix0.Inverse();
	m_localMatrix1 = pinAndPivotMatrix * matrix1.Inverse();
}

CustomUniversal::~CustomUniversal()
{
}


void CustomUniversal::EnableLimit_0(bool state)
{
	m_limit_0_On = state;
}

void CustomUniversal::EnableLimit_1(bool state)
{
	m_limit_1_On = state;
}

void CustomUniversal::EnableMotor_0(bool state)
{
	m_angularmotor_0_On = state;
}

void CustomUniversal::EnableMotor_1(bool state)
{
	m_angularmotor_1_On = state;
}


void CustomUniversal::SetLimis_0(dFloat minAngle, dFloat maxAngle)
{
	_ASSERTE (minAngle < 0.0f);
	_ASSERTE (maxAngle > 0.0f);

	m_minAngle_0 = minAngle;
	m_maxAngle_0 = maxAngle;
}

void CustomUniversal::SetLimis_1(dFloat minAngle, dFloat maxAngle)
{
	_ASSERTE (minAngle < 0.0f);
	_ASSERTE (maxAngle > 0.0f);

	m_minAngle_1 = minAngle;
	m_maxAngle_1 = maxAngle;
}


void CustomUniversal::SubmitConstrainst ()
{
	dFloat angle;
	dFloat sinAngle;
	dFloat cosAngle;
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

	// get the pin fixed to the first body
	const dVector& dir0 = matrix0.m_front;
	// get the pin fixed to the second body
	const dVector& dir1 = matrix1.m_up;

	// construct an orthogonal coordinate system with these two vectors
	dVector dir2 (dir0 * dir1);
	dVector dir3 (dir2 * dir0);
	dir3 = dir3.Scale (1.0f / dSqrt (dir3 % dir3));

	const dVector& p0 = matrix0.m_posit;
	const dVector& p1 = matrix1.m_posit;

	dVector q0 (p0 + dir3.Scale(MIN_JOINT_PIN_LENGTH));
	dVector q1 (p1 + dir1.Scale(MIN_JOINT_PIN_LENGTH));

	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &dir0[0]);
	NewtonUserJointSetRowStiffness (m_joint, 1.0f);

	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &dir1[0]);
	NewtonUserJointSetRowStiffness (m_joint, 1.0f);

	NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &dir2[0]);
	NewtonUserJointSetRowStiffness (m_joint, 1.0f);

	NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &dir0[0]);
	NewtonUserJointSetRowStiffness (m_joint, 1.0f);


	// if limit are enable ...
	if (m_limit_0_On) {
		sinAngle = (matrix0.m_front * matrix1.m_front) % matrix1.m_up;
		cosAngle = matrix0.m_front % matrix1.m_front;
		angle = dAtan2 (sinAngle, cosAngle);
 
		if (angle < m_minAngle_0) {
			dFloat relAngle;
			relAngle = angle - m_minAngle_0;

			// tell joint error will minimize the exceeded angle error
			NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix1.m_up[0]);

			// need high stiffeners here
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);

		} else if (angle > m_maxAngle_0) {
			dFloat relAngle;
			relAngle = angle - m_maxAngle_0;
			
			// tell joint error will minimize the exceeded angle error
			NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix1.m_up[0]);

			// need high stiffness here
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);
		}
	} else if (m_angularmotor_0_On) {
		dFloat relOmega;
		dFloat relAccel;
		dVector omega0 (0.0f, 0.0f, 0.0f);
		dVector omega1 (0.0f, 0.0f, 0.0f);

		// get relative angular velocity
		NewtonBodyGetOmega(m_body0, &omega0[0]);
		if (m_body1) {
			NewtonBodyGetOmega(m_body1, &omega1[0]);
		}

		// calculate the desired acceleration
		relOmega = (omega0 - omega1) % matrix1.m_up;
		relAccel = m_angularAccel_0 - m_angularDamp_0 * relOmega;

		// add and angular constraint row to that will set the relative acceleration to zero
		NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix1.m_up[0]);
		
		// override the joint acceleration.
		NewtonUserJointSetRowAcceleration (m_joint, relAccel);
	}

	// check is the joint limit are enable
	if (m_limit_1_On) {
		sinAngle = (matrix0.m_up * matrix1.m_up) % matrix0.m_front;
		cosAngle = matrix0.m_up % matrix1.m_up;
		angle = dAtan2 (sinAngle, cosAngle);
 
		if (angle < m_minAngle_1) {
			dFloat relAngle;
			relAngle = angle - m_minAngle_1;

			// tell joint error will minimize the exceeded angle error
			NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix0.m_front[0]);

			// need high stiffeners here
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);

		} else if (angle > m_maxAngle_1) {
			dFloat relAngle;
			relAngle = angle - m_maxAngle_1;
			
			// tell joint error will minimize the exceeded angle error
			NewtonUserJointAddAngularRow (m_joint, relAngle, &matrix0.m_front[0]);

			// need high stiffness here
			NewtonUserJointSetRowStiffness (m_joint, 1.0f);
		}

	// check is the joint limit motor is enable
	} else if (m_angularmotor_1_On) {

		dFloat relOmega;
		dFloat relAccel;
		dVector omega0 (0.0f, 0.0f, 0.0f);
		dVector omega1 (0.0f, 0.0f, 0.0f);

		// get relative angular velocity
		NewtonBodyGetOmega(m_body0, &omega0[0]);
		if (m_body1) {
			NewtonBodyGetOmega(m_body1, &omega1[0]);
		}

		// calculate the desired acceleration
		relOmega = (omega0 - omega1) % matrix0.m_front;
		relAccel = m_angularAccel_1 - m_angularDamp_1 * relOmega;

		// add and angular constraint row to that will set the relative acceleration to zero
		NewtonUserJointAddAngularRow (m_joint, 0.0f, &matrix0.m_front[0]);
		
		// override the joint acceleration.
		NewtonUserJointSetRowAcceleration (m_joint, relAccel);
	}
 }
