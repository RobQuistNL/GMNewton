//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple demo list vector class with iterators
//********************************************************************

// NewtonCustomJoint.cpp: implementation of the NewtonCustomJoint class.
//
//////////////////////////////////////////////////////////////////////
#include <stdafx.h>
#include "NewtonCustomJoint.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

NewtonCustomJoint::NewtonCustomJoint (int maxDOF, NewtonBody* body0, NewtonBody* body1)
{
	m_joint = NULL;
	m_body0 = body0;
	m_body1 = body1;
	m_maxDox = maxDOF;
	m_world	= NewtonBodyGetWorld (body0);
	m_joint = NewtonConstraintCreateUserJoint (m_world, maxDOF, SubmitConstrainst, m_body0, m_body1); 

	NewtonJointSetUserData (m_joint, this);
	NewtonJointSetDestructor (m_joint, Destructor);
}

NewtonCustomJoint::~NewtonCustomJoint()
{
	//_ASSERTE (m_joint);
	
	//if the joint has user data it means the application is destroy the joint
	if (NewtonJointGetUserData (m_joint)) {
		// set the joint call to NULL to prevent infinite recursion 
		NewtonJointSetDestructor (m_joint, NULL);  

		// destroy this joint
		NewtonDestroyJoint(m_world, m_joint);
	}
}


void  NewtonCustomJoint::Destructor (const NewtonJoint* me)
{
	NewtonCustomJoint* joint;  

	// get the pointer to the joint class
	joint = (NewtonCustomJoint*) NewtonJointGetUserData (me);  

	// set the joint call to NULL to prevent infinite recursion
	NewtonJointSetDestructor (me, NULL);  
	NewtonJointSetUserData (me, NULL);  

	// delete the joint class
	delete joint;
}


void  NewtonCustomJoint::SubmitConstrainst (const NewtonJoint* me)
{
	NewtonCustomJoint* joint;  

	// get the pointer to the joint class
	joint = (NewtonCustomJoint*) NewtonJointGetUserData (me);  
	joint->SubmitConstrainst();
}



void NewtonCustomJoint::CalculateLocalMatrix (const dVector& pivot, const dVector& dir, dMatrix& localMatrix0, dMatrix& localMatrix1) const
{
	dMatrix matrix0;

	// Get the global matrices of each rigid body.
	NewtonBodyGetMatrix(m_body0, &matrix0[0][0]);

	dMatrix matrix1 (GetIdentityMatrix());
	if (m_body1) {
		NewtonBodyGetMatrix(m_body1, &matrix1[0][0]);
	}

	// create a global matrix at the pivot point with front vector aligned to the pin vector
	dMatrix pinAndPivotMatrix (dgGrammSchmidt(dir));
	pinAndPivotMatrix.m_posit = pivot;
	pinAndPivotMatrix.m_posit.m_w = 1.0f;

	// calculate the relative matrix of the pin and pivot on each body
 	localMatrix0 = pinAndPivotMatrix * matrix0.Inverse();
	localMatrix1 = pinAndPivotMatrix * matrix1.Inverse();
}


void NewtonCustomJoint::CalculateGlobalMatrix (const dMatrix& localMatrix0, const dMatrix& localMatrix1, dMatrix& matrix0, dMatrix& matrix1) const
{
	dMatrix body0Matrix;
	// Get the global matrices of each rigid body.
	NewtonBodyGetMatrix(m_body0, &body0Matrix[0][0]);

	dMatrix body1Matrix (GetIdentityMatrix());
	if (m_body1) {
		NewtonBodyGetMatrix(m_body1, &body1Matrix[0][0]);
	}
	matrix0 = localMatrix0 * body0Matrix;
	matrix1 = localMatrix1 * body1Matrix;
}
