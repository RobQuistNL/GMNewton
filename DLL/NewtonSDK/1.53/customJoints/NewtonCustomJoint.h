//********************************************************************
// Newton Game dynamics 
// copyright 2000-2004
// By Julio Jerez
// VC: 6.0
// simple demo list vector class with iterators
//********************************************************************


// NewtonCustomJoint.h: interface for the NewtonCustomJoint class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_NEWTONCUSTOMJOINT_H__47435A34_E604_4336_A471_2179B7F60721__INCLUDED_)
#define AFX_NEWTONCUSTOMJOINT_H__47435A34_E604_4336_A471_2179B7F60721__INCLUDED_

#include "stdafx.h"
#include "Newton.h"
#include "dVector.h"
#include "dMatrix.h"


// this is the base class to implement custom joints, it is not a joint it just provide funtionality
// for the user to implement it own joints
class NewtonCustomJoint  
{
	public:
	NewtonCustomJoint(int maxDOF, NewtonBody* body0, NewtonBody* body1);

	protected:
	virtual ~NewtonCustomJoint();

	// the application need to implement this function for each derived joint. See examples for more detail
	virtual void SubmitConstrainst () = 0;

	void CalculateGlobalMatrix (const dMatrix& localMatrix0, const dMatrix& localMatrix1, dMatrix& matrix0, dMatrix& matrix1) const;
	void CalculateLocalMatrix (const dVector& pivod, const dVector& dir, dMatrix& localMatrix0, dMatrix& localMatrix1) const;

	int m_maxDox;
	NewtonBody* m_body0;
	NewtonBody* m_body1;
	NewtonJoint* m_joint;
	NewtonWorld* m_world;

	private:
	// this are the callback needed to have transparent c++ method interfaces 
	static void  Destructor (const NewtonJoint* me);	
	static void  SubmitConstrainst (const NewtonJoint* me);
};



#endif 
