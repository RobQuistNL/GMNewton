#include <cstddef>
#include <stdafx.h>
#include "newton.h"


CustomRigid::CustomRigid(const dVector& pivot, NewtonBody* child, NewtonBody* parent)
   :NewtonCustomJoint(6, child, parent)
{
   dVector pin;
   dMatrix matrix;

   // use front vector of child matrix as pin
   NewtonBodyGetMatrix (child, &matrix[0][0]);


   // calculate the two local matrix of the pivot point
   CalculateLocalMatrix (pivot, matrix.m_front, m_localMatrix0, m_localMatrix1);
}

void CustomRigid::SubmitConstrainst ()
{
   dMatrix matrix0;
   dMatrix matrix1;

   // calculate the position of the pivot point and the Jacobian direction vectors, in global space.
   CalculateGlobalMatrix (m_localMatrix0, m_localMatrix1, matrix0, matrix1);

   // Restrict the movement on the pivot point along all tree orthonormal direction
   NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_front[0]);
   NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_up[0]);
   NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix1.m_posit[0], &matrix0.m_right[0]);

   // get a point along the pin axis at some reasonable large distance from the pivot
   dVector q0 (matrix0.m_posit + matrix0.m_front.Scale(MIN_JOINT_PIN_LENGTH));
   dVector q1 (matrix1.m_posit + matrix1.m_front.Scale(MIN_JOINT_PIN_LENGTH));

   // two constraints row perpendicular to the pin vector
   NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix0.m_up[0]);
   NewtonUserJointAddLinearRow (m_joint, &q0[0], &q1[0], &matrix0.m_right[0]);

   // now get the this ankle point
   dVector r0 (matrix0.m_posit + matrix0.m_up.Scale(MIN_JOINT_PIN_LENGTH));
   dVector r1 (matrix1.m_posit + matrix1.m_up.Scale(MIN_JOINT_PIN_LENGTH));
   NewtonUserJointAddLinearRow (m_joint, &r0[0], &r1[0], &matrix0.m_right[0]);
}
