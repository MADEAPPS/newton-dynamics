/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndJointFixDebrisLink.h"

#define D_DEBRIS_DAMPER			dFloat32 (50.0f)
#define D_DEBRIS_SPRING			dFloat32 (2000.0f)
#define D_DEBRIS_REGULARIZER	dFloat32 (0.001f)

ndJointFixDebrisLink::ndJointFixDebrisLink(ndBodyKinematic* const body0, ndBodyKinematic* const body1)
	:ndJointBilateralConstraint(6, body0, body1, dGetIdentityMatrix())
{
	dAssert(body0->GetInvMass() > dFloat32(0.0f));
	dMatrix dummy;

	//dVector posit0(body0->GetMatrix().TransformVector(body0->GetCentreOfMass()));
	//dVector posit1(body1->GetMatrix().TransformVector(body1->GetCentreOfMass()));
	//
	//dVector dist(posit1 - posit0);
	//m_distance = dSqrt(dist.DotProduct(dist).GetScalar());
	//dAssert(m_distance > dFloat32(1.0e-3f));
	//
	//dMatrix matrix0(dist);
	//matrix0.m_posit = posit0;
	//CalculateLocalMatrix(matrix0, m_localMatrix0, dummy);
	//
	//dMatrix matrix1(matrix0);
	//matrix1.m_posit = body1->GetMatrix().TransformVector(body1->GetCentreOfMass());
	//CalculateLocalMatrix(matrix1, dummy, m_localMatrix1);

	dVector posit0(body0->GetMatrix().TransformVector(body0->GetCentreOfMass()));
	dVector posit1(body1->GetMatrix().TransformVector(body1->GetCentreOfMass()));
	
	dVector pivot((posit1 + posit0).Scale (dFloat32 (0.5f)));
	//m_distance = dSqrt(dist.DotProduct(dist).GetScalar());

	dVector dir(posit1 - posit0);
	dAssert(dir.DotProduct(dir).GetScalar() > dFloat32(1.0e-3f));
	
	dMatrix matrix(dir);
	matrix.m_posit = pivot;
	matrix.m_posit.m_w = 1.0f;
	CalculateLocalMatrix(matrix, m_localMatrix0, m_localMatrix1);
	
	SetSolverModel(m_secundaryCloseLoop);
}

ndJointFixDebrisLink::~ndJointFixDebrisLink()
{
}

void ndJointFixDebrisLink::JacobianDerivative(ndConstraintDescritor& desc)
{
	dMatrix matrix0;
	dMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	//const dVector step (matrix0.UnrotateVector (matrix1.m_posit - matrix0.m_posit));
	//const dVector veloc0(m_body0->GetVelocity());
	//const dVector veloc1(m_body1->GetVelocityAtPoint(matrix0.m_posit));
	//const dVector relVeloc(veloc1 - veloc0);
	//
	//dVector distanceOffset(dVector::m_zero);
	//distanceOffset.m_x = m_distance;
	//for (dInt32 i = 0; i < 3; i++)
	//{
	//	const dVector& dir = matrix0[i];
	//	
	//	AddLinearRowJacobian(desc, matrix0.m_posit, matrix0.m_posit, dir);
	//	const dFloat32 x = step[i] - distanceOffset[i];
	//	const dFloat32 v = relVeloc.DotProduct(dir).GetScalar();
	//	const dFloat32 a = -CalculateSpringDamperAcceleration(desc.m_timestep, D_DEBRIS_SPRING, x, D_DEBRIS_DAMPER, v);
	//
	//	SetMotorAcceleration(desc, a);
	//	SetDiagonalRegularizer(desc, D_DEBRIS_REGULARIZER);
	//
	//	ndJacobian& jacobian0 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
	//	ndJacobian& jacobian1 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;
	//	jacobian0.m_linear = dir;
	//	jacobian0.m_angular = dVector::m_zero;
	//	jacobian1.m_linear = dir.Scale(dFloat32(-1.0f));
	//	jacobian1.m_angular = dVector::m_zero;
	//}

	for (dInt32 i = 0; i < 3; i++)
	{
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[i]);
		SetDiagonalRegularizer(desc, D_DEBRIS_REGULARIZER);
	}

	dFloat32 cosAngle = matrix1.m_front.DotProduct(matrix0.m_front).GetScalar();
	if (cosAngle >= dFloat32(0.998f)) 
	{
		SubmitAngularAxisCartisianApproximation(desc, matrix0, matrix1);
	}
	else 
	{
		SubmitAngularAxis(desc, matrix0, matrix1);
	}
}

void ndJointFixDebrisLink::SubmitAngularAxisCartisianApproximation(ndConstraintDescritor& desc, const dMatrix& matrix0, const dMatrix& matrix1)
{
	// since very small angle rotation commute, we can issue
	// three angle around the matrix1 axis in any order.
	dFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	AddAngularRowJacobian(desc, matrix1.m_up, angle0);
	SetDiagonalRegularizer(desc, D_DEBRIS_REGULARIZER);

	dFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
	AddAngularRowJacobian(desc, matrix1.m_right, angle1);
	SetDiagonalRegularizer(desc, D_DEBRIS_REGULARIZER);
	
	dFloat32 angle2 = CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front);
	AddAngularRowJacobian(desc, matrix1.m_front, angle2);
	SetDiagonalRegularizer(desc, D_DEBRIS_REGULARIZER);
}

void ndJointFixDebrisLink::SubmitAngularAxis(ndConstraintDescritor& desc, const dMatrix& matrix0, const dMatrix& matrix1)
{
	// calculate cone angle
	dVector lateralDir(matrix1.m_front.CrossProduct(matrix0.m_front));
	dAssert(lateralDir.DotProduct(lateralDir).GetScalar() > dFloat32 (1.0e-6f));
	lateralDir = lateralDir.Normalize();
	dFloat32 coneAngle = dAcos(dClamp(matrix1.m_front.DotProduct(matrix0.m_front).GetScalar(), dFloat32(-1.0f), dFloat32(1.0f)));
	dMatrix coneRotation(dQuaternion(lateralDir, coneAngle), matrix1.m_posit);

	AddAngularRowJacobian(desc, lateralDir, -coneAngle);
	SetDiagonalRegularizer(desc, D_DEBRIS_REGULARIZER);

	dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
	AddAngularRowJacobian(desc, sideDir, dFloat32(0.0f));
	SetDiagonalRegularizer(desc, D_DEBRIS_REGULARIZER);

	// calculate pitch angle
	dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
	dFloat32 pitchAngle = dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
	AddAngularRowJacobian(desc, matrix0.m_front, pitchAngle);
	SetDiagonalRegularizer(desc, D_DEBRIS_REGULARIZER);
	//dTrace(("%f %f\n", coneAngle * dRadToDegree, pitchAngle * dRadToDegree));
}
