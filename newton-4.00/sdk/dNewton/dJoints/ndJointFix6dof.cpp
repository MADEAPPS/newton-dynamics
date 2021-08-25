/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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
#include "ndJointFix6dof.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointFix6dof)

ndJointFix6dof::ndJointFix6dof(const dMatrix& frameInGlbalSpace, ndBodyKinematic* const body0, ndBodyKinematic* const body1)
	:ndJointBilateralConstraint(6, body0, body1, frameInGlbalSpace)
	,m_softness(dFloat32(0.0f))
	,m_maxForce(D_MAX_BOUND)
	,m_maxTorque(D_MAX_BOUND)
{
}

ndJointFix6dof::ndJointFix6dof(const dLoadSaveBase::dLoadDescriptor& desc)
	:ndJointBilateralConstraint(dLoadSaveBase::dLoadDescriptor(desc))
	,m_softness(dFloat32(0.0f))
	,m_maxForce(D_MAX_BOUND)
	,m_maxTorque(D_MAX_BOUND)
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	
	m_softness = xmlGetFloat(xmlNode, "softness");
	m_maxForce = xmlGetFloat(xmlNode, "maxForce");
	m_maxTorque = xmlGetFloat(xmlNode, "maxTorque");
}

ndJointFix6dof::~ndJointFix6dof()
{
}

void ndJointFix6dof::SetAsSoftJoint(bool)
{
	dAssert(0);
	//SetSolverModel(mode ? m_secundaryCloseLoop : m_primaryOpenLoop);
}

void ndJointFix6dof::SetRegularizer(dFloat32 regularizer)
{
	m_softness = dClamp(regularizer, dFloat32(0.0f), dFloat32(1.0f));
}

void ndJointFix6dof::JacobianDerivative(ndConstraintDescritor& desc)
{
	dMatrix matrix0;
	dMatrix matrix1;

//for (dInt32 i = 0; i < 3; i++)
//{
//	if ((dAbs(m_jointForce[i + 0].m_force) >= m_maxForce * 0.99f) ||
//		(dAbs(m_jointForce[i + 3].m_force) >= m_maxTorque * 0.99f))
//	{
//		SetActive(false);
//		return;
//	}
//}


	dAssert(IsActive());
	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	for (dInt32 i = 0; i < 3; i++)
	{
		AddLinearRowJacobian(desc, matrix0.m_posit, matrix1.m_posit, matrix1[i]);
		SetLowerFriction(desc, -m_maxForce);
		SetHighFriction(desc, m_maxForce);
		SetDiagonalRegularizer(desc, m_softness);
	}

	dFloat32 cosAngle = matrix1.m_front.DotProduct(matrix0.m_front).GetScalar();
	if (cosAngle >= dFloat32(0.998f)) 
	{
		// about 3.5 degree deviation, consider small angular approximation  
		SubmitAngularAxisCartisianApproximation(desc, matrix0, matrix1);
	}
	else 
	{
		// beyond 3.5 degree need to decompose the relative matrix into an orthonormal basics 
		SubmitAngularAxis(desc, matrix0, matrix1);
	}
}

void ndJointFix6dof::SubmitAngularAxisCartisianApproximation(ndConstraintDescritor& desc, const dMatrix& matrix0, const dMatrix& matrix1)
{
	// since very small angle rotation commute, we can issue
	// three angle around the matrix1 axis in any order.
	dFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
	AddAngularRowJacobian(desc, matrix1.m_up, angle0);
	SetLowerFriction(desc, -m_maxTorque);
	SetHighFriction(desc, m_maxTorque);
	SetDiagonalRegularizer(desc, m_softness);

	dFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
	AddAngularRowJacobian(desc, matrix1.m_right, angle1);
	SetLowerFriction(desc, -m_maxTorque);
	SetHighFriction(desc, m_maxTorque);
	SetDiagonalRegularizer(desc, m_softness);
	
	dFloat32 angle2 = CalculateAngle(matrix0.m_up, matrix1.m_up, matrix1.m_front);
	AddAngularRowJacobian(desc, matrix1.m_front, angle2);
	SetLowerFriction(desc, -m_maxTorque);
	SetHighFriction(desc, m_maxTorque);
	SetDiagonalRegularizer(desc, m_softness);
}

void ndJointFix6dof::SubmitAngularAxis(ndConstraintDescritor& desc, const dMatrix& matrix0, const dMatrix& matrix1)
{
	// calculate cone angle
	dVector lateralDir(matrix1.m_front.CrossProduct(matrix0.m_front));
	dAssert(lateralDir.DotProduct(lateralDir).GetScalar() > dFloat32 (1.0e-6f));
	lateralDir = lateralDir.Normalize();
	dFloat32 coneAngle = dAcos(dClamp(matrix1.m_front.DotProduct(matrix0.m_front).GetScalar(), dFloat32(-1.0f), dFloat32(1.0f)));
	dMatrix coneRotation(dQuaternion(lateralDir, coneAngle), matrix1.m_posit);

	AddAngularRowJacobian(desc, lateralDir, -coneAngle);
	SetLowerFriction(desc, -m_maxTorque);
	SetHighFriction(desc, m_maxTorque);
	SetDiagonalRegularizer(desc, m_softness);

	dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
	AddAngularRowJacobian(desc, sideDir, dFloat32(0.0f));
	SetLowerFriction(desc, -m_maxTorque);
	SetHighFriction(desc, m_maxTorque);
	SetDiagonalRegularizer(desc, m_softness);

	// calculate pitch angle
	dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
	dFloat32 pitchAngle = dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
	AddAngularRowJacobian(desc, matrix0.m_front, pitchAngle);
	SetLowerFriction(desc, -m_maxTorque);
	SetHighFriction(desc, m_maxTorque);
	SetDiagonalRegularizer(desc, m_softness);
	//dTrace(("%f %f\n", coneAngle * dRadToDegree, pitchAngle * dRadToDegree));
}

void ndJointFix6dof::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(dLoadSaveBase::dSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "m_softness", m_softness);
	xmlSaveParam(childNode, "m_softness", m_maxForce);
	xmlSaveParam(childNode, "m_softness", m_maxTorque);
}
