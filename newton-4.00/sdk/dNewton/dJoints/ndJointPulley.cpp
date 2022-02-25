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

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndJointPulley.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointPulley)

ndJointPulley::ndJointPulley(ndFloat32 gearRatio,
	const ndVector& body0Pin, ndBodyKinematic* const body0,
	const ndVector& body1Pin, ndBodyKinematic* const body1)
	:ndJointBilateralConstraint(1, body0, body1, dGetIdentityMatrix())
	,m_gearRatio(gearRatio)
{
	// calculate the two local matrix of the pivot point
	ndMatrix dommyMatrix;

	// calculate the local matrix for body body0
	ndMatrix pinAndPivot0(body0Pin);
	CalculateLocalMatrix(pinAndPivot0, m_localMatrix0, dommyMatrix);
	m_localMatrix0.m_posit = ndVector::m_wOne;

	// calculate the local matrix for body body1  
	ndMatrix pinAndPivot1(body1Pin);
	CalculateLocalMatrix(pinAndPivot1, dommyMatrix, m_localMatrix1);
	m_localMatrix1.m_posit = ndVector::m_wOne;

	// set as kinematic loop
	SetSolverModel(m_jointkinematicCloseLoop);
}

ndJointPulley::ndJointPulley(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointBilateralConstraint(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_gearRatio(ndFloat32(1.0f))
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	m_gearRatio = xmlGetFloat(xmlNode, "gearRatio");
}

ndJointPulley::~ndJointPulley()
{
}

void ndJointPulley::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBilateralConstraint::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "gearRatio", m_gearRatio);
}

void ndJointPulley::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;

	// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
	CalculateGlobalMatrix(matrix0, matrix1);

	AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));

	ndJacobian& jacobian0 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM0;
	ndJacobian& jacobian1 = desc.m_jacobian[desc.m_rowsCount - 1].m_jacobianM1;

	jacobian0.m_linear = matrix0.m_front.Scale(m_gearRatio);
	jacobian1.m_linear = matrix1.m_front;

	const ndVector& veloc0 = m_body0->GetVelocity();
	const ndVector& veloc1 = m_body1->GetVelocity();

	const ndVector relVeloc(veloc0 * jacobian0.m_linear + veloc1 * jacobian1.m_linear);
	const ndFloat32 w = relVeloc.AddHorizontal().GetScalar() * ndFloat32(0.5f);
	SetMotorAcceleration(desc, -w * desc.m_invTimestep);
}

