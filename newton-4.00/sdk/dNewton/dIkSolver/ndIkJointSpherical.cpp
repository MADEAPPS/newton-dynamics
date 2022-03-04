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
#include "ndIkJointSpherical.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndIkJointSpherical)

ndIkJointSpherical::ndIkJointSpherical(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointSpherical(pinAndPivotFrame, child, parent)
	,ndIkInterface()
{
}

ndIkJointSpherical::ndIkJointSpherical(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointSpherical(ndLoadSaveBase::ndLoadDescriptor(desc))
	,ndIkInterface()
{
	dAssert(0);
	//const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	//m_maxConeAngle = xmlGetFloat(xmlNode, "maxConeAngle");
	//m_coneFriction = xmlGetFloat(xmlNode, "coneFriction");
	//m_minTwistAngle = xmlGetFloat(xmlNode, "minTwistAngle");
	//m_maxTwistAngle = xmlGetFloat(xmlNode, "maxTwistAngle");
	//m_twistFriction = xmlGetFloat(xmlNode, "twistFriction");
	//m_coneFrictionRegularizer = xmlGetFloat(xmlNode, "coneFrictionRegularizer");
	//m_twistFrictionRegularizer = xmlGetFloat(xmlNode, "twistFrictionRegularizer");
}

ndIkJointSpherical::~ndIkJointSpherical()
{
}

void ndIkJointSpherical::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointSpherical::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	dAssert(0);
	//xmlSaveParam(childNode, "maxConeAngle", m_maxConeAngle);
	//xmlSaveParam(childNode, "coneFriction", m_coneFriction);
	//xmlSaveParam(childNode, "minTwistAngle", m_minTwistAngle);
	//xmlSaveParam(childNode, "maxTwistAngle", m_maxTwistAngle);
	//xmlSaveParam(childNode, "twistFriction", m_twistFriction);
	//xmlSaveParam(childNode, "coneFrictionRegularizer", m_coneFrictionRegularizer);
	//xmlSaveParam(childNode, "twistFrictionRegularizer", m_twistFrictionRegularizer);
}

void ndIkJointSpherical::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	ndJointSpherical::DebugJoint(debugCallback);
}

void ndIkJointSpherical::SubmitAccel(const ndMatrix&, const ndMatrix& matrix1, ndConstraintDescritor& desc)
{
	// if we have the alpha, there is not need to find a shortest path, any set of axis will do 
	for (ndInt32 i = 0; i < 3; ++i)
	{
		ndFloat32 accel = (matrix1[i] * m_accel0.m_angular - matrix1[i] * m_accel1.m_angular).AddHorizontal().GetScalar();
		AddAngularRowJacobian(desc, matrix1[i], ndFloat32(0.0f));
		SetMotorAcceleration(desc, accel);
	}
}

void ndIkJointSpherical::JacobianDerivative(ndConstraintDescritor& desc)
{
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	ApplyBaseRows(matrix0, matrix1, desc);
	SubmitLimits(matrix0, matrix1, desc);

	if (!m_ikMode)
	{
		SubmitAccel(matrix0, matrix1, desc);
	}
}