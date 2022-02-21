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
#include "ndJointIkBallAndSocket.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndJointIkBallAndSocket)

ndJointIkBallAndSocket::ndJointIkBallAndSocket(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
	:ndJointBallAndSocket(pinAndPivotFrame, child, parent)
	,m_coneRow()
	,m_twistRow()
	,m_biConeRow()
{
}

ndJointIkBallAndSocket::ndJointIkBallAndSocket(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointBallAndSocket(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_coneRow()
	,m_twistRow()
	,m_biConeRow()
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

ndJointIkBallAndSocket::~ndJointIkBallAndSocket()
{
}

void ndJointIkBallAndSocket::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointBallAndSocket::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	dAssert(0);
	//xmlSaveParam(childNode, "maxConeAngle", m_maxConeAngle);
	//xmlSaveParam(childNode, "coneFriction", m_coneFriction);
	//xmlSaveParam(childNode, "minTwistAngle", m_minTwistAngle);
	//xmlSaveParam(childNode, "maxTwistAngle", m_maxTwistAngle);
	//xmlSaveParam(childNode, "twistFriction", m_twistFriction);
	//xmlSaveParam(childNode, "coneFrictionRegularizer", m_coneFrictionRegularizer);
	//xmlSaveParam(childNode, "twistFrictionRegularizer", m_twistFrictionRegularizer);
}

void ndJointIkBallAndSocket::DebugJoint(ndConstraintDebugCallback& debugCallback) const
{
	ndJointBallAndSocket::DebugJoint(debugCallback);
}

bool ndJointIkBallAndSocket::IsIk() const
{
	return true;
}

void ndJointIkBallAndSocket::SetIkSolver()
{
	m_coneRow.Set();
	m_twistRow.Set();
	m_biConeRow.Set();
}

void ndJointIkBallAndSocket::ResetIkSolver()
{
	m_coneRow.Reset();
	m_twistRow.Reset();
	m_biConeRow.Reset();
}

void ndJointIkBallAndSocket::StopIkMotor(ndFloat32 timestep)
{
	dAssert(0);
}

bool ndJointIkBallAndSocket::SetIkMotor(ndFloat32 timestep, const ndJacobian& forceBody0, const ndJacobian& forceBody1)
{
	dAssert(0);
	return 0;
}

void ndJointIkBallAndSocket::JacobianDerivative(ndConstraintDescritor& desc)
{
	m_coneLimits = false;
	m_twistLimits = false;
	ndJointBallAndSocket::JacobianDerivative(desc);

	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);

	ndFloat32 cosAngleCos = matrix1.m_front.DotProduct(matrix0.m_front).GetScalar();
	if (cosAngleCos >= ndFloat32(0.998f))
	{
		// special case where the front axis are almost aligned
		// solve by using Cartesian approximation
		//SubmitAngularAxisCartesianApproximation(matrix0, matrix1, desc);
		//ndFloat32 coneAngle = ndAcos(dClamp(cosAngleCos, ndFloat32(-1.0f), ndFloat32(1.0f)));
		//if (coneAngle > m_maxConeAngle)
		//ndFloat32 pitchAngle = -CalculateAngle(matrix0[1], matrix1[1], matrix1[0]);
		//if (dAbs (pitchAngle) < (ndFloat32 (2.0f) * ndDegreeToRad))
		//{
			// two rows to restrict rotation around around the parent coordinate system
			//ndFloat32 angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
		AddAngularRowJacobian(desc, matrix1.m_up, ndFloat32 (0.0f));
		SetMotorAcceleration(desc, m_coneRow.m_motorAccel);
		SetMotorAcceleration(desc, 1.0f);
		SetLowerFriction(desc, m_coneRow.m_minForce);
		SetHighFriction(desc, m_coneRow.m_maxForce);
			
		//ndFloat32 angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
		AddAngularRowJacobian(desc, matrix1.m_right, ndFloat32(0.0f));
		SetMotorAcceleration(desc, m_biConeRow.m_motorAccel);
		//SetMotorAcceleration(desc, 0.5f);
		SetLowerFriction(desc, m_biConeRow.m_minForce);
		SetHighFriction(desc, m_biConeRow.m_maxForce);


		AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
		SetMotorAcceleration(desc, m_twistRow.m_motorAccel);
		//SetMotorAcceleration(desc, 0.5f);
		SetLowerFriction(desc, m_twistRow.m_minForce);
		SetHighFriction(desc, m_twistRow.m_maxForce);
	}
	else
	{
		ndVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
		dAssert(lateralDir.DotProduct(lateralDir).GetScalar() > 1.0e-6f);
		lateralDir = lateralDir.Normalize();
		const ndVector sideDir(lateralDir.CrossProduct(matrix0.m_front));

		AddAngularRowJacobian(desc, lateralDir, ndFloat32(0.0f));
		SetMotorAcceleration(desc, m_coneRow.m_motorAccel);
		//SetMotorAcceleration(desc, 0.5f);
		SetLowerFriction(desc, m_coneRow.m_minForce);
		SetHighFriction(desc, m_coneRow.m_maxForce);

		AddAngularRowJacobian(desc, sideDir, ndFloat32(0.0f));
		SetMotorAcceleration(desc, m_biConeRow.m_motorAccel);
		//SetMotorAcceleration(desc, 0.5f);
		SetLowerFriction(desc, m_biConeRow.m_minForce);
		SetHighFriction(desc, m_biConeRow.m_maxForce);


		AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
		SetMotorAcceleration(desc, m_twistRow.m_motorAccel);
		//SetMotorAcceleration(desc, 0.1f);
		SetLowerFriction(desc, m_twistRow.m_minForce);
		SetHighFriction(desc, m_twistRow.m_maxForce);
	}

	//AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
	//SetMotorAcceleration(desc, m_twistRow.m_motorAccel);
	//SetMotorAcceleration(desc, 0.1f);
	//SetLowerFriction(desc, m_twistRow.m_minForce);
	//SetHighFriction(desc, m_twistRow.m_maxForce);

}