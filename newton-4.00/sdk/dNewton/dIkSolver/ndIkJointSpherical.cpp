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
	,m_coneRow()
	,m_twistRow()
	,m_biConeRow()
{
}

ndIkJointSpherical::ndIkJointSpherical(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointSpherical(ndLoadSaveBase::ndLoadDescriptor(desc))
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

bool ndIkJointSpherical::IsIk() const
{
	return true;
}

void ndIkJointSpherical::SetIkSolver()
{
	m_coneRow.Set();
	m_twistRow.Set();
	m_biConeRow.Set();
}

void ndIkJointSpherical::ResetIkSolver()
{
	m_coneRow.Reset();
	m_twistRow.Reset();
	m_biConeRow.Reset();
}

void ndIkJointSpherical::StopIkMotor(ndFloat32 timestep)
{
	dAssert(0);
}

bool ndIkJointSpherical::SetIkMotor(ndFloat32 timestep, const ndJacobian& forceBody0, const ndJacobian& forceBody1)
{
	dAssert(0);
	return 0;
}

void ndIkJointSpherical::JacobianDerivative(ndConstraintDescritor& desc)
{
//ndJointSpherical::JacobianDerivative(desc);
//return;
	ndMatrix matrix0;
	ndMatrix matrix1;
	CalculateGlobalMatrix(matrix0, matrix1);
	ApplyBaseRows(matrix0, matrix1, desc);

	ndVector omega(GetBody0()->GetMatrix().UnrotateVector(GetBody0()->GetOmega()));
return;

	ndFloat32 cosAngleCos = matrix1.m_front.DotProduct(matrix0.m_front).GetScalar();
	if (cosAngleCos >= ndFloat32(0.999f))
	{
		// special case where the front axis are almost aligned
		// solve by using Cartesian approximation
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
		SetMotorAcceleration(desc, 0.5f);
		SetLowerFriction(desc, m_twistRow.m_minForce);
		SetHighFriction(desc, m_twistRow.m_maxForce);
	}
	else
	{
		//matrix1 = dGetIdentityMatrix();
		//ndVector xxxxxxxxx(30.0f * ndDegreeToRad, 45.0f * ndDegreeToRad, 45.0f * ndDegreeToRad, 0.0f);
		//matrix0 = dPitchMatrix(xxxxxxxxx.m_x) * dYawMatrix(xxxxxxxxx.m_y) * dRollMatrix(xxxxxxxxx.m_z) * matrix1;
		//
		//
		//ndMatrix xxx(matrix0 * matrix1.Inverse());
		//ndVector e0;
		//ndVector e1;
		//xxx.CalcPitchYawRoll(e0, e1);
		//
		//ndMatrix xxx1 = dPitchMatrix(e0.m_x);
		//ndMatrix xxx2 = dYawMatrix(e0.m_y) * dRollMatrix(e0.m_z);
		//ndMatrix Q = matrix1.Inverse() * xxx2 * matrix1;
		//ndQuaternion zzzz(Q);
		//ndVector xxxxxxx((zzzz & ndVector::m_triplexMask).Normalize());
		//
		//
		//ndVector lateralDir__(matrix1[0].CrossProduct(matrix0[0]));
		////dAssert(lateralDir.DotProduct3(lateralDir) > 1.0e-6f);
		//lateralDir__ = lateralDir__.Normalize();
		//ndFloat32 coneAngle = ndAcos(dClamp(matrix1.m_front.DotProduct(matrix0.m_front).GetScalar(), ndFloat32(-1.0f), ndFloat32(1.0f)));
		//ndMatrix coneRotation(ndQuaternion(lateralDir__, coneAngle), matrix1.m_posit);
		//ndMatrix pitchMatrix(matrix0 * (matrix1 * coneRotation).Inverse());
		//
		//
		//ndVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
		//dAssert(lateralDir.DotProduct(lateralDir).GetScalar() > 1.0e-6f);
		//lateralDir = lateralDir.Normalize();

		ndMatrix xxx(matrix0 * matrix1.Inverse());
		ndVector e0;
		ndVector e1;
		xxx.CalcPitchYawRoll(e0, e1);
		ndMatrix xxx1 = dPitchMatrix(e0.m_x);
		ndMatrix xxx2 = dYawMatrix(e0.m_y) * dRollMatrix(e0.m_z);
		ndMatrix Q = matrix1.Inverse() * xxx2 * matrix1;

		//ndQuaternion lateralDir(Q);
		ndVector lateralDir((ndQuaternion (Q) & ndVector::m_triplexMask).Normalize());

		//AddAngularRowJacobian(desc, lateralDir, ndFloat32(0.0f));
		//SetMotorAcceleration(desc, m_coneRow.m_motorAccel);
		////SetMotorAcceleration(desc, 0.5f);
		//SetLowerFriction(desc, m_coneRow.m_minForce);
		//SetHighFriction(desc, m_coneRow.m_maxForce);
		//
		//const ndVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
		//AddAngularRowJacobian(desc, sideDir, ndFloat32(0.0f));
		//SetMotorAcceleration(desc, m_biConeRow.m_motorAccel);
		////SetMotorAcceleration(desc, 0.5f);
		//SetLowerFriction(desc, m_biConeRow.m_minForce);
		//SetHighFriction(desc, m_biConeRow.m_maxForce);
		//
		//
		//AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
		//SetMotorAcceleration(desc, m_twistRow.m_motorAccel);
		////SetMotorAcceleration(desc, 0.1f);
		//SetLowerFriction(desc, m_twistRow.m_minForce);
		//SetHighFriction(desc, m_twistRow.m_maxForce);
	}

	//AddAngularRowJacobian(desc, matrix0.m_front, ndFloat32(0.0f));
	//SetMotorAcceleration(desc, m_twistRow.m_motorAccel);
	//SetMotorAcceleration(desc, 0.1f);
	//SetLowerFriction(desc, m_twistRow.m_minForce);
	//SetHighFriction(desc, m_twistRow.m_maxForce);

}