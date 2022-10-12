/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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
#include "ndMultiBodyVehicle.h"
#include "ndMultiBodyVehicleTireJoint.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndMultiBodyVehicleTireJoint)

ndMultiBodyVehicleTireJoint::ndMultiBodyVehicleTireJoint(const ndMatrix& pinAndPivotFrame, ndBodyKinematic* const child, ndBodyKinematic* const parent, const ndMultiBodyVehicleTireJointInfo& info, ndMultiBodyVehicle* const vehicle)
	:ndJointWheel(pinAndPivotFrame, child, parent, info)
	,m_vehicle(vehicle)
	,m_frictionModel(info)
	,m_lateralSlip(ndFloat32 (0.0f))
	,m_longitudinalSlip(ndFloat32(0.0f))
{
	m_frictionModel.m_laterialStiffness = ndMax(ndAbs(m_frictionModel.m_laterialStiffness), ndFloat32(5.0e4f));
	m_frictionModel.m_longitudinalStiffness = ndMax(ndAbs(m_frictionModel.m_longitudinalStiffness), ndFloat32(5.0e4f));
}

ndMultiBodyVehicleTireJoint::ndMultiBodyVehicleTireJoint(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndJointWheel(ndLoadSaveBase::ndLoadDescriptor(desc))
	,m_vehicle(nullptr)
	,m_frictionModel()
	,m_lateralSlip(ndFloat32(0.0f))
	,m_longitudinalSlip(ndFloat32(0.0f))
{
	ndAssert(0);
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	m_lateralSlip = xmlGetFloat(xmlNode, "lateralSlip");
	m_longitudinalSlip = xmlGetFloat(xmlNode, "longitudinalSlip");
}

ndMultiBodyVehicleTireJoint::~ndMultiBodyVehicleTireJoint()
{
}

void ndMultiBodyVehicleTireJoint::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndJointWheel::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	ndAssert(0);
	xmlSaveParam(childNode, "lateralSlip", m_lateralSlip);
	xmlSaveParam(childNode, "longitudinalSlip", m_longitudinalSlip);
}

ndFloat32 ndMultiBodyVehicleTireJoint::GetSideSlip() const
{
	return m_lateralSlip;
}

ndFloat32 ndMultiBodyVehicleTireJoint::GetLongitudinalSlip() const
{
	return m_longitudinalSlip;
}

void ndMultiBodyVehicleTireJoint::JacobianDerivative(ndConstraintDescritor& desc)
{
	m_regularizer = m_info.m_regularizer * m_vehicle->m_downForce.m_suspensionStiffnessModifier;
	ndJointWheel::JacobianDerivative(desc);
}

