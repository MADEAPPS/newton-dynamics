/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndFileFormatStdafx.h"
#include "ndFileFormatJointWheel.h"

ndFileFormatJointWheel::ndFileFormatJointWheel()
	:ndFileFormatJoint(ndJointWheel::StaticClassName())
{
}

ndFileFormatJointWheel::ndFileFormatJointWheel(const char* const className)
	:ndFileFormatJoint(className)
{
}

void ndFileFormatJointWheel::SaveJoint(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndJointBilateralConstraint* const joint)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_JOINT_CLASS, ndJointWheel::StaticClassName());
	ndFileFormatJoint::SaveJoint(scene, classNode, joint);

	ndJointWheel* const exportJoint = (ndJointWheel*)joint;
	const ndWheelDescriptor& info = exportJoint->GetInfo();

	xmlSaveParam(classNode, "radios", info.m_radios);
	xmlSaveParam(classNode, "springK", info.m_springK);
	xmlSaveParam(classNode, "damperC", info.m_damperC);
	xmlSaveParam(classNode, "upperStop", info.m_upperStop);
	xmlSaveParam(classNode, "lowerStop", info.m_lowerStop);
	xmlSaveParam(classNode, "regularizer", info.m_regularizer);
	xmlSaveParam(classNode, "brakeTorque", info.m_brakeTorque);
	xmlSaveParam(classNode, "handBrakeTorque", info.m_handBrakeTorque);
	xmlSaveParam(classNode, "steeringAngle", info.m_steeringAngle * ndRadToDegree);
}

ndJointBilateralConstraint* ndFileFormatJointWheel::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap)
{
	ndJointWheel* const joint = new ndJointWheel();
	LoadJoint(node, bodyMap, joint);
	return joint;
}

void ndFileFormatJointWheel::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, ndJointBilateralConstraint* const joint)
{
	ndFileFormatJoint::LoadJoint((nd::TiXmlElement*)node->FirstChild(D_JOINT_CLASS), bodyMap, joint);

	ndJointWheel* const inportJoint = (ndJointWheel*)joint;

	ndWheelDescriptor info;
	info.m_radios = xmlGetFloat(node, "radios");
	info.m_springK = xmlGetFloat(node, "springK");
	info.m_damperC = xmlGetFloat(node, "damperC");
	info.m_regularizer = xmlGetFloat(node, "regularizer");

	info.m_upperStop = xmlGetFloat(node, "upperStop");
	info.m_lowerStop = xmlGetFloat(node, "lowerStop");
	info.m_brakeTorque = xmlGetFloat(node, "brakeTorque");
	info.m_steeringAngle = xmlGetFloat(node, "steeringAngle");
	info.m_handBrakeTorque = xmlGetFloat(node, "handBrakeTorque");

	inportJoint->SetInfo(info);
}