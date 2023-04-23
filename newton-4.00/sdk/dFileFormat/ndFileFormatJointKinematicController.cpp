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
#include "ndFileFormatJointKinematicController.h"

ndFileFormatJointKinematicController::ndFileFormatJointKinematicController()
	:ndFileFormatJoint(ndJointKinematicController::StaticClassName())
{
}

ndFileFormatJointKinematicController::ndFileFormatJointKinematicController(const char* const className)
	:ndFileFormatJoint(className)
{
}

void ndFileFormatJointKinematicController::SaveJoint(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndJointBilateralConstraint* const joint)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_JOINT_CLASS, ndJointKinematicController::StaticClassName());
	ndFileFormatJoint::SaveJoint(scene, classNode, joint);

	ndJointKinematicController* const exportJoint = (ndJointKinematicController*)joint;

	xmlSaveParam(classNode, "maxSpeed", exportJoint->GetMaxSpeed());
	xmlSaveParam(classNode, "maxOmega", exportJoint->GetMaxOmega());
	xmlSaveParam(classNode, "controlMode", ndInt32(exportJoint->GetControlMode()));
	xmlSaveParam(classNode, "maxLinearFriction", exportJoint->GetMaxLinearFriction());
	xmlSaveParam(classNode, "maxAngularFriction", exportJoint->GetMaxAngularFriction());
	xmlSaveParam(classNode, "angularFrictionCoefficient", exportJoint->GetAngularViscousFrictionCoefficient());
}

ndJointBilateralConstraint* ndFileFormatJointKinematicController::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap)
{
	ndJointHinge* const joint = new ndJointHinge();
	LoadJoint(node, bodyMap, joint);
	return joint;
}

void ndFileFormatJointKinematicController::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, ndJointBilateralConstraint* const joint)
{
	ndFileFormatJoint::LoadJoint((nd::TiXmlElement*)node->FirstChild(D_JOINT_CLASS), bodyMap, joint);

	ndJointKinematicController* const inportJoint = (ndJointKinematicController*)joint;

	ndFloat32 maxSpeed = xmlGetFloat(node, "maxSpeed");
	ndFloat32 maxOmega = xmlGetFloat(node, "maxOmega");
	ndInt32 controlMode = xmlGetInt(node, "controlMode");
	ndFloat32 maxLinearFriction = xmlGetFloat(node, "maxLinearFriction");
	ndFloat32 maxAngularFriction = xmlGetFloat(node, "maxAngularFriction");
	ndFloat32 angularFrictionCoefficient = xmlGetFloat(node, "angularFrictionCoefficient");

	inportJoint->SetMaxSpeed(maxSpeed);
	inportJoint->SetMaxOmega(maxOmega);
	inportJoint->SetMaxLinearFriction(maxLinearFriction);
	inportJoint->SetMaxAngularFriction(maxAngularFriction);
	inportJoint->SetAngularViscousFrictionCoefficient(angularFrictionCoefficient);
	inportJoint->SetControlMode(ndJointKinematicController::ndControlModes(controlMode));
}