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
#include "ndFileFormatJointSpherical.h"

ndFileFormatJointSpherical::ndFileFormatJointSpherical()
	:ndFileFormatJoint(ndJointSpherical::StaticClassName())
{
}

ndFileFormatJointSpherical::ndFileFormatJointSpherical(const char* const className)
	:ndFileFormatJoint(className)
{
}

void ndFileFormatJointSpherical::SaveJoint(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndJointBilateralConstraint* const joint)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_JOINT_CLASS, ndJointSpherical::StaticClassName());
	ndFileFormatJoint::SaveJoint(scene, classNode, joint);

	ndFloat32 spring;
	ndFloat32 damper;
	ndFloat32 regularizer;
	ndFloat32 minTwistAngle; 
	ndFloat32 maxTwistAngle;
	ndJointSpherical* const exportJoint = (ndJointSpherical*)joint;

	exportJoint->GetSpringDamper(regularizer, spring, damper);
	exportJoint->GetTwistLimits(minTwistAngle, maxTwistAngle);

	xmlSaveParam(classNode, "rotationTarget", exportJoint->GetOffsetRotation());
	xmlSaveParam(classNode, "springConstant", spring);
	xmlSaveParam(classNode, "damperConstant", damper);
	xmlSaveParam(classNode, "springRegularizer", regularizer);
	xmlSaveParam(classNode, "minTwistAngle", minTwistAngle * ndRadToDegree);
	xmlSaveParam(classNode, "maxTwistAngle", maxTwistAngle * ndRadToDegree);
	xmlSaveParam(classNode, "maxConeAngle", exportJoint->GetConeLimit() * ndRadToDegree);
}

ndJointBilateralConstraint* ndFileFormatJointSpherical::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap)
{
	ndJointSpherical* const joint = new ndJointSpherical();
	LoadJoint(node, bodyMap, joint);
	return joint;
}

void ndFileFormatJointSpherical::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, ndJointBilateralConstraint* const joint)
{
	ndFileFormatJoint::LoadJoint((nd::TiXmlElement*)node->FirstChild(D_JOINT_CLASS), bodyMap, joint);

	ndJointSpherical* const inportJoint = (ndJointSpherical*)joint;

	ndMatrix target(xmlGetMatrix(node, "rotationTarget"));
	ndFloat32 spring = xmlGetFloat(node, "springConstant");
	ndFloat32 damper = xmlGetFloat(node, "damperConstant");
	ndFloat32 regularizer = xmlGetFloat(node, "springRegularizer");
	ndFloat32 minTwistAngle = xmlGetFloat(node, "minTwistAngle") * ndDegreeToRad;
	ndFloat32 maxTwistAngle = xmlGetFloat(node, "maxTwistAngle") * ndDegreeToRad;
	ndFloat32 maxConeAngle = xmlGetFloat(node, "maxConeAngle") * ndDegreeToRad;

	inportJoint->SetAsSpringDamper(regularizer, spring, damper);
	inportJoint->SetTwistLimits(minTwistAngle, maxTwistAngle);
	inportJoint->SetConeLimit(maxConeAngle);
}