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
#include "ndFileFormatJointHinge.h"

ndFileFormatJointHinge::ndFileFormatJointHinge()
	:ndFileFormatJoint(ndJointHinge::StaticClassName())
{
}

ndFileFormatJointHinge::ndFileFormatJointHinge(const char* const className)
	:ndFileFormatJoint(className)
{
}

void ndFileFormatJointHinge::SaveJoint(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndJointBilateralConstraint* const joint)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_JOINT_CLASS, ndJointHinge::StaticClassName());
	ndFileFormatJoint::SaveJoint(scene, classNode, joint);

	ndFloat32 spring;
	ndFloat32 damper;
	ndFloat32 regularizer;
	ndFloat32 minTwistAngle; 
	ndFloat32 maxTwistAngle;
	ndJointHinge* const exportJoint = (ndJointHinge*)joint;

	exportJoint->GetSpringDamper(regularizer, spring, damper);
	exportJoint->GetLimits(minTwistAngle, maxTwistAngle);

	xmlSaveParam(classNode, "targetAngle", exportJoint->GetTargetAngle() * ndRadToDegree);
	xmlSaveParam(classNode, "springConstant", spring);
	xmlSaveParam(classNode, "damperConstant", damper);
	xmlSaveParam(classNode, "springRegularizer", regularizer);
	xmlSaveParam(classNode, "minTwistAngle", minTwistAngle * ndRadToDegree);
	xmlSaveParam(classNode, "maxTwistAngle", maxTwistAngle * ndRadToDegree);
	xmlSaveParam(classNode, "limitState", exportJoint->GetLimitState() ? 1 : 0);
}

ndJointBilateralConstraint* ndFileFormatJointHinge::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap)
{
	ndJointHinge* const joint = new ndJointHinge();
	LoadJoint(node, bodyMap, joint);
	return joint;
}

void ndFileFormatJointHinge::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, ndJointBilateralConstraint* const joint)
{
	ndFileFormatJoint::LoadJoint((nd::TiXmlElement*)node->FirstChild(D_JOINT_CLASS), bodyMap, joint);

	ndJointHinge* const inportJoint = (ndJointHinge*)joint;

	ndFloat32 offsetAngle = xmlGetFloat(node, "targetAngle") * ndDegreeToRad;
	ndFloat32 spring = xmlGetFloat(node, "springConstant");
	ndFloat32 damper = xmlGetFloat(node, "damperConstant");
	ndFloat32 regularizer = xmlGetFloat(node, "springRegularizer");
	ndFloat32 minTwistAngle = xmlGetFloat(node, "minTwistAngle") * ndDegreeToRad;
	ndFloat32 maxTwistAngle = xmlGetFloat(node, "maxTwistAngle") * ndDegreeToRad;
	ndInt32 state = xmlGetInt(node, "limitState");

	inportJoint->SetTargetAngle(offsetAngle);
	inportJoint->SetAsSpringDamper(regularizer, spring, damper);
	inportJoint->SetLimits(minTwistAngle, maxTwistAngle);
	inportJoint->SetLimitState(state ? true : false);
}