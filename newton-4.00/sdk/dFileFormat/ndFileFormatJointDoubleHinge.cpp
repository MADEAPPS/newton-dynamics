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
#include "ndFileFormatJointDoubleHinge.h"

ndFileFormatJointDoubleHinge::ndFileFormatJointDoubleHinge()
	:ndFileFormatJoint(ndJointDoubleHinge::StaticClassName())
{
}

ndFileFormatJointDoubleHinge::ndFileFormatJointDoubleHinge(const char* const className)
	:ndFileFormatJoint(className)
{
}

void ndFileFormatJointDoubleHinge::SaveJoint(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndJointBilateralConstraint* const joint)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_JOINT_CLASS, ndJointDoubleHinge::StaticClassName());
	ndFileFormatJoint::SaveJoint(scene, classNode, joint);

	ndFloat32 spring0;
	ndFloat32 damper0;
	ndFloat32 regularizer0;
	ndFloat32 minTwistAngle0; 
	ndFloat32 maxTwistAngle0;

	ndFloat32 spring1;
	ndFloat32 damper1;
	ndFloat32 regularizer1;
	ndFloat32 minTwistAngle1;
	ndFloat32 maxTwistAngle1;

	ndJointDoubleHinge* const exportJoint = (ndJointDoubleHinge*)joint;

	exportJoint->GetLimits0(minTwistAngle0, maxTwistAngle0);
	exportJoint->GetLimits1(minTwistAngle1, maxTwistAngle1);
	exportJoint->GetSpringDamper0(regularizer0, spring0, damper0);
	exportJoint->GetSpringDamper1(regularizer1, spring1, damper1);
	
	xmlSaveParam(classNode, "offsetAngle0", exportJoint->GetOffsetAngle0() * ndRadToDegree);
	xmlSaveParam(classNode, "springConstant0", spring0);
	xmlSaveParam(classNode, "damperConstant0", damper0);
	xmlSaveParam(classNode, "springRegularizer0", regularizer0);
	xmlSaveParam(classNode, "minTwistAngle0", minTwistAngle0 * ndRadToDegree);
	xmlSaveParam(classNode, "maxTwistAngle0", maxTwistAngle0 * ndRadToDegree);
	//xmlSaveParam(classNode, "limitState", exportJoint->GetLimitState() ? 1 : 0);

	xmlSaveParam(classNode, "offsetAngle1", exportJoint->GetOffsetAngle1() * ndRadToDegree);
	xmlSaveParam(classNode, "springConstant1", spring1);
	xmlSaveParam(classNode, "damperConstant1", damper1);
	xmlSaveParam(classNode, "springRegularizer1", regularizer1);
	xmlSaveParam(classNode, "minTwistAngle1", minTwistAngle1 * ndRadToDegree);
	xmlSaveParam(classNode, "maxTwistAngle1", maxTwistAngle1 * ndRadToDegree);
}

ndJointBilateralConstraint* ndFileFormatJointDoubleHinge::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap)
{
	ndJointDoubleHinge* const joint = new ndJointDoubleHinge();
	LoadJoint(node, bodyMap, joint);
	return joint;
}

void ndFileFormatJointDoubleHinge::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, ndJointBilateralConstraint* const joint)
{
	ndFileFormatJoint::LoadJoint((nd::TiXmlElement*)node->FirstChild(D_JOINT_CLASS), bodyMap, joint);

	ndJointDoubleHinge* const inportJoint = (ndJointDoubleHinge*)joint;

	ndFloat32 offsetAngle0 = xmlGetFloat(node, "offsetAngle0") * ndDegreeToRad;
	ndFloat32 spring0 = xmlGetFloat(node, "springConstant0");
	ndFloat32 damper0 = xmlGetFloat(node, "damperConstant0");
	ndFloat32 regularizer0 = xmlGetFloat(node, "springRegularizer0");
	ndFloat32 minTwistAngle0 = xmlGetFloat(node, "minTwistAngle0") * ndDegreeToRad;
	ndFloat32 maxTwistAngle0 = xmlGetFloat(node, "maxTwistAngle0") * ndDegreeToRad;
	//ndInt32 state = xmlGetInt(node, "limitState");

	ndFloat32 offsetAngle1 = xmlGetFloat(node, "offsetAngle1") * ndDegreeToRad;
	ndFloat32 spring1 = xmlGetFloat(node, "springConstant1");
	ndFloat32 damper1 = xmlGetFloat(node, "damperConstant1");
	ndFloat32 regularizer1 = xmlGetFloat(node, "springRegularizer1");
	ndFloat32 minTwistAngle1 = xmlGetFloat(node, "minTwistAngle1") * ndDegreeToRad;
	ndFloat32 maxTwistAngle1 = xmlGetFloat(node, "maxTwistAngle1") * ndDegreeToRad;
	
	inportJoint->SetOffsetAngle0(offsetAngle0);
	inportJoint->SetAsSpringDamper0(regularizer0, spring0, damper0);
	inportJoint->SetLimits0(minTwistAngle0, maxTwistAngle0);
	//inportJoint->SetLimitState(state ? true : false);

	inportJoint->SetOffsetAngle1(offsetAngle1);
	inportJoint->SetAsSpringDamper1(regularizer1, spring1, damper1);
	inportJoint->SetLimits1(minTwistAngle1, maxTwistAngle1);
}