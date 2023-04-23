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
#include "ndFileFormatJointCylinder.h"

ndFileFormatJointCylinder::ndFileFormatJointCylinder()
	:ndFileFormatJoint(ndJointCylinder::StaticClassName())
{
}

ndFileFormatJointCylinder::ndFileFormatJointCylinder(const char* const className)
	:ndFileFormatJoint(className)
{
}

void ndFileFormatJointCylinder::SaveJoint(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndJointBilateralConstraint* const joint)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_JOINT_CLASS, ndJointCylinder::StaticClassName());
	ndFileFormatJoint::SaveJoint(scene, classNode, joint);

	ndFloat32 spring;
	ndFloat32 damper;
	ndFloat32 regularizer;
	ndFloat32 minTwistAngle; 
	ndFloat32 maxTwistAngle;
	
	ndFloat32 spring1;
	ndFloat32 damper1;
	ndFloat32 regularizer1;
	ndFloat32 minPositLimit;
	ndFloat32 maxPositLimit;

	ndJointCylinder* const exportJoint = (ndJointCylinder*)joint;

	exportJoint->GetLimitsAngle(minTwistAngle, maxTwistAngle);
	exportJoint->GetLimitsPosit(minPositLimit, maxPositLimit);
	exportJoint->GetSpringDamperAngle(regularizer, spring, damper);
	exportJoint->GetSpringDamperPosit(regularizer1, spring1, damper1);
	
	xmlSaveParam(classNode, "offsetAngle", exportJoint->GetOffsetAngle() * ndRadToDegree);
	xmlSaveParam(classNode, "springConstantAngle", spring);
	xmlSaveParam(classNode, "damperConstantAngle", damper);
	xmlSaveParam(classNode, "springRegularizerAngle", regularizer);
	xmlSaveParam(classNode, "minTwistAngle", minTwistAngle * ndRadToDegree);
	xmlSaveParam(classNode, "maxTwistAngle", maxTwistAngle * ndRadToDegree);
	xmlSaveParam(classNode, "limitStateAngle", exportJoint->GetLimitStateAngle() ? 1 : 0);
	
	xmlSaveParam(classNode, "offsetPosit", exportJoint->GetOffsetPosit());
	xmlSaveParam(classNode, "springConstantPosit", spring1);
	xmlSaveParam(classNode, "damperConstantPosit", damper1);
	xmlSaveParam(classNode, "springRegularizerPosit", regularizer1);
	xmlSaveParam(classNode, "minPosit", minPositLimit);
	xmlSaveParam(classNode, "maxPosit", maxPositLimit);
	xmlSaveParam(classNode, "limitStatePosit", exportJoint->GetLimitStatePosit() ? 1 : 0);
}

ndJointBilateralConstraint* ndFileFormatJointCylinder::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap)
{
	ndJointCylinder* const joint = new ndJointCylinder();
	LoadJoint(node, bodyMap, joint);
	return joint;
}

void ndFileFormatJointCylinder::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, ndJointBilateralConstraint* const joint)
{
	ndFileFormatJoint::LoadJoint((nd::TiXmlElement*)node->FirstChild(D_JOINT_CLASS), bodyMap, joint);

	ndJointCylinder* const inportJoint = (ndJointCylinder*)joint;

	ndFloat32 offsetAngle = xmlGetFloat(node, "offsetAngle") * ndDegreeToRad;
	ndFloat32 springAngle = xmlGetFloat(node, "springConstantAngle");
	ndFloat32 damperAngle = xmlGetFloat(node, "damperConstantAngle");
	ndFloat32 regularizerAngle = xmlGetFloat(node, "springRegularizerAngle");
	ndFloat32 minTwistAngle = xmlGetFloat(node, "minTwistAngle") * ndDegreeToRad;
	ndFloat32 maxTwistAngle = xmlGetFloat(node, "maxTwistAngle") * ndDegreeToRad;
	ndInt32 stateAngle = xmlGetInt(node, "limitStateAngle");

	ndFloat32 offsetPosit = xmlGetFloat(node, "offsetPosit");
	ndFloat32 springPosit = xmlGetFloat(node, "springConstantPosit");
	ndFloat32 damperPosit = xmlGetFloat(node, "damperConstantPosit");
	ndFloat32 regularizerPosit = xmlGetFloat(node, "springRegularizerPosit");
	ndFloat32 minPosit = xmlGetFloat(node, "minPosit");
	ndFloat32 maxPosit = xmlGetFloat(node, "maxPosit");
	ndInt32 statePosit = xmlGetInt(node, "limitStatePosit");

	inportJoint->SetOffsetAngle(offsetAngle);
	inportJoint->SetAsSpringDamperAngle(regularizerAngle, springAngle, damperAngle);
	inportJoint->SetLimitsAngle(minTwistAngle, maxTwistAngle);
	inportJoint->SetLimitStateAngle(stateAngle ? true : false);

	inportJoint->SetOffsetPosit(offsetPosit);
	inportJoint->SetAsSpringDamperPosit(regularizerPosit, springPosit, damperPosit);
	inportJoint->SetLimitsPosit(minPosit, maxPosit);
	inportJoint->SetLimitStatePosit(statePosit ? true : false);
}