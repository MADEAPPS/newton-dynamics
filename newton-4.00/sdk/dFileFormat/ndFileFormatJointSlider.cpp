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
#include "ndFileFormatJointSlider.h"

ndFileFormatJointSlider::ndFileFormatJointSlider()
	:ndFileFormatJoint(ndJointSlider::StaticClassName())
{
}

ndFileFormatJointSlider::ndFileFormatJointSlider(const char* const className)
	:ndFileFormatJoint(className)
{
}

void ndFileFormatJointSlider::SaveJoint(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndJointBilateralConstraint* const joint)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_JOINT_CLASS, ndJointSlider::StaticClassName());
	ndFileFormatJoint::SaveJoint(scene, classNode, joint);

	ndFloat32 spring;
	ndFloat32 damper;
	ndFloat32 regularizer;
	ndFloat32 minTwistPosit; 
	ndFloat32 maxTwistPosit;
	ndJointSlider* const exportJoint = (ndJointSlider*)joint;

	exportJoint->GetSpringDamper(regularizer, spring, damper);
	exportJoint->GetLimits(minTwistPosit, maxTwistPosit);

	xmlSaveParam(classNode, "offsetPosit", exportJoint->GetOffsetPosit());
	xmlSaveParam(classNode, "springConstant", spring);
	xmlSaveParam(classNode, "damperConstant", damper);
	xmlSaveParam(classNode, "springRegularizer", regularizer);
	xmlSaveParam(classNode, "minTwistPosit", minTwistPosit);
	xmlSaveParam(classNode, "maxTwistPosit", maxTwistPosit);
	xmlSaveParam(classNode, "limitState", exportJoint->GetLimitState() ? 1 : 0);
}

ndJointBilateralConstraint* ndFileFormatJointSlider::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap)
{
	ndJointSlider* const joint = new ndJointSlider();
	LoadJoint(node, bodyMap, joint);
	return joint;
}

void ndFileFormatJointSlider::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, ndJointBilateralConstraint* const joint)
{
	ndFileFormatJoint::LoadJoint((nd::TiXmlElement*)node->FirstChild(D_JOINT_CLASS), bodyMap, joint);

	ndJointSlider* const inportJoint = (ndJointSlider*)joint;

	ndFloat32 offsetPosit = xmlGetFloat(node, "offsetPosit");
	ndFloat32 spring = xmlGetFloat(node, "springConstant");
	ndFloat32 damper = xmlGetFloat(node, "damperConstant");
	ndFloat32 regularizer = xmlGetFloat(node, "springRegularizer");
	ndFloat32 minTwistPosit = xmlGetFloat(node, "minTwistPosit");
	ndFloat32 maxTwistPosit = xmlGetFloat(node, "maxTwistPosit");
	ndInt32 state = xmlGetInt(node, "limitState");

	inportJoint->SetOffsetPosit(offsetPosit);
	inportJoint->SetAsSpringDamper(regularizer, spring, damper);
	inportJoint->SetLimits(minTwistPosit, maxTwistPosit);
	inportJoint->SetLimitState(state ? true : false);
}