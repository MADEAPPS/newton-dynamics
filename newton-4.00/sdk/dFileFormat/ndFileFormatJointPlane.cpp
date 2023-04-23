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
#include "ndFileFormatJointPlane.h"

ndFileFormatJointPlane::ndFileFormatJointPlane()
	:ndFileFormatJoint(ndJointPlane::StaticClassName())
{
}

ndFileFormatJointPlane::ndFileFormatJointPlane(const char* const className)
	:ndFileFormatJoint(className)
{
}

void ndFileFormatJointPlane::SaveJoint(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndJointBilateralConstraint* const joint)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_JOINT_CLASS, ndJointPlane::StaticClassName());
	ndFileFormatJoint::SaveJoint(scene, classNode, joint);

	ndJointPlane* const exportJoint = (ndJointPlane*)joint;
	xmlSaveParam(classNode, "ControlRotation", exportJoint->GetEnableControlRotation() ? 1 : 0);
}

ndJointBilateralConstraint* ndFileFormatJointPlane::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap)
{
	ndJointPlane* const joint = new ndJointPlane();
	LoadJoint(node, bodyMap, joint);
	return joint;
}

void ndFileFormatJointPlane::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, ndJointBilateralConstraint* const joint)
{
	ndFileFormatJoint::LoadJoint((nd::TiXmlElement*)node->FirstChild(D_JOINT_CLASS), bodyMap, joint);

	ndJointPlane* const inportJoint = (ndJointPlane*)joint;

	ndInt32 controlRotation = xmlGetInt(node, "ControlRotation");
	inportJoint->EnableControlRotation(controlRotation ? true : false);
}