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
#include "ndFileFormatJointPulley.h"

ndFileFormatJointPulley::ndFileFormatJointPulley()
	:ndFileFormatJoint(ndJointPulley::StaticClassName())
{
}

ndFileFormatJointPulley::ndFileFormatJointPulley(const char* const className)
	:ndFileFormatJoint(className)
{
}

void ndFileFormatJointPulley::SaveJoint(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndJointBilateralConstraint* const joint)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_JOINT_CLASS, ndJointPulley::StaticClassName());
	ndFileFormatJoint::SaveJoint(scene, classNode, joint);

	ndJointPulley* const exportJoint = (ndJointPulley*)joint;
	xmlSaveParam(classNode, "ratio", exportJoint->GetRatio());
}

ndJointBilateralConstraint* ndFileFormatJointPulley::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap)
{
	ndJointPulley* const joint = new ndJointPulley();
	LoadJoint(node, bodyMap, joint);
	return joint;
}

void ndFileFormatJointPulley::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, ndJointBilateralConstraint* const joint)
{
	ndFileFormatJoint::LoadJoint((nd::TiXmlElement*)node->FirstChild(D_JOINT_CLASS), bodyMap, joint);

	ndJointPulley* const inportJoint = (ndJointPulley*)joint;

	ndFloat32 ratio = xmlGetFloat(node, "ratio");
	inportJoint->SetRatio(ratio);
}