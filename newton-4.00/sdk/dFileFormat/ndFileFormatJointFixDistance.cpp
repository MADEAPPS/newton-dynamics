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
#include "ndFileFormatJointFixDistance.h"

ndFileFormatJointFixDistance::ndFileFormatJointFixDistance()
	:ndFileFormatJoint(ndJointFixDistance::StaticClassName())
{
}

ndFileFormatJointFixDistance::ndFileFormatJointFixDistance(const char* const className)
	:ndFileFormatJoint(className)
{
}

void ndFileFormatJointFixDistance::SaveJoint(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndJointBilateralConstraint* const joint)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_JOINT_CLASS, ndJointFixDistance::StaticClassName());
	ndFileFormatJoint::SaveJoint(scene, classNode, joint);

	ndJointFixDistance* const exportJoint = (ndJointFixDistance*)joint;
	xmlSaveParam(classNode, "distance", exportJoint->GetDistance());
}

ndJointBilateralConstraint* ndFileFormatJointFixDistance::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap)
{
	ndJointFixDistance* const joint = new ndJointFixDistance();
	LoadJoint(node, bodyMap, joint);
	return joint;
}

void ndFileFormatJointFixDistance::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, ndJointBilateralConstraint* const joint)
{
	ndFileFormatJoint::LoadJoint((nd::TiXmlElement*)node->FirstChild(D_JOINT_CLASS), bodyMap, joint);

	ndJointFixDistance* const inportJoint = (ndJointFixDistance*)joint;

	ndFloat32 distance = xmlGetFloat(node, "distance");
	inportJoint->SetDistance(distance);
}