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
#include "ndFileFormatSave.h"
#include "ndFileFormatJointVehicleTorsionBar.h"

ndFileFormatJointVehicleTorsionBar::ndFileFormatJointVehicleTorsionBar()
	:ndFileFormatJoint(ndMultiBodyVehicleTorsionBar::StaticClassName())
{
}

ndFileFormatJointVehicleTorsionBar::ndFileFormatJointVehicleTorsionBar(const char* const className)
	:ndFileFormatJoint(className)
{
}

void ndFileFormatJointVehicleTorsionBar::SaveJoint(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndJointBilateralConstraint* const joint)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_JOINT_CLASS, ndMultiBodyVehicleTorsionBar::StaticClassName());
	ndFileFormatJoint::SaveJoint(scene, classNode, joint);

	ndFloat32 spring;
	ndFloat32 damper;
	ndFloat32 regularizer;

	ndMultiBodyVehicleTorsionBar* const exportJoint = (ndMultiBodyVehicleTorsionBar*)joint;
	exportJoint->GetTorsionTorque(spring, damper, regularizer);
	xmlSaveParam(classNode, "springConstant", spring);
	xmlSaveParam(classNode, "damperConstant", damper);
	xmlSaveParam(classNode, "springRegularizer", regularizer);

	ndAssert(0);
	const ndFixSizeArray<ndMultiBodyVehicleTorsionBar::ndAxles, 2>& axles = exportJoint->GetAxels();
	if (axles.GetCount())
	{
		nd::TiXmlElement* const axleNode = new nd::TiXmlElement("axleCount");
		classNode->LinkEndChild(axleNode);
		axleNode->SetAttribute("count", axles.GetCount());
		for (ndInt32 i = 0; i < axles.GetCount(); i++)
		{
			nd::TiXmlElement* const axleNodePair = new nd::TiXmlElement("axle");
			axleNode->LinkEndChild(axleNodePair);

			ndInt32 body0NodeId = scene->FindBodyId(axles[i].m_leftTire);
			ndInt32 body1NodeId = scene->FindBodyId(axles[i].m_rightTire);

			xmlSaveParam(axleNodePair, "leftTireBody", body0NodeId);
			xmlSaveParam(axleNodePair, "rightTireBody", body1NodeId);
		}
	}
}

ndJointBilateralConstraint* ndFileFormatJointVehicleTorsionBar::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap)
{
	ndMultiBodyVehicleTorsionBar* const joint = new ndMultiBodyVehicleTorsionBar();
	LoadJoint(node, bodyMap, joint);
	return joint;
}

void ndFileFormatJointVehicleTorsionBar::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, ndJointBilateralConstraint* const joint)
{
	ndFileFormatJoint::LoadJoint((nd::TiXmlElement*)node->FirstChild(D_JOINT_CLASS), bodyMap, joint);

	ndAssert(0);
	//ndMultiBodyVehicleTorsionBar* const importJoint = (ndMultiBodyVehicleTorsionBar*)joint;
	//ndFloat32 idleOmega = xmlGetFloat(node, "idleOmega");
	//ndFloat32 clutchTorque = xmlGetFloat(node, "clutchTorque");
	//ndFloat32 internalTorqueLoss = xmlGetFloat(node, "internalTorqueLoss");
	//
	//importJoint->SetIdleOmega(idleOmega);
	//importJoint->SetClutchTorque(clutchTorque);
	//importJoint->SetInternalTorqueLoss(internalTorqueLoss);
}
