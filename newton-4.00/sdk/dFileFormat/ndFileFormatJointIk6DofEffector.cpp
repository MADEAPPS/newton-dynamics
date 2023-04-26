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
#include "ndFileFormatJointIk6DofEffector.h"

ndFileFormatJointIk6DofEffector::ndFileFormatJointIk6DofEffector()
	:ndFileFormatJoint(ndIk6DofEffector::StaticClassName())
{
}

ndFileFormatJointIk6DofEffector::ndFileFormatJointIk6DofEffector(const char* const className)
	:ndFileFormatJoint(className)
{
}

void ndFileFormatJointIk6DofEffector::SaveJoint(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndJointBilateralConstraint* const joint)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_JOINT_CLASS, ndIk6DofEffector::StaticClassName());
	ndFileFormatJoint::SaveJoint(scene, classNode, joint);

	ndFloat32 spring0;
	ndFloat32 damper0;
	ndFloat32 regularizer0;
	ndFloat32 spring1;
	ndFloat32 damper1;
	ndFloat32 regularizer1;

	ndIk6DofEffector* const exportJoint = (ndIk6DofEffector*)joint;

	exportJoint->GetLinearSpringDamper(regularizer0, spring0, damper0);
	xmlSaveParam(classNode, "targetFrame", exportJoint->GetOffsetMatrix());

	xmlSaveParam(classNode, "linearSpringConstant", spring0);
	xmlSaveParam(classNode, "linearDamperConstant", damper0);
	xmlSaveParam(classNode, "linearSpringRegularizer", regularizer0);
	xmlSaveParam(classNode, "maxForce", exportJoint->GetMaxForce());

	exportJoint->GetAngularSpringDamper(regularizer1, spring1, damper1);
	xmlSaveParam(classNode, "angularSpringConstant", spring1);
	xmlSaveParam(classNode, "angularDamperConstant", damper1);
	xmlSaveParam(classNode, "angularSpringRegularizer", regularizer1);
	xmlSaveParam(classNode, "maxTorque", exportJoint->GetMaxTorque());

	xmlSaveParam(classNode, "axisX", exportJoint->GetAxisX() ? 1 : 0);
	xmlSaveParam(classNode, "axisY", exportJoint->GetAxisY() ? 1 : 0);
	xmlSaveParam(classNode, "axisZ", exportJoint->GetAxisZ() ? 1 : 0);
	xmlSaveParam(classNode, "rotationType", ndInt32(exportJoint->GetRotationAxis()));
}

ndJointBilateralConstraint* ndFileFormatJointIk6DofEffector::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap)
{
	ndIk6DofEffector* const joint = new ndIk6DofEffector();
	LoadJoint(node, bodyMap, joint);
	return joint;
}

void ndFileFormatJointIk6DofEffector::LoadJoint(const nd::TiXmlElement* const node, const ndTree<ndSharedPtr<ndBody>, ndInt32>& bodyMap, ndJointBilateralConstraint* const joint)
{
	ndFileFormatJoint::LoadJoint((nd::TiXmlElement*)node->FirstChild(D_JOINT_CLASS), bodyMap, joint);

	ndIk6DofEffector* const inportJoint = (ndIk6DofEffector*)joint;

	ndMatrix targetFrame (xmlGetMatrix(node, "targetFrame"));

	ndFloat32 linearSpringConstant = xmlGetFloat(node, "linearSpringConstant");
	ndFloat32 linearDamperConstant = xmlGetFloat(node, "linearDamperConstant");
	ndFloat32 linearSpringRegularizer = xmlGetFloat(node, "linearSpringRegularizer");
	ndFloat32 maxForce = xmlGetFloat(node, "maxForce");

	ndFloat32 angularSpringConstant = xmlGetFloat(node, "angularSpringConstant");
	ndFloat32 angularDamperConstant = xmlGetFloat(node, "angularDamperConstant");
	ndFloat32 angularSpringRegularizer = xmlGetFloat(node, "angularSpringRegularizer");
	ndFloat32 maxTorque = xmlGetFloat(node, "maxTorque");

	ndInt32 axisX = xmlGetInt(node, "axisX");
	ndInt32 axisY = xmlGetInt(node, "axisY");
	ndInt32 axisZ = xmlGetInt(node, "axisZ");
	ndInt32 rotationType = xmlGetInt(node, "rotationType");

	inportJoint->SetMaxForce(maxForce);
	inportJoint->SetMaxTorque(maxTorque);
	inportJoint->SetOffsetMatrix(targetFrame);
	inportJoint->EnableAxisX(axisX ? true : false);
	inportJoint->EnableAxisY(axisY ? true : false);
	inportJoint->EnableAxisZ(axisZ ? true : false);
	inportJoint->EnableRotationAxis(ndIk6DofEffector::ndRotationType (rotationType));
	inportJoint->GetLinearSpringDamper(linearSpringRegularizer, linearSpringConstant, linearDamperConstant);
	inportJoint->GetLinearSpringDamper(angularSpringRegularizer, angularSpringConstant, angularDamperConstant);
}