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
#include "ndFileFormatJointIkSwivelPositionEffector.h"

ndFileFormatJointIkSwivelPositionEffector::ndFileFormatJointIkSwivelPositionEffector()
	:ndFileFormatJoint(ndIkSwivelPositionEffector::StaticClassName())
{
}

ndFileFormatJointIkSwivelPositionEffector::ndFileFormatJointIkSwivelPositionEffector(const char* const className)
	:ndFileFormatJoint(className)
{
}

void ndFileFormatJointIkSwivelPositionEffector::SaveJoint(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndJointBilateralConstraint* const joint)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, "ndJointClass", ndIkSwivelPositionEffector::StaticClassName());
	ndFileFormatJoint::SaveJoint(scene, classNode, joint);

	ndFloat32 spring0;
	ndFloat32 damper0;
	ndFloat32 regularizer0;
	ndFloat32 spring1;
	ndFloat32 damper1;
	ndFloat32 regularizer1;
	ndFloat32 minRadio;
	ndFloat32 maxRadio;

	ndIkSwivelPositionEffector* const exportJoint = (ndIkSwivelPositionEffector*)joint;

	exportJoint->GetLinearSpringDamper(regularizer0, spring0, damper0);
	xmlSaveParam(classNode, "linearSpringConstant", spring0);
	xmlSaveParam(classNode, "linearDamperConstant", damper0);
	xmlSaveParam(classNode, "linearSpringRegularizer", regularizer0);
	xmlSaveParam(classNode, "maxForce", exportJoint->GetMaxForce());

	exportJoint->GetAngularSpringDamper(regularizer1, spring1, damper1);
	xmlSaveParam(classNode, "angularSpringConstant", spring1);
	xmlSaveParam(classNode, "angularDamperConstant", damper1);
	xmlSaveParam(classNode, "angularSpringRegularizer", regularizer1);
	xmlSaveParam(classNode, "maxTorque", exportJoint->GetMaxTorque());

	exportJoint->GetWorkSpaceConstraints(minRadio, maxRadio);
	xmlSaveParam(classNode, "minWorkSpaceRadio", minRadio);
	xmlSaveParam(classNode, "maxWorkSpaceRadio", maxRadio);
	xmlSaveParam(classNode, "enableSwivelControl", exportJoint->GetSwivelMode() ? 1 : 0);
}
