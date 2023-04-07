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

void ndFileFormatJointDoubleHinge::SaveJoint(ndFileFormat* const scene, nd::TiXmlElement* const parentNode, const ndJointBilateralConstraint* const joint)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, "ndJointDoubleHinge", ndJointDoubleHinge::StaticClassName());
	ndFileFormatJoint::SaveJoint(scene, classNode, joint);

	//ndFloat32 spring0;
	//ndFloat32 damper0;
	//ndFloat32 regularizer0;
	ndFloat32 minTwistAngle0; 
	ndFloat32 maxTwistAngle0;
	ndFloat32 minTwistAngle1;
	ndFloat32 maxTwistAngle1;

	ndJointDoubleHinge* const exportJoint = (ndJointDoubleHinge*)joint;
	
	//exportJoint->GetSpringDamper(regularizer, spring, damper);
	exportJoint->GetLimits0(minTwistAngle0, maxTwistAngle0);
	exportJoint->GetLimits1(minTwistAngle1, maxTwistAngle1);
	
	xmlSaveParam(classNode, "offsetAngle0", exportJoint->GetOffsetAngle0() * ndRadToDegree);
	//xmlSaveParam(classNode, "springConstant", spring);
	//xmlSaveParam(classNode, "damperConstant", damper);
	//xmlSaveParam(classNode, "springRegularizer", regularizer);
	xmlSaveParam(classNode, "minTwistAngle0", minTwistAngle0 * ndRadToDegree);
	xmlSaveParam(classNode, "maxTwistAngle0", minTwistAngle0 * ndRadToDegree);
	//xmlSaveParam(classNode, "limitState", exportJoint->GetLimitState() ? 1 : 0);

	xmlSaveParam(classNode, "offsetAngle1", exportJoint->GetOffsetAngle1() * ndRadToDegree);

	xmlSaveParam(classNode, "minTwistAngle1", minTwistAngle1 * ndRadToDegree);
	xmlSaveParam(classNode, "maxTwistAngle1", minTwistAngle1 * ndRadToDegree);
}
