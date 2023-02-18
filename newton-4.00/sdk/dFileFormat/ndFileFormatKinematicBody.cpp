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
#include "ndFileFormatKinematicBody.h"

ndFileFormatKinematicBody::ndFileFormatKinematicBody()
	:ndFileFormatBody(ndBodyKinematic::StaticClassName())
{
}

ndFileFormatKinematicBody::ndFileFormatKinematicBody(const char* const className)
	: ndFileFormatBody(className)
{
}

void ndFileFormatKinematicBody::SaveBody(nd::TiXmlElement* const parentNode, ndBody* const body)
{
	nd::TiXmlElement* const classNode = new nd::TiXmlElement(ndBodyKinematic::StaticClassName());
	parentNode->LinkEndChild(classNode);
	ndFileFormatBody::SaveBody(classNode, body);

	ndBodyKinematic* const kinematic = body->GetAsBodyKinematic();
	ndAssert(kinematic);
	
	ndFloat32 invMass = kinematic->GetInvMass();
	xmlSaveParam(classNode, "invMass", invMass);
	if (invMass > ndFloat32(0.0f))
	{
		ndVector euler0;
		ndVector euler1;
		ndVector inertia(kinematic->GetMassMatrix());
		xmlSaveParam(classNode, "inertia", inertia);
		kinematic->GetPrincipalAxis().CalcPitchYawRoll(euler0, euler1);
		xmlSaveParam(classNode, "principalAxis", euler0);
	}
	xmlSaveParam(classNode, "maxAngleStep", kinematic->GetMaxAngularStep());
	xmlSaveParam(classNode, "maxLinearStep", kinematic->GetMaxLinearStep());

	//m_shapeInstance.Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

}
