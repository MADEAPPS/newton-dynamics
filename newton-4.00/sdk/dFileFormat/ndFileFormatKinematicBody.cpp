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

void ndFileFormatKinematicBody::SaveBody(ndFileFormat* const scene, nd::TiXmlElement* const parentNode, const ndBody* const body)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, "ndBodyClass", ndBodyKinematic::StaticClassName());
	ndFileFormatBody::SaveBody(scene, classNode, body);

	ndBodyKinematic* const kinematic = ((ndBody*)body)->GetAsBodyKinematic();
	ndAssert(kinematic);

	const ndShapeInstance* const collision = &kinematic->GetCollisionShape();
	ndFileFormatRegistrar* const handler = ndFileFormatRegistrar::GetHandler(collision->ClassName());
	ndAssert(handler);
	handler->SaveCollision(scene, classNode, collision);

	ndFloat32 invMass = kinematic->GetInvMass();
	xmlSaveParam(classNode, "invMass", invMass);
	if (invMass > ndFloat32(0.0f))
	{
		ndVector euler0;
		ndVector euler1;
		ndVector inertia(kinematic->GetMassMatrix());
		ndMatrix matrix(kinematic->GetPrincipalAxis());
		matrix.CalcPitchYawRoll(euler0, euler1);
		euler0 = euler0.Scale(ndRadToDegree);

		xmlSaveParam(classNode, "inertia", inertia);
		xmlSaveParam(classNode, "principalAxis", euler0);
		xmlSaveParam(classNode, "useSkewInertia", matrix.TestIdentity() ? 0 : 1);
	}
	
	xmlSaveParam(classNode, "maxLinearStep", kinematic->GetMaxLinearStep());
	xmlSaveParam(classNode, "maxAngleStep", kinematic->GetMaxAngularStep() * ndRadToDegree);
}

void ndFileFormatKinematicBody::LoadBody(const nd::TiXmlElement* const node, const ndTree<ndShape*, ndInt32>& shapeMap, ndBody* const body)
{
	ndFileFormatBody::LoadBody((nd::TiXmlElement*)node->FirstChild("ndBodyClass"), shapeMap, body);

	ndBodyKinematic* const kinBody = (ndBodyKinematic*)body;



	ndFloat32 invMass = xmlGetFloat(node, "invMass");
	if (invMass > ndFloat32(0.0f))
	{
		ndAssert(0);
		//ndVector euler0;
		//ndVector euler1;
		//ndVector inertia(kinematic->GetMassMatrix());
		//ndMatrix matrix(kinematic->GetPrincipalAxis());
		//matrix.CalcPitchYawRoll(euler0, euler1);
		//euler0 = euler0.Scale(ndRadToDegree);
		//
		//xmlSaveParam(classNode, "inertia", inertia);
		//xmlSaveParam(classNode, "principalAxis", euler0);
		//xmlSaveParam(classNode, "useSkewInertia", matrix.TestIdentity() ? 0 : 1);
	}

	ndFloat32 stepInUnitPerSeconds = xmlGetFloat(node, "maxLinearStep");
	ndFloat32 angleInRadian = xmlGetFloat(node, "maxAngleStep") * ndDegreeToRad;
	kinBody->SetDebugMaxLinearAndAngularIntegrationStep(angleInRadian, stepInUnitPerSeconds);
}