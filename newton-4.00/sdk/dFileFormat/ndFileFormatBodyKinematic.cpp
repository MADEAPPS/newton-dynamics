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
#include "ndFileFormatBodyKinematic.h"

ndFileFormatBodyKinematic::ndFileFormatBodyKinematic()
	:ndFileFormatBody(ndBodyKinematic::StaticClassName())
{
}

ndFileFormatBodyKinematic::ndFileFormatBodyKinematic(const char* const className)
	: ndFileFormatBody(className)
{
}

void ndFileFormatBodyKinematic::SaveBody(ndFileFormatSave* const scene, nd::TiXmlElement* const parentNode, const ndBody* const body)
{
	nd::TiXmlElement* const classNode = xmlCreateClassNode(parentNode, D_BODY_CLASS, ndBodyKinematic::StaticClassName());
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
		ndVector euler1;
		ndVector inertia(kinematic->GetMassMatrix());
		ndMatrix matrix(kinematic->GetPrincipalAxis());
		ndVector euler0 (matrix.CalcPitchYawRoll(euler1));
		euler0 = euler0.Scale(ndRadToDegree);

		xmlSaveParam(classNode, "inertia", inertia);
		xmlSaveParam(classNode, "principalAxis", euler0);
		xmlSaveParam(classNode, "useSkewInertia", matrix.TestIdentity() ? 0 : 1);
	}
	
	xmlSaveParam(classNode, "maxLinearStep", kinematic->GetMaxLinearStep());
	xmlSaveParam(classNode, "maxAngleStep", kinematic->GetMaxAngularStep() * ndRadToDegree);
}

ndBody* ndFileFormatBodyKinematic::LoadBody(const nd::TiXmlElement* const node, const ndTree<ndShape*, ndInt32>& shapeMap)
{
	ndBodyKinematic* const body = new ndBodyKinematic();
	LoadBody(node, shapeMap, body);
	return body;
}

void ndFileFormatBodyKinematic::LoadBody(const nd::TiXmlElement* const node, const ndTree<ndShape*, ndInt32>& shapeMap, ndBody* const body)
{
	ndFileFormatBody::LoadBody((nd::TiXmlElement*)node->FirstChild(D_BODY_CLASS), shapeMap, body);
	
	ndBodyKinematic* const kinBody = ((ndBody*)body)->GetAsBodyKinematic();
	
	ndFileFormatRegistrar* const collisionHandler = ndFileFormatRegistrar::GetHandler(ndShapeInstance::StaticClassName());
	ndAssert(collisionHandler);
	ndSharedPtr<ndShapeInstance> instance(collisionHandler->LoadCollision((nd::TiXmlElement*)node->FirstChild(D_INSTANCE_CLASS), shapeMap));
	kinBody->SetCollisionShape(*(*instance));
	
	ndFloat32 invMass = xmlGetFloat(node, "invMass");
	if (invMass > ndFloat32(0.0f))
	{
		ndInt32 useSkew = xmlGetInt(node, "useSkewInertia");
		ndVector inertia(xmlGetVector3(node, "inertia"));
		ndMatrix II(ndGetIdentityMatrix());
		II[0][0] = inertia.m_x;
		II[1][1] = inertia.m_y;
		II[2][2] = inertia.m_z;
		if (useSkew)
		{
			ndVector euler(xmlGetVector3(node, "principalAxis"));
			ndMatrix principalAxisMatrix (ndPitchMatrix(euler.m_x * ndDegreeToRad) * ndYawMatrix(euler.m_y * ndDegreeToRad) * ndRollMatrix(euler.m_z * ndDegreeToRad));
			principalAxisMatrix = principalAxisMatrix * II * principalAxisMatrix.OrthoInverse();
		}
		kinBody->SetMassMatrix(ndFloat32(1.0f) / invMass, II);
	}
	
	ndFloat32 stepInUnitPerSeconds = xmlGetFloat(node, "maxLinearStep");
	ndFloat32 angleInRadian = xmlGetFloat(node, "maxAngleStep") * ndDegreeToRad;
	kinBody->SetDebugMaxLinearAndAngularIntegrationStep(angleInRadian, stepInUnitPerSeconds);
}