/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndArchimedesBuoyancyVolume.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndArchimedesBuoyancyVolume);

ndArchimedesBuoyancyVolume::ndArchimedesBuoyancyVolume()
	:ndBodyTriggerVolume()
	,m_plane(ndVector::m_zero)
	,m_density(1.0f)
	,m_hasPlane(0)
{
}

ndArchimedesBuoyancyVolume::ndArchimedesBuoyancyVolume(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndBodyTriggerVolume(ndLoadSaveBase::ndLoadDescriptor(desc))
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;
	m_plane = xmlGetVector3(xmlNode, "planeNormal");
	m_plane.m_w = xmlGetFloat(xmlNode, "planeDist");
	m_density = xmlGetFloat(xmlNode, "density");
	m_hasPlane = xmlGetInt(xmlNode, "hasPlane") ? true : false;
}

void ndArchimedesBuoyancyVolume::CalculatePlane(ndBodyKinematic* const body)
{
	class ndCastTriggerPlane : public ndRayCastClosestHitCallback
	{
		public:
		ndCastTriggerPlane()
			:ndRayCastClosestHitCallback()
		{
		}

		ndUnsigned32 OnRayPrecastAction(const ndBody* const body, const ndShapeInstance* const)
		{
			return ((ndBody*)body)->GetAsBodyTriggerVolume() ? 1 : 0;
		}
	};

	ndMatrix matrix(body->GetMatrix());

	ndVector p0(matrix.m_posit);
	ndVector p1(matrix.m_posit);

	p0.m_y += 30.0f;
	p1.m_y -= 30.0f;

	ndCastTriggerPlane rayCaster;
	m_hasPlane = body->GetScene()->RayCast(rayCaster, p0, p1);
	if (m_hasPlane)
	{ 
		ndFloat32 dist = -rayCaster.m_contact.m_normal.DotProduct(rayCaster.m_contact.m_point).GetScalar();
		m_plane = ndPlane(rayCaster.m_contact.m_normal, dist);
	}
}

void ndArchimedesBuoyancyVolume::OnTriggerEnter(ndBodyKinematic* const body, ndFloat32)
{
	CalculatePlane(body);
}
	
void ndArchimedesBuoyancyVolume::OnTrigger(ndBodyKinematic* const kinBody, ndFloat32)
{
	ndBodyDynamic* const body = kinBody->GetAsBodyDynamic();
	if (!m_hasPlane)
	{
		CalculatePlane(body);
	}

	if (body && m_hasPlane && (body->GetInvMass() != 0.0f))
	{
		ndVector mass (body->GetMassMatrix());
		ndVector centerOfPreasure(ndVector::m_zero);
		ndMatrix matrix (body->GetMatrix());
		ndShapeInstance& collision = body->GetCollisionShape();

		ndFloat32 volume = collision.CalculateBuoyancyCenterOfPresure (centerOfPreasure, matrix, m_plane);
		if (volume > 0.0f)
		{
			// if some part of the shape si under water, calculate the buoyancy force base on 
			// Archimedes's buoyancy principle, which is the buoyancy force is equal to the 
			// weight of the fluid displaced by the volume under water. 
			//ndVector cog(ndVector::m_zero);
			const ndFloat32 viscousDrag = 0.99f;

			ndShapeMaterial material(collision.GetMaterial());
			body->GetCollisionShape().SetMaterial(material);
			ndFloat32 density = material.m_userParam[0].m_floatData;
			ndFloat32 desplacedVolume = density * collision.GetVolume();
				
			ndFloat32 displacedMass = mass.m_w * volume / desplacedVolume;
			ndVector cog(body->GetCentreOfMass());
			centerOfPreasure -= matrix.TransformVector(cog);
				
			// now with the mass and center of mass of the volume under water, calculate buoyancy force and torque
			ndVector force(ndFloat32(0.0f), ndFloat32(-DEMO_GRAVITY * displacedMass), ndFloat32(0.0f), ndFloat32(0.0f));
			ndVector torque(centerOfPreasure.CrossProduct(force));
				
			body->SetForce(body->GetForce() + force);
			body->SetTorque(body->GetTorque() + torque);
				
			// apply a fake viscous drag to damp the under water motion 
			ndVector omega(body->GetOmega());
			ndVector veloc(body->GetVelocity());
			omega = omega.Scale(viscousDrag);
			veloc = veloc.Scale(viscousDrag);
			body->SetOmega(omega);
			body->SetVelocity(veloc);
				
			//// test delete bodies inside trigger
			//collisionMaterial.m_userParam[1].m_float += timestep;
			//NewtonCollisionSetMaterial(collision, &collisionMaterial);
			//if (collisionMaterial.m_userParam[1].m_float >= 30.0f) {
			//	// delete body after 2 minutes inside the pool
			//	NewtonDestroyBody(visitor);
			//}
		}
	}
}

void ndArchimedesBuoyancyVolume::OnTriggerExit(ndBodyKinematic* const, ndFloat32)
{
	//dTrace(("exit trigger body: %d\n", body->GetId()));
}

void ndArchimedesBuoyancyVolume::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndBodyTriggerVolume::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "planeNormal", m_plane);
	xmlSaveParam(childNode, "planeDist", m_plane.m_w);
	xmlSaveParam(childNode, "density", m_density);
	xmlSaveParam(childNode, "hasPlane", m_hasPlane ? 1 : 0);
}

