/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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
#include "ndPhysicsWorld.h"
#include "ndArchimedesBuoyancyVolume.h"

#define FLUID_VISCOSITY	ndFloat32 (0.995f)

ndArchimedesBuoyancyVolume::ndArchimedesBuoyancyVolume()
	:ndBodyTriggerVolume()
	,m_plane(ndVector::m_zero)
	,m_density(1.0f)
	,m_hasPlane(0)
{
	// gives the trigger some mass to make it acc as a dynamics body
	SetMassMatrix(1.0f, 1.0f, 1.0f, 1.0f);
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
			return ndUnsigned32(((ndBody*)body)->GetAsBodyTriggerVolume() ? 1 : 0);
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
	ndTrace(("enter trigger body %d\n", body->GetId()));
	CalculatePlane(body);
}

void ndArchimedesBuoyancyVolume::SpecialUpdate(ndFloat32 timestep)
{
	ndBodyTriggerVolume::SpecialUpdate(timestep);

	// here the app can do manipulate the body ex: setting velocity
	// in this examnple we do nothing
	//SetVelocity(ndVector(1.0f, 0.0f, 0.0f, 0.0f));
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
			// if some part of the shape is under water, 
			// calculate the buoyancy force base on Archimedes's buoyancy principle, 
			// which is the buoyancy force is equal to the weight of the fluid 
			// displaced by the volume under water. 
			const ndFloat32 viscousDrag = FLUID_VISCOSITY;

			ndShapeMaterial material(collision.GetMaterial());
			body->GetCollisionShape().SetMaterial(material);
			ndFloat32 density = material.m_userParam[ndDemoContactCallback::m_density].m_floatData;
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
		}
	}
}

void ndArchimedesBuoyancyVolume::OnTriggerExit(ndBodyKinematic* const body, ndFloat32)
{
	//ndTrace(("exit trigger body: %d\n", body->GetId()));
	ndExpandTraceMessage("exit trigger body: %d\n", body->GetId());
}
