/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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
#include "ndSkyBox.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndDemoEntityManager.h"

class ndArchimedesBuoyancyVolume : public ndBodyTriggerVolume
{
	public:
	ndArchimedesBuoyancyVolume()
		:ndBodyTriggerVolume()
		,m_plane(dVector::m_zero)
		,m_density(1.0f)
		,m_hasPlane(0)
	{
	}

	void CalculatePlane(ndBodyKinematic* const body)
	{
		class ndCastTriggerPlane : public ndRayCastClosestHitCallback
		{
			public:
			ndCastTriggerPlane(const ndScene* const scene)
				:ndRayCastClosestHitCallback(scene)
			{
			}

			dUnsigned32 OnRayPrecastAction(const ndBody* const body, const ndShapeInstance* const collision)
			{
				return ((ndBody*)body)->GetAsBodyTriggerVolume() ? 1 : 0;
			}
		};

		dMatrix matrix(body->GetMatrix());

		dVector p0(matrix.m_posit);
		dVector p1(matrix.m_posit);

		p0.m_y += 30.0f;
		p1.m_y -= 30.0f;

		ndCastTriggerPlane rayCaster(body->GetScene());
		dFloat32 param = rayCaster.TraceRay(p0, p1);
		param = rayCaster.TraceRay(p0, p1);
		bool hasPlane = false;
		if (param < 1.0f)
		{
			hasPlane = true;
			dFloat32 dist = -rayCaster.m_contact.m_normal.DotProduct(rayCaster.m_contact.m_point).GetScalar();
			m_plane = dPlane(rayCaster.m_contact.m_normal, dist);
		}
		m_hasPlane = hasPlane;
	}

	void OnTriggerEnter(ndBodyKinematic* const body, dFloat32 timestep)
	{
		CalculatePlane(body);
	}
	
	void OnTrigger(ndBodyKinematic* const kinBody, dFloat32 timestep)
	{
		ndBodyDynamic* const body = kinBody->GetAsBodyDynamic();
		if (!m_hasPlane)
		{
			CalculatePlane(body);
		}

		if (body && m_hasPlane && (body->GetInvMass() != 0.0f))
		{
			dVector mass (body->GetMassMatrix());
			dVector centerOfPreasure(dVector::m_zero);
			dMatrix matrix (body->GetMatrix());
			ndShapeInstance& collision = body->GetCollisionShape();

			dFloat32 volume = collision.CalculateBuoyancyCenterOfPresure (centerOfPreasure, matrix, m_plane);
			if (volume > 0.0f)
			{
				// if some part of the shape si under water, calculate the buoyancy force base on 
				// Archimedes's buoyancy principle, which is the buoyancy force is equal to the 
				// weight of the fluid displaced by the volume under water. 
				//dVector cog(dVector::m_zero);
				const dFloat32 viscousDrag = 0.99f;

				ndShapeMaterial material(collision.GetMaterial());
				body->GetCollisionShape().SetMaterial(material);
				dFloat32 density = material.m_userParam[0].m_floatData;
				dFloat32 desplacedVolume = density * collision.GetVolume();
				
				dFloat32 displacedMass = mass.m_w * volume / desplacedVolume;
				dVector cog(body->GetCentreOfMass());
				centerOfPreasure -= matrix.TransformVector(cog);
				
				// now with the mass and center of mass of the volume under water, calculate buoyancy force and torque
				dVector force(dFloat32(0.0f), dFloat32(-DEMO_GRAVITY * displacedMass), dFloat32(0.0f), dFloat32(0.0f));
				dVector torque(centerOfPreasure.CrossProduct(force));
				
				body->SetForce(body->GetForce() + force);
				body->SetTorque(body->GetTorque() + torque);
				
				// apply a fake viscous drag to damp the under water motion 
				dVector omega(body->GetOmega());
				dVector veloc(body->GetVelocity());
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

	void OnTriggerExit(ndBodyKinematic* const body, dFloat32 timestep)
	{
		//dTrace(("exit trigger body: %d\n", body->GetId()));
	}

	dPlane m_plane;
	dFloat32 m_density;
	bool m_hasPlane;
};

static void BuildFloor(ndDemoEntityManager* const scene)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	ndShapeInstance box(new ndShapeBox(200.0f, 1.0f, 200.f));
	dMatrix uvMatrix(dGetIdentityMatrix());
	uvMatrix[0][0] *= 0.025f;
	uvMatrix[1][1] *= 0.025f;
	uvMatrix[2][2] *= 0.025f;
	ndDemoMesh* const geometry = new ndDemoMesh("box", scene->GetShaderCache(), &box, "marbleCheckBoard.tga", "marbleCheckBoard.tga", "marbleCheckBoard.tga", 1.0f, uvMatrix);
	
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit.m_y = -0.5f;
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(geometry, dGetIdentityMatrix());
	
	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(box);
	
	world->AddBody(body);
	
	scene->AddEntity(entity);
	geometry->Release();
}

static void AddTrigger(ndDemoEntityManager* const scene)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	ndShapeInstance box(new ndShapeBox(20.0f, 10.0f, 20.0f));
	dMatrix uvMatrix(dGetIdentityMatrix());
	uvMatrix[0][0] *= 1.0f / 20.0f;
	uvMatrix[1][1] *= 1.0f / 10.0f;
	uvMatrix[2][2] *= 1.0f / 20.0f;
	uvMatrix.m_posit = dVector(0.5f, 0.5f, 0.5f, 1.0f);
	ndDemoMesh* const geometry = new ndDemoMesh("trigger", scene->GetShaderCache(), &box, "metal_30.tga", "metal_30.tga", "logo_php.tga", 0.5f, uvMatrix);

	dVector floor(FindFloor(*world, dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = floor;
	matrix.m_posit.m_w = 1.0f;
	matrix.m_posit.m_y += 2.0f;

	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(geometry, dGetIdentityMatrix());

	//ndBodyDynamic* const body = new ndBodyDynamic();
	ndBodyTriggerVolume* const body = new ndArchimedesBuoyancyVolume();
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(box);

	world->AddBody(body);

	scene->AddEntity(entity);
	geometry->Release();
}

static void AddShape(ndDemoEntityManager* const scene, const dMatrix& location,
	const ndShapeInstance& shape, dFloat32 mass, dFloat32 density)
{
	ndDemoMesh* const mesh = new ndDemoMesh("shape", scene->GetShaderCache(), &shape, "marble.tga", "marble.tga", "marble.tga");

	dMatrix matrix(location);
	ndPhysicsWorld* const world = scene->GetWorld();

	dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y + 10.0f;

	ndBodyDynamic* const body = new ndBodyDynamic();
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(mesh, dGetIdentityMatrix());

	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(shape);
	body->SetMassMatrix(mass, shape);
	body->SetGyroMode(true);

	// save the density with the body shape.
	ndShapeMaterial material;
	material.m_userParam[0].m_floatData = density;
	body->GetCollisionShape().SetMaterial(material);

	world->AddBody(body);
	scene->AddEntity(entity);

	mesh->Release();
}

static void AddSphere(ndDemoEntityManager* const scene, const dVector& origin)
{
	dFloat32 diameter = 0.5f;
	ndShapeInstance shape(new ndShapeSphere(diameter));

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;

	AddShape(scene, matrix, shape, 10.0f, 1.1f);
}

static void AddCapsule(ndDemoEntityManager* const scene, const dVector& origin)
{
	dFloat32 diameter = 1.0f;
	ndShapeInstance shape(new ndShapeCapsule(diameter * 0.5f, diameter * 0.5f, diameter * 1.0f));

	dMatrix matrix(dRollMatrix(90.0f * dDegreeToRad));
	matrix.m_posit = origin;

	AddShape(scene, matrix, shape, 10.0f, 1.0f);
}

static void AddBox(ndDemoEntityManager* const scene, const dVector& origin)
{
	ndShapeInstance shape(new ndShapeBox(1.0f, 2.0f, 0.7f));
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;

	AddShape(scene, matrix, shape, 10.0f, 0.9f);
}

static void AddConvexHull(ndDemoEntityManager* const scene, const dVector& origin)
{
	dVector points[1024];
	const dInt32 pointCount = 10;
	dInt32 count = 0;
	for (dInt32 i = 0; i < pointCount; i++)
	{
		dFloat32 y = 0.7f * dCos((dFloat32(2.0f) * dPi) * i / pointCount);
		dFloat32 z = 0.7f * dSin((dFloat32(2.0f) * dPi) * i / pointCount);
		points[count++] = dVector(-0.5f, 0.7f * y, 0.7f* z, 0.0f);
		points[count++] = dVector( 0.5f, 0.7f * y, 0.7f* z, 0.0f);
		points[count++] = dVector(0.25f, y, z, 0.0f);
		points[count++] = dVector(-0.25f, y, z, 0.0f);
	}

	//ndShapeInstance shape(new ndShapeBox(1.0f, 2.0f, 0.7f));
	ndShapeInstance shape(new ndShapeConvexHull(count, sizeof (dVector), 0.0f, &points[0].m_x));
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;

	AddShape(scene, matrix, shape, 10.0f, 0.8f);
}


void ndBasicTrigger (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloor(scene);

	// build a floor
	AddTrigger(scene);

	AddBox(scene, dVector(0.0f, 0.0f, -3.0f, 1.0f));
	AddSphere(scene, dVector(0.0f, 0.0f, 0.0f, 1.0f));
	AddCapsule(scene, dVector(0.0f, 0.0f, 3.0f, 1.0f));
	AddConvexHull(scene, dVector(-2.0f, 0.0f, -2.0f, 1.0f));

	dQuaternion rot;
	dVector origin(-40.0f, 5.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
