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
#include "ndSkyBox.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndContactCallback.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndArchimedesBuoyancyVolume.h"


static void AddBox(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 density, ndFloat32 mass)
{
	ndBodyKinematic* const body = AddBox(scene, origin, mass, 1.0f, 1.0f, 1.0f);
	ndShapeMaterial material;
	material.m_userParam[ndDemoContactCallback::m_density].m_floatData = density;
	body->GetCollisionShape().SetMaterial(material);
}

static void AddSphere1(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 density, ndFloat32 mass)
{
	ndBodyKinematic* const body = AddSphere(scene, origin, mass, 0.5f);
	ndShapeMaterial material;
	material.m_userParam[ndDemoContactCallback::m_density].m_floatData = density;
	body->GetCollisionShape().SetMaterial(material);
}

static void AddCapsule(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 density, ndFloat32 mass)
{
	ndBodyKinematic* const body = AddCapsule(scene, origin, mass, 0.5f, 0.5f, 1.0f);
	ndShapeMaterial material;
	material.m_userParam[ndDemoContactCallback::m_density].m_floatData = density;
	body->GetCollisionShape().SetMaterial(material);
}

static void AddConvexHull(ndDemoEntityManager* const scene, const ndMatrix& origin, const ndInt32 segments, ndFloat32 radius, ndFloat32 high, ndFloat32 density, ndFloat32 mass)
{
	ndBodyKinematic* const body = AddConvexHull(scene, origin, mass, radius, high, segments);
	ndShapeMaterial material;
	material.m_userParam[ndDemoContactCallback::m_density].m_floatData = density;
	body->GetCollisionShape().SetMaterial(material);
}


class NewtonPhantom : public ndModel
{
	// A Phantom collision shape can be moved around the world, gathering contact
	// information with other ndBody's without effecting the simulation

public:
	NewtonPhantom(ndScene* scene) :
		ndModel(),
		phantomShape(new ndShapeBox(1.0f, 1.0f, 1.0f)),
		worldMatrix(ndGetIdentityMatrix()),
		notification(scene)
	{
	}
	~NewtonPhantom() = default;

	void OnAddToWorld() override
	{

	}

	void OnRemoveFromToWorld() override
	{

	}

	void transform(const ndMatrix& matrix) { worldMatrix = matrix; }
	ndInt32 getContactCount() const { return contactCount; }
	ndVector getContactPoint() const { return contactPoint; }


	void Update(ndWorld* const world, ndFloat32) override
	{
		
		// calc the current AABB in world space
		ndVector boxMin;
		ndVector boxMax;
		phantomShape.CalculateAabb(worldMatrix, boxMin, boxMax);

		ndBodiesInAabbNotify notifyCallback;
		world->BodiesInAabb(notifyCallback, boxMin, boxMax);

		for (ndInt32 i = 0; i < notifyCallback.m_bodyArray.GetCount(); ++i)
		{
			ndBody* const nbody = const_cast<ndBody*> (notifyCallback.m_bodyArray[i]);
			ndBodyKinematic* const kBody = nbody->GetAsBodyKinematic();

			const ndShapeInstance& otherShape = kBody->GetCollisionShape();
			const ndMatrix& otherMatrix = notifyCallback.m_bodyArray[i]->GetMatrix();

			// ignore self collision
			if (otherShape.GetShape() != phantomShape.GetShape())
			{
				ndFixSizeArray<ndContactPoint, 16> contactBuffer;

				ndVector phantomVelocity = ndVector::m_zero;
				ndVector otherVelocity = ndVector::m_zero;

				ndContactSolver contSolver;
				contSolver.CalculateContacts(&phantomShape, worldMatrix, phantomVelocity, &otherShape, otherMatrix, otherVelocity, contactBuffer, &notification);
				contactCount = contactBuffer.GetCount();

				// 
				std::cout << contactCount << std::endl;

				
				if (contactCount)
				{
					for (int j = 0; j < contactCount; ++j)
					{
						const ndContactPoint& cPnt = contactBuffer[j];
						contactPoint = cPnt.m_point;
					}
				}
			}
		}
	}
	void PostUpdate(ndWorld* const, ndFloat32) override
	{
	}
	void PostTransformUpdate(ndWorld* const, ndFloat32) override
	{
	}

private:
	ndShapeInstance phantomShape;
	ndMatrix worldMatrix;
	ndContactNotify notification;
	ndInt32 contactCount = 0;
	ndVector contactPoint = ndVector(std::numeric_limits<float>::max());
}; // end class NewtonPhantom


void ndObjectPlacement(ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildFloorBox(scene, ndGetIdentityMatrix());
	BuildFlatPlane(scene, true);
	class PlaceMatrix : public ndMatrix
	{
	public:
		PlaceMatrix(ndFloat32 x, ndFloat32 y, ndFloat32 z)
			:ndMatrix(ndGetIdentityMatrix())
		{
			m_posit.m_x = x;
			m_posit.m_y = y;
			m_posit.m_z = z;
		}
	};

	AddBox(scene, PlaceMatrix(0.0f, 20.0f, -3.0f), 0.6f, 10.0f);
	// create a Phantom model that contains a collision shape and transform matrix
	NewtonPhantom* const phantom = new NewtonPhantom(scene->GetWorld()->GetScene());
	ndSharedPtr<ndModel> phantomPtr(phantom);
	scene->GetWorld()->AddModel(phantomPtr);
	//AddSphere1(scene, PlaceMatrix(0.0f, 5.0f, 0.0f), 0.5f, 10.0f);
	//AddCapsule(scene, PlaceMatrix(0.0f, 5.0f, 3.0f), 0.7f, 10.0f);
	//AddConvexHull(scene, PlaceMatrix(-2.0f, 5.0f, -2.0f), 7, 1.0f, 1.5f, 0.8f, 10.0f);
	//AddConvexHull(scene, PlaceMatrix(-2.0f, 5.0f,  2.0f), 21, 1.0f, 1.5f, 0.7f, 10.0f);
	//AddConvexHull(scene, PlaceMatrix( 2.0f, 5.0f,  3.0f), 210, 1.0f, 1.5f, 0.9f, 10.0f);

	ndQuaternion rot;
	ndVector origin(-40.0f, 5.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
