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
	typedef void* RenderableNode;
	// NewtonPhantom: A class representing a phantom collision shape in a physics simulation.
	// Phantom shapes can interact with other physics bodies to gather collision data
	// without affecting the physics simulation itself.

	public:
	// Constructor: Initializes a new phantom shape with a given shape and scene.
	NewtonPhantom(ndScene* scene, ndShape* shape, RenderableNode phantomNode) :
		ndModel(),
		phantomNode(phantomNode),
		phantomShape(shape),
		worldMatrix(ndGetIdentityMatrix()),
		notification(scene)
	{
		// The constructor initializes the phantom shape, world matrix, and notification system.
	}
	~NewtonPhantom() = default;


	// GetContactCount: Returns the number of contact points detected.
	ndInt32 getContactCount() const
	{
		return contactCount;
	}

	// GetContactPoint: Returns the current contact point in the simulation.
	ndVector getContactPoint() const { return contactPoint; }

	// Update: Overridden function that updates the phantom's state in each simulation step.
	void Update(ndWorld* const, ndFloat32) override
	{
		//contactCount = -1;
		//ndMatrix phantomPose;
		//eigenToNewton(phantomNode->getSpaceTime().worldTransform, phantomPose);
		//worldMatrix = phantomPose;
		//
		//// Calculates the current Axis-Aligned Bounding Box (AABB) in world space.
		//ndVector boxMin, boxMax;
		//phantomShape.CalculateAabb(worldMatrix, boxMin, boxMax);
		//
		////LOG(DBUG) << "phantom min{ " << boxMin.GetX() << ", " << boxMin.GetY() << ", " << boxMin.GetZ() << "}";
		////LOG(DBUG) << "phantom max{ " << boxMax.GetX() << ", " << boxMax.GetY() << ", " << boxMax.GetZ() << "}";
		//
		//// Notify callback for bodies within the AABB.
		//ndBodiesInAabbNotify notifyCallback;
		//world->BodiesInAabb(notifyCallback, boxMin, boxMax);
		//
		//// Processes each body in the AABB.
		//for (ndInt32 i = 0; i < notifyCallback.m_bodyArray.GetCount(); ++i)
		//{
		//	ndBody* const nbody = const_cast<ndBody*> (notifyCallback.m_bodyArray[i]);
		//	ndBodyKinematic* const kBody = nbody->GetAsBodyKinematic();
		//
		//	const ndShapeInstance& otherShape = kBody->GetCollisionShape();
		//	const ndMatrix& otherMatrix = notifyCallback.m_bodyArray[i]->GetMatrix();
		//
		//	// Ignores collisions with phantom's 'resting on' body
		//	ndBodyNotify* const notify = nbody->GetNotifyCallback();
		//	Renderable* const node = static_cast<Renderable*> (notify->GetUserData());
		//
		//	// if (node && restingOn && node == restingOn.get()) continue;
		//
		//	 // Ignores collisions with itself
		//	if (otherShape.GetShape() != phantomShape.GetShape())
		//	{
		//		if (node)
		//		{
		//			LOG(DBUG) << "CONTACTING " << node->getName();
		//			node->getSpaceTime().updateWorldBounds(true);
		//			AlignedBox3f box = node->getSpaceTime().worldBound;
		//			LOG(DBUG) << "min{ " << box.min().x() << ", " << box.min().y() << ", " << box.min().z() << "}";
		//			LOG(DBUG) << "max{ " << box.max().x() << ", " << box.max().y() << ", " << box.max().z() << "}";
		//		}
		//
		//
		//		ndFixSizeArray<ndContactPoint, 16> contactBuffer;
		//		ndVector phantomVelocity = ndVector::m_zero;
		//		ndVector otherVelocity = ndVector::m_zero;
		//
		//		ndContactSolver contSolver;
		//		contSolver.CalculateContacts(&phantomShape, worldMatrix, phantomVelocity, &otherShape, otherMatrix, otherVelocity, contactBuffer, &notification);
		//		contactCount = contactBuffer.GetCount();
		//		LOG(DBUG) << "CONTACT COUNT: " << contactCount;
		//
		//		if (contactCount)
		//		{
		//
		//			for (int j = 0; j < contactCount; ++j)
		//			{
		//				const ndContactPoint& cPnt = contactBuffer[j];
		//				contactPoint = cPnt.m_point;
		//			}
		//		}
		//	}
		//	else
		//	{
		//
		//	}
		//}
	}

	private:
	RenderableNode phantomNode = nullptr;
	ndShapeInstance phantomShape;

	ndMatrix worldMatrix;
	ndContactNotify notification;
	ndInt32 contactCount = -1;
	ndVector contactPoint = ndVector(1.0e10f);
};


void ndObjectPlacement (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, ndGetIdentityMatrix());

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
	//AddSphere1(scene, PlaceMatrix(0.0f, 5.0f, 0.0f), 0.5f, 10.0f);
	//AddCapsule(scene, PlaceMatrix(0.0f, 5.0f, 3.0f), 0.7f, 10.0f);
	//AddConvexHull(scene, PlaceMatrix(-2.0f, 5.0f, -2.0f), 7, 1.0f, 1.5f, 0.8f, 10.0f);
	//AddConvexHull(scene, PlaceMatrix(-2.0f, 5.0f,  2.0f), 21, 1.0f, 1.5f, 0.7f, 10.0f);
	//AddConvexHull(scene, PlaceMatrix( 2.0f, 5.0f,  3.0f), 210, 1.0f, 1.5f, 0.9f, 10.0f);

	ndQuaternion rot;
	ndVector origin(-40.0f, 5.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
