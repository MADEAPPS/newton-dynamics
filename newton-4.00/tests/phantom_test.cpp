
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

#include <cstdio>
#include "ndNewton.h"
#include <gtest/gtest.h>

constexpr ndFloat32 TIME_STEP = (1.0f / 60.0f);

static ndBodyDynamic* BuildSphere(const ndVector& pos, const ndVector& gravity = { 0.f })
{
	// Create the rigid body and configure gravity for it.
	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndBodyNotify(gravity));

	// Set the position of the sphere in the world.
	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit = pos;
	body->SetMatrix(matrix);

	// Attach the collision shape and use a convenience function to automatically
	// compute the inertia matrix for the body.
	ndShapeInstance sphere(new ndShapeSphere(1.0f));
	body->SetCollisionShape(sphere);
	body->SetMassMatrix(2.0f, sphere);

	// Disable damping for the tests to better compare the Newton
	// results with the analytical ones.
	body->SetAngularDamping(ndVector(0.f));
	body->SetLinearDamping(0.f);

	return body;
}

class NewtonPhantom : public ndModel
{
	class NewtonPhantomModelNotify : public ndModelNotify
	{
		public:
		NewtonPhantomModelNotify()
			:ndModelNotify()
		{
		}

		NewtonPhantomModelNotify(const NewtonPhantomModelNotify& src)
			:ndModelNotify(src)
		{
		}

		~NewtonPhantomModelNotify()
		{
		}

		ndModelNotify* Clone() const
		{
			return new NewtonPhantomModelNotify(*this);
		}

		void Update(ndFloat32)
		{
			NewtonPhantom* const model = (NewtonPhantom*)GetModel();
			model->Update();
		}

		void PostUpdate(ndFloat32)
		{
		}

		void PostTransformUpdate(ndFloat32)
		{
		}
	};


	// A Phantom collision shape can be moved around the world, gathering contact
	// information with other ndBody's without effecting the simulation

	public:
	NewtonPhantom(ndScene* scene) :
		ndModel(),
		phantomShape(new ndShapeBox(1.0f, 1.0f, 1.0f)),
		worldMatrix(ndGetIdentityMatrix()),
		notification(scene)
	{
		//ndTrace(("Fix NewtonPhantom\n"));
		SetNotifyCallback(new NewtonPhantomModelNotify);
	}
	~NewtonPhantom() = default;

	virtual void OnAddToWorld()
	{
		// do nothing since is does not uses bodies or joints
	}

	virtual void OnRemoveFromToWorld()
	{
		// do nothing since is does not uses bodies or joints
	}

	void transform(const ndMatrix& matrix) { worldMatrix = matrix; }
	ndInt32 getContactCount() const { return contactCount; }
	ndVector getContactPoint() const { return contactPoint; }

	void Update()
	{
		// calc the current AABB in world space
		ndVector boxMin;
		ndVector boxMax;
		phantomShape.CalculateAabb(worldMatrix, boxMin, boxMax);
	
		ndBodiesInAabbNotify notifyCallback;
		ndWorld* const world = GetWorld();
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
	
				// should be 1 when a box hits a sphere dead on, correct?
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

private:
	ndShapeInstance phantomShape;
	ndMatrix worldMatrix;
	ndContactNotify notification;
	ndInt32 contactCount = 0;
	ndVector contactPoint = ndVector(std::numeric_limits<float>::max());
}; // end class NewtonPhantom


TEST(CollisionShape, Phantom)
{
	ndWorld world;
	world.SetSubSteps(2);
	world.SetThreadCount(std::thread::hardware_concurrency() - 1);

	// Create a sphere at the origin. No gravity will act on it by default.
	ndVector spherePos = ndVector(0.0f, 0.0f, 0.0f, 1.0f);
	ndSharedPtr<ndBody> sphere(BuildSphere(spherePos));
	world.AddBody(sphere);

	// create a Phantom model that contains a collision shape and transform matrix
	NewtonPhantom* const phantom = new NewtonPhantom(world.GetScene());
	ndSharedPtr<ndModel> phantomPtr(phantom);
	world.AddModel(phantomPtr);

	// move the phantom above the sphere
	ndMatrix phantomMatrix = ndGetIdentityMatrix();
	phantomMatrix.m_posit.m_y = 3.0f;
	phantom->transform(phantomMatrix);

	ndFloat32 moveAmount = 0.05f;

	// Simulate one second.
	for (int i = 0; i < 60; i++)
	{
		// move the phantom down until it touches the sphere
		phantomMatrix.m_posit.m_y -= moveAmount;
		phantom->transform(phantomMatrix);

		world.Update(TIME_STEP);
		world.Sync();

		if (phantom->getContactCount()) break;
	}

	EXPECT_TRUE(phantom->getContactCount() == 1);

	// phantom box should contact the sphere y at about it's
	// radius, which is 1.0f;
	ndVector contactPoint = phantom->getContactPoint();
	EXPECT_NEAR(contactPoint.m_y, 1.0f, 1E-3);
}
