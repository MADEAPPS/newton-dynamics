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

#include "ndNewton.h"
#include <gtest/gtest.h>

class csBodyTrigger : public ndBodyTriggerVolume 
{
	public:
	csBodyTrigger()
		:ndBodyTriggerVolume() 
	{ 
	}

	virtual ~csBodyTrigger() 
	{
	}

	virtual void OnTrigger(ndBodyKinematic* const, ndFloat32) 
	{ 
		//std::cout << "Trigger\n"; 
		ndTrace(("trigger\n"));
	}
	virtual void OnTriggerEnter(ndBodyKinematic* const, ndFloat32)
	{ 
		//std::cout << "Enter\n"; 
		ndTrace(("enter Trigger\n"));
	}
	virtual void OnTriggerExit(ndBodyKinematic* const, ndFloat32)
	{ 
		//std::cout << "Exit\n"; 
		ndTrace(("exit Trigger\n"));
	}
};

TEST(StaticTrigger, TriggerCollision)
{
	ndWorld world;
	ndShapeInstance shapeinst(new ndShapeSphere(ndFloat32(0.5f)));

	ndMatrix matrix(ndGetIdentityMatrix());

	csBodyTrigger* triggerbody = new csBodyTrigger();
	triggerbody->SetCollisionShape(shapeinst);
	triggerbody->SetMatrix(matrix);
	ndSharedPtr<ndBody> triggerPtr(triggerbody);
	world.AddBody(triggerPtr);

	//matrix.m_posit.m_y = 5; <--- commenting out this line makes the body start in the trigger

	ndBodyDynamic* movingbody = new ndBodyDynamic();
	movingbody->SetNotifyCallback(new ndBodyNotify(ndBigVector(ndFloat32(0), ndFloat32(-9.81f), ndFloat32(0), ndFloat32(0))));
	movingbody->SetCollisionShape(shapeinst);
	movingbody->SetMatrix(matrix);
	movingbody->SetMassMatrix(ndFloat32(10), shapeinst);
	movingbody->SetDebugMaxLinearAndAngularIntegrationStep(ndPi, ndFloat32(2.0f));
	ndSharedPtr<ndBody> movingPtr(movingbody);
	world.AddBody(movingPtr);

	for (int i = 0; i < 480; i++)
	{
		world.Update(1.0f / 60.0f);
		world.Sync();
	}

	world.CleanUp();
}

TEST(DynamicsTrigger, TriggerCollision)
{
	ndWorld* world = new ndWorld();
	ndShapeInstance shape(new ndShapeBox(1, 1, 1));
	ndMatrix matrix(ndGetIdentityMatrix());

	// Trigger body without mass doesn't move
	csBodyTrigger* kinebody = new csBodyTrigger();
	kinebody->SetCollisionShape(shape);
	kinebody->SetMatrix(matrix);
	kinebody->SetVelocity(ndVector(ndFloat32(4), ndFloat32(0), ndFloat32(0), ndFloat32(0)));
	world->AddBody(ndSharedPtr<ndBody>(kinebody));

	matrix.m_posit.m_x += 4;
	ndBodyDynamic* body = new ndBodyDynamic();
	body->SetCollisionShape(shape);
	body->SetMassMatrix(1, shape);
	body->SetMatrix(matrix);
	body->SetNotifyCallback(new ndBodyNotify(ndVector(ndFloat32(0), ndFloat32(0), ndFloat32(0), ndFloat32(0))));
	world->AddBody(ndSharedPtr<ndBody>(body));

	// No movement, since is have infinite mass (static body)
	world->Update(1.0f / 60.0f);
	world->Sync();
	ndAssert(kinebody->GetMatrix().m_posit.m_x == 0);

	// Add mass and reset velocity, expect movement because has mass and velocity
	kinebody->SetMassMatrix(1, shape);
	kinebody->SetVelocity(ndVector(ndFloat32(4), ndFloat32(0), ndFloat32(0), ndFloat32(0)));
	world->Update(1.0f / 60.0f);
	world->Sync();
	ndAssert(kinebody->GetMatrix().m_posit.m_x != 0);

	// the trigger in the trigger notification 
	for (int i = 0; i < 480; i++)
	{
		world->Update(1.0f / 60.0f);
		world->Sync();
	}

	world->CleanUp();
	delete world;
}
