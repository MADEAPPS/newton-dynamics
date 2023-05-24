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
		std::cout << "Trigger\n"; 
	}
	virtual void OnTriggerEnter(ndBodyKinematic* const, ndFloat32)
	{ 
		std::cout << "Enter\n"; 
	}
	virtual void OnTriggerExit(ndBodyKinematic* const, ndFloat32)
	{ 
		std::cout << "Exit\n"; 
	}
};

TEST(Collisions, TriggerCollision)
{
	ndWorld world;
	ndShapeInstance shapeinst(new ndShapeSphere(ndFloat32(0.5f)));

	ndMatrix matrix(ndGetIdentityMatrix());

	csBodyTrigger* triggerbody = new csBodyTrigger();
	triggerbody->SetCollisionShape(shapeinst);
	triggerbody->SetMatrix(matrix);
	ndSharedPtr<ndBody> triggerPtr(triggerbody);
	world.AddBody(triggerPtr);

	matrix.m_posit.m_y = 5;

	ndBodyDynamic* movingbody = new ndBodyDynamic();
	movingbody->SetNotifyCallback(new ndBodyNotify(ndBigVector(ndFloat32(0), ndFloat32(-9.81f), ndFloat32(0), ndFloat32(0))));
	movingbody->SetCollisionShape(shapeinst);
	movingbody->SetMatrix(matrix);
	movingbody->SetMassMatrix(ndFloat32(10), shapeinst);
	ndSharedPtr<ndBody> movingPtr(movingbody);
	world.AddBody(movingPtr);

	for (int i = 0; i < 480; i++)
	{
		world.Update(1.0f / 60.0f);
		world.Sync();
	}

	world.CleanUp();
}