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

class csBodyNotify : public ndBodyNotify 
{
	public:
	csBodyNotify(ndBigVector const& defaultGravity) : ndBodyNotify(defaultGravity) {}

	virtual ~csBodyNotify() {}
	virtual void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep) 
	{
		ndVector dir(GetBody()->GetPosition() * -1);
		ndFloat32 dis(dir.m_x * dir.m_x + dir.m_y * dir.m_y + dir.m_z * dir.m_z);
		dir = dir.Normalize();
		ndFloat32 g(ndFloat32(6.67e-11f) * (m_mass / dis));
		SetGravity(dir.Scale (g));
		ndBodyNotify::OnApplyExternalForce(threadIndex, timestep);
	}

	ndFloat32 m_mass;

};

TEST(Extremes, OrbitalCollisions)
{
	ndWorld world;
	world.SetSubSteps(1);
	ndShapeInstance shapeinst(new ndShapeSphere(ndFloat32(50000)));
	ndShapeInstance planetshape(new ndShapeSphere(ndFloat32(6357000)));

	ndMatrix matrix(ndGetIdentityMatrix());
	ndBodyKinematic* staticbody = new ndBodyKinematic();
	staticbody->SetCollisionShape(planetshape);
	staticbody->SetMatrix(matrix);
	staticbody->SetMassMatrix(ndFloat32(5.9722e+24), planetshape);
	ndSharedPtr<ndBody> staticPtr(staticbody);
	world.AddBody(staticPtr);

	matrix.m_posit.m_y = 6357000 + 1021140;

	for (int i = 0; i < 20; i++) 
	{
		ndBodyDynamic* movingbody = new ndBodyDynamic();
		csBodyNotify* n = new csBodyNotify(ndVector(ndFloat32(0), ndFloat32(0), ndFloat32(0), ndFloat32(0)));
		n->m_mass = ndFloat32(5.9722e+24) * ndFloat32(1.5f);
		movingbody->SetNotifyCallback(n);
		movingbody->SetCollisionShape(shapeinst);
		movingbody->SetMatrix(matrix);
		movingbody->SetMassMatrix(ndFloat32(10), shapeinst);
		movingbody->SetVelocity(ndVector(ndFloat32(9000), ndFloat32(0), ndFloat32(0), ndFloat32(0)));
		movingbody->SetDebugMaxLinearAndAngularIntegrationStep(10.0f, 10000.0f);
		//movingbody->SetDebugMaxLinearAndAngularIntegrationStep(ndPi, 2.0f);
		ndSharedPtr<ndBody> movingPtr(movingbody);
		world.AddBody(movingPtr);
		matrix.m_posit.m_y += 100005;
	}

	for (int i = 0; i < 1000; i++)
	{
		world.Update(1.0f / 60.0f);
		//world.Update(100.0f);
		//world.Update(1.0f);
		world.Sync();
	}

	world.CleanUp();
}