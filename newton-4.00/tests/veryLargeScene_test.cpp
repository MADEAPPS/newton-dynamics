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

TEST(Extremes, OrbitalDistances)
{
	ndWorld world;
	ndShapeInstance shapeinst(new ndShapeSphere(ndFloat32(0.5f)));

	ndMatrix matrix(ndGetIdentityMatrix());

	ndShapeInstance planetshape(new ndShapeSphere(ndFloat32(6357000)));

	ndBodyKinematic* staticbody = new ndBodyKinematic();
	staticbody->SetCollisionShape(planetshape);
	staticbody->SetMatrix(matrix);
	staticbody->SetMassMatrix(ndFloat32(5.9722e+24), planetshape);
	ndSharedPtr<ndBody> staticPtr(staticbody);
	world.AddBody(staticPtr);

	matrix.m_posit.m_y = 6357000 + 1021140;

	ndBodyDynamic* movingbody = new ndBodyDynamic();
	movingbody->SetNotifyCallback(new ndBodyNotify(ndBigVector(ndFloat32(0), ndFloat32(-9.81f), ndFloat32(0), ndFloat32(0))));
	movingbody->SetCollisionShape(shapeinst);
	movingbody->SetMatrix(matrix);
	movingbody->SetMassMatrix(ndFloat32(10), shapeinst);
	movingbody->SetDebugMaxLinearAndAngularIntegrationStep(ndPi, 2.0f);
	ndSharedPtr<ndBody> movingPtr(movingbody);
	world.AddBody(movingPtr);

	for (int i = 0; i < 480; i++)
	{
		world.Update(1.0f / 60.0f);
		world.Sync();
	}

	world.CleanUp();
}
