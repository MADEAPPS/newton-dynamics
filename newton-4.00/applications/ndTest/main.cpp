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

#include "testStdafx.h"


// memory allocation for Newton
static void* PhysicsAlloc(size_t sizeInBytes)
{
	void* const ptr = malloc(sizeInBytes);
	return ptr;
}

// memory free use by the engine
static void PhysicsFree(void* ptr)
{
	free (ptr);
}

void *operator new (size_t size)
{
	// this should not happens on this test
	// newton should never use global operator new and delete.
	//dAssert(0);
	return PhysicsAlloc(size);
}

void operator delete (void* ptr)
{
	PhysicsFree(ptr);
}

class CheckMemoryLeaks
{
	public:
	CheckMemoryLeaks()
	{
		#if defined(_DEBUG) && defined(_MSC_VER)
			// Track all memory leaks at the operating system level.
			// make sure no Newton tool or utility leaves leaks behind.
			dUnsigned32 flags = _CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF) & 0xffff;
			flags = flags | _CRTDBG_REPORT_FLAG;
			flags = flags | _CRTDBG_CHECK_EVERY_1024_DF;
			_CrtSetDbgFlag(flags);
			//_CrtSetBreakAlloc (127);
		#endif

		atexit(CheckMemoryLeaksCallback);
		// Set the memory allocation function before creation the newton world
		// this is the only function that can be called before the creation of the newton world.
		// it should be called once, and the the call is optional 
		ndMemory::SetMemoryAllocators(PhysicsAlloc, PhysicsFree);
	}

	static void CheckMemoryLeaksCallback()
	{
		#if defined(_DEBUG) && defined(_MSC_VER)
		_CrtDumpMemoryLeaks();
		#endif
	}
};
static CheckMemoryLeaks checkLeaks;

class ndDemoEntityNotify: public ndBodyNotify
{
	public:
	ndDemoEntityNotify()
		:ndBodyNotify(ndVector (0.0f, -10.0f, 0.0f, 0.0f))
	{
		// here we set the application user data that 
		// goes with the game engine, for now is just null
		m_applicationUserData = nullptr;
	}

	//virtual void OnApplyExternalForce(dInt32 threadIndex, dFloat32 timestep)
	virtual void OnApplyExternalForce(dInt32, dFloat32)
	{
		ndBodyDynamic* const dynamicBody = GetBody()->GetAsBodyDynamic();
		if (dynamicBody)
		{
			ndVector massMatrix(dynamicBody->GetMassMatrix());
			ndVector force(ndVector(0.0f, -10.0f, 0.0f, 0.0f).Scale(massMatrix.m_w));
			dynamicBody->SetForce(force);
			dynamicBody->SetTorque(ndVector::m_zero);
		}
	}

	//virtual void OnTransform(dInt32 threadIndex, const ndMatrix& matrix)
	virtual void OnTransform(dInt32, const ndMatrix&)
	{
		// apply this transformation matrix to the application user data.
		//dAssert(0);
	}

	void* m_applicationUserData;
};

ndVector FindFloor(const ndWorld& world, const ndVector& origin, dFloat32 dist)
{
	// shot a vertical ray from a high altitude and collect the intersection parameter.
	ndVector p0(origin);
	ndVector p1(origin - ndVector(0.0f, dAbs(dist), 0.0f, 0.0f));

	ndRayCastClosestHitCallback rayCaster;
	world.RayCast(rayCaster, p0, p1);
	return (rayCaster.m_param < 1.0f) ? rayCaster.m_contact.m_point : p0;
}

void BuildFloorBox(ndWorld& world)
{
	world.Sync();
	ndShapeInstance box(new ndShapeBox(200.0f, 1.0f, 200.f));
	ndBodyDynamic* const body = new ndBodyDynamic();

	ndMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit.m_y = -0.5f;

	body->SetNotifyCallback(new ndDemoEntityNotify);
	body->SetMatrix(matrix);
	body->SetCollisionShape(box);
	world.AddBody(body);
}

void BuildPyramid(ndWorld& world, dFloat32 mass, const ndVector& origin, const ndVector& size, int count)
{
	ndMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	world.Sync();
	//ndShapeInstance xxx(new ndShapeSphere(1.0f));
	ndShapeInstance box(new ndShapeBox(size.m_x, size.m_y, size.m_z));

	ndVector floor(FindFloor(world, ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y + size.m_y / 2.0f;

	// get the dimension from shape itself
	ndVector minP(0.0f);
	ndVector maxP(0.0f);
	box.CalculateAabb(dGetIdentityMatrix(), minP, maxP);

	dFloat32 stepz = maxP.m_z - minP.m_z + 0.03125f;
	dFloat32 stepy = (maxP.m_y - minP.m_y) - 0.01f;
		  
	//dFloat32 y0 = matrix.m_posit.m_y + stepy / 2.0f;
	dFloat32 z0 = matrix.m_posit.m_z - stepz * count / 2;

	z0 = 0.0f;
	count = 1;
	for (int j = 0; j < count; j++) 
	{
		matrix.m_posit.m_z = z0;
		const dInt32 count1 = count - j;
		for (int i = 0; i < count1; i++)
		{
			ndBodyDynamic* const body = new ndBodyDynamic();

			body->SetNotifyCallback(new ndDemoEntityNotify);
			body->SetMatrix(matrix);
			body->SetCollisionShape(box);
			body->SetMassMatrix(mass, box);

			world.AddBody(body);
			matrix.m_posit.m_z += stepz;
		}
		z0 += stepz * 0.5f;
		matrix.m_posit.m_y += stepy;
	}
}

void BuildSphere(ndWorld& world, dFloat32 mass, const ndVector& origin, const dFloat32 diameter, int count, dFloat32 offsetHigh)
{
	ndMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	world.Sync();

	ndShapeInstance sphere(new ndShapeSphere(diameter * 0.5f));

	ndVector floor(FindFloor(world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y + diameter * 0.5f * 0.99f;

	matrix.m_posit.m_y += offsetHigh;

	for (int i = 0; i < count; i++)
	{
		ndBodyDynamic* const body = new ndBodyDynamic();

		body->SetNotifyCallback(new ndDemoEntityNotify);
		body->SetMatrix(matrix);
		body->SetCollisionShape(sphere);
		body->SetMassMatrix(mass, sphere);

		world.AddBody(body);
		matrix.m_posit += matrix.m_up.Scale(diameter * 0.99f);
	}
}

int main (int, const char*) 
{
	ndWorld world;
	world.SetSubSteps(2);
	//world.SetThreadCount(2);

	ndVector size(0.5f, 0.25f, 0.8f, 0.0f); 
	ndVector origin(0.0f, 0.0f, 0.0f, 0.0f);
	BuildFloorBox(world);
	//BuildPyramid(world, 10.0f, origin, size, 20);
	//BuildSphere(world, 1.0f, origin + ndVector(0.0f, 0.0f, 0.0f, 0.0f), 1.0f, 2, 0.0f);
	BuildSphere(world, 1.0f, origin + ndVector(3.0f, 0.0f, 0.0f, 0.0f), 1.0f, 1, 1.0f);
	//BuildSphere(world, 1.0f, origin + ndVector(6.0f, 0.0f, 0.0f, 0.0f), 1.0f, 1, 0.0f);
	//BuildSphere(world, 1.0f, origin + ndVector(9.0f, 0.0f, 0.0f, 0.0f), 1.0f, 1, 0.0f);
	
	dFloat32 totalTime = 0;
	for (dInt32 i = 0; i < 10000; i ++)
	{
		world.Update(1.0f / 60.0f);
		totalTime += world.GetUpdateTime();
		//newton.Sync();
	}

	return 0;
}
