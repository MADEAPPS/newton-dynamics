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
	return malloc(sizeInBytes);
}

// memory free use by the engine
static void PhysicsFree(void* ptr)
{
	free (ptr);
}

void *operator new (size_t size)
{
	return dMalloc(size);
}

void operator delete (void* ptr)
{
	dFree(ptr);
}

class CheckMemoryLeaks
{
	public:
	CheckMemoryLeaks()
	{
		#if defined(_DEBUG) && defined(_MSC_VER)
			// Track all memory leaks at the operating system level.
			// make sure no Newton tool or utility leaves leaks behind.
			_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CRTDBG_REPORT_FLAG);
			//_CrtSetBreakAlloc (318776);
		#endif

		atexit(CheckMemoryLeaksCallback);
		// Set the memory allocation function before creation the newton world
		// this is the only function that can be called before the creation of the newton world.
		// it should be called once, and the the call is optional 
		dSetMemoryAllocators(PhysicsAlloc, PhysicsFree);
	}

	static void CheckMemoryLeaksCallback()
	{
		#if defined(_DEBUG) && defined(_MSC_VER)
		_CrtDumpMemoryLeaks();
		#endif
	}
};
static CheckMemoryLeaks checkLeaks;

class DemobodyNotify: public dBodyNotify
{
	public:
	virtual void OnApplyExternalForce(dInt32 threadIndex, dFloat32 timestep)
	{
		dDynamicBody* const body = m_body->GetAsDynamicBody();
		dAssert(body);

		dVector massMatrix (body->GetMassMatrix());
		dVector force(dVector(0.0f, -10.0f, 0.0f, 0.0f).Scale (massMatrix.m_w));
		body->SetForce(force);
		body->SetTorque(dVector::m_zero);
	}

	virtual void OnTranform(dInt32 threadIndex, const dMatrix& matrix)
	{
		dAssert(0);
	}
};

void BuildPyramid(dNewton& world, dFloat32 mass, const dVector& origin, const dVector& size, int count)
{
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	world.Sync();
	dShapeInstance box(new dShapeBox(size.m_x, size.m_y, size.m_z));

	dVector floor(0.0f);
	matrix.m_posit.m_y = floor.m_y + size.m_y / 2.0f;

	// get the dimension from shape itself
	dVector minP(0.0f);
	dVector maxP(0.0f);
	box.CalculateAABB(dGetIdentityMatrix(), minP, maxP);

	dFloat32 stepz = maxP.m_z - minP.m_z + 0.03125f;
	dFloat32 stepy = (maxP.m_y - minP.m_y) - 0.01f;
		  
	dFloat32 y0 = matrix.m_posit.m_y + stepy / 2.0f;
	dFloat32 z0 = matrix.m_posit.m_z - stepz * count / 2;
	
	matrix.m_posit.m_y = y0;
	for (int j = 0; j < count; j++) 
	{
		matrix.m_posit.m_z = z0;
		const dInt32 count1 = count - j;
		for (int i = 0; i < count1; i++)
		{
			dDynamicBody* const body = new dDynamicBody();

			body->SetNotifyCallback(new DemobodyNotify);
			
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

int main (int argc, const char * argv[]) 
{
	dNewton newton;
	newton.SetSubSteps(2);
	//newton.SetThreadCount(4);
		
	dVector size(0.5f, 0.25f, 0.8f, 0.0f); 
	dVector origin(0.5f, 0.25f, 0.8f, 0.0f);
	BuildPyramid(newton, 10.0f, origin, size, 20);

	for (int i = 0; i < 10000; i ++)
	{
		newton.Update(1.0f / 60.0f);
		//newton.Sync();
	}

	return 0;
}

