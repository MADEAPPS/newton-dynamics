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
#include "dMatrix.h"


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
	return PhysicsAlloc(size);
}

void operator delete (void* ptr)
{
	PhysicsFree(ptr);
}


static void ForceAndTorqueCallback (ndBodyDynamicC body, dFloat32 timestep)
{
}

static void SetTransformCallback(ndBodyDynamicC body, const dFloat32* const matrix)
{
}

static dVector FindFloor(ndWorldC world, const dVector& origin, dFloat32 dist)
{
	// shot a vertical ray from a high altitude and collect the intersection parameter.
	dVector p0(origin);
	dVector p1(origin - dVector(0.0f, dAbs(dist), 0.0f, 0.0f));
	
	//ndRayCastClosestHitCallback rayCaster(world.GetScene());
	//dFloat32 param = rayCaster.TraceRay(p0, p1);
	//return (param < 1.0f) ? rayCaster.m_contact.m_point : p0;
	return dVector(0.0f);
}

/*
void BuildPyramid(ndWorld& world, dFloat32 mass, const dVector& origin, const dVector& size, int count)
{
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	world.Sync();
ndShapeInstance xxx(new ndShapeSphere(1.0f));
	ndShapeInstance box(new ndShapeBox(size.m_x, size.m_y, size.m_z));

	dVector floor(FindFloor(world, dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	matrix.m_posit.m_y = floor.m_y + size.m_y / 2.0f;

	// get the dimension from shape itself
	dVector minP(0.0f);
	dVector maxP(0.0f);
	box.CalculateAABB(dGetIdentityMatrix(), minP, maxP);

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
*/

void BuildSphere(ndWorldC world, dFloat32 mass, const dVector& origin, const dFloat32 diameter, int count, dFloat32 xxxx)
{
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	ndWorldSync(world);

	ndShapeC sphereShape = ndCreateSphere(diameter * 0.5f);
	ndShapeInstanceC sphereInstance = ndCreateInstance(sphereShape);
	ndShapeRelease(sphereShape);

	dVector floor(FindFloor(world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
	//matrix.m_posit.m_y = floor.m_y + diameter * 0.5f * 0.99f;
	//matrix.m_posit.m_y += xxxx;

	//for (int i = 0; i < count; i++)
	//{
	//	ndBodyDynamic* const body = new ndBodyDynamic();
	//
	//	body->SetNotifyCallback(new ndDemoEntityNotify);
	//	body->SetMatrix(matrix);
	//	body->SetCollisionShape(sphere);
	//	body->SetMassMatrix(mass, sphere);
	//
	//	world.AddBody(body);
	//	matrix.m_posit += matrix.m_up.Scale(diameter * 0.99f);
	//}

	ndDestroyInstance(sphereInstance);
}

void BuildFloor(ndWorldC world)
{
	ndWorldSync(world);

	ndShapeC boxShape = ndCreateBox(200.0f, 1.0f, 200.f);
	ndShapeInstanceC boxInstance = ndCreateInstance(boxShape);
	ndShapeRelease(boxShape);

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit.m_y = -0.5f;

	ndBodyDynamicC body = ndCreateBodyDynamic();
	ndBodyDynamicSetMatrix(body, &matrix[0][0]);
	ndBodyDynamicSetCallbacks(body, nullptr, ForceAndTorqueCallback, SetTransformCallback);
	ndBodyDynamicSetCollisionShape(body, boxInstance);
	ndWorldAddBody(world, body);

	ndDestroyInstance(boxInstance);
}


int main (int argc, const char * argv[]) 
{
	#if defined(_DEBUG) && defined(_MSC_VER)
	_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CRTDBG_REPORT_FLAG);
	//_CrtSetBreakAlloc (127);
	#endif

	ndWorldC world = ndCreateWorld();
	ndWorldSetSubSteps(world, 2);
	ndWorldSetThreadCount(world, 4);

	//dVector size(0.5f, 0.25f, 0.8f, 0.0f); 
	dVector origin(0.0f, 0.0f, 0.0f, 0.0f);
	BuildFloor(world);

	//BuildPyramid(world, 10.0f, origin, size, 20);
	//BuildSphere(world, 1.0f, origin + dVector(0.0f, 0.0f, 0.0f, 0.0f), 1.0f, 2, 0.0f);
	BuildSphere(world, 1.0f, origin + dVector(3.0f, 0.0f, 0.0f, 0.0f), 1.0f, 1, 1.0f);
	//BuildSphere(world, 1.0f, origin + dVector(6.0f, 0.0f, 0.0f, 0.0f), 1.0f, 1, 0.0f);
	//BuildSphere(world, 1.0f, origin + dVector(9.0f, 0.0f, 0.0f, 0.0f), 1.0f, 1, 0.0f);
	
	dFloat32 totalTime = 0;
	for (int i = 0; i < 10000; i ++)
	{
		totalTime += ndWorldGetUpdateTime(world);
		ndWorldUpdate(world, 1.0f / 60.0f);
	}

	ndDestroyWorld(world);

	#if defined(_DEBUG) && defined(_MSC_VER)
	_CrtDumpMemoryLeaks();
	#endif
	return 0;
}
