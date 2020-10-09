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


class RayCastData
{
	public:
	RayCastData()
		:m_param(1.2f)
	{
	}

	dVector m_hitPoint;
	dFloat m_param;
};
	
static dFloat32 RayCastFilterCallback(void* const userData, const ndRayCastContactC* const contact, dFloat32 intersetParam)
{
	RayCastData* const data = (RayCastData*)userData;
	if (intersetParam < data->m_param)
	{
		data->m_param = intersetParam;
		data->m_hitPoint.m_x = contact->m_point.m_x;
		data->m_hitPoint.m_y = contact->m_point.m_y;
		data->m_hitPoint.m_z = contact->m_point.m_z;
		data->m_hitPoint.m_w = contact->m_point.m_w;
	}
	return intersetParam;
}

static dVector FindFloor(ndWorldC world, const dVector& origin, dFloat32 dist)
{
	// shot a vertical ray from a high altitude and collect the intersection parameter.
	dVector p0(origin);
	dVector p1(origin - dVector(0.0f, dAbs(dist), 0.0f, 0.0f));

	RayCastData contact;
	dFloat32 param = ndWorldRayCast(world, &p0.m_x, &p1.m_x, &contact, RayCastFilterCallback, nullptr);
	return (param < 1.0f) ? contact.m_hitPoint : p0;
}

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
	matrix.m_posit.m_y = floor.m_y + diameter * 0.5f * 0.99f;
	matrix.m_posit.m_y += xxxx;

	for (int i = 0; i < count; i++)
	{
		ndBodyDynamicC body = ndCreateBodyDynamic();
	
		ndBodyDynamicSetMatrix(body, &matrix[0][0]);
		ndBodyDynamicSetCollisionShape(body, sphereInstance);
		ndBodyDynamicSetCallbacks(body, nullptr, ForceAndTorqueCallback, SetTransformCallback);
		ndBodyDynamicSetMassMatrix(body, mass, sphereInstance);
	
		ndWorldAddBody(world, body);
		matrix.m_posit += matrix.m_up.Scale(diameter * 0.99f);
	}

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
	ndBodyDynamicSetCollisionShape(body, boxInstance);
	ndBodyDynamicSetCallbacks(body, nullptr, ForceAndTorqueCallback, SetTransformCallback);
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

	dVector origin(0.0f, 0.0f, 0.0f, 0.0f);
	BuildFloor(world);
	BuildSphere(world, 1.0f, origin + dVector(3.0f, 0.0f, 0.0f, 0.0f), 1.0f, 1, 1.0f);
	
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
