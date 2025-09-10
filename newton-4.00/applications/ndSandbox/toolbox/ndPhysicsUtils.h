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

#ifndef __PHYSICS_UTIL__
#define __PHYSICS_UTIL__

#define DEMO_GRAVITY  ndFloat32(-10.0f)
//#define DEMO_GRAVITY  ndFloat32(-1.0f)
//#define DEMO_GRAVITY  ndFloat32(0.0f)

class ndDemoEntityManager;

class ndParamMapper
{
	public:
	ndParamMapper()
		:m_x0(0.0f)
		,m_scale(0.0f)
	{
	}

	ndParamMapper(ndFloat32 x0, ndFloat32 x1)
		:m_x0(x0 + (x1 - x0) * 0.5f)
		, m_scale((x1 - x0) * 0.5f)
	{
	}

	ndFloat32 Interpolate(const ndFloat32 t) const
	{
		return m_x0 + m_scale * t;
	}

	ndFloat32 CalculateParam(const ndFloat32 value) const
	{
		return (value - m_x0) / m_scale;
	}

	ndFloat32 m_x0;
	ndFloat32 m_scale;
};

ndVector FindFloor(const ndWorld& world, const ndVector& origin, ndFloat32 dist);
ndMatrix FindFloor(const ndWorld& world, const ndMatrix& origin, const ndShapeInstance& shape, ndFloat32 dist);
ndBodyKinematic* MousePickBody(ndWorld* const nWorld, const ndVector& origin, const ndVector& end, ndFloat32& paramter, ndVector& positionOut, ndVector& normalOut);

void SetModelVisualMesh(ndDemoEntityManager* const scene, ndModelArticulation* const model);

ndSharedPtr<ndBody> CreateRigidNody(ndDemoEntityManager* const scene,
	const ndShapeInstance& shape,
	const ndMatrix& location,
	ndFloat32 mass,
	const char* const textName,
	ndRenderPrimitiveMesh::ndUvMapingMode mappingMode);

ndSharedPtr<ndBody> CreateBox(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 mass, ndFloat32 sizex, ndFloat32 sizey, ndFloat32 sizez, const char* const textName = "wood_0.png");
ndSharedPtr<ndBody> CreateSphere(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 mass, ndFloat32 radius, const char* const textName = "wood_0.png");
ndSharedPtr<ndBody> CreateCapsule(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 mass, ndFloat32 radius0, ndFloat32 radius1, ndFloat32 high, const char* const textName = "wood_1.png");

ndSharedPtr<ndBody> AddSphere(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 mass, ndFloat32 radius, const char* const textName = "smilli.png");
ndSharedPtr<ndBody> AddBox(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 mass, ndFloat32 sizex, ndFloat32 sizey, ndFloat32 sizez, const char* const textName = "wood_0.png");
ndSharedPtr<ndBody> AddCapsule(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 mass, ndFloat32 radius0, ndFloat32 radius1, ndFloat32 high, const char* const textName = "wood_1.png");
ndSharedPtr<ndBody> AddConvexHull(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 mass, ndFloat32 radius, ndFloat32 high, ndInt32 segments, const char* const textName = "wood_0.png");

void AddPlanks(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 mass, ndInt32 count);
void AddCapsulesStacks(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 mass, ndFloat32 radius0, ndFloat32 radius1, ndFloat32 high, ndInt32 rows_x, ndInt32 rows_z, ndInt32 columHigh);

#endif