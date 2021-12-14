/* Copyright (c) <2003-2021> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef __D_CONVEX_FRACTURE_2_H__
#define __D_CONVEX_FRACTURE_2_H__

#include "ndSandboxStdafx.h"

class ndDemoMesh;
class ndDemoDebrisEntity;
class ndDemoEntityManager;
class ndDemoDebriEntityRoot;
class ndDemoDebrisRootEntity;

class ndConvexFractureModel_2: public ndModel
{
	class ndAtom
	{
		public:
		ndAtom();
		ndAtom(const ndAtom& atom);
		~ndAtom();

		ndVector m_centerOfMass;
		ndVector m_momentOfInertia;
		ndDemoDebrisEntity* m_mesh;
		ndShapeInstance* m_collision;
		ndFloat32 m_massFraction;
	};

	public:
	class ndDesc
	{
		public:
		ndDesc()
			:m_pointCloud()
			,m_shape(nullptr)
			,m_outTexture(nullptr)
			,m_innerTexture(nullptr)
			,m_breakImpactSpeed(10.0f)
		{
		}

		ndArray<ndVector> m_pointCloud;
		ndShapeInstance* m_shape;
		const char* m_outTexture;
		const char* m_innerTexture;
		ndFloat32 m_breakImpactSpeed;
	};

	class ndEffect : public ndList<ndAtom>
	{
		public:
		ndEffect(ndConvexFractureModel_2* const manager, const ndDesc& desc);
		ndEffect(const ndEffect& effect);
		~ndEffect();

		private:
		ndBodyKinematic* m_body;
		ndShapeInstance* m_shape;
		ndDemoMesh* m_visualMesh;
		ndDemoDebrisRootEntity* m_debrisRootEnt;
		ndFloat32 m_breakImpactSpeed;
		friend ndConvexFractureModel_2;
	};

	public:
	ndConvexFractureModel_2(ndDemoEntityManager* const scene);
	~ndConvexFractureModel_2();

	void AddEffect(const ndEffect& effect, ndFloat32 mass, const ndMatrix& location);

	//virtual void AppUpdate(ndWorld* const world);
	virtual void Update(ndWorld* const world, ndFloat32 timestep);
	virtual void PostUpdate(ndWorld* const world, ndFloat32 timestep);

	void UpdateEffect(ndWorld* const world, ndEffect& effect);

	void ExplodeLocation(ndBodyDynamic* const body, const ndMatrix& matrix, ndFloat32 factor) const;

	ndList<ndEffect> m_effectList;
	ndList<ndEffect> m_pendingEffect;
	ndDemoEntityManager* m_scene;
	ndSpinLock m_lock;
};

#endif
