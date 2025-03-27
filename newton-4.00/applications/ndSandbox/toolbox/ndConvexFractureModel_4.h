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

#ifndef __ND_CONVEX_FRACTURE_4_H__
#define __ND_CONVEX_FRACTURE_4_H__

#include "ndSandboxStdafx.h"

class ndDemoMesh;
class ndDemoDebrisEntity;
class ndDemoEntityManager;
class ndDemoDebriEntityRoot;
class ndDemoDebrisRootEntity;

class ndConvexFractureModel_4: public ndModel
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
			,m_outerShape(nullptr)
			,m_innerShape(nullptr)
			,m_outTexture(nullptr)
			,m_innerTexture(nullptr)
			,m_breakImpactSpeed(10.0f)
		{
		}

		ndArray<ndVector> m_pointCloud;
		ndShapeInstance* m_outerShape;
		ndShapeInstance* m_innerShape;
		const char* m_outTexture;
		const char* m_innerTexture;
		ndFloat32 m_breakImpactSpeed;
	};

	class ndEffect : public ndList<ndAtom>
	{
		public:
		ndEffect(ndConvexFractureModel_4* const manager, const ndDesc& desc);
		ndEffect(const ndEffect& effect);
		~ndEffect();

		private:
		ndBodyKinematic* m_body;
		ndShapeInstance* m_shape;
		ndDemoMesh* m_visualMesh;
		ndDemoDebrisRootEntity* m_debrisRootEnt;
		ndFloat32 m_breakImpactSpeed;
		friend ndConvexFractureModel_4;
	};

	public:
	ndConvexFractureModel_4(ndDemoEntityManager* const scene);
	~ndConvexFractureModel_4();

	void AddEffect(const ndEffect& effect, ndFloat32 mass, const ndMatrix& location);

	virtual void Update(ndFloat32 timestep);
	virtual void PostUpdate(ndFloat32 timestep);

	void UpdateEffect(ndEffect& effect);

	void ExplodeLocation(ndBodyDynamic* const body, const ndMatrix& matrix, ndFloat32 factor) const;

	ndList<ndEffect> m_effectList;
	ndList<ndEffect> m_pendingEffect;
	ndDemoEntityManager* m_scene;
	ndSpinLock m_lock;
};

#endif
