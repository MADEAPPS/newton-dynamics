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

#ifndef __ND_CONVEX_FRACTURE_1_H__
#define __ND_CONVEX_FRACTURE_1_H__

#include "ndSandboxStdafx.h"

class ndDemoMesh;
class ndDemoDebrisEntity;
class ndDemoEntityManager;
class ndDemoDebriEntityRoot;
class ndDemoDebrisRootEntity;

class ndExplodeConvexShapeModel: public ndModel
{
	class ndAtom
	{
		public:
		ndAtom();
		ndAtom(const ndAtom& atom);
		~ndAtom();

		ndVector m_centerOfMass;
		ndVector m_momentOfInertia;
		ndDemoDebrisEntity* m_debriEnt;
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
		ndEffect(ndExplodeConvexShapeModel* const manager, const ndDesc& desc);
		ndEffect(const ndEffect& effect);
		~ndEffect();

		private:
		ndSharedPtr<ndBody> m_body;
		ndShapeInstance* m_shape;
		ndSharedPtr<ndDemoMeshInterface> m_visualMesh;
		ndDemoDebrisRootEntity* m_debrisRootEnt;
		ndFloat32 m_breakImpactSpeed;

		friend ndExplodeConvexShapeModel;
	};

	public:
	ndExplodeConvexShapeModel(ndDemoEntityManager* const scene);
	~ndExplodeConvexShapeModel();

	virtual void OnAddToWorld() { ndAssert(0); }
	virtual void OnRemoveFromToWorld() { ndAssert(0); }

	void AddEffect(const ndEffect& effect, ndFloat32 mass, const ndMatrix& location);

	virtual void Update(ndFloat32 timestep);
	virtual void PostUpdate(ndFloat32 timestep);

	void UpdateEffect(ndEffect& effect);

	ndList<ndEffect> m_effectList;
	ndList<ndEffect> m_pendingEffect;
	ndDemoEntityManager* m_scene;
	ndSpinLock m_lock;
};

#endif
