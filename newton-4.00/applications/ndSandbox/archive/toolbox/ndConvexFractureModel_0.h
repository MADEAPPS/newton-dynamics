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

#ifndef __ND_CONVEX_FRACTURE_0_H__
#define __ND_CONVEX_FRACTURE_0_H__

#include "ndSandboxStdafx.h"
#include "ndDemoEntityNotify.h"

class ndDemoMesh;
class ndDemoDebrisEntity;
class ndDemoEntityManager;
class ndDemoDebriEntityRoot;
class ndDemoDebrisRootEntity;

class ndConvexFracture : public ndClassAlloc
{
	class ndDebrisNotify : public ndDemoEntityNotify
	{
		public:
		ndDebrisNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity);
		void OnObjectPick() const;
	};

	public:
	ndConvexFracture();
	~ndConvexFracture();

	void GenerateEffect(ndDemoEntityManager* const scene);

	void AddEffect(ndDemoEntityManager* const scene, const ndMatrix& location);

	void ExplodeLocation(ndBodyDynamic* const body, const ndMatrix& location, ndFloat32 factor) const;

	ndMatrix m_textureMatrix;
	ndArray<ndVector> m_pointCloud;
	ndMeshEffect* m_singleManifoldMesh;
	const char* m_innerTexture;
	ndFloat32 m_tileFactor;
	ndFloat32 m_mass;
	ndFloat32 m_breakImpactSpeed;
	ndInt32 m_interiorMaterialIndex;

	private:
	ndDemoDebrisRootEntity* m_debriRootEnt;
};

#endif
