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

#ifndef __D_CONVEX_FRACTURE_0_H__
#define __D_CONVEX_FRACTURE_0_H__

#include "ndSandboxStdafx.h"

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

	void ExplodeLocation(ndBodyDynamic* const body, const ndMatrix& location, dFloat32 factor) const;

	ndMatrix m_textureMatrix;
	ndArray<ndVector> m_pointCloud;
	ndMeshEffect* m_singleManifoldMesh;
	const char* m_innerTexture;
	dFloat32 m_tileFactor;
	dFloat32 m_mass;
	dFloat32 m_breakImpactSpeed;
	dInt32 m_interiorMaterialIndex;

	private:
	ndDemoDebrisRootEntity* m_debriRootEnt;
};

#endif
