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

#ifndef _D_MAKE_STATIC_MAP_H_
#define _D_MAKE_STATIC_MAP_H_

#include "ndSandboxStdafx.h"

class ndDemoMesh;
class ndDemoEntity;
class ndDemoEntityManager;

class fbxDemoEntity : public ndDemoEntity
{
	public:
	fbxDemoEntity(ndDemoEntity* const parent)
		:ndDemoEntity(dGetIdentityMatrix(), parent)
		,m_fbxMeshEffect(nullptr)
	{
	}

	~fbxDemoEntity()
	{
		if (m_fbxMeshEffect)
		{
			delete m_fbxMeshEffect;
		}
	}

	void SetRenderMatrix(const dMatrix& matrix)
	{
		m_matrix = matrix;
	}

	void CleanIntermidiate();

	dMeshEffect* m_fbxMeshEffect;
};


fbxDemoEntity* LoadFbxMesh(ndDemoEntityManager* const scene, const char* const meshName);

ndBodyKinematic* BuildStaticMesh(ndDemoEntityManager* const scene, const char* const meshName);


#endif