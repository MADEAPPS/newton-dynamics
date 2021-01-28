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

#ifndef _D_LOAD_FBX_MESH_H_
#define _D_LOAD_FBX_MESH_H_

#include "ndSandboxStdafx.h"
#include "ndDemoEntity.h"

class ndDemoEntityManager;

class fbxDemoEntity : public ndDemoEntity
{
	public:
	fbxDemoEntity(ndDemoEntity* const parent);
	~fbxDemoEntity();

	void SetRenderMatrix(const dMatrix& matrix)
	{
		m_matrix = matrix;
	}

	void CleanIntermediate();
	void BuildRenderMeshes(ndDemoEntityManager* const scene);
	void ApplyTransform(const dMatrix& cordinateSystem);

	ndMeshEffect* m_fbxMeshEffect;
};

fbxDemoEntity* LoadFbxMesh(const char* const meshName);

#endif