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

#ifndef _D_LOAD_FBX_MESH_H_
#define _D_LOAD_FBX_MESH_H_

#include "ndSandboxStdafx.h"
#include "ndDemoEntity.h"

class ndDemoEntityManager;
class ndAnimationSequence;

class fbxDemoEntity : public ndDemoEntity
{
	public:
	fbxDemoEntity(ndDemoEntity* const parent);
	fbxDemoEntity(const fbxDemoEntity& source);
	~fbxDemoEntity();

	void SetRenderMatrix(const ndMatrix& matrix)
	{
		m_matrix = matrix;
	}

	void CleanIntermediate();
	ndNodeBaseHierarchy* CreateClone() const;
	void ApplyTransform(const ndMatrix& cordinateSystem);
	void BuildRenderMeshes(ndDemoEntityManager* const scene);

	ndMeshEffect* m_fbxMeshEffect;
};

fbxDemoEntity* LoadFbxMesh(const char* const meshName);
ndAnimationSequence* LoadFbxAnimation(const char* const meshName);

#endif