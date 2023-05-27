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

#include "ndSandboxStdafx.h"
#include "ndMeshLoader.h"

ndMeshLoader::ndMeshLoader(ndFloat32 scale)
	:ndFbxMeshLoader()
	,m_scale(scale)
{
}

ndMeshLoader::~ndMeshLoader()
{
}

ndMesh* ndMeshLoader::LoadMesh(const char* const fbxMeshName, bool loadAnimation)
{
	char pathName[1024];
	dGetWorkingFileName(fbxMeshName, pathName);
	ndMesh* const mesh = ndFbxMeshLoader::LoadMesh(pathName, loadAnimation);
	if (m_scale != ndFloat32(1.0f))
	{
		ndMatrix scaleMatrix(ndGetIdentityMatrix());
		scaleMatrix[0][0] = m_scale;
		scaleMatrix[1][1] = m_scale;
		scaleMatrix[2][2] = m_scale;
		mesh->ApplyTransform(scaleMatrix);
	}

	//ndMesh::Save(mesh, "xxx.ndm");
	return mesh;
}

ndAnimationSequence* ndMeshLoader::LoadAnimation(const char* const meshName)
{
	return ndFbxMeshLoader::LoadAnimation(meshName);
}
