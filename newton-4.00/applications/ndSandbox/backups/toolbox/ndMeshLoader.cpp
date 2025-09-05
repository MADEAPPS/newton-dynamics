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
#include "ndDemoEntity.h"

ndMeshLoader::ndMeshLoader()
	:ndFbxMeshLoader()
{
}

ndMeshLoader::~ndMeshLoader()
{
}

ndMesh* ndMeshLoader::LoadMesh(const char* const fbxMeshName, bool loadAnimation)
{
	char pathName[1024];
	ndGetWorkingFileName(fbxMeshName, pathName);
	ndMesh* const mesh = ndFbxMeshLoader::LoadMesh(pathName, loadAnimation);
	return mesh;
}

ndDemoEntity* ndMeshLoader::LoadEntity(const char* const fbxMeshName, ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndMesh> fbxEntity(LoadMesh(fbxMeshName, false));
	ndDemoEntity* const rootEntity = *fbxEntity ? new ndDemoEntity(scene, *fbxEntity) : nullptr;
	return rootEntity;
}