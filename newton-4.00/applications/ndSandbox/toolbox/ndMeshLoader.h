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

#ifndef _ND_MESH_LOADER_H_
#define _ND_MESH_LOADER_H_

class ndDemoEntity;
class ndDemoEntityManager;

class ndMeshLoader: public ndFbxMeshLoader
{
	public:
	ndMeshLoader();
	virtual ~ndMeshLoader();

	virtual ndMesh* LoadMesh(const char* const fbxMeshName, bool loadAnimation = false);
	virtual ndDemoEntity* LoadEntity(const char* const fbxMeshName, ndDemoEntityManager* const scene);
};

#endif