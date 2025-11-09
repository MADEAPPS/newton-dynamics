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

#include "ndCore.h"

class ndMesh;
class ndString;

class ndMeshLoader : public ndClassAlloc
{
	public:
	D_COLLISION_API ndMeshLoader();
	D_COLLISION_API virtual ~ndMeshLoader();

	D_COLLISION_API virtual bool LoadMesh(const ndString& pathMeshName);
	D_COLLISION_API virtual void SaveMesh(const ndString& pathMeshName);

	public:
	ndSharedPtr<ndMesh> m_mesh;
};

#endif