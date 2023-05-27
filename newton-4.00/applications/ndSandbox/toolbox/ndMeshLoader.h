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

class ndMeshLoader: public ndFbxMeshLoader
{
	public:
	ndMeshLoader(ndFloat32 scale = 1.0f);
	virtual ~ndMeshLoader();

	ndAnimationSequence* LoadAnimation(const char* const meshName);
	ndMeshEffectNode* LoadMesh(const char* const meshName, bool loadAnimation = false);

	ndFloat32 m_scale;
};

#endif