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

//class ndDemoEntityManager;
#include "ndFbxMeshLoader.h"

class ndMeshLoader: public ndFbxMeshLoader
{
	public:
	ndMeshLoader();
	ndMeshLoader(const ndMeshLoader& src);
	virtual ~ndMeshLoader();
	bool LoadEntity(ndRender* const renderer, const ndString& fbxPathMeshName);
	ndSharedPtr<ndAnimationSequence> GetAnimationSequence(const ndString& fbxPathAnimName);
	const ndSharedPtr<ndAnimationSequence> FindSequence(const ndString& fbxPathAnimName) const;

	void SetTranslationTracks(const ndString& boneName);

	ndSharedPtr<ndMesh> m_mesh;
	ndSharedPtr<ndRenderSceneNode> m_renderMesh;
	ndTree<ndSharedPtr<ndAnimationSequence>, ndString> m_animationCache;
};

#endif