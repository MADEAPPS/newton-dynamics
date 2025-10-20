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

#include "ndFbxMeshLoader.h"

//class ndMeshLoader: public ndFbxMeshLoader
class ndMeshLoader : public ndClassAlloc
{
	public:
	ndMeshLoader();
	ndMeshLoader(const ndMeshLoader& src);
	virtual ~ndMeshLoader();

	virtual bool LoadMesh(const ndString& pathMeshName);
	virtual void SaveMesh(const ndString& pathMeshName);

	ndSharedPtr<ndAnimationSequence> GetAnimationSequence(const ndString& pathAnimName);
	const ndSharedPtr<ndAnimationSequence> FindSequence(const ndString& pathAnimName) const;
	void SetTranslationTracks(const ndString& boneName);

	virtual bool ImportFbx(const ndString& fbxPathMeshName);
	ndSharedPtr<ndAnimationSequence> ImportFbxAnimation(const ndString& fbxPathAnimName);

	protected:
	ndString GetPath(const ndString& fullPathName) const;
	ndString GetName(const ndString& fullPathName) const;

	public:
	ndSharedPtr<ndMesh> m_mesh;
	ndTree<ndSharedPtr<ndAnimationSequence>, ndString> m_animationCache;
};

#endif