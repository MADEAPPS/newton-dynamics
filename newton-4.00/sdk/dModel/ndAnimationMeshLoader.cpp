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

#include "ndModelStdafx.h"
#include "ndAnimationSequence.h"
#include "ndAnimationMeshLoader.h"
#include "ndAnimationKeyframesTrack.h"

ndAnimationMeshLoader::ndAnimationMeshLoader()
	:ndMeshLoader()
{
}

ndAnimationMeshLoader::~ndAnimationMeshLoader()
{
}

const ndSharedPtr<ndAnimationSequence> ndAnimationMeshLoader::FindSequence(const ndString& fbxPathAnimName) const
{
	ndTree<ndSharedPtr<ndAnimationSequence>, ndString>::ndNode* const node = m_animationCache.Find(fbxPathAnimName);
	if (node)
	{
		return node->GetInfo();
	}
	return ndSharedPtr<ndAnimationSequence>(nullptr);
}

ndSharedPtr<ndAnimationSequence> ndAnimationMeshLoader::ImportFbxAnimation(const ndString& fbxPathAnimName)
{
	ndTree<ndSharedPtr<ndAnimationSequence>, ndString>::ndNode* node = m_animationCache.Find(fbxPathAnimName);
	if (!node)
	{
		ndFbxMeshLoader animLoader;
		ndSharedPtr<ndAnimationSequence> sequence (animLoader.LoadAnimation(fbxPathAnimName.GetStr()));
		if (sequence)
		{
			node = m_animationCache.Insert(sequence, fbxPathAnimName);
		}
	}
	return node ? node->GetInfo() : ndSharedPtr<ndAnimationSequence>(nullptr);
}

ndSharedPtr<ndAnimationSequence> ndAnimationMeshLoader::GetAnimationSequence(const ndString& pathAnimName)
{
	//ndAssert(0);
	//return ndSharedPtr<ndAnimationSequence>(nullptr);
	return ImportFbxAnimation(pathAnimName);
}

void ndAnimationMeshLoader::SetTranslationTracks(const ndString& boneName)
{
	ndTree<ndSharedPtr<ndAnimationSequence>, ndString>::Iterator it(m_animationCache);
	for (it.Begin(); it; it++)
	{
		const ndSharedPtr<ndAnimationSequence>& cycle = it.GetNode()->GetInfo();
		for (ndList<ndAnimationKeyFramesTrack>::ndNode* node = cycle->GetTracks().GetFirst(); node; node = node->GetNext())
		{
			ndAnimationKeyFramesTrack& track = node->GetInfo();
			ndString name(track.GetName());
			name.ToLower();
			if (name.Find(boneName) != -1)
			{
				ndAnimationKeyFramesTrack& translationTrack = cycle->GetTranslationTrack();
				ndVector translation(ndVector::m_zero);
				ndReal offset = ndReal(track.m_position[0].m_x);
				for (ndInt32 i = 0; i < track.m_position.GetCount(); ++i)
				{
					translation.m_x = track.m_position[i].m_x - offset;
					translationTrack.m_position.PushBack(translation);
					translationTrack.m_position.m_time.PushBack(track.m_position.m_time[i]);
					track.m_position[i].m_x = offset;
				}
				break;
			}
		}
	}
}

ndString ndAnimationMeshLoader::GetPath(const ndString& fullPathName) const
{
	const char* ptr = strrchr(fullPathName.GetStr(), '/');
	if (!ptr)
	{
		ptr = strrchr(fullPathName.GetStr(), '\\');
	}
	return ndString (fullPathName.GetStr(), ndInt32(fullPathName.Size() - strlen(ptr + 1)));
}

ndString ndAnimationMeshLoader::GetName(const ndString& fullPathName) const
{
	const char* ptr1 = strrchr(fullPathName.GetStr(), '.');
	const char* ptr0 = strrchr(fullPathName.GetStr(), '/');
	if (!ptr0)
	{
		ptr0 = strrchr(fullPathName.GetStr(), '\\');
	}
	ndAssert(ptr0);
	ndAssert(ptr1);
	ndInt32 start = ndInt32(fullPathName.Size() - strlen(ptr0 + 1));
	ndInt32 end = ndInt32(fullPathName.Size() - strlen(ptr1));
	return fullPathName.SubString(start, end - start);
}

bool ndAnimationMeshLoader::ImportFbx(const ndString& fbxPathMeshName)
{
	ndFbxMeshLoader loader;
	m_mesh = ndSharedPtr<ndMesh>(loader.LoadMesh(fbxPathMeshName.GetStr(), false));
#if 0
	//ndAssert(0);
	ndTrace(("exporting mesh %s\n", fbxPathMeshName.GetStr()));
	ndString tmpName(fbxPathMeshName);
	tmpName.ToLower();
	ndString exportName(tmpName.SubString(0, tmpName.Find(".fbx")) + ".nd");
	SaveMesh(exportName);
#endif
	return m_mesh;
}

