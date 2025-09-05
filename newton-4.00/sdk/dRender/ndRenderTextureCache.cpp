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

#include "ndRenderStdafx.h"
#include "ndRender.h"
#include "ndRenderTexture.h"
#include "ndRenderTextureCache.h"

ndRenderTextureCache::ndRenderTextureCache(ndRender* const owner)
	:ndTree<ndSharedPtr<ndRenderTexture>, ndUnsigned64>()
	,m_owner(owner)
{
}

ndSharedPtr<ndRenderTexture> ndRenderTextureCache::GetTexture(const ndString& pathname)
{
	char pngName[256];

	snprintf(pngName, sizeof(pngName), "%s", pathname.GetStr());
	strtolwr(pngName);
	const char* const fileNameEnd = strstr(pngName, ".png");
	if (!fileNameEnd)
	{
		strcat(pngName, ".png");
		ndTrace(("subtitute texture %s with %s version\n", pathname.GetStr(), pngName));
		ndAssert(0);
	}

	ndUnsigned64 hash = ndCRC64(pngName);
	ndNode* node = Find(hash);
	if (!node)
	{
		node = Insert(ndRenderTexture::Load(*m_owner->m_context, pngName), hash);
	}
	return node->GetInfo();
}

ndSharedPtr<ndRenderTexture> ndRenderTextureCache::GetCubeMap(const ndFixSizeArray<ndString, 6>& pathnames)
{
	ndFixSizeArray<ndString, 6> pngName;
	ndAssert(pathnames.GetCount() == 6);
	
	ndUnsigned64 hash = 0;
	for (ndInt32 i = 0; i < pathnames.GetCount(); ++i)
	{
		ndAssert(pathnames[i].Size());
		char tmp[256];
		snprintf(tmp, sizeof(tmp), "%s", pathnames[i].GetStr());
		strtolwr(tmp);
		const char* const fileNameEnd = strstr(tmp, ".png");
		if (!fileNameEnd)
		{
			strcat(tmp, ".png");
			ndTrace(("subtitute texture %s with %s version\n", pathnames[i].GetStr(), tmp));
		}
		pngName.PushBack(ndString(tmp));
		hash = ndCRC64(tmp, hash);
	}
	
	ndNode* node = Find(hash);
	if (!node)
	{
		node = Insert(ndRenderTexture::LoadCubeMap(*m_owner->m_context, pngName), hash);
	}
	return node->GetInfo();
}