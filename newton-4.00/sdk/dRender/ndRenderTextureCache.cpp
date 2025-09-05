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

#include "ndRenderTexture.h"
#include "ndRenderTextureCache.h"

#if 0
class ndTextureEntry
{
	public:
	ndTextureEntry()
	{
	}

	ndUnsigned32 m_ref;
	GLuint m_textureID;
	ndUnsigned64 m_hash;
};

class ndTextureCache : public ndList<ndTextureEntry>
{
	public: 
	~ndTextureCache()
	{
		ndAssert(GetCount() == 0);
		ndAssert(m_idMap.GetCount() == 0);
		ndAssert(m_hashMap.GetCount() == 0);
	}

	ndTextureEntry* Find(const char* const texName)
	{
		ndUnsigned64 crc = MakeHash(texName);
		ndTree<ndList<ndTextureEntry>::ndNode*, ndUnsigned64>::ndNode* const node = m_hashMap.Find(crc);
		return node ? &node->GetInfo()->GetInfo() : nullptr;
	}

	ndTextureEntry* InsertText (const char* const texName, GLuint id)
	{
		ndList<ndTextureEntry>::ndNode* const node = Append();
		ndTextureEntry& entry = node->GetInfo();
		entry.m_ref = 1;
		entry.m_textureID = id;
		entry.m_hash = MakeHash(texName);
		m_hashMap.Insert(node, entry.m_hash);
		m_idMap.Insert(node, entry.m_textureID);
		return &entry;
	}

	void RemoveById (GLuint id)
	{
		ndTree<ndList<ndTextureEntry>::ndNode*, GLuint>::ndNode* const node = m_idMap.Find(id);
		if (node)
		{
			ndList<ndTextureEntry>::ndNode* const texNode = node->GetInfo();
			ndTextureEntry& entry = texNode->GetInfo();
			entry.m_ref = entry.m_ref - 1;
			ndAssert(entry.m_ref != 0xffffffff);
			if (entry.m_ref == 0)
			{
				glDeleteTextures(1, &entry.m_textureID);
				m_idMap.Remove(node);
				m_hashMap.Remove(entry.m_hash);
				Remove(texNode);
			}
		}
	}

	void AddReference(GLuint id)
	{
		ndTree<ndList<ndTextureEntry>::ndNode*, GLuint>::ndNode* const node = m_idMap.Find(id);
		ndAssert(node);
		if (node)
		{
			ndList<ndTextureEntry>::ndNode* const texNode = node->GetInfo();
			ndTextureEntry& entry = texNode->GetInfo();
			entry.m_ref = entry.m_ref + 1;
		}
	}

	ndUnsigned64 MakeHash(const char* const texName) const
	{
		char name[256];
		strcpy(name, texName);
		strtolwr(name);
		ndUnsigned64 crc = ndCRC64(name);
		return crc;
	}

	void CleanUp()
	{
		ndAssert(GetCount() == 0);
		while (GetCount())
		{
			ndTextureEntry& entry = GetFirst()->GetInfo();
			RemoveById(entry.m_textureID);
		}
	}

	static ndTextureCache& GetChache()
	{
		static ndTextureCache texCache;
		return texCache;
	}

	ndTree<ndList<ndTextureEntry>::ndNode*, GLuint> m_idMap;
	ndTree<ndList<ndTextureEntry>::ndNode*, ndUnsigned64> m_hashMap;
};

GLuint LoadCubeMapTexture(
	const char* const filename_x0, const char* const filename_x1,
	const char* const filename_y0, const char* const filename_y1,
	const char* const filename_z0, const char* const filename_z1)
{
	const char* namesArray[6];
	GLenum faceArray[6];

	namesArray[0] = filename_x0;
	namesArray[1] = filename_x1;
	faceArray[0] = GL_TEXTURE_CUBE_MAP_POSITIVE_X;
	faceArray[1] = GL_TEXTURE_CUBE_MAP_NEGATIVE_X;

	namesArray[2] = filename_y0;
	namesArray[3] = filename_y1;
	faceArray[2] = GL_TEXTURE_CUBE_MAP_POSITIVE_Y;
	faceArray[3] = GL_TEXTURE_CUBE_MAP_NEGATIVE_Y;

	namesArray[4] = filename_z0;
	namesArray[5] = filename_z1;
	faceArray[4] = GL_TEXTURE_CUBE_MAP_POSITIVE_Z;
	faceArray[5] = GL_TEXTURE_CUBE_MAP_NEGATIVE_Z;

	GLuint texturecubemap;
	glActiveTexture(GL_TEXTURE0);
	glGenTextures(1, &texturecubemap);
	glBindTexture(GL_TEXTURE_CUBE_MAP, texturecubemap);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	ndTextureCache& cache = ndTextureCache::GetChache();
	for (ndInt32 i = 0; i < 6; ++i)
	{
		char fullPathName[2048];
		ndGetWorkingFileName(namesArray[i], fullPathName);
		ndAssert(!cache.Find(namesArray[i]));
		
		unsigned width;
		unsigned height;
		unsigned char* pBits;
		lodepng_decode_file(&pBits, &width, &height, fullPathName, LCT_RGBA, 8);

		glTexImage2D(faceArray[i], 0, GL_RGBA, int(width), int(height), 0, GL_RGBA, GL_UNSIGNED_BYTE, pBits);
		lodepng_free(pBits);
	}

	ndTextureEntry* const texture = cache.InsertText(namesArray[0], texturecubemap);
	return texture->m_textureID;
}

void ReleaseTexture (GLuint texture)
{
	ndTextureCache::GetChache().RemoveById (texture);
}

GLuint AddTextureRef(GLuint texture)
{
	ndTextureCache::GetChache().AddReference(texture);
	return texture;
}

void TextureCacheCleanUp()
{
	ndTextureCache& cache = ndTextureCache::GetChache();
	cache.CleanUp();
}
#endif


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
		node = Insert(ndRenderTexture::Load(pngName), hash);
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
		node = Insert(ndRenderTexture::LoadCubeMap(pngName), hash);
	}
	return node->GetInfo();
}