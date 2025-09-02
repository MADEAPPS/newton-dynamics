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
#include "ndPngToOpenGl.h"

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

static GLuint LoadGpuImage(const unsigned char* const buffer, ndInt32 width, ndInt32 hight, TextureImageFormat format)
{
	GLint iWidth = width;
	GLint iHeight = hight;

	GLenum eFormat = GL_RGBA;
	GLint iComponents = GL_RGBA;
	switch (format)
	{
		case m_rgb:
			eFormat = GL_RGB;
			iComponents = GL_RGBA;
			break;

		case m_rgba:
			eFormat = GL_RGBA;
			iComponents = GL_RGBA;
			break;

		case m_luminace:
			// apple remove this from thier opengl
			//eFormat = GL_LUMINANCE_ALPHA; 
			eFormat = GL_RED;
			iComponents = GL_RGBA;
			break;
	};

	GLuint texture = 0;
	glGenTextures(1, &texture);
	if (texture)
	{
		glBindTexture(GL_TEXTURE_2D, texture);

		// select modulate to mix texture with color for shading
		//glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

		// when texture area is small, tri linear filter mip mapped
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

		// when texture area is large, bilinear filter the first mipmap
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		float anisotropic = 0.0f;
		glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &anisotropic);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, anisotropic);

		// build our texture mip maps
		//gluBuild2DMipmaps(GL_TEXTURE_2D, iComponents, iWidth, iHeight, eFormat, GL_UNSIGNED_BYTE, buffer);
		glTexImage2D(GL_TEXTURE_2D,	0, GL_RGB, iWidth, iHeight,	0, eFormat, GL_UNSIGNED_BYTE, buffer);
		ndAssert(glGetError() == GL_NO_ERROR);
			
		glGenerateMipmap(GL_TEXTURE_2D);
		ndAssert(glGetError() == GL_NO_ERROR);
	}

	return texture;
}


//	Loads the texture from the specified file and stores it in iTexture. Note
//	that we're using the GLAUX library here, which is generally discouraged,
//	but in this case spares us having to write a bitmap loading routine.
GLuint LoadTexture(const char* const filename)
{
	char pngName[1024];
	char fullPathName[2048];

	if (!strlen(filename))
	{
		return 0;
	}

	snprintf(pngName, sizeof (pngName), "%s", filename);
	strtolwr(pngName);
	char* const fileNameEnd = strstr(pngName, ".tga");
	if (fileNameEnd)
	{
		*fileNameEnd = 0;
		strcat(pngName, ".png");
		ndTrace(("subtitute texture %s with %s version\n", filename, pngName));
		ndAssert(0);
	}
	ndGetWorkingFileName(pngName, fullPathName);

	ndTextureCache& cache = ndTextureCache::GetChache();
	ndTextureEntry* texture = cache.Find(fullPathName);
	if (texture)
	{
		texture->m_ref += 1;
	}
	else
	{
		unsigned width;
		unsigned height;
		unsigned char* pBits;
		lodepng_decode_file(&pBits, &width, &height, fullPathName, LCT_RGBA, 8);

		unsigned* const buffer = (unsigned*)pBits;
		for (ndInt32 i = 0; i < ndInt32(height / 2); i++)
		{
			unsigned* const row0 = &buffer[i * width];
			unsigned* const row1 = &buffer[(height - 1 - i) * width];
			for (ndInt32 j = 0; j < ndInt32(width); ++j)
			{
				ndSwap(row0[j], row1[j]);
			}
		}

		GLuint textureId = LoadGpuImage(pBits, int(width), int(height), m_rgba);
		lodepng_free(pBits);

		if (!textureId)
		{
			return 0;
		}
		texture = cache.InsertText(fullPathName, textureId);
	}

	return texture->m_textureID;
} 

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

