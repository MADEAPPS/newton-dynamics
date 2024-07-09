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
			// Most likely case
			//eFormat = GL_BGR;
			eFormat = GL_RGB;
			iComponents = GL_RGBA;
			break;

		case m_rgba:
			//eFormat = GL_BGRA;
			eFormat = GL_RGBA;
			iComponents = GL_RGBA;
			break;

		case m_luminace:
			//eFormat = GL_LUMINANCE;
			eFormat = GL_LUMINANCE_ALPHA;
			//eFormat = GL_ALPHA;
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
		gluBuild2DMipmaps(GL_TEXTURE_2D, iComponents, iWidth, iHeight, eFormat, GL_UNSIGNED_BYTE, buffer);
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

	ndAssert(strstr(filename, ".png"));

	sprintf(pngName, "%s", filename);
	strtolwr(pngName);
	char* const fileNameEnd = strstr(pngName, ".tga");
	if (fileNameEnd)
	{
		*fileNameEnd = 0;
		strcat(pngName, ".png");
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

		unsigned* const ptr = (ndUnsigned32*)pBits;
		for (unsigned y = 0; y < height / 2; ++y)
		{
			unsigned* const ptr0 = ptr + y * width;
			unsigned* const ptr1 = ptr + (height - y - 1) * width;
			for (unsigned x = 0; x < width; ++x)
			{
				ndSwap(ptr0[x], ptr1[x]);
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

		unsigned* const ptr = (ndUnsigned32*)pBits;
		for (unsigned y = 0; y < height / 2; ++y)
		{
			unsigned* const ptr0 = ptr + y * width;
			unsigned* const ptr1 = ptr + (height - y - 1) * width;
			for (unsigned x = 0; x < width; ++x)
			{
				ndSwap(ptr0[x], ptr1[x]);
			}
		}
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

static void TargaToPng(const char* const filename)
{
	//char fullPathName[2048];
	//ndGetWorkingFileName(filename, fullPathName);
	//strcat(fullPathName, ".tga");
	//
	//FILE* const pFile = fopen(fullPathName, "rb");
	//ndAssert(pFile);
	//
	//TGAHEADER tgaHeader;		
	//size_t ret = fread(&tgaHeader, 18, 1, pFile);
	//ret = 0;
	//
	//// Do byte swap for big vs little endian
	//tgaHeader.colorMapStart = SWAP_INT16(tgaHeader.colorMapStart);
	//tgaHeader.colorMapLength = SWAP_INT16(tgaHeader.colorMapLength);
	//tgaHeader.xstart = SWAP_INT16(tgaHeader.xstart);
	//tgaHeader.ystart = SWAP_INT16(tgaHeader.ystart);
	//tgaHeader.width = SWAP_INT16(tgaHeader.width);
	//tgaHeader.height = SWAP_INT16(tgaHeader.height);
	//
	//// Get width, height, and depth of texture
	//ndUnsigned32 width = tgaHeader.width;
	//ndUnsigned32 height = tgaHeader.height;
	//short sDepth = tgaHeader.bits / 8;
	//ndAssert((sDepth == 3) || (sDepth == 4));
	//
	//// Put some validity checks here. Very simply, I only understand
	//// or care about 8, 24, or 32 bit targa's.
	////if(tgaHeader.bits != 8 && tgaHeader.bits != 24 && tgaHeader.bits != 32) 
	//if (!((tgaHeader.bits == 8) || (tgaHeader.bits == 24) || (tgaHeader.bits == 32)))
	//{
	//	fclose(pFile);
	//	return;
	//}
	//
	//// Calculate size of image buffer
	//ndUnsigned32 lImageSize = ndUnsigned32(width * height * sDepth);
	//
	//// Allocate memory and check for success
	//unsigned char* const pBits = (unsigned char*)ndMemory::Malloc(width * height * sizeof(ndInt32));
	//if (pBits == nullptr)
	//{
	//	fclose(pFile);
	//	return;
	//}
	//
	//// Read in the bits
	//// Check for read error. This should catch RLE or other 
	//// weird formats that I don't want to recognize
	//ndInt32 readret = ndInt32(fread(pBits, lImageSize, 1, pFile));
	//if (readret != 1)
	//{
	//	fclose(pFile);
	//	ndMemory::Free(pBits);
	//	return;
	//}
	//
	//ndGetWorkingFileName(filename, fullPathName);
	//strcat(fullPathName, ".png");
	//
	//switch (sDepth)
	//{
	//	case 1:
	//	{
	//		ndAssert(0);
	//		//	format = m_luminace;
	//		break;
	//	}
	//
	//	case 3:
	//	{
	//		for (ndInt32 i = ndInt32(height * width * 3 - 3); i >= 0; i -= 3)
	//		{
	//			unsigned char r = pBits[i + 0];
	//			unsigned char g = pBits[i + 1];
	//			unsigned char b = pBits[i + 2];
	//			pBits[i + 2] = r;
	//			pBits[i + 1] = g;
	//			pBits[i + 0] = b;
	//		}
	//		lodepng_encode_file(fullPathName, pBits, width, height, LCT_RGB, 8);
	//		break;
	//	}
	//
	//	case 4:
	//	{
	//		for (ndInt32 i = ndInt32(height * width * 4 - 4); i >= 0; i -= 4)
	//		{
	//			unsigned char r = pBits[i + 0];
	//			unsigned char g = pBits[i + 1];
	//			unsigned char b = pBits[i + 2];
	//			unsigned char a = pBits[i + 3];
	//			pBits[i + 0] = b;
	//			pBits[i + 1] = g;
	//			pBits[i + 2] = r;
	//			pBits[i + 3] = a;
	//		}
	//		lodepng_encode_file(fullPathName, pBits, width, height, LCT_RGBA, 8);
	//		break;
	//	}
	//};
	//
	//// Done with File
	//fclose(pFile);
	//ndMemory::Free(pBits);

	char fullPathName[2048];
	ndGetWorkingFileName(filename, fullPathName);
	strcat(fullPathName, ".png");

	unsigned width;
	unsigned height;
	unsigned char* pBits;
	lodepng_decode_file(&pBits, &width, &height, fullPathName, LCT_RGBA, 8);

	unsigned* const ptr = (ndUnsigned32*)pBits;
	for (unsigned y = 0; y < height / 2; ++y)
	{
		unsigned* const ptr0 = ptr + y * width;
		unsigned* const ptr1 = ptr + (height - y - 1) * width;
		for (unsigned x = 0; x < width; ++x)
		{
			ndSwap(ptr0[x], ptr1[x]);
		}
	}
	lodepng_encode_file(fullPathName, pBits, width, height, LCT_RGBA, 8);
	lodepng_free(pBits);
}

void TargaToPng()
{
	//TargaToPng("smilli");
//#if (defined(WIN32) || defined(_WIN32))
#if 0
	char appPath[1024];
	char outPathName[1024];
	GetModuleFileNameA(nullptr, appPath, sizeof(appPath));
	strtolwr(appPath);
	
	char* const end = strstr(appPath, "applications");
	end[0] = 0;
	sprintf(outPathName, "%sapplications/media", appPath);
	
	char rootPath[2048];
	sprintf(rootPath, "%s/*.png", outPathName);
	
	WIN32_FIND_DATAA data;
	HANDLE handle = FindFirstFile(rootPath, &data);
	
	if (handle != INVALID_HANDLE_VALUE)
	{
		do
		{
			char fileName[256];
			sprintf(fileName, "%s", data.cFileName);
			strtolwr(fileName);
			char* const fileNameEnd = strstr(fileName, ".png");
			*fileNameEnd = 0;
	
			TargaToPng(fileName);
		} while (FindNextFile(handle, &data));
		FindClose(handle);
	}
#endif
}
