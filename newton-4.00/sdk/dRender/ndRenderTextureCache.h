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

#ifndef __NR_RENDER_TEXTURE_CACHE_H__ 
#define __NR_RENDER_TEXTURE_CACHE_H__ 

#include "ndRenderStdafx.h"

#if 0

void TextureCacheCleanUp();
GLuint LoadTexture(const char* const filename);

GLuint LoadCubeMapTexture(
	const char* const filename_x0, const char* const filename_x1,
	const char* const filename_y0, const char* const filename_y1,
	const char* const filename_z0, const char* const filename_z1);

GLuint AddTextureRef (GLuint texture);
void ReleaseTexture (GLuint texture);

const char* FindTextureById (GLuint textureID);

#endif

class ndRender;
class ndRenderTexture;

class ndRenderTextureCache: public ndTree<ndSharedPtr<ndRenderTexture>, ndUnsigned64>
{
	public:
	ndRenderTextureCache(ndRender* const owner);

	ndSharedPtr<ndRenderTexture> GetTexture(const ndString& pathname);
	ndSharedPtr<ndRenderTexture> GetCubeMap(const ndFixSizeArray<ndString, 6>& pathnames);

	ndRender* m_owner;
};

#endif


