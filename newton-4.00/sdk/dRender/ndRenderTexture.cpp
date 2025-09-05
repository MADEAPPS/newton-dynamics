/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndRenderStdafx.h"
#include "ndRenderTexture.h"
#include "ndRenderTextureImage.h"

ndRenderTexture::ndRenderTexture()
	:ndContainersFreeListAlloc<ndRenderTexture>()
	,m_hash(0)
{
}

ndRenderTexture::~ndRenderTexture()
{
}

ndSharedPtr<ndRenderTexture> ndRenderTexture::Load(const ndString& pathname)
{
	char tmp[256];
	snprintf(tmp, sizeof(tmp), "%s", pathname.GetStr());
	strtolwr(tmp);
	const char* const fileNameEnd = strstr(tmp, ".png");
	if (!fileNameEnd)
	{
		strcat(tmp, ".png");
		ndTrace(("subtitute texture %s with %s version\n", pathname.GetStr(), tmp));
		ndAssert(0);
	}

	ndSharedPtr<ndRenderTexture> texture(nullptr);
	unsigned width;
	unsigned height;
	unsigned char* pBits;
	unsigned ret = lodepng_decode_file(&pBits, &width, &height, tmp, LCT_RGBA, 8);
	ndAssert(!ret);
	if (!ret)
	{
		// from targa legacy reasons, the texture is upsizedown, 
		// so I have to flip it
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
		texture = ndSharedPtr<ndRenderTexture>(new ndRenderTextureImage(pBits, int(width), int(height), m_rgba));
		lodepng_free(pBits);
	}
	return texture;
}

ndSharedPtr<ndRenderTexture> ndRenderTexture::LoadCubeMap(const ndFixSizeArray<ndString, 6>& pathnames)
{
	ndFixSizeArray<ndUnsigned32, 6> faceArray;
	faceArray.PushBack(GL_TEXTURE_CUBE_MAP_POSITIVE_X);
	faceArray.PushBack(GL_TEXTURE_CUBE_MAP_NEGATIVE_X);
	faceArray.PushBack(GL_TEXTURE_CUBE_MAP_POSITIVE_Y);
	faceArray.PushBack(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y);
	faceArray.PushBack(GL_TEXTURE_CUBE_MAP_POSITIVE_Z);
	faceArray.PushBack(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z);

	ndSharedPtr<ndRenderTexture> texture(new ndRenderTextureCubeMapImage());
	ndRenderTextureCubeMapImage* const cubeMap = (ndRenderTextureCubeMapImage*)*texture;
	for (ndInt32 i = 0; i < pathnames.GetCount(); ++i)
	{
		ndAssert(pathnames[i].Size());
		char tmp[256];
		snprintf(tmp, sizeof(tmp), "%s", pathnames[i].GetStr());
		strtolwr(tmp);
		char* const fileNameEnd = strstr(tmp, ".png");
		if (!fileNameEnd)
		{
			ndAssert(0);
			*fileNameEnd = 0;
			strcat(tmp, ".png");
			ndTrace(("subtitute texture %s with %s version\n", pathnames[i].GetStr(), tmp));
		}

		unsigned width;
		unsigned height;
		unsigned char* pBits = nullptr;
		lodepng_decode_file(&pBits, &width, &height, tmp, LCT_RGBA, 8);
		ndAssert(pBits);

		cubeMap->LoadFace(faceArray[i], pBits, int(width), int(height), m_rgba);
		lodepng_free(pBits);
	}

	return texture;
}