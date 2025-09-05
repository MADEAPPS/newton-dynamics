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
#include "ndRenderTextureImage.h"

ndRenderTextureImageCommon::ndRenderTextureImageCommon()
	:ndRenderTexture()
	,m_texture(0)
{
}

ndRenderTextureImageCommon::~ndRenderTextureImageCommon()
{
	if (m_texture)
	{
		glDeleteTextures(1, &m_texture);
	}
}

ndRenderTextureImage::ndRenderTextureImage(const unsigned char* const buffer, ndInt32 width, ndInt32 hight, TextureImageFormat format)
	:ndRenderTextureImageCommon()
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

	glGenTextures(1, &m_texture);
	if (m_texture)
	{
		glBindTexture(GL_TEXTURE_2D, m_texture);

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
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, iWidth, iHeight, 0, eFormat, GL_UNSIGNED_BYTE, buffer);
		ndAssert(glGetError() == GL_NO_ERROR);

		glGenerateMipmap(GL_TEXTURE_2D);
		ndAssert(glGetError() == GL_NO_ERROR);
	}
}


ndRenderTextureCubeMapImage::ndRenderTextureCubeMapImage()
	:ndRenderTextureImageCommon()
{
	glActiveTexture(GL_TEXTURE0);
	glGenTextures(1, &m_texture);
	glBindTexture(GL_TEXTURE_CUBE_MAP, m_texture);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
}

void ndRenderTextureCubeMapImage::LoadFace(unsigned mapSide, const unsigned char* const buffer, ndInt32 width, ndInt32 height, TextureImageFormat)
{
	glTexImage2D(mapSide, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
}