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

#ifndef __PNG_TO_OPENGL__ 
#define __PNG_TO_OPENGL__ 

#include "ndSandboxStdafx.h"

enum TextureImageFormat
{
	m_rgb,
	m_rgba,
	m_luminace,
};

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


