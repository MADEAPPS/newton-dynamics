/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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
#include "ndTargaToOpenGl.h"
#include "ndSkyBox.h"
#include "ndDemoCamera.h"

ndSkyBox::ndSkyBox(GLuint shader)
	:ndDemoEntity(dGetIdentityMatrix(), nullptr)
	,m_shader(shader)
	,m_indexBuffer(0)
	,m_vertexBuffer(0)
	,m_texturecubemap(0)
	,m_vertextArrayBuffer(0)
	,m_matrixUniformLocation(0)
	,m_textureMatrixLocation(0)
{
	GLfloat size = 200.0f;
	static GLfloat vertices[] =
	{
		 size, size, size,  -size, size, size,  -size,-size, size,  size,-size, size, // v0,v1,v2,v3 (front)
		 size, size, size,   size,-size, size,   size,-size,-size,  size, size,-size, // v0,v3,v4,v5 (right)
		 size, size, size,   size, size,-size,  -size, size,-size, -size, size, size, // v0,v5,v6,v1 (top)
		-size, size, size,  -size, size,-size,  -size,-size,-size, -size,-size, size, // v1,v6,v7,v2 (left)
		-size,-size,-size,   size,-size,-size,   size,-size, size, -size,-size, size, // v7,v4,v3,v2 (bottom)
		 size,-size,-size,  -size,-size,-size,  -size, size,-size,  size, size,-size  // v4,v7,v6,v5 (back)
	};

	// index array for glDrawElements()
	// A cube requires 36 indices = 6 sides * 2 tris * 3 verts
	static dInt32 indices[] =
	{
		0, 1, 2,   2, 3, 0,    // v0-v1-v2, v2-v3-v0 (front)
		4, 5, 6,   6, 7, 4,    // v0-v3-v4, v4-v5-v0 (right)
		8, 9,10,  10,11, 8,    // v0-v5-v6, v6-v1-v0 (top)
		12,13,14,  14,15,12,    // v1-v6-v7, v7-v2-v1 (left)
		16,17,18,  18,19,16,    // v7-v4-v3, v3-v2-v7 (bottom)
		20,21,22,  22,23,20     // v4-v7-v6, v6-v5-v4 (back)
	};

	m_textureMatrix = dGetIdentityMatrix();

	m_textureMatrix[1][1] = -1.0f;
	m_textureMatrix[1][3] = size;
		
	SetupCubeMap();

	glGenVertexArrays(1, &m_vertextArrayBuffer);
	glBindVertexArray(m_vertextArrayBuffer);
	
	glGenBuffers(1, &m_vertexBuffer); //m_vbo
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	
	//glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPT) * 24, &vtx[0], GL_STATIC_DRAW);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), &vertices[0], GL_STATIC_DRAW);
	
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0);
	
	glGenBuffers(1, &m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 36 * sizeof(dInt32), &indices[0], GL_STATIC_DRAW);
	
	glBindVertexArray(0);
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glUseProgram(m_shader);
	m_textureMatrixLocation = glGetUniformLocation(m_shader, "textureMatrix");
	m_matrixUniformLocation = glGetUniformLocation(m_shader, "projectionViewModelMatrix");
	glUseProgram(0);
}

void ndSkyBox::LoadCubeTexture(GLenum face, char* const filename)
{
	#pragma pack(1)
	struct TGAHEADER
	{
		char identsize;					// Size of ID field that follows header (0)
		char colorMapType;				// 0 = None, 1 = palette
		char imageType;					// 0 = none, 1 = indexed, 2 = rgb, 3 = grey, +8=rle
		unsigned short colorMapStart;	// First color map entry
		unsigned short colorMapLength;	// Number of colors
		unsigned char colorMapBits;		// bits per palette entry
		unsigned short xstart;			// image x origin
		unsigned short ystart;			// image y origin
		unsigned short width;			// width in pixels
		unsigned short height;			// height in pixels
		char bits;						// bits per pixel (8 16, 24, 32)
		char descriptor;				// image descriptor
	};
	#pragma pack(8)

	char fullPathName[2048];
	dGetWorkingFileName(filename, fullPathName);
	
	FILE* const pFile = fopen(fullPathName, "rb");
	dAssert(pFile);

	TGAHEADER tgaHeader;		// TGA file header
	size_t ret = fread(&tgaHeader, 18, 1, pFile);
	ret = 0;
	
	// Do byte swap for big vs little Indian
	tgaHeader.colorMapStart = SWAP_INT16(tgaHeader.colorMapStart);
	tgaHeader.colorMapLength = SWAP_INT16(tgaHeader.colorMapLength);
	tgaHeader.xstart = SWAP_INT16(tgaHeader.xstart);
	tgaHeader.ystart = SWAP_INT16(tgaHeader.ystart);
	tgaHeader.width = SWAP_INT16(tgaHeader.width);
	tgaHeader.height = SWAP_INT16(tgaHeader.height);
	
	// Get width, height, and depth of texture
	dInt32 width = tgaHeader.width;
	dInt32 height = tgaHeader.height;
	short sDepth = tgaHeader.bits / 8;
	dAssert((sDepth == 3) || (sDepth == 4));
	
	// Put some validity checks here. Very simply, I only understand
	// or care about 8, 24, or 32 bit targa's.
	if (tgaHeader.bits != 8 && tgaHeader.bits != 24 && tgaHeader.bits != 32)
	{
		dAssert(0);
		fclose(pFile);
		return;
	}
	
	// Calculate size of image buffer
	unsigned lImageSize = width * height * sDepth;
	
	// Allocate memory and check for success
	char* const pBits = (char*)dMemory::Malloc(width * height * sizeof(dInt32));
	if (pBits == nullptr)
	{
		dAssert(0);
		fclose(pFile);
		return;
	}
	
	dInt32 readret = dInt32(fread(pBits, lImageSize, 1, pFile));
	if (readret != 1)
	{
		dAssert(0);
		fclose(pFile);
		delete[] pBits;
		return;
	}

	glTexImage2D(face, 0, 4, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, pBits);
	//gluBuild2DMipmaps(face, 4, width, height, GL_BGR, GL_UNSIGNED_BYTE, pBits);

	//dAssert(glGetError() == GL_NO_ERROR);
	for (GLenum err = glGetError(); err != GL_NO_ERROR; err = glGetError())
	{
		// it looks like I am loading a texture with an invalid format, I am just ignopring this for now 
		dTrace(("****** opengl error 0x%x\n", err));
	}
	
	// Done with File
	fclose(pFile);
	dMemory::Free(pBits);
}

void ndSkyBox::SetupCubeMap()
{
	glActiveTexture(GL_TEXTURE0);
	glGenTextures(1, &m_texturecubemap);
	glBindTexture(GL_TEXTURE_CUBE_MAP, m_texturecubemap);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); 
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); 
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	LoadCubeTexture(GL_TEXTURE_CUBE_MAP_POSITIVE_X, (char*)"NewtonSky0003.tga");
	LoadCubeTexture(GL_TEXTURE_CUBE_MAP_NEGATIVE_X, (char*)"NewtonSky0001.tga");

	LoadCubeTexture(GL_TEXTURE_CUBE_MAP_POSITIVE_Y, (char*)"NewtonSky0006.tga");
	LoadCubeTexture(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, (char*)"NewtonSky0005.tga");

	LoadCubeTexture(GL_TEXTURE_CUBE_MAP_POSITIVE_Z, (char*)"NewtonSky0002.tga");
	LoadCubeTexture(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, (char*)"NewtonSky0004.tga");
}

ndSkyBox::~ndSkyBox()
{
	if (m_texturecubemap)
	{
		glDeleteTextures(1, &m_texturecubemap);
	}

	if (m_indexBuffer)
	{
		glDeleteBuffers(1, &m_indexBuffer);
	}
	
	if (m_vertexBuffer)
	{
		glDeleteBuffers(1, &m_vertexBuffer);
	}

	if (m_vertextArrayBuffer)
	{
		glDeleteVertexArrays(1, &m_vertextArrayBuffer);
	}
}

void ndSkyBox::Render(dFloat32, ndDemoEntityManager* const scene, const dMatrix&) const
{
	glCullFace(GL_FRONT);
	glFrontFace(GL_CCW);
	glDepthMask(GL_FALSE);

	ndDemoCamera* const camera = scene->GetCamera();
	
	dMatrix skyMatrix(dGetIdentityMatrix());
	dMatrix viewMatrix(camera->GetViewMatrix());
	skyMatrix.m_posit = viewMatrix.UntransformVector(dVector(0.0f, 0.25f, 0.0f, 1.0f));

	//dMatrix viewModelMatrix(skyMatrix * camera->GetViewMatrix());
	dMatrix projectionViewModelMatrix(skyMatrix * camera->GetViewMatrix() * camera->GetProjectionMatrix());
	
	glUseProgram(m_shader);
	glUniformMatrix4fv(m_textureMatrixLocation, 1, false, &m_textureMatrix[0][0]);
	glUniformMatrix4fv(m_matrixUniformLocation, 1, false, &projectionViewModelMatrix[0][0]);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_CUBE_MAP, m_texturecubemap);
	glBindVertexArray(m_vertextArrayBuffer);
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);

	glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDisableVertexAttribArray(0);
	glBindVertexArray(0);

	glUseProgram(0);
	glDepthMask(GL_TRUE);
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);
}
