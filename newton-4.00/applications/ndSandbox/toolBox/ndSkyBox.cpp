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

// define vertex format
struct dVector3f
{
	GLfloat m_x;
	GLfloat m_y;
	GLfloat m_z;
};

struct dTexCoord2f
{
	GLfloat m_u;
	GLfloat m_v;
};

struct ndMeshPoint
{
	dVector3f m_posit;
	dVector3f m_normal;
	dTexCoord2f m_uv;
};


ndSkyBox::ndSkyBox(GLuint shader)
	:ndDemoEntity(dGetIdentityMatrix(), nullptr)
	,m_shader(shader)
	,m_indexBuffer(0)
	,m_vertexBuffer(0)
	,m_texturecubemap(0)
	,m_vetextArrayBuffer(0)
	,matrixUniformLocation(0)
	,textureMatrixLocation(0)
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

	// texture coordinate array
	//static GLfloat texCoords[] =
	//{
	//	1, 0,   0, 0,   0, 1,   1, 1,               // v0,v1,v2,v3 (front)
	//	0, 0,   0, 1,   1, 1,   1, 0,               // v0,v3,v4,v5 (right)
	//	1, 1,   1, 0,   0, 0,   0, 1,               // v0,v5,v6,v1 (top)
	//	1, 0,   0, 0,   0, 1,   1, 1,               // v1,v6,v7,v2 (left)
	//	0, 1,   1, 1,   1, 0,   0, 0,               // v7,v4,v3,v2 (bottom)
	//	0, 1,   1, 1,   1, 0,   0, 0                // v4,v7,v6,v5 (back)
	//};

	// index array for glDrawElements()
	// A cube requires 36 indices = 6 sides * 2 tris * 3 verts
	static int indices[] =
	{
		0, 1, 2,   2, 3, 0,    // v0-v1-v2, v2-v3-v0 (front)
		4, 5, 6,   6, 7, 4,    // v0-v3-v4, v4-v5-v0 (right)
		8, 9,10,  10,11, 8,    // v0-v5-v6, v6-v1-v0 (top)
		12,13,14,  14,15,12,    // v1-v6-v7, v7-v2-v1 (left)
		16,17,18,  18,19,16,    // v7-v4-v3, v3-v2-v7 (bottom)
		20,21,22,  22,23,20     // v4-v7-v6, v6-v5-v4 (back)
	};

	//struct dVectex3f
	//{
	//	GLfloat x;
	//	GLfloat y;
	//	GLfloat z;
	//};
	//struct dTexCoord2f
	//{
	//	GLfloat u;
	//	GLfloat v;
	//};
	//
	//struct VertexPT
	//{
	//	dVectex3f posit;
	//	dTexCoord2f uv;
	//};

	//dVectex3f vtx[24];
	//for (int i = 0; i < 24; i++) 
	//{
	//	vtx[i].posit.x = vertices[i * 3 + 0];
	//	vtx[i].posit.y = vertices[i * 3 + 1];
	//	vtx[i].posit.z = vertices[i * 3 + 2];
	//	
	//	vtx[i].uv.u = texCoords[i * 2 + 0];
	//	vtx[i].uv.v = texCoords[i * 2 + 1];
	//}

	m_textureMatrix = dGetIdentityMatrix();

	m_textureMatrix[1][1] = -1.0f;
	m_textureMatrix[1][3] = size;
		
	SetupCubeMap();

	glGenVertexArrays(1, &m_vetextArrayBuffer);
	glBindVertexArray(m_vetextArrayBuffer);
	
	glGenBuffers(1, &m_vertexBuffer); //m_vbo
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	
	//glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPT) * 24, &vtx[0], GL_STATIC_DRAW);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), &vertices[0], GL_STATIC_DRAW);
	
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (void*)0);

	//glEnableVertexAttribArray(1);
	//glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(VertexPT), (void*)sizeof(dVectex3f));
	
	glGenBuffers(1, &m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 36 * sizeof(int), &indices[0], GL_STATIC_DRAW);
	
	glBindVertexArray(0);
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glUseProgram(m_shader);
	textureMatrixLocation = glGetUniformLocation(m_shader, "textureMatrix");
	matrixUniformLocation = glGetUniformLocation(m_shader, "projectionViewModelMatrix");
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
	int width = tgaHeader.width;
	int height = tgaHeader.height;
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
	
	int readret = int(fread(pBits, lImageSize, 1, pFile));
	if (readret != 1)
	{
		dAssert(0);
		fclose(pFile);
		delete[] pBits;
		return;
	}

	glTexImage2D(face, 0, 4, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, pBits);
	//gluBuild2DMipmaps(face, 4, width, height, GL_BGR, GL_UNSIGNED_BYTE, pBits);
	dAssert(glGetError() == GL_NO_ERROR);
	
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

	LoadCubeTexture(GL_TEXTURE_CUBE_MAP_POSITIVE_X, "NewtonSky0003.tga");
	LoadCubeTexture(GL_TEXTURE_CUBE_MAP_NEGATIVE_X, "NewtonSky0001.tga");

	LoadCubeTexture(GL_TEXTURE_CUBE_MAP_POSITIVE_Y, "NewtonSky0006.tga");
	LoadCubeTexture(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, "NewtonSky0005.tga");

	LoadCubeTexture(GL_TEXTURE_CUBE_MAP_POSITIVE_Z, "NewtonSky0002.tga");
	LoadCubeTexture(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, "NewtonSky0004.tga");
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

	if (m_vetextArrayBuffer)
	{
		glDeleteVertexArrays(1, &m_vetextArrayBuffer);
	}
}

void ndSkyBox::Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix__) const
{
	glCullFace(GL_FRONT);
	glFrontFace(GL_CCW);
	glDepthMask(GL_FALSE);

	ndDemoCamera* const camera = scene->GetCamera();
	
	dMatrix skyMatrix(dGetIdentityMatrix());
	dMatrix viewMatrix(camera->GetViewMatrix());
	skyMatrix.m_posit = viewMatrix.UntransformVector(dVector(0.0f, 0.25f, 0.0f, 1.0f));
	//skyMatrix.m_posit = viewMatrix.UntransformVector(dVector(0.0f, 0.25f, -800.0f, 1.0f));

	dMatrix viewModelMatrix(skyMatrix * camera->GetViewMatrix());
	dMatrix projectionViewModelMatrix(skyMatrix * camera->GetViewMatrix() * camera->GetProjectionMatrix());
	
	glUseProgram(m_shader);
	//glUniformMatrix4fv(glGetUniformLocation(m_shader, "P"), 1, false, &camera->GetProjectionMatrix()[0][0]);
	//glUniformMatrix4fv(glGetUniformLocation(m_shader, "V"), 1, false, &camera->GetViewMatrix()[0][0]);
	//glUniformMatrix4fv(glGetUniformLocation(m_shader, "M"), 1, false, &skyMatrix[0][0]);
	//glUniformMatrix4fv(glGetUniformLocation(m_shader, "viewModelMatrix"), 1, false, &viewModelMatrix[0][0]);

	glUniformMatrix4fv(textureMatrixLocation, 1, false, &m_textureMatrix[0][0]);
	glUniformMatrix4fv(matrixUniformLocation, 1, false, &projectionViewModelMatrix[0][0]);

		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_CUBE_MAP, m_texturecubemap);
		glBindVertexArray(m_vetextArrayBuffer);
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
