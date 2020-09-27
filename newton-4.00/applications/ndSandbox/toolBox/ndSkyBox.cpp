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


#ifdef USING_GLES_4

ndSkyBox::ndSkyBox(GLuint shader)
	:ndDemoEntity(dGetIdentityMatrix(), nullptr)
	,m_shader(shader)
	,m_texturecubemap(0)
	,m_vao(0)
	,m_ibo(0)
	//,m_textures(nullptr)
{
	//m_textures = new TextureEx();

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
	m_uvOffest = sizeof(vertices);

	// texture coordinate array
	static GLfloat texCoords[] =
	{
		1, 0,   0, 0,   0, 1,   1, 1,               // v0,v1,v2,v3 (front)
		0, 0,   0, 1,   1, 1,   1, 0,               // v0,v3,v4,v5 (right)
		1, 1,   1, 0,   0, 0,   0, 1,               // v0,v5,v6,v1 (top)
		1, 0,   0, 0,   0, 1,   1, 1,               // v1,v6,v7,v2 (left)
		0, 1,   1, 1,   1, 0,   0, 0,               // v7,v4,v3,v2 (bottom)
		0, 1,   1, 1,   1, 0,   0, 0                // v4,v7,v6,v5 (back)
	};

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

	struct dVectex3f
	{
		GLfloat x;
		GLfloat y;
		GLfloat z;
	};

	struct dTexCoord2f
	{
		GLfloat u;
		GLfloat v;
	};

	struct VertexPT
	{
		dVectex3f posit;
		dTexCoord2f uv;
	};

	VertexPT vtx[24];
	for (int i = 0; i < 24; i++) 
	{//VertexPTN
		vtx[i].posit.x = vertices[i * 3 + 0];
		vtx[i].posit.y = vertices[i * 3 + 1];
		vtx[i].posit.z = vertices[i * 3 + 2];
		//
		vtx[i].uv.u = texCoords[i * 2 + 0];
		vtx[i].uv.v = texCoords[i * 2 + 1];
	}
	// Setup the cubemap texture.

//	SetupCubeMap();

	glGenVertexArrays(1, &m_vao);
	glBindVertexArray(m_vao);
	
	glGenBuffers(1, &m_vertexBuffer); //m_vbo
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
	
	glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPT) * 24, &vtx[0], GL_STATIC_DRAW);
	
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPT), (void*)0);

	glEnableVertexAttribArray(1);
	//glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(VertexPT), (void*)(offsetof(VertexPT, VertexPT::uv)));
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(VertexPT), (void*)sizeof(dVectex3f));
	
	glGenBuffers(1, &m_ibo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ibo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 36 * sizeof(int), &indices[0], GL_STATIC_DRAW);
	
	glBindVertexArray(0);
	
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void ndSkyBox::SetupCubeMap()
{
/*
	glActiveTexture(GL_TEXTURE0);
	// Exemple this call is deprecated.
	//glEnable(GL_TEXTURE_CUBE_MAP);
	glGenTextures(1, &m_texturecubemap);
	glBindTexture(GL_TEXTURE_CUBE_MAP, m_texturecubemap);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	//
	int ws, hs, cp = 0;
	//
	unsigned char* textemp = nullptr;
	//char* textemp2 = nullptr;
	GLenum format = 0;
	//
	char fullPathName[2048];
	//
	GetFullFilePathEx("NewtonSky0003.tga", fullPathName);
	textemp = m_textures->TextureFromFileData(fullPathName, ws, hs, cp);
	// I have try to convert the newton tga texture for use with new opengl but I always get problems with data.
	// I have success to remove all opengl error when I load the texture but now it is inverted and with wrong color...
	// I have temporary disable it for use a other texture loader.
	//m_textures1[0] = LoadTextureEx("NewtonSky0003.tga", cp, ws, hs, textemp2);
	//
	//
	if (cp == 1) format = GL_RED;
	if (cp == 3) format = GL_RGB;
	if (cp == 4) format = GL_RGBA;
	//if (cp == 5) format = GL_BGRA;
	//
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X, 0, format, ws, hs, 0, format, GL_UNSIGNED_BYTE, textemp);
	m_textures->TextureFreeData(textemp);
	textemp = nullptr;
	// this delete the date when you use LoadTextureEx
	//glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X, 0, format, ws, hs, 0, format, GL_UNSIGNED_BYTE, textemp2);
	//if (textemp2) deleteTextureData(textemp2);
	//textemp2 = nullptr;
	//
	ws, hs, cp = 0;
	GetFullFilePathEx("NewtonSky0001.tga", fullPathName);
	textemp = m_textures->TextureFromFileData(fullPathName, ws, hs, cp);
	//
	if (cp == 1) format = GL_RED;
	if (cp == 3) format = GL_RGB;
	if (cp == 4) format = GL_RGBA;
	//
	glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 0, format, ws, hs, 0, format, GL_UNSIGNED_BYTE, textemp);
	m_textures->TextureFreeData(textemp);
	textemp = nullptr;
	//
	ws, hs, cp = 0;
	GetFullFilePathEx("NewtonSky0005.tga", fullPathName);
	textemp = m_textures->TextureFromFileData(fullPathName, ws, hs, cp);
	//
	if (cp == 1) format = GL_RED;
	if (cp == 3) format = GL_RGB;
	if (cp == 4) format = GL_RGBA;
	//
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 0, format, ws, hs, 0, format, GL_UNSIGNED_BYTE, textemp);
	m_textures->TextureFreeData(textemp);
	textemp = nullptr;
	//
	ws, hs, cp = 0;
	GetFullFilePathEx("NewtonSky0006.tga", fullPathName);
	textemp = m_textures->TextureFromFileData(fullPathName, ws, hs, cp);
	//
	if (cp == 1) format = GL_RED;
	if (cp == 3) format = GL_RGB;
	if (cp == 4) format = GL_RGBA;
	//
	glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 0, format, ws, hs, 0, format, GL_UNSIGNED_BYTE, textemp);
	m_textures->TextureFreeData(textemp);
	textemp = nullptr;
	//
	ws, hs, cp = 0;
	GetFullFilePathEx("NewtonSky0002.tga", fullPathName);
	textemp = m_textures->TextureFromFileData(fullPathName, ws, hs, cp);
	//
	if (cp == 1) format = GL_RED;
	if (cp == 3) format = GL_RGB;
	if (cp == 4) format = GL_RGBA;
	//
	glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 0, format, ws, hs, 0, format, GL_UNSIGNED_BYTE, textemp);
	m_textures->TextureFreeData(textemp);
	textemp = nullptr;
	//
	ws, hs, cp = 0;
	GetFullFilePathEx("NewtonSky0004.tga", fullPathName);
	textemp = m_textures->TextureFromFileData(fullPathName, ws, hs, cp);
	//
	if (cp == 1) format = GL_RED;
	if (cp == 3) format = GL_RGB;
	if (cp == 4) format = GL_RGBA;
	//
	glTexImage2D(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 0, format, ws, hs, 0, format, GL_UNSIGNED_BYTE, textemp);
	m_textures->TextureFreeData(textemp);
	textemp = nullptr;
*/
}

ndSkyBox::~ndSkyBox()
{
	//if (m_textures)
	//{
	//	delete m_textures;
	//}
	
	// delete VBO when program terminated
	if (m_vertexBuffer)
	{
		glDeleteBuffers(1, &m_vertexBuffer);
	}
	
	if (m_indexBuffer)
	{
		glDeleteBuffers(1, &m_indexBuffer);
	}

	if (m_vao)
	{
		glDeleteVertexArrays(1, &m_vao);
	}
}

void ndSkyBox::Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix__) const
{
/*
	dMatrix skyMatrix(dGetIdentityMatrix());
	glCullFace(GL_FRONT);
	glFrontFace(GL_CCW);
	glDepthMask(GL_FALSE);

	ndDemoCamera* const camera = scene->GetCamera();

	skyMatrix = camera->GetViewMatrix().Inverse();
	skyMatrix[0] = dVector(1.0f, 0.0f, 0.0f, 0.0f); // front
	skyMatrix[1] = dVector(0.0f, 1.0f, 0.0f, 0.0f); // up
	skyMatrix[2] = dVector(0.0f, 0.0f, 1.0f, 0.0f); // right
	
	glUseProgram(m_shader);
	glUniformMatrix4fv(glGetUniformLocation(skyshad, "P"), 1, false, &camera->GetProjectionMatrix()[0][0]);
	glUniformMatrix4fv(glGetUniformLocation(skyshad, "V"), 1, false, &camera->GetViewMatrix()[0][0]);
	glUniformMatrix4fv(glGetUniformLocation(skyshad, "M"), 1, false, &skyMatrix[0][0]);


		//glActiveTexture(GL_TEXTURE0);
		//glBindTexture(GL_TEXTURE_CUBE_MAP, m_texturecubemap);
		//glBindVertexArray(m_vao);
		//glEnableVertexAttribArray(0);
		//glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
		//glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
		//glBindBuffer(GL_ARRAY_BUFFER, 0);
		//glDisableVertexAttribArray(0);
		//glBindVertexArray(0);

	glUseProgram(0);
	glDepthMask(GL_TRUE);
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);
*/
}

#else
ndSkyBox::ndSkyBox(GLuint shader)
	:ndDemoEntity(dGetIdentityMatrix(), nullptr)
	,m_shader(shader)
	,m_displayList(glGenLists(1))
{
	m_textures[0] = LoadTexture("NewtonSky0001.tga");
	m_textures[1] = LoadTexture("NewtonSky0002.tga");
	m_textures[2] = LoadTexture("NewtonSky0003.tga");
	m_textures[3] = LoadTexture("NewtonSky0004.tga");
	m_textures[4] = LoadTexture("NewtonSky0005.tga");
	m_textures[5] = LoadTexture("NewtonSky0006.tga");

	glNewList(m_displayList, GL_COMPILE);
	DrawMesh();
	glEndList();
}

ndSkyBox::~ndSkyBox()
{
	if (m_displayList) 
	{
		glDeleteLists(m_displayList, 1);
	}
	for (int i = 0; i < int(sizeof(m_textures) / sizeof(m_textures[0])); i++) 
	{
		ReleaseTexture(m_textures[i]);
	}
}

void ndSkyBox::DrawMesh() const
{
	dVector size(200.0f);

	glUseProgram(m_shader);
	glUniform1i(glGetUniformLocation(m_shader, "texture"), 0);

	glColor3f(1.0f, 1.0f, 1.0f);

	GLfloat padd = 1.0e-3f;
	glDisable(GL_BLEND);
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	// front
	glBindTexture(GL_TEXTURE_2D, m_textures[0]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(GLfloat(size.m_x), GLfloat(size.m_y), -GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(-GLfloat(size.m_x), GLfloat(size.m_y), -GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(-GLfloat(size.m_x), -GLfloat(size.m_y), -GLfloat(size.m_z));
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(GLfloat(size.m_x), -GLfloat(size.m_y), -GLfloat(size.m_z));
	glEnd();

	// left
	glBindTexture(GL_TEXTURE_2D, m_textures[3]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(-GLfloat(size.m_x), GLfloat(size.m_y), -GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(-GLfloat(size.m_x), GLfloat(size.m_y), GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(-GLfloat(size.m_x), -GLfloat(size.m_y), GLfloat(size.m_z));
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(-GLfloat(size.m_x), -GLfloat(size.m_y), -GLfloat(size.m_z));
	glEnd();

	// right
	glBindTexture(GL_TEXTURE_2D, m_textures[1]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(GLfloat(size.m_x), GLfloat(size.m_y), GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(GLfloat(size.m_x), GLfloat(size.m_y), -GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(GLfloat(size.m_x), -GLfloat(size.m_y), -GLfloat(size.m_z));
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(GLfloat(size.m_x), -GLfloat(size.m_y), GLfloat(size.m_z));
	glEnd();

	// back
	glBindTexture(GL_TEXTURE_2D, m_textures[2]);
	glBegin(GL_QUADS);
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(-GLfloat(size.m_x), GLfloat(size.m_y), GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(GLfloat(size.m_x), GLfloat(size.m_y), GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(GLfloat(size.m_x), -GLfloat(size.m_y), GLfloat(size.m_z));
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(-GLfloat(size.m_x), -GLfloat(size.m_y), GLfloat(size.m_z));
	glEnd();

	// top
	glBindTexture(GL_TEXTURE_2D, m_textures[4]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(-GLfloat(size.m_x), GLfloat(size.m_y), -GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(GLfloat(size.m_x), GLfloat(size.m_y), -GLfloat(size.m_z));
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(GLfloat(size.m_x), GLfloat(size.m_y), GLfloat(size.m_z));
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(-GLfloat(size.m_x), GLfloat(size.m_y), GLfloat(size.m_z));
	glEnd();

	// bottom
	glBindTexture(GL_TEXTURE_2D, m_textures[5]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f + padd, 0.0f + padd); glVertex3f(-GLfloat(size.m_x), -GLfloat(size.m_y), GLfloat(size.m_z));
	glTexCoord2f(0.0f + padd, 1.0f - padd); glVertex3f(GLfloat(size.m_x), -GLfloat(size.m_y), GLfloat(size.m_z));
	glTexCoord2f(1.0f - padd, 1.0f - padd); glVertex3f(GLfloat(size.m_x), -GLfloat(size.m_y), -GLfloat(size.m_z));
	glTexCoord2f(1.0f - padd, 0.0f + padd); glVertex3f(-GLfloat(size.m_x), -GLfloat(size.m_y), -GLfloat(size.m_z));
	glEnd();

	//	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);

	glUseProgram(0);
}

void ndSkyBox::Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& worldMatrix) const
{
	dMatrix matrix;

	// get the model viewMatrix; 
	glGetFloat(GL_MODELVIEW_MATRIX, &matrix[0][0]);

	dMatrix skyMatrix(dGetIdentityMatrix());
	skyMatrix.m_posit = matrix.UntransformVector(dVector(0.0f, 0.25f, 0.0f, 1.0f));

	glPushMatrix();
	glMultMatrix(&skyMatrix[0][0]);

	if (m_displayList) 
	{
		glCallList(m_displayList);
	}
	else 
	{
		DrawMesh();
	}

	// render the rest of the hierarchy
	glPopMatrix();
}

#endif