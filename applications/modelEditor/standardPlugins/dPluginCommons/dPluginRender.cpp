/////////////////////////////////////////////////////////////////////////////
// Name:        dSceneNodeInfo.h
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 08:02:08
// RCS-ID:      
// Copyright:   Copyright (c) <2010> <Newton Game Dynamics>
// License:     
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
// 
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely
/////////////////////////////////////////////////////////////////////////////

#include "dPluginStdAfx.h"
#include "dPluginRender.h"

dPluginRender::dPluginRender(void)
	:dSceneRender()
	,m_wireFrameDisplayList()
	,m_flatShadedDisplayList()
{
}

dPluginRender::~dPluginRender(void)
{
	CleanupDisplayListCache(m_wireFrameDisplayList);
	CleanupDisplayListCache(m_flatShadedDisplayList);
}


bool dPluginRender::Init()
{
	return true;
}


void dPluginRender::CleanupDisplayListCache(dTree<int, const NewtonMesh*>& cache)
{
	while (cache.GetCount()) {
		dTree<int, const NewtonMesh*>::dTreeNode* const node = cache.GetRoot();
		DestroyDisplayList(node->GetInfo());
		cache.Remove(node);
	}
}

int dPluginRender::GetViewPortWidth() const
{
	GLint viewport[4]; 
	glGetIntegerv(GL_VIEWPORT, viewport); 
	return int (viewport[2]);
}

int dPluginRender::GetViewPortHeight() const
{
	GLint viewport[4]; 
	glGetIntegerv(GL_VIEWPORT, viewport); 
	return int (viewport[3]);
}

dMatrix dPluginRender::GetProjectionMatrix () const
{
	dMatrix matrix;
	glGetFloatv(GL_PROJECTION_MATRIX, &matrix[0][0]);
	return matrix;
}

dMatrix dPluginRender::GetModelViewMatrix() const
{
	dMatrix matrix;
	glGetFloatv(GL_MODELVIEW_MATRIX, &matrix[0][0]);
	return matrix;
}


void dPluginRender::SetPerspectiveProjection (int width, int height, dFloat fov, dFloat frontPlane, dFloat backPlane)
{
	// set the view port for this render section
	glViewport(0, 0, (GLint) width, (GLint) height);

	// set the projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	//m_backPlane = 10000.0f;
	gluPerspective(fov, GLfloat (width) /GLfloat(height), frontPlane, backPlane);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}


void dPluginRender::SetOrtographicProjection (int width, int height, dFloat minPlane, dFloat maxPlane)
{
	// set the view port for this render section
	glViewport(0, 0, (GLint) width, (GLint) height);

	// set the projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	//glOrtho(- width, width, -height, height, 0.1f, 1000.0f);
	glOrtho(0.0f, width, 0.0f, height, minPlane, maxPlane);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void dPluginRender::SetProjectionMatrix(const dMatrix& projection)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glLoadMatrix(&projection[0][0]);
}


void dPluginRender::SetModelViewMatrix(const dMatrix& modelview)
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glLoadMatrix(&modelview[0][0]);
}

int dPluginRender::CreateDisplayList(int range)
{
	return glGenLists(range);
}

void dPluginRender::DestroyDisplayList(int lists, int range)
{
	glDeleteLists(lists, range);
}

void dPluginRender::BeginDisplayList(int displayList)
{
	glNewList(displayList, GL_COMPILE);
}

void dPluginRender::EndDisplayList()
{
	glEndList();
}


void dPluginRender::LoadMatrix(const dMatrix& matrix)
{
	glLoadMatrix(&matrix[0][0]);
}

void dPluginRender::PushMatrix(const dMatrix& matrix)
{
	glPushMatrix();
	glMultMatrix(&matrix[0][0]);
}

void dPluginRender::PopMatrix()
{
	glPopMatrix();
}

void dPluginRender::DrawDisplayList(int displayList)
{
	glCallList(displayList);
}


void dPluginRender::BeginRender()
{
	// Our shading model--Goraud (smooth). 
	glShadeModel (GL_SMOOTH);

	// Culling. 
	glCullFace (GL_BACK);
	glFrontFace (GL_CCW);
	glEnable (GL_CULL_FACE);

	//	glEnable(GL_DITHER);

	// z buffer test
	glEnable(GL_DEPTH_TEST);
	glDepthFunc (GL_LEQUAL);

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_FASTEST);

	glClearColor (0.5f, 0.5f, 0.5f, 0.0f );
	//glClear( GL_COLOR_BUFFER_BIT );
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// set default lightning
	//	glDisable(GL_BLEND);
	glEnable (GL_LIGHTING);

	// make sure the model view matrix is set to identity before setting world space light sources
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

/*
	dFloat cubeColor[] = { 1.0f, 1.0f, 1.0f, 1.0 };
	glMaterialfv(GL_FRONT, GL_SPECULAR, cubeColor);
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, cubeColor);
	glMaterialf(GL_FRONT, GL_SHININESS, 50.0);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

	// one light form the Camera eye point
	GLfloat lightDiffuse0[] = { 0.5f, 0.5f, 0.5f, 0.0 };
	GLfloat lightAmbient0[] = { 0.0f, 0.0f, 0.0f, 0.0 };
	dVector camPosition (m_camera->m_matrix.m_posit);
	GLfloat lightPosition0[] = {camPosition.m_x, camPosition.m_y, camPosition.m_z};

	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition0);
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient0);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse0);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightDiffuse0);
	glEnable(GL_LIGHT0);


	// set just one directional light
	GLfloat lightDiffuse1[] = { 0.7f, 0.7f, 0.7f, 0.0 };
	GLfloat lightAmbient1[] = { 0.2f, 0.2f, 0.2f, 0.0 };
	GLfloat lightPosition1[] = { -500.0f, 200.0f, 500.0f, 0.0 };

	glLightfv(GL_LIGHT1, GL_POSITION, lightPosition1);
	glLightfv(GL_LIGHT1, GL_AMBIENT, lightAmbient1);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, lightDiffuse1);
	glLightfv(GL_LIGHT1, GL_SPECULAR, lightDiffuse1);
	glEnable(GL_LIGHT1);
*/


}

void dPluginRender::EndRender()
{
}


void dPluginRender::EnableLighting()
{
	glEnable (GL_LIGHTING);
}

void dPluginRender::DisableLighting()
{
	glDisable (GL_LIGHTING);
}


void dPluginRender::EnableTexture()
{
	glEnable (GL_TEXTURE_2D);
}

void dPluginRender::DisableTexture()
{
	glDisable (GL_TEXTURE_2D);
}

void dPluginRender::EnableBlend()
{
	glEnable (GL_BLEND);
}

void dPluginRender::DisableBlend()
{
	glDisable (GL_BLEND);
}

void dPluginRender::EnableZbuffer()
{
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
}

void dPluginRender::DisableZbuffer()
{
	glDisable(GL_DEPTH_TEST);
}

void dPluginRender::EnableBackFace()
{
	glCullFace (GL_BACK);
	glFrontFace (GL_CCW);
	glEnable(GL_CULL_FACE);
}

void dPluginRender::DisableBackFace()
{
	glDisable(GL_CULL_FACE);
}

void dPluginRender::EnableZBias(dFloat val)
{
	glEnable(GL_POLYGON_OFFSET_FILL);
	glPolygonOffset (val, val);
}

void dPluginRender::DisableZBias()
{
	glDisable(GL_POLYGON_OFFSET_FILL);
}


void dPluginRender::SetColor(const dVector& color)
{
	glColor3f (color.m_x, color.m_y, color.m_z);
}


void dPluginRender::SetMaterialDiffuse(const dVector& color)
{
	glMaterialfv(GL_FRONT, GL_DIFFUSE, &color.m_x);
}

void dPluginRender::SetMaterialAmbient(const dVector& color)
{
	glMaterialfv(GL_FRONT, GL_AMBIENT, &color.m_x);
}

void dPluginRender::SetMaterialSpecular(const dVector& color)
{
	glMaterialfv(GL_FRONT, GL_SPECULAR, &color.m_x);
}

void dPluginRender::SetMaterialShininess(dFloat normalizedPower)
{
	glMateriali (GL_FRONT, GL_SPECULAR, int (normalizedPower * 127.0f));
}


void dPluginRender::BeginLine()
{
	glBegin(GL_LINES);
}

void dPluginRender::BeginTriangle()
{
	glBegin(GL_TRIANGLES);
}


 void dPluginRender::End()
{
	glEnd();
}

void dPluginRender::SubmitNormal(const dVector& normal)
{
	glNormal3f(normal.m_x, normal.m_y, normal.m_z); 
}

void dPluginRender::SubmitVertex(const dVector& posit) 
{
	glVertex3f(posit.m_x, posit.m_y, posit.m_z); 
}




void dPluginRender::Print (int displayListFont, dFloat x, dFloat y, const char* const fmt, ... )
{
	dMatrix projection (GetProjectionMatrix());
	dMatrix viewmodel (GetModelViewMatrix());

	int width = GetViewPortWidth();
	int height = GetViewPortHeight();

	SetOrtographicProjection (width, height, 0.0f, 1.0f);
	SetModelViewMatrix(GetIdentityMatrix());

	glRasterPos2f(x, dFloat(GetViewPortHeight()) - y);

	va_list argptr;
	char string[4096];

	va_start (argptr, fmt);
	vsprintf (string, fmt, argptr);
	va_end( argptr );


	glPushAttrib(GL_LIST_BIT);
	glListBase(displayListFont - ' ');	
	int lenght = (int) strlen (string);
	glCallLists (lenght, GL_UNSIGNED_BYTE, string);	
	glPopAttrib();				

	SetProjectionMatrix(projection);
	SetModelViewMatrix(viewmodel);
}

int dPluginRender::GetCachedWireframeDisplayList(NewtonMesh* const mesh)
{
	dTree<int, const NewtonMesh*>::dTreeNode* node = m_wireFrameDisplayList.Find(mesh);
	if (!node) {
		int displayList = CreateDisplayList();
		BeginDisplayList(displayList);

		int stride = NewtonMeshGetVertexStrideInByte(mesh) / sizeof (dFloat64);
		const dFloat64* const vertexList = NewtonMeshGetVertexArray(mesh);

		BeginLine();
		for (void* edge = NewtonMeshGetFirstEdge (mesh); edge; edge = NewtonMeshGetNextEdge (mesh, edge)) {
			int i0;
			int i1;
			NewtonMeshGetEdgeIndices (mesh, edge, &i0, &i1);
			dVector p0 (dFloat(vertexList[i0 * stride + 0]), dFloat(vertexList[i0 * stride + 1]), dFloat(vertexList[i0 * stride + 2]), 0.0f);
			dVector p1 (dFloat(vertexList[i1 * stride + 0]), dFloat(vertexList[i1 * stride + 1]), dFloat(vertexList[i1 * stride + 2]), 0.0f);
			DrawLine(p0, p1);
		}

		End();
		EndDisplayList();
		node = m_wireFrameDisplayList.Insert(displayList, mesh);
	}

	return node->GetInfo();
}


int dPluginRender::GetCachedFlatShadedDisplayList(NewtonMesh* const mesh)
{
	dTree<int, const NewtonMesh*>::dTreeNode* node = m_flatShadedDisplayList.Find(mesh);
	if (!node) {
		int displayList = CreateDisplayList();
		BeginDisplayList(displayList);

		int stride = NewtonMeshGetVertexStrideInByte(mesh) / sizeof (dFloat64);
		const dFloat64* const vertexList = NewtonMeshGetVertexArray(mesh);

		BeginTriangle();
		for (void* face = NewtonMeshGetFirstFace(mesh); face; face = NewtonMeshGetNextFace(mesh, face)) {
			if (!NewtonMeshIsFaceOpen (mesh, face)) {
				int indices[1024];
				int vertexCount = NewtonMeshGetFaceIndexCount (mesh, face);
				_ASSERTE (vertexCount < sizeof (indices)/sizeof (indices[0]));
				NewtonMeshGetFaceIndices (mesh, face, indices);

				dFloat64 normal[4];
				NewtonMeshCalculateFaceNormal (mesh, face, normal);
				SubmitNormal(dVector (dFloat (normal[0]), dFloat (normal[1]), dFloat (normal[2]), 0.0f));

				int i0 =  indices[0];
				int i1 =  indices[1];
				dVector p0 (dFloat(vertexList[i0 * stride + 0]), dFloat(vertexList[i0 * stride + 1]), dFloat(vertexList[i0 * stride + 2]), 0.0f);
				dVector p1 (dFloat(vertexList[i1 * stride + 0]), dFloat(vertexList[i1 * stride + 1]), dFloat(vertexList[i1 * stride + 2]), 0.0f);
				for (int i = 2; i < vertexCount; i ++) {
					int i2 = indices[i];
					dVector p2 (dFloat(vertexList[i2 * stride + 0]), dFloat(vertexList[i2 * stride + 1]), dFloat(vertexList[i2 * stride + 2]), 0.0f);
					DrawTriangle(p0, p1, p2);
					p1 = p2;
				}
			}
		}

		End();
		EndDisplayList();
		node = m_flatShadedDisplayList.Insert(displayList, mesh);
	}

	return node->GetInfo();
}


void dPluginRender::InvalidateCachedDisplayList(const NewtonMesh* const mesh)
{
	dTree<int, const NewtonMesh*>::dTreeNode* const wireFrameNode = m_wireFrameDisplayList.Find(mesh);
	if (wireFrameNode) {
		DestroyDisplayList(wireFrameNode->GetInfo());
		m_wireFrameDisplayList.Remove(wireFrameNode);
	}

	dTree<int, const NewtonMesh*>::dTreeNode* const flatShadedNode = m_flatShadedDisplayList.Find(mesh);
	if (flatShadedNode) {
		DestroyDisplayList(flatShadedNode->GetInfo());
		m_flatShadedDisplayList.Remove(flatShadedNode);
	}

}