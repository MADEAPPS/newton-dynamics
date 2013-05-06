/////////////////////////////////////////////////////////////////////////////
// Name:        pyMesh.h
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

#include "StdAfx.h"
#include "pyMesh.h"
#include "pyScene.h"

pyMesh::pyMesh(pyScene* scene, void* meshNode)
	:pyBaseNodeInfo<dMeshNodeInfo>(scene, meshNode)
{
}

pyMesh::~pyMesh(void)
{
}


void pyMesh::SetName (const char* name)
{
	char tmp[256];
	strcpy (tmp, name);
	if (!strstr (name, "_mesh")) {
		strcat (tmp, "_mesh");
	}

	dMeshNodeInfo* info = GetInfo();
	info->SetName(tmp);
}


int pyMesh::GetVertexCount()
{
	dMeshNodeInfo* meshInfo = GetInfo();
	NewtonMesh* mesh = meshInfo->GetMesh();
	return NewtonMeshGetVertexCount(mesh);
}


void* pyMesh::GetFirstTriangle()
{
	dMeshNodeInfo* meshInfo = GetInfo();
	NewtonMesh* mesh = meshInfo->GetMesh();

	for (void* face = NewtonMeshGetFirstFace (mesh); face;  face = NewtonMeshGetNextFace (mesh, face)) {
		if (!NewtonMeshIsFaceOpen(mesh, face)) {
			return face;
		}
	}
	return NULL;
}

void* pyMesh::GetNextTriangle(void* face)
{
	dMeshNodeInfo* meshInfo = GetInfo();
	NewtonMesh* mesh = meshInfo->GetMesh();

	for (face = NewtonMeshGetNextFace (mesh, face); face; face = NewtonMeshGetNextFace (mesh, face)) {
		if (!NewtonMeshIsFaceOpen(mesh, face)) {
			return face;
		}
	}
	return NULL;
}


pyVertex pyMesh::GetVertex(int i)
{
_ASSERTE (0);
return pyVertex();
/*
	dMeshNodeInfo* meshInfo = GetInfo();
	NewtonMesh* mesh = meshInfo->GetMesh();

	int stride = NewtonMeshGetVertexStrideInByte(mesh) / sizeof (dFloat);
	dFloat* points = NewtonMeshGetVertexArray(mesh);

	const dMatrix& matrix = meshInfo->GetPivotMatrix ();
	dVector p (matrix.TransformVector(dVector (points[stride * i + 0], points[stride * i + 1], points[stride * i + 2], 0.0f))) ;

	pyVertex vertex;
	vertex.x = p.m_x;
	vertex.y = p.m_y;
	vertex.z = p.m_z;
	return vertex;
*/
}

pyVertex pyMesh::GetNormal(int i)
{
_ASSERTE (0);
return pyVertex();
	/*
	dMeshNodeInfo* meshInfo = GetInfo();
	NewtonMesh* mesh = meshInfo->GetMesh();

	int stride = NewtonMeshGetPointStrideInByte(mesh) / sizeof (dFloat);
	dFloat* points = NewtonMeshGetNormalArray(mesh);

	const dMatrix& matrix = meshInfo->GetPivotMatrix ();
	dVector p (matrix.RotateVector(dVector (points[stride * i + 0], points[stride * i + 1], points[stride * i + 2], 0.0f))) ;

	pyVertex vertex;
	vertex.x = p.m_x;
	vertex.y = p.m_y;
	vertex.z = p.m_z;
	return vertex;
*/
}


pyVertex pyMesh::GetUV0(int i)
{
	_ASSERTE (0);
	return pyVertex();
	/*

	dMeshNodeInfo* meshInfo = GetInfo();
	NewtonMesh* mesh = meshInfo->GetMesh();

	int stride = NewtonMeshGetPointStrideInByte(mesh) / sizeof (dFloat);
	dFloat* uv = NewtonMeshGetUV0Array(mesh);

	pyVertex point;
	point.x = uv[i * stride + 0];
	point.y = uv[i * stride + 1];
	point.z = 0.0;
	return point;
*/
}

pyVertex pyMesh::GetUV1(int i)
{
	_ASSERTE (0);
	return pyVertex();
	/*

	dMeshNodeInfo* meshInfo = GetInfo();
	NewtonMesh* mesh = meshInfo->GetMesh();

	int stride = NewtonMeshGetPointStrideInByte(mesh) / sizeof (dFloat);
	dFloat* uv = NewtonMeshGetUV1Array(mesh);

	pyVertex point;
	point.x = uv[i * stride + 0];
	point.y = uv[i * stride + 1];
	point.z = 0.0;
	return point;
*/
}




pyTriangle pyMesh::GetTriangle (void* triangle)
{
	dMeshNodeInfo* meshInfo = GetInfo();
	NewtonMesh* mesh = meshInfo->GetMesh();

	int vertexIndices[128];
	int pointIndices[128];

	_ASSERTE (NewtonMeshGetFaceIndexCount (mesh, triangle) == 3);
	NewtonMeshGetFaceIndices (mesh, triangle, vertexIndices);
	NewtonMeshGetFacePointIndices (mesh, triangle, pointIndices);

	pyTriangle tria;
	tria.p0.vertex = vertexIndices[0];
	tria.p1.vertex = vertexIndices[1];
	tria.p2.vertex = vertexIndices[2];

	tria.p0.normal = pointIndices[0];
	tria.p1.normal = pointIndices[1];
	tria.p2.normal = pointIndices[2];

	tria.p0.uv0 = pointIndices[0];
	tria.p1.uv0 = pointIndices[1];
	tria.p2.uv0 = pointIndices[2];

	tria.p0.uv1 = pointIndices[0];
	tria.p1.uv1 = pointIndices[1];
	tria.p2.uv1 = pointIndices[2];

	tria.materialIndex = NewtonMeshGetFaceMaterial(mesh, triangle);
	return tria;
}



void pyMesh::BuildFromVertexListIndexList(int faceCount, const int *const faceIndexCount, const int *const faceMaterialIndex, 
	const double *const vertex, const int *const vertexIndex, const double* const normals, const double* const uvs)
{

	// find the highest vertex index
	int indexCount = 0;
	int vetexCount = 0;
	for (int i = 0; i < faceCount; i ++) {
		int count = faceIndexCount[i];
		for (int j = 0; j < count; j ++) {
			vetexCount = max (vetexCount, vertexIndex[indexCount] + 1);
			indexCount ++;
		}
	}
	
	int* nIndex = new int [indexCount];
	dVector* v = new dVector [vetexCount];
	dVector* n = new dVector [indexCount];
	dVector* uv0 = new dVector [indexCount];
	dVector* uv1 = new dVector [indexCount];;

	for (int i = 0; i < vetexCount; i ++) {
		int j = i * 3;
		v[i] = dVector (dFloat (vertex[j + 0]), dFloat (vertex[j + 1]), dFloat (vertex[j + 2]), 0.0f);
	}

	for (int i = 0; i < indexCount; i ++) {
		nIndex[i] = i;

		int j = i * 3;
		n[i] = dVector (dFloat (normals[j + 0]), dFloat (normals[j + 1]), dFloat (normals[j + 2]), 0.0f);

		j = i * 2;
		uv0[i] = dVector (dFloat (uvs[j + 0]), dFloat (uvs[j + 1]), 0.0f, 0.0f);
		uv1[i] = dVector (0.0f, 0.0f, 0.0f, 0.0f);
	}

	dMeshNodeInfo* meshInfo = GetInfo();
	meshInfo->BuildFromVertexListIndexList(faceCount, faceIndexCount, faceMaterialIndex, 
										   &v[0].m_x, sizeof (dVector), vertexIndex, &n[0].m_x, sizeof (dVector), nIndex,
										   &uv0[0].m_x, sizeof (dVector), nIndex, &uv1[0].m_x, sizeof (dVector), nIndex);

	delete[] nIndex;
	delete[] v;
	delete[] n;
	delete[] uv0;
	delete[] uv1;
}

void pyMesh::SmoothNormals (float angleInDegrees)
{
	dMeshNodeInfo* meshInfo = GetInfo();
	meshInfo->SmoothNormals(angleInDegrees * 3.1416f / 180.0f);
}