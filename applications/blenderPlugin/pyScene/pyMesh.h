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

#pragma once
#include "pyTypes.h" 
#include "pyBaseNodeInfo.h"

class pyScene;
class dMeshNodeInfo;

class pyMesh: public pyBaseNodeInfo<dMeshNodeInfo>
{
	public:
	pyMesh(pyScene* scene, void* meshNode);
	~pyMesh(void);

	void SetName (const char* name); 

	void BuildFromVertexListIndexList(int faceCount, const int *const faceIndexCount, const int *const faceMaterialIndex, 
									  const double *const vertex, const int *const vertexIndex, const double* const normals, const double* const uvs);


	void SmoothNormals (float angleInDegrees);

	int GetVertexCount();
	pyVertex GetVertex(int i);

	pyVertex GetNormal(int i);
	pyVertex GetUV0(int i);
	pyVertex GetUV1(int i);

	void* GetFirstTriangle();
	void* GetNextTriangle(void* current);
	pyTriangle GetTriangle (void* triangle);

};
