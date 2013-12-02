/////////////////////////////////////////////////////////////////////////////
// Name:        NewtonMeshEffectExport.cpp
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 07:45:05
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
#include "OffExportImport.h"


dImportPlugin* OffImport::GetPlugin()
{
	static OffImport plugin;
	return &plugin;
}

bool OffImport::Import (const char* const fileName, dPluginInterface* const interface)
{
	dScene* const scene = interface->GetScene();
	dAssert (scene);
	NewtonWorld* const world = scene->GetNewtonWorld();
	NewtonMesh* mesh = NewtonMeshLoadOFF (world, fileName);
	if (mesh) {
		// some OFF files are vert dirty, make sure we do not have any degenerated faces

//NewtonCollision* collision = NewtonCreateConvexHullFromMesh (world, mesh, 0, 0);
//NewtonMesh* mesh1 = NewtonMeshCreateFromCollision(collision);
//NewtonMeshDestroy(mesh);
//NewtonDestroyCollision(collision);
//mesh = mesh1;
//NewtonMeshTriangulate (xxx1);
//int vertexCount = NewtonMeshGetVertexCount(xxx1);
//int actualCount = 0;
//for (void* vertex = NewtonMeshGetFirstVertex(xxx1); vertex; vertex = NewtonMeshGetNextVertex(xxx1, vertex)) 
//{
//	++actualCount;
//}
//dAssert(actualCount == vertexCount );

		NewtonMeshFixTJoints (mesh);

		dPluginScene* const asset = new dPluginScene(world);

		dString name (GetNameFromPath(fileName));
		name = name.SubString (0, name.Find ('.'));
		
		dPluginScene::dTreeNode* const sceneNode = asset->CreateSceneNode (asset->GetRoot());
		dSceneModelInfo* const sceneNodeInfo = (dSceneModelInfo*) asset->GetInfoFromNode(sceneNode);
		sceneNodeInfo->SetName(name.GetStr());	

		dPluginScene::dTreeNode* const meshNode = asset->CreateMeshNode(sceneNode);
		dMeshNodeInfo* const instance = (dMeshNodeInfo*) asset->GetInfoFromNode(meshNode);
		instance->SetName ((name + "_mesh").GetStr());
		instance->ReplaceMesh (mesh);

		interface->MergeScene (asset);
		asset->Release();

		return true;
	}

	return false;
}



dExportPlugin* OffExport::GetPlugin()
{
	static OffExport plugin;
	return &plugin;
}


void OffExport::Export (const char* const fileName, dPluginInterface* const interface)
{
	dScene* const scene = interface->GetScene();
	dAssert (scene);

	int count = 0;
	NewtonMesh* mesh = NULL;
	dScene::dTreeNode* const geometryCache = scene->FindGetGeometryCacheNode ();
	for (void* link = scene->GetFirstChildLink(geometryCache); link; link = scene->GetNextChildLink(geometryCache, link)) {
		dScene::dTreeNode* const node = scene->GetNodeFromLink(link);
		dNodeInfo* const info = scene->GetInfoFromNode(node);
		if (info->IsType(dMeshNodeInfo::GetRttiType())) {
			if (info->GetEditorFlags() & dPluginInterface::m_selected) {
				dMeshNodeInfo* const meshInfo = (dMeshNodeInfo*) info;
				mesh = meshInfo->GetMesh();
				count ++;
			}
		}
	}

	if (count == 1) {
		NewtonMeshSaveOFF (mesh, fileName);
	}
}

