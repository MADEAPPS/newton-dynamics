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
#include "NewtonMeshEffectExportImport.h"


dImportPlugin* NewtonMeshEffectImport::GetPlugin()
{
	static NewtonMeshEffectImport plugin;
	return &plugin;
}

#define MESH_HEADER	"Newton Mesh"

void NewtonMeshEffectImport::DeserializeCallback (void* const serializeHandle, void* const buffer, int size)
{
	FILE* const file = (FILE*) serializeHandle;
	fread (buffer, size, 1, file);
}

bool NewtonMeshEffectImport::Import (const char* const fileName, dPluginInterface* const interface)
{
	FILE* const file = fopen (fileName, "rb");
	if (file) {
		dScene* const scene = interface->GetScene();
		dAssert (scene);

		char name[2048];
		fread (name, strlen(MESH_HEADER), 1, file);
		if (!strncmp (name, MESH_HEADER, strlen(MESH_HEADER))) {

			int size;
			NewtonWorld* const world = scene->GetNewtonWorld();
			dPluginScene* const asset = new dPluginScene(world);

			dString ext ("_node");
			fread (&size, sizeof (int), 1, file);
			dAssert (size < sizeof (name));
			fread (name, size, 1, file);
			name[size] = 0;

			NewtonMesh* const newtonMesh = NewtonMeshCreateFromSerialization (world, DeserializeCallback, file);
			dAssert (newtonMesh);

			dPluginScene::dTreeNode* const sceneNode = asset->CreateSceneNode(asset->GetRoot());
			dSceneModelInfo* const sceneNodeInfo = (dSceneModelInfo*) asset->GetInfoFromNode(sceneNode);
			dString nodeName (name);
			nodeName += ext;
			sceneNodeInfo->SetName(nodeName.GetStr());

			dPluginScene::dTreeNode* const boxMesh = asset->CreateMeshNode(sceneNode);
			dMeshNodeInfo* const instance = (dMeshNodeInfo*) asset->GetInfoFromNode(boxMesh);
			instance->SetName(name);
			instance->ReplaceMesh (newtonMesh);


			interface->MergeScene (asset);
			asset->Release();
		}

		fclose (file);
		return true;
	}
	return false;
}



dExportPlugin* NewtonMeshEffectExport::GetPlugin()
{
	static NewtonMeshEffectExport plugin;
	return &plugin;
}


void NewtonMeshEffectExport::SerializeCallback (void* const serializeHandle, const void* const buffer, int size)
{
	FILE* const file = (FILE*) serializeHandle;
	fwrite (buffer, size, 1, file);
}

void NewtonMeshEffectExport::Export (const char* const fileName, dPluginInterface* const interface)
{
	dScene* const scene = interface->GetScene();
	dAssert (scene);

	int count = 0;
	dScene::dTreeNode* const geometryCache = scene->FindGetGeometryCacheNode ();
	for (void* link = scene->GetFirstChildLink(geometryCache); link; link = scene->GetNextChildLink(geometryCache, link)) {
		dScene::dTreeNode* const node = scene->GetNodeFromLink(link);
		dNodeInfo* const info = scene->GetInfoFromNode(node);
		if (info->IsType(dMeshNodeInfo::GetRttiType())) {
			if (info->GetEditorFlags() & dNodeInfo::m_selected) {
				count ++;
			}
		}
	}

	if (count == 1) {
		FILE* const file = fopen (fileName, "wb");
		if (file) {
			for (void* link = scene->GetFirstChildLink(geometryCache); link; link = scene->GetNextChildLink(geometryCache, link)) {
				dScene::dTreeNode* const node = scene->GetNodeFromLink(link);
				dNodeInfo* const info = scene->GetInfoFromNode(node);
				if (info->IsType(dMeshNodeInfo::GetRttiType())) {
					if (info->GetEditorFlags() & dNodeInfo::m_selected) {
						dMeshNodeInfo* const meshInfo = (dMeshNodeInfo*) info;

						char* const newton = MESH_HEADER;
						fwrite (newton, strlen (newton), 1, file);

						dString name (meshInfo->GetName()); 
						int size = name.Size();
						fwrite (&size, sizeof (int), 1, file);
						fwrite (name.GetStr(), size, 1, file);
						NewtonMeshSerialize (meshInfo->GetMesh(), SerializeCallback, file);
					}
				}
			}

			fclose (file);
		}
	}
}

