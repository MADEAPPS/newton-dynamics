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

NewtonMeshEffectExport::NewtonMeshEffectExport()
	:dExportPlugin()
{
}

NewtonMeshEffectExport::~NewtonMeshEffectExport()
{
}


NewtonMeshEffectExport* NewtonMeshEffectExport::GetPlugin()
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
	FILE* const file = fopen (fileName, "wb");
	if (file) {
		dScene* const scene = interface->GetScene();
		dAssert (scene);


		dScene::dTreeNode* const geometryCache = scene->FindGetGeometryCacheNode ();
		int count = 0;
		for (void* link = scene->GetFirstChildLink(geometryCache); link; link = scene->GetNextChildLink(geometryCache, link)) {
			dScene::dTreeNode* const node = scene->GetNodeFromLink(link);
			dNodeInfo* const info = scene->GetInfoFromNode(node);
			if (info->IsType(dMeshNodeInfo::GetRttiType())) {
				if (info->GetEditorFlags() & dPluginInterface::m_selected) {
					count ++;
				}
			}
		}
		dAssert (count >= 1);
		fwrite (& count, sizeof (int), 1, file);

		for (void* link = scene->GetFirstChildLink(geometryCache); link; link = scene->GetNextChildLink(geometryCache, link)) {
			dScene::dTreeNode* const node = scene->GetNodeFromLink(link);
			dNodeInfo* const info = scene->GetInfoFromNode(node);
			if (info->IsType(dMeshNodeInfo::GetRttiType())) {
				if (info->GetEditorFlags() & dPluginInterface::m_selected) {
					dMeshNodeInfo* const meshInfo = (dMeshNodeInfo*) info;
					NewtonMeshSerialize (meshInfo->GetMesh(), SerializeCallback, file);
				}
			}
		}

		fclose (file);
	}
}

