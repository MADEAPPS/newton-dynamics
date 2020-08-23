/////////////////////////////////////////////////////////////////////////////
// Name:        dMeshNGD.h
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
#include "dUndoRedoSaveSelectedMesh.h"


dUndoRedoSaveSelectedMesh::dUndoRedoSaveSelectedMesh (dPluginInterface* const interface)
	:dUndoRedo()
	,dList<InfoMeshPair>()
	,m_interface(interface)
{
	dScene* const scene = interface->GetScene();
	dAssert (scene);
	dScene::dTreeNode* const geometryCache = scene->FindGetGeometryCacheNode ();
	dAssert (geometryCache);
	for (void* link = scene->GetFirstChildLink(geometryCache); link; link = scene->GetNextChildLink(geometryCache, link)) {
		dScene::dTreeNode* const node = scene->GetNodeFromLink(link);
		dNodeInfo* const info = scene->GetInfoFromNode(node);
		if (info->IsType(dMeshNodeInfo::GetRttiType())) {
			if (info->GetEditorFlags() & dNodeInfo::m_selected) {
				Append((dMeshNodeInfo*) info);
			}
		}
	}
}

dUndoRedoSaveSelectedMesh::~dUndoRedoSaveSelectedMesh()
{
}

dUndoRedo* dUndoRedoSaveSelectedMesh::CreateRedoState() const
{
	return new dUndoRedoSaveSelectedMesh (m_interface);
}


void dUndoRedoSaveSelectedMesh::RestoreState(dUndodeRedoMode mode)
{
	dSceneRender* const render = m_interface->GetRender();
	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		InfoMeshPair& pair = node->GetInfo();
		render->InvalidateCachedDisplayList (pair.m_meshInfo->GetMesh());
		pair.m_meshInfo->ReplaceMesh(NewtonMeshCreateFromMesh(pair.m_newtonMesh));
	}
}

