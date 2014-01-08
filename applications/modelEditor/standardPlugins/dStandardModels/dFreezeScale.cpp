/////////////////////////////////////////////////////////////////////////////
// Name:        dMeshNGD.cpp
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
#include "dFreezeScale.h"
#include "dUndoRedoSaveSelectedMesh.h"

dFreezeSceneScale::dFreezeSceneScale()
	:dPluginTool()
{
}

dFreezeSceneScale::~dFreezeSceneScale()
{
}


dFreezeSceneScale* dFreezeSceneScale::GetPlugin()
{
	static dFreezeSceneScale plugin;
	return &plugin;
}



bool dFreezeSceneScale::Execute (dPluginInterface* const interface)
{
	dPluginScene* const scene = interface->GetScene();
	interface->Push(new dUndoCurrentScene(interface, scene));
	scene->FreezeScale();

	dSceneRender* const render = interface->GetRender();
	dScene::dTreeNode* const geometryCache = scene->FindGetGeometryCacheNode ();
	for (void* link = scene->GetFirstChildLink(geometryCache); link; link = scene->GetNextChildLink(geometryCache, link)) {
		dScene::dTreeNode* const meshNode = scene->GetNodeFromLink(link);
		dMeshNodeInfo* const meshInfo = (dMeshNodeInfo*)scene->GetInfoFromNode(meshNode);
		if (meshInfo->IsType(dMeshNodeInfo::GetRttiType())) {
			render->InvalidateCachedDisplayList (meshInfo->GetMesh());
		}
	}

	return true;
}



dFreezeGeometryScale::dFreezeGeometryScale()
	:dPluginTool()
{
}

dFreezeGeometryScale::~dFreezeGeometryScale()
{
}


dFreezeGeometryScale* dFreezeGeometryScale::GetPlugin()
{
	static dFreezeGeometryScale plugin;
	return &plugin;
}



bool dFreezeGeometryScale::Execute (dPluginInterface* const interface)
{
	dPluginScene* const scene = interface->GetScene();
	interface->Push(new dUndoCurrentScene(interface, scene));
	scene->FreezeGeometryPivot();

	dSceneRender* const render = interface->GetRender();
	dScene::dTreeNode* const geometryCache = scene->FindGetGeometryCacheNode ();
	for (void* link = scene->GetFirstChildLink(geometryCache); link; link = scene->GetNextChildLink(geometryCache, link)) {
		dScene::dTreeNode* const meshNode = scene->GetNodeFromLink(link);
		dMeshNodeInfo* const meshInfo = (dMeshNodeInfo*)scene->GetInfoFromNode(meshNode);
		if (meshInfo->IsType(dMeshNodeInfo::GetRttiType())) {
			render->InvalidateCachedDisplayList (meshInfo->GetMesh());
		}
	}


	return true;
}

