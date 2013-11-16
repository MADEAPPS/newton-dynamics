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

#include "dfbxStdafx.h"
#include "dfbxImport.h"


dImportPlugin* fbxImport::GetPlugin()
{
	static fbxImport plugin;
	return &plugin;
}

bool fbxImport::Import (const char* const fileName, dPluginInterface* const interface)
{
	dAssert (0);
/*
	dScene* const scene = interface->GetScene();
	dAssert (scene);
	NewtonWorld* const world = scene->GetNewtonWorld();
	NewtonMesh* const mesh = NewtonMeshLoadOFF (world, fileName);
	if (mesh) {
		// some OFF files are vert dirty, make sure we do not have any degenerated faces
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
*/
	return false;
}



