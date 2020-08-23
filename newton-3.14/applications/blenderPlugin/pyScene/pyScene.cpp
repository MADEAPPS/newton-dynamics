/////////////////////////////////////////////////////////////////////////////
// Name:        pyScene.h
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
#include "pyScene.h"

pyScene::pyScene(void)
{
	m_scene = new dScene (NewtonCreate ());
}

pyScene::~pyScene(void)
{
	NewtonDestroy (m_scene->GetNewtonWorld());
}

void pyScene::Load (const char* name)
{
	m_scene->CleanUp();
	m_scene->Deserialize(name);

	dScene::Iterator iter (*m_scene);
	for (iter.Begin(); iter; iter ++) {
		dScene::dTreeNode* node = iter.GetNode();
		dNodeInfo* info = m_scene->GetInfoFromNode(node);
		if (info->GetTypeId() == dMeshNodeInfo::GetRttiType()) {
			dMeshNodeInfo* mesh = (dMeshNodeInfo*) m_scene->GetInfoFromNode(node);
			mesh->ConvertToTriangles();
		}
	}

	dMatrix globalRotation (dPitchMatrix(3.14159265f * 0.5f) * dRollMatrix(3.14159265f * 0.5f));
	m_scene->BakeTransform (globalRotation);
}

void pyScene::Save (const char* name)
{
	dMatrix globalRotation (dPitchMatrix(3.14159265f * 0.5f) * dRollMatrix(3.14159265f * 0.5f));
	globalRotation = globalRotation.Inverse();
	m_scene->BakeTransform (globalRotation);
	m_scene->Serialize (name);
}


void* pyScene::GetRoot()
{
	return m_scene->GetRootNode();
}

void* pyScene::GetFirstChildLink(void* parent)
{
	dScene::dTreeNode* root = (dScene::dTreeNode*) parent;
	return m_scene->GetFirstChild(root);
}

void* pyScene::GetNextChildLink(void* parent, void* link)
{
	dScene::dTreeNode* root = (dScene::dTreeNode*) parent;
	return m_scene->GetNextChild(root, link);
}


void* pyScene::GetNodeFromLink(void* link)
{
	return m_scene->GetNodeFromLink(link);
}

const char* pyScene::GetNodeName (void* node)
{
	dNodeInfo* info = m_scene->GetInfoFromNode((dScene::dTreeNode*) node);
	return info->GetName();
}



bool pyScene::IsSceneNode (void* node)
{
	dNodeInfo* info = m_scene->GetInfoFromNode((dScene::dTreeNode*) node);
	return info->IsType(dSceneNodeInfo::GetRttiType()) ? true : false; 
}

bool pyScene::IsMeshNode (void* node)
{
	dNodeInfo* info = m_scene->GetInfoFromNode((dScene::dTreeNode*) node);
	return info->IsType(dMeshNodeInfo::GetRttiType()) ? true : false;  
}

bool pyScene::IsMaterialNode (void* node)
{
	dNodeInfo* info = m_scene->GetInfoFromNode((dScene::dTreeNode*) node);
	return info->IsType(dMaterialNodeInfo::GetRttiType()) ? true : false;  
}

bool pyScene::IsTextureNode (void* node)
{
	dNodeInfo* info = m_scene->GetInfoFromNode((dScene::dTreeNode*) node);
	return info->IsType(dTextureNodeInfo::GetRttiType()) ? true : false;  
}

void pyScene::AddReference (void* parentNode, void* childNode)
{
	m_scene->AddReference((dScene::dTreeNode*) parentNode, (dScene::dTreeNode*) childNode);
}

void* pyScene::CreateSceneNode(void* parent)
{
	if (parent == NULL) {
		parent = m_scene->GetRootNode();
	}
	return m_scene->CreateSceneNode((dScene::dTreeNode*) parent);
}

void* pyScene::CreateMeshNode(void* parent)
{
	return m_scene->CreateMeshNode((dScene::dTreeNode*) parent);
}

void* pyScene::CreateRigidbodyNode(void* parent)
{
	return m_scene->CreateRigidbodyNode ((dScene::dTreeNode*) parent);
}

void* pyScene::CreateMaterialNode(void* parentMesh, int materilID)
{
	return m_scene->CreateMaterialNode ((dScene::dTreeNode*) parentMesh, materilID);
}

void* pyScene::CreateTextureNode(const char* pathName)
{
	return m_scene->CreateTextureNode (pathName);
}

