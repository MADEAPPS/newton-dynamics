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

#ifndef _D_TRINAGULATE_MESH_H_
#define _D_TRINAGULATE_MESH_H_


class InfoMeshPair
{
	public:
	InfoMeshPair (dMeshNodeInfo* const info)
		:m_meshInfo (info)
		,m_newtonMesh (NewtonMeshCreateFromMesh(info->GetMesh())) 
	{
		m_meshInfo->AddRef();
	}

	InfoMeshPair (const InfoMeshPair& copy)
		:m_meshInfo (copy.m_meshInfo)
		,m_newtonMesh(NewtonMeshCreateFromMesh(copy.m_newtonMesh))
	{
		m_meshInfo->AddRef();
	}

	~InfoMeshPair ()
	{
		NewtonMeshDestroy (m_newtonMesh);
		m_meshInfo->Release();
	}

	dMeshNodeInfo* m_meshInfo;
	NewtonMesh* m_newtonMesh;
};

class UndoRedodMeshes: public dUndoRedo, dList<InfoMeshPair>
{
	public:
	UndoRedodMeshes (dPluginInterface* const interface)
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
				if (info->GetEditorFlags() & dPluginInterface::m_selected) {
					Append((dMeshNodeInfo*) info);
				}
			}
		}
	}

	~UndoRedodMeshes()
	{
	}

	using dUndoRedo::operator new;
	using dUndoRedo::operator delete;

	protected:
	virtual void RestoreState(dUndodeRedoMode mode)
	{
		dSceneRender* const render = m_interface->GetRender();
		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			InfoMeshPair& pair = node->GetInfo();
			render->InvalidateCachedDisplayList (pair.m_meshInfo->GetMesh());
			pair.m_meshInfo->ReplaceMesh(NewtonMeshCreateFromMesh(pair.m_newtonMesh));
		}
	}

	virtual dUndoRedo* CreateRedoState() const
	{
		return new UndoRedodMeshes (m_interface);
	}

	dPluginInterface* m_interface;
};


class dMeshTriangulateMesh: public dPluginTool
{
	public:
	dMeshTriangulateMesh();
	~dMeshTriangulateMesh();
	static dMeshTriangulateMesh* GetPlugin();

	virtual const char* GetMenuName () { return GetSignature();}
	virtual const char* GetDescription () {return "Triangulate selected meshes";}
	virtual const char* GetSignature () {return "Triangulate Mesh";}
	virtual bool Execute (dPluginInterface* const interface);
};

#endif