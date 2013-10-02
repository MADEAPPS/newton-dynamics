/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

// NewtonModelEditor.cpp : Defines the entry point for the application.
//

#include "dPluginStdafx.h"
#include "dPluginUtils.h"
#include "dPluginScene.h"
#include "dPluginRender.h"
#include "dPluginRecord.h"
#include "dPluginCamera.h"
#include "dPluginInterface.h"
#include "dUndoCurrentScene.h"

typedef dPluginRecord** (CALLBACK* GetPluginArray)();

dPluginInterface::dPluginInterface(void)
	:dUndoRedoManager()
	,m_scene(NULL)
	,m_render(NULL)
	,m_currentCamera(NULL)
	,m_filePathFile(NULL)
{
	m_render = new dPluginRender;
	m_render->Init();
}

dPluginInterface::~dPluginInterface(void)
{
	while (m_allPlugins.GetFirst()) {
		FreeLibrary(m_allPlugins.GetFirst()->GetInfo());
		m_allPlugins.Remove(m_allPlugins.GetFirst());
	}

	DestroyScene ();

	m_render->Release();
}


void dPluginInterface::DestroyScene ()
{
	if (m_scene) {
		m_scene->Release();
		m_scene = NULL; 
	}
}

void dPluginInterface::RefreshExplorerEvent(bool clear) const
{

}

const char* dPluginInterface::GetFilePath() const
{
	return m_filePathFile;
}


dPluginInterface::dPluginDll::dListNode* dPluginInterface::LoadPlugins(const char* const path)
{
	char plugInPath[2048];
	char rootPathInPath [2048];
	GetAplicationDirectory(plugInPath);
	strcat (plugInPath, path);
	sprintf (rootPathInPath, "%s/*.dll", plugInPath);

	dPluginDll::dListNode* const firstNode = m_allPlugins.GetLast();

	// scan for all plugins in this folder
	_finddata_t data;
	intptr_t handle = _findfirst (rootPathInPath, &data);
	if (handle != -1) {
		do {
			sprintf (rootPathInPath, "%s/%s", plugInPath, data.name);

			HMODULE module = LoadLibrary(rootPathInPath);
			if (module) {
				// get the interface function pointer to the Plug in classes
				GetPluginArray GetPluginsTable = (GetPluginArray) GetProcAddress (module, "GetPluginArray"); 
				if (GetPluginsTable) {
					m_allPlugins.Append(module);
				} else {
					FreeLibrary(module);
				}
			}
		} while (_findnext (handle, &data) == 0);
		_findclose (handle);
	}

	for (dPluginDll::dListNode* dllNode = firstNode ? firstNode->GetNext() : m_allPlugins.GetFirst(); dllNode; dllNode = dllNode->GetNext()) {
		HMODULE module = dllNode->GetInfo();

		GetPluginArray GetPluginsTable = (GetPluginArray) GetProcAddress (module, "GetPluginArray"); 
		dPluginRecord** const table = GetPluginsTable();

		for (int i = 0; table[i]; i ++) {
			dPluginRecord* const plugin = table[i];
			const char* const pluginName = plugin->GetSignature();
			dCRCTYPE key = dCRC64 (pluginName);
			_ASSERTE (!m_pluginDictionary.Find(key));
			m_pluginDictionary.Insert(plugin, key);
		}
	}

	return firstNode ? firstNode->GetNext() : m_allPlugins.GetFirst();
}


dPluginScene* dPluginInterface::GetScene() const
{
	return m_scene;
}

void dPluginInterface::SetScene(dPluginScene* const scene)
{
	m_scene = scene;
}

dSceneRender* dPluginInterface::GetRender() const
{
	return m_render;
}

dPluginCamera* dPluginInterface::GetCamera() const
{
	return m_currentCamera;
}

dPluginRecord* dPluginInterface::GetPlugin(const char* const signature) const
{
	dTree<dPluginRecord*, dCRCTYPE>::dTreeNode* const node = m_pluginDictionary.Find(dCRC64 (signature));
	_ASSERTE (node);

	dPluginRecord* plugin = NULL;
	if (node) {
		plugin = node->GetInfo();
	}
	return plugin;
}

void* dPluginInterface::GetFirstPluginNode() const
{
	dTree<dPluginRecord*, dCRCTYPE>::Iterator iter (m_pluginDictionary);
	iter.Begin();
	return iter.GetNode(); 
}

void* dPluginInterface::GetNextPluginNode(void* const pluginNode) const
{
	dTree<dPluginRecord*, dCRCTYPE>::Iterator iter (m_pluginDictionary);
	iter.Set ((dTree<dPluginRecord*, dCRCTYPE>::dTreeNode*)pluginNode);
	iter ++;
	return iter.GetNode();
}

dPluginRecord* dPluginInterface::GetPluginFromNode(void* const pluginNode) const
{
	dTree<dPluginRecord*, dCRCTYPE>::dTreeNode* const node = (dTree<dPluginRecord*, dCRCTYPE>::dTreeNode*)pluginNode;
	return node ? node->GetInfo() : NULL;
}

bool dPluginInterface::HasMeshSelection (dCRCTYPE meshType)
{
	dScene::dTreeNode* const geometryCache = m_scene->FindGetGeometryCacheNode ();
	if (geometryCache) {
		for (void* link = m_scene->GetFirstChildLink(geometryCache); link; link = m_scene->GetNextChildLink(geometryCache, link)) {
			dScene::dTreeNode* const node = m_scene->GetNodeFromLink(link);
			dNodeInfo* const info = m_scene->GetInfoFromNode(node);
			if (info->IsType(meshType)) {
				if (info->GetEditorFlags() & dPluginInterface::m_selected) {
					return true;
				}
			}
		}
	}
	return false;
}


//void dPluginInterface::ClearExplorerExpand()
//{
//	m_ExplorerExpand.RemoveAll();
//}

//void dPluginInterface::AddExplorerExpandNode(void* const incidentLink, bool state)
//{
//	m_ExplorerExpand.Insert(state ? 1 : 0, incidentLink);
//}


//void dPluginInterface::SetExplorerExpandNodeState(void* const incidentLink, bool state)
//{
//	dTree<int, void*>::dTreeNode* const ptr = m_ExplorerExpand.Find(incidentLink);
//	_ASSERTE (ptr);
//	if (ptr) {
//		ptr->GetInfo() = state ? 1 : 0;
//	}
//}

//bool dPluginInterface::GetExplorerExpandNodeState(void* const incidentLink) const
//{
//	dTree<int, void*>::dTreeNode* const ptr = m_ExplorerExpand.Find(incidentLink);
//	return ptr ? (ptr->GetInfo() ? true : false) : false;
//}

//void dPluginInterface::ClearSelection()
//{
//	m_selection.RemoveAll();
//}


//bool dPluginInterface::IsNodeSelected(void* const incidentLink) const
//{
//	return m_selection.Find(incidentLink) ? true : false;
//}


//void dPluginInterface::AddToSelection(void* const incidentLink)
//{
//	m_selection.Insert(0, incidentLink);
//}


//void dPluginInterface::RemoveFromSelection(void* const incidentLink)
//{
//	m_selection.Remove (incidentLink);
//}

//void* dPluginInterface::GetFirtSelectedNode() const
//{
//	dTree<int, void*>::Iterator iter (m_selection);
//	iter.Begin();
//	return iter.GetNode() ? iter.GetNode()->GetKey() : NULL;
//}

//void* dPluginInterface::GetNextSelectedNode(void* const node) const
//{
//	dTree<int, void*>::Iterator iter (m_selection);
//	iter.Set (m_selection.Find(node));
//	iter ++;
//	return iter.GetNode() ? iter.GetNode()->GetKey() : NULL;
//}

void dPluginInterface::MergeScene (dPluginScene* const scene)
{
	dPluginScene::Iterator iter (*scene);
//	for (iter.Begin(); iter; iter ++) {
//		dPluginScene::dTreeNode* const sceneNode = iter.GetNode();
//		dSceneNodeInfo* const sceneInfo = (dSceneNodeInfo*) scene->GetInfoFromNode(sceneNode);
//		sceneInfo->SetEditorFlags(sceneInfo->GetEditorFlags() | m_expanded);
//	}

	Push(new dUndoCurrentScene(this, scene));
	m_scene->MergeScene(scene);
}
