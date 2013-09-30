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

#ifndef _D_PLUGIN_INTERFACE_H_
#define _D_PLUGIN_INTERFACE_H_

#include "dPluginScene.h"
#include "dPluginUtils.h"

class dPluginMesh;
class dSceneRender;
class dPluginCamera;
class dPluginRecord;


class dPluginInterface: public dUndoRedoManager
{
	public:
	class dPluginDll: public dList <HMODULE>
	{
	};
/*
	class dSceneExplorer: public dHierarchy<dSceneExplorer>
	{
		public:
		dSceneExplorer (dNodeInfo* const info)
			:dHierarchy<dSceneExplorer>(info->GetName())
			,m_info(info)
		{
			m_info->AddRef();
		}

		virtual dBaseHierarchy* CreateClone () const
		{
			dAssert(0);
			return NULL;
		}

		~dSceneExplorer ()
		{
			m_info->Release();
		}

		dNodeInfo* m_info;
	};

	class ExplorerDictionary: public dTree<dSceneExplorer*, dNodeInfo*>
	{
		public:
	};
*/
/*
	class AssetPluginAssociation
	{
		public:
		AssetPluginAssociation ()
			:m_asset(NULL)
			,m_plugin(NULL)
		{
		}

		AssetPluginAssociation (dPluginScene* const asset, dPluginMesh* const plugin)
			:m_asset(asset)
			,m_plugin(plugin)
		{
			m_asset->AddRef();
		}

		AssetPluginAssociation (const AssetPluginAssociation& copy)
			:m_asset(copy.m_asset)
			,m_plugin(copy.m_plugin)
		{
			m_asset->AddRef();
		}

		~AssetPluginAssociation()
		{
			if (m_asset) {
				m_asset->Release();
			}
		}

		dPluginScene* m_asset;
		dPluginMesh* m_plugin;
	};
*/


	DPLUGIN_API dPluginInterface(void);
	DPLUGIN_API virtual  ~dPluginInterface(void);
	
	DPLUGIN_API virtual dPluginScene* GetScene() const;
	DPLUGIN_API virtual void SetScene(dPluginScene* const scene);

	DPLUGIN_API virtual dSceneRender* GetRender() const;

	DPLUGIN_API virtual dPluginCamera* GetCamera() const;
	DPLUGIN_API virtual dPluginRecord* GetPlugin(const char* const signature) const;

	DPLUGIN_API virtual void* GetFirstPluginNode() const;
	DPLUGIN_API virtual void* GetNextPluginNode(void* const pluginNode) const;
	DPLUGIN_API virtual dPluginRecord* GetPluginFromNode(void* const pluginNode) const;

//	virtual dAssetList::dListNode* AddAsset(dPluginScene* const scene, dPluginMesh* plugin);
//	virtual void RemoveAsset(dAssetList::dListNode* const node);
//	virtual void RemoveAllAsset();
//	virtual dAssetList::dListNode* GetCurrentAssetNode() const;
//	virtual void SetCurrentAssetNode(dAssetList::dListNode* const node);
//	virtual dAssetList::dListNode* GetFirstAssetNode() const;
//	virtual dAssetList::dListNode* GetNextAssetNode(dAssetList::dListNode* const node) const;
//	virtual dPluginScene* GetAssetFromNode(dAssetList::dListNode* const node) const;
//	virtual dPluginMesh* GetAssetPluginFromNode(dAssetList::dListNode* const node) const;

	DPLUGIN_API virtual const char* GetFilePath() const;

	DPLUGIN_API virtual void ClearSelection();
	DPLUGIN_API virtual bool IsNodeSelected(void* const incidentLink) const;
	DPLUGIN_API virtual void AddToSelection(void* const incidentLink);
	DPLUGIN_API virtual void RemoveFromSelection(void* const incidentLink);
	DPLUGIN_API virtual void* GetFirtSelectedNode() const;
	DPLUGIN_API virtual void* GetNextSelectedNode(void* const incidentLink) const;


//	DPLUGIN_API virtual void ClearExplorerExpand();
//	DPLUGIN_API virtual void AddExplorerExpandNode(void* const incidentLink, bool state);
//	DPLUGIN_API virtual void SetExplorerExpandNodeState(void* const incidentLink, bool state);
//	DPLUGIN_API virtual bool GetExplorerExpandNodeState(void* const incidentLink) const;


	DPLUGIN_API virtual void DestroyScene ();
	DPLUGIN_API virtual void MergeScene (dPluginScene* const asset);
//	DPLUGIN_API virtual void MergeExplorer();
	DPLUGIN_API virtual void RefreshExplorerEvent(bool clear) const;

	protected:
	DPLUGIN_API dPluginDll::dListNode* LoadPlugins(const char* const path);

	

	private:
	dPluginScene* m_scene;
	dSceneRender* m_render;
//	dSceneExplorer* m_sceneExplorer;

	dPluginCamera* m_currentCamera;
	dPluginDll m_allPlugins;
	const char* m_filePathFile;

	dTree<int, void*> m_selection;
//	ExplorerDictionary m_explorerDictionary;
	
	dTree<dPluginRecord*, dCRCTYPE> m_pluginDictionary;
	static int m_totalMemoryUsed;
};


#endif