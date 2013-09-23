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


//	class dAssetList: public dList <AssetPluginAssociation>
//	{
//	};

	DPLUGIN_API dPluginInterface(void);
	virtual DPLUGIN_API  ~dPluginInterface(void);
	
	virtual DPLUGIN_API dPluginScene* GetScene() const;
	virtual DPLUGIN_API void SetScene(dPluginScene* const scene);

//	virtual dPluginScene* GetAsset() const;
	virtual DPLUGIN_API dSceneRender* GetRender() const;

	virtual DPLUGIN_API dPluginCamera* GetCamera() const;
	virtual DPLUGIN_API dPluginRecord* GetPlugin(const char* const signature) const;

	virtual DPLUGIN_API void* GetFirstPluginNode() const;
	virtual DPLUGIN_API void* GetNextPluginNode(void* const pluginNode) const;
	virtual DPLUGIN_API dPluginRecord* GetPluginFromNode(void* const pluginNode) const;

//	virtual dAssetList::dListNode* AddAsset(dPluginScene* const scene, dPluginMesh* plugin);
//	virtual void RemoveAsset(dAssetList::dListNode* const node);
//	virtual void RemoveAllAsset();
//	virtual dAssetList::dListNode* GetCurrentAssetNode() const;
//	virtual void SetCurrentAssetNode(dAssetList::dListNode* const node);
//	virtual dAssetList::dListNode* GetFirstAssetNode() const;
//	virtual dAssetList::dListNode* GetNextAssetNode(dAssetList::dListNode* const node) const;
//	virtual dPluginScene* GetAssetFromNode(dAssetList::dListNode* const node) const;
//	virtual dPluginMesh* GetAssetPluginFromNode(dAssetList::dListNode* const node) const;

	virtual DPLUGIN_API const char* GetFilePath() const;

	virtual DPLUGIN_API void ClearSelection();
	virtual DPLUGIN_API bool IsNodeSelected(void* const incidentLink) const;
	virtual DPLUGIN_API void AddToSelection(void* const incidentLink);
	virtual DPLUGIN_API void RemoveFromSelection(void* const incidentLink);
	virtual DPLUGIN_API void* GetFirtSelectedNode() const;
	virtual DPLUGIN_API void* GetNextSelectedNode(void* const incidentLink) const;


	virtual DPLUGIN_API void ClearExplorerExpand();
	virtual DPLUGIN_API void AddExplorerExpandNode(void* const incidentLink, bool state);
	virtual DPLUGIN_API void SetExplorerExpandNodeState(void* const incidentLink, bool state);
	virtual DPLUGIN_API bool GetExplorerExpandNodeState(void* const incidentLink) const;


	protected:
	DPLUGIN_API void LoadPlugins(const char* const path, dPluginDll& plugins);

	private:
	dPluginScene* m_scene;
	dSceneRender* m_render;
	dPluginCamera* m_currentCamera;
	dPluginDll m_allPlugins;
	const char* m_filePathFile;
//	dAssetList m_assetCache;
//	dAssetList::dListNode* m_currentAsset;

	dTree<int, void*> m_selection;
	dTree<int, void*> m_ExplorerExpand;
	dTree<dPluginRecord*, dCRCTYPE> m_pluginDictionary;
	static int m_totalMemoryUsed;
};


#endif