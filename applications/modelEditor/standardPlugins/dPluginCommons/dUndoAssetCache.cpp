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

#include "dPluginStdafx.h"
#include "dUndoAssetCache.h"
/*
dUndoAssetCache::dUndoAssetCache (dPluginInterface* const interface)
	:dUndoCurrentAsset(interface)
	,m_backupAssets()
	,m_currentScene(NULL)
{
	_ASSERTE (m_interface == interface);
	
	dPluginInterface::dAssetList::dListNode* const currentAsset = interface->GetCurrentAssetNode();

	for (dPluginInterface::dAssetList::dListNode* assetNode = interface->GetFirstAssetNode(); assetNode; assetNode = interface->GetNextAssetNode(assetNode)) {

		const dPluginInterface::AssetPluginAssociation& assetPlugin = assetNode->GetInfo();
		dPluginInterface::AssetPluginAssociation& backup = m_backupAssets.Append()->GetInfo(); 

		if (assetNode == currentAsset) {
			m_currentScene = m_backupAssets.GetLast();
		}

		dPluginScene* const scene = assetPlugin.m_asset;
		backup.m_asset = new dPluginScene(scene->GetNewtonWorld());
		backup.m_plugin = assetPlugin.m_plugin;

		dScene::dTreeNode* const rootNode = scene->GetRootNode();
		backup.m_asset->DeleteRootNode ();
		backup.m_asset->AddRootNode(scene->GetInfoFromNode(rootNode));

		for (dScene::dTreeNode* node = scene->GetFirstNode(); node; node = scene->GetNextNode(node)) {
			if (node != rootNode) {
				backup.m_asset->AddNode(scene->GetInfoFromNode(node), NULL);
			}
		}

		for (dScene::dTreeNode* node = scene->GetFirstNode(); node; node = scene->GetNextNode(node)) {	
			dScene::dTreeNode* const bkChildNode = (dScene::dTreeNode*) backup.m_asset->Find(scene->GetInfoFromNode(node)->GetUniqueID());
			for (void* parent = scene->GetFirstParent(node); parent; parent = scene->GetNextParent(node, parent)) {	
				dScene::dTreeNode* const parentNode = scene->GetNodeFromLink (parent);
				dScene::dTreeNode* const bkParentNode = (dScene::dTreeNode*) backup.m_asset->Find(scene->GetInfoFromNode(parentNode)->GetUniqueID());
				backup.m_asset->AddReference (bkParentNode, bkChildNode);
			}
		}
	}
}

dUndoAssetCache::~dUndoAssetCache()
{
}

void dUndoAssetCache::RestoreState()
{
	m_interface->RemoveAllAsset();

	int index = 0;
	int currentIndex = 0;
//DTRACE (("restore: "));
	dPluginInterface::dAssetList::dListNode* currentAsset = NULL;
	for (dList<dPluginInterface::AssetPluginAssociation>::dListNode* undoNode = m_backupAssets.GetFirst(); undoNode; undoNode = undoNode->GetNext()) {
		const dPluginInterface::AssetPluginAssociation& asset = undoNode->GetInfo();
		dPluginInterface::dAssetList::dListNode* const lastAssetNode = m_interface->AddAsset(asset.m_asset, asset.m_plugin);

		dNodeInfo* const info = asset.m_asset->GetInfoFromNode(asset.m_asset->GetRootNode());
//DTRACE (("%s ", info->GetName()));
		if (undoNode == m_currentScene) {
			currentIndex = index;
			currentAsset = lastAssetNode;
		}
		index ++;
	}
	m_interface->SetCurrentAssetNode(currentAsset);
//DTRACE (("\n"));

	dUndoCurrentAsset::RestoreState();
}

dUndoRedo* dUndoAssetCache::CreateRedoState() const
{
	return new dUndoAssetCache (m_interface);
}
*/




