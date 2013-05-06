/////////////////////////////////////////////////////////////////////////////
// Name:        dScene.cpp
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

#include "dPluginStdafx.h"
#include "dPluginMesh.h"
#include "dPluginInterface.h"
#include "dUndoCurrentAsset.h"


dUndoCurrentAsset::dUndoCurrentAsset(dPluginInterface* const interface) 
	:dUndoRedo()
	,nodeIndex (0)
	,m_interface(interface)
	,m_backup()
{
	dPluginInterface::dAssetList::dListNode* const currentAssetNode = interface->GetCurrentAssetNode();
	if (currentAssetNode) {
		for (dPluginInterface::dAssetList::dListNode* node = interface->GetFirstAssetNode(); node; node = interface->GetNextAssetNode(node)) {
			if (node == currentAssetNode) {
				break;
			}
			nodeIndex ++;
		}

		dPluginInterface::AssetPluginAssociation& assetPlugin = currentAssetNode->GetInfo();
		dPluginScene* const scene = assetPlugin.m_asset;

		m_backup.m_asset = new dPluginScene(scene->GetNewtonWorld());
		m_backup.m_plugin = assetPlugin.m_plugin;

		dTree<int, void*> filter;
		for (void* link = interface->GetFirtSelectedNode(); link; link = interface->GetNextSelectedNode (link)) {
			filter.Insert(link);
		}

		dScene::dTreeNode* const rootNode = scene->GetRootNode();
		m_backup.m_asset->DeleteRootNode ();
		m_backup.m_asset->AddRootNode(scene->GetInfoFromNode(rootNode));
		for (dScene::dTreeNode* node = scene->GetFirstNode(); node; node = scene->GetNextNode(node)) {
			if (node != rootNode) {
				m_backup.m_asset->AddNode(scene->GetInfoFromNode(node), NULL);
			}
		}



		for (dScene::dTreeNode* sceneParentNode = scene->GetFirstNode(); sceneParentNode; sceneParentNode = scene->GetNextNode(sceneParentNode)) {	
			dScene::dTreeNode* const backupParentNode = (dScene::dTreeNode*) m_backup.m_asset->Find(scene->GetInfoFromNode(sceneParentNode)->GetUniqueID());
			for (void* link = scene->GetFirstChild(sceneParentNode); link; link = scene->GetNextParent(sceneParentNode, link)) {	
				dScene::dTreeNode* const sceneChildNode = scene->GetNodeFromLink (link);
				dScene::dTreeNode* const backupChildNode = (dScene::dTreeNode*) m_backup.m_asset->Find(scene->GetInfoFromNode(sceneChildNode)->GetUniqueID());
				m_backup.m_asset->AddReference (backupParentNode, backupChildNode);


				void* childLink = NULL;
				for (void* bklink = m_backup.m_asset->GetFirstChild(backupParentNode); bklink; bklink = m_backup.m_asset->GetNextParent(backupParentNode, bklink)) {
					if (m_backup.m_asset->GetNodeFromLink(bklink) == backupChildNode) {
						childLink = bklink;
						break;
					}
				}

				if (filter.Find(link)) {
					m_selection.Insert(0, childLink);
				}
				m_exploreStatus.Insert (interface->GetExplorerExpandNodeState(link) ? 1 : 0, childLink);
			}
		}
	}
}


dUndoCurrentAsset::~dUndoCurrentAsset() 
{
}


dUndoRedo* dUndoCurrentAsset::CreateRedoState() const
{
	return new dUndoCurrentAsset (m_interface);
}

void dUndoCurrentAsset::RestoreState()
{
	if (m_backup.m_asset) {
		dScene::dTreeNode* const rootNode = m_backup.m_asset->GetRootNode();

		dPluginInterface::dAssetList::dListNode* currentAssetNode = m_interface->GetFirstAssetNode();
		for (int i = 0; i < nodeIndex; i ++) {
			currentAssetNode = m_interface->GetNextAssetNode(currentAssetNode);
		}
		_ASSERTE (currentAssetNode);

		dPluginInterface::AssetPluginAssociation& assetPlugin = currentAssetNode->GetInfo();
		assetPlugin.m_asset->Release();

		assetPlugin.m_asset = m_backup.m_asset;
		assetPlugin.m_plugin = m_backup.m_plugin;
		m_backup.m_asset->AddRef();

		m_interface->ClearSelection();
		dTree<int, void*>::Iterator selectIter (m_selection);
		for (selectIter.Begin(); selectIter; selectIter ++) {
			void* const link = selectIter.GetNode()->GetKey();
			m_interface->AddToSelection (link);
		}

		m_interface->ClearExplorerExpand();
		dTree<int, void*>::Iterator explorerIter (m_exploreStatus);
		for (explorerIter.Begin(); explorerIter; explorerIter ++) {
			dTree<int, void*>::dTreeNode* const expandNode = explorerIter.GetNode();
			void* const link = expandNode->GetKey();
			m_interface->AddExplorerExpandNode (link, expandNode->GetInfo() ? true : false);
		}
	}
}
