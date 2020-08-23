/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "toolbox_stdafx.h"
#include "EditorExplorer.h"
#include "NewtonModelEditor.h"
#include "EditorAssetBrowser.h"
#include "EditorAssetExplorer.h"


/*
FXDEFMAP(EditorAssetBrowser) MessageMap[]=
{
	FXMAPFUNC(SEL_LEFTBUTTONPRESS,		0,  EditorAssetBrowser::onMouseBottonEvent),
	FXMAPFUNC(SEL_RIGHTBUTTONPRESS,		0,	EditorAssetBrowser::onMouseBottonEvent),
	FXMAPFUNC(SEL_DOUBLECLICKED,		0,	EditorAssetBrowser::onMouseBottonEvent),
};
FXIMPLEMENT(EditorAssetBrowser, FXComboBox, MessageMap,ARRAYNUMBER(MessageMap))
*/

EditorAssetBrowser::EditorAssetBrowser()
	:FXComboBox()
{
}

EditorAssetBrowser::EditorAssetBrowser (FXComposite* const parent, NewtonModelEditor* const mainFrame)
	:FXComboBox(parent, 0, mainFrame, NewtonModelEditor::ID_SELECT_ASSET, COMBOBOX_STATIC|LAYOUT_FILL_X| FRAME_SUNKEN|FRAME_THICK|LAYOUT_SIDE_TOP)
	,m_mainFrame(mainFrame)
{
	setNumVisible(10);
}

EditorAssetBrowser::~EditorAssetBrowser(void)
{
}


void EditorAssetBrowser::Clear ()
{
	for (int i = getNumItems()-1; i >= 0; i --) {
		removeItem(i);
	}
}

void EditorAssetBrowser::AddAsset (dPluginInterface::dAssetList::dListNode* const assetPluginNode)
{
	// create the a visual state for this scene
	dPluginScene* const asset = m_mainFrame->GetAssetFromNode(assetPluginNode);
	m_assetList.Insert(asset);

	dNodeInfo* const info = asset->GetInfoFromNode(asset->GetRootNode());
	appendItem(info->GetName(), assetPluginNode);
}


void EditorAssetBrowser::AddAssetAndPopulate (dPluginInterface::dAssetList::dListNode* const assetPluginNode)
{
	// create the a visual state for this scene
	AddAsset (assetPluginNode);

	dPluginScene* const asset = m_mainFrame->GetAssetFromNode(assetPluginNode);
	AssetList::dTreeNode* const assetNode = m_assetList.Find(asset);

	m_mainFrame->ClearExplorerExpand();
	InitExploreExpandedState(assetNode, asset->GetRootNode());
	if (getNumItems() == 1) {
		setCurrentItem (-1, FALSE);
	}
	setCurrentItem (getNumItems() - 1, FALSE);
}

void EditorAssetBrowser::InitExploreExpandedState(AssetList::dTreeNode* const assetNode, dScene::dTreeNode* const parentNode)
{
	dPluginScene* const asset = assetNode->GetKey(); 
	for (void* link = asset->GetFirstChild(parentNode); link; link = asset->GetNextChild(parentNode, link)) {
		dScene::dTreeNode* const node = asset->GetNodeFromLink (link);

		dNodeInfo* const info = asset->GetInfoFromNode(node);
		if (info->IsType(dSceneNodeInfo::GetRttiType())) {
			bool hasSceneNodes = 0;
			for (void* childLink = asset->GetFirstChild(node); childLink; childLink = asset->GetNextChild(parentNode, childLink)) {
				dScene::dTreeNode* const childSceneNode = asset->GetNodeFromLink (childLink);
				dNodeInfo* const scenNodeInfo = asset->GetInfoFromNode(childSceneNode);
				hasSceneNodes |= scenNodeInfo->IsType(dSceneNodeInfo::GetRttiType()) ? 1 : 0;
			}
			m_mainFrame->AddExplorerExpandNode (link, hasSceneNodes);

		} else {
			m_mainFrame->AddExplorerExpandNode (link, false);
		}
		InitExploreExpandedState(assetNode, node);
	}
}

dPluginInterface::dAssetList::dListNode* EditorAssetBrowser::GetCurrentAssetPluginNode() const
{
	return (getCurrentItem() >= 0) ? (dPluginInterface::dAssetList::dListNode*) getItemData(getCurrentItem()) : NULL;
}

