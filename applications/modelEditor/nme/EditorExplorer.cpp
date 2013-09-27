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

#include "toolbox_stdafx.h"
#include "EditorExplorer.h"
#include "NewtonModelEditor.h"
//#include "EditorAssetBrowser.h"
//#include "EditorAssetExplorer.h"


EditorExplorer::EditorExplorer(NewtonModelEditor* const mainFrame)
	:wxTreeCtrl (mainFrame)
	,m_mainFrame(mainFrame)
{
/*
	FXVerticalFrame* const contents = new FXVerticalFrame(this, LAYOUT_SIDE_LEFT|FRAME_NONE|LAYOUT_FILL, 0,0,0,0, 0,0,0,0);
	m_tabBook = new FXTabBook(contents, mainFrame, NewtonModelEditor::ID_EDITOR_MODE, TABBOOK_BOTTOMTABS|FRAME_NONE|LAYOUT_FILL, 0,0,0,0, 0,0,0,0);

	// add asset database viewer
	new FXTabItem(m_tabBook, D_EDIT_MODE_ASSET, NULL, FRAME_NONE|LAYOUT_FILL);
	FXVerticalFrame* const assetContainer = new FXVerticalFrame(m_tabBook, LAYOUT_SIDE_LEFT|FRAME_NONE|LAYOUT_FILL, 0,0,0,0, 0,0,0,0);
	new FXLabel(assetContainer, "Assets database:");	
	m_assetBrowser = new EditorAssetBrowser(assetContainer, mainFrame);
	// add the current asset
	new FXLabel(assetContainer, "Current Asset:");	
	m_assetExplorer = new EditorAssetExplorer(assetContainer, mainFrame);

		
	// add a scene explorer viewer
	FXTabItem* const sceneExplorerTab = new FXTabItem(m_tabBook, D_EDIT_MODE_SCENE, NULL, FRAME_NONE|LAYOUT_FILL);
	sceneExplorerTab;
	int ID_OPEN_TREE = 1000;
	m_sceneExplorer = new FXTreeList(m_tabBook, mainFrame, ID_OPEN_TREE, TREELIST_BROWSESELECT|TREELIST_SHOWS_LINES|TREELIST_SHOWS_BOXES|LAYOUT_FILL_X|LAYOUT_FILL_Y);
*/
}

EditorExplorer::~EditorExplorer(void)
{
}

/*
void EditorExplorer::ReleaseAllAssets ()
{
	m_assetBrowser->ReleaseAllAssets ();
	m_mainFrame->RemoveAllAsset();
}




*/

/*
void EditorExplorer::PopulateCurrentAsset ()
{
	dPluginInterface::dAssetList::dListNode* const assetPluginNode = m_mainFrame->GetCurrentAssetNode();
	if (assetPluginNode) {
		dPluginScene* const asset = m_mainFrame->GetAssetFromNode(assetPluginNode);
		m_mainFrame->SetCurrentAssetNode(assetPluginNode);
		m_assetExplorer->Populate (asset);
	} else {
		m_assetExplorer->Populate (NULL);
	}
}

void EditorExplorer::SetBrowserSelection ()
{
	dPluginInterface::dAssetList::dListNode* const node = m_assetBrowser->GetCurrentAssetPluginNode();
	if (node != m_mainFrame->GetCurrentAssetNode()) {
		m_mainFrame->Push (new dUndoAssetCache(m_mainFrame));
		m_mainFrame->SetCurrentAssetNode(node);
		PopulateCurrentAsset ();
	}
}

dPluginScene* EditorExplorer::GetCurrentAsset() const
{
	dPluginInterface::dAssetList::dListNode* const assetPluginNode = m_assetBrowser->GetCurrentAssetPluginNode();
	if (assetPluginNode) {
		return m_mainFrame->GetAssetFromNode(assetPluginNode);
	} else {
		return NULL;
	}
}

void EditorExplorer::AddAsset (dPluginScene* const asset, dPluginMesh* const plugin)
{
	m_mainFrame->Push(new dUndoAssetCache(m_mainFrame));
	
	// add this asset to the dPluginInterface class
	dPluginInterface::dAssetList::dListNode* const assetPluginNode = m_mainFrame->AddAsset(asset, plugin);

	// update all bounding boxes
	asset->UpdateAllOOBB();

	// add this asset to the assetDatabase browser
	m_assetBrowser->AddAssetAndPopulate (assetPluginNode);
	m_assetExplorer->Populate (m_mainFrame->GetAssetFromNode (assetPluginNode));
}


void EditorExplorer::RefreshAllViewer ()
{
	m_assetBrowser->Clear();
	int index = 0;
	int currentIndex = 0;
	dPluginInterface::dAssetList::dListNode* const currentAsset = m_mainFrame->GetCurrentAssetNode();	
	for (dPluginInterface::dAssetList::dListNode* assetNode = m_mainFrame->GetFirstAssetNode(); assetNode; assetNode = m_mainFrame->GetNextAssetNode(assetNode)) {	
		m_assetBrowser->AddAsset (assetNode);

		if (assetNode == currentAsset) {
			currentIndex = index;
		}
		index ++;
	}

	// populate the asset browser
	PopulateCurrentAsset();
	m_assetBrowser->setCurrentItem(-1, FALSE);
	if (currentAsset) {
		m_assetBrowser->setCurrentItem(currentIndex, FALSE);
	}

	// populate the current asset explorer
	m_assetExplorer->Populate (GetCurrentAsset());
}


void EditorExplorer::HandleSelectionEvent (const dList<dScene::dTreeNode*>& traceToRoot) const
{
	m_assetExplorer->HandleSelectionEvent (traceToRoot);
}
*/

/*
void EditorExplorer::PopulateModel(const dPluginScene* const scene, wxTreeItemId modelItem)
{
	dScene::dTreeNode* const modelNode = (dScene::dTreeNode*) modelItem->getData();
	for (void* link = scene->GetFirstChild(modelNode); link; link = scene->GetNextChild(modelNode, link)) {
		dScene::dTreeNode* const node = scene->GetNodeFromLink (link);
		dNodeInfo* const info = scene->GetInfoFromNode(node);

//		FXTreeItem* const childItem = (FXTreeItem*) m_sceneExplorer->appendItem(modelItem, info->GetName(), NULL, NULL, NULL, TRUE);
		wxTreeItemId childItem = AppendItem(modelItem, wxT(info->GetName())));
//		childItem->setData (node);
		PopulateModel(scene, childItem);
	}
}
*/

/*
void EditorExplorer::Populate (const dPluginScene* const scene, wxTreeItemId rootItem)
{

	ExplorerData* const nodeData = ((ExplorerData*)GetItemData(rootItem));
	dScene::dTreeNode* const rootNode = nodeData->m_node;

	// add all models
	//	for (dScene::dTreeNode* node = scene->GetFirstNode(); node; node = scene->GetNextNode(node)) {
	for (void* link = scene->GetFirstChild(rootNode); link; link = scene->GetNextChild(rootNode, link)) {
		dScene::dTreeNode* const node = scene->GetNodeFromLink (link);
		dNodeInfo* const info = scene->GetInfoFromNode(node);
//dCRCTYPE xxx = info->GetTypeId();
//dNodeInfo::GetRttiType();
//dSceneModelInfo::GetRttiType();
//		if (info->IsType(dSceneModelInfo::GetRttiType())) {
			//FXTreeItem* const modelItem = (FXTreeItem*) m_sceneExplorer->appendItem(rootItem , info->GetName(), NULL, NULL, NULL, TRUE);
			wxTreeItemId modelItem = AppendItem(rootItem, wxT(info->GetName()), -1, -1, new ExplorerData(node));
			//modelItem->setData (node);
//			PopulateModel(scene, modelItem);
//		}
	}
	m_sceneExplorer->expandTree(rootItem, true);
}
*/


void EditorExplorer::Clear()
{
	DeleteAllItems();
}


void EditorExplorer::ReconstructScene(const dPluginScene* const scene)
{
	if (GetRootItem() == NULL) {
		dScene::dTreeNode* const rootNode = scene->GetRootNode();
		dNodeInfo* const rootInfo = scene->GetInfoFromNode(rootNode);
		AddRoot(wxT (rootInfo->GetName()) , -1, -1, new ExplorerData(rootNode));
	}
		
	dList<wxTreeItemId> stack;
	stack.Append(GetRootItem());
	while (stack.GetCount()) {
		wxTreeItemId rootItem (stack.GetLast()->GetInfo());
		stack.Remove(stack.GetLast());

		ExplorerData* const nodeData = ((ExplorerData*)GetItemData(rootItem));
		dScene::dTreeNode* const rootNode = nodeData->m_node;

		for (void* link = scene->GetFirstChild(rootNode); link; link = scene->GetNextChild(rootNode, link)) {
			dScene::dTreeNode* const childNode = scene->GetNodeFromLink (link);
			bool found = false;
			for (wxTreeItemId childItem = GetLastChild(rootItem); childItem; childItem = GetPrevSibling(childItem)) {
				ExplorerData* const data = ((ExplorerData*)GetItemData(childItem));
				dScene::dTreeNode* const node = data->m_node;
				if (node == childNode) {
					found = true;
					break;
				}
			}
			
			if (!found) {
				dNodeInfo* const info = scene->GetInfoFromNode(childNode);
				wxTreeItemId item = AppendItem(rootItem, wxT(info->GetName()), -1, -1, new ExplorerData(childNode));

dClassInfo::GetRttiType();
dSceneNodeInfo::GetRttiType();

//				dSceneNodeInfo* model = (dSceneNodeInfo*)info;
//				model->GetTypeId();
//				model->dSceneNodeInfo::GetTypeId();
//				dSceneNodeInfo::GetRttiType();
//				dSceneNodeInfo::BaseClassName();

			}
		}
		
	
		for (wxTreeItemId childItem = GetLastChild(rootItem); childItem; childItem = GetPrevSibling(childItem)) {
			stack.Append(childItem);
		}
	}
}