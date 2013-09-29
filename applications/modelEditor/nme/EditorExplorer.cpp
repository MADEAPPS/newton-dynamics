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

BEGIN_EVENT_TABLE (EditorExplorer, wxTreeCtrl)


	EVT_TREE_BEGIN_LABEL_EDIT (NewtonModelEditor::ID_EDIT_NODE_NAME, OnBeginEdit)
	EVT_TREE_END_LABEL_EDIT (NewtonModelEditor::ID_EDIT_NODE_NAME, OnEndEdit)

END_EVENT_TABLE()


class EditorExplorer::ExplorerData: public wxTreeItemData
{
	public:
	ExplorerData (dScene::dTreeNode* const node)
		:wxTreeItemData()
		,m_node(node)
	{
	}
	~ExplorerData ()
	{
	}

	dScene::dTreeNode* m_node;
};



class EditorExplorer::TraverseExplorer
{
	public:
	TraverseExplorer (){}
	virtual bool TraverseCallback (wxTreeItemId rootItem) const = 0;
	void TraverseExplorer::Traverse(const EditorExplorer* const me) const
	{
		dList<wxTreeItemId> itemList;

		itemList.Append(me->GetRootItem()); 
		while (itemList.GetCount()) {
			wxTreeItemId item = itemList.GetLast()->GetInfo();
			itemList.Remove(itemList.GetLast());

			bool state = TraverseCallback (item);
			if (!state) {
				break;
			}

			for (wxTreeItemId childItem = me->GetLastChild(item); childItem; childItem = me->GetPrevSibling(childItem)) {
				itemList.Append(childItem); 
			}
		}
	}
};

class EditorExplorer::ChangeNames: public TraverseExplorer
{
	public:
	ChangeNames(EditorExplorer* const me, dNodeInfo* const nodeInfo)
		:m_me(me) 
		,m_nodeInfo(nodeInfo)
	{
		Traverse(me);
	}

	bool TraverseCallback (wxTreeItemId item) const
	{
		dPluginScene* const scene = m_me->m_mainFrame->GetScene(); 		

		ExplorerData* const data = (ExplorerData*) m_me->GetItemData(item);
		if (m_nodeInfo == scene->GetInfoFromNode(data->m_node)) {
			m_me->SetItemText(item, wxString (m_nodeInfo->GetName()));
		}
		return true;
	}

	EditorExplorer* m_me;
	dNodeInfo* m_nodeInfo;
};


class EditorExplorer::UndoRedoChangeName: public dUndoRedo
{
	public:
	UndoRedoChangeName(EditorExplorer* const me, const wxString& name, dNodeInfo* const nodeInfo)
		:dUndoRedo()
		,m_me(me) 
		,m_nodeInfo(nodeInfo)
		,m_name(name) 
	{
	}

	~UndoRedoChangeName()
	{
	}

	protected:
	virtual void RestoreState(dUndodeRedoMode mode)
	{
		m_nodeInfo->SetName(m_name.c_str());
		ChangeNames chamgeNames(m_me, m_nodeInfo);
	}

	virtual dUndoRedo* CreateRedoState() const
	{
		return new UndoRedoChangeName (m_me, wxString(m_nodeInfo->GetName()), m_nodeInfo);
	}

	wxString m_name;
	EditorExplorer* m_me;
	dNodeInfo* m_nodeInfo;
};



EditorExplorer::EditorExplorer(NewtonModelEditor* const mainFrame)
	:wxTreeCtrl (mainFrame, NewtonModelEditor::ID_EDIT_NODE_NAME, wxDefaultPosition, wxSize (200, 160), wxTR_EDIT_LABELS | wxTR_MULTIPLE | wxTR_EXTENDED | wxTR_HAS_BUTTONS | wxTR_LINES_AT_ROOT)
	,m_mainFrame(mainFrame)
{

	wxBitmap* const bitmap = m_mainFrame->FindIcon ("explorer.gif");
	int w = bitmap->GetWidth();
	int h = bitmap->GetHeight();

	wxImageList* const imageList = new wxImageList (w, h, true, 16);

	imageList->Add (*m_mainFrame->FindIcon ("explorer.gif"));
	imageList->Add (*m_mainFrame->FindIcon ("cache.gif"));
	imageList->Add (*m_mainFrame->FindIcon ("sceneNode.gif"));
	imageList->Add (*m_mainFrame->FindIcon ("texture.gif"));
	imageList->Add (*m_mainFrame->FindIcon ("material.gif"));
	imageList->Add (*m_mainFrame->FindIcon ("geometry.gif"));

	AssignImageList(imageList);
	
}

EditorExplorer::~EditorExplorer(void)
{
}


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
	for (void* link = scene->GetFirstChildLink(modelNode); link; link = scene->GetNextChild(modelNode, link)) {
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
	for (void* link = scene->GetFirstChildLink(rootNode); link; link = scene->GetNextChild(rootNode, link)) {
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
		AddRoot(wxT (rootInfo->GetName()) , 0, -1, new ExplorerData(rootNode));
	}
		
	dList<wxTreeItemId> stack;
	stack.Append(GetRootItem());
	while (stack.GetCount()) {
		wxTreeItemId rootItem (stack.GetLast()->GetInfo());
		stack.Remove(stack.GetLast());

		ExplorerData* const nodeData = ((ExplorerData*)GetItemData(rootItem));
		dScene::dTreeNode* const rootNode = nodeData->m_node;

		for (void* link = scene->GetFirstChildLink(rootNode); link; link = scene->GetNextChildLink(rootNode, link)) {
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

				if (info->IsType(dSceneCacheInfo::GetRttiType())) {
					PrependItem(rootItem, wxT(info->GetName()), 1, -1, new ExplorerData(childNode));
				} else {
					int imageId = -1;
					if (info->IsType(dSceneNodeInfo::GetRttiType())) {
						imageId = 2;
					} else if (info->IsType(dTextureNodeInfo::GetRttiType())) {
						imageId = 3;
					} else if (info->IsType(dMaterialNodeInfo::GetRttiType())) {
						imageId = 4;
					} else if (info->IsType(dGeometryNodeInfo::GetRttiType())) {
						imageId = 5;
					}

					if (info->IsType(dGeometryNodeInfo::GetRttiType())) {
						PrependItem(rootItem, wxT(info->GetName()), imageId, -1, new ExplorerData(childNode));
					} else {
						AppendItem(rootItem, wxT(info->GetName()), imageId, -1, new ExplorerData(childNode));
					}
				}
			}
		}
	
	
		for (wxTreeItemId childItem = GetLastChild(rootItem); childItem; childItem = GetPrevSibling(childItem)) {
			stack.Append(childItem);
		}
	}

//	virtual void Expand(const wxTreeItemId& item) = 0;
}

void EditorExplorer::OnBeginEdit (wxTreeEvent& event)
{
}

void EditorExplorer::OnEndEdit (wxTreeEvent& event)
{
	wxString name (event.GetLabel());
	if (!name.IsEmpty()) {

		wxTreeItemId item (event.GetItem());
		ExplorerData* const data = (ExplorerData*) GetItemData(item);
		dScene::dTreeNode* const node = data->m_node;
		dPluginScene* const scene = m_mainFrame->GetScene(); 
		dNodeInfo* const info = scene->GetInfoFromNode(node);

		m_mainFrame->Push (new UndoRedoChangeName(this, wxString(info->GetName()), info));

		info->SetName(name.c_str());
		ChangeNames changeName (this, info);
	}
}

