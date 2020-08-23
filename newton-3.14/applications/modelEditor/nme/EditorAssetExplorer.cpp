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

FXDEFMAP(EditorAssetExplorer) MessageMap[]=
{
	FXMAPFUNC(SEL_LEFTBUTTONPRESS,		0,  EditorAssetExplorer::onMouseBottonEvent),
	FXMAPFUNC(SEL_RIGHTBUTTONPRESS,		0,	EditorAssetExplorer::onMouseBottonEvent),
	FXMAPFUNC(SEL_DOUBLECLICKED,		0,	EditorAssetExplorer::onMouseBottonEvent),
};
FXIMPLEMENT(EditorAssetExplorer, FXTreeList, MessageMap,ARRAYNUMBER(MessageMap))


class EditorAssetExplorer::ChangeInfoName: public dUndoRedo
{
	public:
	ChangeInfoName(dNodeInfo* const info, EditorAssetExplorer* const manager)
		:dUndoRedo()
		,m_nodeInfo(info)
		,m_manager(manager)
	{
		strcpy (m_name, info->GetName());
	}

	virtual ~ChangeInfoName()
	{
	}

	virtual void RestoreState()
	{
		m_nodeInfo->SetName(m_name);
		m_manager->Populate (m_manager->m_mainFrame->m_explorer->GetCurrentAsset());
	}

	virtual dUndoRedo* CreateRedoState() const
	{
		return new ChangeInfoName(m_nodeInfo, m_manager);
	}

	dNodeInfo* m_nodeInfo;
	EditorAssetExplorer* m_manager;
	char m_name[256];
};



EditorAssetExplorer::EditorAssetExplorer()
	:FXTreeList()
{
}

EditorAssetExplorer::EditorAssetExplorer(FXComposite* const parent, NewtonModelEditor* const mainFrame)
	:FXTreeList(parent, mainFrame, NewtonModelEditor::ID_ASSET_MANAGER, TREELIST_EXTENDEDSELECT|TREELIST_SHOWS_LINES|TREELIST_SHOWS_BOXES|LAYOUT_FILL_X|LAYOUT_FILL_Y)
	,m_mainFrame(mainFrame)
{
	m_rootItem = appendItem(NULL, "root", NULL, NULL, NULL, TRUE);
}

EditorAssetExplorer::~EditorAssetExplorer()
{
}


void EditorAssetExplorer::create()
{
	FXTreeList::create();
	m_rootIcon = m_mainFrame->FindIcon("explorer.gif");
	m_sceneIcon = m_mainFrame->FindIcon("sceneNode.gif");

	//m_meshIcon = m_mainFrame->FindIcon("meshNode.gif");
	m_meshIcon = m_mainFrame->FindIcon("object_move.gif");
	

	//m_textureIcon = m_mainFrame->FindIcon("imageNode.gif");
	m_textureIcon = m_mainFrame->FindIcon("object_turn.gif");

	m_geometryIcon = m_mainFrame->FindIcon("object_move.gif");
	
	m_materialIcon = m_mainFrame->FindIcon("object_scale.gif");
	m_textureCacheIcon = m_mainFrame->FindIcon("fileNew.gif");
	m_materialCacheIcon = m_mainFrame->FindIcon("fileNew.gif");
	m_geometryCacheIcon = m_mainFrame->FindIcon("fileNew.gif");
}


void EditorAssetExplorer::Populate (dPluginScene* const scene)
{
	clearItems(TRUE);

	if (scene) {
		dScene::dTreeNode* const rootNode = scene->GetRootNode();
		dNodeInfo* const rootInfo = scene->GetInfoFromNode(rootNode);

		// root item node store the pointer to the scene root node
		FXTreeItem* const rootItem = appendItem(NULL, rootInfo->GetName(), m_rootIcon, m_rootIcon, NULL, TRUE);
		rootItem->setData (rootNode);
		expandTree(rootItem, FALSE);
		
		PopulateTextureChache(scene, rootItem);
		PopulateMaterialChache(scene, rootItem);
		PopulateGeometryChache(scene, rootItem);

		// populate scene graph
		for (void* link = scene->GetFirstChild(rootNode); link; link = scene->GetNextChild(rootNode, link)) {
			dScene::dTreeNode* const node = scene->GetNodeFromLink (link);
			dNodeInfo* const info = scene->GetInfoFromNode(node);
			if (info->IsType(dSceneNodeInfo::GetRttiType())) {
				// all children items store the point to the link node
				FXTreeItem* const modelItem = (FXTreeItem*) appendItem(rootItem, info->GetName(), m_sceneIcon, m_sceneIcon, NULL, TRUE);
				modelItem->setData (link);
				PopulateSceneNode(scene, modelItem);
				ExpandItem(scene, modelItem);
			}
		}
	} else {
		m_rootItem = appendItem(NULL, "root", NULL, NULL, NULL, TRUE);
		expandTree(m_rootItem, FALSE);
	}
}

void EditorAssetExplorer::PopulateTextureChache(dPluginScene* const scene, FXTreeItem* const rootItem)
{
	dScene::dTreeNode* const rootNode = scene->GetRootNode();
	dScene::dTreeNode* const cacheNode = scene->GetTextureCacheNode ();
	dNodeInfo* const cacheInfo = scene->GetInfoFromNode(cacheNode);

	FXTreeItem* const textureCacheItem = (FXTreeItem*) appendItem(rootItem, cacheInfo->GetName(), m_textureCacheIcon, m_textureCacheIcon, NULL, TRUE);
	for (void* link = scene->GetFirstChild(rootNode); link; link = scene->GetNextChild(rootNode, link)) {
		if (scene->GetNodeFromLink(link) == cacheNode) {
			textureCacheItem->setData (link);
			break;
		}
	}

	for (void* link = scene->GetFirstChild(cacheNode); link; link = scene->GetNextChild(cacheNode, link)) {
		dScene::dTreeNode* const node = scene->GetNodeFromLink (link);
		dNodeInfo* const info = scene->GetInfoFromNode(node);
		FXTreeItem* const textureItem = (FXTreeItem*) appendItem(textureCacheItem, info->GetName(), m_textureIcon, m_textureIcon, NULL, TRUE);
		textureItem->setData(link);
	}
}

void EditorAssetExplorer::PopulateMaterialChache(dPluginScene* const scene, FXTreeItem* const rootItem)
{
	dScene::dTreeNode* const rootNode = scene->GetRootNode();
	dScene::dTreeNode* const cacheNode = scene->GetMaterialCacheNode ();
	dNodeInfo* const cacheInfo = scene->GetInfoFromNode(cacheNode);

	FXTreeItem* const materialCacheItem = (FXTreeItem*) appendItem(rootItem, cacheInfo->GetName(), m_materialCacheIcon, m_materialCacheIcon, NULL, TRUE);
	for (void* link = scene->GetFirstChild(rootNode); link; link = scene->GetNextChild(rootNode, link)) {
		if (scene->GetNodeFromLink(link) == cacheNode) {
			materialCacheItem->setData (link);
			break;
		}
	}

	for (void* link = scene->GetFirstChild(cacheNode); link; link = scene->GetNextChild(cacheNode, link)) {
		dScene::dTreeNode* const node = scene->GetNodeFromLink (link);
		dNodeInfo* const info = scene->GetInfoFromNode(node);
		FXTreeItem* const materialItem = (FXTreeItem*) appendItem(materialCacheItem, info->GetName(), m_materialIcon, m_materialIcon, NULL, TRUE);
		materialItem->setData(link);
	}
}

void EditorAssetExplorer::PopulateGeometryChache(dPluginScene* const scene, FXTreeItem* const rootItem)
{
	dScene::dTreeNode* const rootNode = scene->GetRootNode();
	dScene::dTreeNode* const cacheNode = scene->GetGeometryCacheNode ();
	dNodeInfo* const cacheInfo = scene->GetInfoFromNode(cacheNode);

	FXTreeItem* const geometryCacheItem = (FXTreeItem*) appendItem(rootItem, cacheInfo->GetName(), m_geometryCacheIcon, m_geometryCacheIcon, NULL, TRUE);
	for (void* link = scene->GetFirstChild(rootNode); link; link = scene->GetNextChild(rootNode, link)) {
		if (scene->GetNodeFromLink(link) == cacheNode) {
			geometryCacheItem->setData (link);
			break;
		}
	}

	for (void* link = scene->GetFirstChild(cacheNode); link; link = scene->GetNextChild(cacheNode, link)) {
		dScene::dTreeNode* const node = scene->GetNodeFromLink (link);
		dNodeInfo* const info = scene->GetInfoFromNode(node);
		FXTreeItem* const geometryItem = (FXTreeItem*) appendItem(geometryCacheItem, info->GetName(), m_geometryIcon, m_geometryIcon, NULL, TRUE);
		geometryItem->setData(link);
	}

}


void EditorAssetExplorer::PopulateSceneNode(dPluginScene* const scene, FXTreeItem* const modelItem)
{
	// root item node store the pointer to the scene root node
	// all children items store the point to the link node
	dScene::dTreeNode* const modelNode = scene->GetNodeFromLink (modelItem->getData());
	for (void* link = scene->GetFirstChild(modelNode); link; link = scene->GetNextChild(modelNode, link)) {
		dScene::dTreeNode* const node = scene->GetNodeFromLink (link);
		dNodeInfo* const info = scene->GetInfoFromNode(node);

		FXIcon* icon = NULL;
		if (info->IsType(dSceneNodeInfo::GetRttiType())) {
			icon = m_sceneIcon;
		} else if (info->IsType(dGeometryNodeInfo::GetRttiType())) {
			icon = m_meshIcon;
		} else if (info->IsType(dMaterialNodeInfo::GetRttiType())) {
			icon = m_materialIcon;
		} else if (info->IsType(dTextureNodeInfo::GetRttiType())) {
			icon = m_textureIcon;
		}
		FXTreeItem* const childItem = (FXTreeItem*) appendItem(modelItem, info->GetName(), icon, icon, NULL, TRUE);
		// all children items store the point to the link node
		childItem->setData (link);
		PopulateSceneNode(scene, childItem);
	}
}


void EditorAssetExplorer::ChangeItemName (FXTreeItem* const item)
{
	dPluginScene* const scene = m_mainFrame->m_explorer->GetCurrentAsset();
	if (scene) {
		void* const link = item->getData();

		if (link == scene->GetRootNode()) {
			_ASSERTE (0);

		} else {

			dScene::dTreeNode* const itemNode = scene->GetNodeFromLink (link);
			dNodeInfo* const info = scene->GetInfoFromNode(itemNode);

			FXInputDialog input (m_mainFrame, "enter new name", "");
			input.setText(info->GetName());
			if (input.execute(PLACEMENT_OWNER)) {
				FXString name (input.getText());
				if(stricmp (name.text(), info->GetName())) {
					m_mainFrame->Push (new ChangeInfoName(info, this));

					info->SetName(name.text());

					dList<FXTreeItem*> itemList;
					for (FXTreeItem* item = m_rootItem->getFirst(); item; item = item->getNext()) {
						itemList.Append(item);
					}

					while (itemList.GetCount()) {
						FXTreeItem* const item = itemList.GetLast()->GetInfo();
						itemList.Remove(itemList.GetLast());
						
						_ASSERTE (item->getData());
						dScene::dTreeNode* const node = scene->GetNodeFromLink (item->getData());
						if (node == itemNode) {
							item->setText(name.text());
						}

						for (FXTreeItem* child = item->getFirst(); child; child = child->getNext()) {
							itemList.Append(child);
						}
					}
				}
			}
		}
		recalc();
	}
}


void EditorAssetExplorer::ExpandItem(dPluginScene* const scene, FXTreeItem* const modelItem)
{
	// all children items store the point to the link node

//	dScene::dTreeNode* const node = scene->GetNodeFromLink(modelItem->getData());
//	if (m_mainFrame->IsNodeSelected(node)) {

	void* const link = modelItem->getData();
	if (m_mainFrame->IsNodeSelected(link)) {
		selectItem(modelItem, FALSE);
	}

	if (m_mainFrame->GetExplorerExpandNodeState(link)) {
		expandTree(modelItem, FALSE);
		for (FXTreeItem* child = modelItem->getFirst(); child; child = child->getNext()) {
			ExpandItem(scene, child);
		}
	} else {
		collapseTree(modelItem, FALSE);
	}
}


long EditorAssetExplorer::onMouseBottonEvent (FXObject* sender, FXSelector id, void* eventPtr)
{
	int ret = 1;
	int messageType = FXSELTYPE(id);
	switch (messageType)
	{
		case SEL_LEFTBUTTONPRESS:
		{
			// handle expand and contract
			ret = FXTreeList::onLeftBtnPress(sender, id, eventPtr);

			FXEvent* const event=(FXEvent*)eventPtr;
			FXTreeItem* const item = getItemAt(event->win_x,event->win_y);
			int code = hitItem(item, event->win_x,event->win_y);
			if (code == 3) {
				// this is a expand / collapse click command
				HandleExpandEvent (item);
			} else {
				// this is a selection command
				HandleSelectionEvent (item);
			}
			break;
		}

		case SEL_RIGHTBUTTONPRESS:
		{
			// open menu options (base of node type)
			ret = FXTreeList::onRightBtnPress(sender, id, eventPtr);
			break;
		}

		case SEL_DOUBLECLICKED:
		{
			// change a node name
			ret = FXTreeList::onDoubleClicked (sender, id, eventPtr);

			ChangeItemName ((FXTreeItem*)eventPtr);
			break;
		}
	}
	return ret;
}


void EditorAssetExplorer::HandleExpandEvent (FXTreeItem* const item)
{
	dPluginScene* const scene = m_mainFrame->m_explorer->GetCurrentAsset();
	if (scene) {
		void* const link = item->getData();
		// root item node store the pointer to the scene root node
		// all children items store the point to the link node
		if (link != scene->GetRootNode()) {
			m_mainFrame->Push(new dUndoCurrentAsset(m_mainFrame));
			m_mainFrame->SetExplorerExpandNodeState(link, item->isExpanded() ? true : false);
		}
	}
}


void EditorAssetExplorer::HandleSelectionEvent (FXTreeItem* const item)
{
	dPluginScene* const scene = m_mainFrame->m_explorer->GetCurrentAsset();
	if (scene) {
		void* const link = item->getData();
		// root item node store the pointer to the scene root node
		// all children items store the point to the link node
		//if (link != scene->GetRootNode()) {
		//if (scene->GetNodeFromLink(link) != scene->GetRootNode()) {
		if (link) {
			//dScene::dTreeNode* const node = scene->GetNodeFromLink(link);
			if (m_mainFrame->IsControlDown()) {
				if (m_mainFrame->IsShiftDown())	{
					if (!m_mainFrame->IsNodeSelected(link)) {
						m_mainFrame->Push(new dUndoCurrentAsset(m_mainFrame));
						m_mainFrame->AddToSelection(link);
						m_mainFrame->m_explorer->RefreshAllViewer();
						m_mainFrame->RefrehViewports();
					}

				} else if (m_mainFrame->IsAltDown()) {
					if (m_mainFrame->IsNodeSelected(link)) {
						m_mainFrame->Push(new dUndoCurrentAsset(m_mainFrame));
						m_mainFrame->RemoveFromSelection(link);
						m_mainFrame->m_explorer->RefreshAllViewer();
						m_mainFrame->RefrehViewports();
					}

				} else {

					m_mainFrame->Push(new dUndoCurrentAsset(m_mainFrame));
					if (m_mainFrame->IsNodeSelected(link)) {
						m_mainFrame->RemoveFromSelection(link);
					} else {
						m_mainFrame->AddToSelection(link);
					}
					m_mainFrame->m_explorer->RefreshAllViewer();
					m_mainFrame->RefrehViewports();
				}
			} else {
				m_mainFrame->Push(new dUndoCurrentAsset(m_mainFrame));

				m_mainFrame->ClearSelection();
				m_mainFrame->AddToSelection(link);

				m_mainFrame->m_explorer->RefreshAllViewer();
				m_mainFrame->RefrehViewports();

			}

		}
	}
}

void EditorAssetExplorer::HandleSelectionEvent (const dList<dScene::dTreeNode*>& traceToRoot)
{

	dPluginScene* const scene = m_mainFrame->m_explorer->GetCurrentAsset();
	_ASSERTE (scene);

	int stack = 0;
	FXTreeItem* itemPool[1024*8];
	dList<dScene::dTreeNode*>::dListNode* traceNodePool[1024*8];
	
	dList<dScene::dTreeNode*>::dListNode* traceNode = traceToRoot.GetFirst();
	for (FXTreeItem* item = getFirstItem()->getFirst(); item; item = item->getNext()) {
		dScene::dTreeNode* const sceneNode = scene->GetNodeFromLink (item->getData());
		if (sceneNode == traceNode->GetInfo()) {
			itemPool[stack] = item;
			traceNodePool[stack] = traceNode;
			stack ++;
		}
	}
	
	while (stack) {
		stack --;
		FXTreeItem* const item = itemPool[stack];
		dList<dScene::dTreeNode*>::dListNode* const traceNode = traceNodePool[stack]->GetNext();
		if (traceNode) {
			for (FXTreeItem* childItem = item->getFirst(); childItem; childItem = childItem->getNext()) {
				dScene::dTreeNode* const sceneNode = scene->GetNodeFromLink (childItem->getData());
				if (sceneNode == traceNode->GetInfo()) {
					itemPool[stack] = childItem;
					traceNodePool[stack] = traceNode;
					stack ++;
				}
			}

		} else {
			HandleSelectionEvent (item);
			break;
		}
	}
}