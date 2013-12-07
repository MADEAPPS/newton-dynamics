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

#define EXPLORER_PANEL_DEFAULT_WITH		256
#define EXPLORER_PANEL_DEFAULT_HEIGHT	160


BEGIN_EVENT_TABLE (EditorExplorer, wxTreeCtrl)

	EVT_CHAR (OnKeyboardItem)
	EVT_TREE_ITEM_EXPANDED(NewtonModelEditor::ID_EDIT_NODE_NAME, OnExpandItem) 
	EVT_TREE_ITEM_COLLAPSED(NewtonModelEditor::ID_EDIT_NODE_NAME, OnCollapseItem) 
	EVT_TREE_SEL_CHANGED(NewtonModelEditor::ID_EDIT_NODE_NAME, OnSelectItem)
	EVT_TREE_DELETE_ITEM(NewtonModelEditor::ID_EDIT_NODE_NAME, OnDeleteItem)
//	EVT_TREE_KEY_DOWN (NewtonModelEditor::ID_EDIT_NODE_NAME, OnKeyboardItem)
	EVT_TREE_BEGIN_LABEL_EDIT (NewtonModelEditor::ID_EDIT_NODE_NAME, OnBeginEditItemName)
	EVT_TREE_END_LABEL_EDIT (NewtonModelEditor::ID_EDIT_NODE_NAME, OnEndEditItemName)

END_EVENT_TABLE()



class EditorExplorer::ExplorerData: public wxTreeItemData
{
	public:
	ExplorerData (dNodeInfo* const info)
		:wxTreeItemData()
		,m_info(info)
	{
	}
	~ExplorerData ()
	{
	}

	dNodeInfo* m_info;
};


class EditorExplorer::TraverseExplorer
{
	public:
	TraverseExplorer (){}

	virtual bool TraverseCallback (wxTreeItemId rootItem) = 0;

	void TraverseExplorer::Traverse(const EditorExplorer* const me)
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

	bool TraverseCallback (wxTreeItemId item)
	{
		ExplorerData* const data = (ExplorerData*) m_me->GetItemData(item);
		if (m_nodeInfo == data->m_info) {
			m_me->SetItemText(item, wxString (m_nodeInfo->GetName()));
		}
		return true;
	}

	EditorExplorer* m_me;
	dNodeInfo* m_nodeInfo;
};

class EditorExplorer::SelectDuplicatesItems: public EditorExplorer::TraverseExplorer
{
	public:
	SelectDuplicatesItems (EditorExplorer* const me)
		:EditorExplorer::TraverseExplorer()
		,m_me(me)
	{
		Traverse(me);
	}

	virtual bool TraverseCallback (wxTreeItemId item)
	{
		ExplorerData* const nodeData = ((ExplorerData*)m_me->GetItemData(item));
		if (m_me->IsSelected(item) && !(nodeData->m_info->GetEditorFlags() & dNodeInfo::m_selected)) {
			m_me->SelectItem (item, false);
		} else if (!m_me->IsSelected(item) && (nodeData->m_info->GetEditorFlags() & dNodeInfo::m_selected)) {
			m_me->SelectItem (item, true);
		}
		return true;
	}
	EditorExplorer* m_me;
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



class EditorExplorer::UndoRedoSelection: public dUndoRedo, public dList<dNodeInfo*>
{
	public:
	using dUndoRedo::operator new;
	using dUndoRedo::operator delete;

	UndoRedoSelection(EditorExplorer* const me)
		:dUndoRedo()
		,dList<dNodeInfo*>()
		,m_me(me) 
	{
		dPluginScene* const scene = m_me->m_mainFrame->GetScene();
		for (dScene::dTreeNode* node = scene->GetFirstNode(); node; node = scene->GetNextNode(node)) {
			dNodeInfo* const info = scene->GetInfoFromNode(node);
			if (info->GetEditorFlags() & dNodeInfo::m_selected) {
				Append (info);
			}
		}
	}

	~UndoRedoSelection()
	{
	}

	virtual void RestoreState(dUndodeRedoMode mode)
	{
		dPluginScene* const scene = m_me->m_mainFrame->GetScene();
		for (dScene::dTreeNode* node = scene->GetFirstNode(); node; node = scene->GetNextNode(node)) {
			dNodeInfo* const info = scene->GetInfoFromNode(node);
			info->SetEditorFlags(info->GetEditorFlags() & ~dNodeInfo::m_selected);
		}

		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			dNodeInfo* const info = node->GetInfo();
			info->SetEditorFlags(info->GetEditorFlags() | dNodeInfo::m_selected);
		}

		m_me->m_recursiveSelectionCall ++;
		SelectDuplicatesItems selectedDuplicatedItems(m_me);
		m_me->m_recursiveSelectionCall --;
	}

	virtual dUndoRedo* CreateRedoState() const
	{
		return new UndoRedoSelection (m_me);
	}

	EditorExplorer* m_me;
};


class EditorExplorer::SelectionList: public EditorExplorer::TraverseExplorer, public dList<dNodeInfo*>
{
	public:
	SelectionList (EditorExplorer* const me)
		:EditorExplorer::TraverseExplorer()
		,dList<dNodeInfo*>()
		,m_me(me)
	{
		Traverse(me);
	}

	virtual bool TraverseCallback (wxTreeItemId item)
	{
		if (m_me->IsSelected(item)) {
			ExplorerData* const nodeData = ((ExplorerData*)m_me->GetItemData(item));
			Append(nodeData->m_info);
		}
		return true;
	}
	EditorExplorer* m_me;
};


EditorExplorer::EditorExplorer(NewtonModelEditor* const mainFrame)
	:wxTreeCtrl (mainFrame, NewtonModelEditor::ID_EDIT_NODE_NAME, wxDefaultPosition, wxSize (EXPLORER_PANEL_DEFAULT_WITH, EXPLORER_PANEL_DEFAULT_HEIGHT), wxTR_EDIT_LABELS | wxTR_MULTIPLE | wxTR_EXTENDED | wxTR_HAS_BUTTONS | wxTR_LINES_AT_ROOT)
	,m_mainFrame(mainFrame)
	,m_recursiveSelectionCall(0)
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

void EditorExplorer::Clear()
{
	DeleteAllItems();
}

void EditorExplorer::OnKeyboardItem (wxKeyEvent& event)
{
    int keyCode = event.GetKeyCode();
	switch (keyCode)
	{
        case WXK_ESCAPE:
        {
            // send a display refresh event in case the runtime update is stopped bu the user.
            wxMenuEvent exitEvent (wxEVT_COMMAND_MENU_SELECTED, wxID_EXIT);
            m_mainFrame->GetEventHandler()->ProcessEvent(exitEvent);
            return;
        }

		case WXK_DELETE:
		{
			wxTreeEvent event(wxEVT_COMMAND_TREE_DELETE_ITEM, this, wxTreeItemId());
			GetEventHandler()->ProcessEvent (event);
			return;
		}
	}

	event.Skip();
}

void EditorExplorer::OnDeleteItem (wxTreeEvent& event)
{
//	wxArrayTreeItemIds items;
//	size_t count = GetSelections(items);
//	dAssert (0);
}

void EditorExplorer::OnSelectItem (wxTreeEvent& event)
{
	m_recursiveSelectionCall ++;
	if (m_recursiveSelectionCall == 1) {
		m_mainFrame->Push (new UndoRedoSelection(this));

//	wxTreeItemId item (event.GetItem());
//	wxTreeItemId oldItem (event.GetOldItem());
//	wxString xxx0 (item ? GetItemText(item).c_str() : "");
//	wxString xxx1 (oldItem ? GetItemText(oldItem).c_str() : "");
//	dTrace (("item: %s    olditem: %s\n", xxx0.mb_str(), xxx1.mb_str()));

		dPluginScene* const scene = m_mainFrame->GetScene();
		for (dScene::dTreeNode* node = scene->GetFirstNode(); node; node = scene->GetNextNode(node)) {
			dNodeInfo* const info = scene->GetInfoFromNode(node);
			info->SetEditorFlags(info->GetEditorFlags() & ~dNodeInfo::m_selected);
		}

		SelectionList selectionList(this);
		for (SelectionList::dListNode* node = selectionList.GetFirst(); node; node = node->GetNext()) {
			dNodeInfo* const info = node->GetInfo();
			info->SetEditorFlags(info->GetEditorFlags() | dNodeInfo::m_selected);
		}

		SelectDuplicatesItems selectedDuplicatedItems(this);
	}

	m_recursiveSelectionCall --;
	dAssert (m_recursiveSelectionCall >= 0);
}


void EditorExplorer::OnBeginEditItemName (wxTreeEvent& event)
{
}

void EditorExplorer::OnEndEditItemName (wxTreeEvent& event)
{
	wxString name (event.GetLabel());
	if (!name.IsEmpty()) {
		wxTreeItemId item (event.GetItem());
		ExplorerData* const data = (ExplorerData*) GetItemData(item);
		dNodeInfo* const info = data->m_info;
		m_mainFrame->Push (new UndoRedoChangeName(this, wxString(info->GetName()), info));
		info->SetName(name.c_str());
		ChangeNames changeName (this, info);
	}
}

void EditorExplorer::OnExpandItem (wxTreeEvent& event)
{
	wxTreeItemId item (event.GetItem());
	ExplorerData* const nodeData = ((ExplorerData*)GetItemData(item));
	nodeData->m_info->SetEditorFlags(nodeData->m_info->GetEditorFlags() | dNodeInfo::m_expanded);
}

void EditorExplorer::OnCollapseItem (wxTreeEvent& event)
{
	wxTreeItemId item (event.GetItem());
	ExplorerData* const nodeData = ((ExplorerData*)GetItemData(item));
	nodeData->m_info->SetEditorFlags(nodeData->m_info->GetEditorFlags() & ~dNodeInfo::m_expanded);
}


void EditorExplorer::ReconstructScene(const dPluginScene* const scene)
{
	if (GetRootItem() == NULL) {
		dScene::dTreeNode* const rootNode = scene->GetRootNode();
		dNodeInfo* const rootInfo = scene->GetInfoFromNode(rootNode);
		AddRoot(wxT (rootInfo->GetName()) , 0, -1, new ExplorerData(rootInfo));
	}
		
	dList<wxTreeItemId> stack;
	stack.Append(GetRootItem());
	while (stack.GetCount()) {
		wxTreeItemId rootItem (stack.GetLast()->GetInfo());
		stack.Remove(stack.GetLast());

		ExplorerData* const nodeData = ((ExplorerData*)GetItemData(rootItem));
		dScene::dTreeNode* const rootNode = scene->FindNode (nodeData->m_info);
		dAssert (rootNode);

		for (void* link = scene->GetFirstChildLink(rootNode); link; link = scene->GetNextChildLink(rootNode, link)) {
			dScene::dTreeNode* const childNode = scene->GetNodeFromLink (link);
			bool found = false;
			for (wxTreeItemId childItem = GetLastChild(rootItem); childItem; childItem = GetPrevSibling(childItem)) {
				ExplorerData* const data = ((ExplorerData*)GetItemData(childItem));
				dScene::dTreeNode* const node = scene->FindNode (data->m_info);
				dAssert (node);
				if (node == childNode) {
					found = true;
					break;
				}
			}
			
			if (!found) {
				dNodeInfo* const info = scene->GetInfoFromNode(childNode);
				if (info->IsType(dSceneCacheInfo::GetRttiType())) {
					PrependItem(rootItem, wxT(info->GetName()), 1, -1, new ExplorerData(info));
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
						PrependItem(rootItem, wxT(info->GetName()), imageId, -1, new ExplorerData(info));
					} else {
						AppendItem(rootItem, wxT(info->GetName()), imageId, -1, new ExplorerData(info));
					}
				}
			}
		}
	
		for (wxTreeItemId childItem = GetLastChild(rootItem); childItem; childItem = GetPrevSibling(childItem)) {
			stack.Append(childItem);
		}
	}

	stack.Append(GetRootItem());
	while (stack.GetCount()) {
		wxTreeItemId rootItem (stack.GetLast()->GetInfo());
		stack.Remove(stack.GetLast());

		ExplorerData* const nodeData = ((ExplorerData*)GetItemData(rootItem));
		const unsigned itemFlags = nodeData->m_info->GetEditorFlags();
		if (itemFlags & dNodeInfo::m_expanded) {
			Expand (rootItem);
		} else {
			Collapse (rootItem);
		}
		for (wxTreeItemId childItem = GetLastChild(rootItem); childItem; childItem = GetPrevSibling(childItem)) {
			stack.Append(childItem);
		}
	}
}
