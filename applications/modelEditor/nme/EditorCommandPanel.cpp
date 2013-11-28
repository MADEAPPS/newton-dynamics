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
#include "NewtonModelEditor.h"
#include "EditorCommandPanel.h"



EditorCommandPanel::EditorCommandPanel (NewtonModelEditor* const mainFrame)
	:wxFoldPanelBar (mainFrame, NewtonModelEditor::ID_EDIT_NODE_NAME)
{
/*
	FXVerticalFrame* const contents = new FXVerticalFrame(this, LAYOUT_SIDE_LEFT|FRAME_NONE|LAYOUT_FILL, 0,0,0,0, 0,0,0,0);
	//	FXTabItem* effectItem = new FXTabItem(m_tabBook, "&Simple List", NULL, FRAME_NONE|LAYOUT_FILL);
	//	FXHorizontalFrame* dirframe=new FXHorizontalFrame(m_tabBook,FRAME_THICK|FRAME_RAISED|LAYOUT_FILL);
	//	FXHorizontalFrame* boxframe=new FXHorizontalFrame(dirframe,FRAME_THICK|FRAME_SUNKEN|LAYOUT_FILL_X|LAYOUT_FILL_Y, 0,0,0,0, 0,0,0,0);
	//	FXTreeList* dirlist = new FXTreeList(m_tabBook, NULL,0,DIRLIST_SHOWFILES|TREELIST_SHOWS_LINES|TREELIST_SHOWS_BOXES|FRAME_NONE|LAYOUT_FILL);

	int ID_OPEN_TREE = 1000;
	//int ID_FILEFILTER = 1001;

	FXHorizontalFrame* const treeframe = new FXHorizontalFrame(contents,FRAME_SUNKEN|FRAME_THICK|LAYOUT_FILL_X|LAYOUT_FILL_Y,0,0,0,0, 0,0,0,0);
	//	FXDirList* dirlist=new FXDirList(treeframe,this,ID_OPEN_TREE,DIRLIST_SHOWFILES|DIRLIST_NO_OWN_ASSOC|TREELIST_BROWSESELECT|TREELIST_SHOWS_LINES|TREELIST_SHOWS_BOXES|LAYOUT_FILL_X|LAYOUT_FILL_Y);
	new FXTreeList(treeframe, mainFrame, ID_OPEN_TREE, TREELIST_BROWSESELECT|TREELIST_SHOWS_LINES|TREELIST_SHOWS_BOXES|LAYOUT_FILL_X|LAYOUT_FILL_Y);
*/
}

EditorCommandPanel::~EditorCommandPanel()
{
}


