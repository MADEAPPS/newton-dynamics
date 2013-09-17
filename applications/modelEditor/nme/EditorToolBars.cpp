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
#include "EditorToolBars.h"
#include "NewtonModelEditor.h"


EditorToolBar::EditorToolBar(FXComposite* const parent, FXToolBarShell* const dragShell, unsigned flags)
	:FXToolBar(parent, dragShell, flags)
{
	new FXToolBarGrip(this, this, FXMenuBar::ID_TOOLBARGRIP,TOOLBARGRIP_DOUBLE);
}

EditorToolBar::~EditorToolBar(void)
{
}


void EditorToolBar::Hide()
{
	hide();
	recalc();
}

void EditorToolBar::Unhide()
{
	show();
	recalc();
}


EditorFileToolBar::EditorFileToolBar(FXComposite* const parent, FXToolBarShell* const dragShell, NewtonModelEditor* const mainFrame)
	:EditorToolBar(parent, dragShell, LAYOUT_DOCK_NEXT|LAYOUT_SIDE_TOP|FRAME_RAISED)
{
	new FXButton(this,"\tNew\tCreate new document.", mainFrame->FindIcon("fileNew.gif"), mainFrame, NewtonModelEditor::ID_NEW, BUTTON_TOOLBAR|FRAME_RAISED|LAYOUT_TOP|LAYOUT_LEFT);
	new FXButton(this,"\tOpen\tOpen document file.", mainFrame->FindIcon("fileOpen.gif"), mainFrame, NewtonModelEditor::ID_LOAD_SCENE, BUTTON_TOOLBAR|FRAME_RAISED|LAYOUT_TOP|LAYOUT_LEFT);
	new FXButton(this,"\tSave\tSave document."     , mainFrame->FindIcon("fileSave.gif"), mainFrame, NewtonModelEditor::ID_SAVE_SCENE, BUTTON_TOOLBAR|FRAME_RAISED|LAYOUT_TOP|LAYOUT_LEFT);
	new FXButton(this,"\tSave As\tSave document to another file.", mainFrame->FindIcon("fileSaveAs.gif"), mainFrame, NewtonModelEditor::ID_SAVE_SCENE_AS, BUTTON_TOOLBAR|FRAME_RAISED|LAYOUT_TOP|LAYOUT_LEFT);
}

EditorFileToolBar::~EditorFileToolBar(void)
{
}


EditorNavigationToolBar::EditorNavigationToolBar(FXComposite* const parent, FXToolBarShell* const dragShell, NewtonModelEditor* const mainFrame)
	:EditorToolBar(parent, dragShell, LAYOUT_DOCK_SAME|LAYOUT_SIDE_TOP|FRAME_RAISED)
{
	new FXButton(this,"\tUndo\tUndo previous action.", mainFrame->FindIcon("undo.gif"), mainFrame, NewtonModelEditor::ID_UNDO, BUTTON_TOOLBAR|FRAME_RAISED|LAYOUT_TOP|LAYOUT_LEFT);
	new FXButton(this,"\tRedo\tRedo previous action.", mainFrame->FindIcon("redo.gif"), mainFrame, NewtonModelEditor::ID_REDO, BUTTON_TOOLBAR|FRAME_RAISED|LAYOUT_TOP|LAYOUT_LEFT);

//	new FXButton(this,"\tCursor\tCursor mode.", mainFrame->FindIcon("cursor.gif"), mainFrame, NewtonModelEditor::ID_SELECT_COMMAND_MODE, BUTTON_TOOLBAR|FRAME_RAISED|LAYOUT_TOP|LAYOUT_LEFT);
	new FXButton(this,"\tSelect\tObject selection mode.", mainFrame->FindIcon("object_cursor.gif"), mainFrame, NewtonModelEditor::ID_SELECT_COMMAND_MODE, BUTTON_TOOLBAR|FRAME_RAISED|LAYOUT_TOP|LAYOUT_LEFT);
	new FXButton(this,"\tMove\tObject translation mode.", mainFrame->FindIcon("object_move.gif"), mainFrame, NewtonModelEditor::ID_TRANSLATE_COMMAND_MODE, BUTTON_TOOLBAR|FRAME_RAISED|LAYOUT_TOP|LAYOUT_LEFT);
	new FXButton(this,"\tRotate\tObject rotation mode.", mainFrame->FindIcon("object_turn.gif"), mainFrame, NewtonModelEditor::ID_ROTATE_COMMAND_MODE, BUTTON_TOOLBAR|FRAME_RAISED|LAYOUT_TOP|LAYOUT_LEFT);
	new FXButton(this,"\tScale\tObject scale mode.", mainFrame->FindIcon("object_scale.gif"), mainFrame, NewtonModelEditor::ID_SCALE_COMMAND_MODE, BUTTON_TOOLBAR|FRAME_RAISED|LAYOUT_TOP|LAYOUT_LEFT);


	
//	new FXLabel(this,"  edit mode:", NULL, BUTTON_TOOLBAR|FRAME_NONE|LAYOUT_TOP|LAYOUT_LEFT);
//	FXComboBox* const editMode = new FXComboBox(this, 2, mainFrame, NewtonModelEditor::ID_EDITOR_MODE, COMBOBOX_STATIC|FRAME_SUNKEN|FRAME_THICK|LAYOUT_SIDE_TOP);
//	editMode->setNumVisible(2);
//	editMode->appendItem(D_EDIT_MODE_ASSET);
//	editMode->appendItem(D_EDIT_MODE_SCENE);
}

EditorNavigationToolBar::~EditorNavigationToolBar(void)
{
}


