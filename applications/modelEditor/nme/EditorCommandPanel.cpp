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
	:wxFoldPanelBar (mainFrame, NewtonModelEditor::ID_EDIT_NODE_NAME, wxDefaultPosition, wxSize (200, 160))
{
	wxFoldPanel item = AddFoldPanel(_T("Test me"), false);
	AddFoldPanelWindow(item, new wxCheckBox(item.GetParent(), wxID_ANY, _T("I like this")));

	item = AddFoldPanel(_T("Test me too!"), true);
	AddFoldPanelWindow(item, new wxCheckBox(item.GetParent(), wxID_ANY, _T("I like this")));

	item = AddFoldPanel(_T("Test me too 2!"), true);
	AddFoldPanelWindow(item, new wxCheckBox(item.GetParent(), wxID_ANY, _T("I like this")));

}

EditorCommandPanel::~EditorCommandPanel()
{
}



