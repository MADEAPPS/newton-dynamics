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
#include "NewtonModelEditor.h"
#include "EditorCommandPanel.h"

#define CMD_PANEL_DEFFAULT_WITH		256
#define CMD_PANEL_DEFFAULT_HEIGHT	160

class EditorCommandPanel::FoldBarCommandPanel: public wxFoldPanelBar
{
	public:
	FoldBarCommandPanel (wxWindow* const mainFrame)
		:wxFoldPanelBar (mainFrame, NewtonModelEditor::ID_EDIT_NODE_NAME, wxDefaultPosition, wxSize (CMD_PANEL_DEFFAULT_WITH, CMD_PANEL_DEFFAULT_HEIGHT))
		,m_mainFrame ((NewtonModelEditor*)mainFrame->GetParent())
	{
		wxFoldPanel item = AddFoldPanel(_T("Test me"), false);
		AddFoldPanelWindow(item, new wxCheckBox(item.GetParent(), wxID_ANY, _T("I like this")));

		item = AddFoldPanel(_T("Test me too!"), false);
		AddFoldPanelWindow(item, new wxCheckBox(item.GetParent(), wxID_ANY, _T("I like this")));

		item = AddFoldPanel(_T("Test me too 2!"), false);
		AddFoldPanelWindow(item, new wxCheckBox(item.GetParent(), wxID_ANY, _T("I like this")));
	}

	~FoldBarCommandPanel()
	{
	}

	NewtonModelEditor* const m_mainFrame;
};



EditorCommandPanel::EditorCommandPanel (NewtonModelEditor* const mainFrame)
	:wxScrolledWindow (mainFrame, wxID_ANY, wxDefaultPosition, wxSize (CMD_PANEL_DEFFAULT_WITH, CMD_PANEL_DEFFAULT_HEIGHT))
{
	// the sizer will take care of determining the needed scroll size
	// (if you don't use sizers you will need to manually set the viewport size)
	wxBoxSizer* const sizer = new wxBoxSizer(wxVERTICAL);

	// add a series of widgets
//	for (int w=1; w<=10; w++)
//	{
//		wxButton* b = new wxButton(this, wxID_ANY, wxString::Format(wxT("Button %i"), w));
//		sizer->Add(b, 0, wxALL, 3);
//	}
	FoldBarCommandPanel* const commandPanel = new FoldBarCommandPanel(this);
	sizer->Add(commandPanel, 0, wxALL, 0);

	SetSizer(sizer);

	// this part makes the scrollbars show up
	FitInside(); // ask the sizer about the needed size
	SetScrollRate(5, 5);
}


EditorCommandPanel::~EditorCommandPanel()
{
}



