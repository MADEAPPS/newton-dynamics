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
#include "EditorMainMenu.h"
#include "NewtonModelEditor.h"

EditorMainMenu::EditorMainMenu(NewtonModelEditor* const parent)
	:wxMenuBar()
	,m_mainFrame(parent)
	,m_fileMenu(NULL)
	,m_editMenu(NULL)
	,m_viewMenu(NULL)
	,m_toolMenu(NULL)
	,m_meshMenu(NULL)
	,m_helpMenu(NULL)
	,m_importPlugins(NULL)
	,m_exportPlugins(NULL)
{
	CreateFileMenu();
	CreateEditMenu();
	CreateViewMenu();
	CreateToolMenu();
	CreateMeshMenu();
	CreateHelpMenu();

	// add main menus to menu bar
	Append (m_fileMenu, wxT("File"));
	Append (m_editMenu, wxT("Edit"));
	Append (m_viewMenu, wxT("View"));
	Append (m_toolMenu, wxT("Tools"));
	Append (m_meshMenu, wxT("Mesh"));
	Append (m_helpMenu, wxT("Help"));
}

EditorMainMenu::~EditorMainMenu(void)
{
}

dPluginRecord* EditorMainMenu::GetPlugin (wxMenu* const paneMenu, int id)
{
	wxMenuItem* const item = paneMenu->FindItemByPosition (id);
	dAssert (item);
	EditorPlugin* const plugin = (EditorPlugin*)item->GetRefData();
	dAssert(plugin);
	return plugin->m_plugin;
}

void EditorMainMenu::AddPlugin (wxMenu* const menu, dPluginRecord* const plugin)
{
	BasePluginBaseMenuId* const baseIdData = (BasePluginBaseMenuId*)menu->GetRefData();
	dAssert(baseIdData);
	int id = baseIdData->m_baseID + menu->GetMenuItemCount();
	wxMenuItem* const item = menu->Append (id, wxString (plugin->GetMenuName ()));
	item->SetRefData (new EditorPlugin(plugin));
}


void EditorMainMenu::CreateHelpMenu()
{
	wxMenu* const menu = new wxMenu;
	menu->Append(wxID_HELP, wxT("About"));
	m_helpMenu = menu;
}

void EditorMainMenu::CreateEditMenu()
{
	wxMenu* const menu = new wxMenu;
	menu->Append(wxID_UNDO, wxT("&Undo\tCtrl-Z"), wxT("Undo previous action"));
	menu->Append(wxID_REDO, wxT("&Redo\tCtrl-Y"), wxT("Redo previous action"));
	menu->Append(NewtonModelEditor::ID_CLEAR_UNDO_HISTORY, wxT("Clear undo history"), wxT("clear the undo redo history"));
	m_editMenu = menu;
}

wxWindow* EditorMainMenu::GetViewControl (int controlId) const
{
	wxMenuItem* const item = m_viewMenu->FindItem (controlId);
	dAssert (item);
	EditorViewControl* const control = (EditorViewControl*)item->GetRefData();
	dAssert(control);
	return control->m_control;
}

void EditorMainMenu::AddViewControl (int controlId, const char* const title, wxWindow* const control)
{
	char help[1024];
	sprintf (help, "hide/unhide %s pane", title);

	wxMenuItem* const item = m_viewMenu->AppendCheckItem (controlId, wxT(title), wxT(help));
	item->SetRefData (new EditorViewControl(control));
	item->Check();
}

void EditorMainMenu::CreateViewMenu()
{
	wxMenu* const menu = new wxMenu;
	m_viewMenu = menu;
}


void EditorMainMenu::CreateFileMenu()
{
	wxMenu* const menu = new wxMenu;

	// make the Mac Happy
	menu->Append(wxID_ABOUT, wxT("About"));

	menu->AppendSeparator();
	menu->Append(wxID_PREFERENCES, wxT("Preferences"));

	// new scene
	menu->AppendSeparator();
	menu->Append (wxID_NEW, wxT("New"), wxT("clean scene"));

	menu->AppendSeparator();
	menu->Append (wxID_OPEN, wxT("&Open scene\tCtrl-O"), wxT("Open an existing scene"));
	menu->Append (wxID_SAVE, wxT("Save scene\tCtrl-S"), wxT("Save current scene"));
	menu->Append (wxID_SAVEAS, wxT("Save scene as"), wxT("Save current scene to different file"));

	// add import export plugins file sub menu
	{
		menu->AppendSeparator();
		m_importPlugins = new wxMenu;
		menu->AppendSubMenu(m_importPlugins, wxT("Import plugins ..."), wxT("import third party file to the scene"));
		m_importPlugins->SetRefData(new BasePluginBaseMenuId(NewtonModelEditor::ID_IMPORT_PLUGINS));

		m_exportPlugins = new wxMenu;
		menu->AppendSubMenu(m_exportPlugins, wxT("Export plugins ..."), wxT("export slect mesh to a third party file format"));
		m_exportPlugins->SetRefData(new BasePluginBaseMenuId(NewtonModelEditor::ID_EXPORT_PLUGINS));
	}

	//quick editor
	menu->AppendSeparator();
	menu->Append(wxID_EXIT, wxT("E&xit\tAlt-X"), wxT("Quit SDK editor") );

	m_fileMenu = menu;
}


void EditorMainMenu::CreateToolMenu()
{
	wxMenu* const menu = new wxMenu;
	menu->SetRefData(new BasePluginBaseMenuId(NewtonModelEditor::ID_TOOL_PLUGINS));
	m_toolMenu = menu;
}

void EditorMainMenu::CreateMeshMenu()
{
	wxMenu* const menu = new wxMenu;
	menu->SetRefData(new BasePluginBaseMenuId(NewtonModelEditor::ID_MESH_PLUGINS));
	m_meshMenu = menu;
}