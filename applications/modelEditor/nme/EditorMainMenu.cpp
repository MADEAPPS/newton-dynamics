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
#include "EditorMainMenu.h"
#include "NewtonModelEditor.h"

EditorMainMenu::EditorMainMenu(NewtonModelEditor* const parent)
	:wxMenuBar()
	,m_mainFrame(parent)
	,m_fileMenu(NULL)
	,m_editMenu(NULL)
	,m_viewMenu(NULL)
	,m_meshMenu(NULL)
	,m_helpMenu(NULL)
	,m_importPlugins(NULL)
	,m_exportPlugins(NULL)
{
	CreateFileMenu();
	CreateEditMenu();
	CreateViewMenu();
	CreateMeshMenu();
	CreateHelpMenu();

	// add main menus to menu bar
	Append (m_fileMenu, wxT("&File"));
	Append (m_editMenu, wxT("&Edit"));
	Append (m_viewMenu, wxT("&View"));
	Append (m_meshMenu, wxT("&Mesh"));
	Append (m_helpMenu, wxT("&Help"));

/*


	// models menu
	{
		// this many is going to be populate bu the plugin manager
		m_modelMenu  = new FXMenuPane(this);
		new FXMenuTitle(this, "Models", NULL, m_modelMenu);
		for (int i = NewtonModelEditor::ID_MODEL_PLUGINS; i < NewtonModelEditor::ID_MAX_MODELS_PLUGINS; i ++) {
			new FXMenuCommand(m_modelMenu, FXString::null, NULL, mainFrame, i);
		}
	}


	// option menu
	{
		m_optionsMenu = new FXMenuPane(this);
		new FXMenuTitle(this, "Options", NULL, m_optionsMenu);

		new FXMenuCheck(m_optionsMenu, "hide file toolBar", mainFrame, NewtonModelEditor::ID_HIDE_FILE_TOOLBAR);
		new FXMenuCheck(m_optionsMenu, "hide navigation toolBar", mainFrame, NewtonModelEditor::ID_HIDE_NAVIGATION_TOOLBAR);
		new FXMenuCheck(m_optionsMenu, "hide explorer panel", mainFrame, NewtonModelEditor::ID_HIDE_EXPLORER_PANEL);
		new FXMenuCheck(m_optionsMenu, "hide command panel", mainFrame, NewtonModelEditor::ID_HIDE_COMMAND_PANEL);

		{
			new FXMenuSeparator(m_optionsMenu);
			m_preferencesMenu = new FXMenuPane(this);
			new FXMenuCascade(m_optionsMenu, "Preferences...", NULL, m_preferencesMenu);
			new FXMenuCommand(m_preferencesMenu, "keyboard shortcuts", NULL, mainFrame, NewtonModelEditor::ID_KEYBOARD_SHORCUTS);

		}
		
	}

*/

	
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
	wxMenuItem* const item = menu->Append (baseIdData->m_baseID + menu->GetMenuItemCount(), wxString (plugin->GetMenuName ()));
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
//("&Hello\tCtrl-H");
	menu->Append(wxID_UNDO, wxT("&Undo\tCtrl-Z"), wxT("Undo previous action"));
	menu->Append(wxID_REDO, wxT("&Redo\tCtrl-Y"), wxT("Redo previous action"));
	menu->Append(NewtonModelEditor::ID_CLEAR_UNDO_HISTORY, wxT("Clear undo history"), wxT("clear the undo redo history"));

	m_editMenu = menu;
}

void EditorMainMenu::CreateViewMenu()
{
	wxMenu* const menu = new wxMenu;

	wxMenuItem* const item = menu->AppendCheckItem(NewtonModelEditor::ID_HIDE_EXPLORER_PANE, wxT("scene explorer"), wxT("hide/unhide scene explore pane"));
	item->Check();

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
	menu->Append (wxID_OPEN, wxT("&Open scene\tCtrl-O"), wxT("open an existing scene"));
	menu->Append (wxID_SAVE, wxT("Save scene\tCtrl-S"), wxT("Save current scene"));
	menu->Append (wxID_SAVEAS, wxT("Save scene as"), wxT("Save current scene to different file"));


/*
		new FXMenuSeparator(menu);
		new FXMenuCommand(menu, "Load asset", mainFrame->FindIcon("fileOpen.gif"), mainFrame, NewtonModelEditor::ID_LOAD_ASSET);
		new FXMenuCommand(menu, "Save asset", mainFrame->FindIcon("fileSave.gif"), mainFrame, NewtonModelEditor::ID_SAVE_ASSET);
		new FXMenuCommand(menu, "Save asset as ...", mainFrame->FindIcon("fileSaveAs.gif"), mainFrame, NewtonModelEditor::ID_SAVE_ASSET_AS);
	}

	// load save assets
	{
		//			new FXMenuSeparator(menu);
		//			new FXMenuCommand(menu, "Load asset", NULL, mainFrame, NewtonModelEditor::ID_LOAD_ASSET);
		//			new FXMenuCommand(menu, "Save asset", NULL, mainFrame, NewtonModelEditor::ID_SAVE_ASSET);
		//			new FXMenuCommand(menu, "Save asset as ...", NULL, mainFrame, NewtonModelEditor::ID_SAVE_ASSET_AS);
	}


	// add Recent file sub menu
	{
		new FXMenuSeparator(menu);
		m_recentFilesMenu = new FXMenuPane(this);
		new FXMenuCascade(menu, "Recent Files...", NULL, m_recentFilesMenu);
		for (int i = NewtonModelEditor::ID_RECENT_FILES; i < NewtonModelEditor::ID_MAX_RECENT_FILES; i ++) {
			new FXMenuCommand(m_recentFilesMenu, FXString::null, NULL, mainFrame, i);
		}
	}
*/

	// add import export plugins file sub menu
	{
		menu->AppendSeparator();
		m_importPlugins = new wxMenu;
		menu->AppendSubMenu(m_importPlugins, wxT("Import plugins..."), wxT("execute and scen import plug in"));
//		new FXMenuCascade(menu, "Import plugins...", NULL, m_importPlugins);
//		for (int i = NewtonModelEditor::ID_IMPORT_PLUGINS; i < NewtonModelEditor::ID_MAX_IMPORT_PLUGINS; i ++) {
//			new FXMenuCommand(m_importPlugins, FXString::null, NULL, mainFrame, i);
//		}

		m_exportPlugins = new wxMenu;
		menu->AppendSubMenu(m_exportPlugins, wxT("Export plugins..."), wxT("execute and scene iexport plug in"));
//		for (int i = NewtonModelEditor::ID_EXPORT_PLUGINS; i < NewtonModelEditor::ID_MAX_EXPORT_PLUGINS; i ++) {
//			new FXMenuCommand(m_exportPlugins, FXString::null, NULL, mainFrame, i);
//		}
	}

	//quick editor
	menu->AppendSeparator();
	menu->Append(wxID_EXIT, wxT("E&xit\tAlt-X"), wxT("Quit SDK editor") );

	m_fileMenu = menu;
}


void EditorMainMenu::CreateMeshMenu()
{
	// mesh menu
//	{
//		// this many is going to be populate bu the plugin manager
//		m_meshMenu  = new FXMenuPane(this);
//		new FXMenuTitle(this, "Mesh", NULL, m_meshMenu);
//		for (int i = NewtonModelEditor::ID_MESH_PLUGINS; i < NewtonModelEditor::ID_MAX_MESH_PLUGINS; i ++) {
//			new FXMenuCommand(m_meshMenu, FXString::null, NULL, mainFrame, i);
//		}
//	}

	wxMenu* const menu = new wxMenu;
	menu->SetRefData(new BasePluginBaseMenuId(NewtonModelEditor::ID_MESH_PLUGINS));
	m_meshMenu = menu;
}