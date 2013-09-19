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
//EditorMainMenu::EditorMainMenu(FXComposite* const parent, FXToolBarShell* const dragShell, NewtonModelEditor* const mainFrame)
//	:FXMenuBar(parent, dragShell, LAYOUT_DOCK_SAME|LAYOUT_SIDE_TOP|LAYOUT_FILL_X|FRAME_RAISED)
//	,m_mainFrame(mainFrame)
	:wxMenuBar()
	,m_fileMenu(NULL)
	,m_helpMenu(NULL)
{
//	new FXToolBarGrip(this, this, FXMenuBar::ID_TOOLBARGRIP,TOOLBARGRIP_DOUBLE);


	CreateFileMenu();
	CreateHelpMenu();

	// add main menus to menu bar
	Append (m_fileMenu, wxT("&File"));
	Append (m_helpMenu, wxT("&Help"));


/*
	}

	// edit menu
	{
		m_editMenu = new FXMenuPane(this);
		new FXMenuTitle(this, "Edit", NULL, m_editMenu);

		new FXMenuCommand(m_editMenu, "Undo", mainFrame->FindIcon("undo.gif"), mainFrame, NewtonModelEditor::ID_UNDO);
		new FXMenuCommand(m_editMenu, "Redo", mainFrame->FindIcon("redo.gif"), mainFrame, NewtonModelEditor::ID_REDO);
		new FXMenuCommand(m_editMenu, "Clear Undo History", NULL, mainFrame, NewtonModelEditor::ID_CLEAR_UNDO);
		new FXMenuSeparator(m_editMenu);
	}


	// mesh menu
	{
		// this many is going to be populate bu the plugin manager
		m_meshMenu  = new FXMenuPane(this);
		new FXMenuTitle(this, "Mesh", NULL, m_meshMenu);
		for (int i = NewtonModelEditor::ID_MESH_PLUGINS; i < NewtonModelEditor::ID_MAX_MESH_PLUGINS; i ++) {
			new FXMenuCommand(m_meshMenu, FXString::null, NULL, mainFrame, i);
		}
	}


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
/*
	delete m_helpMenu;
	delete menu;
	delete m_editMenu;
	delete m_meshMenu;
	delete m_modelMenu;
	delete m_optionsMenu;
	delete m_importPlugins;
	delete m_exportPlugins;
	delete m_preferencesMenu;
	delete m_recentFilesMenu;
*/
}


/*
FXString EditorMainMenu::GetRecentFile(int id)
{
	int i = 0; 
	for (dList<FXString>::dListNode* node = m_recentFilesList.GetFirst(); node; node = node->GetNext()) {
		if (i == id) {
			const char * const name = node->GetInfo().text();
			FILE* const file = fopen (name, "rb");
			if (file) {
				fclose (file);
				return name;
			} else {
				m_recentFilesList.Remove(node);
				for (int i = 0; i < NewtonModelEditor::ID_MAX_RECENT_FILES - NewtonModelEditor::ID_RECENT_FILES; i ++) {
					FXMenuCommand* const item = (FXMenuCommand*) m_recentFilesMenu->childAtIndex(i);
					item->setText(FXString::null);
				}

				int index = 0;
				for (dList<FXString>::dListNode* node = m_recentFilesList.GetFirst(); node; node = node->GetNext()) {
					FXMenuCommand* const item = (FXMenuCommand*) m_recentFilesMenu->childAtIndex(index);
					item->setText(node->GetInfo());
					index ++;
				}
			}
			break;
		}
		i ++;
	}
	return FXString::null;
}



void EditorMainMenu::AddRecentFile(const FXString& filePathName)
{
	for (dList<FXString>::dListNode* node = m_recentFilesList.GetFirst(); node; node = node->GetNext()) {
		if (node->GetInfo() == filePathName) {
			m_recentFilesList.Remove(node);
			break;
		}
	}
	m_recentFilesList.Addtop(filePathName);

	if (m_recentFilesList.GetCount() > (NewtonModelEditor::ID_MAX_RECENT_FILES - NewtonModelEditor::ID_RECENT_FILES)) {
		m_recentFilesList.Remove(m_recentFilesList.GetLast());
	}

	for (int i = 0; i < NewtonModelEditor::ID_MAX_RECENT_FILES - NewtonModelEditor::ID_RECENT_FILES; i ++) {
		FXMenuCommand* const item = (FXMenuCommand*) m_recentFilesMenu->childAtIndex(i);
		item->setText(FXString::null);
	}

	int i = 0;
	for (dList<FXString>::dListNode* node = m_recentFilesList.GetFirst(); node; node = node->GetNext()) {
		FXMenuCommand* const item = (FXMenuCommand*) m_recentFilesMenu->childAtIndex(i);
		item->setText(node->GetInfo());
		i ++;
	}
}




void EditorMainMenu::AddPlugin (FXMenuPane* const paneMenu, dPluginRecord* const plugin)
{
	for (int i = 0; i < D_MAX_PLUGINS_COUNT; i ++) {
		FXMenuCommand* const item = (FXMenuCommand*) paneMenu->childAtIndex(i);
		if (item->getText().empty()) {
			item->setText(plugin->GetMenuName ());
			item->setUserData(plugin);
			return;
		}
	}
	_ASSERTE (0);
}

dPluginRecord* EditorMainMenu::GetPlugin (FXMenuPane* const paneMenu, int id)
{
	_ASSERTE (id < D_MAX_PLUGINS_COUNT);
	FXMenuCommand* const item = (FXMenuCommand*) paneMenu->childAtIndex(id);
	return (dPluginRecord*)item->getUserData();
}
*/


void EditorMainMenu::CreateHelpMenu()
{
	wxMenu* const menu = new wxMenu;
	menu->Append(wxID_HELP, wxT("About"));
	m_helpMenu = menu;
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
	menu->Append (wxID_OPEN, wxT("Open scene"), wxT("open an existing scene"));
	menu->Append (wxID_SAVE, wxT("Save scene"), wxT("Save current scene"));
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


	// add import export plugins file sub menu
	{
		new FXMenuSeparator(menu);
		m_importPlugins = new FXMenuPane(this);
		new FXMenuCascade(menu, "Import plugins...", NULL, m_importPlugins);
		for (int i = NewtonModelEditor::ID_IMPORT_PLUGINS; i < NewtonModelEditor::ID_MAX_IMPORT_PLUGINS; i ++) {
			new FXMenuCommand(m_importPlugins, FXString::null, NULL, mainFrame, i);
		}

		m_exportPlugins = new FXMenuPane(this);
		new FXMenuCascade(menu, "Export plugins...", NULL, m_exportPlugins);
		for (int i = NewtonModelEditor::ID_EXPORT_PLUGINS; i < NewtonModelEditor::ID_MAX_EXPORT_PLUGINS; i ++) {
			new FXMenuCommand(m_exportPlugins, FXString::null, NULL, mainFrame, i);
		}
	}

*/

	//quick editor
	menu->AppendSeparator();
	//new FXMenuCommand(menu, "Quit", NULL, getApp(), FXApp::ID_QUIT);
	menu->Append(wxID_EXIT, wxT("E&xit\tAlt-X"), wxT("Quit SDK editor") );

	m_fileMenu = menu;
}