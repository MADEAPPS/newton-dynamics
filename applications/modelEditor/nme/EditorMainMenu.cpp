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


EditorMainMenu::EditorMainMenu(FXComposite* const parent, FXToolBarShell* const dragShell, NewtonModelEditor* const mainFrame)
	:FXMenuBar(parent, dragShell, LAYOUT_DOCK_SAME|LAYOUT_SIDE_TOP|LAYOUT_FILL_X|FRAME_RAISED)
	,m_mainFrame(mainFrame)
{
	new FXToolBarGrip(this, this, FXMenuBar::ID_TOOLBARGRIP,TOOLBARGRIP_DOUBLE);

	// file menu
	{
		m_fileMenu = new FXMenuPane(this);
		new FXMenuTitle(this, "File", NULL, m_fileMenu);

		// new scene
		{
			new FXMenuCommand(m_fileMenu, "New", mainFrame->FindIcon("fileNew.gif"), mainFrame, NewtonModelEditor::ID_NEW);
		}

		// load save scene
		{
//			new FXMenuSeparator(m_fileMenu);
//			new FXMenuCommand(m_fileMenu, "Load scene", mainFrame->FindIcon("fileOpen.gif"), mainFrame, NewtonModelEditor::ID_LOAD_SCENE);
//			new FXMenuCommand(m_fileMenu, "Save scene ", mainFrame->FindIcon("fileSave.gif"), mainFrame, NewtonModelEditor::ID_SAVE_SCENE);
//			new FXMenuCommand(m_fileMenu, "Save scene as ...", mainFrame->FindIcon("fileSaveAs.gif"), mainFrame, NewtonModelEditor::ID_SAVE_SCENE_AS);

			new FXMenuSeparator(m_fileMenu);
			new FXMenuCommand(m_fileMenu, "Load asset", mainFrame->FindIcon("fileOpen.gif"), mainFrame, NewtonModelEditor::ID_LOAD_ASSET);
			new FXMenuCommand(m_fileMenu, "Save asset", mainFrame->FindIcon("fileSave.gif"), mainFrame, NewtonModelEditor::ID_SAVE_ASSET);
			new FXMenuCommand(m_fileMenu, "Save asset as ...", mainFrame->FindIcon("fileSaveAs.gif"), mainFrame, NewtonModelEditor::ID_SAVE_ASSET_AS);
		}

		// load save assets
		{
//			new FXMenuSeparator(m_fileMenu);
//			new FXMenuCommand(m_fileMenu, "Load asset", NULL, mainFrame, NewtonModelEditor::ID_LOAD_ASSET);
//			new FXMenuCommand(m_fileMenu, "Save asset", NULL, mainFrame, NewtonModelEditor::ID_SAVE_ASSET);
//			new FXMenuCommand(m_fileMenu, "Save asset as ...", NULL, mainFrame, NewtonModelEditor::ID_SAVE_ASSET_AS);
		}

		
		// add Recent file sub menu
		{
			new FXMenuSeparator(m_fileMenu);
			m_recentFilesMenu = new FXMenuPane(this);
			new FXMenuCascade(m_fileMenu, "Recent Files...", NULL, m_recentFilesMenu);
			for (int i = NewtonModelEditor::ID_RECENT_FILES; i < NewtonModelEditor::ID_MAX_RECENT_FILES; i ++) {
				new FXMenuCommand(m_recentFilesMenu, FXString::null, NULL, mainFrame, i);
			}
		}
		

		// add import export plugins file sub menu
		{
			new FXMenuSeparator(m_fileMenu);
			m_importPlugins = new FXMenuPane(this);
			new FXMenuCascade(m_fileMenu, "Import plugins...", NULL, m_importPlugins);
			for (int i = NewtonModelEditor::ID_IMPORT_PLUGINS; i < NewtonModelEditor::ID_MAX_IMPORT_PLUGINS; i ++) {
				new FXMenuCommand(m_importPlugins, FXString::null, NULL, mainFrame, i);
			}

			m_exportPlugins = new FXMenuPane(this);
			new FXMenuCascade(m_fileMenu, "Export plugins...", NULL, m_exportPlugins);
			for (int i = NewtonModelEditor::ID_EXPORT_PLUGINS; i < NewtonModelEditor::ID_MAX_EXPORT_PLUGINS; i ++) {
				new FXMenuCommand(m_exportPlugins, FXString::null, NULL, mainFrame, i);
			}
		}

		// quick editor
		{
			new FXMenuSeparator(m_fileMenu);
			new FXMenuCommand(m_fileMenu, "Quit", NULL, getApp(), FXApp::ID_QUIT);
		}
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

	// add help menu
	{
		m_helpMenu = new FXMenuPane(this);
		new FXMenuTitle(this, "Help", NULL, m_helpMenu);
		new FXMenuCommand(m_helpMenu, "About", NULL, mainFrame, NewtonModelEditor::ID_ABOUT);
	}

}

EditorMainMenu::~EditorMainMenu(void)
{
	delete m_helpMenu;
	delete m_fileMenu;
	delete m_editMenu;
	delete m_meshMenu;
	delete m_modelMenu;
	delete m_optionsMenu;
	delete m_importPlugins;
	delete m_exportPlugins;
	delete m_preferencesMenu;
	delete m_recentFilesMenu;
}



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

