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

#ifndef _EDITOR_MAIN_MENU_H_
#define _EDITOR_MAIN_MENU_H_


#include <dList.h>

class NewtonModelEditor;


class EditorMainMenu: public wxMenuBar
{
	public:
	EditorMainMenu (NewtonModelEditor* const parent);
	~EditorMainMenu(void);

	

	private:
	void CreateFileMenu();
	void CreateEditMenu();
	void CreateHelpMenu();

	void AddPlugin (wxMenu* const menu, dPluginRecord* const plugin);
/*
	FXString GetRecentFile(int id);
	void AddRecentFile(const FXString& filePathName);

	dPluginRecord* GetPlugin (FXMenuPane* const paneMenu, int id);
	
	
	NewtonModelEditor* m_mainFrame;
	FXMenuPane* m_fileMenu;
	FXMenuPane* m_editMenu;
	FXMenuPane* m_meshMenu;
	FXMenuPane* m_modelMenu;
	FXMenuPane* m_helpMenu;
	FXMenuPane* m_optionsMenu;
	FXMenuPane* m_importPlugins;
	FXMenuPane* m_exportPlugins;
	FXMenuPane* m_recentFilesMenu;
	FXMenuPane* m_preferencesMenu;
	dList<FXString> m_recentFilesList;
*/

	wxMenu* m_fileMenu;
	wxMenu* m_editMenu;
	wxMenu* m_helpMenu;

	wxMenu* m_importPlugins;
	wxMenu* m_exportPlugins;

	friend class NewtonModelEditor;
};


#endif