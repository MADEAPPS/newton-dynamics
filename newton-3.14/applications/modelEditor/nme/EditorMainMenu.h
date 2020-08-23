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

#ifndef _EDITOR_MAIN_MENU_H_
#define _EDITOR_MAIN_MENU_H_


#include <dList.h>

class NewtonModelEditor;


class EditorMainMenu: public wxMenuBar
{
	public:
	class BasePluginBaseMenuId: public wxObjectRefData
	{
		public: 
		BasePluginBaseMenuId (int baseID)
			:wxObjectRefData()
			,m_baseID(baseID)
		{
		}

		int m_baseID;				
	};

	class EditorPlugin: public wxObjectRefData
	{
		public: 
		EditorPlugin (dPluginRecord* const plugin)
			:wxObjectRefData()
			,m_plugin(plugin)
		{
		}

		dPluginRecord* const m_plugin;
	};

	class EditorViewControl: public wxObjectRefData
	{
		public: 
		EditorViewControl (wxWindow* const control)
			:wxObjectRefData()
			,m_control(control)
		{
		}
		wxWindow* m_control;
	};

	EditorMainMenu (NewtonModelEditor* const parent);
	~EditorMainMenu(void);

	private:
	void CreateFileMenu();
	void CreateEditMenu();
	void CreateViewMenu();
	void CreateToolMenu();
	void CreateMeshMenu();
	void CreateHelpMenu();

	void AddPlugin (wxMenu* const menu, dPluginRecord* const plugin);
	dPluginRecord* GetPlugin (wxMenu* const paneMenu, int id);

	wxWindow* GetViewControl (int controlId) const;
	void AddViewControl (int controlId, const char* const title, wxWindow* const control);

	NewtonModelEditor* m_mainFrame;
	wxMenu* m_fileMenu;
	wxMenu* m_editMenu;
	wxMenu* m_viewMenu;
	wxMenu* m_toolMenu;
	wxMenu* m_meshMenu;
	wxMenu* m_helpMenu;

	wxMenu* m_importPlugins;
	wxMenu* m_exportPlugins;

	friend class NewtonModelEditor;
};


#endif