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

#ifndef __NEWTON_MODEL_EDITOR_H_
#define __NEWTON_MODEL_EDITOR_H_


//class GLVisual;
//class dPluginScene;
//class EditorCanvas;
class EditorMainMenu;
class EditorRenderViewport;
//class EditorExplorer;
//class EditorCommandPanel;

//#define D_MAX_PLUGINS_COUNT 128
//#define D_EDIT_MODE_ASSET "asset"
//#define D_EDIT_MODE_SCENE "scene"

class NewtonModelEditor: public wxFrame, public dPluginInterface
{
	public:
/*
	enum EditMode 
	{
		m_editAsset,
		m_editScene
	};

	enum NavigationMode
	{
		m_selectNode,
		m_translateNode,
		m_rotateNode,
		m_scaleNode,
		m_panViewport,
		m_moveViewport,
		m_zoomViewport,
		m_rotateViewport,
	};
*/
	enum 
	{
		ID_CANVAS = wxID_HIGHEST,

		// object selection modes
		ID_CURSOR_COMMAND_MODE,
		ID_SELECT_COMMAND_MODE,
		ID_TRANSLATE_COMMAND_MODE,
		ID_ROTATE_COMMAND_MODE,
		ID_SCALE_COMMAND_MODE,

		// editor navigation modes
		//ID_VIEWPORT_MAXIMIZE,
		ID_VIEWPORT_PANNING,
		ID_VIEWPORT_MOVE,
		ID_VIEWPORT_ROTATE,
		ID_VIEWPORT_ZOOM,

		// file menu
//		ID_NEW,
/*
		ID_EDITOR_MODE,
		ID_SET_VIEW_MODE,
		ID_SET_RENDER_MODE,

		// file menu
		ID_NEW,

		// scene menu
		ID_LOAD_SCENE,
		ID_SAVE_SCENE,
		ID_SAVE_SCENE_AS,

		// asset menu
		ID_LOAD_ASSET,
		ID_SAVE_ASSET,
		ID_SAVE_ASSET_AS,

		ID_ASSET_MANAGER,
		ID_SELECT_ASSET,
		

		// options menu
		ID_HIDE_FILE_TOOLBAR,
		ID_HIDE_NAVIGATION_TOOLBAR,
		ID_HIDE_EXPLORER_PANEL,
		ID_HIDE_COMMAND_PANEL,
		ID_KEYBOARD_SHORCUTS,

		// undo, redo
		ID_UNDO,
		ID_REDO,
		ID_CLEAR_UNDO,

		// help menu
		ID_ABOUT,



		// editor state
		ID_RECENT_FILES,
		ID_MAX_RECENT_FILES = ID_RECENT_FILES + 12,

		ID_MESH_PLUGINS,
		ID_MAX_MESH_PLUGINS = ID_MESH_PLUGINS + D_MAX_PLUGINS_COUNT,

		ID_MODEL_PLUGINS,
		ID_MAX_MODELS_PLUGINS = ID_MODEL_PLUGINS + D_MAX_PLUGINS_COUNT,

		ID_IMPORT_PLUGINS,
		ID_MAX_IMPORT_PLUGINS = ID_IMPORT_PLUGINS + D_MAX_PLUGINS_COUNT,

		ID_EXPORT_PLUGINS,
		ID_MAX_EXPORT_PLUGINS = ID_EXPORT_PLUGINS + D_MAX_PLUGINS_COUNT,
*/
	};

	NewtonModelEditor(const wxString& title, const wxPoint& pos, const wxSize& size);
	~NewtonModelEditor();

/*
	void Initilialize();
	void RefrehViewports();

	// GUI interface functions
	void create();

	bool IsAltDown() const;
	bool IsShiftDown() const;
	bool IsControlDown() const;
	
	long onKeyboardHandle(FXObject* sender, FXSelector id, void* eventPtr);
	
	long onEditorMode(FXObject* sender, FXSelector id, void* eventPtr); 

	long onPaint(FXObject* sender, FXSelector id, void* eventPtr);
//	long onClearViewports(FXObject* sender, FXSelector id, void* eventPtr);

	long onHideFileToolbar(FXObject* sender, FXSelector id, void* eventPtr);
	long onHideNavigationToolbar(FXObject* sender, FXSelector id, void* eventPtr);
	long onHideExplorerPanel(FXObject* sender, FXSelector id, void* eventPtr);
	long onHideCommandPanel(FXObject* sender, FXSelector id, void* eventPtr);

	long onNew(FXObject* sender, FXSelector id, void* eventPtr); 
	long onLoadScene(FXObject* sender, FXSelector id, void* eventPtr); 
	long onLoadRecentScene(FXObject* sender, FXSelector id, void* eventPtr); 

	long onLoadAsset(FXObject* sender, FXSelector id, void* eventPtr); 

	long onUndo(FXObject* sender, FXSelector id, void* eventPtr); 
	long onRedo(FXObject* sender, FXSelector id, void* eventPtr); 
	long onClearUndoHistory(FXObject* sender, FXSelector id, void* eventPtr); 

	long onSelectionCommandMode(FXObject* sender, FXSelector id, void* eventPtr); 

	long onImport(FXObject* sender, FXSelector id, void* eventPtr); 
	long onExport(FXObject* sender, FXSelector id, void* eventPtr); 

	long onMesh (FXObject* sender, FXSelector id, void* eventPtr); 
	long onModel (FXObject* sender, FXSelector id, void* eventPtr); 


	long onAssetSelected (FXObject* sender, FXSelector id, void* eventPtr); 
//	long onAssetDeslectd (FXObject* sender, FXSelector id, void* eventPtr); 


	void LoadIcon (const char* const iconName);
	FXIcon* FindIcon (const char* const iconName) const;
	static void* PhysicsAlloc (int sizeInBytes);
	static void PhysicsFree (void *ptr, int sizeInBytes);

	private:
	void CreateScene();
	void DestroyScene();
	
	void SaveConfig();
	void LoadConfig();
	void LoadPlugins(const char* const path);

	void LoadScene (const FXString& fileName);

	

	void PopNavigationMode();
	void PushNavigationMode(NavigationMode mode);
	void ShowNavigationMode(NavigationMode mode) const;


	EditorCanvas* GetCanvas(FXObject* const sender) const;

	FX4Splitter* m_workshop;
	EditorCanvas* m_canvas[4];

	FXStatusBar* m_statusbar;

	FXToolBarShell* m_mainMenuDragShell;
	EditorMainMenu* m_mainMenu;

	FXToolBarShell* m_fileToolbarShell;
	FXToolBar* m_fileToolbar;

	FXToolBarShell* m_navigationToolbarShell;
	FXToolBar* m_navigationToolbar;

	FXToolBarShell* m_explorerToolbarShell;
	EditorExplorer* m_explorer;

	FXToolBarShell* m_commandPanelToolbarShell;
	EditorCommandPanel* m_commandPanel;

	GLVisual* m_sharedVisual;
	FXDockSite* m_docks[4]; 
	
	
	
	
	
	EditMode m_editMode;
	bool m_altKey;
	bool m_shiftKey;
	bool m_controlKey;
	bool m_navigationKey;

	FXTextField* m_showNavigationMode;

	friend class EditorCanvas;
	friend class EditorMainMenu;
	friend class EditorFileToolBar;
	friend class EditorAssetExplorer;
	friend class EditorRenderViewport;
	friend class EditorNavigationToolBar;
*/

	protected:	
	DECLARE_EVENT_TABLE()

	void OnExit(wxCommandEvent& event);
	void OnNew(wxCommandEvent& event);
	void OnAbout(wxCommandEvent& event);


	void LoadResources ();
	void DeleteResources ();
	void CreateFileToolBar();
	void CreateRenderViewPort();
	void CreateNavigationToolBar();
	void CreateObjectSelectionToolBar();
	

	void LoadIcon (const char* const iconName);
	
	wxAuiManager m_mgr;
	wxMenuBar* m_mainMenu;
	wxStatusBar* m_statusBar;
	wxAuiToolBar* m_fileToolbar;
	wxAuiToolBar* m_navigationToolbar;
	wxAuiToolBar* m_objectSelectionToolbar;

	EditorRenderViewport* m_renderViewport;

	wxString m_lastFilePath;
	wxString m_currentFileName;
	dTree<wxBitmap*, dCRCTYPE> m_icons;
};



#endif