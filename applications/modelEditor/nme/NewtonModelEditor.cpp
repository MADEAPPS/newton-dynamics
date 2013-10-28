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

// NewtonModelEditor.cpp : Defines the entry point for the application.
//


#include "toolbox_stdafx.h"
#include "EditorExplorer.h"
#include "EditorMainMenu.h"
#include "NewtonModelEditor.h"
#include "EditorRenderViewport.h"
#include "NewtonModelEditorApp.h"


#define TOOLBAR_ICON_SIZE	32


typedef dPluginRecord** (CALLBACK* GetPluginArray)();

BEGIN_EVENT_TABLE (NewtonModelEditor, wxFrame)

	EVT_KEY_UP(OnKeyUp)	
	EVT_KEY_DOWN(OnKeyDown)

	EVT_MENU (wxID_EXIT, OnExit)
	EVT_MENU (wxID_ABOUT, OnAbout)
	EVT_MENU (wxID_HELP, OnAbout)
	EVT_MENU (wxID_PREFERENCES, OnAbout)

	EVT_MENU (wxID_OPEN, OnOpenScene)

	EVT_MENU (wxID_NEW, OnNew)
	EVT_MENU (ID_VIEWPORT_PANNING, OnChangeNavigationMode)
	EVT_MENU (ID_VIEWPORT_MOVE, OnChangeNavigationMode)
	EVT_MENU (ID_VIEWPORT_ROTATE, OnChangeNavigationMode)
	EVT_MENU (ID_VIEWPORT_ZOOM, OnChangeNavigationMode)

	EVT_CHOICE(ID_VIEW_MODES, OnChangeViewMode)  
	EVT_CHOICE(ID_SHADE_MODES, OnChangeShadeMode)  

	EVT_MENU (wxID_UNDO, OnUndo)
	EVT_MENU (wxID_REDO, OnRedo)
	EVT_MENU (ID_CLEAR_UNDO_HISTORY, OnClearUndoHistory)

	EVT_MENU (ID_HIDE_EXPLORER_PANE, OnHideExplorerPane)
	EVT_AUI_PANE_CLOSE(OnPaneClose)

	EVT_MENU_RANGE (ID_TOOL_PLUGINS, ID_MAX_TOOL_PLUGINS, OnTool)
	EVT_MENU_RANGE (ID_MESH_PLUGINS, ID_MAX_MESH_PLUGINS, OnMesh)
	EVT_MENU_RANGE (ID_IMPORT_PLUGINS, ID_MAX_IMPORT_PLUGINS, OnImport)
	EVT_MENU_RANGE (ID_EXPORT_PLUGINS, ID_MAX_EXPORT_PLUGINS, OnExport)

END_EVENT_TABLE()




NewtonModelEditor::NewtonModelEditor(const wxString& title, const wxPoint& pos, const wxSize& size)
	:wxFrame(NULL, -1, title, pos, size)
	,dPluginInterface()
	,m_mainMenu(NULL)
	,m_statusBar(NULL)
	,m_fileToolbar(NULL)
	,m_navigationToolbar(NULL)
	,m_objectSelectionToolbar(NULL)
	,m_renderViewport(NULL)
	,m_explorer(NULL)
	,m_viewMode(NULL)
	,m_shadeMode(NULL)
	,m_navigationStack(0)
{
	// notify wxAUI which frame to use
	m_mgr.SetManagedWindow(this);

	m_navigationMode[0] = m_panViewport;

//	dAssert (0);

	// Load all static resources
	LoadResources ();

	// add the main menu
	m_mainMenu = new EditorMainMenu(this);
	SetMenuBar (m_mainMenu);

	// create status bar for showing results 
	m_statusBar = CreateStatusBar();

	// create main Tool bars
	CreateFileToolBar();
	CreateObjectSelectionToolBar();
	CreateNavigationToolBar();
	CreateRenderViewPort();
	CreateExploser();
	CreateScene();

	// load configuration form last run
	//	LoadConfig();

	// load all plugins
	LoadPlugins("stdPlugins");
	LoadPlugins("plugins");

	// "commit" all changes made to wxAuiManager
	m_mgr.Update();


	// clean the explorer
	m_explorer->SelectItem (m_explorer->GetRootItem());
	dUndoRedoManager::Clear();
}

NewtonModelEditor::~NewtonModelEditor()
{
	DestroyScene();
	// Clean up the frame manager
	m_mgr.UnInit();

	DeleteResources();
}


void NewtonModelEditor::CreateScene()
{
	NewtonWorld* const world = NewtonCreate();

	// link the work with this user data
	NewtonWorldSetUserData(world, this);
	SetScene (new dPluginScene (world));

	m_explorer->Clear();
	m_explorer->ReconstructScene (GetScene());
}

void NewtonModelEditor::DestroyScene()
{
	Clear();
	NewtonWorld* const world = GetScene()->GetNewtonWorld();

	dPluginInterface::DestroyScene();
	NewtonWorldSetUserData(world, NULL);
	NewtonDestroy(world);
}



void NewtonModelEditor::LoadIcon (const char* const iconName)
{
	if (!m_icons.Find (dCRC64 (iconName))) {
		char appPath [2048];
		char fileName [2048];
		GetAplicationDirectory (appPath);

		sprintf (fileName, "%sicons/%s", appPath, iconName);
		wxBitmap* const bitmap = new wxBitmap(fileName, wxBITMAP_TYPE_GIF);
		wxImage image (bitmap->ConvertToImage());

		if (!image.IsTransparent (0, 0)) {
			unsigned char red = image.GetRed (0, 0);
			unsigned char green = image.GetGreen (0, 0);
			unsigned char blue = image.GetBlue (0, 0);
			unsigned char alpha = image.HasAlpha() ? image.GetAlpha (0, 0) : wxALPHA_TRANSPARENT;
			wxColour colour;
			colour.Set(red, green, blue, alpha);
			wxMask* mask = new wxMask (*bitmap, colour);
			bitmap->SetMask(mask);
		}
		m_icons.Insert (bitmap, dCRC64 (iconName));
	}
}

wxBitmap* NewtonModelEditor::FindIcon (const char* const iconName) const
{
	dAssert (m_icons.Find(dCRC64(iconName)));
	return m_icons.Find(dCRC64(iconName))->GetInfo();

}


void NewtonModelEditor::LoadResources ()
{
	wxInitAllImageHandlers();

	char path[2048];
	GetMediaDirectory (path);
	m_lastFilePath = path;
	m_currentFileName = "";

	LoadIcon ("fileNew.gif");
	LoadIcon ("fileOpen.gif");
	LoadIcon ("fileSave.gif");
	LoadIcon ("fileSaveAs.gif");

	LoadIcon ("undo.gif");
	LoadIcon ("redo.gif");
	LoadIcon ("cursor.gif");
	LoadIcon ("object_cursor.gif");
	LoadIcon ("object_move.gif");
	LoadIcon ("object_turn.gif");
	LoadIcon ("object_scale.gif");

	LoadIcon ("maximize.gif");
	LoadIcon ("camera_move.gif");
	LoadIcon ("camera_turn.gif");
	LoadIcon ("camera_zoom.gif");

	LoadIcon ("explorer.gif");
	LoadIcon ("sceneNode.gif");
	LoadIcon ("cache.gif");
	LoadIcon ("texture.gif");
	LoadIcon ("material.gif");
	LoadIcon ("geometry.gif");
	//	LoadIcon ("meshNode.gif");
	//	LoadIcon ("imageNode.gif");
}


void NewtonModelEditor::LoadPlugins(const char* const path)
{
	
	dPluginDll::dListNode* const firstNode = dPluginInterface::LoadPlugins(path);

	// dispatch plugins by type
	for (dPluginDll::dListNode* dllNode = firstNode; dllNode; dllNode = dllNode->GetNext()) {
		HMODULE module = dllNode->GetInfo();

		GetPluginArray GetPluginsTable = (GetPluginArray) GetProcAddress (module, "GetPluginArray"); 
		dPluginRecord** const table = GetPluginsTable();

		for (int i = 0; table[i]; i ++) {
			dPluginRecord* const plugin = table[i];

			switch (plugin->GetType())
			{
				case dPluginRecord::m_import:
				{
					//m_mainMenu->AddImportPlugin(plugin);
					m_mainMenu->AddPlugin(m_mainMenu->m_importPlugins, plugin);
					break;
				}

				case dPluginRecord::m_export:
				{
					m_mainMenu->AddPlugin(m_mainMenu->m_exportPlugins, plugin);
					break;
				}

				case dPluginRecord::m_model:
				{
					dAssert(0);
					//m_mainMenu->AddPlugin(m_mainMenu->m_modelMenu, plugin);
					break;
				}

				case dPluginRecord::m_tool:
				{
					m_mainMenu->AddPlugin(m_mainMenu->m_toolMenu, plugin);
					break;
				}


				case dPluginRecord::m_mesh:
				{
					m_mainMenu->AddPlugin(m_mainMenu->m_meshMenu, plugin);
					break;
				}

				default:
					dAssert (0);
			}
		}
	}
}



void NewtonModelEditor::DeleteResources ()
{
	dTree<wxBitmap*, dCRCTYPE>::Iterator iter (m_icons);
	for (iter.Begin(); iter; iter ++) {
		wxBitmap* const icon = iter.GetNode()->GetInfo();
		delete icon;
	}
}


void NewtonModelEditor::CreateFileToolBar()
{
	wxAuiToolBar* const toolbar = new wxAuiToolBar(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxAUI_TB_DEFAULT_STYLE | wxAUI_TB_OVERFLOW);
	toolbar->SetToolBitmapSize (wxSize(TOOLBAR_ICON_SIZE, TOOLBAR_ICON_SIZE));

	toolbar->AddTool (wxID_NEW, wxT("Create new scene"), *FindIcon("fileNew.gif"), wxT("Clear and create an empty scene"));

	toolbar->AddSeparator();
	toolbar->AddTool (wxID_OPEN, wxT("Open scene"), *FindIcon("fileOpen.gif"), wxT("Open an existing scene"));
	toolbar->AddTool (wxID_SAVE, wxT("Save scene"), *FindIcon("fileSave.gif"), wxT("Save current scene"));
	toolbar->AddTool (wxID_SAVEAS, wxT("Save scene as"), *FindIcon("fileSaveAs.gif"), wxT("Save current scene to different file"));

	toolbar->Realize();
	m_mgr.AddPane (toolbar, wxAuiPaneInfo(). Name(wxT("File Menu")).Caption(wxT("File menu")).ToolbarPane().Top());

	m_fileToolbar = toolbar;
}

void NewtonModelEditor::CreateObjectSelectionToolBar()
{
	wxAuiToolBar* const toolbar = new wxAuiToolBar(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxAUI_TB_DEFAULT_STYLE | wxAUI_TB_OVERFLOW);
	toolbar->SetToolBitmapSize (wxSize(TOOLBAR_ICON_SIZE, TOOLBAR_ICON_SIZE));

	toolbar->AddTool (wxID_UNDO, wxT("Undo"), *FindIcon("undo.gif"), wxT("Undo previous action"));
	toolbar->AddTool (wxID_REDO, wxT("Redo"), *FindIcon("redo.gif"), wxT("Redo previous action"));

	toolbar->AddSeparator();
	toolbar->AddTool (ID_CURSOR_COMMAND_MODE, wxT("Select cursor"), *FindIcon("cursor.gif"));
	toolbar->AddTool (ID_SELECT_COMMAND_MODE, wxT("Object selection mode"), *FindIcon("object_cursor.gif"));
	toolbar->AddTool (ID_TRANSLATE_COMMAND_MODE, wxT("Object translation mode"), *FindIcon("object_move.gif"));
	toolbar->AddTool (ID_ROTATE_COMMAND_MODE, wxT("Object rotation mode"), *FindIcon("object_turn.gif"));
	toolbar->AddTool (ID_SCALE_COMMAND_MODE, wxT("Object scale mode"), *FindIcon("object_scale.gif"));

	toolbar->Realize();
	m_mgr.AddPane (toolbar, wxAuiPaneInfo(). Name(wxT("Object selection")).Caption(wxT("Object options")).ToolbarPane().Top());
	m_objectSelectionToolbar = toolbar;
}


void NewtonModelEditor::CreateNavigationToolBar()
{
	wxAuiToolBar* const toolbar = new wxAuiToolBar(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxAUI_TB_DEFAULT_STYLE | wxAUI_TB_OVERFLOW);
	toolbar->SetToolBitmapSize (wxSize(TOOLBAR_ICON_SIZE, TOOLBAR_ICON_SIZE));

	toolbar->AddTool (ID_VIEWPORT_PANNING, wxT("pan veiwport"), *FindIcon("maximize.gif"));
	toolbar->AddTool (ID_VIEWPORT_MOVE, wxT("Translate Camera"), *FindIcon("camera_move.gif"));
	toolbar->AddTool (ID_VIEWPORT_ROTATE, wxT("Rotate Camera"), *FindIcon("camera_turn.gif"));
	toolbar->AddTool (ID_VIEWPORT_ZOOM, wxT("Rotate Camera"), *FindIcon("camera_zoom.gif"));

	m_viewMode = new wxChoice(toolbar, ID_VIEW_MODES);
	m_viewMode->AppendString(wxT("top"));
	m_viewModeMap[0] = EditorRenderViewport::m_top;

	m_viewMode->AppendString(wxT("front"));
	m_viewModeMap[1] = EditorRenderViewport::m_front;

	m_viewMode->AppendString(wxT("left"));
	m_viewModeMap[2] = EditorRenderViewport::m_left;

	m_viewMode->AppendString(wxT("perspective"));
	m_viewModeMap[3] = EditorRenderViewport::m_perpective;

	m_viewMode->AppendString(wxT("right"));
	m_viewModeMap[4] = EditorRenderViewport::m_right;

	m_viewMode->AppendString(wxT("bottom"));
	m_viewModeMap[5] = EditorRenderViewport::m_bottom;

	m_viewMode->AppendString(wxT("back"));
	m_viewModeMap[6] = EditorRenderViewport::m_back;

	m_viewMode->SetSelection (3);
	toolbar->AddControl(m_viewMode);

	m_shadeMode = new wxChoice(toolbar, ID_SHADE_MODES);
	m_shadeMode->AppendString(wxT("textured"));
	m_shapeModeMap[0] = EditorRenderViewport::m_textured;

	m_shadeMode->AppendString(wxT("solid"));
	m_shapeModeMap[1] = EditorRenderViewport::m_solid;

	m_shadeMode->AppendString(wxT("wireframe"));
	m_shapeModeMap[2] = EditorRenderViewport::m_wireframe;

	m_shadeMode->SetSelection (1);
	toolbar->AddControl(m_shadeMode);

	toolbar->Realize();
	m_mgr.AddPane (toolbar, wxAuiPaneInfo(). Name(wxT("Navigation options")).Caption(wxT("Navigation options")).ToolbarPane().Top());
	m_navigationToolbar = toolbar;
}


void NewtonModelEditor::CreateRenderViewPort()
{
	m_renderViewport = new EditorRenderViewport (this);
	m_mgr.AddPane (m_renderViewport, wxAuiPaneInfo().Name(wxT("render window")).CenterPane().PaneBorder(false));
}

void NewtonModelEditor::CreateExploser()
{
	m_explorer = new EditorExplorer (this);
	m_mgr.AddPane (m_explorer, wxAuiPaneInfo().Name(wxT("scene explorer")).Caption(wxT("scene explorer")).Left().Layer(1).Position(1).CloseButton(true).MaximizeButton(false));
}

int NewtonModelEditor::GetViewMode() const
{
	int index = m_viewMode->GetCurrentSelection();
	return m_viewModeMap[index];
}




void NewtonModelEditor::RefrehViewports()
{
	wxPaintEvent paint;
	GetEventHandler()->ProcessEvent (paint);
}



int NewtonModelEditor::GetShadeMode() const
{
	int index = m_shadeMode->GetCurrentSelection();
	return m_shapeModeMap[index];
}

int NewtonModelEditor::GetNavigationMode() const
{
	return m_navigationMode[m_navigationStack];
};




void NewtonModelEditor::OnExit(wxCommandEvent& event)
{
	Close ();
}

void NewtonModelEditor::OnKeyUp(wxKeyEvent &event)
{
	dAssert (0);
}


void NewtonModelEditor::OnKeyDown(wxKeyEvent &event)
{
	int keyCode = event.GetKeyCode();
	if (keyCode == WXK_ESCAPE)  {
		// send a display refresh event in case the runtime update is stopped bu the user.
		wxMenuEvent exitEvent (wxEVT_COMMAND_MENU_SELECTED, wxID_EXIT);
		GetEventHandler()->ProcessEvent(exitEvent);
	}

	dAssert (0);

/*
	if (!event.GetModifiers()) {
		int code = keyCode & 0xff; 
		m_key[m_keyMap[code]] = true;
	}
*/
}



void NewtonModelEditor::OnAbout(wxCommandEvent& event)
{
	wxString msg;
	msg.Printf(wxT ("%s %d.%02d"), APPLICATION_NAME, APPLICATION_VERSION / 100, APPLICATION_VERSION % 100);
	wxMessageBox(msg, wxT ("Newton Dynanics"), wxOK | wxICON_INFORMATION, this);
}


void NewtonModelEditor::OnNew (wxCommandEvent& event)
{
	Clear();
	DestroyScene();
	CreateScene();
	RefrehViewports();
}


void NewtonModelEditor::OnChangeViewMode(wxCommandEvent& event)
{
	RefrehViewports();
}

void NewtonModelEditor::OnChangeShadeMode(wxCommandEvent& event)
{
	RefrehViewports();
}


void NewtonModelEditor::OnUndo(wxCommandEvent& event)
{
	dUndoRedoManager::Undo();
	RefrehViewports();
}

void NewtonModelEditor::OnRedo(wxCommandEvent& event)
{
	dUndoRedoManager::Redo();
	RefrehViewports();
}

void NewtonModelEditor::OnClearUndoHistory(wxCommandEvent& event)
{
	dUndoRedoManager::Clear();
}



void NewtonModelEditor::OnHideExplorerPane (wxCommandEvent& event) 
{
	wxAuiPaneInfo& pane = m_mgr.GetPane(m_explorer);
	if (event.GetInt()) {
		pane.Show(true);
	} else {
		pane.Show(false);
	}
	m_mgr.Update();
}

void NewtonModelEditor::OnPaneClose (wxAuiManagerEvent& event)
{
//	wxCommandEvent notify(wxEVT_COMMAND_MENU_SELECTED, ID_HIDE_EXPLORER_PANE);
//	notify.SetEventObject(this);
//	notify.SetInt (0);
//	GetEventHandler()->ProcessEvent (notify);
	wxMenuItem* const item = m_mainMenu->m_viewMenu->FindItem (m_mainMenu->m_viewMenu->FindItem (event.pane->name));
	item->Check(false);
}


void NewtonModelEditor::OnChangeNavigationMode(wxCommandEvent& event)
{
	switch (event.GetId())
	{
		case ID_VIEWPORT_PANNING:
		{
			m_navigationStack = 0;
			m_navigationMode[0] = NewtonModelEditor::m_panViewport;
			break;
		}

		case ID_VIEWPORT_MOVE:
		{
			m_navigationStack = 0;
			m_navigationMode[0] = NewtonModelEditor::m_moveViewport;
			break;
		}

		case ID_VIEWPORT_ROTATE:
		{
			m_navigationStack = 0;
			m_navigationMode[0] = NewtonModelEditor::m_rotateViewport;
			break;
		}

		case ID_VIEWPORT_ZOOM:
		{
			m_navigationStack = 0;
			m_navigationMode[0] = NewtonModelEditor::m_zoomViewport;
			break;
		}


		default:
		{
			dAssert(0);
		}
	}

	RefrehViewports();
	//m_mainFrame->ShowNavigationMode(m_navigationMode[0]);
}


void NewtonModelEditor::OnMesh (wxCommandEvent& event)
{
	int id = event.GetId() - ID_MESH_PLUGINS;

	dPluginMesh* const plugin = (dPluginMesh*) m_mainMenu->GetPlugin(m_mainMenu->m_meshMenu, id);
	_ASSERTE (plugin);

	dPluginScene* const asset = plugin->Create (this);
	if (asset) {
		MergeScene (asset);
		asset->Release();
		m_explorer->ReconstructScene(GetScene());
		RefrehViewports();
	}
}

void NewtonModelEditor::OnTool (wxCommandEvent& event)
{
	int id = event.GetId() - ID_TOOL_PLUGINS;

	dPluginTool* const plugin = (dPluginTool*) m_mainMenu->GetPlugin(m_mainMenu->m_toolMenu, id);
	_ASSERTE (plugin);
	plugin->Execute(this);
}


void NewtonModelEditor::OnOpenScene(wxCommandEvent& event)
{
	wxFileDialog open (this, wxT("Load Newton Dynamics Scene"), wxT("../../../media"), wxT(""), wxT("*.ngd"));
	if (open.ShowModal() == wxID_OK) {
		// link the work with this user data
		NewtonWorld* const world = GetScene()->GetNewtonWorld();
		dPluginScene* const asset = new dPluginScene (world);
		asset->Cleanup();
		m_lastFilePath = open.GetPath();
		if (asset->Deserialize (m_lastFilePath.mb_str())) {
			MergeScene (asset);
			m_explorer->ReconstructScene(GetScene());
			RefrehViewports();
		}
		asset->Release();
	}
}

void NewtonModelEditor::OnImport (wxCommandEvent& event)
{
	int id = event.GetId() - ID_IMPORT_PLUGINS;
	dImportPlugin* const plugin = (dImportPlugin*) m_mainMenu->GetPlugin(m_mainMenu->m_importPlugins, id);
	dAssert (plugin);
	wxFileDialog open (this, wxT(plugin->GetFileDescription ()), wxT("../../../media"), wxT(""), wxT(plugin->GetFileExtension ()));
	if (open.ShowModal() == wxID_OK) {
		if (plugin->Import (open.GetPath().mb_str(), this)) {
			m_explorer->ReconstructScene(GetScene());
			RefrehViewports();
		}
	}
}

void NewtonModelEditor::OnExport (wxCommandEvent& event)
{
	int id = event.GetId() - ID_EXPORT_PLUGINS;
	dExportPlugin* const plugin = (dExportPlugin*) m_mainMenu->GetPlugin(m_mainMenu->m_exportPlugins, id);
	dAssert (plugin);
	if (HasMeshSelection (dMeshNodeInfo::GetRttiType())) {
		wxFileDialog open (this, wxT(plugin->GetFileDescription ()), wxT("../../../media"), wxT(""), wxT(plugin->GetFileExtension ()));
		if (open.ShowModal() == wxID_OK) {
			plugin->Export (open.GetPath().mb_str(), this);
		}
	}
}


void NewtonModelEditor::RefreshExplorerEvent(bool clear) const
{
	dPluginInterface::RefreshExplorerEvent(clear);

	if (clear) {
		m_explorer->Clear();
	}
	m_explorer->ReconstructScene (GetScene());
}


EditorExplorer* NewtonModelEditor::GetExplorer() const
{
	return m_explorer;
}