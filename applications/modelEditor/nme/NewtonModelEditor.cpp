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

	EVT_MENU_RANGE (ID_MESH_PLUGINS, ID_MAX_MESH_PLUGINS, OnMesh)



END_EVENT_TABLE()


#if 0
int main(int argc, char *argv[])
{
	// Enable run-time memory check for debug builds.
	FXApp application("Newton Dynamics demos", "Newton Model Editor");

	// Set the memory allocation function before creation the newton world
	// this is the only function that can be called before the creation of the newton world.
	// it should be called once, and the the call is optional 
	NewtonSetMemorySystem (NewtonModelEditor::PhysicsAlloc, NewtonModelEditor::PhysicsFree);

	application.init(argc,argv);

	// Make main window
	NewtonModelEditor* const mainWindow = new NewtonModelEditor (application);


	// Create the application's windows
	application.create();

	// Initialize the application with after all windows are ready
	mainWindow->Initilialize();
	
	// Run the application
	return application.run();
}

bool NewtonModelEditor::IsControlDown() const
{
	return m_controlKey;
}

bool NewtonModelEditor::IsAltDown() const
{
	return m_altKey;
}

bool NewtonModelEditor::IsShiftDown() const
{
	return m_shiftKey;
}

void NewtonModelEditor::SaveConfig()
{
}

void NewtonModelEditor::LoadConfig()
{
}

// memory allocation for Newton
void* NewtonModelEditor::PhysicsAlloc (int sizeInBytes)
{
	m_totalMemoryUsed += sizeInBytes;
	return new char [sizeInBytes];
}

// memory free use by the engine
void NewtonModelEditor::PhysicsFree (void* ptr, int sizeInBytes)
{
	m_totalMemoryUsed -= sizeInBytes;

	delete[] ptr;
}


FXIcon* NewtonModelEditor::FindIcon (const char* const iconName) const
{
	dTree<FXIcon*, dCRCTYPE>::dTreeNode* const node = m_icons.Find(dCRC64 (iconName));
	_ASSERTE (node);
	return node->GetInfo();
}



void NewtonModelEditor::ShowNavigationMode(NavigationMode mode) const
{ 
	switch (mode) 
	{
		case m_selectNode:
			m_showNavigationMode->setText("select");
			break;

		case m_translateNode:
			m_showNavigationMode->setText("move");
			break;

		case m_rotateNode:
			m_showNavigationMode->setText("rotate");
			break;

		case m_scaleNode:
			m_showNavigationMode->setText("scale");
			break;

		case m_panViewport:
			m_showNavigationMode->setText("cam pan");
			break;

		case m_moveViewport:
			m_showNavigationMode->setText("cam move");
			break;

		case m_zoomViewport:
			m_showNavigationMode->setText("cam zoom");
			break;

		case m_rotateViewport:
			m_showNavigationMode->setText("cam rotate");
			break;
	}
}


void NewtonModelEditor::PushNavigationMode(NavigationMode mode)
{
	if (!m_navigationKey) {

		m_navigationKey = true;
		for (int i = 0; i < sizeof (m_canvas) / sizeof (m_canvas[0]); i ++) {
			m_canvas[i]->m_navigationStack += 1;
			m_canvas[i]->m_navigationMode[m_canvas[i]->m_navigationStack] = mode;
			ShowNavigationMode(mode);
		}
	}
}

void NewtonModelEditor::PopNavigationMode()
{
	if (m_navigationKey) {

		m_navigationKey = false;
		for (int i = 0; i < sizeof (m_canvas) / sizeof (m_canvas[0]); i ++) {
			_ASSERTE (m_canvas[i]->m_navigationStack < 5);
			m_canvas[i]->m_navigationStack = (m_canvas[i]->m_navigationStack > 0) ? m_canvas[i]->m_navigationStack - 1 : 0;
			ShowNavigationMode(m_canvas[i]->m_navigationMode[m_canvas[i]->m_navigationStack]);
		}
		
	}
}



long NewtonModelEditor::onEditorMode(FXObject* sender, FXSelector id, void* eventPtr)
{
	FXTabBook* const tabBook = (FXTabBook*) sender;

	int index = int(eventPtr);
	FXLabel* const child = (FXLabel*)tabBook->childAtIndex(index*2);

	if (!stricmp (child->getText().text(), D_EDIT_MODE_ASSET)) {
		m_editMode = m_editAsset;
	} else {
		m_editMode = m_editScene;
	}

	RefrehViewports();
	return 1;
}

long NewtonModelEditor::onKeyboardHandle(FXObject* sender, FXSelector id, void* eventPtr)
{
	FXEvent* const event=(FXEvent*)eventPtr;
	int selType = FXSELTYPE(id);

	switch (selType) 
	{
		case SEL_KEYPRESS:
		{
			switch (event->code) 
			{
				case KEY_Escape:
					getApp()->addChore(this, ID_CLOSE);
					break;
				
				case KEY_Control_L:
				case KEY_Control_R:
					m_controlKey = true;
					break;

				case KEY_Alt_L:
				case KEY_Alt_R:
					m_altKey = true;
					break;

				case KEY_Shift_L:
				case KEY_Shift_R:
					m_shiftKey = true;
					break;

				case KEY_c:
				case KEY_C:
					PushNavigationMode(m_rotateViewport);
					break;

				case KEY_x:
				case KEY_X:
					PushNavigationMode(m_moveViewport);

				case KEY_v:
				case KEY_V:
					PushNavigationMode(m_panViewport);

				case KEY_z:
				case KEY_Z:
					PushNavigationMode(m_zoomViewport);
					break;

			}
			break;
		}

		case SEL_KEYRELEASE:
		{
			switch (event->code) 
			{
				case KEY_Control_L:
				case KEY_Control_R:
					m_controlKey = false;
					break;

				case KEY_Alt_L:
				case KEY_Alt_R:
					m_altKey = false;
					break;

				case KEY_Shift_L:
				case KEY_Shift_R:
					m_shiftKey = false;
					break;


				case KEY_c:
				case KEY_C:
				case KEY_z:
				case KEY_Z:
				case KEY_x:
				case KEY_X:
				case KEY_v:
				case KEY_V:
					PopNavigationMode();
					break;
			}
			break;
		}
	}

	return 0;
}




long NewtonModelEditor::onHideFileToolbar(FXObject* sender, FXSelector id, void* eventPtr)
{
	FXuval val = FXuval (eventPtr);

	if (val) {
		((EditorFileToolBar*)m_fileToolbar)->Hide();
	} else {
		((EditorFileToolBar*)m_fileToolbar)->Unhide();
	}
	return 1;
}

long NewtonModelEditor::onHideNavigationToolbar(FXObject* sender, FXSelector id, void* eventPtr)
{
	FXuval val = FXuval (eventPtr);

	if (val) {
		((EditorFileToolBar*)m_objectSelectionToolbar)->Hide();
	} else {
		((EditorFileToolBar*)m_objectSelectionToolbar)->Unhide();
	}
	return 1;
}


long NewtonModelEditor::onHideCommandPanel(FXObject* sender, FXSelector id, void* eventPtr)
{
	FXuval val = FXuval (eventPtr);
	if (val) {
		m_commandPanel->Hide();
	} else {
		m_commandPanel->Unhide();
	}
	return 1;
}

EditorCanvas* NewtonModelEditor::GetCanvas(FXObject* const sender) const
{
	return  (EditorCanvas*)(((FXWindow*) sender)->getParent()->getParent());
}

long NewtonModelEditor::onExport(FXObject* sender, FXSelector id, void* eventPtr)
{
//	dExportPlugin* const importer = (dExportPlugin*)m_mainMenu->GetPlugin(m_mainMenu->m_exportPlugins, FXSELID(id)-ID_EXPORT_PLUGINS);
	_ASSERTE (0);
	return 1;
}






long NewtonModelEditor::onSelectionCommandMode(FXObject* sender, FXSelector id, void* eventPtr)
{
	int messageType = FXSELID(id);

	NavigationMode mode = m_selectNode;
	switch (messageType) 
	{
		case ID_SELECT_COMMAND_MODE:
			mode = m_selectNode;
			break;

		case ID_TRANSLATE_COMMAND_MODE:
			mode = m_translateNode;
			break;

		case ID_ROTATE_COMMAND_MODE:
			mode = m_rotateNode;
			break;

		case ID_SCALE_COMMAND_MODE:
			mode = m_scaleNode;
			break;
	}

	for (int i = 0; i < sizeof (m_canvas) / sizeof (m_canvas[0]); i ++) {
		m_canvas[i]->m_navigationStack = 0;
		m_canvas[i]->m_navigationMode[0] = mode;
	}

	ShowNavigationMode(mode);
	return 1;

}

long NewtonModelEditor::onImport(FXObject* sender, FXSelector id, void* eventPtr)
{
	dImportPlugin* const importer = (dImportPlugin*)m_mainMenu->GetPlugin(m_mainMenu->m_importPlugins, FXSELID(id)-ID_IMPORT_PLUGINS);
	_ASSERTE (importer);

	char patterns[2048];
	char title[2048];
	sprintf (title, "Importing %s", importer->GetFileDescription());
	sprintf (patterns, "%s (*%s)", importer->GetFileDescription(), importer->GetFileExtension());
	FXFileDialog open(this, title);
	open.setPatternList(patterns);
	open.setDirectory (m_lastFilePath);
	if(open.execute()){
		importer->Import (open.getFilename().text(), this);
		_ASSERTE (0);
//		m_explorer->Populate (m_scene);
	}
	return 1;
}

long NewtonModelEditor::onModel (FXObject* sender, FXSelector id, void* eventPtr)
{

	dModelPlugin* const plugin = (dModelPlugin*)m_mainMenu->GetPlugin(m_mainMenu->m_modelMenu, FXSELID(id)-ID_MODEL_PLUGINS);
	_ASSERTE (plugin);

	_ASSERTE (0);
	plugin->Create (this);

	return 1;
}






long NewtonModelEditor::onAssetSelected (FXObject* sender, FXSelector id, void* eventPtr)
{
//	m_explorer->PopulateCurrentAsset ();
//	Push(new dUndoAssetCache(this));
//	m_explorer->RefreshAllViewer();
//	m_explorer->PopulateCurrentAsset ();
	m_explorer->SetBrowserSelection ();
	RefrehViewports();
	return 1;
}


long NewtonModelEditor::onPaint(FXObject* sender, FXSelector id, void* eventPtr)
{
	for (int i = 0; i < 4; i ++) {
		EditorCanvas* const canvas = m_canvas[i];
		if (canvas->shown()) {
			canvas->UpdateViewport();
		}
	}

	return 1;
}
#endif









NewtonModelEditor::NewtonModelEditor(const wxString& title, const wxPoint& pos, const wxSize& size)
	:wxFrame(NULL, -1, title, pos, size)
	,dPluginInterface()
//	,m_workshop(NULL)
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
//	,m_commandPanel(NULL)
//	,m_sharedVisual(NULL)
//	,m_editMode(m_editAsset)
//	,m_altKey(false)
//	,m_shiftKey(false)
//	,m_controlKey(false)
//	,m_navigationKey(false)
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

	// "commit" all changes made to wxAuiManager
	m_mgr.Update();

	// create the scene
	Initilialize();
}

NewtonModelEditor::~NewtonModelEditor()
{
	DestroyScene();
	// Clean up the frame manager
	m_mgr.UnInit();

	DeleteResources();
}

void NewtonModelEditor::Initilialize()
{
	// load configuration form last run
//	LoadConfig();

	// load all plugins
	LoadPlugins("stdPlugins");
	LoadPlugins("plugins");

	// create the scene
	CreateScene();
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

	GetScene()->Release();
//	RemoveAllAsset();
//	m_explorer->RefreshAllViewer();

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
					dAssert(0);
					//m_mainMenu->AddPlugin(m_mainMenu->m_exportPlugins, plugin);
					break;
				}

				case dPluginRecord::m_model:
				{
					dAssert(0);
					//m_mainMenu->AddPlugin(m_mainMenu->m_modelMenu, plugin);
					break;
				}

				case dPluginRecord::m_mesh:
				{
					m_mainMenu->AddPlugin(m_mainMenu->m_meshMenu, plugin);
					break;
				}
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

	toolbar->AddTool (wxID_NEW, wxT("Create new scene"), *FindIcon("fileNew.gif"));

	toolbar->AddSeparator();
	toolbar->AddTool (wxID_OPEN, wxT("Open scene"), *FindIcon("fileOpen.gif"));
	toolbar->AddTool (wxID_SAVE, wxT("Save scene"), *FindIcon("fileSave.gif"));
	toolbar->AddTool (wxID_SAVEAS, wxT("Save scene as"), *FindIcon("fileSaveAs.gif"));

	toolbar->Realize();
	m_mgr.AddPane (toolbar, wxAuiPaneInfo(). Name(wxT("File Menu")).Caption(wxT("File menu")).ToolbarPane().Top());

	m_fileToolbar = toolbar;
}

void NewtonModelEditor::CreateObjectSelectionToolBar()
{
	wxAuiToolBar* const toolbar = new wxAuiToolBar(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxAUI_TB_DEFAULT_STYLE | wxAUI_TB_OVERFLOW);
	toolbar->SetToolBitmapSize (wxSize(TOOLBAR_ICON_SIZE, TOOLBAR_ICON_SIZE));

	toolbar->AddTool (wxID_UNDO, wxT("Undo previous action"), *FindIcon("undo.gif"));
	toolbar->AddTool (wxID_REDO, wxT("Redo previous action"), *FindIcon("redo.gif"));

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


void NewtonModelEditor::LoadScene (const char* const fileName)
{
//	m_currentFileName = fileName;

	GetScene()->Cleanup();

		// load the scene from and alchemedia file format
//	GetScene()->makeCurrent();
	GetScene()->Deserialize (fileName);
//	GetScene()->makeNonCurrent();
	ClearSelection();

/*
	// place camera into position
	dMatrix camMatrix (GetIdentityMatrix());
	camMatrix = dYawMatrix(-0.0f * 3.1416f / 180.0f);
	camMatrix.m_posit = dVector (-5.0f, 1.0f, -0.0f, 0.0f);
	GetScene()->SetCameraMatrix(camMatrix, camMatrix.m_posit);

	RestoreSettings ();
*/

	RefrehViewports();
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

void NewtonModelEditor::OnMesh (wxCommandEvent& event)
{
	int id = event.GetId() - ID_MESH_PLUGINS;

	dPluginMesh* const plugin = (dPluginMesh*) m_mainMenu->GetPlugin(m_mainMenu->m_meshMenu, id);
	_ASSERTE (plugin);

	dPluginScene* const asset = plugin->Create (this);
	if (asset) {
		GetScene()->MergeScene(this, asset);
		asset->Release();
		RefrehViewports();
		m_explorer->ReconstructScene(GetScene());
	}
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

/*
void NewtonModelEditor::OnOpenScene(wxCommandEvent& event)
{
	const FXchar patterns[]="Newton Dynamics Files (*.ngd)";

	FXFileDialog open(this,"Load Newton Dynamics scene");
	open.setPatternList(patterns);
	open.setDirectory (m_lastFilePath);
	if(open.execute()){
		onNew (sender, id, eventPtr);
		m_lastFilePath = open.getDirectory();
		m_filePathFile = m_lastFilePath.text();
		m_mainMenu->AddRecentFile(open.getFilename());
		LoadScene (open.getFilename());
	}
}
*/

void NewtonModelEditor::OnOpenScene(wxCommandEvent& event)
{
	wxFileDialog open (this, wxT("Load Newton Dynamics Scene"), wxT("../../../media"), wxT(""), wxT("*.ngd"));
	if (open.ShowModal() == wxID_OK) {
		OnNew (event);
		m_lastFilePath = open.GetPath();
		LoadScene (m_lastFilePath.mb_str());
		m_explorer->ReconstructScene(GetScene());
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
