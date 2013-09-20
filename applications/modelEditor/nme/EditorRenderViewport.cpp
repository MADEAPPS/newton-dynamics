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
//#include "EditorCanvas.h"
//#include "EditorExplorer.h"
#include "NewtonModelEditor.h"
//#include "EditorAssetExplorer.h"
#include "EditorRenderViewport.h"


int EditorRenderViewport::m_attributes[] = {WX_GL_DOUBLEBUFFER, WX_GL_RGBA, WX_GL_DEPTH_SIZE, 32, 0};


BEGIN_EVENT_TABLE (EditorRenderViewport, wxGLCanvas)

	EVT_SIZE(EditorRenderViewport::OnSize)
	EVT_PAINT(EditorRenderViewport::OnPaint)
	EVT_IDLE(EditorRenderViewport::OnIdle)
	EVT_ERASE_BACKGROUND (EditorRenderViewport::OnEraseBackground)
END_EVENT_TABLE()

/*
FXDEFMAP(EditorRenderViewport) MessageMap[]=
{
	FXMAPFUNC(SEL_LEFTBUTTONPRESS,		0,	EditorRenderViewport::onLeftMouseKeyDown),
	FXMAPFUNC(SEL_LEFTBUTTONRELEASE,	0,	EditorRenderViewport::onLeftMouseKeyUp),
	FXMAPFUNC(SEL_LEAVE,                0,	EditorRenderViewport::onLeftMouseKeyUp),

	FXMAPFUNC(SEL_MOTION,		0,			EditorRenderViewport::onMouseMove),

};
FXIMPLEMENT(EditorRenderViewport,FXGLCanvas,MessageMap,ARRAYNUMBER(MessageMap))




EditorRenderViewport::EditorRenderViewport()
{
}



void EditorRenderViewport::create()
{
	FXGLCanvas::create();

	makeCurrent();
	GLenum err = glewInit();
	if (err == GLEW_OK) {
		#ifdef WIN32
			wglSwapIntervalEXT(0);
		#else
			// NOTE: check for GLX_SGI_swap_control extension : http://www.opengl.org/wiki/Swap_Interval#In_Linux_.2F_GLX
			glXSwapIntervalSGI(0); 
		#endif
	}

	// create a fond for print in 3d window
//	m_font = m_render->CreateDisplayList(96);
//	HFONT font = CreateFont(
//		20,                        // nHeight
//		0,                         // nWidth
//		0,                         // nEscapement
//		0,                         // nOrientation
//		FW_BOLD,                   // nWeight
//		FALSE,                     // bItalic
//		FALSE,                     // bUnderline
//		0,                         // cStrikeOut
//		ANSI_CHARSET,              // nCharSet
//		OUT_DEFAULT_PRECIS,        // nOutPrecision
//		CLIP_DEFAULT_PRECIS,       // nClipPrecision
//		DEFAULT_QUALITY,           // nQuality
//		DEFAULT_PITCH | FF_SWISS,  // nPitchAndFamily
//		_T("Arial"));              // lpszFacename
//	HFONT oldfont = (HFONT)SelectObject (m_hDC, font);	
//	wglUseFontBitmaps(m_hDC, 32, 96, m_font);				
//	SelectObject(m_hDC, oldfont);						
//	DeleteObject(font);									
//	return displayLists;

	m_font = m_render->CreateDisplayList(96);
	FXFont* const font = new FXFont(getApp(), "areal");
	font->create();
	glUseFXFont(font, ' ', 96, m_font);
	delete font;

	// build the camera grid
	//BuildConstructionGrid(m_render, 20, 4.0f);
	BuildConstructionGrid(m_render, 20, 1.0f);

	makeNonCurrent();
}








void EditorRenderViewport::UpdateScene (dViewPortModes mode)
{
	BeginRender(mode);

	_ASSERTE (0);


	EndRender();
}




long EditorRenderViewport::onMouseMove(FXObject* sender, FXSelector id, void* eventPtr)
{
	m_mainFrame->ShowNavigationMode(m_canvas->m_navigationMode[m_canvas->m_navigationStack]);

	if (m_leftMouseKeyState) {
		FXEvent* const event = (FXEvent*)eventPtr;

		switch (m_canvas->m_navigationMode[m_canvas->m_navigationStack]) 
		{
			case NewtonModelEditor::m_panViewport:
			{
				dFloat x;
				dFloat y;
				GetPanning(x, y);
				x += (event->win_x - event->last_x);
				y -= (event->win_y - event->last_y);
				SetPanning(x, y);
				//DTRACE (("%d %d\n", event->win_x - event->last_x, event->win_y - event->last_y));
				break;
			}

			case NewtonModelEditor::m_moveViewport:
			{
				dFloat sensitivity = 0.05f;
				int step = event->win_y - event->last_y;
				dFloat factor = step ? (step < 0 ? 1.0f + sensitivity : 1.0f - sensitivity) : 1.0f;
				SetMatrix(GetYawAngle(), GetRollAngle(), GetDistance() * factor);
				break;
			}

			
			case NewtonModelEditor::m_rotateViewport:
			{
				dFloat yaw = GetYawAngle();
				dFloat roll = GetRollAngle();

				dFloat sensitivity = 0.5f * 3.141692f / 180.0f;

				int stepX = event->win_x - event->last_x;
				dFloat deltaYaw = stepX ? (stepX < 0 ? sensitivity : -sensitivity) : 0;
				
				int stepY = event->win_y - event->last_y;
				dFloat deltaRoll = stepY ? (stepY < 0 ? sensitivity: -sensitivity) : 0;

				SetMatrix(yaw + deltaYaw, roll + deltaRoll, GetDistance());
				break;
			}


			case NewtonModelEditor::m_zoomViewport:
			{
				if (event->win_y - event->last_y) {
					dFloat zoomVal = GetZoom();
					dFloat zoomFactor = 1.0 / 64.0f;
					zoomVal *= ((event->win_y - event->last_y) > 0) ? (1.0f + zoomFactor): (1.0f - zoomFactor);
					SetZoom(zoomVal);
				}
				break;
			}

			case NewtonModelEditor::m_selectNode:
			case NewtonModelEditor::m_translateNode:
			case NewtonModelEditor::m_rotateNode:
			case NewtonModelEditor::m_scaleNode:
			{
				if (m_mainFrame->IsControlDown() && (m_mainFrame->IsShiftDown() || m_mainFrame->IsAltDown())) {
					SelectAssetNode (event);
				}
			}
		}

		m_mainFrame->RefrehViewports();
	}
	return 1;
}



long EditorRenderViewport::onLeftMouseKeyUp(FXObject* sender, FXSelector id, void* eventPtr)
{
	m_leftMouseKeyState = false;
	return 1;
}


long EditorRenderViewport::onLeftMouseKeyDown(FXObject* sender, FXSelector id, void* eventPtr)
{
	m_leftMouseKeyState = true;


	switch (m_canvas->m_navigationMode[m_canvas->m_navigationStack])  
	{
		case NewtonModelEditor::m_scaleNode:
		case NewtonModelEditor::m_selectNode:
		case NewtonModelEditor::m_rotateNode:
		case NewtonModelEditor::m_translateNode:
		{
			const FXEvent* const event = (FXEvent*)eventPtr;
			switch (m_mainFrame->m_editMode) 
			{
				case NewtonModelEditor::m_editAsset:
				{
					SelectAssetNode (event);
					break;
				}

				case NewtonModelEditor::m_editScene:
				{
					_ASSERTE (0);
					break;
				}
			}
			break;
		}
	}
	return 1;
}
	
void EditorRenderViewport::SelectAssetNode (const FXEvent* const event)
{
	dPluginScene* const asset = m_mainFrame->GetAsset();		
	if (asset) {

		makeCurrent();	
		dVector p0 (m_render->ScreenToGlobal(dVector (event->win_x, event->win_y, 0.0f, 1.0f)));
		dVector p1 (m_render->ScreenToGlobal(dVector (event->win_x, event->win_y, 1.0f, 1.0f)));
		makeNonCurrent();

		dList<dScene::dTreeNode*> trace;
		dFloat hit = asset->RayCast (p0, p1, trace);
		if (hit < 1.0f) {
			m_mainFrame->m_explorer->HandleSelectionEvent (trace);
		}
	}
}



void EditorRenderViewport::UpdateAsset (dViewPortModes mode)
{
	BeginRender(mode);

	dPluginScene* const asset = m_mainFrame->GetAsset();		

	if (asset) {
		//asset->RenderWireframe(m_render);
		//asset->RenderFlatShaded(m_render);
		asset->RenderSolidWireframe(m_render);

		asset->RenderWireframeSelection (m_render, m_mainFrame);

		// draw the gizmo for the selections
		RenderSelectedNodeGizmo();

	}
	EndRender();
}

void EditorRenderViewport::RenderSelectedNodeGizmo () const
{
	dPluginScene* const scene = m_mainFrame->GetAsset();		
	EditorAssetExplorer* const assetBrowser = m_mainFrame->m_explorer->m_assetExplorer;

	for (void* link = m_mainFrame->GetFirtSelectedNode(); link; link = m_mainFrame->GetNextSelectedNode(link)) {
		FXTreeItem* item = assetBrowser->findItemByData(link);
		_ASSERTE (item);
		dMatrix globalMatrix (GetIdentityMatrix());
		while (item) {
			void* const link = item->getData();
			if (link) {
				dScene::dTreeNode* const sceneNode = scene->GetNodeFromLink (link);
				if (sceneNode) {
					dSceneNodeInfo* const sceneInfo = (dSceneNodeInfo*) scene->GetInfoFromNode(sceneNode);
					if (sceneInfo->IsType(dSceneNodeInfo::GetRttiType())) {
						globalMatrix = globalMatrix * sceneInfo->GetTransform();
					}
				}
			}
			item = item->getParent();
		}
		dMatrix axis;
		dVector scale;
		dMatrix stretchAxis;
		globalMatrix.PolarDecomposition (axis, scale, stretchAxis);
		//DrawNodeSelectionGizmo(m_render, axis);
		DrawNodeSelectAndMoveGizmo(m_render, axis);
	}
}

*/


EditorRenderViewport::EditorRenderViewport (NewtonModelEditor* const mainFrame)
	:wxGLCanvas (mainFrame, wxID_ANY, wxDefaultPosition, wxSize (300, 300), wxSUNKEN_BORDER|wxFULL_REPAINT_ON_RESIZE, _("GLRenderCanvas"), m_attributes)
	,dPluginCamera()
	,m_mainFrame(mainFrame)
	,m_render(mainFrame->GetRender())
	,m_init(false)
	,m_font(0)
//	,m_canvas(canvas)
//	,m_mainFrame(mainFrame)
//	,m_leftMouseKeyState(false)
{
	m_render->AddRef();

}


EditorRenderViewport::~EditorRenderViewport(void)
{
	// delete GUI elements
	DestroyConstructionGrid(m_render);
	m_render->Release();
}

void EditorRenderViewport::Init()
{
	m_init = true;
	BuildConstructionGrid(m_render, 20, 1.0f);
}


int EditorRenderViewport::GetWidth() const 
{ 
	int width;
	int height;
	GetSize (&width, &height);
	return width; 
}

int EditorRenderViewport::GetHeight() const 
{ 
	int width;
	int height;
	GetSize (&width, &height);
	return height; 
}


void EditorRenderViewport::SetCameraMatrix(dViewPortModes mode)
{
	switch (mode) 
	{
		case m_perpective:
			SetPerspectiveMatrix(m_render, GetWidth(), GetHeight());
			SetGridMatrix (dPitchMatrix(90.0f * 3.141592f /180.0f));
			break;

		case m_top:
			SetOrtographicMatrix(m_render, GetWidth(), GetHeight(), dPitchMatrix(90.0f * 3.141592f /180.0f) * dRollMatrix(90.0f * 3.141592f /180.0f));
			SetGridMatrix (dPitchMatrix(90.0f * 3.141592f /180.0f));
			break;

		case m_bottom:
			SetOrtographicMatrix(m_render, GetWidth(), GetHeight(), dPitchMatrix(-90.0f * 3.141592f /180.0f) * dRollMatrix(90.0f * 3.141592f /180.0f));
			SetGridMatrix (dPitchMatrix(-90.0f * 3.141592f /180.0f));
			break;

		case m_front:
			SetOrtographicMatrix(m_render, GetWidth(), GetHeight(), dYawMatrix(90.0f * 3.141592f /180.0f));
			SetGridMatrix (dYawMatrix(90.0f * 3.141592f /180.0f));
			break;

		case m_back:
			SetOrtographicMatrix(m_render, GetWidth(), GetHeight(), dYawMatrix(-90.0f * 3.141592f /180.0f));
			SetGridMatrix (dYawMatrix(-90.0f * 3.141592f /180.0f));
			break;


		case m_left:
			SetOrtographicMatrix(m_render, GetWidth(), GetHeight(), GetIdentityMatrix());
			SetGridMatrix (GetIdentityMatrix());
			break;

		case m_right:
			SetOrtographicMatrix(m_render, GetWidth(), GetHeight(), dYawMatrix(3.141592f));
			SetGridMatrix (dYawMatrix(3.141592f));
			break;

		default:
			_ASSERTE (0);
	}
}



void EditorRenderViewport::OnSize(wxSizeEvent& event)
{
	// this is also necessary to update the context on some platforms
	wxGLCanvas::OnSize(event);
}


void EditorRenderViewport::OnEraseBackground(wxEraseEvent& WXUNUSED(event))
{
	// Do nothing, to avoid flashing on MSW
}

void EditorRenderViewport::OnPaint (wxPaintEvent& WXUNUSED(event))
{
	wxPaintDC dc(this);
	RenderFrame ();
}


void EditorRenderViewport::OnIdle(wxIdleEvent& event)
{
	wxClientDC dc(this);
	RenderFrame ();
//	event.RequestMore(); // render continuously, not only once on idle
}


void EditorRenderViewport::RenderFrame ()
{
	SetCurrent();
	if (GetContext()) {
		if (!m_init) {
			Init();
		}
		BeginRender ();
		EndRender();
	}
}



void EditorRenderViewport::BeginRender ()
{
	// start rendering scene
	m_render->BeginRender();

	// set the camera matrix for this view port		
	SetCameraMatrix(m_perpective);

	// draw construction grid 
	DrawConstructionGrid(m_render);

	// draw the gizmo
	DrawGizmo(m_render, m_font);

//m_render->SetColor(dVector(1.0f, 1.0f, 1.0f, 1.0f));
//m_render->Print (m_font, 10, 10, "this is a test");
}



void EditorRenderViewport::EndRender()
{
	m_render->EndRender();

	// draw everything and swap the display buffer
	glFlush();

	// Swap
	SwapBuffers();
}



