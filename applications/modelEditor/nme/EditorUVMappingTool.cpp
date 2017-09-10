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
#include "NewtonModelEditor.h"
#include "EditorUVMappingTool.h"


int EditorUVMappingTool::m_attributes[] = {WX_GL_DOUBLEBUFFER, WX_GL_RGBA, WX_GL_DEPTH_SIZE, 32, 0};

BEGIN_EVENT_TABLE (EditorUVMappingTool, wxGLCanvas)
	EVT_SIZE (OnSize)
	EVT_PAINT(OnPaint)
	EVT_IDLE(OnIdle)
	EVT_ERASE_BACKGROUND (OnEraseBackground)
END_EVENT_TABLE()


EditorUVMappingTool::EditorUVMappingTool (NewtonModelEditor* const mainFrame)
	:wxGLCanvas (mainFrame, wxID_ANY, wxDefaultPosition, wxSize (300, 300), wxSUNKEN_BORDER|wxFULL_REPAINT_ON_RESIZE, _("GLRenderCanvas"), m_attributes)
	,dPluginCamera()
	,m_mainFrame(mainFrame)
	,m_render(mainFrame->GetRender())
{
	m_render->AddRef();
}


EditorUVMappingTool::~EditorUVMappingTool(void)
{
	// delete GUI elements
	DestroyConstructionGrid(m_render);
	m_render->Release();
}

void EditorUVMappingTool::OnEraseBackground(wxEraseEvent& WXUNUSED(event))
{
	// Do nothing, to avoid flashing on MSW
}


void EditorUVMappingTool::OnIdle(wxIdleEvent& event)
{
	wxClientDC dc(this);
	RenderFrame ();
//	event.RequestMore(); // render continuously, not only once on idle
}

void EditorUVMappingTool::OnSize(wxSizeEvent& event)
{
	// this is also necessary to update the context on some platforms
	wxGLCanvas::OnSize(event);
}



void EditorUVMappingTool::OnPaint (wxPaintEvent& WXUNUSED(event))
{
	wxPaintDC dc(this);
	RenderFrame ();
}



void EditorUVMappingTool::RenderFrame ()
{
/*
	SetCurrent();
	if (GetContext()) {
		if (!m_init) {
			Init();
		}
		BeginRender ();
		UpdateScene();
		EndRender();
	}
*/
}
