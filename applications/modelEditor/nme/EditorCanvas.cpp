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

// EditorCanvas.cpp : Defines the entry point for the application.
//


#include "toolbox_stdafx.h"
#include "EditorCanvas.h"
#include "NewtonModelEditor.h"
#include "EditorRenderViewport.h"


FXDEFMAP(EditorCanvas) MessageMap[]=
{
	FXMAPFUNC(SEL_COMMAND,		EditorCanvas::ID_VIEWPORT_MAXIMIZE,	EditorCanvas::onViewPortMaximize),
	FXMAPFUNC(SEL_COMMAND,		EditorCanvas::ID_VIEWPORT_PANNING,	EditorCanvas::onViewPortPanning),
	FXMAPFUNC(SEL_COMMAND,		EditorCanvas::ID_VIEWPORT_MOVE,	EditorCanvas::onViewPortMove),
	FXMAPFUNC(SEL_COMMAND,		EditorCanvas::ID_VIEWPORT_ROTATE, EditorCanvas::onViewPortRotate),
	FXMAPFUNC(SEL_COMMAND,		EditorCanvas::ID_VIEWPORT_ZOOM,	EditorCanvas::onViewPortZoom),
	

};
FXIMPLEMENT(EditorCanvas,FXVerticalFrame,MessageMap,ARRAYNUMBER(MessageMap))



EditorCanvas::EditorCanvas()
{
}

EditorCanvas::EditorCanvas(FXComposite* const parent, NewtonModelEditor* const mainFrame, EditorCanvas* const shareContext)
	:FXVerticalFrame(parent,FRAME_SUNKEN|LAYOUT_FILL, 0,0,0,0, 0,0,0,0 ,0,0)
	,m_mainFrame(mainFrame)
	,m_navigationStack(0)
{
	m_navigationMode[0] = NewtonModelEditor::m_panViewport;

	FXToolBar* const toolbar = new FXToolBar(this,FRAME_RAISED|LAYOUT_SIDE_TOP|LAYOUT_FILL_X,0,0,0,0, 4,4,4,4, 0,0);
	new FXButton(toolbar, "\tMaximize\tMaximize view port", mainFrame->FindIcon("maximize.gif"), this, ID_VIEWPORT_MAXIMIZE, BUTTON_TOOLBAR|FRAME_RAISED|LAYOUT_TOP|LAYOUT_LEFT);
	new FXButton(toolbar, "\tPan\tViewport Pan", mainFrame->FindIcon("camera_move.gif"), this, ID_VIEWPORT_PANNING, BUTTON_TOOLBAR|FRAME_RAISED|LAYOUT_TOP|LAYOUT_LEFT);
	new FXButton(toolbar, "\tMoveCamera\tCamera move", mainFrame->FindIcon("camera_move.gif"), this, ID_VIEWPORT_MOVE, BUTTON_TOOLBAR|FRAME_RAISED|LAYOUT_TOP|LAYOUT_LEFT);
	new FXButton(toolbar, "\tRotateCamera\tCamera rotate", mainFrame->FindIcon("camera_turn.gif"), this, ID_VIEWPORT_ROTATE, BUTTON_TOOLBAR|FRAME_RAISED|LAYOUT_TOP|LAYOUT_LEFT);
	new FXButton(toolbar, "\tZoomCamera\tCamera zoom", mainFrame->FindIcon("camera_zoom.gif"), this, ID_VIEWPORT_ZOOM, BUTTON_TOOLBAR|FRAME_RAISED|LAYOUT_TOP|LAYOUT_LEFT);


	FXComboBox* const drawmode = new FXComboBox(toolbar, 3, mainFrame, NewtonModelEditor::ID_SET_RENDER_MODE, COMBOBOX_STATIC|FRAME_SUNKEN|FRAME_THICK|LAYOUT_SIDE_TOP);
	drawmode->setNumVisible(3);
	drawmode->appendItem("textured");
	drawmode->appendItem("solid");
	drawmode->appendItem("wireframe");

	m_viewmodes = new FXComboBox(toolbar, 7, mainFrame, NewtonModelEditor::ID_SET_VIEW_MODE, COMBOBOX_STATIC|FRAME_SUNKEN|FRAME_THICK|LAYOUT_SIDE_TOP);
	m_viewmodes->setNumVisible(7);
	m_viewModelMap[m_viewmodes->appendItem("top")] = EditorRenderViewport::m_top;
	m_viewModelMap[m_viewmodes->appendItem("front")] = EditorRenderViewport::m_front;
	m_viewModelMap[m_viewmodes->appendItem("left")] = EditorRenderViewport::m_left;
	m_viewModelMap[m_viewmodes->appendItem("perspective")] = EditorRenderViewport::m_perpective;
	m_viewModelMap[m_viewmodes->appendItem("right")] = EditorRenderViewport::m_right;
	m_viewModelMap[m_viewmodes->appendItem("bottom")] = EditorRenderViewport::m_bottom;
	m_viewModelMap[m_viewmodes->appendItem("back")] = EditorRenderViewport::m_back;

	m_renderViewport = new EditorRenderViewport (this, mainFrame, this, shareContext ? shareContext->m_renderViewport : NULL);
}

EditorCanvas::~EditorCanvas(void)
{
}




long EditorCanvas::onViewPortMaximize(FXObject* sender, FXSelector id, void* eventPtr) 
{
	FX4Splitter* const splitter = m_mainFrame->m_workshop;
	if (splitter->getExpanded() == FX4Splitter::ExpandAll) {
		for (int i = 0; i < 4; i ++){
			if (this == m_mainFrame->m_canvas[i]) {
				splitter->setExpanded(1<<i);
			}
		}
	} else {
		splitter->setExpanded();
	}
	return 1;
}

long EditorCanvas::onViewPortPanning(FXObject* sender, FXSelector id, void* eventPtr) 
{
	m_navigationStack = 0;
	m_navigationMode[0] = NewtonModelEditor::m_panViewport;
	m_mainFrame->ShowNavigationMode(m_navigationMode[0]);
	return 1;
}

long EditorCanvas::onViewPortMove(FXObject* sender, FXSelector id, void* eventPtr)
{
	m_navigationStack = 0;
	m_navigationMode[0] = NewtonModelEditor::m_moveViewport;
	m_mainFrame->ShowNavigationMode(m_navigationMode[0]);
	return 1;
}

long EditorCanvas::onViewPortRotate(FXObject* sender, FXSelector id, void* eventPtr)
{
	m_navigationStack = 0;
	m_navigationMode[0] = NewtonModelEditor::m_rotateViewport;
	m_mainFrame->ShowNavigationMode(m_navigationMode[0]);
	return 1;
}

long EditorCanvas::onViewPortZoom(FXObject* sender, FXSelector id, void* eventPtr)
{
	m_navigationStack = 0;
	m_navigationMode[0] = NewtonModelEditor::m_zoomViewport;
	m_mainFrame->ShowNavigationMode(m_navigationMode[0]);
	return 1;
}



void EditorCanvas::UpdateViewport()
{
	EditorRenderViewport::dViewPortModes mode = EditorRenderViewport::dViewPortModes (m_viewModelMap[m_viewmodes->getCurrentItem()]);

	switch (m_mainFrame->m_editMode)
	{
		case NewtonModelEditor::m_editAsset:
			m_renderViewport->UpdateAsset(mode);
			break;

		case NewtonModelEditor::m_editScene:
			m_renderViewport->UpdateScene(mode);
			break;
	}
}
