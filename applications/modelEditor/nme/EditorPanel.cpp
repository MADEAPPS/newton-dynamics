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
#include "EditorPanel.h"
#include "NewtonModelEditor.h"

/*
EditorPanel::EditorPanel(FXComposite* const parent, NewtonModelEditor* const mainFrame, const char* const title)
	:FXVerticalFrame(parent, LAYOUT_SIDE_TOP|LAYOUT_FILL, 0,0,0,0, 0,0,0,0, 0,0)
	,m_mainFrame(mainFrame)
{
	FXDockBar* const dockbar = (FXDockBar*) getParent();
	FXDockTitle* const caption = new FXDockTitle(this, title, dockbar, FXToolBar::ID_TOOLBARGRIP,LAYOUT_FILL_X|FRAME_SUNKEN|JUSTIFY_CENTER_X);
	caption->setBackColor(FXRGBA(0x80, 0x80, 0x80, 0xff));
//	new FXMDIDeleteButton(this,dockbar,FXWindow::ID_HIDE,LAYOUT_FILL_Y);

}

EditorPanel::~EditorPanel(void)
{
}


void EditorPanel::Hide()
{
	hide();
	recalc();
}

void EditorPanel::Unhide()
{
	show();
	recalc();
}
*/