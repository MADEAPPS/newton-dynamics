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

#ifndef _EDITOR_COMMAND_PANEL_H_
#define _EDITOR_COMMAND_PANEL_H_


#include "foldpanelbar.h"

class NewtonModelEditor;
class DemoEntityManager;


//class EditorCommandPanel: public wxFoldPanelBar
//class EditorCommandPanel: public  wxScrollBar
class EditorCommandPanel: public wxScrolledWindow
{
	class FoldBarCommandPanel;

	public:
	EditorCommandPanel(NewtonModelEditor* const mainFrame);
	~EditorCommandPanel(void);
};


#endif