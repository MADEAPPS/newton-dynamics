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

#ifndef _EDITOR_PANEL_H_
#define _EDITOR_PANEL_H_

class NewtonModelEditor;
class DemoEntityManager;


//class EditorPanel: public FXHorizontalFrame 
class EditorPanel: public FXVerticalFrame
{
	public:
	EditorPanel(FXComposite* const parent, NewtonModelEditor* const mainFrame, const char* const title);
	~EditorPanel(void);

	void Hide();
	void Unhide();
	NewtonModelEditor* const m_mainFrame;
};


#endif