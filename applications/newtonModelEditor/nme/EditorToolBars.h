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


#ifndef _EDITOR_TOOBAR_H_
#define _EDITOR_TOOBAR_H_

class NewtonModelEditor;
class DemoEntityManager;


class EditorToolBar: public FXToolBar 
{
	public:
	EditorToolBar(FXComposite* const parent, FXToolBarShell* const dragShell, unsigned flags);
	~EditorToolBar(void);

	void Hide();
	void Unhide();

	protected:
	FXIcon* LoadIcon (const char* const filename) const;
};



class EditorFileToolBar: public EditorToolBar 
{
	public:
	EditorFileToolBar(FXComposite* const parent, FXToolBarShell* const dragShell, NewtonModelEditor* const mainFrame);
	~EditorFileToolBar(void);
};


class EditorNavigationToolBar: public EditorToolBar 
{
	public:
	EditorNavigationToolBar(FXComposite* const parent, FXToolBarShell* const dragShell, NewtonModelEditor* const mainFrame);
	~EditorNavigationToolBar(void);
};


#endif