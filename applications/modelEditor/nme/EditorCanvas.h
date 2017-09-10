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

#ifndef _EDITOR_CANVAS_H_
#define _EDITOR_CANVAS_H_

#include "NewtonModelEditor.h"

//class NewtonModelEditor;
class EditorRenderViewport;

class EditorCanvas: public FXVerticalFrame
{
	public:
	enum {
		ID_VIEWPORT_MAXIMIZE = FXMainWindow::ID_LAST + 2000,
		ID_VIEWPORT_PANNING,
		ID_VIEWPORT_MOVE,
		ID_VIEWPORT_ROTATE,
		ID_VIEWPORT_ZOOM,
	};



	EditorCanvas();
	EditorCanvas(FXComposite* const parent, NewtonModelEditor* const mainFrame, EditorCanvas* const shareContext);
	virtual ~EditorCanvas(void);

	void UpdateViewport();


	long onViewPortMaximize(FXObject* sender, FXSelector id, void* eventPtr); 
	long onViewPortPanning(FXObject* sender, FXSelector id, void* eventPtr); 
	long onViewPortZoom(FXObject* sender, FXSelector id, void* eventPtr); 
	long onViewPortMove(FXObject* sender, FXSelector id, void* eventPtr); 
	long onViewPortRotate(FXObject* sender, FXSelector id, void* eventPtr); 

	NewtonModelEditor* m_mainFrame;
	FXComboBox* m_viewmodes; 
	EditorRenderViewport* m_renderViewport;

	int m_viewModelMap[32];

	int m_navigationStack;
	NewtonModelEditor::NavigationMode m_navigationMode[16];

	FXDECLARE(EditorCanvas)
};


#endif