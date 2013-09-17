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

#ifndef __EDITOR_RENDER_VIEWPORT_H__
#define __EDITOR_RENDER_VIEWPORT_H__

class EditorCanvas;
class NewtonModelEditor;

class GLVisual: public FXGLVisual
{
	public:
	GLVisual (FXApp* const app)
		:FXGLVisual (app, VISUAL_DOUBLEBUFFER)
	{
	}
};


class EditorRenderViewport: public FXGLCanvas, public dPluginCamera
{
	public:


	enum dViewPortModes
	{
		m_perpective,
		m_left,
		m_right,
		m_front,
		m_back,
		m_top, 
		m_bottom
	};

	EditorRenderViewport();
	EditorRenderViewport(FXComposite* const parent, NewtonModelEditor* const mainFrame, EditorCanvas* const canvas, EditorRenderViewport* const shareContext);
	~EditorRenderViewport(void);
	void create();

	void UpdateScene (dViewPortModes mode);
	void UpdateAsset (dViewPortModes mode);

	long onLeftMouseKeyDown(FXObject* sender, FXSelector id, void* eventPtr);
	long onLeftMouseKeyUp(FXObject* sender, FXSelector id, void* eventPtr);
	long onMouseMove(FXObject* sender, FXSelector id, void* eventPtr);
	

	protected:
	void SetCameraMatrix(dViewPortModes mode);

	void BeginRender(dViewPortModes mode);
	void EndRender();


	void SelectAssetNode (const FXEvent* const event);

	void RenderSelectedNodeGizmo () const;

	int m_font;
	EditorCanvas* m_canvas;
	dSceneRender* m_render;
	NewtonModelEditor* m_mainFrame;
	bool m_leftMouseKeyState;

	FXDECLARE(EditorRenderViewport)
};

#endif