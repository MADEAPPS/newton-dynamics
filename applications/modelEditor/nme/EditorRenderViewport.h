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


//class EditorCanvas;
class NewtonModelEditor;


class EditorRenderViewport: public wxGLCanvas, public dPluginCamera
{
	public:

	enum dShadingModes
	{
		m_textured,
		m_solid,
		m_wireframe,
	};

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

	EditorRenderViewport (NewtonModelEditor* const parent);
	~EditorRenderViewport();

	int GetWidth() const;
	int GetHeight() const;


/*
	void create();

	void UpdateScene (dViewPortModes mode);
	void UpdateAsset (dViewPortModes mode);

	

	protected:
	

	void BeginRender(dViewPortModes mode);
	void EndRender();


	void SelectAssetNode (const FXEvent* const event);

	void RenderSelectedNodeGizmo () const;

	
	EditorCanvas* m_canvas;
	
	
	bool m_leftMouseKeyState;

	FXDECLARE(EditorRenderViewport)
*/

	protected:
	DECLARE_EVENT_TABLE()


//	long onLeftMouseKeyUp(FXObject* sender, FXSelector id, void* eventPtr);
//	long onMouseMove(FXObject* sender, FXSelector id, void* eventPtr);

//	void OnKeyUp(wxKeyEvent &event);
//	void OnKeyDown(wxKeyEvent &event);
	void OnMouse(wxMouseEvent &event);

	void OnSize(wxSizeEvent &event);
	void OnIdle(wxIdleEvent &event);
	void OnPaint(wxPaintEvent& event);
	void OnEraseBackground(wxEraseEvent& event);

	void Init();
	void RenderFrame ();
	void BeginRender();
	void EndRender();
	void SetCameraMatrix(dViewPortModes mode);

	void LeftMouseKeyDown ();
	void LeftMouseKeyUp ();

	NewtonModelEditor* m_mainFrame;
	dSceneRender* m_render;

	int m_font;
	bool m_init;

	int m_lastMouseX;
	int m_lastMouseY;
	bool m_leftMouseKeyState;
	static int m_attributes[];
};



#endif