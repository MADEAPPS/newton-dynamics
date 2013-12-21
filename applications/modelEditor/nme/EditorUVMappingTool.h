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

#ifndef __EDITOR_UV_MAPPING_TOOL_H__
#define __EDITOR_UV_MAPPING_TOOL_H__

class NewtonModelEditor;

class EditorUVMappingTool: public wxGLCanvas, public dPluginCamera
{
	public:
	EditorUVMappingTool (NewtonModelEditor* const parent);
	~EditorUVMappingTool();


	void RenderFrame ();
	protected:
	DECLARE_EVENT_TABLE()

	void OnSize(wxSizeEvent &event);
	void OnIdle(wxIdleEvent &event);
	void OnPaint(wxPaintEvent& event);
	void OnEraseBackground(wxEraseEvent& event);

	NewtonModelEditor* m_mainFrame;
	dSceneRender* m_render;

	static int m_attributes[];
};



#endif