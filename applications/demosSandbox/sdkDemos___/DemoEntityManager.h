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
#ifndef __DEMO_MAIN_FRAME_H__
#define __DEMO_MAIN_FRAME_H__


struct GLFWwindow;
struct ImDrawData;

class DemoEntityManager
{
	public:
	DemoEntityManager ();
	~DemoEntityManager ();

	void Run();

	private:
	void BeginFrame();
	void EndFrame();
	void LoadDefaultFont();
	void ShowMainMenuBar();

	
	static void RenderDrawListsCallback(ImDrawData* const draw_data);
	static void KeyCallback(GLFWwindow* const window, int key, int, int action, int mods);

	static void CursorposCallback  (GLFWwindow* const window, double x, double y);
	static void MouseScrollCallback (GLFWwindow* const window, double x, double y);
	static void MouseButtonCallback(GLFWwindow* const window, int button, int action, int mods);
	static void ErrorCallback(int error, const char* const description);
	GLFWwindow* m_mainFrame;
	int	m_defaultFont;
	bool m_mousePressed[3];
};

#endif