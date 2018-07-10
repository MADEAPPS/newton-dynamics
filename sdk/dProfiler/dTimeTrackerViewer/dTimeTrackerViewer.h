// dTimeTrackerViewer.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "imgui_impl_glfw.h"

class dTimeTrackerViewer
{
	public:
	dTimeTrackerViewer();
	~dTimeTrackerViewer();

	void Run();

	private:
	static void error_callback(int error, const char* description);
	static void KeyCallback(GLFWwindow* const window, int key, int, int action, int mods);

	GLFWwindow* m_window;
	ImVec4 m_clear_color;
	GLFWkeyfun m_keyboardChainCallback;
};

