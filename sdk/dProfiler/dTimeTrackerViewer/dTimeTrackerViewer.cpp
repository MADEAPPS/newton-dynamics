// dTimeTrackerViewer.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "dProfilerTrace.h"
#include "imgui_impl_glfw.h"
#include "dTimeTrackerViewer.h"


int _tmain(int argc, _TCHAR* argv[])
{
	dTimeTrackerViewer app;
	app.Run();
}


dTimeTrackerViewer::dTimeTrackerViewer()
	:m_recentFiles()
	,m_currentTrace(NULL)
{
	// Setup window
	glfwSetErrorCallback(error_callback);
//	if (!glfwInit()) {
//		return 1;
	glfwInit();

	m_window = glfwCreateWindow(1280, 720, "Time Tracker visualizer", NULL, NULL);
	glfwMakeContextCurrent(m_window);

	// Setup ImGui binding
	ImGui_ImplGlfw_Init(m_window, true);

	// Load Fonts
	// (there is a default font, this is only if you want to change it. see extra_fonts/README.txt for more details)
	//ImGuiIO& io = ImGui::GetIO();
	//io.Fonts->AddFontDefault();
	//io.Fonts->AddFontFromFileTTF("../../extra_fonts/Cousine-Regular.ttf", 15.0f);
	//io.Fonts->AddFontFromFileTTF("../../extra_fonts/DroidSans.ttf", 16.0f);
	//io.Fonts->AddFontFromFileTTF("../../extra_fonts/ProggyClean.ttf", 13.0f);
	//io.Fonts->AddFontFromFileTTF("../../extra_fonts/ProggyTiny.ttf", 10.0f);
	//io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesJapanese());

	ImGuiIO& io = ImGui::GetIO();
	io.UserData = this;

	bool show_test_window = true;
	bool show_another_window = false;
	m_clear_color = ImColor(114, 144, 154);
	m_keyboardChainCallback = glfwSetKeyCallback(m_window, KeyCallback);


	// 
	m_recentFiles.Append("../xxxx.tt");
}

dTimeTrackerViewer::~dTimeTrackerViewer()
{
	if (m_currentTrace) {
		delete m_currentTrace;
	}
	// Cleanup
	ImGui_ImplGlfw_Shutdown();
	glfwTerminate();
}

void dTimeTrackerViewer::error_callback(int error, const char* description)
{
	fprintf(stderr, "Error %d: %s\n", error, description);
}

void dTimeTrackerViewer::KeyCallback(GLFWwindow* const window, int key, int, int action, int mods)
{
	ImGuiIO& io = ImGui::GetIO();
	dTimeTrackerViewer* const me = (dTimeTrackerViewer*) io.UserData;
	me->m_keyboardChainCallback(window, key, 0, action, mods);
	if (key == GLFW_KEY_ESCAPE) {
		glfwSetWindowShouldClose(window, 1);
	}
}

void dTimeTrackerViewer::Run()
{
	// Main loop
	while (!glfwWindowShouldClose(m_window)) {
		glfwPollEvents();
		ImGui_ImplGlfw_NewFrame();

		if (ImGui::BeginMainMenuBar()) {
			if (ImGui::BeginMenu("File")) {
				if (ImGui::MenuItem("Open", "")) {
					dAssert(0);
				}
				if (m_recentFiles.GetCount()) {
					ImGui::Separator();
					for (dList<dString>::dListNode* node = m_recentFiles.GetFirst(); node; node = node->GetNext()) {
						const dString& name = node->GetInfo();
						if (ImGui::MenuItem(name.GetStr(), "")) {
							FILE* const file = fopen (name.GetStr(), "rb");
							if (file) {
								if (m_currentTrace) {
									delete m_currentTrace;
								}
								m_currentTrace = new dProfilerTrace(file);
								fclose (file);
							}
						}
					}
				}

				ImGui::EndMenu();
			}

			if (ImGui::BeginMenu("Help")) {
				ImGui::EndMenu();
			}
			ImGui::EndMainMenuBar();
		}

/*
		// 1. Show a simple window
		// Tip: if we don't call ImGui::Begin()/ImGui::End() the widgets appears in a window automatically called "Debug"
		{
			static float f = 0.0f;
			ImGui::Text("Hello, world!");
			ImGui::SliderFloat("float", &f, 0.0f, 1.0f);
			ImGui::ColorEdit3("clear color", (float*)&clear_color);
			if (ImGui::Button("Test Window")) show_test_window ^= 1;
			if (ImGui::Button("Another Window")) show_another_window ^= 1;
			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
		}

		// 2. Show another simple window, this time using an explicit Begin/End pair
		if (show_another_window) {
			ImGui::SetNextWindowSize(ImVec2(200, 100), ImGuiSetCond_FirstUseEver);
			ImGui::Begin("Another Window", &show_another_window);
			ImGui::Text("Hello");
			ImGui::End();
		}

		// 3. Show the ImGui test window. Most of the sample code is in ImGui::ShowTestWindow()
		if (show_test_window) {
			ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiSetCond_FirstUseEver);
			ImGui::ShowTestWindow(&show_test_window);
		}
*/
		// Rendering
		int display_w, display_h;
		glfwGetFramebufferSize(m_window, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClearColor(m_clear_color.x, m_clear_color.y, m_clear_color.z, m_clear_color.w);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui::Render();
		glfwSwapBuffers(m_window);
	}
}