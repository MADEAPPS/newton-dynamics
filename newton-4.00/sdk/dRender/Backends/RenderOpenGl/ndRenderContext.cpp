/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndRenderStdafx.h"
#include "ndRender.h"
#include "ndRenderContext.h"
#include "ndRenderTexture.h"
#include "ndRenderShaderCache.h"
#include "ndRenderTextureImage.h"

ndRenderContext::ndRenderContext(ndRender* const owner, ndInt32 width, ndInt32 height, const char* const title)
	:ndClassAlloc()
	,m_owner (owner)
	,m_shaderCache(nullptr)
	,m_defaultFont(0)
	,m_prevKey(0)
	,m_imGuiEnabled(false)
{
	glfwSetErrorCallback(ErrorCallback);
	glfwInit();

	// Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
	// GL ES 2.0 + GLSL 100
	//const char* glsl_version = "#version 100";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
	// GL 3.2 + GLSL 150
	//const char* glsl_version = "#version 150";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
	// GL 3.0 + GLSL 130
	//const char* glsl_version = "#version 130";
	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
	//glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);          // 3.0+ only
#endif

#if defined (_DEBUG)
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
#endif

	// Create window with graphics context
	m_mainFrame = glfwCreateWindow(width, height, title, nullptr, nullptr);
	glfwMakeContextCurrent(m_mainFrame);
	glfwSwapInterval(0); // Enable vsync

	ndInt32 monitorsCount;
	GLFWmonitor** monitors = glfwGetMonitors(&monitorsCount);
	if (monitorsCount > 1)
	{
		ndInt32 window_x;
		ndInt32 window_y;
		ndInt32 monitor_x;
		ndInt32 monitor_y;

		glfwGetMonitorPos(monitors[1], &monitor_x, &monitor_y);
		glfwGetWindowPos(m_mainFrame, &window_x, &window_y);
		glfwSetWindowPos(m_mainFrame, monitor_x + window_x, monitor_y + 64);
	}
	// attach myself to the main frame
	glfwSetWindowUserPointer(m_mainFrame, this);

	SetInputCallbacks();
	//void ndRenderContext::FramebufferSizeCallback(GLFWwindow * window, int x, int y)
	glfwSetFramebufferSizeCallback(m_mainFrame, FramebufferSizeCallback);

#if (defined(_DEBUG) && defined(WIN32))	
	glDebugMessageCallback(OpenMessageCallback, m_mainFrame);
	glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DONT_CARE, 0, nullptr, GL_TRUE);
	glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);
#endif

	m_mousePressed[0] = false;
	m_mousePressed[1] = false;
	m_mousePressed[2] = false;

	m_shaderCache = ndSharedPtr<ndRenderShaderCache>(new ndRenderShaderCache());
}

ndRenderContext::~ndRenderContext()
{
	m_shaderCache->Cleanup();

	if (m_imGuiEnabled)
	{
		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImGui::DestroyContext();
	}

	glfwDestroyWindow(m_mainFrame);
	glfwTerminate();
}

void ndRenderContext::SetTitle(const char* const title)
{
	glfwSetWindowTitle(m_mainFrame, title);
}

void ndRenderContext::Terminate()
{
	glfwSetWindowShouldClose(m_mainFrame, 1);
}

void ndRenderContext::SetViewport(ndInt32 width, ndInt32 height)
{
	glViewport(0, 0, width, height);
}

#if (defined(_DEBUG) && defined(WIN32))
void APIENTRY ndRenderContext::OpenMessageCallback(GLenum source,
	GLenum type,
	GLuint id,
	GLenum severity,
	GLsizei length,
	const GLchar* message,
	const void* userParam)
{
	if (userParam)
	{
		switch (id)
		{
			case 2:		 // no sure why on Intel embedded systems I get this warding, 
				// ID_RECOMPILE_FRAGMENT_SHADER performance warning has been generated.
				// Fragment shader recompiled due to state change., length = 120
			case 131154:  // Pixel-path performance warning: Pixel transfer is synchronized with 3D rendering.
			case 131185:  // nvidia driver report will use VIDEO memory as the source for buffer object operations
			case 131139: //	for some reason when using different target I get this on nvidia gpus.
				//	no one seems to know what cause this
				// Rasterization quality warning : A non - fullscreen clear caused a fallback from CSAA to MSAA.
				return;
		}
		ndTrace(("GL CALLBACK: %s source = 0x%x, type = 0x%x, id = %d, severity = 0x%x, message = %s, length = %d \n",
			(type == GL_DEBUG_TYPE_ERROR ? "** GL ERROR **" : ""), source, type, id, severity, message, length));
		//ndAssert(0);
	}
}
#endif

void ndRenderContext::InitImGui(const char* const fontPathName)
{
	ndAssert(!m_imGuiEnabled);
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	//io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
	//io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;           // Enable Docking
	
	// Setup Dear ImGui style
	//ImGui::StyleColorsDark();
	ImGui::StyleColorsLight();
	ImGuiStyle* const style = &ImGui::GetStyle();
	style->Colors[ImGuiCol_WindowBg] = ImVec4(0.94f, 0.94f, 0.94f, 0.5f);
	
	// Setup Platform/Renderer back ends
	const char* glsl_version = "#version 450";
	ImGui_ImplGlfw_InitForOpenGL(m_mainFrame, true);
	ImGui_ImplOpenGL3_Init(glsl_version);

	m_imGuiEnabled = true;

	LoadFont(fontPathName);

	// Setup ImGui binding
	io.UserData = this;

	// Keyboard mapping. ImGui will use those indices to peek into the io.KeyDown[] array.
	io.KeyMap[ImGuiKey_Tab] = GLFW_KEY_TAB;
	io.KeyMap[ImGuiKey_LeftArrow] = GLFW_KEY_LEFT;
	io.KeyMap[ImGuiKey_RightArrow] = GLFW_KEY_RIGHT;
	io.KeyMap[ImGuiKey_UpArrow] = GLFW_KEY_UP;
	io.KeyMap[ImGuiKey_DownArrow] = GLFW_KEY_DOWN;
	io.KeyMap[ImGuiKey_PageUp] = GLFW_KEY_PAGE_UP;
	io.KeyMap[ImGuiKey_PageDown] = GLFW_KEY_PAGE_DOWN;
	io.KeyMap[ImGuiKey_Home] = GLFW_KEY_HOME;
	io.KeyMap[ImGuiKey_End] = GLFW_KEY_END;
	io.KeyMap[ImGuiKey_Delete] = GLFW_KEY_DELETE;
	io.KeyMap[ImGuiKey_Backspace] = GLFW_KEY_BACKSPACE;
	io.KeyMap[ImGuiKey_Enter] = GLFW_KEY_ENTER;
	io.KeyMap[ImGuiKey_Escape] = GLFW_KEY_ESCAPE;
	io.KeyMap[ImGuiKey_A] = GLFW_KEY_A;
	io.KeyMap[ImGuiKey_C] = GLFW_KEY_C;
	io.KeyMap[ImGuiKey_V] = GLFW_KEY_V;
	io.KeyMap[ImGuiKey_X] = GLFW_KEY_X;
	io.KeyMap[ImGuiKey_Y] = GLFW_KEY_Y;
	io.KeyMap[ImGuiKey_Z] = GLFW_KEY_Z;

#ifdef _MSC_VER 
	io.ImeWindowHandle = glfwGetWin32Window(m_mainFrame);
#endif

	SetInputCallbacks();
}

void ndRenderContext::SetInputCallbacks()
{
	glfwSetKeyCallback(m_mainFrame, KeyCallback);
	glfwSetCharCallback(m_mainFrame, CharCallback);
	glfwSetScrollCallback(m_mainFrame, MouseScrollCallback);
	glfwSetCursorPosCallback(m_mainFrame, CursorposCallback);
	glfwSetMouseButtonCallback(m_mainFrame, MouseButtonCallback);
}

void ndRenderContext::ErrorCallback(ndInt32 error, const char* description)
{
	ndTrace(("Error %d: %s\n", error, description));
	fprintf(stderr, "Error %d: %s\n", error, description);
	ndAssert(0);
}

void ndRenderContext::FramebufferSizeCallback(GLFWwindow* window, int width, int height)
{
	ndRenderContext* const self = (ndRenderContext*)glfwGetWindowUserPointer(window);
	self->SetViewport(width, height);
}

void ndRenderContext::LoadFont(const char* const fontPathName)
{
	// Build texture atlas
	ImGuiIO& io = ImGui::GetIO();

	// Load Fonts
	// (there is a default font, this is only if you want to change it. see extra_fonts/README.txt for more details)
	//io.Fonts->AddFontDefault();

	float pixedSize = 18;
	//char pathName[2048];
	//const char* const name = "Cousine-Regular.ttf";
	////char* const name = "calibri.ttf";
	////char* const name = "courbd.ttf";
	//
	//ndGetWorkingFileName(name, pathName);
	io.Fonts->AddFontFromFileTTF(fontPathName, pixedSize);
	//io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, nullptr, io.Fonts->GetGlyphRangesJapanese());

	// Load as RGBA 32-bits (75% of the memory is wasted, but default font is so small) 
	// because it is more likely to be compatible with user's existing shaders. 
	// If your ImTextureId represent a higher-level concept than just a GL texture id, 
	// consider calling GetTexDataAsAlpha8() instead to save on GPU memory.
	unsigned char* pixels;
	ndInt32 width, height;
	io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);

	// Upload texture to graphics system
	GLint last_texture;
	GLuint font_texture;
	glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture);
	glGenTextures(1, &font_texture);
	glBindTexture(GL_TEXTURE_2D, font_texture);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);

	// Store our identifier
	m_defaultFont = GLint(font_texture);
	io.Fonts->TexID = (void*)(intptr_t)m_defaultFont;

	// Restore state
	glBindTexture(GL_TEXTURE_2D, GLuint(last_texture));
}

bool ndRenderContext::ShouldFinish() const
{
	return glfwWindowShouldClose(m_mainFrame) ? true : false;
}

ndInt32 ndRenderContext::GetWidth() const
{
	ndInt32 w, h;
	glfwGetWindowSize(m_mainFrame, &w, &h);
	return w;
}

ndInt32 ndRenderContext::GetHeight() const
{
	ndInt32 w, h;
	glfwGetWindowSize(m_mainFrame, &w, &h);
	return h;
}

bool ndRenderContext::PollEvents() const
{
	// Poll and handle events (inputs, window resize, etc.)
	// You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
	// - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
	// - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
	// Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
	glfwPollEvents();
	
	ndInt32 w, h;
	glfwGetWindowSize(m_mainFrame, &w, &h);
	return (w != 0) && (h != 0);
}

void ndRenderContext::Present() const
{
	glfwSwapBuffers(m_mainFrame);
}

void ndRenderContext::BeginFrame()
{
	ImGuiIO& io = ImGui::GetIO();
	
	// Setup display size (every frame to accommodate for window resizing)
	ndInt32 w, h;
	ndInt32 display_w, display_h;
	glfwGetWindowSize(m_mainFrame, &w, &h);
	glfwGetFramebufferSize(m_mainFrame, &display_w, &display_h);
	io.DisplaySize = ImVec2((ndReal)w, (ndReal)h);
	io.DisplayFramebufferScale = ImVec2(w > 0 ? ((ndReal)display_w / (ndReal)w) : 0, h > 0 ? ((ndReal)display_h / (ndReal)h) : 0);

	if (m_imGuiEnabled)
	{
		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
	}
}

void ndRenderContext::EndFrame()
{
	if (m_imGuiEnabled)
	{
		ImGui::EndFrame();
	}
}

void ndRenderContext::MouseButtonCallback(GLFWwindow* window, ndInt32 button, ndInt32 action, ndInt32)
{
	//if (button >= 0 && button < 3) 
	//{
	//	ImGuiIO& io = ImGui::GetIO();
	//	if (action == GLFW_PRESS) 
	//	{
	//		io.MouseDown[button] = true;    
	//	} 
	//	else if (action == GLFW_RELEASE) 
	//	{
	//		io.MouseDown[button] = false;    
	//	}
	//}
	ndRenderContext* const self = (ndRenderContext*)glfwGetWindowUserPointer(window);
	self->m_owner->m_owner->MouseButtonCallback(button, action);
}

void ndRenderContext::CharCallback(GLFWwindow* window, ndUnsigned32 ch)
{
	//ImGuiIO& io = ImGui::GetIO();
	//io.AddInputCharacter((unsigned short)ch);
	ndRenderContext* const self = (ndRenderContext*)glfwGetWindowUserPointer(window);
	self->m_owner->m_owner->CharCallback(ch);
}

void ndRenderContext::CursorposCallback(GLFWwindow* window, double x, double y)
{
	ndRenderContext* const self = (ndRenderContext*)glfwGetWindowUserPointer(window);
	//ImGuiIO& io = ImGui::GetIO();
	//io.MousePos = ImVec2((float)x, (float)y);
	self->m_owner->m_owner->CursorposCallback(ndReal(x), ndReal(y));
}

void ndRenderContext::MouseScrollCallback(GLFWwindow* window, double x, double y)
{
	ndRenderContext* const self = (ndRenderContext*)glfwGetWindowUserPointer(window);
	//ImGuiIO& io = ImGui::GetIO();
	//io.MouseWheel += float(y);
	self->m_owner->m_owner->MouseScrollCallback(ndReal(x), ndReal(y));
}

//void ndDemoEntityManager::KeyCallback(GLFWwindow* const window, ndInt32 key, ndInt32, ndInt32 action, ndInt32 mods)
void ndRenderContext::KeyCallback(GLFWwindow* const window, ndInt32 key, ndInt32, ndInt32 action, ndInt32)
{
	ndRenderContext* const self = (ndRenderContext*)glfwGetWindowUserPointer(window);
	ImGuiIO& io = ImGui::GetIO();
	if (action == GLFW_PRESS)
		io.KeysDown[key] = true;
	if (action == GLFW_RELEASE)
		io.KeysDown[key] = false;
	
	io.KeyCtrl = io.KeysDown[GLFW_KEY_LEFT_CONTROL] || io.KeysDown[GLFW_KEY_RIGHT_CONTROL];
	io.KeyShift = io.KeysDown[GLFW_KEY_LEFT_SHIFT] || io.KeysDown[GLFW_KEY_RIGHT_SHIFT];
	io.KeyAlt = io.KeysDown[GLFW_KEY_LEFT_ALT] || io.KeysDown[GLFW_KEY_RIGHT_ALT];
	io.KeySuper = io.KeysDown[GLFW_KEY_LEFT_SUPER] || io.KeysDown[GLFW_KEY_RIGHT_SUPER];
	
	if ((key == GLFW_KEY_F10) && (key != self->m_prevKey))
	{
		self->m_owner->m_owner->KeyCallback(key, action);
	}
	
	if (key == GLFW_KEY_ESCAPE)
	{
		self->Terminate();
	}
	
	if ((key == GLFW_KEY_F1) && (key != self->m_prevKey))
	{
		self->m_owner->m_owner->KeyCallback(key, action);
	}
	
	self->m_prevKey = io.KeysDown[key] ? key : 0;
}

void ndRenderContext::EndGuiRenderStates()
{
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ndRenderContext::SetGuiRenderStates()
{
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glEnable(GL_SCISSOR_TEST);
}

void ndRenderContext::SetCollorPassRenderStates()
{
	glDisable(GL_SCISSOR_TEST);

	// Culling. 
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);
	glEnable(GL_CULL_FACE);

	//	glEnable(GL_DITHER);
	// z buffer test
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
}

void ndRenderContext::ClearFrameBuffer(const ndVector& color)
{
	glClearColor(GLfloat(color.m_x), GLfloat(color.m_y), GLfloat(color.m_z), GLfloat(color.m_w));
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

ndSharedPtr<ndRenderTexture> ndRenderContext::LoadTexture(const ndString& pathname)
{
	char tmp[256];
	snprintf(tmp, sizeof(tmp), "%s", pathname.GetStr());
	strtolwr(tmp);
	const char* const fileNameEnd = strstr(tmp, ".png");
	if (!fileNameEnd)
	{
		strcat(tmp, ".png");
		ndTrace(("subtitute texture %s with %s version\n", pathname.GetStr(), tmp));
		ndAssert(0);
	}

	ndSharedPtr<ndRenderTexture> texture(nullptr);
	unsigned width;
	unsigned height;
	unsigned char* pBits;
	unsigned ret = lodepng_decode_file(&pBits, &width, &height, tmp, LCT_RGBA, 8);
	ndAssert(!ret);
	if (!ret)
	{
		// from targa legacy reasons, the texture is upsizedown, 
		// so I have to flip it
		unsigned* const buffer = (unsigned*)pBits;
		for (ndInt32 i = 0; i < ndInt32(height / 2); i++)
		{
			unsigned* const row0 = &buffer[i * width];
			unsigned* const row1 = &buffer[(height - 1 - i) * width];
			for (ndInt32 j = 0; j < ndInt32(width); ++j)
			{
				ndSwap(row0[j], row1[j]);
			}
		}
		texture = ndSharedPtr<ndRenderTexture>(new ndRenderTextureImage(pBits, int(width), int(height), ndRenderTexture::m_rgba));
		lodepng_free(pBits);
	}
	return texture;
}

ndSharedPtr<ndRenderTexture> ndRenderContext::LoadCubeMap(const ndFixSizeArray<ndString, 6>& pathnames)
{
	ndFixSizeArray<ndUnsigned32, 6> faceArray;
	faceArray.PushBack(GL_TEXTURE_CUBE_MAP_POSITIVE_X);
	faceArray.PushBack(GL_TEXTURE_CUBE_MAP_NEGATIVE_X);
	faceArray.PushBack(GL_TEXTURE_CUBE_MAP_POSITIVE_Y);
	faceArray.PushBack(GL_TEXTURE_CUBE_MAP_NEGATIVE_Y);
	faceArray.PushBack(GL_TEXTURE_CUBE_MAP_POSITIVE_Z);
	faceArray.PushBack(GL_TEXTURE_CUBE_MAP_NEGATIVE_Z);

	ndSharedPtr<ndRenderTexture> texture(new ndRenderTextureCubeMapImage());
	ndRenderTextureCubeMapImage* const cubeMap = (ndRenderTextureCubeMapImage*)*texture;
	for (ndInt32 i = 0; i < pathnames.GetCount(); ++i)
	{
		ndAssert(pathnames[i].Size());
		char tmp[256];
		snprintf(tmp, sizeof(tmp), "%s", pathnames[i].GetStr());
		strtolwr(tmp);
		char* const fileNameEnd = strstr(tmp, ".png");
		if (!fileNameEnd)
		{
			ndAssert(0);
			*fileNameEnd = 0;
			strcat(tmp, ".png");
			ndTrace(("subtitute texture %s with %s version\n", pathnames[i].GetStr(), tmp));
		}

		unsigned width;
		unsigned height;
		unsigned char* pBits = nullptr;
		lodepng_decode_file(&pBits, &width, &height, tmp, LCT_RGBA, 8);
		ndAssert(pBits);

		cubeMap->LoadFace(faceArray[i], pBits, int(width), int(height), ndRenderTexture::m_rgba);
		lodepng_free(pBits);
	}

	return texture;
}