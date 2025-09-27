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

#ifndef _ND_RENDER_CONTEXT_H__
#define _ND_RENDER_CONTEXT_H__

#include "ndRenderStdafx.h"

#if (defined(WIN32) || defined(_WIN32))
	#define WIN32_LEAN_AND_MEAN		
	
	#define GLFW_EXPOSE_NATIVE_WIN32
	#include <windows.h>
	#include <commctrl.h>
	#include <crtdbg.h>


#else
	#include <unistd.h>
	//#include <glatter.h>
	//#include <GL/gl.h>
	//#include <GLFW/glfw3.h>
	//#include <GLFW/glfw3native.h>
#endif

#include <glatter.h>
//#include <GL/glu.h>
#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

class ndRender;
class ndRenderTexture;
class ndRenderShaderCache;

class ndRenderContext: public ndClassAlloc
{
	public:
	ndRenderContext(ndRender* const owner, ndInt32 width, ndInt32 height, const char* const title);
	virtual ~ndRenderContext();

	void Terminate();
	void Present() const;
	bool PollEvents() const;
	bool ShouldFinish() const;

	ndInt32 GetWidth() const;
	ndInt32 GetHeight() const;
	void SetTitle(const char* const title);
	void InitImGui(const char* const fontPathName);

	void EndFrame();
	void BeginFrame();
	void ClearFrameBuffer(const ndVector& color);

	private:
	void LoadFont(const char* const fontPathName);
	ndSharedPtr<ndRenderTexture> LoadTexture(const ndString& pathname);
	ndSharedPtr<ndRenderTexture> LoadCubeMap(const ndFixSizeArray<ndString, 6>& pathnames);

	void SetInputCallbacks();

	void EndGuiRenderStates();
	void SetGuiRenderStates();
	void SetCollorPassRenderStates();

	void SetViewport(ndInt32 width, ndInt32 height);
	static void ErrorCallback(ndInt32 error, const char* const description);
	static void ResizeWindowsCallback(GLFWwindow* window, int x, int y);

	static void CharCallback(GLFWwindow* window, ndUnsigned32 ch);
	static void CursorposCallback(GLFWwindow* const window, double x, double y);
	static void MouseScrollCallback(GLFWwindow* const window, double x, double y);
	static void MouseButtonCallback(GLFWwindow* const window, ndInt32 button, ndInt32 action, ndInt32 mods);
	static void KeyCallback(GLFWwindow* const window, ndInt32 key, ndInt32, ndInt32 action, ndInt32 mods);

#if (defined(_DEBUG) && defined(WIN32))
	static void APIENTRY OpenMessageCallback(
		GLenum source, GLenum type, GLuint id, GLenum severity,
		GLsizei length, const GLchar* message, const void* userParam);
#endif

	ndRender* m_owner;
	GLFWwindow* m_mainFrame;
	ndSharedPtr<ndRenderShaderCache> m_shaderCache;
	GLint m_defaultFont;
	ndInt32 m_prevKey;
	bool m_imGuiEnabled;
	bool m_mousePressed[3];

	friend class ndRenderTexture;
	friend class ndRenderPassGui;
	friend class ndRenderPassColor;
	friend class ndRenderSceneCamera;
	friend class ndRenderPassTransparency;
	friend class ndRenderPassShadowsImplement;
	friend class ndRenderPrimitiveImplement;
	friend class ndRenderPassEnvironmentImplement;
};

#endif 

