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
	// Exclude rarely-used stuff from Windows headers
	#define WIN32_LEAN_AND_MEAN		
	
	#define GLFW_EXPOSE_NATIVE_WIN32
	#include <windows.h>
	#include <commctrl.h>
	#include <crtdbg.h>

	// insert the Im gui back end here
	//#include <backends/imgui_impl_glfw.h>
	//#include <backends/imgui_impl_opengl3.h>

#else
	#error 
#endif


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

	ndRender* m_owner;

	ndSharedPtr<ndRenderShaderCache> m_shaderCache;

	ndInt32 m_prevKey;
	bool m_imGuiEnabled;
	bool m_mousePressed[3];

	friend class ndRenderTexture;
	friend class ndGuiRenderPass;
	friend class ndColorRenderPass;
	friend class ndRenderSceneCamera;
	friend class ndRenderPrimitiveMeshImplement;
	friend class ndEnvironmentRenderPassImplement;
};

#endif 

