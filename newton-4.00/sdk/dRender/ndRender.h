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

#ifndef _ND_RENDER_H__
#define _ND_RENDER_H__

#include "ndRenderStdafx.h"

class ndRenderPass;
class ndRenderContext;
class ndRenderSceneNode;
class ndRenderSceneCamera;
class ndRenderTextureCache;
class ndRenderPassShadowsImplement;

class ndRender: public ndClassAlloc
{
	public:
	class ndUserCallback : public ndClassAlloc
	{
		public:
		ndUserCallback()
			:ndClassAlloc()
		{
		}

		virtual ~ndUserCallback()
		{
		}

		virtual void KeyCallback(ndInt32, ndInt32) = 0;
		virtual void CharCallback(ndUnsigned32 ch) = 0;
		virtual void CursorposCallback(ndReal x, ndReal y) = 0;
		virtual void MouseScrollCallback(ndReal x, ndReal y) = 0;
		virtual void MouseButtonCallback(ndInt32 button, ndInt32 action) = 0;
	};

	ndRender(ndSharedPtr<ndUserCallback>& owner, ndInt32 width, ndInt32 height, const char* const title);
	virtual ~ndRender();

	void Present();
	void Terminate();
	bool PollEvents() const;
	bool ShouldFinish() const;
	void SetTitle(const char* const title);
	void InitImGui(const char* const fontPathName);

	ndInt32 GetWidth() const;
	ndInt32 GetHeight() const;
	
	void ResetScene();
	void Render(ndFloat32 timestep);

	void ClearFrameBuffer(const ndVector& color);
	void AddRenderPass(const ndSharedPtr<ndRenderPass>& renderPass);

	void AddSceneNode(const ndSharedPtr<ndRenderSceneNode>& body);
	void RemoveSceneNode(const ndSharedPtr<ndRenderSceneNode>& body);

	void SetCamera(const ndSharedPtr<ndRenderSceneCamera>& camera);
	void SetSunLight(const ndVector& direction, const ndVector& intensity);

	ndSharedPtr<ndUserCallback>& GetOwner();
	ndSharedPtr<ndRenderSceneCamera>& GetCamera();
	ndSharedPtr<ndRenderSceneCamera> GetCamera() const;
	ndSharedPtr<ndRenderTextureCache>& GetTextureCache();

	void InterpolateTransforms(ndFloat32 param);

	private:
	ndSharedPtr<ndUserCallback> m_owner;
	ndSharedPtr<ndRenderContext> m_context;
	ndSharedPtr<ndRenderSceneCamera> m_camera;
	ndSharedPtr<ndRenderTextureCache> m_textureCache;
	ndList<ndSharedPtr<ndRenderSceneNode>> m_scene;
	ndList<ndSharedPtr<ndRenderPass>> m_renderPasses;

	ndVector m_sunLightDir;
	ndVector m_sunLightAmbient;
	ndVector m_sunLightIntesity;
	ndVector m_backgroundColor;

	ndRenderPassShadowsImplement* m_cachedShadowPass;

	friend class ndRenderContext;
	friend class ndRenderPassGui;
	friend class ndRenderPassColor;
	friend class ndRenderPassShadows;
	friend class ndRenderSceneCamera;
	friend class ndRenderTextureCache;
	friend class ndRenderPassEnvironment;
	friend class ndRenderPassShadowsImplement;
	friend class ndRenderPrimitiveMeshImplement;
};

#endif 

