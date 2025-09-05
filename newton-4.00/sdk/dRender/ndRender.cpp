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
#include "ndRenderPass.h"
#include "ndRenderTexture.h"
#include "ndRenderContext.h"
#include "ndRenderSceneNode.h"
#include "ndRenderSceneCamera.h"
#include "ndRenderTextureCache.h"

ndRender::ndRender(ndSharedPtr<ndUserCallback>& owner, ndInt32 width, ndInt32 height, const char* const title)
	:ndClassAlloc()
	,m_owner(owner)
	,m_context(nullptr)
	,m_camera(nullptr)
	,m_textureCache(nullptr)
	,m_scene()
	,m_renderPasses()
	,m_sunLightDir(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(1.0f))
	,m_sunLightAmbient(ndFloat32(0.2f), ndFloat32(0.2f), ndFloat32(0.2f), ndFloat32(0.0f))
	,m_sunLightIntesity(ndFloat32(0.7f), ndFloat32(0.7f), ndFloat32(0.7f), ndFloat32(0.0f))
	,m_backgroundColor(ndFloat32(0.45f), ndFloat32(0.55f), ndFloat32(0.60f), ndFloat32(1.0f))
{
	m_context = ndSharedPtr<ndRenderContext>(new ndRenderContext(this, width, height, title));

	m_camera = ndSharedPtr< ndRenderSceneCamera>(new ndRenderSceneCamera(this));
	m_textureCache = ndSharedPtr<ndRenderTextureCache>(new ndRenderTextureCache(this));
}

ndRender::~ndRender()
{
}

void ndRender::SetTitle(const char* const title)
{
	m_context->SetTitle(title);
}

void ndRender::InitImGui(const char* const fontPathName)
{
	m_context->InitImGui(fontPathName);
}

void ndRender::SetSunLight(const ndVector& direction, const ndVector& intensity)
{
	m_sunLightIntesity = intensity & ndVector::m_triplexMask;
	m_sunLightDir = (direction & ndVector::m_triplexMask).Normalize();
}

ndSharedPtr<ndRender::ndUserCallback>& ndRender::GetOwner()
{
	return m_owner;
}

void ndRender::SetCamera(const ndSharedPtr<ndRenderSceneCamera>& camera)
{
	m_camera = camera;
}

ndSharedPtr<ndRenderSceneCamera>& ndRender::GetCamera()
{
	return m_camera;
}

ndSharedPtr<ndRenderSceneCamera> ndRender::GetCamera() const
{
	return m_camera;
}

ndSharedPtr<ndRenderTextureCache>& ndRender::GetTextureCache()
{
	return m_textureCache;
}

ndInt32 ndRender::GetWidth() const
{
	return m_context->GetWidth();
}

ndInt32 ndRender::GetHeight() const
{
	return m_context->GetHeight();
}

bool ndRender::ShouldFinish() const
{
	return m_context->ShouldFinish();
}

void ndRender::Terminate()
{
	m_context->Terminate();
}

bool ndRender::PollEvents() const
{
	return m_context->PollEvents();
}

void ndRender::Present()
{
	m_context->Present();
}

void ndRender::AddRenderPass(const ndSharedPtr<ndRenderPass>& renderPass)
{
	ndAssert(renderPass->m_owner == this);
	m_renderPasses.Append(renderPass);
}

void ndRender::ClearFrameBuffer(const ndVector& color)
{
	m_context->ClearFrameBuffer(color);
}

void ndRender::ResetScene()
{
	m_scene.RemoveAll();
	for (ndList<ndSharedPtr<ndRenderPass>>::ndNode* node = m_renderPasses.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndRenderPass>& pass = node->GetInfo();
		pass->ResetScene();
	}
}

void ndRender::AddSceneNode(const ndSharedPtr<ndRenderSceneNode>& node)
{
	ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* const handle = m_scene.Append(node);
	ndAssert(!node->m_owner);
	ndAssert(!node->m_sceneHandle);
	node->m_owner = this;
	node->m_sceneHandle = handle;
}

void ndRender::RemoveSceneNode(const ndSharedPtr<ndRenderSceneNode>& node)
{
	ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* const handle = node->m_sceneHandle;
	if (handle)
	{
		node->m_owner = nullptr;
		node->m_sceneHandle = nullptr;
		m_scene.Remove(handle);
	}
}

void ndRender::Render(ndFloat32 timestep)
{
	m_context->BeginFrame();
	m_context->ClearFrameBuffer(m_backgroundColor);

	ImGuiIO& io = ImGui::GetIO();
	ndInt32 fb_width = (ndInt32)(io.DisplaySize.x * io.DisplayFramebufferScale.x);
	ndInt32 fb_height = (ndInt32)(io.DisplaySize.y * io.DisplayFramebufferScale.y);
	if (!(fb_width == 0 || fb_height == 0))
	{
		ndInt32 display_w = m_context->GetWidth();
		ndInt32 display_h = m_context->GetHeight();
		m_camera->SetViewMatrix(display_w, display_h);

		if (m_renderPasses.GetCount())
		{
			for (ndList<ndSharedPtr<ndRenderPass>>::ndNode* node = m_renderPasses.GetFirst(); node; node = node->GetNext())
			{
				const ndSharedPtr<ndRenderPass>& pass = node->GetInfo();
				pass->RenderScene(timestep);
			}
		}
		else
		{
			m_context->EndFrame();
		}
	}
	Present();
}

void ndRender::InterpolateTransforms(ndFloat32 param)
{
	m_camera->InterpolateTransforms(param);
	for (ndList<ndSharedPtr<ndRenderSceneNode>>::ndNode* node = m_scene.GetFirst(); node; node = node->GetNext())
	{
		ndRenderSceneNode* const sceneNode = *node->GetInfo();
		sceneNode->InterpolateTransforms(param);
	}
}