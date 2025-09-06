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

ndRenderContext::ndRenderContext(ndRender* const owner, ndInt32, ndInt32, const char* const)
	:ndClassAlloc()
	,m_owner (owner)
	,m_shaderCache(nullptr)
	,m_prevKey(0)
	,m_imGuiEnabled(false)
{
}

ndRenderContext::~ndRenderContext()
{
}

void ndRenderContext::SetTitle(const char* const)
{
}

void ndRenderContext::Terminate()
{
}

void ndRenderContext::SetViewport(ndInt32, ndInt32)
{
}

void ndRenderContext::EndGuiRenderStates()
{
}

void ndRenderContext::InitImGui(const char* const)
{
}

void ndRenderContext::SetInputCallbacks()
{
}

void ndRenderContext::LoadFont(const char* const)
{
}

bool ndRenderContext::ShouldFinish() const
{
	return false;
}

ndInt32 ndRenderContext::GetWidth() const
{
	return 0;
}

ndInt32 ndRenderContext::GetHeight() const
{
	return 0;
}

bool ndRenderContext::PollEvents() const
{
	return 0;
}

void ndRenderContext::Present() const
{
}

void ndRenderContext::BeginFrame()
{
}

void ndRenderContext::EndFrame()
{
}


void ndRenderContext::SetGuiRenderStates()
{
}

void ndRenderContext::SetCollorPassRenderStates()
{
}

void ndRenderContext::ClearFrameBuffer(const ndVector&)
{
}

ndSharedPtr<ndRenderTexture> ndRenderContext::LoadTexture(const ndString&)
{
	return ndSharedPtr<ndRenderTexture>();
}

ndSharedPtr<ndRenderTexture> ndRenderContext::LoadCubeMap(const ndFixSizeArray<ndString, 6>&)
{
	return ndSharedPtr<ndRenderTexture>();
}
