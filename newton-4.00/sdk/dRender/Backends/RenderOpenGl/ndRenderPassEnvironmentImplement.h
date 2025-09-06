/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/
#ifndef __ND_ENVIRONMENT_RENDER_PASS_IMPLEMENT_H__
#define __ND_ENVIRONMENT_RENDER_PASS_IMPLEMENT_H__

#include "ndRenderPass.h"
#include "ndRenderContext.h"

class ndRenderSceneCamera;

class ndRenderPassEnvironmentImplement : public ndClassAlloc
{
	public:
	ndRenderPassEnvironmentImplement(ndRenderContext* const context);
	~ndRenderPassEnvironmentImplement();
	void RenderScene(const ndRenderSceneCamera* const camera, const ndRenderTexture* const texture);

	ndRenderContext* m_context;
	GLuint m_indexBuffer;
	GLuint m_vertexBuffer;
	GLuint m_vertextArrayBuffer;
	GLint m_invViewModelProjectionTextureMatrix;
};

#endif