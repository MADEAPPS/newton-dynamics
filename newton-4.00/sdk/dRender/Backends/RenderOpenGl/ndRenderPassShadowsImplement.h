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
#ifndef __ND_RENDER_PASS_SHADOWS_IMPLEMNETH__
#define __ND_RENDER_PASS_SHADOWS_IMPLEMNETH__

#include "ndRenderPass.h"

class ndRenderPassShadowsImplement : public ndClassAlloc
{
	public:
	ndRenderPassShadowsImplement(ndRenderContext* const context);
	virtual ~ndRenderPassShadowsImplement();

	//virtual void RenderScene(ndFloat32 timestep) override;

	ndMatrix m_lightProjectToTextureSpace;
	ndVector m_viewPortTiles[4];
	ndRenderContext* m_context;

	ndInt32 m_width;
	ndInt32 m_height;
	GLuint m_shadowMapTexture;
	GLuint m_frameBufferObject;
	GLuint m_modelProjectionMatrixLocation;

};

#endif