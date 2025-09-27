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
#ifndef __ND_RENDER_PASS_SHADOWS_H__
#define __ND_RENDER_PASS_SHADOWS_H__

#include "ndRenderPass.h"

class ndRenderPassShadowsImplement;

class ndRenderPassShadows: public ndRenderPass
{
	public:
	ndRenderPassShadows(ndRender* const owner);
	virtual ~ndRenderPassShadows();

	virtual void RenderScene() override;

	ndSharedPtr<ndRenderPassShadowsImplement> m_implement;
};

#endif