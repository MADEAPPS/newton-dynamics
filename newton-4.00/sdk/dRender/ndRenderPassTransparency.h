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
#ifndef __ND_RENDER_PASS_COLOR_TRANSPARENCY_H__
#define __ND_RENDER_PASS_COLOR_TRANSPARENCY_H__

#include "ndRenderPass.h"

class ndRenderPassTransparency: public ndRenderPass
{
	public:
	ndRenderPassTransparency(ndRender* const owner);
	virtual ~ndRenderPassTransparency();

	virtual void RenderScene(ndFloat32 timestep) override;
};

#endif