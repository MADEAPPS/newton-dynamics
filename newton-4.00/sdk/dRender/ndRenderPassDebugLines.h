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
#ifndef __ND_RENDER_PASS_DEBUG_LINES_H__
#define __ND_RENDER_PASS_DEBUG_LINES_H__

#include "ndRenderPass.h"

class ndRenderPrimitive;

class ndRenderPassDebugLines : public ndRenderPass
{
	public:
	class ndLine
	{
		public:
		ndVector m_point;
		ndVector m_color;
	};

	class ndDebugLineOptions
	{
		public:
		bool m_showCentreOfMass;
	};

	ndRenderPassDebugLines(ndRender* const owner, ndWorld* const world);
	~ndRenderPassDebugLines();

	void SetDebugDisplayOptions(const ndDebugLineOptions& options);

	protected:
	void RenderDebugLines();
	void GenerateCenterOfMass();
	virtual void RenderScene() override;
	
	ndDebugLineOptions m_options;

	ndWorld* m_world;
	ndArray<ndLine> m_debugLines;
	ndSharedPtr<ndRenderPrimitive> m_renderLinesPrimitive;
};

#endif