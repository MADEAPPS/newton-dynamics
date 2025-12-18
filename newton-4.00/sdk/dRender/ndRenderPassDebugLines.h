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
		ndDebugLineOptions()
		{
			memset(this, 0, sizeof(ndDebugLineOptions));
		}

		bool m_showBodyAABB;
		bool m_showBodyFrame;
		bool m_showBroadPhase;
		bool m_showCentreOfMass;
		bool m_showJointDebugInfo;
		bool m_showModelsDebugInfo;
	};

	ndRenderPassDebugLines(ndRender* const owner, ndWorld* const world);
	~ndRenderPassDebugLines();

	const ndArray<ndLine>& GetVertex() const;
	void SetDebugDisplayOptions(const ndDebugLineOptions& options);

	protected:
	class ndCallback;
	void RenderDebugLines();
	void GenerateBodyAABB();
	void GenerateBroadphase();
	void GenerateBodyFrames();
	void GenerateJointsDebug();
	void GenerateModelsDebug();
	void GenerateCenterOfMass();
	virtual void RenderScene() override;

	

	ndDebugLineOptions m_options;

	ndWorld* m_world;
	ndArray<ndLine> m_debugLines;
	ndSharedPtr<ndRenderPrimitive> m_renderLinesPrimitive;
};

#endif