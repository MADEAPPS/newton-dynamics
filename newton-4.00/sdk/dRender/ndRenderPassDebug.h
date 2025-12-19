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
#ifndef __ND_RENDER_PASS_DEBUG_H__
#define __ND_RENDER_PASS_DEBUG_H__

#include "ndRenderPass.h"

class ndRenderPrimitive;

class ndRenderPassDebug : public ndRenderPass
{
	public:
	class ndPoint
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

		bool m_showContacts;
		bool m_showBodyAABB;
		bool m_showBodyFrame;
		bool m_showBroadPhase;
		bool m_showCentreOfMass;
		bool m_showJointDebugInfo;
		bool m_showModelsDebugInfo;
	};

	ndRenderPassDebug(ndRender* const owner, ndWorld* const world);
	~ndRenderPassDebug();

	const ndArray<ndPoint>& GetVertex() const;
	const ndArray<ndPoint>& GetPoints() const;
	void SetDebugDisplayOptions(const ndDebugLineOptions& options);

	protected:
	class ndCallback;
	void GenerateBodyAABB();
	void GenerateContacts();
	void GenerateBroadphase();
	void GenerateBodyFrames();
	void GenerateJointsDebug();
	void GenerateModelsDebug();
	void GenerateCenterOfMass();
	virtual void RenderScene() override;
	
	ndDebugLineOptions m_options;
	ndArray<ndPoint> m_debugLines;
	ndArray<ndPoint> m_debugPoints;
	ndSharedPtr<ndRenderPrimitive> m_renderLinesPrimitive;
	ndSharedPtr<ndRenderPrimitive> m_renderPointsPrimitive;
	ndWorld* m_world;
};

#endif