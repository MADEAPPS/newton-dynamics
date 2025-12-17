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
#ifndef __ND_DEBUG_DISPLAY_RENDER_PASS_H__
#define __ND_DEBUG_DISPLAY_RENDER_PASS_H__

class ndDemoEntityManager;

class ndDebugDisplayRenderPass : public ndRenderPass
{
	public:
	class ndDebugMesh
	{
		public:
		ndDebugMesh()
			:m_zBuffer()
			,m_flatShaded()
			//,m_wireFrameOpenEdge()
			,m_wireFrameShareEdge()
		{
		}

		ndSharedPtr<ndRenderPrimitive> m_zBuffer;
		ndSharedPtr<ndRenderPrimitive> m_flatShaded;
		ndSharedPtr<ndRenderPrimitive> m_wireFrameShareEdge;
		//ndSharedPtr<ndWireFrameDebugMesh> m_wireFrameOpenEdge;
	};

	ndDebugDisplayRenderPass(ndDemoEntityManager* const owner);
	~ndDebugDisplayRenderPass();

	void SetDebugDisplayOptions();

	private:
	void RenderDebugLines();
	void RenderCollisionShape();
	void GenerateCenterOfMass();

	virtual void ResetScene() override;
	virtual void RenderScene() override;
	ndDebugMesh* CreateRenderPrimitive(const ndShapeInstance& shapeInstance) const;

	ndVector m_awakeColor;
	ndVector m_sleepColor;
	ndDemoEntityManager* m_manager;
	ndTree<ndSharedPtr<ndDebugMesh>, ndShape*> m_meshCache;

	ndInt32 m_showCollisionMeshMode;
	bool m_showCenterOfMass;

	class ndLine
	{
		public:
		ndVector m_p0;
		ndVector m_p1;
		ndVector m_color;
	};
	ndArray<ndLine> m_debugLines;
	ndSharedPtr<ndRenderPrimitive> m_renderLinesPrimitive;
};

#endif