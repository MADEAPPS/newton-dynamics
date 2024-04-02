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


#ifndef __DEBUG_DISPLAY_H__
#define __DEBUG_DISPLAY_H__

#include "ndSandboxStdafx.h"

class ndDemoEntityManager;

class ndDebugNotify : public ndShapeDebugNotify
{
	public:
	ndDebugNotify(ndDemoEntityManager* const manager = nullptr, ndBodyKinematic* const body = nullptr)
		:ndShapeDebugNotify()
		,m_body(body)
		,m_manager(manager)
	{
	}

	void DrawPolygon(ndInt32, const ndVector* const, const ndEdgeType* const)
	{
	}

	ndBodyKinematic* m_body;
	ndDemoEntityManager* m_manager;
};

inline void RenderParticles(ndDemoEntityManager* const )
{
	ndAssert(0);
}

class ndDebugDisplay
{
	public:
	class ndDebudPass
	{
		public:
		virtual ~ndDebudPass() {}
		virtual void Init(ndDemoEntityManager* const scene) = 0;
		virtual void CleanUp() = 0;
		virtual void Render(ndDemoEntityManager* const scene) = 0;
		virtual void UpdateBuffers(ndPhysicsWorld* const world) = 0;
	};

	class ndContactPoints: public ndDebudPass
	{
		public:
		ndContactPoints();
		~ndContactPoints();

		virtual void Init(ndDemoEntityManager* const scene);
		virtual void CleanUp();

		virtual void Render(ndDemoEntityManager* const scene);
		virtual void UpdateBuffers(ndPhysicsWorld* const world);
		
		ndArray<glVector3> m_points;

		GLuint m_vertexBuffer;
		GLuint m_vertextArrayBuffer;

		GLint m_pixelSizeLocation;
		GLint m_pixelColorLocation;

		GLint m_viewModelMatrixLocation;
		GLint m_projectionMatrixLocation;

		
		ndSpinLock m_lock;
	};

	ndDebugDisplay();
	~ndDebugDisplay();

	void Init(ndDemoEntityManager* const scene);
	void Cleanup();

	void UpdateContactPoints(ndPhysicsWorld* const world);


	void RenderContactPoints(ndDemoEntityManager* const scene);
	//void RenderParticles(ndDemoEntityManager* const scene);
	//void RenderWorldScene(ndDemoEntityManager* const scene);
	//void RenderBodiesAABB(ndDemoEntityManager* const scene);
	//void RenderBodyFrame(ndDemoEntityManager* const scene);
	//void RenderCenterOfMass(ndDemoEntityManager* const scene);
	//
	//void RenderJointsDebugInfo(ndDemoEntityManager* const scene);
	//void RenderModelsDebugInfo(ndDemoEntityManager* const scene);
	//void RenderNormalForces(ndDemoEntityManager* const scene, ndFloat32 scale = 0.005f);
	//void RenderPolygon(ndDemoEntityManager* const scene, const ndVector* const points, ndInt32 count, const ndVector& color);

	ndContactPoints m_contactsPonts;
};


#endif

