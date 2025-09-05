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
#include "ndDemoDebugMesh.h"

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
	//ndAssert(0);
}

class ndDebugDisplay
{
	public:
	class ndColorPoint
	{
		public:
		glVector3 m_point;
		glVector3 m_color;
	};

	class ndDebugPass
	{
		public:
		ndDebugPass();
		virtual ~ndDebugPass();
		
		virtual void Render(ndDemoEntityManager* const scene) = 0;
		virtual void UpdateBuffers(ndDemoEntityManager* const scene) = 0;

		virtual void Init(ndDemoEntityManager* const scene);
		virtual void CleanUp();

		void LoadBufferData(ndArray<ndColorPoint>& data);
		void RenderBuffer(ndDemoEntityManager* const scene, GLenum mode, GLint pointCount, GLuint vertexArrauBuffer, GLuint vertextBuffer);

		ndSpinLock m_lock;
		ndArray<ndColorPoint> m_points;

		GLuint m_shader;
		GLuint m_vertexBuffer;
		GLuint m_vertextArrayBuffer;

		ndInt32 m_frameTick0;
		ndInt32 m_frameTick1;
		GLint m_projectionViewModelMatrixLocation;
	};

	class ndContactPoints: public ndDebugPass
	{
		public:
		virtual void Render(ndDemoEntityManager* const scene);
		virtual void UpdateBuffers(ndDemoEntityManager* const scene);
	};

	class ndCenterOfMass : public ndDebugPass
	{
		public:
		virtual void Render(ndDemoEntityManager* const scene);
		virtual void UpdateBuffers(ndDemoEntityManager* const scene);
		//ndFloat32 m_scale;
	};

	class ndNormalForces : public ndDebugPass
	{
		public:
		virtual void Render(ndDemoEntityManager* const scene);
		virtual void UpdateBuffers(ndDemoEntityManager* const scene);

		ndFloat32 m_scale;
	};

	class ndModelsDebugInfo : public ndDebugPass
	{
		public:
		virtual void Init(ndDemoEntityManager* const scene);
		virtual void CleanUp();

		virtual void Render(ndDemoEntityManager* const scene);
		virtual void UpdateBuffers(ndDemoEntityManager* const scene);

		void LoadBufferData();

		ndArray<ndColorPoint> m_lines;
		ndFloat32 m_scale;
		ndFloat32 m_lineThickness;
		ndFloat32 m_pointThickness;

		GLuint m_lineBuffer;
		GLuint m_lineArrayBuffer;
	};

	class ndJointDebugInfo : public ndModelsDebugInfo
	{
		public:
		virtual void UpdateBuffers(ndDemoEntityManager* const scene);
	};

	class ndShapesDebugInfo : public ndDebugPass
	{
		public:
		class ndDebugMesh
		{
			public:
			ndDebugMesh()
				:m_zBufferShaded()
				,m_flatShaded()
				,m_wireFrameOpenEdge()
				,m_wireFrameShareEdge()
			{
			}

			ndSharedPtr<ndZbufferDebugMesh> m_zBufferShaded;
			ndSharedPtr<ndFlatShadedDebugMesh> m_flatShaded;
			ndSharedPtr<ndWireFrameDebugMesh> m_wireFrameOpenEdge;
			ndSharedPtr<ndWireFrameDebugMesh> m_wireFrameShareEdge;
		};

		class ndDebugMeshCache : public ndTree<ndDebugMesh, const ndShape*>
		{
		};

		void LoadBufferData(ndDemoEntityManager* const scene);

		virtual void Init(ndDemoEntityManager* const scene);
		virtual void CleanUp();
		virtual void Render(ndDemoEntityManager* const scene);
		virtual void UpdateBuffers(ndDemoEntityManager* const scene);
		ndInt32 m_debugMode;
		ndDebugMeshCache m_meshCache;
	};

	ndDebugDisplay();
	~ndDebugDisplay();

	void Init(ndDemoEntityManager* const scene);
	void Cleanup();

	void UpdateCenterOfMass(ndDemoEntityManager* const scene);
	void UpdateNormalForces(ndDemoEntityManager* const scene);
	void UpdateContactPoints(ndDemoEntityManager* const scene);
	void UpdateJointsDebugInfo(ndDemoEntityManager* const scene);
	void UpdateModelsDebugInfo(ndDemoEntityManager* const scene);
	void UpdateDebugShapes(ndDemoEntityManager* const scene, ndInt32 collisionDisplayMode);

	void RenderCenterOfMass(ndDemoEntityManager* const scene);
	void RenderContactPoints(ndDemoEntityManager* const scene);
	void RenderModelsDebugInfo(ndDemoEntityManager* const scene);
	void RenderJointsDebugInfo(ndDemoEntityManager* const scene);
	void RenderNormalForces(ndDemoEntityManager* const scene, ndFloat32 scale = 0.005f);
	void RenderDebugShapes(ndDemoEntityManager* const scene, ndInt32 collisionDisplayMode);

	//void RenderParticles(ndDemoEntityManager* const scene);
	//void RenderWorldScene(ndDemoEntityManager* const scene);
	//void RenderBodiesAABB(ndDemoEntityManager* const scene);
	//void RenderBodyFrame(ndDemoEntityManager* const scene);
	//void RenderPolygon(ndDemoEntityManager* const scene, const ndVector* const points, ndInt32 count, const ndVector& color);

	ndCenterOfMass m_centerOfMass;
	ndNormalForces m_normalForces;
	ndContactPoints m_contactsPonts;
	ndJointDebugInfo m_jointDebugInfo;
	ndModelsDebugInfo m_modelsDebugInfo;
	ndShapesDebugInfo m_shapesDebugInfo;
};


#endif

