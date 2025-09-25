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

#include "ndSandboxStdafx.h"
#include "ndMeshLoader.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoCameraNode.h"
#include "ndContactCallback.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"

class ndObjectPlacementHelp : public ndDemoEntityManager::ndDemoHelper
{
	virtual void PresentHelp(ndDemoEntityManager* const scene) override
	{
		ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "Basic object placement");
		scene->Print(color, "Right click and drag the location where");
		scene->Print(color, "where you want to place a dynamic body.");
		scene->Print(color, "Left click while keeping right click down");
		scene->Print(color, "to place a dynamics body at the location.");
		scene->Print(color, "An object will spawn only if the location is stable.");
	}
};

class ndObjectPlacementCamera : public ndDemoCameraNode
{
	public:
	enum ndPlacementState
	{
		m_none,
		m_inTraceMode,
		m_releasePlacemnet,
		m_hasValidPlacement,
	};

	class ndShapeCast : public ndConvexCastNotify
	{
		public:
		ndShapeCast(const ndWorld& world, const ndVector& start, const ndVector& end, const ndShapeInstance& shape)
			:ndConvexCastNotify()
		{
			ndMatrix origin(ndGetIdentityMatrix());
			origin.m_posit = start;
			origin.m_posit.m_w = ndFloat32(1.0f);
			world.ConvexCast(*this, shape, origin, end);
		}

		virtual ndUnsigned32 OnRayPrecastAction(const ndBody* const, const ndShapeInstance* const)
		{
			return 1;
		}
	};

	ndObjectPlacementCamera(ndRender* const owner)
		:ndDemoCameraNode(owner)
		,m_placementMatrix(ndGetIdentityMatrix())
		,m_castingShape(nullptr)
		,m_meshPrimitive(nullptr)
		,m_ghostPrimitive(nullptr)
		,m_yaw(ndFloat32(0.0f))
		,m_pitch(ndFloat32(0.0f))
		,m_yawRate(ndFloat32(0.04f))
		,m_pitchRate(ndFloat32(0.02f))
		,m_mousePosX(ndFloat32(0.0f))
		,m_mousePosY(ndFloat32(0.0f))
		,m_frontSpeed(ndFloat32(15.0f))
		,m_sidewaysSpeed(ndFloat32(10.0f))
		,m_state(m_none)
		,m_showIcon(false)
	{
		ndMeshLoader loader;
		ndDemoEntityManager::ndRenderCallback* const renderCallback = (ndDemoEntityManager::ndRenderCallback*)*owner->GetOwner();
		ndDemoEntityManager* const scene = renderCallback->m_owner;

		ndSharedPtr<ndRenderSceneNode> entity(loader.LoadEntity(*scene->GetRenderer(), ndGetWorkingFileName("tpot.fbx")));
		m_castingShape = loader.m_mesh->CreateCollision();

		m_primitiveOffsetMatrix = entity->m_primitiveMatrix;
		m_meshPrimitive = entity->GetPrimitive();

		m_ghostPrimitive = ndSharedPtr<ndRenderPrimitive>(m_meshPrimitive->Clone());
		ndRenderPrimitiveMesh* const ghoshMesh = (ndRenderPrimitiveMesh*)*m_ghostPrimitive;
		for (ndList<ndRenderPrimitiveMeshSegment>::ndNode* ptr = ghoshMesh->m_segments.GetFirst(); ptr; ptr = ptr->GetNext())
		{
			ndRenderPrimitiveMeshSegment& segment = ptr->GetInfo();
			segment.m_material.m_opacity = ndFloat32 (0.3f);
			segment.m_material.m_diffuse = ndVector(1.0f, 1.0f, 1.0f, 1.0f);
			segment.m_material.m_specular = ndVector(0.0f, 0.0f, 0.0f, 1.0f);
			segment.m_material.m_reflection = ndVector(0.0f, 0.0f, 0.0f, 1.0f);
		}
	}

	void Render(const ndRender* const owner, ndFloat32 timestep, const ndMatrix& parentMatrix, ndRenderPassMode renderMode) const override
	{
		ndDemoCameraNode::Render(owner, timestep, parentMatrix, renderMode);

		if (m_showIcon)
		{
			const ndMatrix modelMatrix(m_primitiveOffsetMatrix * m_placementMatrix);
			//m_meshPrimitive->Render(owner, modelMatrix, m_directionalDiffusseShadow);
			m_ghostPrimitive->Render(owner, modelMatrix, m_transparencyBackface);
			m_ghostPrimitive->Render(owner, modelMatrix, m_transparencyFrontface);
		}
	}

	virtual void SetTransform(const ndQuaternion& rotation, const ndVector& position) override
	{
		ndDemoCameraNode::SetTransform(rotation, position);
		const ndMatrix matrix(GetTransform().GetMatrix());
		m_pitch = ndAsin(matrix.m_front.m_y);
		m_yaw = ndAtan2(-matrix.m_front.m_z, matrix.m_front.m_x);
	}

	void TickUpdate(ndFloat32 timestep)
	{
		ndRender* const renderer = GetOwner();
		ndAssert(renderer);
		ndDemoEntityManager::ndRenderCallback* const renderCallback = (ndDemoEntityManager::ndRenderCallback*)*renderer->GetOwner();
		ndDemoEntityManager* const scene = renderCallback->m_owner;

		ndFloat32 mouseX;
		ndFloat32 mouseY;
		scene->GetMousePosition(mouseX, mouseY);

		// slow down the Camera if we have a Body
		ndFloat32 slowDownFactor = scene->IsShiftKeyDown() ? 0.5f / 10.0f : 0.5f;

		ndMatrix targetMatrix(m_transform1.GetMatrix());

		// do camera translation
		if (scene->GetKeyState(ImGuiKey_W))
		{
			targetMatrix.m_posit += targetMatrix.m_front.Scale(m_frontSpeed * timestep * slowDownFactor);
		}
		if (scene->GetKeyState(ImGuiKey_S))
		{
			targetMatrix.m_posit -= targetMatrix.m_front.Scale(m_frontSpeed * timestep * slowDownFactor);
		}
		if (scene->GetKeyState(ImGuiKey_A))
		{
			targetMatrix.m_posit -= targetMatrix.m_right.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
		}
		if (scene->GetKeyState(ImGuiKey_D))
		{
			targetMatrix.m_posit += targetMatrix.m_right.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
		}

		if (scene->GetKeyState(ImGuiKey_Q))
		{
			targetMatrix.m_posit -= targetMatrix.m_up.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
		}

		if (scene->GetKeyState(ImGuiKey_E))
		{
			targetMatrix.m_posit += targetMatrix.m_up.Scale(m_sidewaysSpeed * timestep * slowDownFactor);
		}

		ndMatrix matrix(ndRollMatrix(m_pitch) * ndYawMatrix(m_yaw));
		ndQuaternion newRotation(matrix);
		ndDemoCameraNode::SetTransform(newRotation, targetMatrix.m_posit);

		bool mouseState = !scene->GetCaptured() && (scene->GetMouseKeyState(0) || scene->GetMouseKeyState(1));
		// do camera rotation, only if we do not have anything picked
		if (!UpdatePickBody() && mouseState && (m_state == m_none))
		{
			ndFloat32 mouseSpeedX = mouseX - m_mousePosX;
			ndFloat32 mouseSpeedY = mouseY - m_mousePosY;

			if (ImGui::IsMouseDown(0))
			{
				if (mouseSpeedX > 0.0f)
				{
					m_yaw = ndAnglesAdd(m_yaw, m_yawRate);
				}
				else if (mouseSpeedX < 0.0f)
				{
					m_yaw = ndAnglesAdd(m_yaw, -m_yawRate);
				}

				if (mouseSpeedY > 0.0f)
				{
					m_pitch += m_pitchRate;
				}
				else if (mouseSpeedY < 0.0f)
				{
					m_pitch -= m_pitchRate;
				}
				m_pitch = ndClamp(m_pitch, ndFloat32(-80.0f * ndDegreeToRad), ndFloat32(80.0f * ndDegreeToRad));
			}
		}
		DoPlacement(scene);

		m_mousePosX = mouseX;
		m_mousePosY = mouseY;
	}

	void DoPlacement(ndDemoEntityManager* const scene)
	{
		m_showIcon = false;
		switch (m_state)
		{
			case m_inTraceMode:
			{
				if (!ImGui::IsMouseDown(1))
				{
					m_state = m_none;
				}
				else
				{
					if (TraceLocation(scene) && ImGui::IsMouseDown(0))
					{
						m_state = m_hasValidPlacement;
					}
				}
				break;
			}

			case m_hasValidPlacement:
			{
				// place an object at this location and move to the freeze state
				SpawnObjectAtLocation(scene);
				m_state = m_releasePlacemnet;
				break;
			}

			case m_releasePlacemnet:
			{
				//TraceLocation(scene);
				if (!(ImGui::IsMouseDown(0) && ImGui::IsMouseDown(1)))
				{
					// start the cicle again
					m_state = m_none;
				}
				break;
			}

			case m_none:
			default:
			{
				if (ImGui::IsMouseDown(1))
				{
					m_state = m_inTraceMode;
				}
				break;
			}
		}
	}

	bool InTraceMode() const
	{
		return true;
	}

	ndPlacementState TraceLocation(ndDemoEntityManager* const scene)
	{
		if (!ImGui::IsMouseDown(1))
		{
			return m_none;
		}

		ndMatrix matrixStart;
		if (CastRay(scene, matrixStart))
		{
			//ndWorld* const world = scene->GetWorld();
			//ndMatrix xxx (FindFloor(*world, matrixStart, **m_castingShape, ndFloat32(1.0f)));
			//ndTrace(("(%f %f %f) (%f %f %f)\n", 
			//	xxx.m_posit.m_x, xxx.m_posit.m_y, xxx.m_posit.m_z, 
			//	matrixStart.m_posit.m_x, matrixStart.m_posit.m_y, matrixStart.m_posit.m_z));

			m_showIcon = true;
			m_placementMatrix = matrixStart;
			if (CalculatePlacementMatrix(matrixStart))
			{
				return m_hasValidPlacement;
			}
		}
		return m_inTraceMode;
	}

	//bool CalculatePlacementMatrix(const ndMatrix& matrixStart)
	bool CalculatePlacementMatrix(const ndMatrix& )
	{
		return true;
	}

	bool CastRay(ndDemoEntityManager* const scene, ndMatrix& matrixStart) const
	{
		ndWorld* const world = scene->GetWorld();

		ndFloat32 mouseX;
		ndFloat32 mouseY;
		scene->GetMousePosition(mouseX, mouseY);
		const ndRenderSceneCamera* const camera = FindCameraNode();
		const ndVector p0(camera->ScreenToWorld(ndVector(mouseX, mouseY, ndFloat32(0.0f), ndFloat32(0.0f))));
		const ndVector p1(camera->ScreenToWorld(ndVector(mouseX, mouseY, ndFloat32(1.0f), ndFloat32(0.0f))));

		ndShapeCast caster(*world, p0, p1, **m_castingShape);

		if (caster.m_param <= ndFloat32(1.0f))
		{
			matrixStart = ndGetIdentityMatrix();
			matrixStart.m_posit = p0 + (p1 - p0).Scale(caster.m_param);
			matrixStart.m_posit.m_w = ndFloat32(1.0f);
			return true;
		}
		return false;
	}

	void SpawnObjectAtLocation(ndDemoEntityManager* const scene)
	{
		//ndTrace(("place object\n"));
		ndPhysicsWorld* const world = scene->GetWorld();

		ndSharedPtr<ndRenderSceneNode>entity(new ndRenderSceneNode(m_placementMatrix));
		entity->SetPrimitive(m_meshPrimitive);
		entity->SetPrimitiveMatrix(m_primitiveOffsetMatrix);

		ndFloat32 mass = ndFloat32 (10.0f);
		ndSharedPtr<ndBody> body(new ndBodyDynamic());
		body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
		body->SetMatrix(m_placementMatrix);
		body->GetAsBodyKinematic()->SetCollisionShape(**m_castingShape);
		body->GetAsBodyKinematic()->SetMassMatrix(mass, **m_castingShape);

		world->AddBody(body);
		scene->AddEntity(entity);
	}

	// for now just sone mesh
	ndMatrix m_placementMatrix;
	ndMatrix m_primitiveOffsetMatrix;
	ndSharedPtr<ndShapeInstance> m_castingShape;
	ndSharedPtr<ndRenderPrimitive> m_meshPrimitive;
	ndSharedPtr<ndRenderPrimitive> m_ghostPrimitive;
	ndFloat32 m_yaw;
	ndFloat32 m_pitch;
	ndFloat32 m_yawRate;
	ndFloat32 m_pitchRate;
	ndFloat32 m_mousePosX;
	ndFloat32 m_mousePosY;
	ndFloat32 m_frontSpeed;
	ndFloat32 m_sidewaysSpeed;
	ndPlacementState m_state;
	bool m_showIcon;
};

void ndObjectPlacement(ndDemoEntityManager* const scene)
{
	// build a floor
	ndSharedPtr<ndBody> bodyFloor(BuildFloorBox(scene, ndGetIdentityMatrix(), "blueCheckerboard.png", 0.1f, true));

	class PlaceMatrix : public ndMatrix
	{
		public:
		PlaceMatrix(ndFloat32 x, ndFloat32 y, ndFloat32 z)
			:ndMatrix(ndGetIdentityMatrix())
		{
			m_posit.m_x = x;
			m_posit.m_y = y;
			m_posit.m_z = z;
		}
	};
	AddBox(scene, PlaceMatrix(0.0f, 20.0f, -3.0f), 10.0f, 1.0f, 1.0f, 1.6f);
	AddSphere(scene, PlaceMatrix(0.0f, 5.0f, 0.0f), 10.0f, 0.5f);
	AddCapsule(scene, PlaceMatrix(0.0f, 5.0f, 3.0f), 10.0f, 0.25f, 0.7f, 10.0f);
	AddConvexHull(scene, PlaceMatrix(-2.0f, 5.0f, -5.0f), 7.0f, 1.0f, 1.5f, 10);
	AddConvexHull(scene, PlaceMatrix(-2.0f, 5.0f,  5.0f), 10.0f, 1.0f, 1.5f, 15);
	AddConvexHull(scene, PlaceMatrix( 2.0f, 5.0f,  3.0f), 30.0f, 1.0f, 1.5f, 20);

	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndObjectPlacementHelp());
	scene->SetDemoHelp(demoHelper);

	// set a special object placemnet Camera;
	ndRender* const renderer = *scene->GetRenderer();
	ndSharedPtr<ndRenderSceneNode> camera(new ndObjectPlacementCamera(renderer));
	renderer->SetCamera(camera);

	ndQuaternion rot;
	ndVector origin(-20.0f, 5.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}
