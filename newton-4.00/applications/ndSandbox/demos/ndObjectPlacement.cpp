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
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoCameraNode.h"
#include "ndContactCallback.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"


#if 0
class NewtonPhantom : public ndModelNotify
{
	class PhantomPlacement : public ndDemoEntity
	{
		public:
		PhantomPlacement(ndDemoEntityManager* const scene)
			:ndDemoEntity(ndGetIdentityMatrix())
		{
			//dAssert (0);
	
			//ndWorld* const world = scene->GetWorld();
			ndMatrix matrix(ndGetIdentityMatrix());
			//DemoEntity* const cowEntity = DemoEntity::LoadNGD_mesh("teapot.fbx", world, scene->GetShaderCache());
			//NewtonMesh* const cowMesh = cowEntity->GetMesh()->CreateNewtonMesh(world, dGetIdentityMatrix());
			
			ndMeshLoader loader;
			ndSharedPtr<ndDemoEntity> entity(loader.LoadEntity("teapot.fbx", scene));

			//NewtonCollision* const shape = NewtonCreateConvexHullFromMesh(world, cowMesh, 0, 0);
			//m_phantom = NewtonCreateKinematicBody(world, shape, &matrix[0][0]);
			ndSharedPtr<ndShapeInstance> shape(CreateConvexHull(*entity));
			
			//m_solideMesh = (DemoMesh*)cowEntity->GetMesh();
			//m_solideMesh->AddRef();
			m_redMesh = CreatePhantomMesh(scene, *shape, ndVector(1.0f, 0.0f, 0.0f, 1.0f));
			m_blueMesh = CreatePhantomMesh(scene, *shape, ndVector(0.0f, 0.0f, 0.5f, 1.0f));
			//SetMesh(m_redMesh, dGetIdentityMatrix());
			//
			//NewtonBodySetUserData(m_phantom, this);
			//NewtonBodySetMassProperties(m_phantom, 10.0f, shape);
			//NewtonBodySetTransformCallback(m_phantom, DemoEntity::TransformCallback);
			//
			//delete cowEntity;
			//NewtonMeshDestroy(cowMesh);
			//NewtonDestroyCollision(shape);
		}
	
		~PhantomPlacement()
		{	
			//m_redMesh->Release();
			//m_blueMesh->Release();
			//m_solideMesh->Release();
		}
	
		ndDemoMesh* CreatePhantomMesh(ndDemoEntityManager* const scene, ndShapeInstance* const shape, const ndVector& color)
		{
			//DemoMesh* const mesh = new DemoMesh("primitive", scene->GetShaderCache(), shape, "smilli.png", "smilli.png", "smilli.png", 0.5f);
			ndDemoMesh* const mesh = new ndDemoMesh("primitive", scene->GetShaderCache(), shape, "smilli.png", "smilli.png", "smilli.png", 0.5f);
			
			ndDemoSubMesh& subMesh = mesh->GetFirst()->GetInfo();
			subMesh.m_material.m_specular = color;
			subMesh.m_material.m_diffuse = color;
			subMesh.m_material.m_ambient = color;
			//mesh->OptimizeForRender();
			return mesh;
		}

		ndShapeInstance* CreateConvexHull(ndDemoEntity* const entity) const
		{
			ndArray<ndVector> points;
			const ndDemoMesh* const mesh = (ndDemoMesh*)*entity->GetMesh();
			if (mesh)
			{
				ndAssert(0);
				return nullptr;
			}

			ndMatrix parentMatrix[32];
			ndDemoEntity* stackMem[32];
			ndInt32 stack = 0;
			//for (ndDemoEntity* child = entity->GetFirstChild(); child; child = child->GetNext())
			for (ndList<ndSharedPtr<ndDemoEntity>>::ndNode* node = entity->GetChildren().GetFirst(); node; node = node->GetNext())
			{
				stackMem[stack] = *node->GetInfo();
				parentMatrix[stack] = ndGetIdentityMatrix();
				stack++;
			}

			while (stack)
			{
				stack--;
				ndDemoEntity* const ent = stackMem[stack];
				const ndMatrix matrix(ent->GetCurrentMatrix() * parentMatrix[stack]);
				const ndDemoMesh* const entMesh = (ndDemoMesh*)*ent->GetMesh();
				ndArray<ndVector> localPoints;
				entMesh->GetVertexArray(localPoints);

				ndMatrix meshMatrix(ent->GetMeshMatrix() * matrix);
				for (ndInt32 i = 0; i < localPoints.GetCount(); ++i)
				{
					ndVector p(meshMatrix.TransformVector(localPoints[i]));
					points.PushBack(p);
				}

				//for (ndDemoEntity* child = ent->GetFirstChild(); child; child = child->GetNext())
				for (ndList<ndSharedPtr<ndDemoEntity>>::ndNode* node = ent->GetChildren().GetFirst(); node; node = node->GetNext())
				{
					stackMem[stack] = *node->GetInfo();
					parentMatrix[stack] = matrix;
					stack++;
				}
			}

			if (!points.GetCount())
			{
				return nullptr;
			}

			ndShapeInstance* const instance = new ndShapeInstance(new ndShapeConvexHull(ndInt32(points.GetCount()), sizeof(ndVector), 0.01f, &points[0].m_x));
			const ndMatrix matrix(entity->GetMeshMatrix());
			instance->SetLocalMatrix(matrix);
			return instance;
		}

		//DemoMesh* CreatePhantomMesh(DemoEntityManager* const scene, NewtonCollision* const shape, const dVector& color)
		//{
		//	DemoMesh* const mesh = new DemoMesh("primitive", scene->GetShaderCache(), shape, "smilli.png", "smilli.png", "smilli.png", 0.5f);
		//	DemoSubMesh& subMesh = mesh->GetFirst()->GetInfo();
		//	subMesh.m_specular = color;
		//	subMesh.m_diffuse = color;
		//	subMesh.m_ambient = color;
		//	mesh->OptimizeForRender();
		//	return mesh;
		//}
	
		//void SetPhantomMesh(bool redOrBlue)
		//{
		//	redOrBlue ? SetMesh(m_redMesh, dGetIdentityMatrix()) : SetMesh(m_blueMesh, dGetIdentityMatrix());
		//}
	
		//NewtonBody* m_phantom;
		//DemoMesh* m_solideMesh;
		ndSharedPtr<ndDemoMeshInterface> m_redMesh;
		ndSharedPtr<ndDemoMeshInterface> m_blueMesh;
	};

	// A Phantom collision shape can be moved around the world, gathering contact
	// information with other ndBody's without effecting the simulation
	class ndRayPickingCallback : public ndRayCastClosestHitCallback
	{
		public:
		ndRayPickingCallback()
			:ndRayCastClosestHitCallback()
		{
		}

		ndFloat32 OnRayCastAction(const ndContactPoint& contact, ndFloat32 intersetParam)
		{
			return ndRayCastClosestHitCallback::OnRayCastAction(contact, intersetParam);
		}
	};

	//class ndBodiesInAabbNotify : public ndBodiesInAabbNotify
	//{
	//};

	public:
	NewtonPhantom(ndDemoEntityManager* const scene)
		:ndModelNotify()
		//,phantomShape(new ndShapeBox(1.0f, 1.0f, 1.0f))
		,worldMatrix(ndGetIdentityMatrix())
		//,notification(scene)
		,m_phantomEntity(ndSharedPtr<ndDemoEntity>(new PhantomPlacement(scene)))
	{
		//contactPoint = ndVector(1.0e20f);
		scene->AddEntity(m_phantomEntity);
	}

	~NewtonPhantom()
	{
	}

	virtual ndModelNotify* Clone() const override
	{
		ndAssert(0);
		return nullptr;
	}

	void transform(const ndMatrix& matrix) 
	{ 
		worldMatrix = matrix; 
	}

	//ndInt32 getContactCount() const { return contactCount; }
	//ndVector getContactPoint() const { return contactPoint; }

	void Update(ndFloat32) override
	{
		ndPhysicsWorld* const world = (ndPhysicsWorld*)GetModel()->GetWorld();
		ndDemoEntityManager* const scene = world->GetManager();
		ndDemoCamera* const camera = scene->GetCamera();

		ndFloat32 mouseX;
		ndFloat32 mouseY;
		int buttonState0;
		int buttonState1;
		scene->GetMousePosition(mouseX, mouseY);
		buttonState0 = scene->GetMouseKeyState(0) ? 1 : 0;
		buttonState1 = scene->GetMouseKeyState(1) ? 1 : 0;

		if (buttonState1)
		{
			ndVector p0(camera->ScreenToWorld(ndVector(mouseX, mouseY, 0.0f, 0.0f)));
			ndVector p1(camera->ScreenToWorld(ndVector(mouseX, mouseY, 1.0f, 0.0f)));

			ndRayPickingCallback rayCaster;
			if (world->RayCast(rayCaster, p0, p1))
			{
				ndTrace(("%f %f\n", mouseX, mouseY));
				worldMatrix.m_posit = p0 + (p1 - p0).Scale(rayCaster.m_param);
				worldMatrix.m_posit.m_w = 1.0f;

				PhantomPlacement* const phatom = (PhantomPlacement*)*m_phantomEntity;
				m_phantomEntity->SetMesh(phatom->m_blueMesh, ndGetIdentityMatrix());
				
				//// calc the current AABB in world space
				//ndVector boxMin;
				//ndVector boxMax;
				//phantomShape.CalculateAabb(worldMatrix, boxMin, boxMax);
				//
				//ndBodiesInAabbNotify notifyCallback;
				//world->BodiesInAabb(notifyCallback, boxMin, boxMax);
				//for (ndInt32 i = 0; i < notifyCallback.m_bodyArray.GetCount(); ++i)
				//{
				//	ndBody* const nbody = const_cast<ndBody*> (notifyCallback.m_bodyArray[i]);
				//	ndBodyKinematic* const kBody = nbody->GetAsBodyKinematic();
				//
				////	const ndShapeInstance& otherShape = kBody->GetCollisionShape();
				////	const ndMatrix& otherMatrix = notifyCallback.m_bodyArray[i]->GetMatrix();
				////
				////	// ignore self collision
				////	if (otherShape.GetShape() != phantomShape.GetShape())
				////	{
				////		ndFixSizeArray<ndContactPoint, 16> contactBuffer;
				////
				////		ndVector phantomVelocity = ndVector::m_zero;
				////		ndVector otherVelocity = ndVector::m_zero;
				////
				////		ndContactSolver contSolver;
				////		contSolver.CalculateContacts(&phantomShape, worldMatrix, phantomVelocity, &otherShape, otherMatrix, otherVelocity, contactBuffer, &notification);
				////		contactCount = contactBuffer.GetCount();
				////
				////		// 
				////		std::cout << contactCount << std::endl;
				////
				////
				////		if (contactCount)
				////		{
				////			for (int j = 0; j < contactCount; ++j)
				////			{
				////				const ndContactPoint& cPnt = contactBuffer[j];
				////				contactPoint = cPnt.m_point;
				////			}
				////		}
				////	}
				//}
			}
		}
		else
		{
			m_phantomEntity->SetMesh(nullptr, ndGetIdentityMatrix());
		}
	}

	void PostUpdate(ndFloat32) override
	{
	}

	void PostTransformUpdate(ndFloat32) override
	{
		m_phantomEntity->SetMatrix(ndQuaternion(worldMatrix), worldMatrix.m_posit);
	}

	private:
	//ndShapeInstance phantomShape;
	//PhantomPlacement* m_phantomEntity;
	ndMatrix worldMatrix;
	ndSharedPtr<ndDemoEntity> m_phantomEntity;

	//ndContactNotify notification;
	//ndInt32 contactCount = 0;
	//ndVector contactPoint;
}; // end class NewtonPhantom

#endif

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
		m_inTraceMode,
		m_hasValidPlacement,
		m_none,
	};

	class ndSphereCast : public ndConvexCastNotify
	{
		public:
		ndSphereCast(const ndWorld& world, const ndVector& start, const ndVector& end, const ndShapeInstance& shape)
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
		,m_castingSphere(new ndShapeSphere(ndFloat32(0.125f)))
		,m_placementColor(0.0f, 0.0f, 0.0f, 0.0f)
		,m_validPlacement(0.0f, 0.0f, 1.0f, 0.0f)
		,m_invalidPlacement(1.0f, 0.0f, 0.0f, 0.0f)
		,m_yaw(ndFloat32(0.0f))
		,m_pitch(ndFloat32(0.0f))
		,m_yawRate(ndFloat32(0.04f))
		,m_pitchRate(ndFloat32(0.02f))
		,m_mousePosX(ndFloat32(0.0f))
		,m_mousePosY(ndFloat32(0.0f))
		,m_frontSpeed(ndFloat32(15.0f))
		,m_sidewaysSpeed(ndFloat32(10.0f))
		,m_state(m_none)
	{
	}

	void Render(const ndRender* const owner, ndFloat32 timeStep, const ndMatrix& parentMatrix, ndRenderPassMode renderMode) const override
	{
		ndDemoCameraNode::Render(owner, timeStep, parentMatrix, renderMode);

		// render the object placemnet Icon
		//const ndMatrix modelMatrix(m_primitiveMatrix * nodeMatrix);
		//mesh->Render(owner, modelMatrix, renderMode);
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
		if (mouseState)
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
		m_placementColor = m_invalidPlacement;
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
					TraceLocation(scene);
				}
				break;
			}

			case m_hasValidPlacement:
			{
				ndAssert(0);
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
			m_showIcon = true;
			if (CalculatePlacementMatrix(matrixStart))
			{
				m_placementColor = m_validPlacement;
				return m_hasValidPlacement;
			}
		}
		return m_inTraceMode;
	}

	bool CalculatePlacementMatrix(const ndMatrix matrixStart)
	{
		return false;
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

		ndSphereCast caster(*world, p0, p1, m_castingSphere);

		if (caster.m_param <= ndFloat32(1.0f))
		{
			matrixStart = ndGetIdentityMatrix();
			matrixStart.m_posit = p0 + (p1 - p0).Scale(caster.m_param);
			matrixStart.m_posit.m_w = ndFloat32(0.0f);
			return true;
		}
		return false;
	}

	ndShapeInstance m_castingSphere;
	ndVector m_placementColor;
	ndVector m_validPlacement;
	ndVector m_invalidPlacement;
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

	//class PlaceMatrix : public ndMatrix
	//{
	//	public:
	//	PlaceMatrix(ndFloat32 x, ndFloat32 y, ndFloat32 z)
	//		:ndMatrix(ndGetIdentityMatrix())
	//	{
	//		m_posit.m_x = x;
	//		m_posit.m_y = y;
	//		m_posit.m_z = z;
	//	}
	//};
	//
	//
	//AddBox(scene, PlaceMatrix(0.0f, 20.0f, -3.0f), 10.0f, 1.0f, 1.0f, 1.6f);
	//AddSphere(scene, PlaceMatrix(0.0f, 5.0f, 0.0f), 10.0f, 0.5f);
	//AddCapsule(scene, PlaceMatrix(0.0f, 5.0f, 3.0f), 10.0f, 0.25f, 0.7f, 10.0f);
	//AddConvexHull(scene, PlaceMatrix(-2.0f, 5.0f, -2.0f), 7.0f, 1.0f, 1.5f, 10);
	//AddConvexHull(scene, PlaceMatrix(-2.0f, 5.0f,  2.0f), 10.0f, 1.0f, 1.5f, 20);
	//AddConvexHull(scene, PlaceMatrix( 2.0f, 5.0f,  3.0f), 30.0f, 1.0f, 1.5f, 40);

	// create a Phantom model that contains a collision shape and transform matrix
	//ndSharedPtr<ndModel> phantomPtr(new ndModel);
	//phantomPtr->SetNotifyCallback(new NewtonPhantom(scene));
	//scene->GetWorld()->AddModel(phantomPtr);

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
