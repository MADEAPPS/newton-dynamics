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
#include "ndSkyBox.h"
#include "ndDemoMesh.h"
#include "ndMeshLoader.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndContactCallback.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndArchimedesBuoyancyVolume.h"

static void AddBox(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 density, ndFloat32 mass)
{
	ndSharedPtr<ndBody> body = AddBox(scene, origin, mass, 1.0f, 1.0f, 1.0f);
	ndShapeMaterial material;
	material.m_userParam[ndDemoContactCallback::m_density].m_floatData = density;
	body->GetAsBodyKinematic()->GetCollisionShape().SetMaterial(material);
}

static void AddSphere1(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 density, ndFloat32 mass)
{
	ndSharedPtr<ndBody> body = AddSphere(scene, origin, mass, 0.5f);
	ndShapeMaterial material;
	material.m_userParam[ndDemoContactCallback::m_density].m_floatData = density;
	body->GetAsBodyKinematic()->GetCollisionShape().SetMaterial(material);
}

static void AddCapsule(ndDemoEntityManager* const scene, const ndMatrix& origin, ndFloat32 density, ndFloat32 mass)
{
	ndSharedPtr<ndBody> body = AddCapsule(scene, origin, mass, 0.5f, 0.5f, 1.0f);
	ndShapeMaterial material;
	material.m_userParam[ndDemoContactCallback::m_density].m_floatData = density;
	body->GetAsBodyKinematic()->GetCollisionShape().SetMaterial(material);
}

static void AddConvexHull(ndDemoEntityManager* const scene, const ndMatrix& origin, const ndInt32 segments, ndFloat32 radius, ndFloat32 high, ndFloat32 density, ndFloat32 mass)
{
	ndSharedPtr<ndBody> body = AddConvexHull(scene, origin, mass, radius, high, segments);
	ndShapeMaterial material;
	material.m_userParam[ndDemoContactCallback::m_density].m_floatData = density;
	body->GetAsBodyKinematic()->GetCollisionShape().SetMaterial(material);
}

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

void ndObjectPlacement(ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildFloorBox(scene, ndGetIdentityMatrix());
	BuildFlatPlane(scene, true);
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

	AddBox(scene, PlaceMatrix(0.0f, 20.0f, -3.0f), 0.6f, 10.0f);
	//AddSphere1(scene, PlaceMatrix(0.0f, 5.0f, 0.0f), 0.5f, 10.0f);
	//AddCapsule(scene, PlaceMatrix(0.0f, 5.0f, 3.0f), 0.7f, 10.0f);
	//AddConvexHull(scene, PlaceMatrix(-2.0f, 5.0f, -2.0f), 7, 1.0f, 1.5f, 0.8f, 10.0f);
	//AddConvexHull(scene, PlaceMatrix(-2.0f, 5.0f,  2.0f), 21, 1.0f, 1.5f, 0.7f, 10.0f);
	//AddConvexHull(scene, PlaceMatrix( 2.0f, 5.0f,  3.0f), 210, 1.0f, 1.5f, 0.9f, 10.0f);

	// create a Phantom model that contains a collision shape and transform matrix
	ndSharedPtr<ndModel> phantomPtr(new ndModel);
	phantomPtr->SetNotifyCallback(new NewtonPhantom(scene));
	scene->GetWorld()->AddModel(phantomPtr);

	ndQuaternion rot;
	ndVector origin(-40.0f, 5.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}

