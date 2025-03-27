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
#include "ndCompoundScene.h"
#include "ndMakeStaticMap.h"
#include "ndDemoDebugMesh.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndBasicPlayerCapsule.h"
#include "ndHeightFieldPrimitive.h"

#if 1
void ndStaticMeshCollisionDemo (ndDemoEntityManager* const scene)
{
	ndMatrix heighfieldLocation (ndGetIdentityMatrix());
	heighfieldLocation.m_posit.m_x = -200.0f;
	heighfieldLocation.m_posit.m_z = -200.0f;

	//BuildPlayArena(scene);
	//BuildFlatPlane(scene, true);
	//BuildGridPlane(scene, 400, 4.0f, 0.0f);
	//BuildCompoundScene(scene, ndGetIdentityMatrix());
	//BuildHeightFieldTerrain(scene, heighfieldLocation);
	//BuildStaticMesh(scene, "flatPlane.fbx", false);
	//BuildStaticMesh(scene, "track.fbx", false);
	//BuildStaticMesh(scene, "testObject.fbx", false);
	//BuildStaticMesh(scene, "marine_rocks_corsica.fbx", false);
	//BuildStaticMesh(scene, "marineRocks1.fbx", false);
	BuildStaticMesh(scene, "marineRocks2.fbx", false);

	ndMatrix location(ndGetIdentityMatrix());
	location.m_posit.m_y += 2.0f;

	ndMatrix localAxis(ndGetIdentityMatrix());
	localAxis[0] = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
	localAxis[1] = ndVector(1.0f, 0.0f, 0.0f, 0.0f);
	localAxis[2] = localAxis[0].CrossProduct(localAxis[1]);

	ndMeshLoader loader;
	ndSharedPtr<ndDemoEntity> man(loader.LoadEntity("walker.fbx", scene));

	//ndFloat32 height = 1.9f;
	//ndFloat32 radio = 0.5f;
	//ndFloat32 mass = 100.0f;
	//new ndBasicPlayerCapsule(scene, man, localAxis, location, mass, radio, height, height/4.0f, true);
	
	location.m_posit.m_x += 8.0f;
	location.m_posit.m_z -= 2.0f;
	//new ndBasicPlayerCapsule(scene, man, localAxis, location, mass, radio, height, height / 4.0f);
	
	location.m_posit.m_z += 4.0f;
	//new ndBasicPlayerCapsule(scene, man, localAxis, location, mass, radio, height, height / 4.0f);

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

	AddBox(scene, PlaceMatrix(3.0f, 1.0f, 0.0f), 30.0f, 2.0f, 0.25f, 2.5f);
	AddBox(scene, PlaceMatrix(3.0f, 1.5f, 1.125f), 30.0f, 2.0f, 0.25f, 2.5f);
	AddBox(scene, PlaceMatrix(1.0f, 1.0f, 0.0f), 30.0f, 1.0f, 0.25f, 1.0f);
	AddConvexHull(scene, PlaceMatrix(0.0f, 1.0f, 2.0f), 10.0f, 0.6f, 1.0f, 15);
	AddConvexHull(scene, PlaceMatrix(0.0f, 1.0f, 0.0f), 10.0f, 0.7f, 1.0f, 10);
	AddConvexHull(scene, PlaceMatrix(1.0f, 1.0f, 0.0f), 10.0f, 0.5f, 1.2f, 6);
	AddConvexHull(scene, PlaceMatrix(1.0f, 1.0f, 2.0f), 10.0f, 0.6f, 1.0f, 15);
	AddConvexHull(scene, PlaceMatrix(2.0f, 1.0f, 0.0f), 10.0f, 0.7f, 1.0f, 10);
	AddConvexHull(scene, PlaceMatrix(1.0f, 1.0f, 1.0f), 10.0f, 0.5f, 1.2f, 6);

	//AddCapsulesStacks(scene, PlaceMatrix(45.0f, 0.0f, 0.0f), 10.0f, 0.5f, 0.5f, 1.0f, 5, 8, 7);

	ndQuaternion rot(ndYawMatrix(30.0f * ndDegreeToRad));
	//ndVector origin(-5.0f, 4.0f, 0.0f, 1.0f);
	ndVector origin(-3.0f, 0.0f, 2.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}

#elif 0

class CConvexCasterModelNotify : public ndModelNotify
{
	class CConvexCastCallBack : public ndConvexCastNotify
	{
		public:
		CConvexCastCallBack()
			:ndConvexCastNotify()
		{}
	
		virtual ndUnsigned32 OnRayPrecastAction(const ndBody* const body, const ndShapeInstance* const) override
		{
			// filter the floor
			ndUnsigned32 ret = ndUnsigned32(body->GetInvMass() ? 1 : 0);
			return ret;
		}
	
		virtual ndFloat32 OnRayCastAction(const ndContactPoint&, ndFloat32) override
		{
			return 0;
		}
	};

	public:
	CConvexCasterModelNotify(ndDemoEntityManager* const scene, ndBodyKinematic* const pParentBody)
		:ndModelNotify()
		,m_CastShape(new ndShapeSphere(5.0))
		,m_pScene(scene)
		,m_pParentBody(pParentBody)
		,m_pKinJoint(nullptr)
	{
	}

	virtual void Debug(ndConstraintDebugCallback&) const override
	{
		// this is a mistake, because it render from outsize the renmder thread.
		//ndWireFrameDebugMesh sharedEdgeMesh (m_pScene->GetShaderCache(), &m_CastShape);
		//const ndVector color(0.5f, 0.5f, 0.5f, 1.0f);
		//sharedEdgeMesh.SetColor(color);
		//sharedEdgeMesh.Render(m_pScene, ndGetIdentityMatrix());
	}

	virtual void Update(ndFloat32) override
	{
		CConvexCastCallBack castCallback;
		
		//if (m_world->ConvexCast(castCallback, m_CastShape, ndGetIdentityMatrix(), ndVector(0.0, 0.001, 0.0, 1.0)))
		//if (m_world->ConvexCast(castCallback, m_CastShape, ndGetIdentityMatrix(), ndVector(0.0, 0.0, 0.001, 1.0)))
		if (world->ConvexCast(castCallback, m_CastShape, ndGetIdentityMatrix(), ndVector(0.001, 0.0, 0.0, 1.0)))
		{
			if (castCallback.m_contacts.GetCount() > 0)
			{
				ndBodyKinematic* const pHitBody = (ndBodyKinematic*)castCallback.m_contacts[0].m_body1;
				ndAssert(pHitBody);
				//auto pUserData = static_cast<ndDemoEntity*>(pHitBody->GetNotifyCallback()->GetUserData());
				//ndAssert(pUserData->GetName() == "My Collision Object");
				if (!m_pKinJoint)
				{
					m_pKinJoint = new ndJointKinematicController(pHitBody, m_pParentBody, castCallback.m_contacts[0].m_point);
					m_pKinJoint->SetMaxAngularFriction(1000);
					m_pKinJoint->SetMaxLinearFriction(1000);
					ndSharedPtr<ndJointBilateralConstraint> jointPtr(m_pKinJoint);
					world->AddJoint(jointPtr);
				}
			}
		}
	}

	ndShapeInstance m_CastShape;
	ndDemoEntityManager* m_pScene;
	ndBodyKinematic* m_pParentBody;
	ndJointKinematicController* m_pKinJoint;
};

static ndBodyKinematic* BuildHeightField(ndDemoEntityManager* const scene)
{
	ndInt32 iDim = 64;
	ndFloat32 dSize = 128, dMaxHeight = 0.0;
	std::vector<ndFloat32> aData; aData.resize(size_t(iDim * iDim));
	ndFloat32 fHorizontalScale = ndFloat32(128.0) / ndFloat32(iDim - 1);
	ndShapeInstance shape(new ndShapeHeightfield(iDim, iDim, ndShapeHeightfield::m_normalDiagonals, fHorizontalScale, fHorizontalScale));
	ndMatrix mLocal(ndGetIdentityMatrix());
	mLocal.m_posit = ndVector(-(dSize * 0.5), 0.0, -(dSize * 0.5), 1.0);
	shape.SetLocalMatrix(mLocal);
	auto pShapeHeightField = shape.GetShape()->GetAsShapeHeightfield();

	for (ndInt32 i = 0; i < iDim * iDim; ++i)
	{
		pShapeHeightField->GetElevationMap()[i] = ndReal(rand() * 2.0 * dMaxHeight / RAND_MAX);
	}

	pShapeHeightField->UpdateElevationMapAabb();
	ndMatrix uvMatrix(ndGetIdentityMatrix());
	uvMatrix[0][0] *= 0.025f;
	uvMatrix[1][1] *= 0.025f;
	uvMatrix[2][2] *= 0.025f;

	ndSharedPtr<ndDemoMeshInterface>geometry(new ndDemoMesh("box", scene->GetShaderCache(), &shape, "marbleCheckBoard.png", "marbleCheckBoard.png", "marbleCheckBoard.png", 1.0f, uvMatrix, false));
	ndMatrix location(ndGetIdentityMatrix());
	ndSharedPtr<ndDemoEntity>entity = new ndDemoEntity(location);
	entity->SetMesh(geometry);

	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	body->SetMatrix(location);
	body->GetAsBodyDynamic()->SetCollisionShape(shape);

	scene->GetWorld()->AddBody(body);
	scene->AddEntity(entity);
	return body->GetAsBodyDynamic();
}

static void AddBody(ndDemoEntityManager* const scene)
{
	ndVector vStart(5.0, 10.0, 0.0, 1.0);
	ndMatrix location(ndGetIdentityMatrix());
	location.m_posit = vStart;
	auto pBody = AddBox(scene, location, 1.0f, 1.0f, 1.0f, 1.0f);
	auto pUserData = static_cast<ndDemoEntity*>(pBody->GetNotifyCallback()->GetUserData());
	pUserData->SetName("My Collision Object");
}

//void ndConvexCastTest(ndDemoEntityManager* const scene)
void ndStaticMeshCollisionDemo(ndDemoEntityManager* const scene)
{
	// build the height field
	auto pFloorBody = BuildHeightField(scene);
	AddBody(scene);

	ndSharedPtr<ndModel> model (new ndModel());
	model->SetNotifyCallback(new CConvexCasterModelNotify(scene, pFloorBody));
	//ndSharedPtr<ndModel> convexCaster(new CConvexCaster(scene, pFloorBody));
	//scene->GetWorld()->AddModel(convexCaster);
	scene->GetWorld()->AddModel(model);

	ndQuaternion rot;
	ndVector origin(-10.0f, 5.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}

#else

static ndBodyKinematic* CreateBody(ndDemoEntityManager* const scene, const ndShapeInstance& shape, const ndMatrix& location, ndFloat32 mass, const char* const textName)
{
	ndPhysicsWorld* const world = scene->GetWorld();

	ndMatrix matrix(FindFloor(*world, location, shape, 200.0f));
	ndSharedPtr<ndDemoMeshInterface> mesh(new ndDemoMesh("shape", scene->GetShaderCache(), &shape, textName, textName, textName));

	ndSharedPtr<ndBody> kinBody (new ndBodyDynamic());
	ndSharedPtr<ndDemoEntity>entity(new ndDemoEntity(matrix));
	entity->SetMesh(mesh);

	kinBody->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	kinBody->SetMatrix(matrix);
	kinBody->GetAsBodyDynamic()->SetCollisionShape(shape);
	kinBody->GetAsBodyDynamic()->SetMassMatrix(mass, shape);

	world->AddBody(kinBody);
	scene->AddEntity(entity);
	return kinBody->GetAsBodyDynamic();
}

ndBodyKinematic* AddChamferCylinder(ndDemoEntityManager* const scene, const ndMatrix& location, ndFloat32 mass, ndFloat32 radius, ndFloat32 width, const char* const textName)
{
	ndShapeInstance shape(new ndShapeChamferCylinder(0.5, 1.0));
	shape.SetScale(ndVector(width, radius, radius, ndFloat32 (1.0)));
	ndBodyKinematic* const body = CreateBody(scene, shape, location, mass, textName);
	return body;
}

static void BuildHeightField(ndDemoEntityManager* const scene)
{
	size_t iDim = 120;
	ndFloat32 dSize = 15, dMaxHeight = 0.0;
	std::vector<ndFloat32> aData; aData.resize(iDim * iDim);
	ndFloat32 fHorizontalScale = ndFloat32(dSize / ndFloat32(iDim - 1));
	ndShapeInstance shape(new ndShapeHeightfield(ndInt32(iDim), ndInt32(iDim), ndShapeHeightfield::m_normalDiagonals, fHorizontalScale, fHorizontalScale));
	//ndShapeInstance shape(new ndShapeHeightfield(ndInt32(iDim), ndInt32(iDim), ndShapeHeightfield::m_invertedDiagonals, fHorizontalScale, fHorizontalScale));
	ndMatrix mLocal(ndGetIdentityMatrix());
	mLocal.m_posit = ndVector(-(dSize * 0.5), 0.0, -(dSize * 0.5), 1.0);
	shape.SetLocalMatrix(mLocal);
	auto pShapeHeightField = shape.GetShape()->GetAsShapeHeightfield();
	for (ndInt32 i = 0; i < ndInt32(iDim * iDim); ++i)
	{
		pShapeHeightField->GetElevationMap()[i] = ndReal(ndFloat32(rand()) * ndFloat32(2.0) * dMaxHeight / RAND_MAX);
	}

	pShapeHeightField->UpdateElevationMapAabb();
	ndMatrix uvMatrix(ndGetIdentityMatrix());
	uvMatrix[0][0] *= 0.025f;
	uvMatrix[1][1] *= 0.025f;
	uvMatrix[2][2] *= 0.025f;

	ndSharedPtr<ndDemoMeshInterface>geometry(new ndDemoMesh("box", scene->GetShaderCache(), &shape, "marbleCheckBoard.png", "marbleCheckBoard.png", "marbleCheckBoard.png", 1.0f, uvMatrix, false));
	ndMatrix location(ndGetIdentityMatrix());
	ndSharedPtr<ndDemoEntity> entity (new ndDemoEntity(location));
	entity->SetMesh(geometry);

	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	body->SetMatrix(location);
	body->GetAsBodyDynamic()->SetCollisionShape(shape);
	
	scene->GetWorld()->AddBody(body);
	scene->AddEntity(entity);
}

static void AddBodies(ndDemoEntityManager* const scene)
{
	ndVector vStart(0.0f, 3.0f, 0.0f, 1.0f);
	ndMatrix location(ndRollMatrix(10.0 * ndDegreeToRad));
	location.m_posit = vStart;

	ndBodyKinematic* const body = AddChamferCylinder(scene, location, 1.0f, 1.0f, 1.25f, "wood_0.png");
	body->SetMatrix(location);
}

//void ndHeightFieldTest(ndDemoEntityManager* const scene)
void ndStaticMeshCollisionDemo(ndDemoEntityManager* const scene)
{
	// build the height field
	BuildHeightField(scene);
	AddBodies(scene);

	ndQuaternion rot;
	ndVector origin(-15.0f, 5.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}

#endif