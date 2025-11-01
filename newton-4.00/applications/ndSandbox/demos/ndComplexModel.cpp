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
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoCameraNodeFollow.h"
#include "ndHeightFieldPrimitive.h"


class ndExcavatorController : public ndModelNotify
{
	public:
	enum ndBodyPartType
	{
		m_chassis = 1,
		m_roller,
		m_thread,
	};

	ndExcavatorController(
		ndDemoEntityManager* const scene, ndSharedPtr<ndMesh>& mesh, 
		ndSharedPtr<ndModel>& model, ndSharedPtr<ndRenderSceneNode>& visualMesh)
		:ndModelNotify()
		,m_scene(scene)
	{
		ndModelArticulation* const articulation = model->GetAsModelArticulation();
		ndAssert(articulation);

		// build the major vehicle components, each major component build its sub parts
		MakeChassis(articulation, mesh, visualMesh);
		MakeCabinAndUpperBody(articulation, mesh, visualMesh);

		MakeLeftTrack(articulation, mesh, visualMesh);
		MakeRightTrack(articulation, mesh, visualMesh);

		MakeThread(articulation, "leftThread", mesh, visualMesh);
		MakeThread(articulation, "rightThread", mesh, visualMesh);
	}

	bool OnContactGeneration(const ndBodyKinematic* const body0, const ndBodyKinematic* const body1) override
	{
		// here the application can use filter to determine what body parts should collide.
		// this greatly improves performance because since articulated models in general 
		// do not self collide, but occationaly some parts do collide. 
		const ndShapeInstance& shape0 = body0->GetCollisionShape();
		const ndShapeInstance& shape1 = body1->GetCollisionShape();

		ndBodyPartType type0 = ndBodyPartType(shape0.m_shapeMaterial.m_userId);
		ndBodyPartType type1 = ndBodyPartType(shape1.m_shapeMaterial.m_userId);
		
		bool collide = ((type0 == m_roller) && (type1 == m_thread)) || ((type0 == m_thread) && (type1 == m_roller));
		return collide;
	}

	static ndSharedPtr<ndModelNotify> CreateExcavator(ndDemoEntityManager* const scene, const ndMatrix& location, ndRenderMeshLoader& mesh)
	{
		// crate a model and set the root transform
		ndSharedPtr<ndRenderSceneNode> vehicleMesh(mesh.m_renderMesh->Clone());
		ndMatrix matrix(location);
		matrix.m_posit = FindFloor(*scene->GetWorld(), matrix.m_posit, 200.0f);

		//this was a mistake, the model souel be crated form teh ndMesh no the visual mesh
		//vehicleMesh->SetTransform(vehicleMesh->GetMatrix() * matrix);
		//vehicleMesh->SetTransform(vehicleMesh->GetMatrix() * matrix);

		ndMatrix saveMatrix(mesh.m_mesh->m_matrix);
		mesh.m_mesh->m_matrix = saveMatrix * matrix;

		// using a model articulation for this vehicle
		ndSharedPtr<ndModel> vehicleModel(new ndModelArticulation());
		ndSharedPtr<ndModelNotify> controller(new ndExcavatorController(scene, mesh.m_mesh, vehicleModel, vehicleMesh));
		vehicleModel->SetNotifyCallback(controller);

		ndWorld* const world = scene->GetWorld();
		scene->AddEntity(vehicleMesh);
		world->AddModel(vehicleModel);
		vehicleModel->AddBodiesAndJointsToWorld();

		// restore mesh Matrix 
		mesh.m_mesh->m_matrix = saveMatrix;
		return controller;
	}

	private:
	// set a body pat type later usage in the concact filter generatior
	void SetBodyType(ndShapeInstance* const collision, ndBodyPartType type)
	{
		collision->m_shapeMaterial.m_userId = type;
	}

	void MakeChassis(
		ndModelArticulation* const articulation, 
		ndSharedPtr<ndMesh>& mesh, 
		ndSharedPtr<ndRenderSceneNode>& visualMeshRoot)
	{
		ndMesh* const chassisMesh = mesh->FindByName("base");
		ndAssert(chassisMesh);

		// create the collision and the world matrix
		ndMatrix matrix(chassisMesh->CalculateGlobalMatrix());
		ndSharedPtr<ndShapeInstance> collision (chassisMesh->CreateCollisionFromChildren());

		// set a body pat type later usage in the concact filter generatior
		SetBodyType(*collision, m_chassis);

		// find the visual mesh for thsi body
		ndRenderSceneNode* const visualMeshNode = visualMeshRoot->FindByName(chassisMesh->GetName());
		ndSharedPtr<ndRenderSceneNode> visualMesh((visualMeshNode == *visualMeshRoot) ? visualMeshRoot : visualMeshRoot->GetSharedPtr());

		ndFloat32 chassisMass = ndFloat32(4000.0f);
		//ndFloat32 chassisMass = ndFloat32(0.0f);

		// create the rigid that represent the chassis
		ndSharedPtr<ndBody> chassisBody(new ndBodyDynamic());
		chassisBody->SetNotifyCallback(new ndDemoEntityNotify(m_scene, visualMesh, nullptr));
		chassisBody->SetMatrix(matrix);
		chassisBody->GetAsBodyDynamic()->SetCollisionShape(**collision);
		chassisBody->GetAsBodyDynamic()->SetMassMatrix(chassisMass, **collision);
		
		articulation->AddRootBody(chassisBody);
	}

	ndSharedPtr<ndBody> MakeBodyPart(
		ndMesh* const childMesh, 
		ndSharedPtr<ndRenderSceneNode>& visualMeshRoot,
		ndSharedPtr<ndShapeInstance> collision,
		ndBodyKinematic* const parentBody,
		ndFloat32 mass)
	{
		// calculate matrix
		const ndMatrix matrix(childMesh->CalculateGlobalMatrix());

		// find the visual mesh
		ndRenderSceneNode* const visualMeshNode = visualMeshRoot->FindByName(childMesh->GetName());
		ndSharedPtr<ndRenderSceneNode> visualMesh(visualMeshNode->GetSharedPtr());

		ndSharedPtr<ndBody> body(new ndBodyDynamic());
		body->SetNotifyCallback(new ndDemoEntityNotify(m_scene, visualMesh, parentBody));
		body->SetMatrix(matrix);
		body->GetAsBodyDynamic()->SetCollisionShape(**collision);
		body->GetAsBodyDynamic()->SetMassMatrix(mass, **collision);
		return body;
	}

	ndSharedPtr<ndBody> MakeBodyPart(
		ndSharedPtr<ndMesh>& mesh, ndSharedPtr<ndRenderSceneNode>& visualMeshRoot,
		ndBodyKinematic* const parentbody,
		const char* const name, ndFloat32 mass)
	{
		ndMesh* const meshNode = mesh->FindByName(name);
		ndAssert(meshNode);

		// build collision mesh
		ndSharedPtr<ndShapeInstance> collision(meshNode->CreateCollisionFromChildren());
		return MakeBodyPart(meshNode, visualMeshRoot, collision, parentbody, mass);
	}

	void MakeCabinAndUpperBody(
		ndModelArticulation* const articulation,
		ndSharedPtr<ndMesh>& mesh, ndSharedPtr<ndRenderSceneNode>& visualMeshRoot)
	{
		ndModelArticulation::ndNode* const rootNode = articulation->GetRoot();

		ndSharedPtr<ndBody> cabinBody (MakeBodyPart(mesh, visualMeshRoot, rootNode->m_body->GetAsBodyKinematic(), "EngineBody", 400.0f));
		SetBodyType(&cabinBody->GetAsBodyDynamic()->GetCollisionShape(), m_chassis);

		ndMatrix hingeFrame(cabinBody->GetMatrix());
		ndSharedPtr<ndJointBilateralConstraint> cabinPivot(new ndJointHinge(hingeFrame, cabinBody->GetAsBodyDynamic(), rootNode->m_body->GetAsBodyDynamic()));
		
		//// set the center of mass of engine
		//dVector com(hingeFrame.UnrotateVector(dVector(-2.0f, 0.0f, 0.0f, 0.0f)));
		//NewtonBodySetCentreOfMass(cabinBody, &com[0]);
		ndModelArticulation::ndNode* const cabinNode = articulation->AddLimb(rootNode, cabinBody, cabinPivot);
	}

	void LinkTires(ndModelArticulation* const articulation,
		ndModelArticulation::ndNode* const master, ndModelArticulation::ndNode* const slave)
	{
		const ndShapeInstance& slaveShape = slave->m_body->GetAsBodyKinematic()->GetCollisionShape();
		const ndShapeInstance& masterShape = master->m_body->GetAsBodyKinematic()->GetCollisionShape();

		ndFloat32 slaveRadio = slaveShape.GetScale().m_y;
		ndFloat32 masterRadio = masterShape.GetScale().m_y;

		ndMatrix pinMatrix0;
		ndMatrix pinMatrix1;
		const ndJointBilateralConstraint* const joint = *master->m_joint;
		joint->CalculateGlobalMatrix(pinMatrix0, pinMatrix1);

		ndFloat32 ratio = slaveRadio / masterRadio;
		ndSharedPtr<ndJointBilateralConstraint> link(new ndJointGear(
			ratio, pinMatrix0[0], slave->m_body->GetAsBodyDynamic(),
			pinMatrix1[0].Scale(ndFloat32(-1.0f)), master->m_body->GetAsBodyDynamic()));

		articulation->AddCloseLoop(link);
	}

	ndModelArticulation::ndNode* MakeRollerTire(
		ndModelArticulation* const articulation,
		ndSharedPtr<ndMesh>& mesh, 
		ndSharedPtr<ndRenderSceneNode>& visualMeshRoot,
		const char* const name)
	{
		ndMesh* const node = mesh->FindByName(name);
		ndAssert(node);
		//ndSharedPtr<ndShapeInstance> tireCollision(node->CreateCollisionTire());
		ndSharedPtr<ndShapeInstance> tireCollision(node->CreateCollisionChamferCylinder());
		SetBodyType(*tireCollision, m_roller);

		//ndFloat32 threadThickness = ndFloat32(0.18f);
		//ndFloat32 threadThickness = ndFloat32(0.0f);
		//ndVector padd(ndVector::m_zero);
		//padd.m_y = threadThickness;
		//padd.m_z = threadThickness;
		//tireCollision->SetScale (tireCollision->GetScale() + padd);

		tireCollision->SetLocalMatrix(ndYawMatrix(90.0f * ndDegreeToRad) * tireCollision->GetLocalMatrix());

		ndModelArticulation::ndNode* const rootNode = articulation->GetRoot();
		ndSharedPtr<ndBody> tireBody(MakeBodyPart(node, visualMeshRoot, tireCollision,
			rootNode->m_body->GetAsBodyKinematic(),	ndFloat32(30.0f)));

		const ndMatrix rollerMatrix(ndYawMatrix(90.0f * ndDegreeToRad) * tireBody->GetMatrix());
		ndSharedPtr<ndJointBilateralConstraint> rollerPivot(new ndJointRoller(rollerMatrix, tireBody->GetAsBodyDynamic(), rootNode->m_body->GetAsBodyDynamic()));
		ndJointRoller* const rollerJoint = (ndJointRoller*)*rollerPivot;
		rollerJoint->SetAsSpringDamperPosit(ndFloat32(0.01f), ndFloat32 (2000.0f), ndFloat32 (50.0f));

		ndModelArticulation::ndNode* const rollerLimb = articulation->AddLimb(rootNode, tireBody, rollerPivot);
		return rollerLimb;
	}

	void MakeLeftTrack(
		ndModelArticulation* const articulation,
		ndSharedPtr<ndMesh>& mesh, 
		ndSharedPtr<ndRenderSceneNode>& visualMeshRoot)
	{
		ndModelArticulation::ndNode* const leftTire_0 = MakeRollerTire(articulation, mesh, visualMeshRoot, "leftGear");
		ndModelArticulation::ndNode* const leftTire_7 = MakeRollerTire(articulation, mesh, visualMeshRoot, "leftFrontRoller");
		ndAssert(leftTire_0);
		ndAssert(leftTire_7);
		LinkTires(articulation, leftTire_0, leftTire_7);
		MakeRollerTire(articulation, mesh, visualMeshRoot, "leftSupportRoller");

		for (ndInt32 i = 0; i < 3; ++i) 
		{
			char name[64];
			snprintf(name, 63, "leftRoller%d", i);
			ndModelArticulation::ndNode* const rollerTire = MakeRollerTire(articulation, mesh, visualMeshRoot, name);
			ndAssert(rollerTire);
			LinkTires(articulation, leftTire_0, rollerTire);
		}

		//// link traction tire to the engine using a differential gear
		//dMatrix engineMatrix;
		//dMatrix chassisMatrix;
		//
		//NewtonBody* const tire = leftTire_0->GetBody();
		//NewtonBody* const engine = m_engineJoint->GetBody0();
		//m_engineJoint->CalculateGlobalMatrix(engineMatrix, chassisMatrix);
		//dMatrix tireMatrix;
		//NewtonBodyGetMatrix(tire, &tireMatrix[0][0]);
		//new dCustomDifferentialGear___(EXCAVATOR_GEAR_GAIN, engineMatrix.m_front.Scale(-1.0f), engineMatrix.m_up, tireMatrix.m_right.Scale(1.0f), engine, tire);
	}

	void MakeRightTrack(
		ndModelArticulation* const articulation,
		ndSharedPtr<ndMesh>& mesh,
		ndSharedPtr<ndRenderSceneNode>& visualMeshRoot)
	{
		ndModelArticulation::ndNode* const rightTire_0 = MakeRollerTire(articulation, mesh, visualMeshRoot, "rightGear");
		ndModelArticulation::ndNode* const rightTire_7 = MakeRollerTire(articulation, mesh, visualMeshRoot, "rightFrontRoller");
		ndAssert(rightTire_0);
		ndAssert(rightTire_7);
		LinkTires(articulation, rightTire_0, rightTire_7);
		MakeRollerTire(articulation, mesh, visualMeshRoot, "rightSupportRoller");

		for (int i = 0; i < 3; ++i)
		{
			char name[64];
			snprintf(name, 63, "rightRoller%d", i);
			ndModelArticulation::ndNode* const rollerTire = MakeRollerTire(articulation, mesh, visualMeshRoot, name);
			ndAssert(rollerTire);
			LinkTires(articulation, rightTire_0, rollerTire);
		}
	}

	void MakeThread(ndModelArticulation* const articulation,
		const char* const baseName,
		ndSharedPtr<ndMesh>& mesh,
		ndSharedPtr<ndRenderSceneNode>& visualMeshRoot)
	{
		// find the first link of the thread.
		char name[256];
		snprintf(name, 255, "%s_00", baseName);
		ndFixSizeArray<ndMesh*, 256> stack;
		ndFixSizeArray<ndMesh*, 256> linkArray;
		stack.PushBack(mesh->FindByName(name));
		ndAssert(stack[0]);

		// get all the thread links in order.
		while (stack.GetCount())
		{
			ndMesh* const node = stack.Pop();
			linkArray.PushBack(node);

			for (ndList<ndSharedPtr<ndMesh>>::ndNode* child = node->GetChildren().GetFirst(); child; child = child->GetNext())
			{
				ndMesh* const childMesh = *child->GetInfo();
				if (childMesh->GetName().Find(baseName) != -1)
				{
					stack.PushBack(*child->GetInfo());
				}
			}
		}

		// make the collision shape. 
		ndSharedPtr<ndShapeInstance> threadCollision(linkArray[0]->CreateCollisionFromChildren());
		SetBodyType(*threadCollision, m_thread);

		ndFloat32 threadLinkMass = ndFloat32(8.0f);
		ndModelArticulation::ndNode* const rootNode = articulation->GetRoot();
		ndSharedPtr<ndBody> linkBody (MakeBodyPart(linkArray[0], visualMeshRoot, threadCollision, rootNode->m_body->GetAsBodyKinematic(), threadLinkMass));

		ndMatrix planeMatrix(linkBody->GetMatrix());
		ndVector planePivot(planeMatrix.m_posit);
		ndVector planeNornal(planeMatrix.m_up);
		ndSharedPtr<ndJointBilateralConstraint> linkJoint(new ndJointPlane(
			planePivot, planeNornal, linkBody->GetAsBodyDynamic(),
			rootNode->m_body->GetAsBodyDynamic()));

		ndModelArticulation::ndNode* const firstLink = articulation->AddLimb(rootNode, linkBody, linkJoint);

		// connent all threads planks with hinge joint
		ndModelArticulation::ndNode* linkNode0 = firstLink;
		for (ndInt32 i = 1; i < linkArray.GetCount(); ++i)
		{
			ndSharedPtr<ndBody> body(MakeBodyPart(linkArray[i], visualMeshRoot, threadCollision, linkNode0->m_body->GetAsBodyKinematic(), threadLinkMass));
			ndMatrix hingeMatrix (ndRollMatrix(90.0f * ndDegreeToRad) * body->GetMatrix());
			ndSharedPtr<ndJointBilateralConstraint> joint (new ndJointHinge(hingeMatrix, body->GetAsBodyKinematic(), linkNode0->m_body->GetAsBodyKinematic()));
			((ndJointHinge*)*joint)->SetAsSpringDamper(ndFloat32(0.1f), ndFloat32(0.0f), ndFloat32(5.0f));
			ndModelArticulation::ndNode* const firstLink1 = articulation->AddLimb(linkNode0, body, joint);
			linkNode0 = firstLink1;
		}

		ndMatrix hingeMatrix(ndRollMatrix(90.0f * ndDegreeToRad) * firstLink->m_body->GetMatrix());
		ndSharedPtr<ndJointBilateralConstraint> joint(new ndJointHinge(hingeMatrix, linkNode0->m_body->GetAsBodyKinematic(), firstLink->m_body->GetAsBodyKinematic()));
		((ndJointHinge*)*joint)->SetAsSpringDamper(ndFloat32(0.1f), ndFloat32(0.0f), ndFloat32(5.0f));
		articulation->AddCloseLoop(joint);
	}

	ndDemoEntityManager* m_scene;
};

class ExcavatorThreadFloorMaterial : public ndApplicationMaterial
{
	public:
	ExcavatorThreadFloorMaterial()
		:ndApplicationMaterial()
	{
	}

	ExcavatorThreadFloorMaterial(const ExcavatorThreadFloorMaterial& src)
		:ndApplicationMaterial(src)
	{
	}

	ndApplicationMaterial* Clone() const
	{
		return new ExcavatorThreadFloorMaterial(*this);
	}

	// this materail process the contatact generated by the thread links and the backgrount
	virtual void OnContactCallback(const ndContact* const joint, ndFloat32) const
	{
		ndContactPointList& contactPoints = (ndContactPointList&)joint->GetContactPoints();
		ndContactPointList::ndNode* next;
		for (ndContactPointList::ndNode* contactPointsNode = contactPoints.GetFirst()->GetNext(); contactPointsNode; contactPointsNode = next)
		{
			next = contactPointsNode->GetNext();
			contactPoints.Remove(contactPointsNode);
		}
	}
};

class ExcavatorThreadRollerMaterial : public ndApplicationMaterial
{
	public:
	ExcavatorThreadRollerMaterial()
		:ndApplicationMaterial()
	{
	}

	ExcavatorThreadRollerMaterial(const ExcavatorThreadFloorMaterial& src)
		:ndApplicationMaterial(src)
	{
	}

	ndApplicationMaterial* Clone() const
	{
		return new ExcavatorThreadRollerMaterial(*this);
	}

	// this material process the contatact generated by the thread links and the backgrount
	virtual void OnContactCallback(const ndContact* const joint, ndFloat32) const
	{
		ndContactPointList& contactPoints = (ndContactPointList&)joint->GetContactPoints();
		ndAssert(contactPoints.GetCount() == 1);
		ndContactMaterial& contact = contactPoints.GetFirst()->GetInfo();
		ndBodyKinematic* const roller = joint->GetBody0()->GetCollisionShape().GetShape()->GetAsShapeChamferCylinder() ? joint->GetBody0() : joint->GetBody1();
		ndAssert(roller->GetCollisionShape().GetShape()->GetAsShapeChamferCylinder());
		const ndMatrix& matrix = roller->GetMatrix();

		contact.RotateTangentDirections(matrix.m_right);
		contact.m_material.m_flags = contact.m_material.m_flags & ~m_friction0Enable;
	}
};

void ndComplexModel(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "blueCheckerboard.png", 0.1f, true));
	//ndSharedPtr<ndBody> mapBody(BuildHeightFieldTerrain(scene, "grass.png", ndGetIdentityMatrix()));

	ndMatrix matrix(ndGetIdentityMatrix());

	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();

	// this material prune extra contact from the thread links and ground 
	ExcavatorThreadFloorMaterial material0;
	callback->RegisterMaterial(material0, ndExcavatorController::m_thread, ndDemoContactCallback::m_default);

	// another material to remove the friction from the thread and roller contacts
	ExcavatorThreadRollerMaterial material1;
	callback->RegisterMaterial(material1, ndExcavatorController::m_thread, ndExcavatorController::m_roller);

	//// add a help menu
	//ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndBackGroundVehicleController::ndHelpLegend());
	//scene->SetDemoHelp(demoHelper);

	ndRenderMeshLoader excavatorLoaded(*scene->GetRenderer());
	excavatorLoaded.LoadMesh(ndGetWorkingFileName("excavator.nd"));
	//ndSharedPtr<ndModelNotify> controller(ndBackGroundVehicleController::CreateAiVehicleProp(scene, ndVector::m_wOne, vmwLoaderRedPaint));

	ndSharedPtr<ndModelNotify> controller(ndExcavatorController::CreateExcavator(scene, matrix, excavatorLoaded));

	//// set this player as the active camera
	//ndBackGroundVehicleController* const playerController = (ndBackGroundVehicleController*)*controller;
	//ndRender* const renderer = *scene->GetRenderer();
	//renderer->SetCamera(playerController->GetCamera());

	matrix.m_posit.m_x -= 10.0f;
	matrix.m_posit.m_y += 4.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
