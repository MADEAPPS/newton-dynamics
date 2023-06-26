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
#include "ndUIEntity.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

namespace ndController_0
{
	#define ND_ACTIONS			12
	#define ND_TOTAL_ACTIONS	(2 * ND_ACTIONS + 1)

	class ndCarpole : public ndModelArticulation
	{
		public:
		ndCarpole()
			:ndModelArticulation()
		{
		}

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			ndModelArticulation::Update(world, timestep);
		}
	};

	void BuildModel(ndCarpole* const model, ndDemoEntityManager* const scene, const ndMatrix& location)
	{
		ndFloat32 xSize = 0.25f;
		ndFloat32 ySize = 0.125f;
		ndFloat32 zSize = 0.15f;
		ndFloat32 carMass = 5.0f;
		ndFloat32 poleMass = 1.0f;
		ndPhysicsWorld* const world = scene->GetWorld();
		
		// add hip body
		ndSharedPtr<ndBody> hipBody(world->GetBody(AddBox(scene, location, carMass, xSize, ySize, zSize, "smilli.tga")));
		ndModelArticulation::ndNode* const modelRoot = model->AddRootBody(hipBody);

		//ndMatrix matrix(hipBody->GetMatrix());
		//matrix.m_posit.m_y += 0.5f;
		//hipBody->SetMatrix(matrix);
		//
		//ndMatrix limbLocation(matrix);
		//limbLocation.m_posit.m_z += zSize * 0.0f;
		//limbLocation.m_posit.m_y -= ySize * 0.5f;
		//limbLocation.m_posit.m_x += xSize * 0.5f * -0.1f;
		//
		//// make single leg
		//ndFloat32 limbLength = 0.3f;
		//ndFloat32 limbRadio = 0.025f;
		//
		//ndSharedPtr<ndBody> legBody(world->GetBody(AddCapsule(scene, ndGetIdentityMatrix(), limbMass, limbRadio, limbRadio, limbLength, "smilli.tga")));
		//ndMatrix legLocation(ndRollMatrix(-90.0f * ndDegreeToRad) * limbLocation);
		//legLocation.m_posit.m_y -= limbLength * 0.5f;
		//legBody->SetMatrix(legLocation);
		//ndMatrix legPivot(ndYawMatrix(90.0f * ndDegreeToRad) * legLocation);
		//legPivot.m_posit.m_y += limbLength * 0.5f;
		//ndSharedPtr<ndJointBilateralConstraint> legJoint(new ndJointHinge(legPivot, legBody->GetAsBodyKinematic(), modelRoot->m_body->GetAsBodyKinematic()));
		//ndJointHinge* const hinge = (ndJointHinge*)*legJoint;
		//hinge->SetAsSpringDamper(0.001f, 1500, 40.0f);
		//model->m_controlJoint = hinge;
		//
		//// make wheel
		//ndFloat32 wheelRadio = 4.0f * limbRadio;
		//ndSharedPtr<ndBody> wheelBody(world->GetBody(AddSphere(scene, ndGetIdentityMatrix(), wheelMass, wheelRadio, "smilli.tga")));
		//ndMatrix wheelMatrix(legPivot);
		//wheelMatrix.m_posit.m_y -= limbLength;
		//wheelBody->SetMatrix(wheelMatrix);
		//ndSharedPtr<ndJointBilateralConstraint> wheelJoint(new ndJointSpherical(wheelMatrix, wheelBody->GetAsBodyKinematic(), legBody->GetAsBodyKinematic()));
		////((ndJointSpherical*)*wheelJoint)->SetAsSpringDamper(ndFloat32(0.001f), ndFloat32(0.0f), ndFloat32(10.f));
		//
		//// tele port the model so that is on the floor
		//ndMatrix probeMatrix(wheelMatrix);
		//probeMatrix.m_posit.m_x += 1.0f;
		//ndMatrix floor(FindFloor(*world, probeMatrix, wheelBody->GetAsBodyKinematic()->GetCollisionShape(), 20.0f));
		//ndFloat32 dist = wheelMatrix.m_posit.m_y - floor.m_posit.m_y;
		//
		//ndMatrix rootMatrix(modelRoot->m_body->GetMatrix());
		//
		//rootMatrix.m_posit.m_y -= dist;
		//wheelMatrix.m_posit.m_y -= dist;
		//legLocation.m_posit.m_y -= dist;
		//
		//legBody->SetMatrix(legLocation);
		//wheelBody->SetMatrix(wheelMatrix);
		//modelRoot->m_body->SetMatrix(rootMatrix);
		//
		//legBody->GetNotifyCallback()->OnTransform(0, legLocation);
		//wheelBody->GetNotifyCallback()->OnTransform(0, wheelMatrix);
		//modelRoot->m_body->GetNotifyCallback()->OnTransform(0, rootMatrix);
		//
		//// add the joints manually, because on this model the wheel is not actuated.
		//world->AddJoint(legJoint);
		//world->AddJoint(wheelJoint);
		//
		//// add model limbs
		//model->AddLimb(modelRoot, legBody, legJoint);
		//
		//model->m_ballBody = wheelBody->GetAsBodyDynamic();
		//
		//model->Init();

		//ndModelArticulation* const articulation = (ndModelArticulation*)model->GetAsModelArticulation();
		ndBodyKinematic* const rootBody = hipBody->GetAsBodyKinematic();
		ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointSlider(rootBody->GetMatrix(), rootBody, world->GetSentinelBody()));
		world->AddJoint(fixJoint);
	}

	ndModelArticulation* CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location)
	{
		ndCarpole* const model = new ndCarpole();
		BuildModel(model, scene, location);
		return model;
	}
}

using namespace ndController_0;
void ndCarpoleController(ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildFloorBox(scene, ndGetIdentityMatrix());
	BuildFlatPlane(scene, true);
	
	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(ndYawMatrix(-0.0f * ndDegreeToRad));

	ndSharedPtr<ndModel> model(CreateModel(scene, matrix));
	scene->GetWorld()->AddModel(model);
	
	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
