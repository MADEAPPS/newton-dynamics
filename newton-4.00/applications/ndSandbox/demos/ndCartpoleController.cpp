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
	enum ndActionSpace
	{
		m_statePut,
		m_pushLeft,
		m_pushRight,
		m_acctionsCount
	};

	enum ndStateSpace
	{
		m_cartPosition,
		m_cartVelocity,
		m_poleAngle,
		m_poleOmega,
		m_stateCount
	};

	class ndDQNTrainer
	{
		public:

		ndBrainReplayBuffer<ndInt32, 1> m_replayBuffer;
		ndBrainReplayTransitionMemory<ndInt32, 1> m_currentTransition;
	};

	class ndCartpole : public ndModelArticulation
	{
		public:
		ndCartpole()
			:ndModelArticulation()
			,m_trainer(nullptr)
		{
		}

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			ndModelArticulation::Update(world, timestep);
		}

		ndSharedPtr<ndDQNTrainer> m_trainer;
	};

	void BuildModel(ndCartpole* const model, ndDemoEntityManager* const scene, const ndMatrix& location)
	{
		ndFloat32 xSize = 0.25f;
		ndFloat32 ySize = 0.125f;
		ndFloat32 zSize = 0.15f;
		ndFloat32 cartMass = 5.0f;
		ndFloat32 poleMass = 1.0f;
		ndPhysicsWorld* const world = scene->GetWorld();
		
		// add hip body
		ndSharedPtr<ndBody> cartBody(world->GetBody(AddBox(scene, location, cartMass, xSize, ySize, zSize, "smilli.tga")));
		ndModelArticulation::ndNode* const modelRoot = model->AddRootBody(cartBody);

		ndMatrix matrix(cartBody->GetMatrix());

		// make single leg
		ndFloat32 poleLength = 0.4f;
		ndFloat32 poleRadio = 0.025f;
		
		ndSharedPtr<ndBody> poleBody(world->GetBody(AddCapsule(scene, ndGetIdentityMatrix(), poleMass, poleRadio, poleRadio, poleLength, "smilli.tga")));
		ndMatrix poleLocation(ndRollMatrix(-90.0f * ndDegreeToRad) * matrix);
		poleLocation.m_posit.m_y += poleLength * 0.5f;
		poleBody->SetMatrix(poleLocation);

		ndMatrix polePivot(ndYawMatrix(90.0f * ndDegreeToRad) * poleLocation);
		polePivot.m_posit.m_y -= poleLength * 0.5f;
		ndSharedPtr<ndJointBilateralConstraint> poleJoint(new ndJointHinge(polePivot, poleBody->GetAsBodyKinematic(), modelRoot->m_body->GetAsBodyKinematic()));

		world->AddJoint(poleJoint);

		// add model limbs
		model->AddLimb(modelRoot, poleBody, poleJoint);

		ndBodyKinematic* const rootBody = cartBody->GetAsBodyKinematic();
		ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointSlider(rootBody->GetMatrix(), rootBody, world->GetSentinelBody()));
		world->AddJoint(fixJoint);
	}

	ndModelArticulation* CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location)
	{
		ndCartpole* const model = new ndCartpole();
		BuildModel(model, scene, location);
		return model;
	}
}

using namespace ndController_0;
void ndCartpoleController(ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildFloorBox(scene, ndGetIdentityMatrix());
	BuildFlatPlane(scene, true);
	
	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(ndYawMatrix(-0.0f * ndDegreeToRad));

	ndSharedPtr<ndModel> model(CreateModel(scene, matrix));
	world->AddModel(model);
	
	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
