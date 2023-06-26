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

// this is an implementation of the vanilla dqn training as decrived 
// on the nature paper below. 
// https://storage.googleapis.com/deepmind-media/dqn/DQNNaturePaper.pdf

namespace ndController_0
{
	#define D_REPLAY_BUFFERSIZE (1024 * 64)

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

	class ndCartpoleBase : public ndModelArticulation
	{
		public:

		virtual void GetObservation() const = 0;

	};

	class ndDQNTrainer
	{
		public:
		ndDQNTrainer(ndCartpoleBase* const model)
			:m_replayBuffer(D_REPLAY_BUFFERSIZE)
			,m_currentTransition()
			,m_model(model)
		{
		}

		void Train()
		{
			m_model->GetObservation();
		}

		ndBrainReplayBuffer<ndInt32, m_stateCount> m_replayBuffer;
		ndBrainReplayTransitionMemory<ndInt32, m_stateCount> m_currentTransition;
		ndCartpoleBase* m_model;
	};

	class ndCartpole : public ndCartpoleBase
	{
		public:
		ndCartpole()
			:ndCartpoleBase()
			,m_trainer(nullptr)
			,m_cart(nullptr)
			,m_pole(nullptr)
		{
		}

		void GetObservation() const
		{
			ndFloat32 posit = m_cart->GetMatrix().m_posit.m_x;
			ndFloat32 veloc = m_cart->GetVelocity().m_x;

			ndFloat32 angle = ndAsin(m_pole->GetMatrix().m_up.m_y);
			ndFloat32 omega = m_pole->GetOmega().m_z;

			m_trainer->m_currentTransition.m_state[m_cartPosition] = ndReal(posit);
			m_trainer->m_currentTransition.m_state[m_cartVelocity] = ndReal(veloc);
			m_trainer->m_currentTransition.m_state[m_poleAngle] = ndReal(angle);
			m_trainer->m_currentTransition.m_state[m_poleOmega] = ndReal(omega);
			//ndTrace(("%f %f %f %f\n", posit, veloc, angle * ndRadToDegree, omega));
		}

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			ndModelArticulation::Update(world, timestep);
			m_trainer->Train();
		}

		ndSharedPtr<ndDQNTrainer> m_trainer;
		ndBodyDynamic* m_cart;
		ndBodyDynamic* m_pole;
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

		model->m_cart = rootBody->GetAsBodyDynamic();
		model->m_pole = poleBody->GetAsBodyDynamic();
		model->m_trainer = ndSharedPtr<ndDQNTrainer>(new ndDQNTrainer(model));
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

	ndSetRandSeed(42);
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
