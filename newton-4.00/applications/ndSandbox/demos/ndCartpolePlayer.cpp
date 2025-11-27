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
#include "ndCartpolePlayer.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"

namespace ndCarpolePlayer
{
	class ndHelpLegend_Sac : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "This setup is a pole mounted on a cart");
			scene->Print(color, "The model is trained using Soft Actor Critic(SAC) algorithm.");
			scene->Print(color, "a classic “hello world” example in reinforcement learning.");
			scene->Print(color, "It consists of a pole attached by a hinge to a sliding cart.");
			scene->Print(color, "The objective goal was to train a neural network to keep");
			scene->Print(color, "the pole balanced in an upright position.");
			scene->Print(color, "You can interact with the simulation and try.");
			scene->Print(color, "to knock the pole over using the mouse.");
		}
	};
	
	class ndHelpLegend_Ppo : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "This setup is a pole mounted on a cart");
			scene->Print(color, "The model is trained using Proximal Policy Gradinet (PPO) algorithm.");
			scene->Print(color, "a classic “hello world” example in reinforcement learning.");
			scene->Print(color, "It consists of a pole attached by a hinge to a sliding cart.");
			scene->Print(color, "The objective goal was to train a neural network to keep");
			scene->Print(color, "the pole balanced in an upright position.");
			scene->Print(color, "You can interact with the simulation and try.");
			scene->Print(color, "to knock the pole over using the mouse.");
		}
	};


	class ndPlaybackController : public ndController
	{
		public:
		ndPlaybackController()
			:ndController()
		{
		}

		ndSharedPtr<ndBrainAgent> m_saveAgent;
	};

	ndController::ndController()
		:ndModelNotify()
		,m_agent(nullptr)
		,m_timestep(0.0f)
	{
	}

	void ndController::Update(ndFloat32 timestep)
	{
		m_timestep = timestep;
		m_agent->Step();
	}

	bool ndController::IsTerminal() const
	{
		const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		ndFloat32 angle = hinge->GetAngle();
		bool fail = ndAbs(angle) > (REWARD_MIN_ANGLE * ndFloat32(2.0f));
		return fail;
	}

	void ndController::ResetModel()
	{
		ndMatrix cartMatrix(ndGetIdentityMatrix());
		cartMatrix.m_posit = m_cart->GetMatrix().m_posit;
		cartMatrix.m_posit.m_x = ndFloat32(0.0f);
		m_cart->SetMatrix(cartMatrix);

		const ndMatrix poleMatrix(m_poleHinge->CalculateGlobalMatrix1());
		m_pole->SetMatrix(poleMatrix);

		m_pole->SetOmega(ndVector::m_zero);
		m_pole->SetVelocity(ndVector::m_zero);

		m_cart->SetOmega(ndVector::m_zero);
		m_cart->SetVelocity(ndVector::m_zero);

		GetModel()->GetAsModelArticulation()->ClearMemory();
	}

	#pragma optimize( "", off )
	ndBrainFloat ndController::CalculateReward() const
	{
		if (IsTerminal())
		{
			return ndBrainFloat(-1.0f);
		}

		ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		ndJointSlider* const slider = (ndJointSlider*)*m_slider;

		ndFloat32 angle = hinge->GetAngle();
		ndFloat32 omega = hinge->GetOmega();
		ndFloat32 speed = slider->GetSpeed();

		ndFloat32 angleReward = ndExp(-ndFloat32(1000.0f) * angle * angle);
		ndFloat32 omegaReward = ndExp(-ndFloat32(1000.0f) * omega * omega);
		//ndFloat32 speedPenalty = ndFloat32(1.0f) - ndExp(-ndFloat32(400.0f) * speed * speed);
		ndFloat32 speedReward = ndExp(-ndFloat32(400.0f) * speed * speed);

		// add a penalty for high speed. 
		// this is the equivalent of adding drag to the slider joint 
		ndFloat32 reward = ndFloat32(1.0f/3.0f) * angleReward + ndFloat32(1.0f / 3.0f) * omegaReward + ndFloat32(1.0f / 3.0f) * speedReward;
		return ndBrainFloat(reward);
	}

	void ndController::ApplyActions(ndBrainFloat* const actions)
	{
		ndBrainFloat action = actions[0];
		ndBrainFloat accel = PUSH_ACCEL * action;
		ndFloat32 pushForce = accel * (m_cart->GetAsBodyDynamic()->GetMassMatrix().m_w);

		ndJointSlider* const slider = (ndJointSlider*)*m_slider;
		const ndMatrix matrix(slider->CalculateGlobalMatrix0());

		ndVector force(m_cart->GetAsBodyDynamic()->GetForce() + matrix.m_front.Scale(pushForce));
		m_cart->GetAsBodyDynamic()->SetForce(force);
	}

	void ndController::GetObservation(ndBrainFloat* const observation)
	{
		const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		const ndJointSlider* const slider = (ndJointSlider*)*m_slider;

		ndFloat32 omega = hinge->GetOmega();
		ndFloat32 angle = hinge->GetAngle();
		ndFloat32 speed = slider->GetSpeed();

		observation[m_poleAngle] = ndBrainFloat(angle);
		observation[m_poleOmega] = ndBrainFloat(omega);
		observation[m_cartSpeed] = ndBrainFloat(speed);
	}

	void ndController::CreateArticulatedModel(
		ndDemoEntityManager* const scene,
		ndModelArticulation* const model,
		ndSharedPtr<ndMesh> mesh,
		ndSharedPtr<ndRenderSceneNode> visualMesh)
	{
		auto CreateRigidBody = [scene](ndSharedPtr<ndMesh>& mesh, ndSharedPtr<ndRenderSceneNode>& visualMesh, ndFloat32 mass, ndBodyDynamic* const parentBody)
		{
			ndSharedPtr<ndShapeInstance> shape(mesh->CreateCollision());

			ndBodyKinematic* const body = new ndBodyDynamic();
			body->SetNotifyCallback(new ndDemoEntityNotify(scene, visualMesh, parentBody));
			body->SetMatrix(mesh->CalculateGlobalMatrix());
			body->SetCollisionShape(*(*shape));
			body->GetAsBodyDynamic()->SetMassMatrix(mass, *(*shape));
			return body;
		};

		// add the cart mesh and body
		m_cart = ndSharedPtr<ndBody>(CreateRigidBody(mesh, visualMesh, CART_MASS, nullptr));
		ndModelArticulation::ndNode* const modelRootNode = model->AddRootBody(m_cart);

		// add the pole mesh and body
		ndSharedPtr<ndMesh> poleMesh(mesh->GetChildren().GetFirst()->GetInfo());
		ndSharedPtr<ndRenderSceneNode> poleEntity(visualMesh->GetChildren().GetFirst()->GetInfo());
		m_pole = ndSharedPtr<ndBody>(CreateRigidBody(poleMesh, poleEntity, POLE_MASS, m_cart->GetAsBodyDynamic()));

		const ndMatrix poleMatrix(ndYawMatrix(ndFloat32(90.0f) * ndDegreeToRad) * m_cart->GetMatrix());
		m_poleHinge = ndSharedPtr<ndJointBilateralConstraint>(new ndJointHinge(poleMatrix, m_pole->GetAsBodyKinematic(), m_cart->GetAsBodyKinematic()));
		model->AddLimb(modelRootNode, m_pole, m_poleHinge);

		ndWorld* const world = scene->GetWorld();
		const ndMatrix sliderMatrix(m_cart->GetMatrix());
		m_slider = ndSharedPtr<ndJointBilateralConstraint>(new ndJointSlider(sliderMatrix, m_cart->GetAsBodyKinematic(), world->GetSentinelBody()));
		model->AddCloseLoop(m_slider);
	}

	ndModelArticulation* ndController::CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, const ndRenderMeshLoader& loader, const char* const name)
	{
		ndMatrix matrix(location);
		matrix.m_posit = FindFloor(*scene->GetWorld(), matrix.m_posit, 200.0f);
		matrix.m_posit.m_y += ndFloat32(0.1f);
		loader.m_mesh->m_matrix = loader.m_mesh->m_matrix * matrix;
		
		ndSharedPtr<ndRenderSceneNode> visualMesh(loader.m_renderMesh->Clone());
		visualMesh->SetTransform(loader.m_mesh->m_matrix);
		visualMesh->SetTransform(loader.m_mesh->m_matrix);
		
		ndModelArticulation* const model = new ndModelArticulation();
		ndSharedPtr<ndModelNotify> controller(new ndPlaybackController());
		model->SetNotifyCallback(controller);
		ndPlaybackController* const playerController = (ndPlaybackController*)(*controller);
		playerController->CreateArticulatedModel(scene, model, loader.m_mesh, visualMesh);

		char nameExt[256];
		snprintf(nameExt, sizeof(nameExt) - 1, "%s.dnn", name);
		ndString fileName(ndGetWorkingFileName(nameExt));
		ndSharedPtr<ndBrain> policy(ndBrainLoad::Load(fileName.GetStr()));
		playerController->m_saveAgent = ndSharedPtr<ndBrainAgent>(new ndController::ndAgent(policy, playerController));
		playerController->m_agent = *playerController->m_saveAgent;

		// add model a visual mesh to the scene and world
		ndWorld* const world = scene->GetWorld();
		world->AddModel(model);
		scene->AddEntity(visualMesh);
		model->AddBodiesAndJointsToWorld();

		return model;
	}
}

using namespace ndCarpolePlayer;

void ndCartpolePlayer_PPO(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	// add a help message
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend_Ppo());
	scene->SetDemoHelp(demoHelper);

	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("cartpole.nd"));
	ndController::CreateModel(scene, matrix, loader, CONTROLLER_NAME_PPO);

	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}

void ndCartpolePlayer_SAC(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	// add a help message
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend_Sac());
	scene->SetDemoHelp(demoHelper);

	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("cartpole.nd"));
	ndController::CreateModel(scene, matrix, loader, CONTROLLER_NAME_SAC);

	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}