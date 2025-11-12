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
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"

namespace ndCarpolePlayer
{
	#define CONTROLLER_NAME_SAC		"cartpoleSac"
	#define CONTROLLER_NAME_PPO		"cartpolePpo"
	#define CART_MASS				ndFloat32(10.0f)
	#define POLE_MASS				ndFloat32(5.0f)
	#define D_PUSH_ACCEL			ndBrainFloat (-10.0f * DEMO_GRAVITY)

	enum ndActionSpace
	{
		m_softPush,
		m_actionsSize
	};

	enum ndStateSpace
	{
		m_poleAngle,
		m_poleOmega,
		m_cartSpeed,
		m_observationsSize
	};

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

	class ndPlaybackController : public ndModelNotify
	{
		class ndAgent : public ndBrainAgentContinuePolicyGradient
		{
			public:
			ndAgent(ndSharedPtr<ndBrain>& brain, ndPlaybackController* const owner)
				:ndBrainAgentContinuePolicyGradient(brain)
				,m_owner(owner)
			{
			}
		
			void GetObservation(ndBrainFloat* const observation)
			{
				m_owner->GetObservation(observation);
			}
		
			virtual void ApplyActions(ndBrainFloat* const actions)
			{
				m_owner->ApplyActions(actions);
			}
		
			ndPlaybackController* m_owner;
		};

		public:
		ndPlaybackController()
			:ndModelNotify()
			,m_timestep(0.0f)
		{
		}

		void CreateArticulatedModel(
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
			ndSharedPtr<ndBody> cart(CreateRigidBody(mesh, visualMesh, CART_MASS, nullptr));
			ndModelArticulation::ndNode* const modelRootNode = model->AddRootBody(cart);

			// add the pole mesh and body
			ndSharedPtr<ndMesh> poleMesh(mesh->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndRenderSceneNode> poleEntity(visualMesh->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndBody> pole(CreateRigidBody(poleMesh, poleEntity, POLE_MASS, cart->GetAsBodyDynamic()));

			const ndMatrix poleMatrix(ndYawMatrix(ndFloat32(90.0f) * ndDegreeToRad) * cart->GetMatrix());
			ndSharedPtr<ndJointBilateralConstraint> poleHinge(new ndJointHinge(poleMatrix, pole->GetAsBodyKinematic(), cart->GetAsBodyKinematic()));
			model->AddLimb(modelRootNode, pole, poleHinge);

			ndWorld* const world = scene->GetWorld();
			const ndMatrix sliderMatrix(cart->GetMatrix());
			ndSharedPtr<ndJointBilateralConstraint> xDirSlider(new ndJointSlider(sliderMatrix, cart->GetAsBodyKinematic(), world->GetSentinelBody()));
			model->AddCloseLoop(xDirSlider);
		}

		void SetController(ndDemoEntityManager* const scene,
			ndSharedPtr<ndMesh> mesh, 
			ndSharedPtr<ndRenderSceneNode> visualMesh, 
			const char* const name)
		{
			// create the model 
			ndModelArticulation* const articulation = GetModel()->GetAsModelArticulation();
			CreateArticulatedModel(scene, articulation, mesh, visualMesh);
			ndModelArticulation::ndNode* const rootNode = GetModel()->GetAsModelArticulation()->GetRoot();
			m_cart = rootNode->m_body;
			m_pole = rootNode->GetFirstChild()->m_body;
			m_poleHinge = rootNode->GetFirstChild()->m_joint;
			m_slider = articulation->GetCloseLoops().GetFirst()->GetInfo().m_joint;

			// load the agent contaller
			char nameExt[256];
			snprintf(nameExt, sizeof(nameExt) - 1, "%s.dnn", name);
			ndString fileName(ndGetWorkingFileName(nameExt));
			ndSharedPtr<ndBrain> policy(ndBrainLoad::Load(fileName.GetStr()));
			m_controller = ndSharedPtr<ndBrainAgentContinuePolicyGradient> (new ndAgent(policy, this));
		}

		void Update(ndFloat32 timestep)
		{
			m_timestep = timestep;
			m_controller->Step();
		}

		void ApplyActions(ndBrainFloat* const actions)
		{
			ndBrainFloat action = actions[0];
			ndBrainFloat accel = D_PUSH_ACCEL * action;
			ndFloat32 pushForce = accel * (m_cart->GetAsBodyDynamic()->GetMassMatrix().m_w);

			ndJointSlider* const slider = (ndJointSlider*)*m_slider;
			const ndMatrix matrix(slider->CalculateGlobalMatrix0());

			ndVector force(m_cart->GetAsBodyDynamic()->GetForce() + matrix.m_front.Scale(pushForce));
			m_cart->GetAsBodyDynamic()->SetForce(force);
		}

		void GetObservation(ndBrainFloat* const observation)
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

		ndSharedPtr<ndBody> m_cart;
		ndSharedPtr<ndBody> m_pole;
		ndSharedPtr<ndJointBilateralConstraint> m_slider;
		ndSharedPtr<ndJointBilateralConstraint> m_poleHinge;
		ndSharedPtr<ndBrainAgentContinuePolicyGradient> m_controller;
		ndFloat32 m_timestep;
	};

	static void CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, const ndRenderMeshLoader& loader, const char* const name)
	{
		ndWorld* const world = scene->GetWorld();
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
		((ndPlaybackController*)(*controller))->SetController(scene, loader.m_mesh, visualMesh, name);

		// add model a visual mesh to the scene and world
		world->AddModel(model);
		scene->AddEntity(visualMesh);
		model->AddBodiesAndJointsToWorld();
	}
}
using namespace ndCarpolePlayer;

void ndCartpolePlayer_SAC(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	// add a help message
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend_Sac());
	scene->SetDemoHelp(demoHelper);

	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("cartpole.nd"));
	CreateModel(scene, matrix, loader, CONTROLLER_NAME_SAC);

	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}

void ndCartpolePlayer_PPO(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	// add a help message
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend_Ppo());
	scene->SetDemoHelp(demoHelper);

	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("cartpole.nd"));
	CreateModel(scene, matrix, loader, CONTROLLER_NAME_PPO);

	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}