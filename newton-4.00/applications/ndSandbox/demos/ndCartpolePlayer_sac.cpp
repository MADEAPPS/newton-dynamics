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

namespace ndCarpolePlayer_sac
{
	#define CONTROLLER_NAME			"cartpoleSac"
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

	class ndHelpLegend : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "This setup is a pole mounted on a cart");
			scene->Print(color, "a classic “hello world” example in reinforcement learning.");
			scene->Print(color, "It consists of a pole attached by a hinge to a sliding cart.");
			scene->Print(color, "The objective goal was to train a neural network to keep");
			scene->Print(color, "the pole balanced in an upright position.");
			scene->Print(color, "The model is trained using Soft Actor Critic(SAC) algorithm.");
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
			world->AddJoint(xDirSlider);
		}

		void SetController(ndDemoEntityManager* const scene,
			ndSharedPtr<ndMesh> mesh, ndSharedPtr<ndRenderSceneNode> visualMesh)
		{
			// create the model 
			CreateArticulatedModel(scene, GetModel()->GetAsModelArticulation(), mesh, visualMesh);
			ndModelArticulation::ndNode* const rootNode = GetModel()->GetAsModelArticulation()->GetRoot();
			m_cart = rootNode->m_body;
			m_pole = rootNode->GetFirstChild()->m_body;
			m_poleHinge = rootNode->GetFirstChild()->m_joint;

			// load the agent contaller
			char name[256];
			snprintf(name, sizeof(name) - 1, "%s.dnn", CONTROLLER_NAME);
			ndString fileName(ndGetWorkingFileName(name));
			ndSharedPtr<ndBrain> policy(ndBrainLoad::Load(fileName.GetStr()));
			m_controller = ndSharedPtr<ndBrainAgentContinuePolicyGradient> (new ndAgent(policy, this));
		}

		void Update(ndFloat32 timestep)
		{
			m_timestep = timestep;
			m_controller->Step();
		}

		ndFloat32 GetPoleAngle() const
		{
			ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
			ndFloat32 angle = hinge->GetAngle();
			return angle;
		}

		void GetObservation(ndBrainFloat* const observation)
		{
			ndVector omega(m_pole->GetOmega());
			ndFloat32 angle = GetPoleAngle();
			observation[m_poleAngle] = ndReal(angle);
			observation[m_poleOmega] = ndReal(omega.m_z);

			const ndVector cartVeloc(m_cart->GetVelocity());
			ndFloat32 cartSpeed = cartVeloc.m_x;
			observation[m_cartSpeed] = ndBrainFloat(cartSpeed);
		}

		void ApplyActions(ndBrainFloat* const actions)
		{
			ndVector force(m_cart->GetAsBodyDynamic()->GetForce());
			ndBrainFloat action = actions[0];
			ndBrainFloat accel = D_PUSH_ACCEL * action;
			force.m_x = ndFloat32(accel * (m_cart->GetAsBodyDynamic()->GetMassMatrix().m_w));
			m_cart->GetAsBodyDynamic()->SetForce(force);
		}

		ndSharedPtr<ndBody> m_cart;
		ndSharedPtr<ndBody> m_pole;
		ndSharedPtr<ndJointBilateralConstraint> m_poleHinge;
		ndSharedPtr<ndBrainAgentContinuePolicyGradient> m_controller;
		ndFloat32 m_timestep;
	};


	static void CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, const ndRenderMeshLoader& loader)
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
		((ndPlaybackController*)(*controller))->SetController(scene, loader.m_mesh, visualMesh);

		// add model a visual mesh to the scene and world
		world->AddModel(model);
		scene->AddEntity(visualMesh);
		model->AddBodiesAndJointsToWorld();
	}
}
using namespace ndCarpolePlayer_sac;


void ndCartpoleSacPlayer(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	// add a help message
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend());
	scene->SetDemoHelp(demoHelper);

	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("cartpole.nd"));
	CreateModel(scene, matrix, loader);

	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}