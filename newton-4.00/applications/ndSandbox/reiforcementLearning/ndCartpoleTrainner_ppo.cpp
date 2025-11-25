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

namespace ndCartpoleTrainer_ppo
{
	#define CONTROLLER_NAME			"cartpolePpo"
	#define CART_MASS				ndFloat32(10.0f)
	#define POLE_MASS				ndFloat32(5.0f)

	#define ND_TRAJECTORY_STEPS		(1024 * 4)
	#define D_PUSH_ACCEL			ndBrainFloat (-10.0f * DEMO_GRAVITY)
	#define D_REWARD_MIN_ANGLE		ndBrainFloat (20.0f * ndDegreeToRad)

	class ndHelpLegend : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "training a cart pole using Proximal Policy Gradient method");
		}
	};

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

	class CartPoleMaterial : public ndApplicationMaterial
	{
		public:
		CartPoleMaterial()
			:ndApplicationMaterial()
		{
		}

		CartPoleMaterial(const CartPoleMaterial& src)
			:ndApplicationMaterial(src)
		{
		}

		ndApplicationMaterial* Clone() const
		{
			return new CartPoleMaterial(*this);
		}

		virtual bool OnAabbOverlap(const ndBodyKinematic* const, const ndBodyKinematic* const) const override
		{
			return false;
		}
	};

	static void CreateArticulatedModel(
		ndDemoEntityManager* const scene,
		ndModelArticulation* const model,
		ndSharedPtr<ndMesh> mesh,
		ndSharedPtr<ndRenderSceneNode> visualMesh)
	{
		auto CreateRigidBody = [scene](ndSharedPtr<ndMesh>& mesh, ndSharedPtr<ndRenderSceneNode>& visualMesh, ndFloat32 mass, ndBodyDynamic* const parentBody)
		{
			ndSharedPtr<ndShapeInstance> shape(mesh->CreateCollision());
			shape->m_shapeMaterial.m_userId = ndDemoContactCallback::m_modelPart;

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

	class ndTrainerController : public ndModelNotify
	{
		class ndAgent : public ndBrainAgentOnPolicyGradient_Agent
		{
			public:
			ndAgent(ndSharedPtr<ndBrainAgentOnPolicyGradient_Trainer>& master, ndTrainerController* const owner)
				:ndBrainAgentOnPolicyGradient_Agent(*master)
				,m_owner (owner)
			{
			}

			ndBrainFloat CalculateReward()
			{
				return m_owner->CalculateReward();
			}

			bool IsTerminal() const
			{
				return m_owner->IsTerminal();
			}

			void GetObservation(ndBrainFloat* const observation)
			{
				m_owner->GetObservation(observation);
			}

			virtual void ApplyActions(ndBrainFloat* const actions)
			{
				m_owner->ApplyActions(actions);
			}

			void ResetModel()
			{
				m_owner->ResetModel();
			}

			ndTrainerController* m_owner;
		};

		public:
		ndTrainerController()
			:ndModelNotify()
			,m_timestep(0.0f)
		{
		}

		void SetController(ndDemoEntityManager* const scene,
			ndSharedPtr<ndBrainAgentOnPolicyGradient_Trainer>& master,
			ndSharedPtr<ndMesh> mesh, ndSharedPtr<ndRenderSceneNode> visualMesh)
		{
			m_controllerTrainer = ndSharedPtr<ndBrainAgentOnPolicyGradient_Agent>(new ndAgent(master, this));
			master->AddAgent(m_controllerTrainer);

			ndModelArticulation* const articulation = GetModel()->GetAsModelArticulation();
			CreateArticulatedModel(scene, articulation, mesh, visualMesh);
			ndModelArticulation::ndNode* const rootNode = articulation->GetRoot();

			m_cart = rootNode->m_body;
			m_pole = rootNode->GetFirstChild()->m_body;
			m_poleHinge = rootNode->GetFirstChild()->m_joint;
			m_slider = articulation->GetCloseLoops().GetFirst()->GetInfo().m_joint;
		}

		void Update(ndFloat32 timestep)
		{
			m_timestep = timestep;
			m_controllerTrainer->Step();
		}

		void PostUpdate(ndFloat32)
		{
			if (IsOutOfBounds())
			{
				TelePort();
			}
		}

		void TelePort() const
		{
			//ndModelArticulation* const model = (ndModelArticulation*)GetModel();
			//ndBodyKinematic* const body = model->GetRoot()->m_body->GetAsBodyKinematic();
			//
			//ndVector posit(body->GetMatrix().m_posit);
			//posit.m_y = 0.0f;
			//posit.m_z = 0.0f;
			//posit.m_w = 0.0f;
			//for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
			//{
			//	ndBodyKinematic* const modelBody = m_bodies[i];
			//	ndMatrix matrix(modelBody->GetMatrix());
			//	matrix.m_posit -= posit;
			//	modelBody->SetMatrix(matrix);
			//}
		}

		void ResetModel()
		{
			ndMatrix cartMatrix(ndGetIdentityMatrix());

			cartMatrix.m_posit = m_cart->GetMatrix().m_posit;
			cartMatrix.m_posit.m_x = ndFloat32(10.0f) * (ndRand() - ndFloat32(0.5f));
			cartMatrix.m_posit.m_y = ndFloat32(0.1f);
			m_cart->SetMatrix(cartMatrix);

			const ndMatrix poleMatrix(m_poleHinge->CalculateGlobalMatrix1());
			m_pole->SetMatrix(poleMatrix);

			m_pole->SetOmega(ndVector::m_zero);
			m_pole->SetVelocity(ndVector::m_zero);

			m_cart->SetOmega(ndVector::m_zero);
			m_cart->SetVelocity(ndVector::m_zero);

			GetModel()->GetAsModelArticulation()->ClearMemory();
		}

		bool IsOutOfBounds() const
		{
			return ndAbs(m_cart->GetMatrix().m_posit.m_x) > ndFloat32(20.0f);
		}

		bool IsTerminal() const
		{
			const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
			ndFloat32 angle = hinge->GetAngle();
			bool fail = ndAbs(angle) > (D_REWARD_MIN_ANGLE * ndFloat32(2.0f));
			return fail;
		}

		#pragma optimize( "", off )
		ndBrainFloat CalculateReward()
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
			ndFloat32 speedPenalty = ndFloat32(1.0f) - ndExp(-ndFloat32(100.0f) * speed * speed);

			// add a penalty for high speed. 
			// this is the equivalent of adding drag to the slider joint 
			ndFloat32 reward = ndFloat32(0.6f) * angleReward + ndFloat32(0.4f) * omegaReward - ndFloat32(0.25f) * speedPenalty;
			return ndBrainFloat(reward);
		}

		void ApplyActions(ndBrainFloat* const actions)
		{
			ndBrainFloat action = actions[0];
			ndFloat32 pushForce = ndBrainFloat(D_PUSH_ACCEL * action * (m_cart->GetAsBodyDynamic()->GetMassMatrix().m_w));

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
		ndSharedPtr<ndBrainAgentOnPolicyGradient_Agent> m_controllerTrainer;
		ndFloat32 m_timestep;
	};

	class TrainingUpdata : public ndDemoEntityManager::OnPostUpdate
	{
		public:
		TrainingUpdata(ndDemoEntityManager* const scene, const ndMatrix& location, const ndRenderMeshLoader& loader)
			:OnPostUpdate()
			,m_master()
			,m_bestActor()
			,m_outFile(nullptr)
			,m_timer(ndGetTimeInMicroseconds())
			,m_maxScore(ndFloat32(-1.0e10f))
			,m_saveScore(m_maxScore)
			,m_discountRewardFactor(0.99f)
			,m_horizon(ndFloat32(1.0f) / (ndFloat32(1.0f) - m_discountRewardFactor))
			,m_lastEpisode(0xfffffff)
			,m_stopTraining(100 * 1000000)
			,m_modelIsTrained(false)
		{
			char name[256];
			snprintf(name, sizeof(name), "%s-ppo.csv", CONTROLLER_NAME);
			m_outFile = fopen(name, "wb");
			fprintf(m_outFile, "ppo\n");

			// create a Soft Actor Critic traniing agent
			ndBrainAgentOnPolicyGradient_Trainer::HyperParameters hyperParameters;

			hyperParameters.m_batchTrajectoryCount = 100;
			hyperParameters.m_numberOfActions = m_actionsSize;
			hyperParameters.m_numberOfObservations = m_observationsSize;
			hyperParameters.m_maxTrajectorySteps = ND_TRAJECTORY_STEPS;
			hyperParameters.m_discountRewardFactor = ndReal(m_discountRewardFactor);

			m_master = ndSharedPtr<ndBrainAgentOnPolicyGradient_Trainer>(new ndBrainAgentOnPolicyGradient_Trainer(hyperParameters));
			m_bestActor = ndSharedPtr< ndBrain>(new ndBrain(*m_master->GetPolicyNetwork()));

			snprintf(name, sizeof(name), "%s.dnn", CONTROLLER_NAME);
			m_master->SetName(name);

			// create a visual mesh and add to the scene.
			ndWorld* const world = scene->GetWorld();
			ndMatrix matrix(location);
			matrix.m_posit.m_y = ndFloat32(0.1f);
			loader.m_mesh->m_matrix = loader.m_mesh->m_matrix * matrix;

			// create an articulated model
			const ndInt32 numberOfAgents = 10;
			for (ndInt32 i = 0; i < numberOfAgents; ++i)
			{
				loader.m_mesh->m_matrix.m_posit.m_x = ndFloat32(10.0f) * (ndRand() - ndFloat32(0.5f));
				ndSharedPtr<ndRenderSceneNode> visualMesh(loader.m_renderMesh->Clone());
				visualMesh->SetTransform(loader.m_mesh->m_matrix);
				visualMesh->SetTransform(loader.m_mesh->m_matrix);

				ndSharedPtr<ndModel>model(CreateModel(scene, loader.m_mesh, visualMesh));

				// add model a visual mesh to the scene and world
				world->AddModel(model);
				scene->AddEntity(visualMesh);
				model->AddBodiesAndJointsToWorld();
			}
		}

		~TrainingUpdata()
		{
			if (m_outFile)
			{
				fclose(m_outFile);
			}
		}

		ndModelArticulation* CreateModel(
			ndDemoEntityManager* const scene, 
			ndSharedPtr<ndMesh> mesh,
			ndSharedPtr<ndRenderSceneNode> visualMesh)
		{
			ndModelArticulation* const model = new ndModelArticulation();
			ndSharedPtr<ndModelNotify> controller(new ndTrainerController());
			model->SetNotifyCallback(controller);
			((ndTrainerController*)(*controller))->SetController(scene, m_master, mesh, visualMesh);

			return model;
		}

		void OnDebug(ndDemoEntityManager* const, bool)
		{
		}

		virtual void Update(ndDemoEntityManager* const manager, ndFloat32)
		{
			ndUnsigned32 stopTraining = m_master->GetFramesCount();
			if (stopTraining <= m_stopTraining)
			{
				ndUnsigned32 episodeCount = m_master->GetEposideCount();
				m_master->OptimizeStep();

				episodeCount -= m_master->GetEposideCount();
				ndFloat32 trajectoryLog = ndLog(m_master->GetAverageFrames() + 0.001f);
				ndFloat32 rewardTrajectory = m_master->GetAverageScore() * trajectoryLog;
				if (rewardTrajectory >= ndFloat32(m_maxScore))
				{
					if (m_lastEpisode != m_master->GetEposideCount())
					{
						m_maxScore = rewardTrajectory;
						m_bestActor->CopyFrom(*m_master->GetPolicyNetwork());
						ndExpandTraceMessage("best actor episode: %d\treward %f\ttrajectoryFrames: %f\n", m_master->GetEposideCount(), 100.0f * m_master->GetAverageScore() / m_horizon, m_master->GetAverageFrames());
						m_lastEpisode = m_master->GetEposideCount();
					}
				}

				if (rewardTrajectory > m_saveScore)
				{
					m_saveScore = ndFloor(rewardTrajectory) + 2.0f;

					// save partial controller in case of crash 
					ndBrain* const actor = m_master->GetPolicyNetwork();
					ndString fileName(ndGetWorkingFileName(m_master->GetName().GetStr()));
					m_master->GetPolicyNetwork()->SaveToFile(fileName.GetStr());
					actor->SaveToFile(fileName.GetStr());
				}

				if (episodeCount && !m_master->IsSampling())
				{
					ndExpandTraceMessage("steps: %d\treward: %g\t  trajectoryFrames: %g\n", m_master->GetFramesCount(), 100.0f * m_master->GetAverageScore() / m_horizon, m_master->GetAverageFrames());
					if (m_outFile)
					{
						fprintf(m_outFile, "%g\n", m_master->GetAverageScore());
						fflush(m_outFile);
					}
				}
			}

			if ((stopTraining >= m_stopTraining) || (m_master->GetAverageScore() > ndBrainFloat(0.97f)))
			{
				m_modelIsTrained = true;
				m_master->GetPolicyNetwork()->CopyFrom(*(*m_bestActor));
				ndString fileName (ndGetWorkingFileName(m_master->GetName().GetStr()));
				m_master->GetPolicyNetwork()->SaveToFile(fileName.GetStr());
				ndExpandTraceMessage("saving to file: %s\n", fileName.GetStr());
				ndExpandTraceMessage("training complete\n");
				ndUnsigned64 timer = ndGetTimeInMicroseconds() - m_timer;
				ndExpandTraceMessage("training time: %g seconds\n", ndFloat32(ndFloat64(timer) * ndFloat32(1.0e-6f)));

				manager->Terminate();
			}
		}

		ndSharedPtr<ndBrainAgentOnPolicyGradient_Trainer> m_master;
		ndSharedPtr<ndBrain> m_bestActor;
		ndList<ndModelArticulation*> m_models;
		FILE* m_outFile;
		ndUnsigned64 m_timer;
		ndFloat32 m_maxScore;
		ndFloat32 m_saveScore;
		ndFloat32 m_discountRewardFactor;
		ndFloat32 m_horizon;
		ndUnsigned32 m_lastEpisode;
		ndUnsigned32 m_stopTraining;
		bool m_modelIsTrained;
	};
}

using namespace ndCartpoleTrainer_ppo;


void ndCartpolePpoTraining(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	// add a help message
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend());
	scene->SetDemoHelp(demoHelper);

	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("cartpole.nd"));

	// create a material that make the objects in training not collsionnle
	CartPoleMaterial material;
	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
	callback->RegisterMaterial(material, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_modelPart);

	ndSetRandSeed(42);
	ndSharedPtr<ndDemoEntityManager::OnPostUpdate>trainer(new TrainingUpdata(scene, matrix, loader));
	scene->RegisterPostUpdate(trainer);

	// supress v sync refresh rate
	scene->SetAcceleratedUpdate();
	
	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 8.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}