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

namespace ndContinueCarpole
{
	#define CONTROLLER_NAME			"cartpoleContinue"

	#define ND_TRAJECTORY_STEPS		(1024 * 4)
	#define D_PUSH_ACCEL			ndBrainFloat (-10.0f * DEMO_GRAVITY)
	#define D_REWARD_MIN_ANGLE		ndBrainFloat (20.0f * ndDegreeToRad)

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

	class ndTrainerController : public ndModelNotify
	{
		class ndAgent : public ndBrainAgentOffPolicyGradient_Agent
		{
			public:
			ndAgent(ndSharedPtr<ndBrainAgentOffPolicyGradient_Trainer>& master, ndTrainerController* const owner)
				:ndBrainAgentOffPolicyGradient_Agent(*master)
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

		void SetControllerTrainer(ndDemoEntityManager* const scene,
			ndSharedPtr<ndBrainAgentOffPolicyGradient_Trainer>& master,
			ndSharedPtr<ndMesh> mesh, ndSharedPtr<ndRenderSceneNode> visualMesh)
		{
			m_controllerTrainer = ndSharedPtr<ndBrainAgentOffPolicyGradient_Agent>(new ndAgent(master, this));
			master->SetAgent(m_controllerTrainer);

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

			ndFloat32 cartMass = ndFloat32(10.0f);
			ndFloat32 poleMass = ndFloat32(5.0f);

			ndModelArticulation* const model = GetModel()->GetAsModelArticulation();

			// add the cart mesh and body
			m_cart = ndSharedPtr<ndBody>(CreateRigidBody(mesh, visualMesh, cartMass, nullptr));
			ndModelArticulation::ndNode* const modelRootNode = model->AddRootBody(m_cart);

			// add the pole mesh and body
			ndSharedPtr<ndMesh> poleMesh(mesh->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndRenderSceneNode> poleEntity(visualMesh->GetChildren().GetFirst()->GetInfo());
			m_pole = ndSharedPtr<ndBody> (CreateRigidBody(poleMesh, poleEntity, poleMass, m_cart->GetAsBodyDynamic()));

			const ndMatrix poleMatrix(ndYawMatrix(ndFloat32 (90.0f) * ndDegreeToRad) * m_cart->GetMatrix());
			m_poleHinge = ndSharedPtr<ndJointBilateralConstraint>(new ndJointHinge(poleMatrix, m_pole->GetAsBodyKinematic(), m_cart->GetAsBodyKinematic()));
			model->AddLimb(modelRootNode, m_pole, m_poleHinge);

			ndWorld* const world = scene->GetWorld();
			const ndMatrix sliderMatrix(m_cart->GetMatrix());
			ndSharedPtr<ndJointBilateralConstraint> xDirSlider(new ndJointSlider(sliderMatrix, m_cart->GetAsBodyKinematic(), world->GetSentinelBody()));
			world->AddJoint(xDirSlider);
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
			ndMatrix cartMatrix(m_cart->GetMatrix());
			ndMatrix poleMatrix(m_pole->GetMatrix());

			ndFloat32 deltax = cartMatrix.m_posit.m_x;

			cartMatrix.m_posit.m_x -= deltax;
			poleMatrix.m_posit.m_x -= deltax;

			m_cart->SetMatrix(cartMatrix);
			m_pole->SetMatrix(poleMatrix);
		}

		void ResetModel()
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

		bool IsOutOfBounds() const
		{
			return ndAbs(m_cart->GetMatrix().m_posit.m_x) > ndFloat32(20.0f);
		}

		ndFloat32 GetPoleAngle() const
		{
			ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
			ndFloat32 angle = hinge->GetAngle();
			return angle;
		}

		bool IsTerminal() const
		{
			ndFloat32 angle = GetPoleAngle();
			bool fail = ndAbs(angle) > (D_REWARD_MIN_ANGLE * ndFloat32(2.0f));
			return fail;
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

		ndBrainFloat CalculateReward()
		{
			if (IsTerminal())
			{
				return ndReal(0.0f);
			}

			// maybe adding a penalty for high velocity, 
			// but it seem if doesn't need it.
			//const ndVector veloc(m_cart->GetVelocity());
			//ndFloat32 speedReward = ndReal(ndExp(-ndFloat32(200.0f) * veloc.m_x * veloc.m_x));

			ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
			ndFloat32 angle = hinge->GetAngle();
			ndFloat32 omega = hinge->GetOmega();
			ndFloat32 angleReward = ndReal(ndExp(-ndFloat32(1000.0f) * angle * angle));
			ndFloat32 omegaReward = ndReal(ndExp(-ndFloat32(1000.0f) * omega * omega));
			//ndTrace(("a=%f w=%f ra=%f rw=%f\n", angle, omega, angleReward, omegaReward));

			ndFloat32 reward = 0.6f * angleReward + 0.4f * omegaReward;
			return ndReal(reward);
		}

		void ApplyActions(ndBrainFloat* const actions)
		{
			ndVector force(m_cart->GetAsBodyDynamic()->GetForce());
			ndBrainFloat action = actions[0];
			ndBrainFloat accel = D_PUSH_ACCEL * action;
			force.m_x = ndFloat32(accel * (m_cart->GetAsBodyDynamic()->GetMassMatrix().m_w));
			m_cart->GetAsBodyDynamic()->SetForce(force);
		}

		ndIkSolver m_solver;
		ndSharedPtr<ndBody> m_cart;
		ndSharedPtr<ndBody> m_pole;
		ndSharedPtr<ndJointBilateralConstraint> m_poleHinge;
		ndSharedPtr<ndBrainAgentOffPolicyGradient_Agent> m_controllerTrainer;
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
			,m_stopTraining(500000)
			,m_modelIsTrained(false)
		{
			char name[256];
			snprintf(name, sizeof(name), "%s-vpg.csv", CONTROLLER_NAME);
			m_outFile = fopen(name, "wb");
			fprintf(m_outFile, "vpg\n");

			// create a Soft Actor Critic traniing agent
			ndBrainAgentOffPolicyGradient_Trainer::HyperParameters hyperParameters;
			hyperParameters.m_numberOfActions = m_actionsSize;
			hyperParameters.m_numberOfObservations = m_observationsSize;
			m_master = ndSharedPtr<ndBrainAgentOffPolicyGradient_Trainer>(new ndBrainAgentOffPolicyGradient_Trainer(hyperParameters));
			
			m_bestActor = ndSharedPtr< ndBrain>(new ndBrain(*m_master->GetPolicyNetwork()));

			snprintf(name, sizeof(name), "%s.dnn", CONTROLLER_NAME);
			m_master->SetName(name);

			// create a visual mesh and add to the scene.
			ndWorld* const world = scene->GetWorld();
			ndMatrix matrix(location);
			matrix.m_posit = FindFloor(*scene->GetWorld(), matrix.m_posit, 200.0f);
			matrix.m_posit.m_y += ndFloat32 (0.1f);
			loader.m_mesh->m_matrix = loader.m_mesh->m_matrix * matrix;

			ndSharedPtr<ndRenderSceneNode> visualMesh(loader.m_renderMesh->Clone());
			visualMesh->SetTransform(loader.m_mesh->m_matrix);
			visualMesh->SetTransform(loader.m_mesh->m_matrix);

			// create an articulated model
			ndSharedPtr<ndModel>model(CreateModel(scene, loader.m_mesh, visualMesh));

			// add model a visual mesh to the scene and world
			world->AddModel(model);
			scene->AddEntity(visualMesh);
			model->AddBodiesAndJointsToWorld();
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
			((ndTrainerController*)(*controller))->SetControllerTrainer(scene, m_master, mesh, visualMesh);

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
					char name[256];
					m_saveScore = ndFloor(rewardTrajectory) + 2.0f;

					// save partial controller in case of crash 
					ndBrain* const actor = m_master->GetPolicyNetwork();
					snprintf(name, sizeof(name), "%s_actor.dnn", CONTROLLER_NAME);
					ndString fileName (ndGetWorkingFileName(name));
					actor->SaveToFile(fileName.GetStr());
					//ndBrain* const critic = m_master->GetValueNetwork();
					//snprintf(name, sizeof(name), "%s_critic.dnn", CONTROLLER_NAME);
					//ndGetWorkingFileName(name, fileName);
					//critic->SaveToFile(fileName);
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

			if ((stopTraining >= m_stopTraining) || (100.0f * m_master->GetAverageScore() / m_horizon > 95.0f))
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

		ndSharedPtr<ndBrainAgentOffPolicyGradient_Trainer> m_master;
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
using namespace ndContinueCarpole;

void ndCartpoleSacPlayer(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	////ndMeshLoader loader;
	////ndSharedPtr<ndDemoEntity> modelMesh(loader.LoadEntity("cartpole.fbx", scene));
	//ndRenderMeshLoader loader(*scene->GetRenderer());
	//loader.LoadMesh(ndGetWorkingFileName("cartpole.nd"));
	//ndSharedPtr<ndDemoEntityManager::OnPostUpdate>trainer(new TrainingUpdata(scene, matrix, loader));
	//scene->RegisterPostUpdate(trainer);
	////#else
	////ndWorld* const world = scene->GetWorld();
	////ndModelArticulation* const model = CreateModel(scene, matrix, modelMesh);
	////world->AddModel(model);
	////model->AddBodiesAndJointsToWorld();
	////
	////// add the deep learning controller
	////char name[256];
	////char fileName[256];
	////snprintf(name, sizeof(name), "%s.dnn", CONTROLLER_NAME);
	////ndGetWorkingFileName(name, fileName);
	////
	////ndSharedPtr<ndBrain> policy(ndBrainLoad::Load(fileName));
	////model->SetNotifyCallback(new RobotModelNotify(policy, model));
	////#endif

	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("cartpole.nd"));
	
	//ndSetRandSeed(42);
	//ndSharedPtr<ndDemoEntityManager::OnPostUpdate>trainer(new TrainingUpdata(scene, matrix, loader));
	//scene->RegisterPostUpdate(trainer);

	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}

void ndCartpoleSacTraining(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("cartpole.nd"));

	ndSetRandSeed(42);
	ndSharedPtr<ndDemoEntityManager::OnPostUpdate>trainer (new TrainingUpdata(scene, matrix, loader));
	scene->RegisterPostUpdate(trainer);

	// supress v sync refress rate
	scene->SetAcceleratedUpdate();
	
	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
