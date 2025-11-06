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
	//#define D_PUSH_ACCEL			ndBrainFloat (-3.0f * DEMO_GRAVITY)
	//#define D_REWARD_MIN_ANGLE		ndBrainFloat (20.0f * ndDegreeToRad)

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

#if 0
	ndModelArticulation* CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, const ndSharedPtr<ndDemoEntity>& modelMesh)
	{
#if 0
		ndModelArticulation* const model = BuildModelOldModel(scene, location);
#else
		
		ndModelArticulation* const model = new ndModelArticulation();
		ndSharedPtr<ndDemoEntity> entity(modelMesh->GetChildren().GetFirst()->GetInfo()->CreateClone());
		scene->AddEntity(entity);

		auto CreateRigidBody = [scene](ndSharedPtr<ndDemoEntity>& entity, const ndMatrix& matrix, ndFloat32 mass, ndBodyDynamic* const parentBody)
		{
			ndSharedPtr<ndShapeInstance> shape(entity->CreateCollision());

			ndBodyKinematic* const body = new ndBodyDynamic();
			body->SetNotifyCallback(new ndBindingRagdollEntityNotify(scene, entity, parentBody, 100.0f));
			body->SetMatrix(matrix);
			body->SetCollisionShape(*(*shape));
			body->GetAsBodyDynamic()->SetMassMatrix(mass, *(*shape));
			return body;
		};

		ndFloat32 cartMass = 5.0f;
		ndFloat32 poleMass = 10.0f;

		ndMatrix matrix(entity->GetCurrentMatrix() * location);
		matrix.m_posit = location.m_posit;
		ndSharedPtr<ndBody> rootBody(CreateRigidBody(entity, matrix, cartMass, nullptr));
		ndModelArticulation::ndNode* const modelRootNode = model->AddRootBody(rootBody);

		ndSharedPtr<ndDemoEntity> poleEntity(entity->GetChildren().GetFirst()->GetInfo());
		const ndMatrix poleMatrix(poleEntity->GetCurrentMatrix() * matrix);
		ndSharedPtr<ndBody> pole(CreateRigidBody(poleEntity, poleMatrix, poleMass, rootBody->GetAsBodyDynamic()));
		ndSharedPtr<ndJointBilateralConstraint> poleHinge(new ndJointHinge(poleMatrix, pole->GetAsBodyKinematic(), rootBody->GetAsBodyKinematic()));
		model->AddLimb(modelRootNode, pole, poleHinge);

		ndWorld* const world = scene->GetWorld();
		const ndMatrix sliderMatrix(rootBody->GetMatrix());
		ndSharedPtr<ndJointBilateralConstraint> xDirSlider(new ndJointSlider(sliderMatrix, rootBody->GetAsBodyKinematic(), world->GetSentinelBody()));
		world->AddJoint(xDirSlider);

		//ndUrdfFile urdf;
		//char fileName[256];
		//ndGetWorkingFileName("cartpole.urdf", fileName);
		//urdf.Export(fileName, model->GetAsModelArticulation());
#endif

		return model;
	}

	class RobotModelNotify : public ndModelNotify
	{
		class ndController : public ndBrainAgentContinuePolicyGradient
		{
			public:
			ndController(const ndSharedPtr<ndBrain>& brain)
				:ndBrainAgentContinuePolicyGradient(brain)
				,m_robot(nullptr)
			{
			}

			ndController(const ndController& src)
				:ndBrainAgentContinuePolicyGradient(src.m_policy)
				,m_robot(nullptr)
			{
			}

			void GetObservation(ndBrainFloat* const observation)
			{
				m_robot->GetObservation(observation);
			}

			virtual void ApplyActions(ndBrainFloat* const actions)
			{
				m_robot->ApplyActions(actions);
			}

			RobotModelNotify* m_robot;
		};

		class ndControllerTrainer : public ndBrainAgentContinuePolicyGradient_Agent
		{
			public:
			ndControllerTrainer(const ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master)
				:ndBrainAgentContinuePolicyGradient_Agent(master)
				,m_solver()
				,m_robot(nullptr)
			{
			}

			ndControllerTrainer(const ndControllerTrainer& src)
				:ndBrainAgentContinuePolicyGradient_Agent(src.m_owner)
				,m_robot(nullptr)
			{
			}

			ndBrainFloat CalculateReward()
			{
				return m_robot->GetReward();
			}

			bool IsTerminal() const
			{
				return m_robot->IsTerminal();
			}

			void GetObservation(ndBrainFloat* const observation)
			{
				m_robot->GetObservation(observation);
			}

			virtual void ApplyActions(ndBrainFloat* const actions)
			{
				m_robot->ApplyActions(actions);
			}

			void ResetModel()
			{
				m_robot->ResetModel();
			}

			ndIkSolver m_solver;
			RobotModelNotify* m_robot;
		};

		public:
		RobotModelNotify(ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master, ndModelArticulation* const robot)
			:ndModelNotify()
			,m_controller(nullptr)
			,m_controllerTrainer(nullptr)
		{
			m_controllerTrainer = new ndControllerTrainer(master);
			m_controllerTrainer->m_robot = this;
			Init(robot);
		}

		RobotModelNotify(const ndSharedPtr<ndBrain>& policy, ndModelArticulation* const robot)
			:ndModelNotify()
			,m_controller(nullptr)
			,m_controllerTrainer(nullptr)
		{
			m_controller = new ndController(policy);
			m_controller->m_robot = this;
			Init(robot);
		}

		RobotModelNotify(const RobotModelNotify& src)
			:ndModelNotify(src)
			,m_controller(src.m_controller)
		{
			//Init(robot);
			ndAssert(0);
		}

		~RobotModelNotify()
		{
			if (m_controller)
			{
				delete m_controller;
			}

			if (m_controllerTrainer)
			{
				delete m_controllerTrainer;
			}
		}

		ndModelNotify* Clone() const
		{
			return new RobotModelNotify(*this);
		}

		void Init(ndModelArticulation* const robot)
		{
			m_cart = robot->GetRoot()->m_body->GetAsBodyDynamic();
			m_pole = robot->GetRoot()->GetLastChild()->m_body->GetAsBodyDynamic();
			m_poleJoint = (ndJointHinge*)*robot->GetRoot()->GetLastChild()->m_joint;

			m_cartMatrix = m_cart->GetMatrix();
			m_poleMatrix = m_pole->GetMatrix();
		}

		//#pragma optimize( "", off )
		ndFloat32 GetPoleAngle() const
		{
			const ndMatrix matrix(m_poleJoint->CalculateGlobalMatrix0());
			ndFloat32 sinAngle = ndClamp (matrix.m_up.m_x, ndFloat32 (-0.99f), ndFloat32(0.99f));
			ndFloat32 angle = ndAsin(sinAngle);
			return angle;
		}

		//#pragma optimize( "", off )
		bool IsTerminal() const
		{
			ndFloat32 angle = GetPoleAngle();
			bool fail = ndAbs(angle) > (D_REWARD_MIN_ANGLE * ndFloat32(2.0f));
			return fail;
		}

		//#pragma optimize( "", off )
		ndReal GetReward() const
		{
			if (IsTerminal())
			{
				return ndReal(0.0f);
			}

			ndIkSolver& solver = m_controllerTrainer->m_solver;
			ndSkeletonContainer* const skeleton = m_cart->GetSkeleton();
			ndAssert(skeleton);

			const ndVector savedForce(m_cart->GetForce());
			ndVector testForce(savedForce);
			testForce.m_x = ndFloat32(0.0f);
			m_cart->SetForce(testForce);
			m_poleJoint->SetAsSpringDamper(ndFloat32(1.0e-3f), ndFloat32(10000.0f), ndFloat32(100.0f));

			ndFloat32 targetAngle = m_poleJoint->GetAngle() * ndFloat32(0.25f);
			m_poleJoint->SetTargetAngle(targetAngle);

			solver.SolverBegin(skeleton, nullptr, 0, GetModel()->GetWorld(), m_timestep);
			solver.Solve();
			ndVector boxAccel(m_cart->GetAccel());
			solver.SolverEnd();

			m_cart->SetForce(savedForce);
			m_poleJoint->SetAsSpringDamper(ndFloat32(0.0), ndFloat32(0.0f), ndFloat32(0.0f));
			ndFloat32 accelError = m_cart->GetInvMass() * savedForce.m_x - boxAccel.m_x;
			ndFloat32 accelReward = ndReal(ndExp(-ndFloat32(0.5f) * accelError * accelError));

			ndFloat32 angle = GetPoleAngle();
			const ndVector veloc(m_cart->GetVelocity());
			ndFloat32 angularReward = ndReal(ndExp(-ndFloat32(1000.0f) * angle * angle));
			ndFloat32 speedReward = ndReal(ndExp(-ndFloat32(200.0f) * veloc.m_x * veloc.m_x));
			//ndFloat32 timeLineReward = ndFloat32(m_controllerTrainer->m_trajectory.GetCount()) / ndFloat32(ND_TRAJECTORY_STEPS);
			
			//ndFloat32 reward = (angularReward + speedReward + timeLineReward) / ndFloat32(3.0f);
			ndFloat32 reward = 0.25f * angularReward + 0.25f * speedReward + 0.5f * accelReward;
			return ndReal(reward);
		}

		//#pragma optimize( "", off )
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
			ndVector force(m_cart->GetForce());
			ndBrainFloat action = actions[0];
			//action = 0.5f;
			ndBrainFloat accel = D_PUSH_ACCEL * action;
			force.m_x = ndFloat32(accel * (m_cart->GetMassMatrix().m_w));
			m_cart->SetForce(force);
		}

		void ResetModel()
		{
			m_cart->SetMatrix(m_cartMatrix);
			m_pole->SetMatrix(m_poleMatrix);

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

		void Update(ndFloat32 timestep)
		{
			m_timestep = timestep;
			if (m_controllerTrainer)
			{
				m_controllerTrainer->Step();
			}
			else
			{
				m_controller->Step();
			}
		}

		void PostUpdate(ndFloat32)
		{
		}

		void PostTransformUpdate(ndFloat32)
		{
		}

		ndMatrix m_cartMatrix;
		ndMatrix m_poleMatrix;
		ndController* m_controller;
		ndControllerTrainer* m_controllerTrainer;
		ndBodyDynamic* m_cart;
		ndBodyDynamic* m_pole;
		ndJointHinge* m_poleJoint;
		ndFloat32 m_timestep;
	};
#endif

	class TrainingUpdata : public ndDemoEntityManager::OnPostUpdate
	{
		public:
		//TrainingUpdata(ndDemoEntityManager* const scene, const ndMatrix& matrix, const ndSharedPtr<ndDemoEntity>& modelMesh)
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
			scene->AddEntity(visualMesh);

#if 0
			ndSharedPtr<ndModel>visualModel(CreateModel(scene, matrix, modelMesh));
			world->AddModel(visualModel);
			visualModel->AddBodiesAndJointsToWorld();

			visualModel->SetNotifyCallback(new RobotModelNotify(m_master, visualModel->GetAsModelArticulation()));
			SetMaterial(visualModel->GetAsModelArticulation());

			// add a hidden battery of model to generate trajectories in parallel
			const ndInt32 countX = 100;
			//const ndInt32 countX = 0;
			for (ndInt32 i = 0; i < countX; ++i)
			{
				ndMatrix location(matrix);
				location.m_posit.m_x += 10.0f * (ndRand() - 0.5f);
				ndSharedPtr<ndModel>model(CreateModel(scene, location, modelMesh));
				world->AddModel(model);
				model->AddBodiesAndJointsToWorld();
				model->SetNotifyCallback(new RobotModelNotify(m_master, model->GetAsModelArticulation()));
				SetMaterial(model->GetAsModelArticulation());

				m_models.Append(model->GetAsModelArticulation());
			}
			scene->SetAcceleratedUpdate();
#endif
		}

		~TrainingUpdata()
		{
			if (m_outFile)
			{
				fclose(m_outFile);
			}
		}

		void HideModel(ndModelArticulation* const model, bool mode) const
		{
			ndAssert(0);
#if 0
			ndModelArticulation::ndNode* stackMem[128];
			ndInt32 stack = 1;
			stackMem[0] = model->GetRoot();
			while (stack)
			{
				stack--;
				ndModelArticulation::ndNode* const node = stackMem[stack];
				ndBody* const body = *node->m_body;
				ndDemoEntityNotify* const userData = (ndDemoEntityNotify*)body->GetNotifyCallback();
				ndDemoEntity* const ent = *userData->m_entity;
				//ent->Hide();
				mode ? ent->Hide() : ent->UnHide();

				for (ndModelArticulation::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
				{
					stackMem[stack] = child;
					stack++;
				}
			}
#endif
		}

		void OnDebug(ndDemoEntityManager* const, bool mode)
		{
			for (ndList<ndModelArticulation*>::ndNode* node = m_models.GetFirst(); node; node = node->GetNext())
			{
				HideModel(node->GetInfo(), mode);
			}
		}

		class InvisibleBodyNotify : public ndDemoEntityNotify
		{
			public:
			InvisibleBodyNotify(const ndDemoEntityNotify* const src)
				:ndDemoEntityNotify(*src)
			{
			}

			InvisibleBodyNotify(const InvisibleBodyNotify& src)
				:ndDemoEntityNotify(src)
			{
			}

			ndBodyNotify* Clone() const
			{
				return new InvisibleBodyNotify(*this);
			}

			virtual bool OnSceneAabbOverlap(const ndBody* const otherBody) const
			{
				const ndBodyKinematic* const body0 = ((ndBody*)GetBody())->GetAsBodyKinematic();
				const ndBodyKinematic* const body1 = ((ndBody*)otherBody)->GetAsBodyKinematic();
				const ndShapeInstance& instanceShape0 = body0->GetCollisionShape();
				const ndShapeInstance& instanceShape1 = body1->GetCollisionShape();
				return instanceShape0.m_shapeMaterial.m_userId != instanceShape1.m_shapeMaterial.m_userId;
			}
		};

		void SetMaterial(ndModelArticulation* const model) const
		{
			ndAssert(0);
#if 0
			ndModelArticulation::ndNode* stackMem[128];
			ndInt32 stack = 1;
			stackMem[0] = model->GetRoot();
			while (stack)
			{
				stack--;
				ndModelArticulation::ndNode* const node = stackMem[stack];
				ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
				body->GetAsBodyDynamic()->SetSleepAccel(body->GetAsBodyDynamic()->GetSleepAccel() * ndFloat32(0.1f));

				ndShapeInstance& instanceShape = body->GetCollisionShape();
				instanceShape.m_shapeMaterial.m_userId = ndDemoContactCallback::m_modelPart;

				ndDemoEntityNotify* const originalNotify = (ndDemoEntityNotify*)body->GetNotifyCallback();
				ndSharedPtr<ndDemoEntity> userData(originalNotify->m_entity);
				originalNotify->m_entity = ndSharedPtr<ndDemoEntity>();
				InvisibleBodyNotify* const notify = new InvisibleBodyNotify((InvisibleBodyNotify*)body->GetNotifyCallback());
				body->SetNotifyCallback(notify);
				notify->m_entity = userData;

				for (ndModelArticulation::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
				{
					stackMem[stack] = child;
					stack++;
				}
			}
#endif
		}

		virtual void Update(ndDemoEntityManager* const manager, ndFloat32)
		{
			//ndAssert(0);
#if 0
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
					char fileName[256];
					m_saveScore = ndFloor(rewardTrajectory) + 2.0f;

					// save partial controller in case of crash 
					ndBrain* const actor = m_master->GetPolicyNetwork();
					char name[256];
					snprintf(name, sizeof(name), "%s_actor.dnn", CONTROLLER_NAME);
					ndGetWorkingFileName(name, fileName);
					actor->SaveToFile(fileName);

					ndBrain* const critic = m_master->GetValueNetwork();
					snprintf(name, sizeof(name), "%s_critic.dnn", CONTROLLER_NAME);
					ndGetWorkingFileName(name, fileName);
					critic->SaveToFile(fileName);
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
				char fileName[1024];
				m_modelIsTrained = true;
				m_master->GetPolicyNetwork()->CopyFrom(*(*m_bestActor));
				ndGetWorkingFileName(m_master->GetName().GetStr(), fileName);
				m_master->GetPolicyNetwork()->SaveToFile(fileName);
				ndExpandTraceMessage("saving to file: %s\n", fileName);
				ndExpandTraceMessage("training complete\n");
				ndUnsigned64 timer = ndGetTimeInMicroseconds() - m_timer;
				ndExpandTraceMessage("training time: %g seconds\n", ndFloat32(ndFloat64(timer) * ndFloat32(1.0e-6f)));

				manager->Terminate();
			}
#endif
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

void ndCartpolePlayerContinue(ndDemoEntityManager* const scene)
{
	ndAssert(0);
	//ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));
	//
	//ndMatrix matrix(ndGetIdentityMatrix());
	//matrix.m_posit.m_y = 0.11f;
	//
	////ndMeshLoader loader;
	////ndSharedPtr<ndDemoEntity> modelMesh(loader.LoadEntity("cartpole.fbx", scene));
	//ndRenderMeshLoader loader(*scene->GetRenderer());
	//loader.LoadMesh(ndGetWorkingFileName("cartpole.nd"));
	//
	//ndSetRandSeed(42);
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
	//
	//matrix.m_posit.m_x -= 0.0f;
	//matrix.m_posit.m_y += 0.5f;
	//matrix.m_posit.m_z += 2.0f;
	//ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	//scene->SetCameraMatrix(rotation, matrix.m_posit);
}

void ndCartpoleContinueTraining(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));

	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("cartpole.nd"));

	ndSetRandSeed(42);
	ndSharedPtr<ndDemoEntityManager::OnPostUpdate>trainer (new TrainingUpdata(scene, matrix, loader));
	scene->RegisterPostUpdate(trainer);
	
	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
