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
#include "ndMeshLoader.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"


namespace ndContinueCarpole
{
	#define ND_TRAIN_AGENT
	#define CONTROLLER_NAME			"cartpoleContinue"

	//#define CONTROLLER_RESUME_TRAINING

	#define D_PUSH_ACCEL			ndBrainFloat (-3.0f * DEMO_GRAVITY)
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
		m_stateSize
	};

	ndModelArticulation* CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, const ndSharedPtr<ndDemoEntity>& modelMesh)
	{
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

		ndFloat32 cartMass = 20.0f;
		ndFloat32 poleMass = 5.0f;

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

		class ndControllerTrainer : public ndBrainAgentContinuePolicyGradient_Trainer
		{
			public:
			ndControllerTrainer(const ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master)
				:ndBrainAgentContinuePolicyGradient_Trainer(master)
				,m_robot(nullptr)
			{
			}

			ndControllerTrainer(const ndControllerTrainer& src)
				:ndBrainAgentContinuePolicyGradient_Trainer(src.m_master)
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

		//RobotModelNotify(const RobotModelNotify& src)
		//	:ndModelNotify(src)
		//	,m_controller(src.m_controller)
		//{
		//	//Init(robot);
		//	ndAssert(0);
		//}

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

		//ndFloat32 GetPoleAngle() const
		//{
		//	//const ndMatrix& matrix = m_poleJoint->GetLocalMatrix0() * m_pole->GetMatrix();
		//	//ndFloat32 angle = ndAsin(matrix.m_right.m_x);
		//	ndFloat32 angle = GetPoleAngle();
		//	return angle;
		//}

		bool IsTerminal() const
		{
			// agent dies if the angle is larger than D_REWARD_MIN_ANGLE * ndFloat32 (2.0f) degrees
			bool fail = ndAbs(m_poleJoint->GetAngle()) > (D_REWARD_MIN_ANGLE * ndFloat32(2.0f));
			return fail;
		}

		//#pragma optimize( "", off )
		ndReal GetReward() const
		{
			if (IsTerminal())
			{
				return ndReal (-1.0f);
			}
			const ndVector veloc(m_cart->GetVelocity());
			ndFloat32 angle = m_poleJoint->GetAngle();
			ndFloat32 angularReward = ndReal(ndExp(-ndFloat32(1000.0f) * angle * angle));
			ndFloat32 linearReward = ndReal(ndExp(-ndFloat32(200.0f) * veloc.m_x * veloc.m_x));

			ndReal reward = 0.5f * angularReward + 0.5f * linearReward;
			return reward;
		}

		void GetObservation(ndBrainFloat* const state)
		{
			//ndFloat32 poleAngle = m_poleJoint->GetAngle();
			//ndFloat32 omega = m_poleJoint->GetOmega();
			const ndVector cartVeloc(m_cart->GetVelocity());

			ndFloat32 cartSpeed = cartVeloc.m_x / ndFloat32(10.0f);
			ndFloat32 poleOmega = m_poleJoint->GetOmega() / ndFloat32(2.0f);
			ndFloat32 poleAngle = m_poleJoint->GetAngle() / (D_REWARD_MIN_ANGLE * ndFloat32(2.0f));

			//ndTrace(("%f %f\n", cartVeloc.m_x, speed));
			//ndTrace(("%f %f\n", m_poleJoint->GetAngle(), poleAngle));
			//ndTrace(("%f %f\n", m_poleJoint->GetOmega(), poleOmega));

			state[m_cartSpeed] = cartSpeed;
			state[m_poleAngle] = poleAngle;
			state[m_poleOmega] = poleOmega;
		}

		void ApplyActions(ndBrainFloat* const actions)
		{
			ndVector force(m_cart->GetForce());
			ndBrainFloat action = actions[0];
			force.m_x = ndFloat32(D_PUSH_ACCEL * action * (m_cart->GetMassMatrix().m_w));
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
			//m_poleJoint->ClearMemory();
		}

		bool IsOutOfBounds() const
		{
			return ndAbs(m_cart->GetMatrix().m_posit.m_x) > ndFloat32(20.0f);
		}

		void Update(ndFloat32)
		{
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
	};

	class TrainingUpdata : public ndDemoEntityManager::OnPostUpdate
	{
		public:
		TrainingUpdata(ndDemoEntityManager* const scene, const ndMatrix& matrix, const ndSharedPtr<ndDemoEntity>& modelMesh)
			:OnPostUpdate()
			,m_master()
			,m_bestActor()
			,m_outFile(nullptr)
			,m_timer(ndGetTimeInMicroseconds())
			,m_maxScore(ndFloat32(-1.0e10f))
			,m_saveScore(m_maxScore)
			,m_discountFactor(0.99f)
			,m_lastEpisode(0xfffffff)
			,m_stopTraining(500 * 1000000)
			,m_modelIsTrained(false)
		{
			char name[256];
			snprintf(name, sizeof(name), "%s-vpg.csv", CONTROLLER_NAME);
			m_outFile = fopen(name, "wb");
			fprintf(m_outFile, "vpg\n");

			m_horizon = ndFloat32(1.0f) / (ndFloat32(1.0f) - m_discountFactor);
			ndBrainAgentContinuePolicyGradient_TrainerMaster::HyperParameters hyperParameters;

			hyperParameters.m_extraTrajectorySteps = 256;
			hyperParameters.m_maxTrajectorySteps = 1024 * 4;

	//hyperParameters.m_threadsCount = 1;
	//hyperParameters.m_maxTrajectorySteps = 256;
	//hyperParameters.m_extraTrajectorySteps = 60;
	//hyperParameters.m_bashTrajectoryCount = 50;

			hyperParameters.m_numberOfActions = m_actionsSize;
			hyperParameters.m_numberOfObservations = m_stateSize;
			hyperParameters.m_discountFactor = ndReal(m_discountFactor);

			m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>(new ndBrainAgentContinuePolicyGradient_TrainerMaster(hyperParameters));
			//m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>(new ndBrainAgentContinueProximaPolicyGradient_TrainerMaster(hyperParameters));
			m_bestActor = ndSharedPtr< ndBrain>(new ndBrain(*m_master->GetPolicyNetwork()));

			snprintf(name, sizeof(name), "%s.dnn", CONTROLLER_NAME);
			m_master->SetName(name);

			#ifdef CONTROLLER_RESUME_TRAINING
				char fileName[256];
				snprintf(name, sizeof(name), "%s_critic.dnn", CONTROLLER_NAME);
				ndGetWorkingFileName(name, fileName);
				ndSharedPtr<ndBrain> valueNetwork(ndBrainLoad::Load(fileName));
				m_master->GetValueNetwork()->CopyFrom(**valueNetwork);

				snprintf(name, sizeof(name), "%s_actor.dnn", CONTROLLER_NAME);
				ndGetWorkingFileName(name, fileName);
				ndSharedPtr<ndBrain> policyNetwork(ndBrainLoad::Load(fileName));
				m_master->GetPolicyNetwork()->CopyFrom(**policyNetwork);
			#endif

			ndWorld* const world = scene->GetWorld();
			ndSharedPtr<ndModel>visualModel (CreateModel(scene, matrix, modelMesh));
			world->AddModel(visualModel);
			visualModel->AddBodiesAndJointsToWorld();

			visualModel->SetNotifyCallback(new RobotModelNotify(m_master, visualModel->GetAsModelArticulation()));
			SetMaterial(visualModel->GetAsModelArticulation());

			// add a hidden battery of model to generate trajectories in parallel
			const ndInt32 countX = 100;
			for (ndInt32 i = 0; i < countX; ++i)
			{
				ndMatrix location(matrix);
				location.m_posit.m_x += 10.0f * (ndRand() - 0.5f);
				ndSharedPtr<ndModel>model (CreateModel(scene, location, modelMesh));
				world->AddModel(model);
				model->AddBodiesAndJointsToWorld();
				model->SetNotifyCallback(new RobotModelNotify(m_master, model->GetAsModelArticulation()));
				SetMaterial(model->GetAsModelArticulation());

				m_models.Append(model->GetAsModelArticulation());
			}
			scene->SetAcceleratedUpdate();
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
		}

		ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster> m_master;
		ndSharedPtr<ndBrain> m_bestActor;
		ndList<ndModelArticulation*> m_models;
		FILE* m_outFile;
		ndUnsigned64 m_timer;
		ndFloat32 m_maxScore;
		ndFloat32 m_saveScore;
		ndFloat32 m_discountFactor;
		ndFloat32 m_horizon;
		ndUnsigned32 m_lastEpisode;
		ndUnsigned32 m_stopTraining;
		bool m_modelIsTrained;
	};
}

using namespace ndContinueCarpole;


void ndCartpoleContinue(ndDemoEntityManager* const scene)
{
	BuildFlatPlane(scene, true);

	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit.m_y = 0.11f;

	ndMeshLoader loader;
	ndSharedPtr<ndDemoEntity> modelMesh(loader.LoadEntity("cartpole.fbx", scene));

#ifdef ND_TRAIN_AGENT
	ndSetRandSeed(42);
	TrainingUpdata* const trainer = new TrainingUpdata(scene, matrix, modelMesh);
	scene->RegisterPostUpdate(trainer);
#else
	ndWorld* const world = scene->GetWorld();
	ndModelArticulation* const model = CreateModel(scene, matrix, modelMesh);
	world->AddModel(model);
	model->AddBodiesAndJointsToWorld();

	// add the deep learning controller
	char name[256];
	char fileName[256];
	snprintf(name, sizeof(name), "%s.dnn", CONTROLLER_NAME);
	ndGetWorkingFileName(name, fileName);

	ndSharedPtr<ndBrain> policy(ndBrainLoad::Load(fileName));
	model->SetNotifyCallback(new RobotModelNotify(policy, model));
#endif

	matrix.m_posit.m_y = 0.5f;
	matrix.m_posit.m_z = 5.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
