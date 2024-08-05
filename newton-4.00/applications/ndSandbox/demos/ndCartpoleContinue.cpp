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
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

namespace ndCarpole_1
{
	//#define ND_TRAIN_AGENT
	#define CONTROLLER_NAME			"cartpoleContinueVPG.dnn"

	#define D_PUSH_ACCEL			ndBrainFloat (15.0f)
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
		m_stateSize
	};

	class RobotModelNotify : public ndModelNotify
	{
		#ifdef ND_TRAIN_AGENT
		class ndController : public ndBrainAgentContinuePolicyGradient_Trainer
		{
			public:
			ndController(const ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master)
				:ndBrainAgentContinuePolicyGradient_Trainer(master)
				,m_robot(nullptr)
			{
			}

			ndController(const ndController& src)
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

	#else

		class ndController : public ndBrainAgentContinuePolicyGradient
		{
			public:
			ndController(const ndSharedPtr<ndBrain>& brain)
				:ndBrainAgentContinuePolicyGradient(brain)
				,m_robot(nullptr)
			{
			}

			ndController(const ndController& src)
				:ndBrainAgentContinuePolicyGradient(src.m_actor)
				, m_robot(nullptr)
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
	#endif

	public:
	#ifdef ND_TRAIN_AGENT
	RobotModelNotify(ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master, ndModelArticulation* const robot)
		:ndModelNotify()
		,m_controller(master)
	{
		Init(robot);
	}

	RobotModelNotify(const RobotModelNotify& src)
		:ndModelNotify(src)
		, m_controller(src.m_controller)
	{
		//Init(robot);
	}

	#else

	RobotModelNotify(const ndSharedPtr<ndBrain>& brain, ndModelArticulation* const robot)
		:ndModelNotify()
		,m_controller(brain)
	{
		Init(robot);
	}

	RobotModelNotify(const RobotModelNotify& src)
		:ndModelNotify(src)
		,m_controller(src.m_controller)
	{
		//Init(robot);
		ndAssert(0);
	}
	#endif

	~RobotModelNotify()
	{
	}

	ndModelNotify* Clone() const
	{
		return new RobotModelNotify(*this);
	}

	void Init(ndModelArticulation* const robot)
	{
		m_controller.m_robot = this;
		m_cart = robot->GetRoot()->m_body->GetAsBodyDynamic();
		m_pole = robot->GetRoot()->GetLastChild()->m_body->GetAsBodyDynamic();
		m_poleJoint = *robot->GetRoot()->GetLastChild()->m_joint;

		m_cartMatrix = m_cart->GetMatrix();
		m_poleMatrix = m_pole->GetMatrix();
	}

	bool IsTerminal() const
	{
		// agent dies if the angle is larger than D_REWARD_MIN_ANGLE * ndFloat32 (2.0f) degrees
		bool fail = ndAbs(GetPoleAngle()) > (D_REWARD_MIN_ANGLE * ndFloat32(2.0f));
		return fail;
	}

	ndFloat32 GetPoleAngle() const
	{
		const ndMatrix& matrix = m_poleJoint->GetLocalMatrix0() * m_pole->GetMatrix();
		ndFloat32 angle = ndAsin(matrix.m_right.m_x);
		return angle;
	}

	ndReal GetReward() const
	{
		if (IsTerminal())
		{
			return ndReal(0.0f);
		}
		ndFloat32 sinAngle = GetPoleAngle();
		ndFloat32 reward = ndReal(ndExp(-ndFloat32(2000.0f) * sinAngle * sinAngle));
		return ndReal(reward);
	}

	void GetObservation(ndBrainFloat* const state)
	{
		ndVector omega(m_pole->GetOmega());
		ndFloat32 angle = GetPoleAngle();
		state[m_poleAngle] = ndReal(angle);
		state[m_poleOmega] = ndReal(omega.m_z);
	}

	void ApplyActions(ndBrainFloat* const actions)
	{
		ndVector force(m_cart->GetForce());
		ndBrainFloat action = actions[0];
		force.m_x = ndFloat32(ndBrainFloat(2.0f) * action * (m_cart->GetMassMatrix().m_w * D_PUSH_ACCEL));
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
	}

	void RandomePush()
	{
		ndVector impulsePush(ndVector::m_zero);
		ndFloat32 randValue = ndClamp(ndGaussianRandom(0.0f, 0.5f), ndFloat32 (-1.0f), ndFloat32(1.0f));
		impulsePush.m_x = 5.0f * randValue * m_cart->GetMassMatrix().m_w;
		m_cart->ApplyImpulsePair(impulsePush, ndVector::m_zero, m_cart->GetScene()->GetTimestep());
	}

	bool IsOutOfBounds() const
	{
		return ndAbs(m_cart->GetMatrix().m_posit.m_x) > ndFloat32(20.0f);
	}

	void Update(ndWorld* const, ndFloat32)
	{
		m_controller.Step();
	}

	void PostUpdate(ndWorld* const, ndFloat32)
	{
	}

	void PostTransformUpdate(ndWorld* const, ndFloat32)
	{
	}

	ndMatrix m_cartMatrix;
	ndMatrix m_poleMatrix;
	ndController m_controller;
	ndBodyDynamic* m_cart;
	ndBodyDynamic* m_pole;
	ndJointBilateralConstraint* m_poleJoint;
	};

	ndModelArticulation* CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location)
	{
		ndUrdfFile urdf;
		char fileName[256];
		ndGetWorkingFileName("cartpole.urdf", fileName);
		ndModelArticulation* const cartPole = urdf.Import(fileName);

		SetModelVisualMesh(scene, cartPole);
		cartPole->SetTransform(location);

		// make the car move along the z axis only (2d problem)
		ndWorld* const world = scene->GetWorld();
		ndBodyKinematic* const boxBody = cartPole->GetRoot()->m_body->GetAsBodyKinematic();
		ndSharedPtr<ndJointBilateralConstraint> xDirSlider(new ndJointSlider(boxBody->GetMatrix(), boxBody, world->GetSentinelBody()));
		world->AddJoint(xDirSlider);

		return cartPole;
	}

	class TrainingUpdata : public ndDemoEntityManager::OnPostUpdate
	{
		public:
		TrainingUpdata(ndDemoEntityManager* const scene, const ndMatrix& matrix)
			:OnPostUpdate()
			,m_master()
			,m_bestActor()
			,m_outFile(nullptr)
			,m_timer(ndGetTimeInMicroseconds())
			,m_maxScore(ndFloat32(-1.0e10f))
			,m_discountFactor(0.99f)
			,m_horizon(ndFloat32(1.0f) / (ndFloat32(1.0f) - m_discountFactor))
			,m_lastEpisode(-1)
			,m_stopTraining(100 * 1000000)
			,m_modelIsTrained(false)
		{
			ndWorld* const world = scene->GetWorld();

			m_outFile = fopen("cartpole-VPG.csv", "wb");
			fprintf(m_outFile, "vpg\n");

			ndBrainAgentContinuePolicyGradient_TrainerMaster::HyperParameters hyperParameters;

			hyperParameters.m_extraTrajectorySteps = 256;
			hyperParameters.m_maxTrajectorySteps = 1024 * 4;
			hyperParameters.m_numberOfActions = m_actionsSize;
			hyperParameters.m_numberOfObservations = m_stateSize;
			hyperParameters.m_discountFactor = ndReal(m_discountFactor);

			m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>(new ndBrainAgentContinuePolicyGradient_TrainerMaster(hyperParameters));
			m_bestActor = ndSharedPtr< ndBrain>(new ndBrain(*m_master->GetActor()));

			m_master->SetName(CONTROLLER_NAME);

			ndModelArticulation* const visualModel = CreateModel(scene, matrix);
			visualModel->AddToWorld(world);

			#ifdef ND_TRAIN_AGENT
				visualModel->SetNotifyCallback(new RobotModelNotify(m_master, visualModel));
			#endif
			SetMaterial(visualModel);

			// add a hidden battery of model to generate trajectories in parallel
			const ndInt32 countX = 32;
			for (ndInt32 i = 0; i < countX; ++i)
			{
				ndMatrix location(matrix);
				location.m_posit.m_x += 3.0f * (ndRand() - 0.5f);
				#if 0
				//ndSharedPtr<ndBrainAgent> agent(new ndRobot::ndControllerTrainer(m_master));
				//ndSharedPtr<ndModel> model(CreateModel(scene, location, agent));
				ndModelArticulation* const model = (ndModelArticulation*)visualModel->Clone();

				//world->AddModel(model);
				////HideModel(model);
				//SetMaterial(model);
				#else
				ndModelArticulation* const model = CreateModel(scene, location);
				model->AddToWorld(world);
				#ifdef ND_TRAIN_AGENT
					model->SetNotifyCallback(new RobotModelNotify(m_master, model));
				#endif
				SetMaterial(model);
				#endif

				scene->SetAcceleratedUpdate();
			}
		}

		~TrainingUpdata()
		{
			if (m_outFile)
			{
				fclose(m_outFile);
			}
		}

		void HideModel(ndModelArticulation* const model) const
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
				ndDemoEntity* const ent = userData->m_entity;
				ent->Hide();

				for (ndModelArticulation::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
				{
					stackMem[stack] = child;
					stack++;
				}
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
				void* const useData = originalNotify->m_entity;
				originalNotify->m_entity = nullptr;
				InvisibleBodyNotify* const notify = new InvisibleBodyNotify((InvisibleBodyNotify*)body->GetNotifyCallback());
				body->SetNotifyCallback(notify);
				notify->m_entity = (ndDemoEntity*)useData;

				for (ndModelArticulation::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
				{
					stackMem[stack] = child;
					stack++;
				}
			}
		}

		virtual void Update(ndDemoEntityManager* const manager, ndFloat32)
		{
			ndInt32 stopTraining = m_master->GetFramesCount();
			if (stopTraining <= m_stopTraining)
			{
				ndInt32 episodeCount = m_master->GetEposideCount();
				m_master->OptimizeStep();

				episodeCount -= m_master->GetEposideCount();
				ndFloat32 rewardTrajectory = m_master->GetAverageFrames() * m_master->GetAverageScore();
				if (rewardTrajectory >= ndFloat32(m_maxScore))
				{
					if (m_lastEpisode != m_master->GetEposideCount())
					{
						m_maxScore = rewardTrajectory;
						m_bestActor->CopyFrom(*m_master->GetActor());
						ndExpandTraceMessage("best actor episode: %d\treward %f\ttrajectoryFrames: %f\n", m_master->GetEposideCount(), 100.0f * m_master->GetAverageScore() / m_horizon, m_master->GetAverageFrames());
						m_lastEpisode = m_master->GetEposideCount();
					}
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

			if ((stopTraining >= m_stopTraining) || (100.0f * m_master->GetAverageScore() / m_horizon > 96.0f))
			{
				char fileName[1024];
				m_modelIsTrained = true;
				m_master->GetActor()->CopyFrom(*(*m_bestActor));
				ndGetWorkingFileName(m_master->GetName().GetStr(), fileName);
				m_master->GetActor()->SaveToFile(fileName);
				ndExpandTraceMessage("saving to file: %s\n", fileName);
				ndExpandTraceMessage("training complete\n");
				ndUnsigned64 timer = ndGetTimeInMicroseconds() - m_timer;
				ndExpandTraceMessage("training time: %g seconds\n", ndFloat32(ndFloat64(timer) * ndFloat32(1.0e-6f)));

				//ndGetWorkingFileName(CRITIC_NAME, fileName);
				//m_master->GetCritic()->SaveToFile(fileName);
				manager->Terminate();
			}
		}

		ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster> m_master;
		ndSharedPtr<ndBrain> m_bestActor;
		FILE* m_outFile;
		ndUnsigned64 m_timer;
		ndFloat32 m_maxScore;
		ndFloat32 m_discountFactor;
		ndFloat32 m_horizon;
		ndInt32 m_lastEpisode;
		ndInt32 m_stopTraining;
		bool m_modelIsTrained;
	};
}

using namespace ndCarpole_1;


void ndCartpoleContinue(ndDemoEntityManager* const scene)
{
	BuildFlatPlane(scene, true);

	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit.m_y = 0.11f;

#ifdef ND_TRAIN_AGENT
	ndSetRandSeed(42);
	TrainingUpdata* const trainer = new TrainingUpdata(scene, matrix);
	scene->RegisterPostUpdate(trainer);
#else
	ndWorld* const world = scene->GetWorld();
	ndModelArticulation* const model = CreateModel(scene, matrix);
	model->AddToWorld(world);

	// add the deep learning controller
	char fileName[256];
	ndGetWorkingFileName(CONTROLLER_NAME, fileName);
	ndSharedPtr<ndBrain> brain(ndBrainLoad::Load(fileName));
	model->SetNotifyCallback(new RobotModelNotify(brain, model));
#endif

	matrix.m_posit.m_y = 0.5f;
	matrix.m_posit.m_z = 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
