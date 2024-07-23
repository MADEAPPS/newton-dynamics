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
	#define CONTROLLER_NAME "cartpoleContinueVPG.dnn"

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

	class ndRobot: public ndModelArticulation
	{
		public:
		class ndController : public ndBrainAgentContinuePolicyGradient
		{
			public:
			ndController(ndSharedPtr<ndBrain>& actor)
				:ndBrainAgentContinuePolicyGradient(actor)
				,m_model(nullptr)
			{
			}

			void GetObservation(ndBrainFloat* const observation)
			{
				m_model->GetObservation(observation);
			}

			virtual void ApplyActions(ndBrainFloat* const actions)
			{
				m_model->ApplyActions(actions);
			}

			ndRobot* m_model;
		};

		class ndControllerTrainer : public ndBrainAgentContinuePolicyGradient_Trainer
		{
			public:
			ndControllerTrainer(ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master)
				:ndBrainAgentContinuePolicyGradient_Trainer(master)
				,m_model(nullptr)
			{
			}

			~ndControllerTrainer()
			{
			}

			ndBrainFloat CalculateReward()
			{
				return m_model->GetReward();
			}

			virtual void ApplyActions(ndBrainFloat* const actions)
			{
				//if (GetEpisodeFrames() >= 7000)
				//{
				//	ndAssert(0);
				//	for (ndInt32 i = 0; i < m_actionsSize; ++i)
				//	{
				//		ndReal gaussianNoise = ndReal(ndGaussianRandom(ndFloat32(actions[i]), ndFloat32(2.0f)));
				//		ndReal clippiedNoisyAction = ndClamp(gaussianNoise, ndReal(-1.0f), ndReal(1.0f));
				//		actions[i] = clippiedNoisyAction;
				//	}
				//}
				//else if (GetEpisodeFrames() >= 6000)
				//{
				//	ndAssert(0);
				//	for (ndInt32 i = 0; i < m_actionsSize; ++i)
				//	{
				//		ndReal gaussianNoise = ndReal(ndGaussianRandom(ndFloat32(actions[i]), ndFloat32(1.0f)));
				//		ndReal clippiedNoisyAction = ndClamp(gaussianNoise, ndReal(-1.0f), ndReal(1.0f));
				//		actions[i] = clippiedNoisyAction;
				//	}
				//}
				m_model->ApplyActions(actions);
			}

			void GetObservation(ndBrainFloat* const observation)
			{
				m_model->GetObservation(observation);
			}

			bool IsTerminal() const
			{
				return m_model->IsTerminal();
			}

			void ResetModel()
			{
				m_model->ResetModel();
			}

			ndRobot* m_model;
		};

		ndRobot(const ndSharedPtr<ndBrainAgent>& agent)
			:ndModelArticulation()
			,m_cartMatrix(ndGetIdentityMatrix())
			,m_poleMatrix(ndGetIdentityMatrix())
			,m_cart(nullptr)
			,m_pole(nullptr)
			,m_agent(agent)
		{
		}

		bool IsTerminal() const
		{
			const ndMatrix& matrix = m_pole->GetMatrix();
			// agent dies if the angle is larger than D_REWARD_MIN_ANGLE * ndFloat32 (2.0f) degrees
			bool fail = ndAbs(ndAsin (matrix.m_front.m_x)) > (D_REWARD_MIN_ANGLE * ndFloat32 (2.0f));
			return fail;
		}

		ndReal GetReward() const
		{
			if (IsTerminal())
			{
				return ndReal(0.0f);
			}
			const ndMatrix& matrix = m_pole->GetMatrix();
			ndFloat32 sinAngle = matrix.m_front.m_x;
			//ndFloat32 reward = ndReal(ndExp(-ndFloat32(10000.0f) * sinAngle * sinAngle));
			ndFloat32 reward = ndReal(ndExp(-ndFloat32(1000.0f) * sinAngle * sinAngle));
			return ndReal(reward);
		}

		void ApplyActions(ndBrainFloat* const actions)
		{
			ndVector force(m_cart->GetForce());
			ndBrainFloat action = actions[0];
			force.m_x = ndFloat32 (ndBrainFloat(2.0f) * action * (m_cart->GetMassMatrix().m_w * D_PUSH_ACCEL));
			m_cart->SetForce(force);
		}

		void GetObservation(ndBrainFloat* const observation)
		{
			ndVector omega(m_pole->GetOmega());
			const ndMatrix& matrix = m_pole->GetMatrix();
			ndFloat32 angle = ndAsin (matrix.m_front.m_x);
			observation[m_poleAngle] = ndReal(angle);
			observation[m_poleOmega] = ndReal(omega.m_z);
		}

		void TelePort() const
		{
			ndVector veloc(m_cart->GetVelocity());
			ndVector posit(m_cart->GetMatrix().m_posit & ndVector::m_triplexMask);
			posit.m_y = 0.0f;
			posit.m_z = 0.0f;
			veloc.m_y = 0.0f;
			veloc.m_z = 0.0f;

			ndMatrix cartMatrix(m_cart->GetMatrix());
			ndVector cartVeloc(m_cart->GetVelocity());
			cartVeloc -= veloc;
			cartMatrix.m_posit -= posit;
			m_cart->SetMatrix(cartMatrix);
			m_cart->SetVelocity(cartVeloc);

			ndMatrix poleMatrix(m_pole->GetMatrix());
			ndVector poleVeloc(m_pole->GetVelocity());
			poleVeloc -= veloc;
			poleMatrix.m_posit -= posit;
			m_pole->SetMatrix(poleMatrix);
			m_pole->SetVelocity(poleVeloc);
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

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			ndModelArticulation::Update(world, timestep);
			m_agent->Step();
		}

		void PostUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			ndModelArticulation::PostUpdate(world, timestep);
		}

		ndMatrix m_cartMatrix;
		ndMatrix m_poleMatrix;
		ndBodyDynamic* m_cart;
		ndBodyDynamic* m_pole;
		ndSharedPtr<ndBrainAgent> m_agent;
	};

	void BuildModel(ndRobot* const model, ndDemoEntityManager* const scene, const ndMatrix& location)
	{
		ndFloat32 xSize = 0.25f;
		ndFloat32 ySize = 0.125f;
		ndFloat32 zSize = 0.15f;
		ndFloat32 cartMass = 5.0f;
		ndFloat32 poleMass = 10.0f;
		ndFloat32 poleLength = 0.4f;
		ndFloat32 poleRadio = 0.05f;
		ndPhysicsWorld* const world = scene->GetWorld();
		
		// make cart
		ndSharedPtr<ndBody> cartBody(world->GetBody(AddBox(scene, location, cartMass, xSize, ySize, zSize, "wood_0.png")));
		ndModelArticulation::ndNode* const modelRoot = model->AddRootBody(cartBody);
		//ndMatrix matrix(cartBody->GetMatrix());
		ndMatrix matrix(location);
		matrix.m_posit.m_y += ySize / 2.0f + 0.05f;
		cartBody->SetMatrix(matrix);
		cartBody->GetAsBodyDynamic()->SetSleepAccel(cartBody->GetAsBodyDynamic()->GetSleepAccel() * ndFloat32(0.1f));
		
		matrix.m_posit.m_y += ySize / 2.0f;

		// make pole leg
		ndSharedPtr<ndBody> poleBody(world->GetBody(AddCapsule(scene, ndGetIdentityMatrix(), poleMass, poleRadio, poleRadio, poleLength, "smilli.png")));
		ndMatrix poleLocation(ndRollMatrix(90.0f * ndDegreeToRad) * matrix);
		poleLocation.m_posit.m_y += poleLength * 0.5f;
		poleBody->SetMatrix(poleLocation);
		poleBody->GetAsBodyDynamic()->SetSleepAccel(poleBody->GetAsBodyDynamic()->GetSleepAccel() * ndFloat32(0.1f));
		
		// link cart and body with a hinge
		ndMatrix polePivot(ndYawMatrix(90.0f * ndDegreeToRad) * poleLocation);
		polePivot.m_posit.m_y -= poleLength * 0.5f;
		ndSharedPtr<ndJointBilateralConstraint> poleJoint(new ndJointHinge(polePivot, poleBody->GetAsBodyKinematic(), modelRoot->m_body->GetAsBodyKinematic()));

		// make the car move alone the z axis only (2d problem)
		ndSharedPtr<ndJointBilateralConstraint> xDirSlider(new ndJointSlider(cartBody->GetMatrix(), cartBody->GetAsBodyDynamic(), world->GetSentinelBody()));
		world->AddJoint(xDirSlider);

		// add path to the model
		world->AddJoint(poleJoint);
		model->AddLimb(modelRoot, poleBody, poleJoint);

		// save some useful data
		model->m_cart = cartBody->GetAsBodyDynamic();
		model->m_pole = poleBody->GetAsBodyDynamic();
		model->m_cartMatrix = cartBody->GetMatrix();
		model->m_poleMatrix = poleBody->GetMatrix();
	}

#ifdef ND_TRAIN_AGENT
	ndRobot* CreateTrainModel(ndDemoEntityManager* const scene, const ndMatrix& location, ndSharedPtr<ndBrainAgent>& agent)
	{
		ndRobot* const model = new ndRobot(agent);
		ndRobot::ndControllerTrainer* const trainer = (ndRobot::ndControllerTrainer*)*agent;
		trainer->m_model = model;
		trainer->SetName(CONTROLLER_NAME);

		BuildModel(model, scene, location);

		scene->SetAcceleratedUpdate();
		return model;
	}

	ndModelArticulation* CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, ndSharedPtr<ndBrainAgent>& agent)
	{
		ndRobot* const model = CreateTrainModel(scene, location, agent);
		return model;
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
			,m_horizon(ndFloat32(0.99f) / (ndFloat32(1.0f) - m_discountFactor))
			,m_lastEpisode(-1)
			,m_stopTraining(50 * 1000000)
			,m_modelIsTrained(false)
		{
			ndWorld* const world = scene->GetWorld();

			m_outFile = fopen("cartpole-VPG.csv", "wb");
			fprintf(m_outFile, "vpg\n");

			ndBrainAgentContinuePolicyGradient_TrainerMaster::HyperParameters hyperParameters;

			hyperParameters.m_extraTrajectorySteps = 256;
			hyperParameters.m_maxTrajectorySteps = 1024 * 8;
			hyperParameters.m_numberOfActions = m_actionsSize;
			hyperParameters.m_numberOfObservations = m_stateSize;
			hyperParameters.m_discountFactor = ndReal(m_discountFactor);

			m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>(new ndBrainAgentContinuePolicyGradient_TrainerMaster(hyperParameters));
			m_bestActor = ndSharedPtr< ndBrain>(new ndBrain(*m_master->GetActor()));

			m_master->SetName(CONTROLLER_NAME);

			ndSharedPtr<ndBrainAgent> visualAgent(new ndRobot::ndControllerTrainer(m_master));
			ndSharedPtr<ndModel> visualModel(CreateModel(scene, matrix, visualAgent));
			world->AddModel(visualModel);
			SetMaterial(visualModel);

			// add a hidden battery of model to generate trajectories in parallel
			const ndInt32 countX = 32;
			for (ndInt32 i = 0; i < countX; ++i)
			{
				ndMatrix location(matrix);
				location.m_posit.m_x += 3.0f * (ndRand() - 0.5f);
				ndSharedPtr<ndBrainAgent> agent(new ndRobot::ndControllerTrainer(m_master));
				ndSharedPtr<ndModel> model(CreateModel(scene, location, agent));
				world->AddModel(model);
				//HideModel(model);
				SetMaterial(model);
			}
		}

		~TrainingUpdata()
		{
			if (m_outFile)
			{
				fclose(m_outFile);
			}
		}

		void HideModel(ndSharedPtr<ndModel>& model) const
		{
			ndRobot* const robot = (ndRobot*)*model;

			ndModelArticulation::ndNode* stackMem[128];
			ndInt32 stack = 1;
			stackMem[0] = robot->GetRoot();
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

			virtual bool OnSceneAabbOverlap(const ndBody* const otherBody) const
			{
				const ndBodyKinematic* const body0 = ((ndBody*)GetBody())->GetAsBodyKinematic();
				const ndBodyKinematic* const body1 = ((ndBody*)otherBody)->GetAsBodyKinematic();
				const ndShapeInstance& instanceShape0 = body0->GetCollisionShape();
				const ndShapeInstance& instanceShape1 = body1->GetCollisionShape();
				return instanceShape0.m_shapeMaterial.m_userId != instanceShape1.m_shapeMaterial.m_userId;
			}
		};

		void SetMaterial(ndSharedPtr<ndModel>& model) const
		{
			ndRobot* const robot = (ndRobot*)*model;

			ndModelArticulation::ndNode* stackMem[128];
			ndInt32 stack = 1;
			stackMem[0] = robot->GetRoot();
			while (stack)
			{
				stack--;
				ndModelArticulation::ndNode* const node = stackMem[stack];
				ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();

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
						ndExpandTraceMessage("best actor episode: %d\treward %f\ttrajectoryFrames: %f\n", m_master->GetEposideCount(), m_master->GetAverageScore(), m_master->GetAverageFrames());
						m_lastEpisode = m_master->GetEposideCount();
					}
				}

				if (episodeCount && !m_master->IsSampling())
				{
					ndExpandTraceMessage("steps: %d\treward: %g\t  trajectoryFrames: %g\n", m_master->GetFramesCount(), m_master->GetAverageScore(), m_master->GetAverageFrames());
					if (m_outFile)
					{
						fprintf(m_outFile, "%g\n", m_master->GetAverageScore());
						fflush(m_outFile);
					}
				}
			}

			//if (stopTraining >= m_stopTraining)
			if ((stopTraining >= m_stopTraining) || (m_master->GetAverageScore() >= m_horizon))
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

#else
	ndModelArticulation* CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location)
	{
		char fileName[1024];
		ndGetWorkingFileName(CONTROLLER_NAME, fileName);

		ndSharedPtr<ndBrain> actor(ndBrainLoad::Load(fileName));
		ndSharedPtr<ndBrainAgent> agent(new ndRobot::ndController(actor));

		ndRobot* const model = new ndRobot(agent);
		((ndRobot::ndController*)*agent)->m_model = model;

		BuildModel(model, scene, location);
		return model;
	}
#endif
}

using namespace ndCarpole_1;

void ndCartpoleContinue(ndDemoEntityManager* const scene)
{
	BuildFlatPlane(scene, true);
	
	ndSetRandSeed(42);
	ndMatrix matrix(ndYawMatrix(-0.0f * ndDegreeToRad));

#ifdef ND_TRAIN_AGENT
	TrainingUpdata* const trainer = new TrainingUpdata(scene, matrix);
	scene->RegisterPostUpdate(trainer);
#else
	ndWorld* const world = scene->GetWorld();
	ndSharedPtr<ndModel> model(CreateModel(scene, matrix));
	world->AddModel(model);
#endif

	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
