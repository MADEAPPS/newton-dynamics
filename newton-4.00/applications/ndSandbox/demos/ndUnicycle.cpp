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


namespace ndUnicycle
{
	#define ND_TRAIN_AGENT
	#define CONTROLLER_NAME "unicycleVPG.dnn"

	#define ND_MAX_WHEEL_TORQUE		(ndFloat32 (10.0f))
	#define ND_MAX_LEG_ANGLE_STEP	(ndFloat32 (4.0f) * ndDegreeToRad)
	#define ND_MAX_LEG_JOINT_ANGLE	(ndFloat32 (30.0f) * ndDegreeToRad)

	enum ndActionSpace
	{
		m_softLegControl,
		m_softWheelControl,
		m_actionsSize
	};

	enum ndStateSpace
	{
		m_topBoxAngle,
		m_topBoxOmega,
		m_jointAngle,
		m_wheelOmega,
		m_isOnAir,
		m_speed,
		m_stateSize
	};

	class ndRobot : public ndModelArticulation
	{
		public:
		class ndBasePose
		{
			public:
			ndBasePose()
				:m_body(nullptr)
			{
			}

			ndBasePose(ndBodyDynamic* const body)
				:m_veloc(body->GetVelocity())
				,m_omega(body->GetOmega())
				,m_posit(body->GetPosition())
				,m_rotation(body->GetRotation())
				,m_body(body)
			{
			}

			void SetPose() const
			{
				m_body->SetMatrix(ndCalculateMatrix(m_rotation, m_posit));
				m_body->SetOmega(m_omega);
				m_body->SetVelocity(m_veloc);
			}

			ndVector m_veloc;
			ndVector m_omega;
			ndVector m_posit;
			ndQuaternion m_rotation;
			ndBodyDynamic* m_body;
		};

		// implement controller player
		class ndController : public ndBrainAgentContinuePolicyGradient<m_stateSize, m_actionsSize>
		{
			public:
			ndController(ndSharedPtr<ndBrain>& actor)
				:ndBrainAgentContinuePolicyGradient<m_stateSize, m_actionsSize>(actor)
				,m_model(nullptr)
			{
			}

			void SetModel(ndRobot* const model)
			{
				m_model = model;
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

		class ndControllerTrainer : public ndBrainAgentContinuePolicyGradient_Trainer<m_stateSize, m_actionsSize>
		{
			public:
			ndControllerTrainer(ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster<m_stateSize, m_actionsSize>>& master)
				:ndBrainAgentContinuePolicyGradient_Trainer<m_stateSize, m_actionsSize>(master)
				,m_model(nullptr)
			{
			}
		
			~ndControllerTrainer()
			{
			}
		
			void SetModel(ndRobot* const model)
			{
				m_model = model;
			}
		
			bool IsTerminal() const
			{
				#define D_REWARD_MIN_ANGLE	(ndFloat32 (20.0f) * ndDegreeToRad)
				const ndMatrix& matrix = m_model->GetRoot()->m_body->GetMatrix();
				ndFloat32 sinAngle = ndClamp(matrix.m_up.m_x, ndFloat32(-0.9f), ndFloat32(0.9f));
				bool fail = ndAbs(ndAsin(sinAngle)) > (D_REWARD_MIN_ANGLE * ndFloat32(2.0f));
				return fail;
			}

			ndBrainFloat CalculateReward()
			{
				ndFloat32 legReward = ndReal(ndExp(-ndFloat32(10000.0f) * m_model->m_legJoint->GetAngle() * m_model->m_legJoint->GetAngle()));
				if (m_model->HasSupportContact())
				{
					ndBodyKinematic* const boxBody = m_model->GetRoot()->m_body->GetAsBodyKinematic();

					const ndVector boxVeloc(boxBody->GetVelocity());
					const ndMatrix& matrix = boxBody->GetMatrix();
					const ndFloat32 sinAngle = matrix.m_up.m_x;
					const ndFloat32 boxReward = ndReal(ndExp(-ndFloat32(1000.0f) * sinAngle * sinAngle));
					const ndFloat32 speedReward = ndReal(ndExp(-ndFloat32(0.5f) * boxVeloc.m_x * boxVeloc.m_x));
					const ndFloat32 reward = (speedReward + boxReward + legReward) / 3.0f;
					return ndReal(reward);
				}
				else
				{
					const ndVector wheelOmega(m_model->m_wheel->GetOmega());
					const ndFloat32 wheelReward = ndReal(ndExp(-ndFloat32(10000.0f) * wheelOmega.m_z * wheelOmega.m_z));
					const ndFloat32 reward = (wheelReward + legReward) / 2.0f;
					return ndReal(reward);
				}
			}
		
			virtual void ApplyActions(ndBrainFloat* const actions)
			{
				m_model->ApplyActions(actions);
		
				const ndFloat32 probability = 1.0f / 1000.0f;
				ndFloat32 applySideImpule = ndRand();
				if (applySideImpule < probability)
				{
					if (m_model->HasSupportContact())
					{
						ndBodyDynamic* const boxBody = m_model->GetRoot()->m_body->GetAsBodyDynamic();
				
						const ndVector front(boxBody->GetMatrix().m_front);
						//const ndVector side(boxBody->GetMatrix().m_front);
						ndFloat32 speed = 1.2f * (ndRand() - 0.5f);
						//ndFloat32 omega = 0.1f * (ndRand() - 0.5f);
						ndVector upVector(front.Scale(speed));
						//ndVector sizeVector(side.Scale(omega));
						ndVector impulse(upVector.Scale(boxBody->GetMassMatrix().m_w * speed));
						boxBody->ApplyImpulsePair(impulse, ndVector::m_zero, m_model->m_timestep);
					}
				}
			}
		
			void GetObservation(ndBrainFloat* const observation)
			{
				m_model->GetObservation(observation);
			}
		
			void ResetModel()
			{
				m_model->ResetModel();
			}
		
			ndRobot* m_model;
		};

		ndRobot(ndSharedPtr<ndBrainAgent> agent)
			:ndModelArticulation()
			,m_agent(agent)
			,m_legJoint(nullptr)
			,m_wheelJoint(nullptr)
		{
		}

		bool HasSupportContact() const
		{
			const ndBodyKinematic::ndContactMap& contacts = m_wheel->GetContactMap();
			ndBodyKinematic::ndContactMap::Iterator it(contacts);
			for (it.Begin(); it; it++)
			{
				const ndContact* const contact = *it;
				if (contact->IsActive())
				{
					return true;
				}
			}
			return false;
		}

		void GetObservation(ndBrainFloat* const state)
		{
			ndBodyKinematic* const body = GetRoot()->m_body->GetAsBodyKinematic();
			const ndVector omega (body->GetOmega());
			const ndVector veloc(body->GetVelocity());
			const ndVector wheelOmega(m_wheel->GetOmega());
			const ndMatrix& matrix = body->GetMatrix();
			const ndFloat32 sinAngle = ndClamp(matrix.m_up.m_x, ndFloat32(-0.9f), ndFloat32(0.9f));
			const ndFloat32 angle = ndAsin(sinAngle);

			state[m_jointAngle] = ndBrainFloat(veloc.m_x);
			state[m_topBoxAngle] = ndBrainFloat(angle);
			state[m_topBoxOmega] = ndBrainFloat(omega.m_z);
			state[m_wheelOmega] = ndBrainFloat(wheelOmega.m_z);
			state[m_isOnAir] = HasSupportContact() ? ndBrainFloat(0.0f) : ndBrainFloat(1.0f);
			state[m_jointAngle] = ndBrainFloat(m_legJoint->GetAngle() / ND_MAX_LEG_JOINT_ANGLE);
		}

		void ApplyActions(ndBrainFloat* const actions)
		{
			if (HasSupportContact())
			{
				ndFloat32 legAngle = ndFloat32(actions[m_softLegControl]) * ND_MAX_LEG_ANGLE_STEP + m_legJoint->GetAngle();
				legAngle = ndClamp(legAngle, -ND_MAX_LEG_JOINT_ANGLE, ND_MAX_LEG_JOINT_ANGLE);
				m_legJoint->SetTargetAngle(legAngle);

				ndBodyDynamic* const wheelBody = m_wheelJoint->GetBody0()->GetAsBodyDynamic();
				const ndMatrix matrix(m_wheelJoint->GetLocalMatrix1() * m_wheelJoint->GetBody1()->GetMatrix());

				ndVector torque(matrix.m_front.Scale(ndFloat32(actions[m_softWheelControl]) * ND_MAX_WHEEL_TORQUE));
				wheelBody->SetTorque(torque);
			}
			else
			{
				m_legJoint->SetTargetAngle(ndFloat32(0.0f));
				ndBodyDynamic* const wheelBody = m_wheelJoint->GetBody0()->GetAsBodyDynamic();
				const ndMatrix matrix(m_wheelJoint->GetLocalMatrix1() * m_wheelJoint->GetBody1()->GetMatrix());
				const ndVector omega(wheelBody->GetOmega());
				const ndVector torque(matrix.m_front.Scale(ND_MAX_WHEEL_TORQUE) * ndSign(omega.m_z));
				wheelBody->SetTorque(torque);
			}
		}

		void ResetModel()
		{
			for (ndInt32 i = 0; i < m_basePose.GetCount(); i++)
			{
				m_basePose[i].SetPose();
			}
			m_legJoint->SetTargetAngle(0.0f);
			m_wheelJoint->SetTargetAngle(0.0f);
		}

		bool IsOutOfBounds() const
		{
			ndBodyKinematic* const body = GetRoot()->m_body->GetAsBodyKinematic();
			return ndAbs(body->GetMatrix().m_posit.m_x) > ndFloat32(30.0f);
		}

		void TelePort() const
		{
			ndBodyKinematic* const body = GetRoot()->m_body->GetAsBodyKinematic();

			//ndVector veloc(body->GetVelocity());
			ndVector posit(body->GetMatrix().m_posit);
			posit.m_y = 0.0f;
			posit.m_z = 0.0f;
			posit.m_w = 0.0f;
			for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
			{
				ndBodyKinematic* const modelBody = m_bodies[i];
				//modelBody->SetVelocity(modelBody->GetVelocity() - veloc);
				ndMatrix matrix(modelBody->GetMatrix());
				matrix.m_posit -= posit;
				modelBody->SetMatrix(matrix);
			}
		}

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			ndModelArticulation::Update(world, timestep);
			m_timestep = timestep;
			m_agent->Step();
		}

		void PostUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			ndModelArticulation::PostUpdate(world, timestep);
			if (IsOutOfBounds())
			{
				TelePort();
			}
		}

		ndSharedPtr<ndBrainAgent> m_agent;
		ndFixSizeArray<ndBasePose, 8> m_basePose;
		ndFixSizeArray<ndBodyDynamic*, 8> m_bodies;
		ndJointHinge* m_legJoint;
		ndJointHinge* m_wheelJoint;
		ndBodyKinematic* m_wheel;
		ndFloat32 m_timestep;
	};

	void BuildModel(ndRobot* const model, ndDemoEntityManager* const scene, const ndMatrix& location)
	{
		ndFloat32 mass = 20.0f;
		ndFloat32 limbMass = 1.0f;
		ndFloat32 wheelMass = 1.0f;

		ndFloat32 xSize = 0.25f;
		ndFloat32 ySize = 0.40f;
		ndFloat32 zSize = 0.30f;
		ndPhysicsWorld* const world = scene->GetWorld();
		
		// add hip body
		ndSharedPtr<ndBody> hipBody(world->GetBody(AddBox(scene, location, mass, xSize, ySize, zSize, "wood_0.png")));
		ndModelArticulation::ndNode* const modelRoot = model->AddRootBody(hipBody);

		//ndMatrix matrix(hipBody->GetMatrix());
		ndMatrix matrix(location);
		matrix.m_posit.m_y += 0.6f;
		hipBody->SetMatrix(matrix);

		ndMatrix limbLocation(matrix);
		limbLocation.m_posit.m_z += zSize * 0.0f;
		limbLocation.m_posit.m_y -= ySize * 0.5f;

		// make single leg
		ndFloat32 limbLength = 0.3f;
		ndFloat32 limbRadio = 0.025f;

		ndSharedPtr<ndBody> legBody(world->GetBody(AddCapsule(scene, ndGetIdentityMatrix(), limbMass, limbRadio, limbRadio, limbLength, "wood_1.png")));
		ndMatrix legLocation(ndRollMatrix(-90.0f * ndDegreeToRad) * limbLocation);
		legLocation.m_posit.m_y -= limbLength * 0.5f;
		legBody->SetMatrix(legLocation);
		ndMatrix legPivot(ndYawMatrix(90.0f * ndDegreeToRad) * legLocation);
		legPivot.m_posit.m_y += limbLength * 0.5f;
		ndSharedPtr<ndJointBilateralConstraint> legJoint(new ndJointHinge(legPivot, legBody->GetAsBodyKinematic(), modelRoot->m_body->GetAsBodyKinematic()));
		ndJointHinge* const hinge = (ndJointHinge*)*legJoint;
		hinge->SetAsSpringDamper(0.02f, 1500, 40.0f);
		model->m_legJoint = hinge;

		// make wheel
		ndFloat32 wheelRadio = 4.0f * limbRadio;
		ndSharedPtr<ndBody> wheelBody(world->GetBody(AddSphere(scene, ndGetIdentityMatrix(), wheelMass, wheelRadio, "wood_0.png")));
		ndMatrix wheelMatrix(legPivot);
		wheelMatrix.m_posit.m_y -= limbLength;
		wheelBody->SetMatrix(wheelMatrix);
		ndSharedPtr<ndJointBilateralConstraint> wheelJoint(new ndJointHinge(wheelMatrix, wheelBody->GetAsBodyKinematic(), legBody->GetAsBodyKinematic()));
		ndJointHinge* const wheelMotor = (ndJointHinge*)*wheelJoint;
		wheelMotor->SetAsSpringDamper(0.02f, 0.0f, 0.2f);
		model->m_wheelJoint = wheelMotor;
		model->m_wheel = wheelBody->GetAsBodyKinematic();

		world->AddJoint(legJoint);
		world->AddJoint(wheelJoint);

		// add model limbs
		ndModelArticulation::ndNode* const legLimb = model->AddLimb(modelRoot, legBody, legJoint);
		model->AddLimb(legLimb, wheelBody, wheelJoint);

		model->m_bodies.SetCount(0);
		model->m_basePose.SetCount(0);
		for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
		{
			ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();
			model->m_bodies.PushBack(body);
			model->m_basePose.PushBack(body);
		}
	}

#ifdef ND_TRAIN_AGENT
	ndRobot* CreateTrainModel(ndDemoEntityManager* const scene, const ndMatrix& location, ndSharedPtr<ndBrainAgent>& agent)
	{
		ndRobot* const model = new ndRobot(agent);
		ndRobot::ndControllerTrainer* const trainer = (ndRobot::ndControllerTrainer*)*agent;
		trainer->m_model = model;
		trainer->SetName(CONTROLLER_NAME);

		BuildModel(model, scene, location);

		ndWorld* const world = scene->GetWorld();
		ndModelArticulation* const articulation = (ndModelArticulation*)model->GetAsModelArticulation();
		ndBodyKinematic* const rootBody = articulation->GetRoot()->m_body->GetAsBodyKinematic();
		ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointPlane(rootBody->GetMatrix().m_posit, ndVector(0.0f, 0.0f, 1.0f, 0.0f), rootBody, world->GetSentinelBody()));
		world->AddJoint(fixJoint);

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
			,m_lastEpisode(-1)
			,m_stopTraining(200 * 1000000)
			,m_modelIsTrained(false)
		{
			ndWorld* const world = scene->GetWorld();

			m_outFile = fopen("unicycle-VPG.csv", "wb");
			fprintf(m_outFile, "vpg\n");

			ndBrainAgentContinuePolicyGradient_TrainerMaster<m_stateSize, m_actionsSize>::HyperParameters hyperParameters;

			hyperParameters.m_discountFactor = ndReal(0.99f);
			hyperParameters.m_maxTrajectorySteps = 1024 * 8;
			hyperParameters.m_extraTrajectorySteps = 256;

			m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster<m_stateSize, m_actionsSize>>(new ndBrainAgentContinuePolicyGradient_TrainerMaster<m_stateSize, m_actionsSize>(hyperParameters));
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
				ndFloat32 step = 3.0f * (ndRand() - 0.5f);
				//step += ndSign(step) * 2.0f;
				location.m_posit.m_x += step;

				ndSharedPtr<ndBrainAgent> agent(new ndRobot::ndControllerTrainer(m_master));
				ndSharedPtr<ndModel> model(CreateModel(scene, location, agent));
				world->AddModel(model);
				//HideModel(model);
				SetMaterial(model);
				m_models.Append(model);
			}
		}

		~TrainingUpdata()
		{
			if (m_outFile)
			{
				fclose(m_outFile);
			}
		}

		void OnDebug(ndDemoEntityManager* const, bool mode)
		{
			for (ndList<ndSharedPtr<ndModel>>::ndNode* node = m_models.GetFirst(); node; node = node->GetNext())
			{
				HideModel(node->GetInfo(), mode);
			}
		}

		void HideModel(ndSharedPtr<ndModel>& model, bool mode) const
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
				mode ? ent->Hide() : ent->UnHide();

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
						ndExpandTraceMessage("best actor episode: %d\taverageFrames: %f\taverageValue %f\n", m_master->GetEposideCount(), m_master->GetAverageFrames(), m_master->GetAverageScore());
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
			if ((stopTraining >= m_stopTraining) || (m_master->GetAverageScore() > ndFloat32(98.5f)))
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

		ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster<m_stateSize, m_actionsSize>> m_master;
		ndSharedPtr<ndBrain> m_bestActor;
		ndList<ndSharedPtr<ndModel>> m_models;
		FILE* m_outFile;
		ndUnsigned64 m_timer;
		ndFloat32 m_maxScore;
		ndInt32 m_lastEpisode;
		ndInt32 m_stopTraining;
		bool m_modelIsTrained;
	};

#else
	ndModelArticulation* CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location)
	{
		// build neural net controller
		char fileName[1024];
		ndGetWorkingFileName(CONTROLLER_NAME, fileName);
		ndSharedPtr<ndBrain> actor(ndBrainLoad::Load(fileName));
		ndSharedPtr<ndBrainAgent> agent(new ndRobot::ndController(actor));

		ndRobot* const model = new ndRobot(agent);
		BuildModel(model, scene, location);
		((ndRobot::ndController*)*agent)->SetModel(model);
		return model;
	}
#endif
}

using namespace ndUnicycle;

void ndUnicycleController(ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildFloorBox(scene, ndGetIdentityMatrix());
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

	ndModelArticulation* const articulation = (ndModelArticulation*)model->GetAsModelArticulation();
	ndBodyKinematic* const rootBody = articulation->GetRoot()->m_body->GetAsBodyKinematic();
	ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointPlane(rootBody->GetMatrix().m_posit, ndVector(0.0f, 0.0f, 1.0f, 0.0f), rootBody, world->GetSentinelBody()));
	world->AddJoint(fixJoint);
#endif

	
	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 3.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
