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
	#define CONTROLLER_NAME			"unicycle-vpg.dnn"

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

	void ExportUrdfModel(ndDemoEntityManager* const scene)
	{
		ndFloat32 mass = 20.0f;
		ndFloat32 limbMass = 1.0f;
		ndFloat32 wheelMass = 1.0f;

		ndFloat32 xSize = 0.25f;
		ndFloat32 ySize = 0.40f;
		ndFloat32 zSize = 0.30f;

		ndSharedPtr<ndModelArticulation> model(new ndModelArticulation);

		ndMatrix location(ndGetIdentityMatrix());
		ndBodyKinematic* const hipBody = CreateBox(scene, ndGetIdentityMatrix(), mass, xSize, ySize, zSize, "wood_0.png");
		ndModelArticulation::ndNode* const modelRoot = model->AddRootBody(hipBody);
		hipBody->SetMatrix(location);
		modelRoot->m_name = "base_link";

		// make single leg
		ndFloat32 limbLength = 0.3f;
		ndFloat32 limbRadio = 0.025f;
		ndBodyKinematic* const legBody = CreateCapsule(scene, ndGetIdentityMatrix(), limbMass, limbRadio, limbRadio, limbLength, "wood_1.png");

		ndMatrix legPivot(ndRollMatrix(ndPi * ndFloat32(0.5f)) * hipBody->GetMatrix());
		legPivot.m_posit.m_y -= (limbLength * 0.5f + ySize * 0.5f);
		legBody->SetMatrix(legPivot);

		legPivot.m_posit.m_y += limbLength * 0.5f;
		ndMatrix legJointMatrix(ndYawMatrix(-ndPi * ndFloat32(0.5f)) * legPivot);
		ndJointHinge* const legJoint = new ndJointHinge(legJointMatrix, legBody, hipBody);
		ndModelArticulation::ndNode* const legLimb = model->AddLimb(modelRoot, legBody, legJoint);
		legJoint->SetAsSpringDamper(0.02f, 1500, 40.0f);

		legLimb->m_name = "leg";

		// make wheel
		ndFloat32 wheelRadio = 4.0f * limbRadio;
		ndBodyKinematic* const wheelBody = CreateSphere(scene, ndGetIdentityMatrix(), wheelMass, wheelRadio, "wood_0.png");
		
		ndMatrix wheelMatrix(legBody->GetMatrix());
		wheelMatrix.m_posit.m_y -= limbLength * 0.5f;
		wheelBody->SetMatrix(wheelMatrix);
		ndMatrix wheelJointMatrix(ndYawMatrix(-ndPi * ndFloat32(0.5f)) * wheelMatrix);
		ndJointHinge* const wheelJoint = new ndJointHinge(wheelJointMatrix, wheelBody, legBody);
		ndModelArticulation::ndNode* const wheel = model->AddLimb(legLimb, wheelBody, wheelJoint);
		wheelJoint->SetAsSpringDamper(0.02f, 0.0f, 0.2f);

		wheel->m_name = "wheel";

		ndUrdfFile urdf;
		char fileName[256];
		ndGetWorkingFileName("unicycle.urdf", fileName);
		urdf.Export(fileName, *model);
	}

	ndModelArticulation* CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location)
	{
		ndUrdfFile urdf;
		char fileName[256];
		ndGetWorkingFileName("unicycle.urdf", fileName);
		ndModelArticulation* const unicycle = urdf.Import(fileName);

		SetModelVisualMesh(scene, unicycle);
		ndMatrix matrix(unicycle->GetRoot()->m_body->GetMatrix() * location);
		matrix.m_posit = location.m_posit;
		unicycle->SetTransform(matrix);

		return unicycle;
	}

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

	class RobotModelNotify : public ndModelNotify
	{
		class ndController : public ndBrainAgentContinuePolicyGradient
		{
			public:
			ndController(const ndSharedPtr<ndBrain>& brain)
				:ndBrainAgentContinuePolicyGradient(brain)
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

		class ndControllerTrainer : public ndBrainAgentContinuePolicyGradient_Trainer
		{
			public:
			ndControllerTrainer(const ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master)
				:ndBrainAgentContinuePolicyGradient_Trainer(master)
				,m_robot(nullptr)
			{
			}

			ndBrainFloat CalculateReward()
			{
				return m_robot->CalculateReward();
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
			m_timestep = 0.0f;
			m_controllerTrainer = new ndControllerTrainer(master);
			m_controllerTrainer->m_robot = this;
			Init(robot);
		}

		RobotModelNotify(const ndSharedPtr<ndBrain>& policy, ndModelArticulation* const model)
			:ndModelNotify()
			,m_controller(nullptr)
			,m_controllerTrainer(nullptr)
		{
			m_timestep = 0.0f;
			m_controller = new ndController(policy);
			m_controller->m_robot = this;
			Init(model);
		}

		RobotModelNotify(const RobotModelNotify& src)
			:ndModelNotify(src)
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

		void Init(ndModelArticulation* const model)
		{
			m_legJoint = (ndJointHinge*)*model->FindByName("leg")->m_joint;
			m_wheelJoint = (ndJointHinge*)*model->FindByName("wheel")->m_joint;
			m_wheel = model->FindByName("wheel")->m_body->GetAsBodyDynamic();

			m_bodies.SetCount(0);
			m_basePose.SetCount(0);
			for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();
				m_bodies.PushBack(body);
				m_basePose.PushBack(body);
			}
		}

		ndModelNotify* Clone() const
		{
			ndAssert(0);
			return nullptr;
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

		ndFloat32 GetAngle() const
		{
			ndModelArticulation* const model = (ndModelArticulation*)GetModel();
			const ndMatrix& matrix = model->GetRoot()->m_body->GetMatrix();
			return matrix.m_up.m_x;
		}

		bool IsTerminal() const
		{
			#define D_REWARD_MIN_ANGLE	(ndFloat32 (20.0f) * ndDegreeToRad)
			ndFloat32 sinAngle = ndClamp(GetAngle(), ndFloat32(-0.9f), ndFloat32(0.9f));
			bool fail = ndAbs(ndAsin(sinAngle)) > (D_REWARD_MIN_ANGLE * ndFloat32(2.0f));
			return fail;
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

		ndBrainFloat CalculateReward()
		{
			//ndFloat32 legReward = ndReal(ndExp(-ndFloat32(10000.0f) * m_legJoint->GetAngle() * m_legJoint->GetAngle()));
			//if (HasSupportContact())
			//{
			//	ndModelArticulation* const model = (ndModelArticulation*)GetModel();
			//	ndBodyKinematic* const boxBody = model->GetRoot()->m_body->GetAsBodyKinematic();
			//
			//	const ndVector boxVeloc(boxBody->GetVelocity());
			//	const ndMatrix& matrix = boxBody->GetMatrix();
			//	const ndFloat32 sinAngle = matrix.m_up.m_x;
			//	const ndFloat32 boxReward = ndReal(ndExp(-ndFloat32(1000.0f) * sinAngle * sinAngle));
			//	const ndFloat32 speedReward = ndReal(ndExp(-ndFloat32(0.5f) * boxVeloc.m_x * boxVeloc.m_x));
			//	const ndFloat32 reward = (speedReward + boxReward + legReward) / 3.0f;
			//	return ndReal(reward);
			//}
			//else
			//{
			//	const ndVector wheelOmega(m_wheel->GetOmega());
			//	const ndFloat32 wheelReward = ndReal(ndExp(-ndFloat32(10000.0f) * wheelOmega.m_z * wheelOmega.m_z));
			//	const ndFloat32 reward = (wheelReward + legReward) / 2.0f;
			//	return ndReal(reward);
			//}

			ndFloat32 legReward = ndReal(ndExp(-ndFloat32(5000.0f) * m_legJoint->GetAngle() * m_legJoint->GetAngle()));
			if (HasSupportContact())
			{
				ndModelArticulation* const model = (ndModelArticulation*)GetModel();
				ndBodyKinematic* const boxBody = model->GetRoot()->m_body->GetAsBodyKinematic();

				const ndVector boxVeloc(boxBody->GetVelocity());
				const ndFloat32 sinAngle = GetAngle();
				const ndFloat32 boxReward = ndReal(ndExp(-ndFloat32(2000.0f) * sinAngle * sinAngle));
				const ndFloat32 speedReward = ndReal(ndExp(-ndFloat32(1.0f) * boxVeloc.m_x * boxVeloc.m_x));
				const ndFloat32 reward = 0.2f * speedReward + 0.5f * boxReward + 0.3f * legReward;
				return ndReal(reward);
			}
			else
			{
				const ndVector wheelOmega(m_wheel->GetOmega());
				const ndFloat32 wheelReward = ndReal(ndExp(-ndFloat32(2000.0f) * wheelOmega.m_z * wheelOmega.m_z));
				const ndFloat32 reward = 0.5f * wheelReward + 0.5f * legReward;
				return ndReal(reward);
			}
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

				if (m_controllerTrainer)
				{
					const ndFloat32 probability = 1.0f / 1000.0f;
					ndFloat32 applySideImpule = ndRand();
					if (applySideImpule < probability)
					{
						ndModelArticulation* const model = (ndModelArticulation*)GetModel();
						ndBodyDynamic* const boxBody = model->GetRoot()->m_body->GetAsBodyDynamic();

						const ndVector front(boxBody->GetMatrix().m_front);
						ndFloat32 speed = 0.5f * (ndRand() - 0.5f);
						ndVector impulse(front.Scale(speed * boxBody->GetMassMatrix().m_w));
						boxBody->ApplyImpulsePair(impulse, ndVector::m_zero, m_timestep);
					}
				}
			}
			else
			{
				m_legJoint->SetTargetAngle(ndFloat32(0.0f));
				ndBodyDynamic* const wheelBody = m_wheelJoint->GetBody0()->GetAsBodyDynamic();
				const ndMatrix matrix(m_wheelJoint->GetLocalMatrix1() * m_wheelJoint->GetBody1()->GetMatrix());
				const ndVector omega(wheelBody->GetOmega());
				const ndVector torque(matrix.m_front.Scale(ND_MAX_WHEEL_TORQUE) * ndSign(-omega.m_z));
				wheelBody->SetTorque(torque);
			}
		}

		void GetObservation(ndBrainFloat* const observation)
		{
			ndModelArticulation* const model = (ndModelArticulation*)GetModel();
			ndBodyKinematic* const body = model->GetRoot()->m_body->GetAsBodyKinematic();
			const ndVector omega(body->GetOmega());
			const ndVector veloc(body->GetVelocity());
			const ndVector wheelOmega(m_wheel->GetOmega());
			const ndFloat32 sinAngle = ndClamp(GetAngle(), ndFloat32(-0.9f), ndFloat32(0.9f));
			const ndFloat32 angle = ndAsin(sinAngle);

			observation[m_speed] = ndBrainFloat(veloc.m_x);
			observation[m_topBoxAngle] = ndBrainFloat(angle);
			observation[m_topBoxOmega] = ndBrainFloat(omega.m_z);
			observation[m_wheelOmega] = ndBrainFloat(wheelOmega.m_z);
			observation[m_isOnAir] = HasSupportContact() ? ndBrainFloat(0.0f) : ndBrainFloat(1.0f);
			observation[m_jointAngle] = ndBrainFloat(m_legJoint->GetAngle() / ND_MAX_LEG_JOINT_ANGLE);
		}

		void TelePort() const
		{
			ndModelArticulation* const model = (ndModelArticulation*)GetModel();
			ndBodyKinematic* const body = model->GetRoot()->m_body->GetAsBodyKinematic();

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

		bool IsOutOfBounds() const
		{
			ndModelArticulation* const model = (ndModelArticulation*)GetModel();
			ndBodyKinematic* const body = model->GetRoot()->m_body->GetAsBodyKinematic();
			return ndAbs(body->GetMatrix().m_posit.m_x) > ndFloat32(30.0f);
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
			if (IsOutOfBounds())
			{
				TelePort();
			}
		}

		void PostTransformUpdate(ndFloat32)
		{
		}

		ndFixSizeArray<ndBasePose, 8> m_basePose;
		ndFixSizeArray<ndBodyDynamic*, 8> m_bodies;

		ndController* m_controller;
		ndControllerTrainer* m_controllerTrainer;
		ndJointHinge* m_legJoint;
		ndJointHinge* m_wheelJoint;
		ndBodyKinematic* m_wheel;
		ndFloat32 m_timestep;
	};

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
			,m_lastEpisode(0xffffffff)
			,m_stopTraining(500 * 1000000)
			,m_modelIsTrained(false)
		{
			ndWorld* const world = scene->GetWorld();
			
			m_outFile = fopen("unicycle-vpg.csv", "wb");
			fprintf(m_outFile, "vpg\n");
			
			ndBrainAgentContinuePolicyGradient_TrainerMaster::HyperParameters hyperParameters;
			
			hyperParameters.m_extraTrajectorySteps = 256;
			hyperParameters.m_maxTrajectorySteps = 1024 * 8;
			hyperParameters.m_numberOfActions = m_actionsSize;
			hyperParameters.m_numberOfObservations = m_stateSize;
			hyperParameters.m_discountFactor = ndReal(m_discountFactor);
			
			m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>(new ndBrainAgentContinuePolicyGradient_TrainerMaster(hyperParameters));
			m_bestActor = ndSharedPtr< ndBrain>(new ndBrain(*m_master->GetPolicyNetwork()));
			
			m_master->SetName(CONTROLLER_NAME);
			
			ndSharedPtr<ndModel>visualModel (CreateModel(scene, matrix));
			ndBodyKinematic* const rootBody = visualModel->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
			ndSharedPtr<ndJointBilateralConstraint> visualPlaneJoint(new ndJointPlane(rootBody->GetMatrix().m_posit, ndVector(0.0f, 0.0f, 1.0f, 0.0f), rootBody, world->GetSentinelBody()));
			world->AddJoint(visualPlaneJoint);

			visualModel->SetNotifyCallback(new RobotModelNotify(m_master, visualModel->GetAsModelArticulation()));
			SetMaterial(visualModel->GetAsModelArticulation());

			world->AddModel(visualModel);
			visualModel->AddBodiesAndJointsToWorld();
			
			// add a hidden battery of model to generate trajectories in parallel
			const ndInt32 countX = 32;
			for (ndInt32 i = 0; i < countX; ++i)
			{
				ndMatrix location(matrix);
				ndFloat32 step = 6.0f * (ndRand() - 0.5f);
				location.m_posit.m_x += step;
			 
				ndSharedPtr<ndModel>model (CreateModel(scene, location));
				ndBodyKinematic* const body = model->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
				ndSharedPtr<ndJointBilateralConstraint> planeJoint(new ndJointPlane(body->GetMatrix().m_posit, ndVector(0.0f, 0.0f, 1.0f, 0.0f), body, world->GetSentinelBody()));
				world->AddJoint(planeJoint);

				model->SetNotifyCallback(new RobotModelNotify(m_master, model->GetAsModelArticulation()));
				SetMaterial(model->GetAsModelArticulation());

				world->AddModel(model);
				model->AddBodiesAndJointsToWorld();

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

		void SetMaterial(ndModelArticulation* const robot) const
		{
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

		void OnDebug(ndDemoEntityManager* const, bool mode)
		{
			for (ndList<ndModelArticulation*>::ndNode* node = m_models.GetFirst(); node; node = node->GetNext())
			{
				HideModel(node->GetInfo(), mode);
			}
		}

		void HideModel(ndModelArticulation* const robot, bool mode) const
		{
			ndModelArticulation::ndNode* stackMem[128];
			ndInt32 stack = 1;
			stackMem[0] = robot->GetRoot();
			while (stack)
			{
				stack--;
				ndModelArticulation::ndNode* const node = stackMem[stack];
				ndBody* const body = *node->m_body;
				ndDemoEntityNotify* const userData = (ndDemoEntityNotify*)body->GetNotifyCallback();
				ndDemoEntity* const ent = *userData->m_entity;
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

		virtual void Update(ndDemoEntityManager* const manager, ndFloat32)
		{
			ndUnsigned32 stopTraining = m_master->GetFramesCount();
			if (stopTraining <= m_stopTraining)
			{
				ndUnsigned32 episodeCount = m_master->GetEposideCount();
				m_master->OptimizeStep();

				episodeCount -= m_master->GetEposideCount();
				ndFloat32 rewardTrajectory = m_master->GetAverageFrames() * m_master->GetAverageScore();
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

			if ((stopTraining >= m_stopTraining) || (100.0f * m_master->GetAverageScore() / m_horizon > 98.0f))
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
		ndFloat32 m_discountFactor;
		ndFloat32 m_horizon;
		ndUnsigned32 m_lastEpisode;
		ndUnsigned32 m_stopTraining;
		bool m_modelIsTrained;
	};
}

using namespace ndUnicycle;


void ndUnicycleController(ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildFloorBox(scene, ndGetIdentityMatrix());
	BuildFlatPlane(scene, true);
	
	ndSetRandSeed(42);

//	ExportUrdfModel(scene);

	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit.m_y = 0.6f;

#ifdef ND_TRAIN_AGENT
	TrainingUpdata* const trainer = new TrainingUpdata(scene, matrix);
	scene->RegisterPostUpdate(trainer);
#else
	ndWorld* const world = scene->GetWorld();
	
	ndModelArticulation* model = CreateModel(scene, matrix);
	world->AddModel(model);
	model->AddBodiesAndJointsToWorld();

	ndBodyKinematic* const rootBody = model->GetRoot()->m_body->GetAsBodyKinematic();
	ndSharedPtr<ndJointBilateralConstraint> planeJoint(new ndJointPlane(rootBody->GetMatrix().m_posit, ndVector(0.0f, 0.0f, 1.0f, 0.0f), rootBody, world->GetSentinelBody()));
	world->AddJoint(planeJoint);

	char fileName[256];
	ndGetWorkingFileName(CONTROLLER_NAME, fileName);
	ndSharedPtr<ndBrain> policy(ndBrainLoad::Load(fileName));
	model->SetNotifyCallback(new RobotModelNotify(policy, model));
#endif
	
	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 3.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
