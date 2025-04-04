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


namespace ndUnicycle
{
	#define ND_TRAIN_AGENT
	#define CONTROLLER_NAME			"unicycle.dnn"

	#define ND_MAX_WHEEL_TORQUE		(ndFloat32 (10.0f))
	#define ND_MAX_LEG_ANGLE_STEP	(ndFloat32 (4.0f) * ndDegreeToRad)
	#define ND_MAX_LEG_JOINT_ANGLE	(ndFloat32 (30.0f) * ndDegreeToRad)

	enum ndActionSpace
	{
		m_legControl,
		m_wheelControl,
		m_actionsSize
	};

	enum ndStateSpace
	{
		m_topBoxAngle,
		m_topBoxOmega,
		m_jointAngle,
		m_jointOmega,
		m_wheelSpeed,
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

		ndFloat32 boxMass = 20.0f;
		ndFloat32 poleMass = 1.0f;
		ndFloat32 ballMass = 1.0f;

		ndMatrix matrix(entity->GetCurrentMatrix() * location);
		matrix.m_posit = location.m_posit;
		ndSharedPtr<ndBody> rootBody(CreateRigidBody(entity, matrix, boxMass, nullptr));
		ndModelArticulation::ndNode* const modelRootNode = model->AddRootBody(rootBody);

		ndSharedPtr<ndDemoEntity> poleEntity(entity->GetChildren().GetFirst()->GetInfo());
		const ndMatrix poleMatrix(poleEntity->GetCurrentMatrix() * matrix);
		ndSharedPtr<ndBody> pole(CreateRigidBody(poleEntity, poleMatrix, poleMass, rootBody->GetAsBodyDynamic()));
		ndSharedPtr<ndJointBilateralConstraint> poleHinge(new ndJointHinge(poleMatrix, pole->GetAsBodyKinematic(), rootBody->GetAsBodyKinematic()));
		ndModelArticulation::ndNode* const poleLink = model->AddLimb(modelRootNode, pole, poleHinge);
		poleLink->m_name = "leg";
		((ndJointHinge*)*poleHinge)->SetAsSpringDamper(0.02f, 1500.0f, 40.0f);
		
		ndSharedPtr<ndDemoEntity> ballEntity(poleEntity->GetChildren().GetFirst()->GetInfo());
		const ndMatrix ballMatrix(ballEntity->GetCurrentMatrix() * poleMatrix);
		ndSharedPtr<ndBody> ball(CreateRigidBody(ballEntity, ballMatrix, ballMass, pole->GetAsBodyDynamic()));
		ndSharedPtr<ndJointBilateralConstraint> ballHinge(new ndJointHinge(ballMatrix, ball->GetAsBodyKinematic(), pole->GetAsBodyKinematic()));
		ndModelArticulation::ndNode* const ballLink = model->AddLimb(poleLink, ball, ballHinge);
		ballLink->m_name = "wheel";
		//((ndJointHinge*)*ballHinge)->SetAsSpringDamper(0.2f, 0.0f, 0.001f);

		//ndUrdfFile urdf;
		//char fileName[256];
		//ndGetWorkingFileName("unicycle.urdf", fileName);
		//urdf.Export(fileName, model->GetAsModelArticulation());

		return model;
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

		ndFloat32 GetBoxAngle() const
		{
			ndModelArticulation* const model = (ndModelArticulation*)GetModel();
			const ndMatrix& matrix = model->GetRoot()->m_body->GetMatrix();
			ndFloat32 sinAngle = ndClamp(matrix.m_up.m_x, ndFloat32(-0.9f), ndFloat32(0.9f));
			return ndAsin (sinAngle);
		}

		#pragma optimize( "", off )
		bool IsTerminal() const
		{
			#define D_REWARD_MIN_ANGLE	(ndFloat32 (20.0f) * ndDegreeToRad)
			bool fail = ndAbs(GetBoxAngle()) > (D_REWARD_MIN_ANGLE * ndFloat32(2.0f));
			return fail;
		}

		void ResetModel()
		{
			for (ndInt32 i = 0; i < m_basePose.GetCount(); i++)
			{
				m_basePose[i].SetPose();
			}
			GetModel()->GetAsModelArticulation()->ClearMemory();
		}

		//#pragma optimize( "", off )
		ndBrainFloat CalculateReward()
		{
			if (IsTerminal())
			{
				return ndBrainFloat (-1.0f);
			}

			const ndFloat32 boxAngle = GetBoxAngle();
			const ndVector wheelVeloc(m_wheel->GetVelocity());

			const ndFloat32 standingReward = ndReal(ndExp(-ndFloat32(2000.0f) * boxAngle * boxAngle));
			ndFloat32 speedReward = ndReal(ndExp(-ndFloat32(100.0f) * wheelVeloc.m_x * wheelVeloc.m_x));

			ndFloat32 reward = 0.5f * standingReward + 0.5f * speedReward;
			return ndReal(reward);
		}

		void ApplyActions(ndBrainFloat* const actions)
		{
			ndFloat32 legAngle = ndFloat32(actions[m_legControl]) * ND_MAX_LEG_ANGLE_STEP + m_legJoint->GetAngle();
			legAngle = ndClamp(legAngle, -ND_MAX_LEG_JOINT_ANGLE, ND_MAX_LEG_JOINT_ANGLE);
			m_legJoint->SetTargetAngle(legAngle);

			ndBodyDynamic* const wheelBody = m_wheelJoint->GetBody0()->GetAsBodyDynamic();
			const ndMatrix matrix(m_wheelJoint->GetLocalMatrix1() * m_wheelJoint->GetBody1()->GetMatrix());

			ndVector torque(matrix.m_front.Scale(ndFloat32(actions[m_wheelControl]) * ND_MAX_WHEEL_TORQUE));
			//ndVector torque(0.0f, 0.0f, - ND_MAX_WHEEL_TORQUE, 0.0f);
			wheelBody->SetTorque(torque);
		}

		void GetObservation(ndBrainFloat* const observation)
		{
			ndModelArticulation* const model = (ndModelArticulation*)GetModel();
			ndBodyKinematic* const boxBody = model->GetRoot()->m_body->GetAsBodyKinematic();
			const ndVector boxOmega(boxBody->GetOmega());
			const ndVector wheelVeloc(m_wheel->GetVelocity());
			

			ndFloat32 wheelSpeed = wheelVeloc.m_x / ndFloat32(10.0f);
			ndFloat32 boxAngularSpeed = boxOmega.m_z / ndFloat32(2.0f);
			ndFloat32 boxAngle = GetBoxAngle() / (D_REWARD_MIN_ANGLE * ndFloat32(2.0f));
			ndFloat32 legAngle = ndBrainFloat(m_legJoint->GetAngle()) / ND_MAX_LEG_JOINT_ANGLE;
			ndFloat32 legAngularSpeed = ndBrainFloat(m_legJoint->GetOmega()) / ndFloat32(2.0f);

			//ndTrace(("%f %f %f %f %f\n", wheelSpeed, boxAngularSpeed, boxAngle, legAngle, legAngularSpeed));

			observation[m_wheelSpeed] = wheelSpeed;
			observation[m_jointAngle] = legAngle;
			observation[m_topBoxAngle] = boxAngle;
			observation[m_jointOmega] = legAngularSpeed;
			observation[m_topBoxOmega] = boxAngularSpeed;
		}

		void TelePort() const
		{
			ndModelArticulation* const model = (ndModelArticulation*)GetModel();
			ndBodyKinematic* const body = model->GetRoot()->m_body->GetAsBodyKinematic();

			ndVector posit(body->GetMatrix().m_posit);
			posit.m_y = 0.0f;
			posit.m_z = 0.0f;
			posit.m_w = 0.0f;
			for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
			{
				ndBodyKinematic* const modelBody = m_bodies[i];
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
		TrainingUpdata(ndDemoEntityManager* const scene, const ndMatrix& matrix, const ndSharedPtr<ndDemoEntity>& modelMesh)
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
			
			m_outFile = fopen("unicycle.csv", "wb");
			fprintf(m_outFile, "vpg\n");
			
			ndBrainAgentContinuePolicyGradient_TrainerMaster::HyperParameters hyperParameters;
			
			hyperParameters.m_extraTrajectorySteps = 256;
			hyperParameters.m_maxTrajectorySteps = 1024 * 2;
			hyperParameters.m_numberOfActions = m_actionsSize;
			hyperParameters.m_numberOfObservations = m_stateSize;
			hyperParameters.m_discountFactor = ndReal(m_discountFactor);
			
			m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>(new ndBrainAgentContinuePolicyGradient_TrainerMaster(hyperParameters));
			//m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>(new ndBrainAgentContinueProximaPolicyGradient_TrainerMaster(hyperParameters));
			m_bestActor = ndSharedPtr<ndBrain>(new ndBrain(*m_master->GetPolicyNetwork()));
			
			m_master->SetName(CONTROLLER_NAME);

			ndSharedPtr<ndModel>visualModel (CreateModel(scene, matrix, modelMesh));
			world->AddModel(visualModel);
			visualModel->AddBodiesAndJointsToWorld();

			ndBodyKinematic* const rootBody = visualModel->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
			ndSharedPtr<ndJointBilateralConstraint> visualPlaneJoint(new ndJointPlane(rootBody->GetMatrix().m_posit, ndVector(0.0f, 0.0f, 1.0f, 0.0f), rootBody, world->GetSentinelBody()));
			//ndSharedPtr<ndJointBilateralConstraint> visualPlaneJoint(new ndJointSlider(rootBody->GetMatrix(), rootBody->GetAsBodyKinematic(), world->GetSentinelBody()));
			world->AddJoint(visualPlaneJoint);
			
			visualModel->SetNotifyCallback(new RobotModelNotify(m_master, visualModel->GetAsModelArticulation()));
			SetMaterial(visualModel->GetAsModelArticulation());
			
			// add a hidden battery of model to generate trajectories in parallel
			const ndInt32 countX = 100;
			//const ndInt32 countX = 0;
			for (ndInt32 i = 0; i < countX; ++i)
			{
				ndMatrix location(matrix);
				ndFloat32 step = 20.0f * (ndRand() - 0.5f);
				location.m_posit.m_x += step;
			 	
				ndSharedPtr<ndModel>model (CreateModel(scene, location, modelMesh));
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

	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit.m_y = 3.05f;

	ndMeshLoader loader;
	ndSharedPtr<ndDemoEntity> modelMesh(loader.LoadEntity("unicycle.fbx", scene));

#ifdef ND_TRAIN_AGENT
	TrainingUpdata* const trainer = new TrainingUpdata(scene, matrix, modelMesh);
	scene->RegisterPostUpdate(trainer);
#else
	ndWorld* const world = scene->GetWorld();
	
	ndSharedPtr<ndModel> model (CreateModel(scene, matrix, modelMesh));
	world->AddModel(model);
	model->AddBodiesAndJointsToWorld();

	ndBodyKinematic* const rootBody = model->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
	ndSharedPtr<ndJointBilateralConstraint> planeJoint(new ndJointPlane(rootBody->GetMatrix().m_posit, ndVector(0.0f, 0.0f, 1.0f, 0.0f), rootBody, world->GetSentinelBody()));
	world->AddJoint(planeJoint);

	char fileName[256];
	ndGetWorkingFileName(CONTROLLER_NAME, fileName);
	ndSharedPtr<ndBrain> policy(ndBrainLoad::Load(fileName));
	model->SetNotifyCallback(new RobotModelNotify(policy, model->GetAsModelArticulation()));
#endif
	
	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.0f;
	matrix.m_posit.m_z += 8.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
