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

	#define ND_MAX_LEG_JOINT_ANGLE	(ndFloat32 (45.0f) * ndDegreeToRad)

	#define ND_MAX_WHEEL_ALPHA		(ndFloat32 (500.0f))

	#define ND_TERMINATION_ANGLE	(ndFloat32 (45.0f) * ndDegreeToRad)
	#define ND_TRAJECTORY_STEPS		(1024 * 4)

	#define USE_SAC

	enum ndActionSpace
	{
		m_wheelTorque,
		m_actionsSize
	};

	enum ndStateSpace
	{
		m_poleAngle,
		m_poleOmega,
		m_wheelOmega,
		m_hasSupportContact,
		m_observationsSize
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
		ndFloat32 ballMass = 5.0f;

		ndMatrix matrix(entity->GetCurrentMatrix() * location);
		matrix.m_posit = location.m_posit;
		ndSharedPtr<ndBody> rootBody(CreateRigidBody(entity, matrix, boxMass, nullptr));
		ndModelArticulation::ndNode* const modelRootNode = model->AddRootBody(rootBody);
		rootBody->GetAsBodyKinematic()->SetAngularDamping(ndVector(0.5f));

		ndSharedPtr<ndDemoEntity> poleEntity(entity->GetChildren().GetFirst()->GetInfo());
		const ndMatrix poleMatrix(poleEntity->GetCurrentMatrix() * matrix);
		ndSharedPtr<ndBody> pole(CreateRigidBody(poleEntity, poleMatrix, poleMass, rootBody->GetAsBodyDynamic()));
		ndSharedPtr<ndJointBilateralConstraint> poleHinge(new ndJointHinge(poleMatrix, pole->GetAsBodyKinematic(), rootBody->GetAsBodyKinematic()));
		ndModelArticulation::ndNode* const poleLink = model->AddLimb(modelRootNode, pole, poleHinge);
		poleLink->m_name = "leg";
		//((ndJointHinge*)*poleHinge)->SetAsSpringDamper(0.02f, 1500.0f, 40.0f);
		
		ndSharedPtr<ndDemoEntity> ballEntity(poleEntity->GetChildren().GetFirst()->GetInfo());
		const ndMatrix ballMatrix(ballEntity->GetCurrentMatrix() * poleMatrix);
		ndSharedPtr<ndBody> ball(CreateRigidBody(ballEntity, ballMatrix, ballMass, pole->GetAsBodyDynamic()));
		ndSharedPtr<ndJointBilateralConstraint> ballHinge(new ndJointHinge(ballMatrix, ball->GetAsBodyKinematic(), pole->GetAsBodyKinematic()));
		ndModelArticulation::ndNode* const ballHingeNode = model->AddLimb(poleLink, ball, ballHinge);
		ballHingeNode->m_name = "wheel";
		//ball->GetAsBodyKinematic()->SetAngularDamping(ndVector(0.5f));
		//((ndJointHinge*)*ballHinge)->SetAsSpringDamper(0.02f, 0.0f, 0.1f);

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

#ifdef USE_SAC
		class ndControllerTrainer : public ndBrainAgentDeterministicPolicyGradient_Agent
#else
		class ndControllerTrainer : public ndBrainAgentContinuePolicyGradient_Agent
#endif
		{
			public:
#ifdef USE_SAC
			ndControllerTrainer(const ndSharedPtr<ndBrainAgentDeterministicPolicyGradient_Trainer>& master)
				:ndBrainAgentDeterministicPolicyGradient_Agent(master)
#else
			ndControllerTrainer(const ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master)
				:ndBrainAgentContinuePolicyGradient_Agent(master)
#endif	
				,m_solver()
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

			ndIkSolver m_solver;
			RobotModelNotify* m_robot;
		};

		public:
#ifdef USE_SAC
		RobotModelNotify(ndSharedPtr<ndBrainAgentDeterministicPolicyGradient_Trainer>& master, ndModelArticulation* const robot)
#else
		RobotModelNotify(ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master, ndModelArticulation* const robot)
#endif
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
			SetModel(model);
			m_poleJoint = (ndJointHinge*)*model->FindByName("leg")->m_joint;
			m_wheelJoint = (ndJointHinge*)*model->FindByName("wheel")->m_joint;
			m_wheel = model->FindByName("wheel")->m_body->GetAsBodyDynamic();
			m_pole = model->FindByName("leg")->m_body->GetAsBodyDynamic();

			m_bodies.SetCount(0);
			m_basePose.SetCount(0);
			for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();
				m_bodies.PushBack(body);
				m_basePose.PushBack(body);
			}
			ResetModel();
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

		//#pragma optimize( "", off )
		ndFloat32 GetBoxAngle() const
		{
			const ndMatrix matrix (m_poleJoint->CalculateGlobalMatrix1());
			ndFloat32 sinAngle = ndClamp(matrix.m_up.m_x, ndFloat32(-0.9f), ndFloat32(0.9f));
			ndFloat32 angle = ndAsin (sinAngle);
			return angle;
		}

		//#pragma optimize( "", off )
		ndFloat32 GetPoleAngle() const
		{
			const ndMatrix matrix(m_poleJoint->CalculateGlobalMatrix0());
			ndFloat32 sinAngle = ndClamp(matrix.m_up.m_x, ndFloat32(-0.9f), ndFloat32(0.9f));
			ndFloat32 angle = ndAsin(sinAngle);
			return angle;
		}

		//#pragma optimize( "", off )
		ndFloat32 CalculateComSpeed() const
		{
			ndFloat32 totalMass = 0.0f;
			ndVector totalMomentum(ndVector::m_zero);
			for (ndModelArticulation::ndNode* node = GetModel()->GetAsModelArticulation()->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();
				ndFloat32 mass = body->GetMassMatrix().m_w;
				totalMass += mass;
				totalMomentum += body->GetVelocity().Scale(mass);
			}
			totalMomentum = totalMomentum.Scale(1.0f / totalMass);
			return totalMomentum.m_x;
		}

		//#pragma optimize( "", off )
		bool IsTerminal() const
		{
			//bool fail = ndAbs(GetBoxAngle()) > ND_TERMINATION_ANGLE;
			bool fail = ndAbs(GetPoleAngle()) > ND_TERMINATION_ANGLE;
			return fail;
		}

		//#pragma optimize( "", off )
		void ResetModel()
		{
			for (ndInt32 i = 0; i < m_basePose.GetCount(); i++)
			{
				m_basePose[i].SetPose();
			}
			GetModel()->GetAsModelArticulation()->ClearMemory();

			ndFloat32 angle = (ndRand() - ndFloat32(0.5f)) * ndFloat32(20.0f);
			const ndMatrix randomRollMatrix(ndRollMatrix(angle * ndDegreeToRad));
			const ndMatrix matrix(randomRollMatrix * GetModel()->GetAsModelArticulation()->GetRoot()->m_body->GetMatrix());
			GetModel()->GetAsModelArticulation()->SetTransform(matrix);
		}
		
		ndFloat32 IsOnAir() const
		{
			ndBodyKinematic::ndContactMap& contacts = m_wheel->GetContactMap();
			ndBodyKinematic::ndContactMap::Iterator it(contacts);
			for (it.Begin(); it; it++)
			{
				ndContact* const contact = *it;
				if (contact->IsActive())
				{
					return 0.0f;
				}
			}
			return 1.0f;
		};

		//#pragma optimize( "", off )
		ndBrainFloat CalculateReward()
		{
			if (IsTerminal())
			{
				return ndBrainFloat (-1.0f);
			}

			ndIkSolver& solver = m_controllerTrainer->m_solver;

			ndBodyKinematic* const rootBody = GetModel()->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
			ndSkeletonContainer* const skeleton = rootBody->GetSkeleton();
			ndAssert(skeleton);

			const ndVector savedTorque(m_wheel->GetTorque());
			ndVector testTorque(savedTorque);
			testTorque.m_z = ndFloat32(0.0f);
			m_wheel->SetTorque(testTorque);

			const ndMatrix poleMatrix(m_wheelJoint->CalculateGlobalMatrix1());
			ndFloat32 sinPoleAngle = ndClamp(poleMatrix.m_up.m_x, ndFloat32(-0.9f), ndFloat32(0.9f));
			ndFloat32 wheelPoleAngle = ndAsin(sinPoleAngle);
			ndFloat32 deltaWheelPoleAngle = wheelPoleAngle * ndFloat32(0.25f);
			ndFloat32 targetAngle = m_wheelJoint->GetAngle() - deltaWheelPoleAngle;
			m_wheelJoint->SetTargetAngle(targetAngle);
			m_wheelJoint->SetAsSpringDamper(ndFloat32(1.0e-3f), ndFloat32(10000.0f), ndFloat32(100.0f));

			solver.SolverBegin(skeleton, nullptr, 0, GetModel()->GetWorld(), m_timestep);
			solver.Solve();
			ndVector wheelAlpha(m_wheel->GetAlpha());
			solver.SolverEnd();

			m_wheel->SetTorque(savedTorque);
			m_wheelJoint->SetAsSpringDamper(ndFloat32(0.0), ndFloat32(0.0f), ndFloat32(0.0f));

			auto AccelerationRewardLinearPenalty = [](ndFloat32 alpha)
			{
				ndFloat32 reward = ndFloat32(1.0f) - ndAbs(alpha) / ndFloat32(5.0f);
				if (reward < ndFloat32(0.0f))
				{
					reward = ndFloat32(0.0f);
				}
				if (reward > ndFloat32(1.0f))
				{
					reward = ndFloat32(1.0f);
				}
				return reward;
			};

			auto AccelerationRewardGaussianPenalty = [](ndFloat32 alpha)
			{
				//ndFloat32 reward = ndReal(ndExp(-ndFloat32(1.0e-1f) * alpha * alpha));
				ndFloat32 reward = ndReal(ndExp(-ndFloat32(0.5f) * alpha * alpha));
				return reward;
			};

			const ndVector wheelMass(m_wheel->GetMassMatrix());
			ndFloat32 alphaError = savedTorque.m_z / wheelMass.m_z - wheelAlpha.m_z;
			//ndFloat32 alphaReward = AccelerationRewardGaussianPenalty(alphaError);
			ndFloat32 alphaReward = AccelerationRewardLinearPenalty(alphaError);

			ndFloat32 poleAngle = GetPoleAngle();
			ndFloat32 poleReward = ndFloat32(ndExp(-ndFloat32(500.0f) * poleAngle * poleAngle));

			//ndFloat32 speedReward = ndFloat32(1.0f);
			//if ((alphaReward > ndFloat32(0.98f)) && (poleReward > ndFloat32(0.98f)))
			//{
			//	const ndVector velocDir(poleMatrix.m_front.CrossProduct(ndVector(0.0f, 1.0f, 0.0f, 0.0f)));
			//	ndFloat32 wheelSpeed = m_wheel->GetVelocity().DotProduct(velocDir).GetScalar();
			//	//speedReward = ndFloat32(ndExp(-ndFloat32(10.0f) * wheelSpeed * wheelSpeed));
			//}
			//ndFloat32 reward = 0.25f * poleReward + 0.25f * speedReward + 0.5f * alphaReward;

			//ndFloat32 robotSpeed = CalculateComSpeed();
			//ndFloat32 speedReward = ndFloat32(ndExp(-ndFloat32(100.0f) * robotSpeed * robotSpeed));
			//poleReward = 0.0f;

			//ndFloat32 reward = 0.25f * poleReward + 0.25f * speedReward + 0.5f * alphaReward;
			ndFloat32 reward = 0.5f * poleReward + 0.5f * alphaReward;
			if (IsOnAir())
			{
				ndVector wheelOmega(m_wheel->GetOmega());
				reward = ndExp(-ndFloat32(100.0f) * wheelOmega.m_z * wheelOmega.m_z);
			}

			return ndReal(reward);
		}

		//#pragma optimize( "", off )
		void ApplyActions(ndBrainFloat* const actions)
		{
			const ndVector wheelMass(m_wheel->GetMassMatrix());
			const ndMatrix wheelMatrix(m_wheelJoint->CalculateGlobalMatrix0());
			ndFloat32 wheelSpeedAlpha = actions[m_wheelTorque] * ND_MAX_WHEEL_ALPHA;
			ndVector torque(wheelMatrix.m_front.Scale(wheelSpeedAlpha * wheelMass.m_z));
			m_wheel->SetTorque(torque);
		}

		//#pragma optimize( "", off )
		void GetObservation(ndBrainFloat* const observation)
		{
			const ndMatrix poleMatrix(m_wheelJoint->CalculateGlobalMatrix1());
			ndFloat32 sinPoleAngle = ndClamp(poleMatrix.m_up.m_x, ndFloat32(-0.9f), ndFloat32(0.9f));
			ndFloat32 poleAngle = ndAsin(sinPoleAngle) / ND_MAX_LEG_JOINT_ANGLE;

			ndFloat32 poleOmega(m_pole->GetOmega().DotProduct(poleMatrix.m_front).GetScalar());
			ndFloat32 wheelOmega(m_wheel->GetOmega().DotProduct(poleMatrix.m_front).GetScalar());

			observation[m_poleAngle] = poleAngle;
			observation[m_poleOmega] = poleOmega;
			observation[m_wheelOmega] = wheelOmega;
			observation[m_hasSupportContact] = IsOnAir();
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
			return ndAbs(body->GetMatrix().m_posit.m_x) > ndFloat32(40.0f);
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
		ndJointHinge* m_poleJoint;
		ndJointHinge* m_wheelJoint;
		ndBodyKinematic* m_wheel;
		ndBodyKinematic* m_pole;
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
			,m_discountRewardFactor(0.99f)
			,m_horizon(ndFloat32(1.0f) / (ndFloat32(1.0f) - m_discountRewardFactor))
			,m_lastEpisode(0xffffffff)
			,m_stopTraining(100 * 1000000)
			,m_modelIsTrained(false)
		{
			ndWorld* const world = scene->GetWorld();
			
			m_outFile = fopen("unicycle.csv", "wb");
			fprintf(m_outFile, "vpg\n");

			#ifdef USE_SAC
				m_stopTraining = 250000;
				ndBrainAgentDeterministicPolicyGradient_Trainer::HyperParameters hyperParameters;
				hyperParameters.m_numberOfActions = m_actionsSize;
				hyperParameters.m_numberOfObservations = m_observationsSize;
				m_master = ndSharedPtr<ndBrainAgentDeterministicPolicyGradient_Trainer>(new ndBrainAgentSoftActorCritic_Trainer(hyperParameters));
				//m_master = ndSharedPtr<ndBrainAgentDeterministicPolicyGradient_Trainer>(new ndBrainAgentDeterministicPolicyGradient_Trainer(hyperParameters));
			#else
				ndBrainAgentContinuePolicyGradient_TrainerMaster::HyperParameters hyperParameters;
				hyperParameters.m_numberOfActions = m_actionsSize;
				hyperParameters.m_numberOfObservations = m_observationsSize;
				hyperParameters.m_maxTrajectorySteps = ND_TRAJECTORY_STEPS;
				hyperParameters.m_discountRewardFactor = ndReal(m_discountRewardFactor);
				hyperParameters.m_regularizerType = ndBrainOptimizer::m_lasso;
				//m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>(new ndBrainAgentContinuePolicyGradient_TrainerMaster(hyperParameters));
				m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>(new ndBrainAgentContinueProximaPolicyGradient_TrainerMaster(hyperParameters));
			#endif

			m_bestActor = ndSharedPtr<ndBrain>(new ndBrain(*m_master->GetPolicyNetwork()));
			m_master->SetName(CONTROLLER_NAME);

			ndSharedPtr<ndModel>visualModel (CreateModel(scene, matrix, modelMesh));
			world->AddModel(visualModel);
			visualModel->AddBodiesAndJointsToWorld();

			ndBodyKinematic* const rootBody = visualModel->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
			ndSharedPtr<ndJointBilateralConstraint> visualPlaneJoint(new ndJointPlane(rootBody->GetMatrix().m_posit, ndVector(0.0f, 0.0f, 1.0f, 0.0f), rootBody, world->GetSentinelBody()));
			world->AddJoint(visualPlaneJoint);
			
			visualModel->SetNotifyCallback(new RobotModelNotify(m_master, visualModel->GetAsModelArticulation()));
			SetMaterial(visualModel->GetAsModelArticulation());
			
			#ifndef USE_SAC
			// add a hidden battery of model to generate trajectories in parallel
			//const ndInt32 countX = 0;
			const ndInt32 countX = 100;
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
			#endif

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

#ifdef USE_SAC
		ndSharedPtr<ndBrainAgentDeterministicPolicyGradient_Trainer> m_master;
#else
		ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster> m_master;
#endif
		ndSharedPtr<ndBrain> m_bestActor;
		ndList<ndModelArticulation*> m_models;
		FILE* m_outFile;
		ndUnsigned64 m_timer;
		ndFloat32 m_maxScore;
		ndFloat32 m_discountRewardFactor;
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
	matrix.m_posit.m_y = 1.9f;

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
