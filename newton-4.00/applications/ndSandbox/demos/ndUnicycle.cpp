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
	#define CONTROLLER_NAME			"unicycle_sac.dnn"

	#define ND_MAX_LEG_JOINT_ANGLE	(ndFloat32 (45.0f) * ndDegreeToRad)

	#define ND_MAX_WHEEL_ALPHA		(ndFloat32 (500.0f))

	#define ND_TERMINATION_ANGLE	(ndFloat32 (45.0f) * ndDegreeToRad)
	#define ND_TRAJECTORY_STEPS		(1024 * 4)

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
		m_velocity,
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
		modelRootNode->m_name = "box";

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

#if 1
		ndSharedPtr<ndJointBilateralConstraint> ballHinge(new ndJointRoller(ballMatrix, ball->GetAsBodyKinematic(), pole->GetAsBodyKinematic()));
		((ndJointRoller*)*ballHinge)->SetAsSpringDamperPosit(0.01f, 1000.0f, 15.0f);
#else
		ndSharedPtr<ndJointBilateralConstraint> ballHinge(new ndJointHinge(ballMatrix, ball->GetAsBodyKinematic(), pole->GetAsBodyKinematic()));
#endif
		ndModelArticulation::ndNode* const ballHingeNode = model->AddLimb(poleLink, ball, ballHinge);
		ballHingeNode->m_name = "wheel";

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

		class ndControllerTrainer : public ndBrainAgentOffPolicyGradient_Agent
		{
			public:
			ndControllerTrainer(const ndSharedPtr<ndBrainAgentOffPolicyGradient_Trainer>& master)
				:ndBrainAgentOffPolicyGradient_Agent(master)
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
		RobotModelNotify(ndSharedPtr<ndBrainAgentOffPolicyGradient_Trainer>& master, ndModelArticulation* const robot)
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

			m_box = model->FindByName("box")->m_body->GetAsBodyDynamic();
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
		bool IsTerminal() const
		{
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

			ndFloat32 angle = (ndRand() - ndFloat32(0.5f)) * ndFloat32(10.0f);
			const ndMatrix randomRollMatrix(ndRollMatrix(angle * ndDegreeToRad));
			const ndMatrix matrix(randomRollMatrix * GetModel()->GetAsModelArticulation()->GetRoot()->m_body->GetMatrix());
			GetModel()->GetAsModelArticulation()->SetTransform(matrix);
		}
		
		ndBrainFloat IsOnAir() const
		{
			ndBodyKinematic::ndContactMap& contacts = m_wheel->GetContactMap();
			ndBodyKinematic::ndContactMap::Iterator it(contacts);
			for (it.Begin(); it; it++)
			{
				ndContact* const contact = *it;
				if (contact->IsActive())
				{
					return ndBrainFloat(0.0f);
				}
			}
			return ndBrainFloat(1.0f);
		};

		//#pragma optimize( "", off )
		ndBrainFloat CalculateReward()
		{
			if (IsTerminal())
			{
				return ndBrainFloat(-1.0f);
			}

			auto CalculateReferenceFrame = [this]()
			{
				ndVector com(ndVector::m_zero);
				ndFloat32 totalMass = ndFloat32(0.0f);
				for (ndInt32 i = m_bodies.GetCount() - 1; i >= 0; --i)
				{
					const ndBodyKinematic* const body = m_bodies[i];
					const ndMatrix matrix(body->GetMatrix());
					const ndVector bodyCom(matrix.TransformVector(body->GetCentreOfMass()));
					ndFloat32 mass = body->GetMassMatrix().m_w;
					totalMass += mass;
					com += bodyCom.Scale(mass);
				}

				com = com.Scale(1.0f / totalMass);
				ndMatrix referenceFrame(m_wheelJoint->CalculateGlobalMatrix1());
				referenceFrame.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
				referenceFrame.m_right = referenceFrame.m_front.CrossProduct(referenceFrame.m_up).Normalize();
				referenceFrame.m_up = referenceFrame.m_right.CrossProduct(referenceFrame.m_front).Normalize();
				referenceFrame.m_posit = com;
				referenceFrame.m_posit.m_w = 1.0f;
				return referenceFrame;
			};
			const ndMatrix comFrame(CalculateReferenceFrame());

			auto PolynomialOmegaReward = [](ndFloat32 omega)
			{
				ndFloat32 maxOmega = 2.0f;
				omega = ndClamp(omega, -maxOmega, maxOmega);
				ndFloat32 r = ndFloat32(1.0f) - ndAbs(omega) / maxOmega;
				ndFloat32 reward = r * r * r * r;
				return reward;
			};

			auto PolynomialAccelerationReward = [](ndFloat32 alpha)
			{
				ndFloat32 maxAlpha = 5.0f;
				alpha = ndClamp(alpha, -maxAlpha, maxAlpha);
				ndFloat32 r = ndFloat32(1.0f) - ndAbs(alpha) / maxAlpha;
				ndFloat32 reward = r * r;
				return reward;
			};

			ndIkSolver& solver = m_controllerTrainer->m_solver;
			ndFixSizeArray<ndJointBilateralConstraint*, 64> extraJoint;
			const ndModelArticulation::ndCenterOfMassDynamics comDynamics(GetModel()->GetAsModelArticulation()->CalculateCentreOfMassDynamics(solver, comFrame, extraJoint, m_timestep));
			const ndVector comOmega(comDynamics.m_omega);
			const ndVector comAlpha(comDynamics.m_alpha);

			ndVector veloc(m_box->GetVelocity());
			ndFloat32 speedReward = ndExp(-100.0f * veloc.m_x * veloc.m_x);

			ndFloat32 omegaReward = PolynomialOmegaReward(comOmega.m_x);
			ndFloat32 alphaReward = PolynomialAccelerationReward(comAlpha.m_x);
			ndFloat32 reward = ndFloat32(0.2f) * speedReward +  ndFloat32(0.4f) * omegaReward + ndFloat32(0.4f) * alphaReward;

			if (IsOnAir())
			{
				// penalize air borne high angular velocity
				const ndMatrix wheelMatrix(m_wheelJoint->CalculateGlobalMatrix0());
				const ndVector wheelOmega(m_wheel->GetOmega());
				ndFloat32 speed = (wheelOmega.DotProduct(wheelMatrix.m_front)).GetScalar();

				ndFloat32 arg = -0.5f * speed * speed;
				reward = ndExp(arg);
			}
			return ndBrainFloat(reward);
		}

		//#pragma optimize( "", off )
		void ApplyActions(ndBrainFloat* const actions)
		{
			const ndVector wheelMass(m_wheel->GetMassMatrix());
			const ndMatrix wheelMatrix(m_wheelJoint->CalculateGlobalMatrix0());

			const ndVector wheelOmega(m_wheel->GetOmega());
			ndFloat32 speed = (wheelOmega.DotProduct(wheelMatrix.m_front)).GetScalar();

			ndFloat32 drag = ndFloat32(0.25f) * speed * speed * ndSign(speed);
			ndFloat32 wheelTorque = wheelMass.m_z * actions[m_wheelTorque] * ND_MAX_WHEEL_ALPHA;

			//ndExpandTraceMessage("%g %g %g\n", speed, drag, wheelTorque);

			ndVector torque(wheelMatrix.m_front.Scale(wheelTorque - drag));
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
			wheelOmega *= 1.0f / 20.0f;

			ndVector veloc(m_box->GetVelocity());
			ndBrainFloat speed = ndClamp(ndBrainFloat(veloc.m_x), ndBrainFloat(-6.0f), ndBrainFloat(6.0f)) / 6.0f;

			observation[m_velocity] = speed;
			observation[m_poleAngle] = ndBrainFloat(poleAngle);
			observation[m_poleOmega] = ndBrainFloat(poleOmega);
			observation[m_wheelOmega] = ndBrainFloat(wheelOmega);
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
			return ndAbs(body->GetMatrix().m_posit.m_x) > ndFloat32(20.0f);
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
		ndBodyKinematic* m_box;
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
			,m_lastEpisode(0xffffffff)
			,m_modelIsTrained(false)
		{
			ndWorld* const world = scene->GetWorld();
			
			m_outFile = fopen("unicycle_sac.csv", "wb");
			fprintf(m_outFile, "sac\n");
			m_stopTraining = 200000;

			ndBrainAgentOffPolicyGradient_Trainer::HyperParameters hyperParameters;
			hyperParameters.m_numberOfActions = m_actionsSize;
			hyperParameters.m_numberOfObservations = m_observationsSize;
			m_master = ndSharedPtr<ndBrainAgentOffPolicyGradient_Trainer>(new ndBrainAgentOffPolicyGradient_Trainer(hyperParameters));

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

				ndFloat32 trajectoryLog = ndLog(m_master->GetAverageFrames() + 0.001f);
				ndFloat32 rewardTrajectory = m_master->GetAverageScore() * trajectoryLog;

				if (!m_master->IsSampling())
				{
					if (rewardTrajectory >= ndFloat32(m_maxScore))
					{
						if (m_lastEpisode != m_master->GetEposideCount())
						{
							m_maxScore = rewardTrajectory;
							m_bestActor->CopyFrom(*m_master->GetPolicyNetwork());
							ndExpandTraceMessage("   best actor episode: %d\treward %f\ttrajectoryFrames: %f\n", m_master->GetEposideCount(), m_master->GetAverageScore(), m_master->GetAverageFrames());
							m_lastEpisode = m_master->GetEposideCount();
						}
					}

					if (episodeCount != m_master->GetEposideCount())
					{
						ndExpandTraceMessage("steps: %d\treward: %g\t  trajectoryFrames: %g\n", m_master->GetFramesCount(), m_master->GetAverageScore(), m_master->GetAverageFrames());
						if (m_outFile)
						{
							fprintf(m_outFile, "%g\n", m_master->GetAverageScore());
							fflush(m_outFile);
						}
					}
				}
			}

			if ((stopTraining >= m_stopTraining) || (m_master->GetAverageScore() > 0.95f))
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

		ndSharedPtr<ndBrainAgentOffPolicyGradient_Trainer> m_master;
		ndSharedPtr<ndBrain> m_bestActor;
		ndList<ndModelArticulation*> m_models;
		FILE* m_outFile;
		ndUnsigned64 m_timer;
		ndFloat32 m_maxScore;
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
	matrix.m_posit.m_y = 2.5f;

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
