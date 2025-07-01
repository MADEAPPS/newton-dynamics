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


// traning models using Q value methods: double deterministic DDRP or Soft Actor Cristic SAC.

// This model attempts to take animation poses and use a reward system to generate a policy  
// that produces the animation.  
// If this phase is successful, we will adapt the reward so that the robot can adjust  
// to the environment with increasing complexity
namespace ndQuadruped_3
{
	#define ND_TRAIN_MODEL
	#define CONTROLLER_NAME "ndQuadruped_3-ddpg.dnn"

	enum ndActionSpace
	{
		m_leg0_x,
		m_leg0_y,
		m_leg0_z,

		//m_leg1_x,
		//m_leg1_y,
		//m_leg1_z,
		//
		//m_leg2_x,
		//m_leg2_y,
		//m_leg2_z,
		//
		//m_leg3_x,
		//m_leg3_y,
		//m_leg3_z,

		m_actionsSize
	};

	enum ndStateSpace
	{
		m_leg0_posit_x,
		m_leg0_posit_y,
		m_leg0_posit_z,

		//m_leg1_posit_x,
		//m_leg1_posit_y,
		//m_leg1_posit_z,
		//
		//m_leg2_posit_x,
		//m_leg2_posit_y,
		//m_leg2_posit_z,
		//
		//m_leg3_posit_x,
		//m_leg3_posit_y,
		//m_leg3_posit_z,

		//m_leg0_veloc_x,
		//m_leg0_veloc_y,
		//m_leg0_veloc_z,
		//
		//m_leg1veloc_x,
		//m_leg1veloc_y,
		//m_leg1veloc_z,
		//
		//m_leg2_veloc_x,
		//m_leg2_veloc_y,
		//m_leg2_veloc_z,
		//
		//m_leg3_veloc_x,
		//m_leg3_veloc_y,
		//m_leg3_veloc_z,

		//m_frameTick,
		m_observationsSize
	};

	#define D_CYCLE_PERIOD			ndFloat32(4.0f)
	#define D_CYCLE_STRIDE_X		ndFloat32(0.3f)
	#define D_CYCLE_STRIDE_Z		ndFloat32(0.3f)
	#define D_CYCLE_AMPLITUDE		ndFloat32(0.27f)
	#define D_POSE_REST_POSITION_Y	ndReal(-0.3f)

	#define D_ACTION_SENSITIVITY			ndReal(0.05f)

	class RobotModelNotify : public ndModelNotify
	{
		class ndController : public ndBrainAgentContinuePolicyGradient
		{
			public:
			ndController(const ndSharedPtr<ndBrain>& brain, RobotModelNotify* const robot)
				:ndBrainAgentContinuePolicyGradient(brain)
				,m_robot(robot)
			{
			}

			ndController(const ndController& src)
				:ndBrainAgentContinuePolicyGradient(src.m_policy)
				,m_robot(src.m_robot)
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

		class ndControllerAgent : public ndBrainAgentDeterministicPolicyGradient_Agent
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
					const ndMatrix matrix(ndCalculateMatrix(m_rotation, m_posit));
					m_body->SetMatrix(matrix);
					m_body->SetOmega(m_omega);
					m_body->SetVelocity(m_veloc);
				}

				ndVector m_veloc;
				ndVector m_omega;
				ndVector m_posit;
				ndQuaternion m_rotation;
				ndBodyDynamic* m_body;
			};

			class ndPoseGenerator : public ndAnimationSequence
			{
				public:
				ndPoseGenerator()
					:ndAnimationSequence()
					,m_amp(D_CYCLE_AMPLITUDE)
					,m_stride_x(D_CYCLE_STRIDE_X)
					,m_stride_z(D_CYCLE_STRIDE_Z)
				{
					m_duration = D_CYCLE_PERIOD;

					m_poseBoundMin.m_x = - m_stride_x * 0.5f;
					m_poseBoundMin.m_y = - m_amp * 0.0f;
					m_poseBoundMin.m_z = - m_stride_z * 0.5f;
					m_poseBoundMin.m_w = 1.0f;

					m_poseBoundMax.m_x = m_stride_x * 0.5f;
					m_poseBoundMax.m_y = m_amp * 1.0f;
					m_poseBoundMax.m_z = m_stride_z * 0.5f;
					m_poseBoundMax.m_w = 1.0f;
				}

				ndVector GetTranslation(ndFloat32) const override
				{
					return ndVector::m_zero;
				}

				void CalculatePose(ndAnimationPose& output, ndFloat32 param) override
				{
					// generate a procedural in place march gait
					ndAssert(param >= ndFloat32(0.0f));
					ndAssert(param <= ndFloat32(1.0f));

					for (ndInt32 i = 0; i < output.GetCount(); i++)
					{
						const ndEffectorInfo& leg = *(ndEffectorInfo*)output[i].m_userData;;
						output[i].m_userParamFloat = 0.0f;
						output[i].m_posit = leg.m_effector->GetRestPosit();
					}

					output[0].m_posit.m_y += 0.25f;

					//param = 0.0f;
					//ndFloat32 gaitFraction = 0.25f;
					//ndFloat32 gaitGuard = gaitFraction * 0.25f;
					//ndFloat32 omega = ndPi / (gaitFraction - gaitGuard);
					//for (ndInt32 i = 0; i < output.GetCount(); i++)
					//{
					//	const ndEffectorInfo& leg = *(ndEffectorInfo*)output[i].m_userData;;
					//	const ndVector localPosit(leg.m_effector->GetRestPosit());
					//	ndFloat32 stride_x = m_stride_x;
					//	//ndFloat32 stride_z = m_stride_z;
					//	ndFloat32 phase = 0.0f;
					//	if (localPosit.m_x > 0.0f)
					//	{
					//		phase = (localPosit.m_z > 0.0f) ? 0.0f : 0.50f;
					//	}
					//	else
					//	{
					//		phase = (localPosit.m_z > 0.0f) ? 0.75f : 0.25f;
					//	}
					//	
					//	//stride_x = 0.0f;
					//	//stride_z = 0.0f;
					//	
					//	ndFloat32 t = ndMod(param - phase + ndFloat32(1.0f), ndFloat32(1.0f));
					//	if ((t >= gaitGuard) && (t <= gaitFraction))
					//	{
					//		output[i].m_posit.m_y += m_amp * ndSin(omega * (t - gaitGuard));
					//		output[i].m_userParamFloat = 1.0f;
					//	
					//		ndFloat32 num = t - gaitGuard;
					//		ndFloat32 den = gaitFraction - gaitGuard;
					//	
					//		ndFloat32 t0 = num / den;
					//		output[i].m_posit.m_x += stride_x * t0 - stride_x * 0.5f;
					//		//output[i].m_posit.m_z += -(stride_z * t0 - stride_z * 0.5f);
					//	}
					//	else
					//	{
					//		if (t <= gaitGuard)
					//		{
					//			t += 1.0f;
					//			output[i].m_userParamFloat = 0.5f;
					//		}
					//	
					//		ndFloat32 num = t - gaitFraction;
					//		ndFloat32 den = 1.0f - (gaitFraction - gaitGuard);
					//		ndFloat32 t0 = num / den;
					//		output[i].m_posit.m_x += -(stride_x * t0 - stride_x * 0.5f);
					//		//output[i].m_posit.m_z += (stride_z * t0 - stride_z * 0.5f);
					//	}
					//	//m_currentPose[i] = output[i].m_posit;
					//}
				}

				ndVector m_poseBoundMin;
				ndVector m_poseBoundMax;
				ndFloat32 m_amp;
				ndFloat32 m_stride_x;
				ndFloat32 m_stride_z;
			};

			ndControllerAgent(const ndSharedPtr<ndBrainAgentDeterministicPolicyGradient_Trainer>& master, RobotModelNotify* const robot)
				:ndBrainAgentDeterministicPolicyGradient_Agent(master)
				,m_robot(robot)
				,m_animPose()
				,m_poseGenerator()
				,m_animBlendTree()
				,m_basePose()
				,m_time(0.0f)
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
				//ndUnsigned32 start = ndRandInt() & 3;
				ndFloat32 duration = ((ndAnimationSequencePlayer*)*m_poseGenerator)->GetSequence();
				//m_animBlendTree->SetTime(duration * ndFloat32(start));
				//m_animBlendTree->SetTime(duration * ndFloat32(start));

				m_animBlendTree->SetTime(0.0f);
				m_time = ndFmod (ndRand(), 1.0f);
			m_time = 0.0f;
				ndFloat32 animFrame = m_time * duration;
				//m_animBlendTree->Update(time);
				m_animBlendTree->SetTime(animFrame);
				m_robot->m_animFrame = animFrame;

				ndVector veloc;
				m_animBlendTree->Evaluate(m_animPose, veloc);
			}

			void InitAnimation()
			{
				ndSharedPtr<ndAnimationSequence> sequence(new ndPoseGenerator());

				m_poseGenerator = ndSharedPtr<ndAnimationBlendTreeNode>(new ndAnimationSequencePlayer(sequence));
				//m_control = new ndUIControlNode(m_poseGenerator);
				//m_animBlendTree = ndSharedPtr<ndAnimationBlendTreeNode>(m_control);
				m_animBlendTree = ndSharedPtr<ndAnimationBlendTreeNode>(m_poseGenerator);

				//ndFloat32 duration = ((ndAnimationSequencePlayer*)*m_poseGenerator)->GetSequence()->GetDuration();
				//m_animBlendTree->SetTime(duration * ndRand());
				m_animBlendTree->SetTime(0.0f);

				ndFloat32 offset_x[] = { 0.2f, 0.2f, 0.2f, 0.2f };
				ndFloat32 offset_z[] = { -0.3f, 0.3f, -0.3f, 0.3f };
				ndFloat32 offset_y[] = { D_POSE_REST_POSITION_Y, D_POSE_REST_POSITION_Y, D_POSE_REST_POSITION_Y, D_POSE_REST_POSITION_Y };

				ndPoseGenerator* const poseGenerator = (ndPoseGenerator*)*sequence;
				for (ndInt32 i = 0; i < m_robot->m_legs.GetCount(); ++i)
				{
					ndEffectorInfo& leg = m_robot->m_legs[i];
					ndAnimKeyframe keyFrame;
					keyFrame.m_userData = &leg;
					m_animPose.PushBack(keyFrame);
					poseGenerator->AddTrack();
				}

				ndModelArticulation* const robot = m_robot->GetModel()->GetAsModelArticulation();
				for (ndModelArticulation::ndNode* node = robot->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
				{
					m_basePose.PushBack(node->m_body->GetAsBodyDynamic());
				}

				ResetModel();
			}

			//ndFloat32 GetPoseSequence() const
			//{
			//	ndAnimationSequencePlayer* const poseGenerator = (ndAnimationSequencePlayer*)*m_poseGenerator;
			//	ndFloat32 seq = poseGenerator->GetTime() / D_CYCLE_PERIOD;
			//	//ndTrace(("%f seq\n", seq));
			//	return seq;
			//}

			RobotModelNotify* m_robot;
			ndAnimationPose m_animPose;
			ndSharedPtr<ndAnimationBlendTreeNode> m_poseGenerator;
			ndSharedPtr<ndAnimationBlendTreeNode> m_animBlendTree;
			ndFixSizeArray<ndBasePose, 32> m_basePose;
			ndFloat32 m_time;
		};

		public:
		class ndEffectorInfo
		{
			public:
			ndEffectorInfo()
				:m_calf(nullptr)
				,m_heel(nullptr)
				,m_thigh(nullptr)
				,m_effector(nullptr)
			{
			}

			ndEffectorInfo(
				ndJointSpherical* const thigh,
				ndJointHinge* const calf,
				ndJointHinge* const foot,
				ndIkSwivelPositionEffector* const effector)
				:m_calf(calf)
				,m_heel(foot)
				,m_thigh(thigh)
				,m_effector(effector)
			{
			}

			ndJointHinge* m_calf;
			ndJointHinge* m_heel;
			ndJointSpherical* m_thigh;
			ndIkSwivelPositionEffector* m_effector;
		};

		RobotModelNotify(ndModelArticulation* const robot)
			:ndModelNotify()
			,m_controller(nullptr)
			,m_controllerTrainer(nullptr)
			,m_timestep(ndFloat32(0.0f))
			,m_animFrame(ndFloat32(0.0f))
		{
			SetModel(robot);
		}

		~RobotModelNotify()
		{
		}

		void SetControllerTrainer(const ndSharedPtr<ndBrainAgentDeterministicPolicyGradient_Trainer>& master)
		{
			m_controllerTrainer = ndSharedPtr<ndControllerAgent>(new ndControllerAgent(master, this));
			m_controllerTrainer->InitAnimation();
		}

		void SetController(const ndSharedPtr<ndBrain>& policy)
		{
			RobotModelNotify* const robot = (RobotModelNotify*)*GetModel()->GetNotifyCallback();
			m_controller = ndSharedPtr<ndController> (new ndController(policy, robot));
		}

		ndVector CalculatePositError(ndInt32 keyFrameIndex) const
		{
			ndJointBilateralConstraint::ndKinematicState kinematicState[4];
			const ndAnimKeyframe keyFrame = m_controllerTrainer->m_animPose[keyFrameIndex];
			const ndEffectorInfo* const leg = (ndEffectorInfo*)keyFrame.m_userData;
			
			leg->m_effector->GetKinematicState(kinematicState);
			const ndVector effectPosit(kinematicState[0].m_posit, kinematicState[1].m_posit, kinematicState[2].m_posit, ndFloat32(0.0f));
			const ndVector animPosit(keyFrame.m_posit);
			const ndVector error(animPosit - effectPosit);
			return error;
		}

		//#pragma optimize( "", off )
		bool IsTerminal() const
		{
			// termination for execive velocity
			for (ndInt32 i = 0; i < m_controllerTrainer->m_basePose.GetCount(); i++)
			{
				const ndBodyKinematic* const body = m_controllerTrainer->m_basePose[i].m_body;
				const ndVector omega(body->GetOmega());
				const ndVector veloc(body->GetVelocity());
				if (veloc.DotProduct(veloc).GetScalar() > 1000.0f)
				{
					return true;
				}
				if (omega.DotProduct(omega).GetScalar() > 2000.0f)
				{
					return true;
				}
			}

			ndSharedPtr<ndAnimationBlendTreeNode> node = m_controllerTrainer->m_poseGenerator;
			ndAnimationSequencePlayer* const sequencePlayer = (ndAnimationSequencePlayer*)*node;
			ndControllerAgent::ndPoseGenerator* const poseGenerator = (ndControllerAgent::ndPoseGenerator*)*sequencePlayer->GetSequence();
			ndVector normalize(poseGenerator->m_poseBoundMax.Reciproc());
			for (ndInt32 i = 0; i < m_controllerTrainer->m_animPose.GetCount(); ++i)
			{
				const ndVector error(CalculatePositError(i));
				const ndVector normalError(error * normalize);
				if (ndAbs(normalError.m_x) > 1.0f)
				{
					return true;
				}

				if (ndAbs(normalError.m_y) > 1.0f)
				{
					return true;
				}

				if (ndAbs(normalError.m_z) > 1.0f)
				{
					return true;
				}
			}

			return false;
		}

		//#pragma optimize( "", off )
		ndBrainFloat CalculateReward() const
		{
			if (IsTerminal())
			{
				return ndBrainFloat(-1.0f);
			}

			ndFloat32 reward = 0.0f;
			for (ndInt32 i = 0; i < m_controllerTrainer->m_animPose.GetCount(); ++i)
			{
				const ndVector error(CalculatePositError(i));
				for (ndInt32 j = 0; j < 3; ++j)
				{
					ndFloat32 error2 = error[j] * error[j];
					ndFloat32 legReward = ndExp(-200000.0f * error2);
					reward += legReward;
				}
				//break;
			}
			//return ndBrainFloat(reward / 12.0f);
			return ndBrainFloat(reward / 3.0f);
		}

		//#pragma optimize( "", off )
		void ResetModel()
		{
			ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
			for (ndInt32 i = 0; i < m_controllerTrainer->m_basePose.GetCount(); i++)
			{
				m_controllerTrainer->m_basePose[i].SetPose();
			}

			model->ClearMemory();
		}

		//#pragma optimize( "", off )
		void GetObservation(ndBrainFloat* const observations)
		{
			//ndInt32 size = m_leg1_posit_x - m_leg0_posit_x;
			//observations[m_frameTick] = ndBrainFloat(m_animFrame);
			//for (ndInt32 i = 0; i < m_legs.GetCount(); ++i)
			for (ndInt32 i = 0; i < 1; ++i)
			{
				ndEffectorInfo& leg = m_legs[i];
				ndJointBilateralConstraint::ndKinematicState kinematicState[4];
				leg.m_effector->GetKinematicState(kinematicState);
				const ndVector restPosit(leg.m_effector->GetRestPosit());
				const ndVector effectorPosit(kinematicState[0].m_posit, kinematicState[1].m_posit, kinematicState[2].m_posit, 0.0f);
				const ndVector effectorRelPosit(effectorPosit - restPosit);
			
				//observations[i * size + m_leg0_posit_x] = effectorRelPosit.m_x;
				//observations[i * size + m_leg0_posit_y] = effectorRelPosit.m_y;
				//observations[i * size + m_leg0_posit_z] = effectorRelPosit.m_z;
				observations[m_leg0_posit_x] = ndBrainFloat(effectorRelPosit.m_x);
				observations[m_leg0_posit_y] = ndBrainFloat(effectorRelPosit.m_y);
				observations[m_leg0_posit_z] = ndBrainFloat(effectorRelPosit.m_z);
			}
		}

		void ApplyActions(ndBrainFloat* const actions)
		{
			ndBodyKinematic* const rootBody = GetModel()->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
			const ndVector upVector(rootBody->GetMatrix().m_up);

			//ndInt32 size = m_leg1_x - m_leg0_x;
			for (ndInt32 i = 0; i < m_legs.GetCount(); ++i)
			//for (ndInt32 i = 0; i < 1; ++i)
			{
				ndEffectorInfo& leg = m_legs[i];
				ndIkSwivelPositionEffector* const effector = leg.m_effector;
				
				const ndVector resPosit(leg.m_effector->GetRestPosit());
				const ndVector effectorPosit(leg.m_effector->GetEffectorPosit());
				ndVector relativePosit(effectorPosit - resPosit);
			
				//relativePosit.m_x += actions[size * i + m_leg0_x] * D_ACTION_SENSITIVITY;
				//relativePosit.m_y += actions[size * i + m_leg0_y] * D_ACTION_SENSITIVITY;
				//relativePosit.m_z += actions[size * i + m_leg0_z] * D_ACTION_SENSITIVITY;

				if (i == 0)
				{
					relativePosit.m_x += actions[3 * i + m_leg0_x] * D_ACTION_SENSITIVITY;
					relativePosit.m_y += actions[3 * i + m_leg0_y] * D_ACTION_SENSITIVITY;
					relativePosit.m_z += actions[3 * i + m_leg0_z] * D_ACTION_SENSITIVITY;
					const ndVector newPosit(relativePosit + resPosit);
					effector->SetLocalTargetPosition(newPosit);
				}
			
				ndFloat32 swivelAngle = effector->CalculateLookAtSwivelAngle(upVector);
				effector->SetSwivelAngle(swivelAngle);
			
				// calculate lookAt angle
				ndMatrix lookAtMatrix0;
				ndMatrix lookAtMatrix1;
				ndJointHinge* const heelHinge = leg.m_heel;
				heelHinge->CalculateGlobalMatrix(lookAtMatrix0, lookAtMatrix1);
				
				ndMatrix upMatrix(ndGetIdentityMatrix());
				upMatrix.m_front = lookAtMatrix0.m_front;
				upMatrix.m_right = (upVector.CrossProduct(upMatrix.m_front) & ndVector::m_triplexMask).Normalize();
				upMatrix.m_up = upMatrix.m_right.CrossProduct(upMatrix.m_front);
				upMatrix = upMatrix * lookAtMatrix0.OrthoInverse();
				const ndFloat32 angle = ndAtan2(upMatrix.m_up.m_z, upMatrix.m_up.m_y);
				heelHinge->SetTargetAngle(angle);
			}
		}

		void PostUpdate(ndFloat32)
		{
		}

		void PostTransformUpdate(ndFloat32)
		{
		}

		virtual void Debug(ndConstraintDebugCallback& context) const
		{
			for (ndInt32 i = 0; i < m_legs.GetCount(); ++i)
			{
				const ndEffectorInfo& leg = m_legs[i];
				leg.m_heel->DebugJoint(context);
				leg.m_effector->DebugJoint(context);
			}
		}

		void Update(ndFloat32 timestep)
		{
			ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
			ndBodyKinematic* const rootBody = model->GetRoot()->m_body->GetAsBodyKinematic();
			rootBody->SetSleepState(false);

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
		
		ndFixSizeArray<ndEffectorInfo, 4> m_legs;
		ndSharedPtr<ndController> m_controller;
		ndSharedPtr<ndControllerAgent> m_controllerTrainer;
		ndFloat32 m_timestep;
		ndFloat32 m_animFrame;
	};

	ndModelArticulation* CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, const ndSharedPtr<ndDemoEntity>& modelMesh)
	{
		ndModelArticulation* const model = new ndModelArticulation();
		RobotModelNotify* const notify = new RobotModelNotify(model);
		model->SetNotifyCallback(notify);

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

		ndFloat32 mass = 20.0f;
		ndFloat32 limbMass = 0.25f;
		ndMatrix matrix(entity->GetCurrentMatrix() * location);

		ndSharedPtr<ndBody> rootBody(CreateRigidBody(entity, matrix, mass, nullptr));
		ndModelArticulation::ndNode* const modelRootNode = model->AddRootBody(rootBody);

		// build all for legs
		//ndInt32 index = 0;
		for (ndList<ndSharedPtr<ndDemoEntity>>::ndNode* node = entity->GetChildren().GetFirst(); node; node = node->GetNext())
		{
			// build thig
			//index++;
			//if (!((index == 4) || (index == 1)))
			//if (!(index == 4))
			//	continue;

			ndSharedPtr<ndDemoEntity> thighEntity(node->GetInfo());
			const ndMatrix thighMatrix(thighEntity->GetCurrentMatrix() * matrix);
			ndSharedPtr<ndBody> thigh(CreateRigidBody(thighEntity, thighMatrix, limbMass, rootBody->GetAsBodyDynamic()));

			ndSharedPtr<ndJointBilateralConstraint> ballJoint(new ndJointSpherical(thighMatrix, thigh->GetAsBodyKinematic(), rootBody->GetAsBodyKinematic()));
			ndModelArticulation::ndNode* const thighNode = model->AddLimb(modelRootNode, thigh, ballJoint);

			// build calf
			ndSharedPtr<ndDemoEntity> calfEntity(thighEntity->GetChildren().GetFirst()->GetInfo());
			const ndMatrix calfMatrix(calfEntity->GetCurrentMatrix() * thighMatrix);
			ndSharedPtr<ndBody> calf(CreateRigidBody(calfEntity, calfMatrix, limbMass, thigh->GetAsBodyDynamic()));

			ndSharedPtr<ndJointBilateralConstraint> calfHinge(new ndJointHinge(calfMatrix, calf->GetAsBodyKinematic(), thigh->GetAsBodyKinematic()));
			ndModelArticulation::ndNode* const calfNode = model->AddLimb(thighNode, calf, calfHinge);

			((ndIkJointHinge*)*calfHinge)->SetLimitState(true);
			((ndIkJointHinge*)*calfHinge)->SetLimits(-70.0f * ndDegreeToRad, 10.0f * ndDegreeToRad);

			// build heel
			ndSharedPtr<ndDemoEntity> heelEntity(calfEntity->GetChildren().GetFirst()->GetInfo());
			const ndMatrix heelMatrix(heelEntity->GetCurrentMatrix() * calfMatrix);
			ndSharedPtr<ndBody> heel(CreateRigidBody(heelEntity, heelMatrix, limbMass, calf->GetAsBodyDynamic()));

			ndSharedPtr<ndJointBilateralConstraint> heelHinge(new ndJointHinge(heelMatrix, heel->GetAsBodyKinematic(), calf->GetAsBodyKinematic()));
			//ndModelArticulation::ndNode* const heelNode = model->AddLimb(calfNode, heel, heelHinge);
			model->AddLimb(calfNode, heel, heelHinge);
			((ndJointHinge*)*heelHinge)->SetAsSpringDamper(0.001f, 2000.0f, 50.0f);

			// create effector
			ndSharedPtr<ndDemoEntity> footEntity(heelEntity->GetChildren().GetFirst()->GetInfo());
			ndMatrix footMatrix(matrix);
			footMatrix.m_posit = (footEntity->GetCurrentMatrix() * heelMatrix).m_posit;

			ndMatrix effectorRefFrame(footMatrix);
			effectorRefFrame.m_posit = thighMatrix.m_posit;

			ndFloat32 regularizer = 0.001f;
			ndFloat32 effectorStrength = 50000.0f;
			ndSharedPtr<ndJointBilateralConstraint> effector (new ndIkSwivelPositionEffector(effectorRefFrame, rootBody->GetAsBodyKinematic(), footMatrix.m_posit, heel->GetAsBodyKinematic()));

			ndFloat32 minWorkScape;
			ndFloat32 maxWorkScape;
			((ndIkSwivelPositionEffector*)*effector)->GetWorkSpaceConstraints(minWorkScape, maxWorkScape);
			((ndIkSwivelPositionEffector*)*effector)->SetWorkSpaceConstraints(minWorkScape + 0.1f, maxWorkScape * 1.1f);
			((ndIkSwivelPositionEffector*)*effector)->SetLinearSpringDamper(regularizer, 4000.0f, 50.0f);
			((ndIkSwivelPositionEffector*)*effector)->SetAngularSpringDamper(regularizer, 4000.0f, 50.0f);
			((ndIkSwivelPositionEffector*)*effector)->SetWorkSpaceConstraints(0.0f, 0.75f * 0.9f);
			((ndIkSwivelPositionEffector*)*effector)->SetMaxForce(effectorStrength);
			((ndIkSwivelPositionEffector*)*effector)->SetMaxTorque(effectorStrength);
			
			model->AddCloseLoop(effector);

			RobotModelNotify::ndEffectorInfo leg;
			leg.m_calf = (ndJointHinge*)*calfHinge;
			leg.m_heel = (ndJointHinge*)*heelHinge;
			leg.m_thigh = (ndJointSpherical*)*ballJoint;
			leg.m_effector = (ndIkSwivelPositionEffector*)*effector;
			notify->m_legs.PushBack(leg);
		}
		return model;
	}

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
			,m_stopTraining(500000)
			,m_modelIsTrained(false)
		{
			ndWorld* const world = scene->GetWorld();

			m_outFile = fopen("ndQuadruped_3-vpg.csv", "wb");
			fprintf(m_outFile, "vpg\n");

			ndBrainAgentDeterministicPolicyGradient_Trainer::HyperParameters hyperParameters;

			hyperParameters.m_numberOfActions = m_actionsSize;
			hyperParameters.m_numberOfObservations = m_observationsSize;
			//hyperParameters.m_discountRewardFactor = ndReal(m_discountRewardFactor);

			m_master = ndSharedPtr<ndBrainAgentDeterministicPolicyGradient_Trainer>(new ndBrainAgentDeterministicPolicyGradient_Trainer(hyperParameters));
			m_bestActor = ndSharedPtr<ndBrain>(new ndBrain(*m_master->GetPolicyNetwork()));
			m_master->SetName(CONTROLLER_NAME);

			ndSharedPtr<ndModel>visualModel(CreateModel(scene, matrix, modelMesh));
			RobotModelNotify* const notify = (RobotModelNotify*)*visualModel->GetAsModel()->GetNotifyCallback();
			notify->SetControllerTrainer(m_master);
			
			SetMaterial(visualModel->GetAsModelArticulation());
			world->AddModel(visualModel);
			visualModel->AddBodiesAndJointsToWorld();
			
			ndBodyKinematic* const rootBody = visualModel->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
			ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(rootBody->GetMatrix(), rootBody, world->GetSentinelBody()));
			world->AddJoint(fixJoint);
			
			//// add a hidden battery of model to generate trajectories in parallel
			//ndInt32 countX = 10;
			//ndInt32 countZ = 10;
			////countX = 0;
			////countZ = 0;
			//
			//// add a hidden battery of model to generate trajectories in parallel
			//for (ndInt32 i = 0; i < countZ; ++i)
			//{
			//	for (ndInt32 j = 0; j < countX; ++j)
			//	{
			//		ndMatrix location(matrix);
			//		location.m_posit.m_x += 20.0f * (ndRand() - 0.5f);
			//		location.m_posit.m_z += 20.0f * (ndRand() - 0.5f);
			//
			//		ndFloat32 step = 20.0f * (ndRand() - 0.5f);
			//		location.m_posit.m_x += step;
			//
			//		//ndSharedPtr<ndModel>model(CreateModel(scene, location, modelMesh, m_master));
			//		ndSharedPtr<ndModel>model(CreateModel(scene, location, modelMesh));
			//		RobotModelNotify* const notify1 = (RobotModelNotify*)*model->GetAsModel()->GetNotifyCallback();
			//		notify1->SetControllerTrainer(m_master);
			//
			//		SetMaterial(model->GetAsModelArticulation());
			//		world->AddModel(model);
			//		model->AddBodiesAndJointsToWorld();
			//
			//		m_models.Append(model->GetAsModelArticulation());
			//
			//		ndBodyKinematic* const rootBody1 = model->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
			//		ndSharedPtr<ndJointBilateralConstraint> fixJoint1(new ndJointFix6dof(rootBody1->GetMatrix(), rootBody1, world->GetSentinelBody()));
			//		world->AddJoint(fixJoint1);
			//
			//	}
			//}

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

		//#pragma optimize( "", off )
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
			
				//if (episodeCount && !m_master->IsSampling())
				if (episodeCount)
				{
					ndExpandTraceMessage("steps: %d\treward: %g\t  trajectoryFrames: %g\n", m_master->GetFramesCount(), 100.0f * m_master->GetAverageScore() / m_horizon, m_master->GetAverageFrames());
					if (m_outFile)
					{
						fprintf(m_outFile, "%g\n", m_master->GetAverageScore());
						fflush(m_outFile);
					}
				}
			}
			
			//if ((stopTraining >= m_stopTraining) || (100.0f * m_master->GetAverageScore() / m_horizon > 95.0f))
			if (stopTraining >= m_stopTraining)
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

		ndSharedPtr<ndBrainAgentDeterministicPolicyGradient_Trainer> m_master;
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

using namespace ndQuadruped_3;

void ndQuadrupedTest_3(ndDemoEntityManager* const scene)
{
	// build a floor
	ndSetRandSeed(94157);

	BuildFloorBox(scene, ndGetIdentityMatrix());
	//BuildFlatPlane(scene, true);

	// register a zero restitution and high friction material for the feet
	ndApplicationMaterial material;
	material.m_restitution = 0.0f;
	material.m_staticFriction0 = 0.8f;
	material.m_staticFriction1 = 0.8f;
	material.m_dynamicFriction0 = 0.8f;
	material.m_dynamicFriction1 = 0.8f;
	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
	callback->RegisterMaterial(material, ndDemoContactCallback::m_frictionTest, ndDemoContactCallback::m_default);

	ndMeshLoader loader;
	ndSharedPtr<ndDemoEntity> modelMesh(loader.LoadEntity("quadrupeSpider.fbx", scene));

	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit.m_y = 0.5f;

	#ifdef ND_TRAIN_MODEL
		scene->RegisterPostUpdate(new TrainingUpdata(scene, matrix, modelMesh));
	#else
		ndWorld* const world = scene->GetWorld();

		char fileName[256];
		ndGetWorkingFileName(CONTROLLER_NAME, fileName);
		ndSharedPtr<ndBrain> policy(ndBrainLoad::Load(fileName));

		ndSharedPtr<ndModel> referenceModel (CreateModel(scene, matrix, modelMesh));
		ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(referenceModel->GetAsModelArticulation()->GetRoot()->m_body->GetMatrix(), referenceModel->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic(), world->GetSentinelBody()));
		world->AddJoint(fixJoint);
	
		RobotModelNotify* const notify = (RobotModelNotify*)*referenceModel->GetAsModel()->GetNotifyCallback();
		notify->SetController(policy);
		world->AddModel(referenceModel);
		referenceModel->AddBodiesAndJointsToWorld();


		//referenceModel->SetNotifyCallback(new RobotModelNotify(policy, referenceModel->GetAsModelArticulation(), true));
		
		//ndSharedPtr<ndUIEntity> quadrupedUI(new ndModelUI(scene, (RobotModelNotify*)*referenceModel->GetNotifyCallback()));
		//scene->Set2DDisplayRenderFunction(quadrupedUI);
		//
		//matrix.m_posit.m_z += 1.5f;
		//
		//ndInt32 countZ = 5;
		//ndInt32 countX = 5;
		//
		////countZ = 0;
		////countX = 0;
		//for (ndInt32 i = 0; i < countZ; ++i)
		//{
		//	for (ndInt32 j = 0; j < countX; ++j)
		//	{
		//		ndMatrix location(matrix);
		//		location.m_posit.m_x += 3.0f * ndFloat32 (j - countX/2);
		//		location.m_posit.m_z += 3.0f * ndFloat32 (i - countZ/2);
		//		ndSharedPtr<ndModel> model (CreateModel(scene, location));
		//		model->SetNotifyCallback(new RobotModelNotify(policy, model->GetAsModelArticulation(), false));
		//		world->AddModel(model);
		//		model->AddBodiesAndJointsToWorld();
		//		//m_models.Append(model);
		//		//SetMaterial(model);
		//	}
		//}

	#endif
	
	matrix.m_posit.m_x -= 8.0f;
	matrix.m_posit.m_y += 1.5f;
	matrix.m_posit.m_z += 0.25f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
