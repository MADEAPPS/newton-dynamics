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

/*
This demo trains a simple robot using a policy gradient method
either vanilla policy gradients, Proximal Policy Optimization(PPO), or a custom variant.

While policy gradient methods tend to be more stable than Q value based methods, 
they require collecting a large amount of data, which can be impractical on hardware with limited resources.
As model complexity grows from medium to large scale, The variance in the collected data increases exponentially.
This happens because uncertainty in the data stems from all control outputs, 
and any moderately complex robot typically deals with an input vector composed of dozens of signals.

To manage this high variance,one common solution is to train thousands of agents in parallel.
This approach is motivated by the theoretical foundation of policy gradient methods, 
which update model parameters based on the expected reward over all possible trajectories.
However, in practice, we must estimate this expectation from a finite set of sampled trajectories.
The challenge arises because, while our ability to generate data grows linearly(or sub linearly) with system resources,
the variance of that data increases exponentially with the size of the action vector.
This becomes a major bottleneck for small to medium sized systems.

In fact, even a high end single GPU system may struggle with this demand, 
which often limits the practical use of policy gradient methods to organizations with access 
to large scale computing resources such as supercomputers plus a large number of humans in the 
loop to supervise data generation.

therefore, it is my opinion and conclusion that Q base methods like DDPG, TD3 and SAC
are more suitable for medium small systems.
*/

// This model attempts to take animation poses and use a reward system to generate a policy  
// that produces the animation.  
// If this phase is successful, we will adapt the reward so that the robot can adjust  
// to the environment with increasing complexity
namespace ndQuadruped_2
{
	#define ND_TRAIN_MODEL

	//#define USE_SAC

	#ifdef USE_SAC
		#define CONTROLLER_NAME "ndQuadruped_2-sac.dnn"
	#else	
		#define CONTROLLER_NAME "ndQuadruped_2-ppo.dnn"
	#endif

	enum Actions
	{
		m_actionPosit_x,
		m_actionPosit_y,
		m_actionPosit_z,
		m_actionsSize
	};

	enum Observations
	{
		m_effectPosit_x,
		m_effectPosit_y,
		m_effectPosit_z,
		m_effectVeloc_x,
		m_effectVeloc_y,
		m_effectVeloc_z,
		m_posePosit_x,
		m_posePosit_y,
		m_posePosit_z,
		m_poseVeloc_x,
		m_poseVeloc_y,
		m_poseVeloc_z,

		m_observationSize
	};

	#define D_CYCLE_PERIOD			ndFloat32(4.0f)
	#define D_CYCLE_STRIDE_X		ndFloat32(0.3f)
	#define D_CYCLE_STRIDE_Z		ndFloat32(0.3f)
	#define D_CYCLE_AMPLITUDE		ndFloat32(0.27f)
	#define D_POSE_REST_POSITION_Y	ndReal(-0.3f)

	#define D_ACTION_SPEED			ndReal(0.005f)

	class RobotModelNotify : public ndModelNotify
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
		
			ndVector GetTranslation(ndFloat32) const
			{
				return ndVector::m_zero;
			}
		
			void CalculatePose(ndAnimationPose& output, ndFloat32 param) override
			{
				// generate a procedural in place march gait
				ndAssert(param >= ndFloat32(0.0f));
				ndAssert(param <= ndFloat32(1.0f));
		
				ndFloat32 gaitFraction = 0.25f;
				ndFloat32 gaitGuard = gaitFraction * 0.25f;
				ndFloat32 omega = ndPi / (gaitFraction - gaitGuard);
		
				for (ndInt32 i = 0; i < output.GetCount(); i++)
				{
					const ndEffectorInfo& leg = *(ndEffectorInfo*)output[i].m_userData;;
					output[i].m_userParamFloat = 0.0f;
					output[i].m_posit = leg.m_effector->GetRestPosit();
				}
		
				for (ndInt32 i = 0; i < output.GetCount(); i++)
				{
					//if (i != 2)
					//{
					//	continue;
					//}
					const ndEffectorInfo& leg = *(ndEffectorInfo*)output[i].m_userData;;
					const ndVector localPosit(leg.m_effector->GetRestPosit());
					ndFloat32 stride_x = m_stride_x;
					//ndFloat32 stride_z = m_stride_z;
					ndFloat32 phase = 0.0f;
					if (localPosit.m_x > 0.0f)
					{
						phase = (localPosit.m_z > 0.0f) ? 0.0f : 0.50f;
					}
					else
					{
						phase = (localPosit.m_z > 0.0f) ? 0.75f : 0.25f;
					}
					
					//stride_x = 0.0f;
					//stride_z = 0.0f;
					
					ndFloat32 t = ndMod(param - phase + ndFloat32(1.0f), ndFloat32(1.0f));
					if ((t >= gaitGuard) && (t <= gaitFraction))
					{
						output[i].m_posit.m_y += m_amp * ndSin(omega * (t - gaitGuard));
						output[i].m_userParamFloat = 1.0f;
					
						ndFloat32 num = t - gaitGuard;
						ndFloat32 den = gaitFraction - gaitGuard;
					
						ndFloat32 t0 = num / den;
						output[i].m_posit.m_x += stride_x * t0 - stride_x * 0.5f;
						//output[i].m_posit.m_z += -(stride_z * t0 - stride_z * 0.5f);
					}
					else
					{
						if (t <= gaitGuard)
						{
							t += 1.0f;
							output[i].m_userParamFloat = 0.5f;
						}
					
						ndFloat32 num = t - gaitFraction;
						ndFloat32 den = 1.0f - (gaitFraction - gaitGuard);
						ndFloat32 t0 = num / den;
						output[i].m_posit.m_x += -(stride_x * t0 - stride_x * 0.5f);
						//output[i].m_posit.m_z += (stride_z * t0 - stride_z * 0.5f);
					}
					//m_currentPose[i] = output[i].m_posit;
				}
			}
		
			//ndVector m_currentPose[4];
			ndVector m_poseBoundMin;
			ndVector m_poseBoundMax;
			ndFloat32 m_amp;
			ndFloat32 m_stride_x;
			ndFloat32 m_stride_z;
		};

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
		
#ifdef USE_SAC
		class ndControllerTrainer : public ndBrainAgentDeterministicPolicyGradient_Agent
#else
		class ndControllerTrainer : public ndBrainAgentContinuePolicyGradient_Agent
#endif
		{
			public:
#ifdef USE_SAC
			ndControllerTrainer(const ndSharedPtr<ndBrainAgentDeterministicPolicyGradient_Trainer>& master, RobotModelNotify* const robot)
				:ndBrainAgentDeterministicPolicyGradient_Agent(master)
#else
			ndControllerTrainer(const ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master, RobotModelNotify* const robot)
				: ndBrainAgentContinuePolicyGradient_Agent(master)
#endif
				,m_robot(robot)
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
		RobotModelNotify(ndModelArticulation* const robot)
			:ndModelNotify()
			,m_animPose0()
			,m_animPose1()
			,m_poseGenerator()
			,m_animBlendTree()
			,m_basePose()
			,m_legs()
			,m_controller(nullptr)
			,m_controllerTrainer(nullptr)
			,m_timestep(ndFloat32(0.0f))
		{
			SetModel(robot);
		}

		~RobotModelNotify()
		{
		}

		#ifdef USE_SAC
		void SetControllerTrainer(const ndSharedPtr<ndBrainAgentDeterministicPolicyGradient_Trainer>& master)
		#else
		void SetControllerTrainer(const ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master)
		#endif
		{
			m_controllerTrainer = ndSharedPtr<ndControllerTrainer>(new ndControllerTrainer(master, this));
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
			for (ndInt32 i = 0; i < m_legs.GetCount(); ++i)
			{
				ndEffectorInfo& leg = m_legs[i];
				ndAnimKeyframe keyFrame;
				keyFrame.m_userData = &leg;
				m_animPose0.PushBack(keyFrame);
				m_animPose1.PushBack(keyFrame);
				poseGenerator->AddTrack();
			}
			
			ndModelArticulation* const robot = GetModel()->GetAsModelArticulation();
			for (ndModelArticulation::ndNode* node = robot->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				m_basePose.PushBack(node->m_body->GetAsBodyDynamic());
			}
			
			ResetModel();
		}

		void SetController(const ndSharedPtr<ndBrain>& policy)
		{
			RobotModelNotify* const robot = (RobotModelNotify*)*GetModel()->GetNotifyCallback();
			m_controller = ndSharedPtr<ndController> (new ndController(policy, robot));
		}

		ndVector CalculatePositError(ndInt32 keyFrameIndex) const
		{
			ndJointBilateralConstraint::ndKinematicState kinematicState[4];
			const ndAnimKeyframe keyFrame = m_animPose0[keyFrameIndex];
			const ndEffectorInfo* const leg = (ndEffectorInfo*)keyFrame.m_userData;
			
			leg->m_effector->GetKinematicState(kinematicState);
			const ndVector effectPosit(kinematicState[0].m_posit, kinematicState[1].m_posit, kinematicState[2].m_posit, ndFloat32(0.0f));
			const ndVector animPosit(keyFrame.m_posit);
			const ndVector error(animPosit - effectPosit);
			return error;
		}

		#pragma optimize( "", off )
		bool IsTerminal() const
		{
			// termination for execive velocity
			for (ndInt32 i = 0; i < m_basePose.GetCount(); i++)
			{
				const ndBodyKinematic* const body = m_basePose[i].m_body;
				ndVector veloc(body->GetVelocity());
				ndVector omega(body->GetOmega());
				if (veloc.DotProduct(veloc).GetScalar() > 1000.0f)
				{
					return true;
				}
				if (omega.DotProduct(omega).GetScalar() > 2000.0f)
				{
					return true;
				}
			}
			
			ndSharedPtr<ndAnimationBlendTreeNode> node = m_poseGenerator;
			ndAnimationSequencePlayer* const sequencePlayer = (ndAnimationSequencePlayer*)*node;
			ndPoseGenerator* const poseGenerator = (ndPoseGenerator*)*sequencePlayer->GetSequence();
			const ndVector bound(poseGenerator->m_poseBoundMax * ndFloat32 (1.25f));
			for (ndInt32 i = 0; i < m_animPose0.GetCount(); ++i)
			{
				const ndVector error(CalculatePositError(i));
				if (ndAbs(error.m_x) > bound.m_x)
				{
					return true;
				}
			
				if (ndAbs(error.m_y) > bound.m_y)
				{
					return true;
				}
			
				if (ndAbs(error.m_z) > bound.m_z)
				{
					return true;
				}
			}

			return false;
		}

		#pragma optimize( "", off )
		ndBrainFloat CalculateReward() const
		{
			if (IsTerminal())
			{
				return -1.0f;
				//return 0.0f;
			}

			//static int xxxx;
			//xxxx++;
			//if (xxxx >= 45)
			//	xxxx *= 1;
			
			ndFloat32 reward = 0.0f;
			
			ndSharedPtr<ndAnimationBlendTreeNode> node = m_poseGenerator;
			ndAnimationSequencePlayer* const sequencePlayer = (ndAnimationSequencePlayer*)*node;
			ndPoseGenerator* const poseGenerator = (ndPoseGenerator*)*sequencePlayer->GetSequence();
			ndVector normalize(poseGenerator->m_poseBoundMax.Reciproc());

			ndFloat32 weight = 0.0f;
			for (ndInt32 i = 0; i < m_animPose0.GetCount(); ++i)
			{
				const ndVector error(CalculatePositError(i));
				const ndVector normalError(error * normalize);
				for (ndInt32 j = 0; j < 3; ++j)
				{
					ndFloat32 dist = ndFloat32(1.0f) - ndClamp(ndAbs(normalError[j]), 0.0f, 1.0f);
					ndFloat32 legReward = ndPow(dist, 6.0f);;
					//ndFloat32 error2 = error[j] * error[j];
					//ndFloat32 legReward = ndExp(-200000.0f * error2);
					reward += legReward;
				}
				weight += 3;
			}
			return reward / weight;
		}

		#pragma optimize( "", off )
		void ResetModel()
		{
			ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
			for (ndInt32 i = 0; i < m_basePose.GetCount(); i++)
			{
				m_basePose[i].SetPose();
			}
			
			model->ClearMemory();
			
			
			//ndFloat32 animSpeed = 1.0f;
			//m_controllerTrainer->m_animBlendTree->Update(timestep * animSpeed);
			
			//ndVector veloc;
			//m_animPose0.CopySource(m_animPose1);
			//m_controllerTrainer->m_animBlendTree->Evaluate(m_animPose1, veloc);
			
			//ndUnsigned32 start = ndRandInt() & 3;
			ndFloat32 duration = ((ndAnimationSequencePlayer*)*m_poseGenerator)->GetSequence();
			//m_animBlendTree->SetTime(duration * ndFloat32(start));
			//m_animBlendTree->SetTime(duration * ndFloat32(start));
			
			m_animBlendTree->SetTime(ndRand() * duration);
			//m_controllerTrainer->m_time = ndFmod(ndRand(), 1.0f);
			//m_time = 0.0f;
			//m_animFrame = animFrame;
			//m_animPose1.CopySource(m_animPose);
			ndVector veloc;
			m_animBlendTree->Evaluate(m_animPose1, veloc);
		}

		#pragma optimize( "", off )
		void GetObservation(ndBrainFloat* const observations)
		{
			ndFloat32 invTimestep = 1.0f / m_timestep;
			for (ndInt32 i = 0; i < m_legs.GetCount(); ++i)
			{
				ndEffectorInfo& leg = m_legs[i];
				ndJointBilateralConstraint::ndKinematicState kinematicState[4];
				leg.m_effector->GetKinematicState(kinematicState);
				const ndVector effectorPosit(kinematicState[0].m_posit, kinematicState[1].m_posit, kinematicState[2].m_posit, 0.0f);
				const ndVector effectorVeloc(kinematicState[0].m_velocity, kinematicState[1].m_velocity, kinematicState[2].m_velocity, 0.0f);

				const ndAnimKeyframe keyFrame0 = m_animPose0[i];
				const ndAnimKeyframe keyFrame1 = m_animPose1[i];
				const ndVector keyFramePosit0(keyFrame0.m_posit);
				const ndVector keyFramePosit1(keyFrame1.m_posit);

				ndInt32 base = m_observationSize * i;
				observations[base + m_effectPosit_x] = effectorPosit.m_x;
				observations[base + m_effectPosit_y] = effectorPosit.m_y;
				observations[base + m_effectPosit_z] = effectorPosit.m_z;
				observations[base + m_effectVeloc_x] = effectorVeloc.m_x;
				observations[base + m_effectVeloc_y] = effectorVeloc.m_y;
				observations[base + m_effectVeloc_z] = effectorVeloc.m_z;
				observations[base + m_posePosit_x] = keyFramePosit0.m_x;
				observations[base + m_posePosit_y] = keyFramePosit0.m_y;
				observations[base + m_posePosit_z] = keyFramePosit0.m_z;
				observations[base + m_poseVeloc_x] = (keyFramePosit1.m_x - keyFramePosit0.m_x) * invTimestep;
				observations[base + m_poseVeloc_y] = (keyFramePosit1.m_y - keyFramePosit0.m_y) * invTimestep;
				observations[base + m_poseVeloc_z] = (keyFramePosit1.m_z - keyFramePosit0.m_z) * invTimestep;
			}
		}

		void ApplyActions(ndBrainFloat* const actions)
		{
			ndBodyKinematic* const rootBody = GetModel()->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
			
			const ndVector upVector(rootBody->GetMatrix().m_up);
			
			ndSharedPtr<ndAnimationBlendTreeNode> node = m_poseGenerator;
			ndAnimationSequencePlayer* const sequencePlayer = (ndAnimationSequencePlayer*)*node;
			ndPoseGenerator* const poseGenerator = (ndPoseGenerator*)*sequencePlayer->GetSequence();
			const ndVector bound(poseGenerator->m_poseBoundMax.Scale(ndFloat32(1.5f)));
			for (ndInt32 i = 0; i < m_legs.GetCount(); ++i)
			{
				ndEffectorInfo& leg = m_legs[i];
				ndIkSwivelPositionEffector* const effector = leg.m_effector;
				ndInt32 base = m_actionsSize * i;
				const ndVector resPosit(leg.m_effector->GetRestPosit());
				//const ndVector effectorPosit(leg.m_effector->GetEffectorPosit());
				
				//ndVector relativePosit(effectorPosit - resPosit);
				//relativePosit.m_x += actions[base + m_actionPosit_x] * D_ACTION_SPEED;
				//relativePosit.m_y += actions[base + m_actionPosit_y] * D_ACTION_SPEED;
				//relativePosit.m_z += actions[base + m_actionPosit_z] * D_ACTION_SPEED;
				//relativePosit.m_x = ndClamp(relativePosit.m_x, -bound.m_x, bound.m_x);
				//relativePosit.m_y = ndClamp(relativePosit.m_y, -bound.m_y, bound.m_y);
				//relativePosit.m_z = ndClamp(relativePosit.m_z, -bound.m_z, bound.m_z);
				
				const ndAnimKeyframe keyFrame = m_animPose0[i];
				ndVector keyFramePosit(keyFrame.m_posit);
				keyFramePosit.m_x += actions[base + m_actionPosit_x] * D_ACTION_SPEED;
				keyFramePosit.m_y += actions[base + m_actionPosit_y] * D_ACTION_SPEED;
				keyFramePosit.m_z += actions[base + m_actionPosit_z] * D_ACTION_SPEED;

				ndVector localPost(keyFramePosit - resPosit);
				localPost.m_x = ndClamp(localPost.m_x, -bound.m_x, bound.m_x);
				localPost.m_y = ndClamp(localPost.m_y, -bound.m_y, bound.m_y);
				localPost.m_z = ndClamp(localPost.m_z, -bound.m_z, bound.m_z);

				const ndVector newPosit(localPost + resPosit);
				effector->SetLocalTargetPosition(newPosit);
				//effector->SetLocalTargetPosition(keyFramePosit);
			
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

			//ndFloat32 animSpeed = 2.0f * m_control->m_animSpeed;
			ndFloat32 animSpeed = 1.0f;
			m_animBlendTree->Update(timestep * animSpeed);

			ndVector veloc;
			m_animPose0.CopySource(m_animPose1);
			m_animBlendTree->Evaluate(m_animPose1, veloc);

			if (m_controllerTrainer)
			{
				m_controllerTrainer->Step();
			}
			else
			{
				ndAssert(0);
				m_controller->Step();
			}
		}

		ndAnimationPose m_animPose0;
		ndAnimationPose m_animPose1;
		ndSharedPtr<ndAnimationBlendTreeNode> m_poseGenerator;
		ndSharedPtr<ndAnimationBlendTreeNode> m_animBlendTree;
		ndFixSizeArray<ndBasePose, 32> m_basePose;
		ndFixSizeArray<ndEffectorInfo, 4> m_legs;
		ndSharedPtr<ndController> m_controller;
		ndSharedPtr<ndControllerTrainer> m_controllerTrainer;

		//ndFloat32 m_time;
		ndFloat32 m_timestep;
		//ndFloat32 m_animFrame;
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
		for (ndList<ndSharedPtr<ndDemoEntity>>::ndNode* node = entity->GetChildren().GetFirst(); node; node = node->GetNext())
		{
			// build thig
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

		notify->InitAnimation();
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
			,m_stopTraining(200 * 1000000)
			,m_modelIsTrained(false)
		{
			ndWorld* const world = scene->GetWorld();

			#ifdef USE_SAC
				m_outFile = fopen("ndQuadruped_2-sac.csv", "wb");
				fprintf(m_outFile, "sac\n");

				m_stopTraining = 200000;
				ndBrainAgentDeterministicPolicyGradient_Trainer::HyperParameters hyperParameters;
				hyperParameters.m_numberOfActions = m_actionsSize * 4;
				hyperParameters.m_numberOfObservations = m_observationSize * 4;
				//m_master = ndSharedPtr<ndBrainAgentDeterministicPolicyGradient_Trainer>(new ndBrainAgentDeterministicPolicyGradient_Trainer(hyperParameters));
				m_master = ndSharedPtr<ndBrainAgentDeterministicPolicyGradient_Trainer>(new ndBrainAgentSoftActorCritic_Trainer(hyperParameters));
			#else
				m_outFile = fopen("ndQuadruped_2-ppo.csv", "wb");
				fprintf(m_outFile, "ppo\n");

				ndBrainAgentContinuePolicyGradient_TrainerMaster::HyperParameters hyperParameters;
				hyperParameters.m_numberOfActions = m_actionsSize * 4;
				hyperParameters.m_numberOfObservations = m_observationSize * 4;
				//m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>(new ndBrainAgentContinuePolicyGradient_TrainerMaster(hyperParameters));
				m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>(new ndBrainAgentContinueProximaPolicyGradient_TrainerMaster(hyperParameters));
			#endif

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

			// add a hidden battery of model to generate trajectories in parallel
#ifndef USE_SAC
			ndInt32 countX = 10;
			ndInt32 countZ = 10;
			//countX = 0;
			//countZ = 0;
			
			// add a hidden battery of model to generate trajectories in parallel
			for (ndInt32 i = 0; i < countZ; ++i)
			{
				for (ndInt32 j = 0; j < countX; ++j)
				{
					ndMatrix location(matrix);
					location.m_posit.m_x += 20.0f * (ndRand() - 0.5f);
					location.m_posit.m_z += 20.0f * (ndRand() - 0.5f);

					ndFloat32 step = 20.0f * (ndRand() - 0.5f);
					location.m_posit.m_x += step;

					ndSharedPtr<ndModel>model(CreateModel(scene, location, modelMesh));
					RobotModelNotify* const notify1 = (RobotModelNotify*)*model->GetAsModel()->GetNotifyCallback();
					notify1->SetControllerTrainer(m_master);

					SetMaterial(model->GetAsModelArticulation());
					world->AddModel(model);
					model->AddBodiesAndJointsToWorld();

					m_models.Append(model->GetAsModelArticulation());

					ndBodyKinematic* const rootBody1 = model->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
					ndSharedPtr<ndJointBilateralConstraint> fixJoint1(new ndJointFix6dof(rootBody1->GetMatrix(), rootBody1, world->GetSentinelBody()));
					world->AddJoint(fixJoint1);

				}
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

		#pragma optimize( "", off )
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
						//fprintf(m_outFile, "%g\n", ndMax (m_master->GetAverageScore(), ndFloat32 (0.0f)));
						fflush(m_outFile);
					}
				}
			}
			
			ndFloat32 stopScore = 100.0f * ndFloat32(m_master->GetAverageFrames() * m_master->GetAverageScore()) / m_horizon;
			if ((stopTraining >= m_stopTraining) || (stopScore > 95.0f * ndFloat32(m_master->m_parameters.m_maxTrajectorySteps)))
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

using namespace ndQuadruped_2;

void ndQuadrupedTest_2(ndDemoEntityManager* const scene)
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
	matrix.m_posit.m_y = 0.75f;

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
