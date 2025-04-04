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


// This model attempts to take animation poses and use a reward system to generate a policy  
// that produces the animation.  
// If this phase is successful, we will adapt the reward so that the robot can adjust  
// to the environment with increasing complexity
namespace ndQuadruped_2
{
	#define ND_TRAIN_MODEL
	#define CONTROLLER_NAME "ndQuadruped_2-vpg.dnn"

	enum ndActionSpace
	{
		m_leg0_x,
		m_leg0_y,
		m_leg0_z,

		m_leg1_x,
		m_leg1_y,
		m_leg1_z,

		m_leg2_x,
		m_leg2_y,
		m_leg2_z,

		m_leg3_x,
		m_leg3_y,
		m_leg3_z,

		m_actionsSize
	};

	enum ndStateSpace
	{
		m_leg0_posit_x,
		m_leg0_posit_y,
		m_leg0_posit_z,

		m_leg1_posit_x,
		m_leg1_posit_y,
		m_leg1_posit_z,

		m_leg2_posit_x,
		m_leg2_posit_y,
		m_leg2_posit_z,

		m_leg3_posit_x,
		m_leg3_posit_y,
		m_leg3_posit_z,

		m_frameTick,
		m_stateSize
	};


#if 0
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

			ndControllerTrainer(ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master)
				:ndBrainAgentContinuePolicyGradient_Trainer(master)
				,m_robot(nullptr)
			{
				ndMemSet(m_rewardsMemories, ndReal(1.0), sizeof(m_rewardsMemories) / sizeof(m_rewardsMemories[0]));
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

			ndFixSizeArray<ndBasePose, 32> m_basePose;
			RobotModelNotify* m_robot;
			ndReal m_rewardsMemories[32];
		};

		class ndPoseGenerator : public ndAnimationSequence
		{
			public:
			ndPoseGenerator(const ndFloat32* const phase)
				:ndAnimationSequence()
				,m_amp(0.27f)
				,m_stride_x(0.3f)
				,m_stride_z(0.3f)
			{
				m_duration = ndFloat32(4.0f);
				for (ndInt32 i = 0; i < 4; i++)
				{
					m_phase[i] = phase[i];
					m_offset[i] = ndVector::m_zero;
					m_currentPose[i] = BasePose(i);
				}
			}

			ndVector GetTranslation(ndFloat32) const
			{
				return ndVector::m_zero;
			}

			ndVector BasePose(ndInt32 index) const
			{
				ndVector base(ndVector::m_wOne);
				base.m_x = m_offset[index].m_x;
				base.m_z = m_offset[index].m_z;
				base.m_y = m_offset[index].m_y;
				return base;
			}

			void CalculatePose(ndAnimationPose& output, ndFloat32 param) const
			{
				// generate a procedural in place march gait
				ndAssert(param >= ndFloat32(0.0f));
				ndAssert(param <= ndFloat32(1.0f));

				ndFloat32 gaitFraction = 0.25f;
				ndFloat32 gaitGuard = gaitFraction * 0.25f;
				ndFloat32 omega = ndPi / (gaitFraction - gaitGuard);

				for (ndInt32 i = 0; i < output.GetCount(); i++)
				{
					output[i].m_userParamFloat = 0.0f;
					output[i].m_posit = BasePose(i);
				}

				for (ndInt32 i = 0; i < output.GetCount(); i++)
				{
					ndFloat32 stride_x = m_stride_x;
					if ((i == 2) || (i == 3))
					{
						stride_x *= -1;
					}

					ndFloat32 stride_z = m_stride_z;
					if ((i == 2) || (i == 3))
					{
						stride_z *= -1;
					}

					ndFloat32 t = ndMod(param - m_phase[i] + ndFloat32(1.0f), ndFloat32(1.0f));
					if ((t >= gaitGuard) && (t <= gaitFraction))
					{
						output[i].m_posit.m_y += m_amp * ndSin(omega * (t - gaitGuard));
						output[i].m_userParamFloat = 1.0f;

						ndFloat32 num = t - gaitGuard;
						ndFloat32 den = gaitFraction - gaitGuard;

						ndFloat32 t0 = num / den;
						//output[i].m_posit.m_x += stride_x * t0 - stridem_x * 0.5f;
						output[i].m_posit.m_z += -(stride_z * t0 - stride_z * 0.5f);
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
						//output[i].m_posit.m_x += -(stride_x * t0 - stridem_x * 0.5f + );
						output[i].m_posit.m_z += (stride_z * t0 - stride_z * 0.5f);
					}

					m_currentPose[i] = output[i].m_posit;
				}
			}

			mutable ndVector m_currentPose[4];
			ndVector m_offset[4];
			ndFloat32 m_phase[4];
			ndFloat32 m_amp;
			ndFloat32 m_stride_x;
			ndFloat32 m_stride_z;
		};

		class ndUIControlNode : public ndAnimationBlendTreeNode
		{
			public:
			ndUIControlNode(ndAnimationBlendTreeNode* const input)
				:ndAnimationBlendTreeNode(input)
				,m_x(ndReal(0.0f))
				,m_y(ndReal(0.0f))
				,m_z(ndReal(0.0f))
				,m_yaw(ndReal(0.0f))
				,m_roll(ndReal(0.0f))
				,m_pitch(ndReal(0.0f))
				,m_animSpeed(ndReal(0.0f))
				,m_enableController(true)
			{
				Reset();
			}

			void Reset()
			{
				m_x = ndReal(0.0f);
				m_y = ndReal(0.0f);
				m_z = ndReal(0.0f);
				m_yaw = ndReal(0.0f);
				m_roll = ndReal(0.0f);
				m_pitch = ndReal(0.0f);
			}

			void Evaluate(ndAnimationPose& output, ndVector& veloc)
			{
				ndAnimationBlendTreeNode::Evaluate(output, veloc);

				ndMatrix matrix(ndPitchMatrix(m_pitch * ndDegreeToRad) * ndYawMatrix(m_yaw * ndDegreeToRad) * ndRollMatrix(m_roll * ndDegreeToRad));
				matrix.m_posit.m_x = -m_x;
				matrix.m_posit.m_y = -m_y;
				matrix.m_posit.m_z = -m_z;
				for (ndInt32 i = 0; i < output.GetCount(); ++i)
				{
					ndAnimKeyframe& keyFrame = output[i];
					ndEffectorInfo* const info = (ndEffectorInfo*)keyFrame.m_userData;
					ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;

					ndMatrix localMatrix(effector->GetLocalMatrix1());
					ndVector p0(localMatrix.TransformVector(keyFrame.m_posit));
					ndVector p1(matrix.TransformVector(p0));
					ndVector p2(localMatrix.UntransformVector(p1));
					keyFrame.m_posit = p2;
				}
			}

			ndReal m_x;
			ndReal m_y;
			ndReal m_z;
			ndReal m_yaw;
			ndReal m_roll;
			ndReal m_pitch;
			ndReal m_animSpeed;
			bool m_enableController;
		};

		public:
		RobotModelNotify(ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master, ndModelArticulation* const robot, bool showDebug)
			:ndModelNotify()
			,m_invDynamicsSolver()
			,m_controller(nullptr)
			,m_controllerTrainer(nullptr)
			,m_world(nullptr)
			,m_timestep(ndFloat32(0.0f))
			,m_modelAlive(true)
			,m_showDebug(showDebug)
		{
			m_controllerTrainer = new ndControllerTrainer(master);
			m_controllerTrainer->m_robot = this;
			Init(robot);
		}

		RobotModelNotify(const ndSharedPtr<ndBrain>& brain, ndModelArticulation* const robot, bool showDebug)
			:ndModelNotify()
			,m_invDynamicsSolver()
			,m_controller(nullptr)
			,m_controllerTrainer(nullptr)
			,m_world(nullptr)
			,m_timestep(ndFloat32 (0.0f))
			,m_modelAlive(true)
			,m_showDebug(showDebug)
		{
			m_controller = new ndController(brain);
			m_controller->m_robot = this;
			Init(robot);
			m_control->m_animSpeed = ndReal (D_MIN_TRAIN_ANIM_SPEED + (1.0f - D_MIN_TRAIN_ANIM_SPEED) * ndRand());
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

		ndModelNotify* Clone() const
		{
			return new RobotModelNotify(*this);
		}

		void Init(ndModelArticulation* const robot)
		{
			static ndInt32 modelId = 0;
			m_modelId = modelId++;

			ndFloat32 phase[] = { 0.0f, 0.75f, 0.25f, 0.5f };
			ndSharedPtr<ndAnimationSequence> sequence(new ndPoseGenerator(phase));

			m_poseGenerator = new ndAnimationSequencePlayer(sequence);
			m_control = new ndUIControlNode(m_poseGenerator);
			m_animBlendTree = ndSharedPtr<ndAnimationBlendTreeNode>(m_control);

			ndFloat32 duration = m_poseGenerator->GetSequence()->GetDuration();
			m_animBlendTree->SetTime(duration * ndRand());

			ndFloat32 offset_x[] = { 0.2f, 0.2f, 0.2f, 0.2f };
			ndFloat32 offset_z[] = { -0.3f, 0.3f, -0.3f, 0.3f };
			ndFloat32 offset_y[] = { D_POSE_REST_POSITION_Y, D_POSE_REST_POSITION_Y, D_POSE_REST_POSITION_Y, D_POSE_REST_POSITION_Y };

			ndFloat32 angles[] = { -90.0f, -90.0f, 90.0f, 90.0f };
			const char* effectorNames[] = { "foot_0",  "foot_1",  "foot_2",  "foot_3" };

			ndPoseGenerator* const poseGenerator = (ndPoseGenerator*)*sequence;

			ndFloat32 effectorStrength = 20.0f * 10.0f * 500.0f;
			const ndMatrix rootMatrix(robot->GetRoot()->m_body->GetMatrix());
			for (ndInt32 i = 0; i < 4; ++i)
			{
				ndModelArticulation::ndNode* const heelNode = robot->FindByName(effectorNames[i]);
				if (heelNode)
				{
					ndModelArticulation::ndNode* const calfNode = heelNode->GetParent();
					ndModelArticulation::ndNode* const thighNode = calfNode->GetParent();

					ndBodyKinematic* const heelBody = heelNode->m_body->GetAsBodyKinematic();
					ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)heelBody->GetNotifyCallback();
					ndDemoEntity* const heelEntirty = *notify->m_entity;
					ndSharedPtr<ndDemoEntity> footEntity(heelEntirty->GetChildren().GetFirst()->GetInfo());

					ndMatrix effectorMatrix(footEntity->GetCurrentMatrix() * heelBody->GetMatrix());
					ndMatrix effectorRefFrame(ndGetIdentityMatrix());
					effectorRefFrame.m_posit = rootMatrix.TransformVector(thighNode->m_joint->GetLocalMatrix1().m_posit);

					ndFloat32 regularizer = 0.001f;
					ndIkSwivelPositionEffector* const effector = new ndIkSwivelPositionEffector(effectorMatrix.m_posit, effectorRefFrame, heelBody, robot->GetRoot()->m_body->GetAsBodyKinematic());
					effector->SetLinearSpringDamper(regularizer, 4000.0f, 50.0f);
					effector->SetAngularSpringDamper(regularizer, 4000.0f, 50.0f);
					effector->SetWorkSpaceConstraints(0.0f, 0.75f * 0.9f);
					effector->SetMaxForce(effectorStrength);
					effector->SetMaxTorque(effectorStrength);
					
					ndEffectorInfo info(thighNode->m_joint, calfNode->m_joint, heelNode->m_joint, ndSharedPtr<ndJointBilateralConstraint>(effector));
					m_effectorsInfo.PushBack(info);
					
					ndAnimKeyframe keyFrame;
					keyFrame.m_userData = &m_effectorsInfo[m_effectorsInfo.GetCount() - 1];
					m_animPose.PushBack(keyFrame);
					poseGenerator->AddTrack();
					poseGenerator->m_phase[i] = phase[i];
					poseGenerator->m_offset[i].m_x = offset_x[i];
					poseGenerator->m_offset[i].m_y = offset_y[i];
					poseGenerator->m_offset[i].m_z = offset_z[i];
				}
			}

			for (ndModelArticulation::ndNode* node = robot->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				m_controllerTrainer->m_basePose.PushBack(node->m_body->GetAsBodyDynamic());
			}
		}

		bool IsTerminal() const
		{
			if (!m_modelAlive)
			{
				return true;
			}

			const ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
			const ndMatrix matrix(model->GetRoot()->m_body->GetMatrix());
			const ndVector& up = matrix.m_up;
			if (up.m_y < D_MODEL_DEAD_ANGLE)
			{
				return true;
			}

			bool isDead = true;
			for (ndInt32 i = sizeof(m_controllerTrainer->m_rewardsMemories) / sizeof(m_controllerTrainer->m_rewardsMemories[0]) - 1; i >= 0; --i)
			{
				isDead &= m_controllerTrainer->m_rewardsMemories[i] < ndReal(0.05f);
			}
			if (isDead)
			{
				return isDead;
			}
			return false;
		}

		ndBrainFloat CalculateDistanceToOrigin() const
		{
			ndFloat32 x = m_control->m_x / D_MAX_SWING_DIST_X;
			ndFloat32 z = m_control->m_z / D_MAX_SWING_DIST_Z;

			// L1 distance
			//const ndFloat32 aliveZone = 0.5f;
			//ndFloat32 h = ndMax(ndAbs(x), ndAbs(z));
			//ndFloat32 dist = ndMax(h - aliveZone, 0.0f);
			//ndFloat32 reward = ndExp(-200.0f * dist * dist);

			// L2 distance
			ndFloat32 dist2 = x * x + z * z;
			ndBrainFloat reward = ndBrainFloat(ndExp(-10.0f * dist2));

			//if (m_id == 0)
			//{
			//	ndExpandTraceMessage("dist reward(%f)\n", reward);
			//}
			return reward;
		}

		ndBrainFloat CalculateZeroMomentPointReward() const
		{
			ndFixSizeArray<ndBigVector, 16> desiredSupportPoint;
			for (ndInt32 i = 0; i < m_animPose.GetCount(); ++i)
			{
				const ndAnimKeyframe& keyFrame = m_animPose[i];
				ndEffectorInfo* const info = (ndEffectorInfo*)keyFrame.m_userData;
				ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;

				if (keyFrame.m_userParamFloat < 1.0f)
				{
					desiredSupportPoint.PushBack(effector->GetGlobalPosition());
				}
			}

			ndBrainFloat reward = ndBrainFloat(0.0f);
			const ndVector zmp(CalculateZeroMomentPoint());
			if (desiredSupportPoint.GetCount() >= 3)
			{
				ndBigVector p0Out;
				ndBigVector p1Out;
				ndBigVector ray_p0(zmp);
				ndBigVector ray_p1(zmp);
				ray_p1.m_y -= ndFloat32(1.5f);

				ScaleSupportShape(desiredSupportPoint);

				ndRayToPolygonDistance(ray_p0, ray_p1, &desiredSupportPoint[0], desiredSupportPoint.GetCount(), p0Out, p1Out);
				const ndBigVector error((p0Out - p1Out) & ndBigVector::m_triplexMask);
				ndFloat32 dist2 = ndFloat32(error.DotProduct(error).GetScalar());
				reward = ndBrainFloat(ndExp(-ndBrainFloat(1000.0f) * dist2));
				//ndTrace(("d2(% f) r(% f)\n", dist2, reward));
			}
			else
			{
				//ndAssert(0);
			}

			return reward;
		}

		bool CalculateExplosionReward() const
		{
			bool isAlive = true;
			const ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
			for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); isAlive && node; node = node->GetNextIterator())
			{
				const ndVector veloc(node->m_body->GetVelocity());
				const ndVector omega(node->m_body->GetOmega());
				ndFloat32 vMag2 = veloc.DotProduct(veloc).GetScalar();
				ndFloat32 wMag2 = omega.DotProduct(omega).GetScalar();
				isAlive = isAlive && (vMag2 < 100.0f);
				isAlive = isAlive && (wMag2 < 500.0f);
				if (!isAlive)
				{
					isAlive = false;
				}
			}
			return isAlive;
		}

		ndReal GetReward() const
		{
			ndBrainFloat dstReward = 0.0f;
			ndBrainFloat zmpReward = 0.0f;

			bool isAlived = CalculateExplosionReward();
			if (isAlived)
			{
				dstReward = CalculateDistanceToOrigin();
				zmpReward = CalculateZeroMomentPointReward();
				//ndBrainFloat reward0 = CalculateZeroOmegaReward();
			}
			else
			{
				// catastrophic penalty,  results in a immediate kill.
				CalculateExplosionReward();
				ndMemSet(m_controllerTrainer->m_rewardsMemories, 0.0f, sizeof(m_controllerTrainer->m_rewardsMemories) / sizeof(m_controllerTrainer->m_rewardsMemories[0]));
			}

			if ((dstReward < 1.0e-3f) || (zmpReward < 1.0e-3f))
			{
				dstReward = 0.0f;
				zmpReward = 0.0f;
			}
			ndBrainFloat reward = 0.80f * zmpReward + 0.20f * dstReward;

			for (ndInt32 i = sizeof(m_controllerTrainer->m_rewardsMemories) / sizeof(m_controllerTrainer->m_rewardsMemories[0]) - 1; i >= 1; --i)
			{
				m_controllerTrainer->m_rewardsMemories[i] = m_controllerTrainer->m_rewardsMemories[i - 1];
			}
			m_controllerTrainer->m_rewardsMemories[0] = reward;
			return reward;
		}

		ndContact* FindContact(ndInt32 index) const
		{
			const ndAnimKeyframe& keyFrame = m_animPose[index];
			ndEffectorInfo* const info = (ndEffectorInfo*)keyFrame.m_userData;
			ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;

			ndBodyKinematic* const body = effector->GetBody0();
			ndBodyKinematic::ndContactMap& contacts = body->GetContactMap();
			ndBodyKinematic::ndContactMap::Iterator it(contacts);
			for (it.Begin(); it; it++)
			{
				ndContact* const contact = *it;
				if (contact->IsActive())
				{
					return contact;
				}
			}
			return nullptr;
		};


		void ApplyActions(ndBrainFloat* const actions)
		{
			//m_control->m_animSpeed = 0.25f;
			if (m_control->m_enableController)
			{
				const ndActionVector& actionVector = *((ndActionVector*)actions);
				m_control->m_x = ndClamp(ndReal(m_control->m_x + actionVector.m_torso_x * D_SWING_STEP), -D_MAX_SWING_DIST_X, D_MAX_SWING_DIST_X);
				m_control->m_z = ndClamp(ndReal(m_control->m_z + actionVector.m_torso_z * D_SWING_STEP), -D_MAX_SWING_DIST_Z, D_MAX_SWING_DIST_Z);
			}

			ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
			ndBodyKinematic* const rootBody = model->GetRoot()->m_body->GetAsBodyKinematic();
			ndSkeletonContainer* const skeleton = rootBody->GetSkeleton();
			ndAssert(skeleton);

			ndVector veloc;
			m_animBlendTree->Evaluate(m_animPose, veloc);

			const ndVector upVector(rootBody->GetMatrix().m_up);
			ndFixSizeArray<ndJointBilateralConstraint*, 32> effectors;
			for (ndInt32 i = 0; i < m_animPose.GetCount(); ++i)
			{
				ndEffectorInfo* const info = &m_effectorsInfo[i];
				effectors.PushBack(*info->m_effector);

				ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;

				ndVector posit(m_animPose[i].m_posit);
				effector->SetLocalTargetPosition(posit);

				ndFloat32 swivelAngle = effector->CalculateLookAtSwivelAngle(upVector);
				effector->SetSwivelAngle(swivelAngle);

				// calculate lookAt angle
				ndMatrix lookAtMatrix0;
				ndMatrix lookAtMatrix1;
				ndJointHinge* const footHinge = (ndJointHinge*)*info->m_foot;
				footHinge->CalculateGlobalMatrix(lookAtMatrix0, lookAtMatrix1);

				ndMatrix upMatrix(ndGetIdentityMatrix());
				upMatrix.m_front = lookAtMatrix1.m_front;
				upMatrix.m_right = (upMatrix.m_front.CrossProduct(upVector) & ndVector::m_triplexMask).Normalize();
				upMatrix.m_up = upMatrix.m_right.CrossProduct(upMatrix.m_front);
				upMatrix = upMatrix * lookAtMatrix0.OrthoInverse();

				const ndFloat32 angle = ndAtan2(upMatrix.m_up.m_z, upMatrix.m_up.m_y);
				footHinge->SetTargetAngle(angle);
			}

			m_invDynamicsSolver.SolverBegin(skeleton, &effectors[0], effectors.GetCount(), m_world, m_timestep);
			m_invDynamicsSolver.Solve();
			m_invDynamicsSolver.SolverEnd();
			CheckModelStability();
		}

		void CheckModelStability()
		{
			const ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
			for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				const ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();
				const ndVector accel(body->GetAccel());
				const ndVector alpha(body->GetAlpha());

				ndFloat32 accelMag2 = accel.DotProduct(accel).GetScalar();
				if (accelMag2 > 1.0e6f)
				{
					m_modelAlive = false;
				}

				ndFloat32 alphaMag2 = alpha.DotProduct(alpha).GetScalar();
				if (alphaMag2 > 1.0e6f)
				{
					m_modelAlive = false;
				}
			}
			if (!m_modelAlive)
			{
				for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
				{
					ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();
					body->SetAccel(ndVector::m_zero);
					body->SetAlpha(ndVector::m_zero);
				}
			}
		}

		void ResetModel()
		{
			m_modelAlive = true;
			m_control->Reset();
			m_control->m_animSpeed = ndReal (D_MIN_TRAIN_ANIM_SPEED + (1.0f - D_MIN_TRAIN_ANIM_SPEED) * ndRand());

			ndMemSet(m_controllerTrainer->m_rewardsMemories, ndReal(1.0), sizeof(m_controllerTrainer->m_rewardsMemories) / sizeof(m_controllerTrainer->m_rewardsMemories[0]));

			ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
			const ndMatrix matrix(model->GetRoot()->m_body->GetMatrix());
			const ndVector& up = matrix.m_up;
			bool state = up.m_y < D_MODEL_DEAD_ANGLE;
			state = state || (matrix.m_posit.m_x > 20.0f);
			state = state || (matrix.m_posit.m_x < -20.0f);
			state = state || (matrix.m_posit.m_z > 20.0f);
			state = state || (matrix.m_posit.m_z < -20.0f);
			if (state)
			{
				for (ndInt32 i = 0; i < m_controllerTrainer->m_basePose.GetCount(); i++)
				{
					m_controllerTrainer->m_basePose[i].SetPose();
				}

				ndFloat32 duration = m_poseGenerator->GetSequence()->GetDuration();

				ndUnsigned32 index = ndRandInt() % 4;
				m_animBlendTree->SetTime(0.25f * ndFloat32(index) * duration);
			}

			ndVector veloc;
			m_animBlendTree->Evaluate(m_animPose, veloc);
		}

		void Update(ndFloat32 timestep)
		{
			m_world = world;
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

		void PostUpdate(ndFloat32 timestep)
		{
			//ndFloat32 animSpeed = (m_control->m_animSpeed > 0.01f) ? (1.0f + 1.0f * m_control->m_animSpeed) : 0.0f;
			ndFloat32 animSpeed = 2.0f * m_control->m_animSpeed;
			m_animBlendTree->Update(timestep * animSpeed);
		}

		void PostTransformUpdate(ndFloat32)
		{
		}

		void ScaleSupportShape(ndFixSizeArray<ndBigVector, 16>& shape) const
		{
			ndBigVector origin(ndBigVector::m_zero);
			for (ndInt32 i = 0; i < shape.GetCount(); ++i)
			{
				origin += shape[i];
			}
			origin = origin.Scale(1.0f / ndFloat32(shape.GetCount()));

			ndFloat32 scale = 0.8f;
			for (ndInt32 i = 0; i < shape.GetCount(); ++i)
			{
				shape[i] = origin + (shape[i] - origin).Scale(scale);
			}
		}

		ndVector CalculateZeroMomentPoint() const
		{
			ndFixSizeArray<ndVector, 32> r;
			ndFixSizeArray<const ndBodyKinematic*, 32> bodies;

			ndVector com(ndVector::m_zero);
			ndFloat32 totalMass = ndFloat32(0.0f);

			ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
			for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				const ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();

				const ndMatrix matrix(body->GetMatrix());
				const ndVector bodyCom(matrix.TransformVector(body->GetCentreOfMass()));
				ndFloat32 mass = body->GetMassMatrix().m_w;
				totalMass += mass;
				com += matrix.TransformVector(body->GetCentreOfMass()).Scale(mass);

				r.PushBack(bodyCom);
				bodies.PushBack(body);
			}
			com = com.Scale(ndFloat32(1.0f) / totalMass);

			ndVector force(ndVector::m_zero);
			ndVector torque(ndVector::m_zero);
			const ndVector gravity(ndFloat32(0.0f), DEMO_GRAVITY, ndFloat32(0.0f), ndFloat32(0.0f));
			for (ndInt32 i = 0; i < bodies.GetCount(); ++i)
			{
				const ndVector centerOfMass(r[i] - com);
				const ndBodyKinematic* const body = bodies[i];
				const ndMatrix bodyInertia(body->CalculateInertiaMatrix());
				const ndVector bodyForce((body->GetAccel() - gravity).Scale(body->GetMassMatrix().m_w));

				force += bodyForce;
				torque += centerOfMass.CrossProduct(bodyForce);
				torque += bodyInertia.RotateVector(body->GetAlpha());
			}
			// remember to clamp the values values before calculating xZmp and zZmp
			//if (ndAbs(force.m_y) > ndFloat32(1.0e-4f))
			if (force.m_y > ndFloat32(1.0e-4f))
			{
				ndAssert(ndAbs(force.m_y) > ndFloat32(0.0f));
				ndFloat32 xZmp = torque.m_z / force.m_y;
				ndFloat32 zZmp = -torque.m_x / force.m_y;
				//ndTrace(("x=%f z=%f\n", xZmp, zZmp));

				com.m_x += xZmp;
				com.m_z += zZmp;
			}
			return com;
		}

		void Debug(ndConstraintDebugCallback& context) const
		{
			if (!m_showDebug)
			{
				return;
			}

			ndModelArticulation* const model = GetModel()->GetAsModelArticulation();

			ndFixSizeArray<const ndBodyKinematic*, 32> bodies;
			
			ndFloat32 totalMass = ndFloat32(0.0f);
			ndVector centerOfMass(ndVector::m_zero);
			for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				const ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
				const ndMatrix matrix(body->GetMatrix());
				ndFloat32 mass = body->GetMassMatrix().m_w;
				totalMass += mass;
				centerOfMass += matrix.TransformVector(body->GetCentreOfMass()).Scale(mass);
				bodies.PushBack(body);
			}
			ndFloat32 invMass = 1.0f / totalMass;
			centerOfMass = centerOfMass.Scale(invMass);
			
			ndVector comLineOfAction(centerOfMass);
			comLineOfAction.m_y -= ndFloat32(0.5f);
			context.DrawLine(centerOfMass, comLineOfAction, ndVector::m_zero);

			ndBodyKinematic* const rootBody = model->GetRoot()->m_body->GetAsBodyKinematic();
			const ndVector upVector(rootBody->GetMatrix().m_up);
			ndFixSizeArray<ndBigVector, 16> supportPoint;
			for (ndInt32 i = 0; i < m_animPose.GetCount(); ++i)
			{
				const ndAnimKeyframe& keyFrame = m_animPose[i];
				ndEffectorInfo* const info = (ndEffectorInfo*)keyFrame.m_userData;
				ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;
				if (i == 0)
				{
					effector->DebugJoint(context);
				}
			
				if (keyFrame.m_userParamFloat < 1.0f)
				{
					ndBodyKinematic* const body = effector->GetBody0();
					supportPoint.PushBack(body->GetMatrix().TransformVector(effector->GetLocalMatrix0().m_posit));
				}
			}
			
			ndVector supportColor(0.0f, 1.0f, 1.0f, 1.0f);
			if (supportPoint.GetCount() >= 3)
			{
				ScaleSupportShape(supportPoint);
				ndFixSizeArray<ndVector, 16> desiredSupportPoint;
				for (ndInt32 i = 0; i < supportPoint.GetCount(); ++i)
				{
					desiredSupportPoint.PushBack(supportPoint[i]);
				}
			
				ndMatrix rotation(ndPitchMatrix(90.0f * ndDegreeToRad));
				rotation.TransformTriplex(&desiredSupportPoint[0].m_x, sizeof(ndVector), &desiredSupportPoint[0].m_x, sizeof(ndVector), desiredSupportPoint.GetCount());
				ndInt32 supportCount = ndConvexHull2d(&desiredSupportPoint[0], desiredSupportPoint.GetCount());
				rotation.OrthoInverse().TransformTriplex(&desiredSupportPoint[0].m_x, sizeof(ndVector), &desiredSupportPoint[0].m_x, sizeof(ndVector), desiredSupportPoint.GetCount());
				ndVector p0(desiredSupportPoint[supportCount - 1]);
				ndBigVector bigPolygon[16];
				for (ndInt32 i = 0; i < supportCount; ++i)
				{
					bigPolygon[i] = desiredSupportPoint[i];
					context.DrawLine(desiredSupportPoint[i], p0, supportColor);
					p0 = desiredSupportPoint[i];
				}
			
				ndBigVector p0Out;
				ndBigVector p1Out;
				ndBigVector ray_p0(centerOfMass);
				ndBigVector ray_p1(comLineOfAction);
				ndRayToPolygonDistance(ray_p0, ray_p1, bigPolygon, supportCount, p0Out, p1Out);
			
				const ndVector centerOfPresure(p0Out);
				context.DrawPoint(centerOfPresure, ndVector(0.0f, 0.0f, 1.0f, 1.0f), 5);
			
				ndVector zmp(CalculateZeroMomentPoint());
				ray_p0 = zmp;
				ray_p1 = zmp;
				ray_p1.m_y -= ndFloat32(0.5f);
				ndRayToPolygonDistance(ray_p0, ray_p1, bigPolygon, supportCount, p0Out, p1Out);
				const ndVector zmpSupport(p0Out);
				context.DrawPoint(zmpSupport, ndVector(1.0f, 0.0f, 0.0f, 1.0f), 5);
			}
			else if (supportPoint.GetCount() == 2)
			{
				ndTrace(("xxxxxxxxxx\n"));
				context.DrawLine(supportPoint[0], supportPoint[1], supportColor);
				//ndBigVector p0Out;
				//ndBigVector p1Out;
				//ndBigVector ray_p0(comMatrix.m_posit);
				//ndBigVector ray_p1(comMatrix.m_posit);
				//ray_p1.m_y -= 1.0f;
				//
				//ndRayToRayDistance(ray_p0, ray_p1, contactPoints[0], contactPoints[1], p0Out, p1Out);
				//context.DrawPoint(p0Out, ndVector(1.0f, 0.0f, 0.0f, 1.0f), 3);
				//context.DrawPoint(p1Out, ndVector(0.0f, 1.0f, 0.0f, 1.0f), 3);
			}
		}

		ndIkSolver m_invDynamicsSolver;
		ndAnimationPose m_animPose;
		ndUIControlNode* m_control;
		ndAnimationSequencePlayer* m_poseGenerator;
		ndFixSizeArray<ndEffectorInfo, 4> m_effectorsInfo;
		ndSharedPtr<ndAnimationBlendTreeNode> m_animBlendTree;
		ndController* m_controller;
		ndControllerTrainer* m_controllerTrainer;
		ndWorld* m_world;
		ndFloat32 m_timestep;
		ndInt32 m_modelId;
		bool m_modelAlive;
		bool m_showDebug;

		friend class ndModelUI;
	};

	class ndModelUI : public ndUIEntity
	{
		public:
		ndModelUI(ndDemoEntityManager* const scene, RobotModelNotify* const modelNotify)
			:ndUIEntity(scene)
			,m_modelNotify(modelNotify)
		{
		}

		~ndModelUI()
		{
		}

		virtual void RenderUI()
		{
		}

		virtual void RenderHelp()
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			//m_scene->Print(color, "Control panel");
			
			RobotModelNotify::ndUIControlNode* const control = m_modelNotify->m_control;

			bool change = false;
			change = change || ImGui::SliderFloat("x", &control->m_x, -D_MAX_SWING_DIST_X, D_MAX_SWING_DIST_X);
			change = change || ImGui::SliderFloat("y", &control->m_y, -0.2f, 0.1f);
			change = change || ImGui::SliderFloat("z", &control->m_z, -D_MAX_SWING_DIST_Z, D_MAX_SWING_DIST_Z );
			change = change || ImGui::SliderFloat("pitch", &control->m_pitch, -15.0f, 15.0f);
			change = change || ImGui::SliderFloat("yaw", &control->m_yaw, -20.0f, 20.0f);
			change = change || ImGui::SliderFloat("roll", &control->m_roll, -15.0f, 15.0f);
			change = change || ImGui::SliderFloat("animSpeed", &control->m_animSpeed, 0.0f, 1.0f);
			change = change || ImGui::Checkbox("enable controller", &control->m_enableController);

			if (change)
			{
				ndBodyKinematic* const body = m_modelNotify->GetModel()->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
				body->SetSleepState(false);
			}
		}

		RobotModelNotify* m_modelNotify;
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
			,m_discountFactor(0.995f)
			,m_horizon(ndFloat32(1.0f) / (ndFloat32(1.0f) - m_discountFactor))
			,m_lastEpisode(0xffffffff)
			,m_stopTraining(500 * 1000000)
			,m_modelIsTrained(false)
		{
			m_outFile = fopen("quadruped_1-vpg.csv", "wb");
			fprintf(m_outFile, "vpg\n");
			
			ndBrainAgentContinuePolicyGradient_TrainerMaster::HyperParameters hyperParameters;
			
			hyperParameters.m_maxTrajectorySteps = 1024 * 8;
			hyperParameters.m_extraTrajectorySteps = 1024 * 2;
			hyperParameters.m_discountFactor = ndReal(m_discountFactor);
			hyperParameters.m_numberOfActions = ND_AGENT_OUTPUT_SIZE;
			hyperParameters.m_numberOfObservations = ND_AGENT_INPUT_SIZE;
			
			m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>(new ndBrainAgentContinuePolicyGradient_TrainerMaster(hyperParameters));
			m_bestActor = ndSharedPtr<ndBrain>(new ndBrain(*m_master->GetPolicyNetwork()));
			m_master->SetName(CONTROLLER_NAME);
			
			auto SpawnModel = [this, scene, &modelMesh](const ndMatrix& matrix, bool debug)
			{
				ndModelArticulation* const model = CreateModel(scene, matrix, modelMesh);
				SetMaterial(model);
				model->SetNotifyCallback(new RobotModelNotify(m_master, model, debug));
				
				((RobotModelNotify*)*model->GetNotifyCallback())->ResetModel();
				return model;
			};
			
			ndSharedPtr<ndModel> visualModel (SpawnModel(matrix, true));

			ndWorld* const world = scene->GetWorld();
			world->AddModel(visualModel);
			visualModel->AddBodiesAndJointsToWorld();
			
			ndSharedPtr<ndUIEntity> quadrupedUI(new ndModelUI(scene, (RobotModelNotify*)*visualModel->GetNotifyCallback()));
			scene->Set2DDisplayRenderFunction(quadrupedUI);
			
			//ndInt32 countX = 22;
			//ndInt32 countZ = 23;
			//countX = 1;
			//countZ = 1;
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
			//		ndSharedPtr<ndModel> model (SpawnModel(location, false));
			//		world->AddModel(model);
			//		model->AddBodiesAndJointsToWorld();
			//
			//		m_models.Append(model->GetAsModelArticulation());
			//	}
			//}
			//scene->SetAcceleratedUpdate();
		}

		~TrainingUpdata()
		{
			if (m_outFile)
			{
				fclose(m_outFile);
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
				ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)body->GetNotifyCallback();
				ndDemoEntity* const ent = *notify->m_entity;
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

		class TrainingRobotBodyNotify : public ndDemoEntityNotify
		{
			public:
			TrainingRobotBodyNotify(const ndDemoEntityNotify* const src)
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

		void SetMaterial(ndModelArticulation* const robot) const
		{
			ndFixSizeArray<ndModelArticulation::ndNode*, 256> stack;
			
			stack.PushBack(robot->GetRoot());
			while (stack.GetCount())
			{
				ndModelArticulation::ndNode* const node = stack.Pop();
				ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
			
				ndShapeInstance& instanceShape = body->GetCollisionShape();
				instanceShape.m_shapeMaterial.m_userId = ndDemoContactCallback::m_modelPart;
			
				ndDemoEntityNotify* const originalNotify = (ndDemoEntityNotify*)body->GetNotifyCallback();
				ndSharedPtr<ndDemoEntity> userData (originalNotify->m_entity);
				originalNotify->m_entity = ndSharedPtr<ndDemoEntity>();
				TrainingRobotBodyNotify* const notify = new TrainingRobotBodyNotify((ndDemoEntityNotify*)body->GetNotifyCallback());
				body->SetNotifyCallback(notify);
				notify->m_entity = userData;
			
				for (ndModelArticulation::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
				{
					stack.PushBack(child);
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
#endif

	#define D_CYCLE_PERIOD		ndFloat32(4.0f)
	#define D_CYCLE_STRIDE_X	ndFloat32(0.3f)
	#define D_CYCLE_STRIDE_Z	ndFloat32(0.3f)
	#define D_CYCLE_AMPLITUDE	ndFloat32(0.27f)
	#define D_POSE_REST_POSITION_Y	ndReal(-0.3f)

	class RobotModelNotify : public ndModelNotify
	{
		class ndControllerTrainer : public ndBrainAgentContinuePolicyGradient_Trainer
		{
			public:
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
					for (ndInt32 i = 0; i < 4; i++)
					{
						m_currentPose[i] = ndVector::m_zero;
					}
				}

				ndVector GetTranslation(ndFloat32) const
				{
					return ndVector::m_zero;
				}

				void CalculatePose(ndAnimationPose& output, ndFloat32 param) const
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

						m_currentPose[i] = output[i].m_posit;
					}
				}

				mutable ndVector m_currentPose[4];
				ndFloat32 m_amp;
				ndFloat32 m_stride_x;
				ndFloat32 m_stride_z;
			};

			ndControllerTrainer(const ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master, RobotModelNotify* const robot)
				:ndBrainAgentContinuePolicyGradient_Trainer(master)
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
				m_animBlendTree->SetTime(0.0f);
				m_animBlendTree->SetTime(0.0f);
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

				//ndFloat32 angles[] = { -90.0f, -90.0f, 90.0f, 90.0f };
				//const char* effectorNames[] = { "foot_0",  "foot_1",  "foot_2",  "foot_3" };

				ndPoseGenerator* const poseGenerator = (ndPoseGenerator*)*sequence;
				for (ndInt32 i = 0; i < m_robot->m_legs.GetCount(); ++i)
				{
					ndEffectorInfo& leg = m_robot->m_legs[i];
					ndAnimKeyframe keyFrame;
					keyFrame.m_userData = &leg;
					m_animPose.PushBack(keyFrame);
					poseGenerator->AddTrack();
				}
			}

			ndFloat32 GetPoseSequence() const
			{
				ndAnimationSequencePlayer* const poseGenerator = (ndAnimationSequencePlayer*)*m_poseGenerator;
				ndFloat32 seq = poseGenerator->GetTime() / D_CYCLE_PERIOD;
				//ndTrace(("%f seq\n", seq));
				return seq;
			}

			virtual void Step() override
			{
				ndBrainAgentContinuePolicyGradient_Trainer::Step();

				ndFloat32 animSpeed = 1.0f;
				m_animBlendTree->Update(m_robot->m_timestep * animSpeed);

				ndVector veloc;
				m_animBlendTree->Evaluate(m_animPose, veloc);
			}

			RobotModelNotify* m_robot;
			ndAnimationPose m_animPose;
			ndSharedPtr<ndAnimationBlendTreeNode> m_poseGenerator;
			ndSharedPtr<ndAnimationBlendTreeNode> m_animBlendTree;
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

		void Init()
		{
			m_controllerTrainer->InitAnimation();
		}

		//RobotModelNotify(ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master, ndModelArticulation* const robot, bool showDebug)
		RobotModelNotify(ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master, ndModelArticulation* const robot)
			:ndModelNotify()
			//, m_controller(nullptr)
			,m_controllerTrainer(nullptr)
			//, m_world(nullptr)
			//, m_timestep(ndFloat32(0.0f))
			//, m_modelAlive(true)
			//, m_showDebug(showDebug)
		{
			SetModel(robot);
			m_controllerTrainer = ndSharedPtr<ndControllerTrainer>(new ndControllerTrainer(master, this));
		}

		~RobotModelNotify()
		{
		}

		bool IsTerminal() const
		{
			return false;
		}

		ndBrainFloat CalculateReward() const
		{
			return 0.0f;
		}

		void GetObservation(ndBrainFloat* const observations)
		{
			ndInt32 size = m_leg1_posit_x - m_leg0_posit_x;
			observations[m_frameTick] = m_controllerTrainer->GetPoseSequence();
			for (ndInt32 i = 0; i < m_legs.GetCount(); ++i)
			{
				ndEffectorInfo& leg = m_legs[i];
				ndJointBilateralConstraint::ndKinematicState kinematicState[4];
				leg.m_effector->GetKinematicState(kinematicState);
				observations[i * size + 0] = kinematicState[0].m_posit;
				observations[i * size + 1] = kinematicState[1].m_posit;
				observations[i * size + 2] = kinematicState[2].m_posit;
			}
		}

		void ResetModel()
		{
			//m_modelAlive = true;
			//m_control->Reset();
			//m_control->m_animSpeed = ndReal(D_MIN_TRAIN_ANIM_SPEED + (1.0f - D_MIN_TRAIN_ANIM_SPEED) * ndRand());
			//ndMemSet(m_controllerTrainer->m_rewardsMemories, ndReal(1.0), sizeof(m_controllerTrainer->m_rewardsMemories) / sizeof(m_controllerTrainer->m_rewardsMemories[0]));

			ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
			model->ClearMemory();

			//const ndMatrix matrix(model->GetRoot()->m_body->GetMatrix());
			//const ndVector& up = matrix.m_up;
			//bool state = up.m_y < D_MODEL_DEAD_ANGLE;
			//state = state || (matrix.m_posit.m_x > 20.0f);
			//state = state || (matrix.m_posit.m_x < -20.0f);
			//state = state || (matrix.m_posit.m_z > 20.0f);
			//state = state || (matrix.m_posit.m_z < -20.0f);
			//if (state)
			//{
			//	for (ndInt32 i = 0; i < m_controllerTrainer->m_basePose.GetCount(); i++)
			//	{
			//		m_controllerTrainer->m_basePose[i].SetPose();
			//	}
			//
			//	ndFloat32 duration = m_poseGenerator->GetSequence()->GetDuration();
			//
			//	ndUnsigned32 index = ndRandInt() % 4;
			//	m_animBlendTree->SetTime(0.25f * ndFloat32(index) * duration);
			//}
			//
			//ndVector veloc;
			//m_animBlendTree->Evaluate(m_animPose, veloc);
		}

		void ApplyActions(ndBrainFloat* const actions)
		{
			ndBodyKinematic* const rootBody = GetModel()->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();

			ndInt32 size = m_leg1_x - m_leg0_x;
			const ndVector upVector(rootBody->GetMatrix().m_up);
			for (ndInt32 i = 0; i < m_legs.GetCount(); ++i)
			{
				ndEffectorInfo& leg = m_legs[i];
				ndIkSwivelPositionEffector* const effector = leg.m_effector;
				
				ndVector posit(leg.m_effector->GetRestPosit());

				ndFloat32 x = actions[size * i + 0] * D_CYCLE_STRIDE_X;
				ndFloat32 z = actions[size * i + 2] * D_CYCLE_STRIDE_Z;
				ndFloat32 y = actions[size * i + 1] * D_CYCLE_AMPLITUDE;

				//if (i == 3)
				//{
				//	z = -1.0f * D_CYCLE_STRIDE_Z;
				//}

				posit.m_x += x;
				posit.m_y += y;
				posit.m_z += z;

				ndFloat32 swivelAngle = effector->CalculateLookAtSwivelAngle(upVector);
				effector->SetSwivelAngle(swivelAngle);
				effector->SetLocalTargetPosition(posit);

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
				ndAssert(0);
				//m_controller->Step();
			}
		}
		
		ndFixSizeArray<ndEffectorInfo, 4> m_legs;
		ndSharedPtr<ndControllerTrainer> m_controllerTrainer;
		ndFloat32 m_timestep;
	};

	ndModelArticulation* CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, const ndSharedPtr<ndDemoEntity>& modelMesh, ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master)
	{
		ndModelArticulation* const model = new ndModelArticulation();
		RobotModelNotify* const notify = new RobotModelNotify(master, model);
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
		ndInt32 index = 0;
		for (ndList<ndSharedPtr<ndDemoEntity>>::ndNode* node = entity->GetChildren().GetFirst(); node; node = node->GetNext())
		{
			// build thig
			index++;
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
			((ndIkSwivelPositionEffector*)*effector)->SetLinearSpringDamper(regularizer, 4000.0f, 50.0f);
			((ndIkSwivelPositionEffector*)*effector)->SetAngularSpringDamper(regularizer, 4000.0f, 50.0f);
			((ndIkSwivelPositionEffector*)*effector)->SetWorkSpaceConstraints(0.0f, 0.75f * 0.9f);
			((ndIkSwivelPositionEffector*)*effector)->SetMaxForce(effectorStrength);
			((ndIkSwivelPositionEffector*)*effector)->SetMaxTorque(effectorStrength);
			
			//ndString name("effector_");
			//name += index;
			model->AddCloseLoop(effector);

			RobotModelNotify::ndEffectorInfo leg;
			leg.m_calf = (ndJointHinge*)*calfHinge;
			leg.m_heel = (ndJointHinge*)*heelHinge;
			leg.m_thigh = (ndJointSpherical*)*ballJoint;
			leg.m_effector = (ndIkSwivelPositionEffector*)*effector;
			notify->m_legs.PushBack(leg);
		}
		notify->Init();
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
			,m_discountFactor(0.99f)
			,m_horizon(ndFloat32(1.0f) / (ndFloat32(1.0f) - m_discountFactor))
			,m_lastEpisode(0xffffffff)
			,m_stopTraining(500 * 1000000)
			,m_modelIsTrained(false)
		{
			ndWorld* const world = scene->GetWorld();

			m_outFile = fopen("ndQuadruped_2-vpg.csv", "wb");
			fprintf(m_outFile, "vpg\n");

			ndBrainAgentContinuePolicyGradient_TrainerMaster::HyperParameters hyperParameters;

			hyperParameters.m_extraTrajectorySteps = 256;
			hyperParameters.m_maxTrajectorySteps = 1024 * 2;
			hyperParameters.m_numberOfActions = m_actionsSize;
			hyperParameters.m_numberOfObservations = m_stateSize;
			hyperParameters.m_discountFactor = ndReal(m_discountFactor);

			m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>(new ndBrainAgentContinuePolicyGradient_TrainerMaster(hyperParameters));
			m_bestActor = ndSharedPtr<ndBrain>(new ndBrain(*m_master->GetPolicyNetwork()));

			m_master->SetName(CONTROLLER_NAME);

			ndSharedPtr<ndModel>visualModel(CreateModel(scene, matrix, modelMesh, m_master));
			SetMaterial(visualModel->GetAsModelArticulation());
			world->AddModel(visualModel);
			visualModel->AddBodiesAndJointsToWorld();

			ndBodyKinematic* const rootBody = visualModel->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
			ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(rootBody->GetMatrix(), rootBody, world->GetSentinelBody()));
			//world->AddJoint(fixJoint);

			// add a hidden battery of model to generate trajectories in parallel

			ndInt32 countX = 10;
			ndInt32 countZ = 10;
			//countX = 1;
			//countZ = 1;
			
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

					ndSharedPtr<ndModel>model(CreateModel(scene, location, modelMesh, m_master));
					SetMaterial(model->GetAsModelArticulation());
					world->AddModel(model);
					model->AddBodiesAndJointsToWorld();

					m_models.Append(model->GetAsModelArticulation());
				}
			}

			//scene->SetAcceleratedUpdate();
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
	matrix.m_posit.m_y = 0.5f;

	#ifdef ND_TRAIN_MODEL
		scene->RegisterPostUpdate(new TrainingUpdata(scene, matrix, modelMesh));
	#else
		ndWorld* const world = scene->GetWorld();

		ndSharedPtr<ndModel> referenceModel (CreateModel(scene, matrix, modelMesh));
		world->AddModel(referenceModel);
		referenceModel->AddBodiesAndJointsToWorld();
		
		ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(referenceModel->GetAsModelArticulation()->GetRoot()->m_body->GetMatrix(), referenceModel->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic(), world->GetSentinelBody()));
		//world->AddJoint(fixJoint);
		
		//char fileName[256];
		//ndGetWorkingFileName(CONTROLLER_NAME, fileName);
		//ndSharedPtr<ndBrain> policy(ndBrainLoad::Load(fileName));
		//referenceModel->SetNotifyCallback(new RobotModelNotify(policy, referenceModel->GetAsModelArticulation(), true));
		//
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
