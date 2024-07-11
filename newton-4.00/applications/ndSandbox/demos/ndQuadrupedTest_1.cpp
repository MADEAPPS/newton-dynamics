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

namespace ndQuadruped_1
{
	#define ND_TRAIN_MODEL
	#define CONTROLLER_NAME "ndQuadruped_1-VPG.dnn"

	class ndLegObservation
	{
		public:
		ndBrainFloat m_state[2 * 5];
		ndBrainFloat m_hasContact;
		ndBrainFloat m_animSequence;
	};

	class ndActionVector
	{
		public:
		ndBrainFloat m_x;
		ndBrainFloat m_z;
	};

	class ndObservationVector
	{
		public:
		ndLegObservation n_legs[4];
		ndActionVector m_torso;
		ndFloat32 m_animSpeed;
	};

	#define ND_AGENT_OUTPUT_SIZE	(sizeof (ndActionVector) / sizeof (ndBrainFloat))
	#define ND_AGENT_INPUT_SIZE		(sizeof (ndObservationVector) / sizeof (ndBrainFloat))

	#define D_MAX_SWING_DIST_X		ndReal(0.10f)
	#define D_MAX_SWING_DIST_Z		ndReal(0.15f)
	#define D_POSE_REST_POSITION_Y	ndReal (-0.3f)
	#define D_MIN_REWARD_ANGLE		ndReal(ndFloat32 (30.0f) * ndDegreeToRad)

	//#define D_SWING_STEP			ndReal(0.01f)
	#define D_SWING_STEP			ndReal(0.005f)

	class ndRobot : public ndModelArticulation
	{
		public:
		class ndEffectorInfo
		{
			public:
			ndEffectorInfo()
			{
			}

			ndEffectorInfo(
				const ndSharedPtr<ndJointBilateralConstraint>& thigh,
				const ndSharedPtr<ndJointBilateralConstraint>& calf,
				const ndSharedPtr<ndJointBilateralConstraint>& foot,
				const ndSharedPtr<ndJointBilateralConstraint>& effector)
				:m_thigh(thigh)
				,m_calf(calf)
				,m_foot(foot)
				,m_effector(effector)
			{
			}

			ndSharedPtr<ndJointBilateralConstraint> m_thigh;
			ndSharedPtr<ndJointBilateralConstraint> m_calf;
			ndSharedPtr<ndJointBilateralConstraint> m_foot;
			ndSharedPtr<ndJointBilateralConstraint> m_effector;
		};

		class ndPoseGenerator : public ndAnimationSequence
		{
			public:
			ndPoseGenerator(const ndFloat32* const phase)
				:ndAnimationSequence()
				,m_amp(0.27f)
				,m_stride_x(0.3f)
				,m_stride_z(-0.3f)
			{
				m_duration = ndFloat32(4.0f);
				for (ndInt32 i = 0; i < 4; i++)
				{
					m_phase[i] = phase[i];
					m_offset[i] = ndFloat32(0.0f);
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
				
				ndFloat32 ycontact = D_POSE_REST_POSITION_Y + m_amp / 2.0f;
				for (ndInt32 i = 0; i < output.GetCount(); i++)
				{
					output[i].m_userParamInt = 0;
					output[i].m_posit = BasePose(i);
				}

				for (ndInt32 i = 0; i < output.GetCount(); i++)
				//for (ndInt32 i = 0; i < 1; i++)
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
					//if ((t >= gaitGuard) && (t <= (gaitFraction - gaitGuard)))
					if (t <= (gaitFraction - gaitGuard))
					{
						output[i].m_posit.m_y += m_amp * ndSin(omega * t);
						output[i].m_userParamInt = output[i].m_posit.m_y < ycontact ? -1 : 1;

						ndFloat32 num = t;
						ndFloat32 den = gaitFraction - gaitGuard;

						ndFloat32 t0 = num / den;
						//output[i].m_posit.m_x += stride_x * t0 - stridem_x * 0.5f;
						output[i].m_posit.m_z += -(stride_z * t0 - stride_z * 0.5f);
					}
					else
					{
						//if (t <= gaitGuard)
						//{
						//	t += 1.0f;
						//}
						
						ndFloat32 num = t - (gaitFraction - gaitGuard);
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

		// implement controller player
		class ndController : public ndBrainAgentContinuePolicyGradient<ND_AGENT_INPUT_SIZE, ND_AGENT_OUTPUT_SIZE>
		{
			public:
			ndController(ndSharedPtr<ndBrain>& actor)
				:ndBrainAgentContinuePolicyGradient<ND_AGENT_INPUT_SIZE, ND_AGENT_OUTPUT_SIZE>(actor)
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

		class ndControllerAgent_trainer : public ndBrainAgentContinuePolicyGradient_Trainer<ND_AGENT_INPUT_SIZE, ND_AGENT_OUTPUT_SIZE>
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

			ndControllerAgent_trainer(ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster<ND_AGENT_INPUT_SIZE, ND_AGENT_OUTPUT_SIZE>>& master)
				:ndBrainAgentContinuePolicyGradient_Trainer<ND_AGENT_INPUT_SIZE, ND_AGENT_OUTPUT_SIZE>(master)
				,m_basePose()
				,m_model(nullptr)
			{
			}

			~ndControllerAgent_trainer()
			{
			}

			void SetModel(ndRobot* const model)
			{
				m_model = model;
				for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
				{
					m_basePose.PushBack(node->m_body->GetAsBodyDynamic());
				}
			}

			ndBrainFloat CalculateReward()
			{
				for (ndInt32 i = sizeof(m_rewardsMemories) / sizeof(m_rewardsMemories[0]) - 1; i >= 1; --i)
				{
					m_rewardsMemories[i] = m_rewardsMemories[i - 1];
				}
				m_rewardsMemories[0] = m_model->CalculateReward();
				return m_rewardsMemories[0];
			}

			virtual void ApplyActions(ndBrainFloat* const actions)
			{
				m_model->ApplyActions(actions);
			}

			void GetObservation(ndBrainFloat* const observation)
			{
				m_model->GetObservation(observation);
			}

			bool IsTerminal() const
			{
				ndMatrix matrix(m_model->GetRoot()->m_body->GetMatrix());
				ndVector up(matrix.m_up);
				if (up.m_y < 0.2f)
				{
					return true;
				}

				bool isDead = true;
				for (ndInt32 i = sizeof(m_rewardsMemories) / sizeof(m_rewardsMemories[0]) - 1; i >= 0; --i)
				{
					isDead &= m_rewardsMemories[i] < ndReal(0.05f);
				}
				if (isDead)
				{
					return isDead;
				}

				//ndInt32 count = 0;
				//ndInt32 isGround = 4;
				//bool airborneLeg[4];
				//bool sequenceAirborne[4];
				//for (ndInt32 i = 0; i < m_model->m_animPose.GetCount(); ++i)
				//{
				//	ndContact* const contact = m_model->FindContact(i);
				//	bool isAirborne = !(contact && contact->IsActive());
				//	isGround -= ndInt32(isAirborne);
				//
				//	const ndAnimKeyframe& keyFrame = m_model->m_animPose[i];
				//	if (keyFrame.m_userParamInt != 0)
				//	{
				//		airborneLeg[count] = isAirborne;
				//		if (keyFrame.m_userParamInt == 0)
				//		{
				//			isAirborne = false;
				//		}
				//		else if (keyFrame.m_userParamInt == 1)
				//		{
				//			isAirborne = true;
				//		}
				//		sequenceAirborne[count] = isAirborne;
				//
				//		count++;
				//	}
				//}
				//if (isGround < 2)
				//{
				//	return false;
				//}
				//
				//for (ndInt32 i = 0; i < count; ++i)
				//{
				//	if (airborneLeg[i] != sequenceAirborne[i])
				//	{
				//		//return true;
				//	}
				//}

				return false;
			}

			void ResetModel()
			{
				m_model->m_control->Reset();
				ndMemSet(m_rewardsMemories, ndReal(1.0), sizeof(m_rewardsMemories) / sizeof(m_rewardsMemories[0]));


				ndMatrix matrix(m_model->GetRoot()->m_body->GetMatrix());
				ndVector up(matrix.m_up);
				if (up.m_y < 0.2f)
				{
					for (ndInt32 i = 0; i < m_basePose.GetCount(); i++)
					{
						m_basePose[i].SetPose();
					}

					ndFloat32 randVar = ndRand();
					//randVar = randVar * randVar;
					//randVar = randVar * randVar * randVar;
					m_model->m_control->m_animSpeed = randVar;

					ndUnsigned32 index = 0;
					//ndUnsigned32 index = ndRandInt() % 4;
					ndFloat32 duration = m_model->m_poseGenerator->GetSequence()->GetDuration();
					m_model->m_animBlendTree->SetTime(ndFloat32(index) * duration * 0.25f);
				}
			}

			ndFixSizeArray<ndBasePose, 32> m_basePose;
			ndRobot* m_model;
			ndReal m_rewardsMemories[32];
		};

		ndRobot(ndSharedPtr<ndBrainAgent>& agent)
			:ndModelArticulation()
			,m_animPose()
			,m_control(nullptr)
			,m_poseGenerator(nullptr)
			,m_effectorsInfo()
			,m_animBlendTree()
			,m_agent(agent)
			,m_timestep(0.0f)
			,m_showDebug(false)
		{
		}

		void Debug(ndConstraintDebugCallback& context) const
		{
			if (!m_showDebug)
			{
				return;
			}
			ndBodyKinematic* const rootBody = GetRoot()->m_body->GetAsBodyKinematic();
			ndFixSizeArray<const ndBodyKinematic*, 32> bodies;
			
			ndFloat32 totalMass = ndFloat32(0.0f);
			ndVector centerOfMass(ndVector::m_zero);
			for (ndModelArticulation::ndNode* node = GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
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

				if (keyFrame.m_userParamInt == 0)
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

			CalculateReward();
		}

		void UpdatePose(ndFloat32 timestep)
		{
			ndBodyKinematic* const rootBody = GetRoot()->m_body->GetAsBodyKinematic();
			ndSkeletonContainer* const skeleton = rootBody->GetSkeleton();
			ndAssert(skeleton);
			ndFixSizeArray<ndJointBilateralConstraint*, 32> effectors;

			ndVector veloc;
			m_animBlendTree->Evaluate(m_animPose, veloc);

			const ndVector upVector(rootBody->GetMatrix().m_up);
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

			m_invDynamicsSolver.SolverBegin(skeleton, &effectors[0], effectors.GetCount(), m_world, timestep);
			m_invDynamicsSolver.Solve();
			m_invDynamicsSolver.SolverEnd();
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
			//m_control->m_animSpeed = 4.0f;
			//m_control->m_animSpeed = 2.0f;
			//m_control->m_animSpeed = 1.0f;
			//m_control->m_animSpeed = 0.5f;
			m_control->m_animSpeed = 0.25f;
			//m_control->m_animSpeed = 0.1f;

			//m_control->m_enableController = 0;

			if (m_control->m_enableController)
			{
				const ndActionVector& actionVector = *((ndActionVector*)actions);
				m_control->m_x = ndClamp(ndReal(m_control->m_x + actionVector.m_x * D_SWING_STEP), -D_MAX_SWING_DIST_X, D_MAX_SWING_DIST_X);
				m_control->m_z = ndClamp(ndReal(m_control->m_z + actionVector.m_z * D_SWING_STEP), -D_MAX_SWING_DIST_Z, D_MAX_SWING_DIST_Z);

				//m_control->m_x = 0;
				//m_control->m_z = 0;
			}
			
			UpdatePose(m_timestep);
		}

		void GetObservation(ndBrainFloat* const observationInput)
		{
			ndObservationVector& observation = *((ndObservationVector*)observationInput);
			for (ndInt32 i = 0; i < m_animPose.GetCount(); ++i)
			{
				const ndAnimKeyframe& keyFrame = m_animPose[i];
				const ndEffectorInfo* const info = (ndEffectorInfo*)keyFrame.m_userData;

				ndInt32 paramCount = 0;
				ndJointBilateralConstraint::ndKinematicState kinematicState[16];
				paramCount += info->m_thigh->GetKinematicState(&kinematicState[paramCount]);
				paramCount += info->m_calf->GetKinematicState(&kinematicState[paramCount]);
				paramCount += info->m_foot->GetKinematicState(&kinematicState[paramCount]);
				for (ndInt32 j = 0; j < paramCount; ++j)
				{
					observation.n_legs[i].m_state[j * 2 + 0] = ndBrainFloat(kinematicState[j].m_posit);
					observation.n_legs[i].m_state[j * 2 + 1] = ndBrainFloat(kinematicState[j].m_velocity);
				}

				observation.n_legs[i].m_hasContact = ndBrainFloat(FindContact(i) ? 1.0f : 0.0f);
				observation.n_legs[i].m_animSequence = ndBrainFloat(keyFrame.m_userParamInt ? 1.0f : 0.0f);
			}

			observation.m_torso.m_x = m_control->m_x;
			observation.m_torso.m_z = m_control->m_z;
			observation.m_animSpeed = m_control->m_animSpeed;
		}

		ndVector CalculateZeroMomentPoint() const
		{
			ndFixSizeArray<ndVector, 32> r;
			ndFixSizeArray<const ndBodyKinematic*, 32> bodies;

			ndVector com(ndVector::m_zero);
			ndFloat32 totalMass = ndFloat32(0.0f);
			for (ndModelArticulation::ndNode* node = GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
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
				shape[i] = origin + (shape[i] - origin).Scale (scale);
			}
		}

		ndBrainFloat CalculateZeroMomentPointReward() const
		{
			ndFixSizeArray<ndBigVector, 16> desiredSupportPoint;
			for (ndInt32 i = 0; i < m_animPose.GetCount(); ++i)
			{
				const ndAnimKeyframe& keyFrame = m_animPose[i];
				ndEffectorInfo* const info = (ndEffectorInfo*)keyFrame.m_userData;
				ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;

				if (keyFrame.m_userParamInt == 0)
				{
					ndBodyKinematic* const body = effector->GetBody0();
					desiredSupportPoint.PushBack(ndBigVector(body->GetMatrix().TransformVector(effector->GetLocalMatrix0().m_posit)));
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
				ndAssert(0);
			}

			return reward;
		}

		ndBrainFloat CalculateZeroOmegaReward() const
		{
			ndFixSizeArray<const ndBodyKinematic*, 32> bodies;

			ndVector com(ndVector::m_zero);
			ndFloat32 totalMass = ndFloat32(0.0f);
			for (ndModelArticulation::ndNode* node = GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				const ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();

				const ndMatrix matrix(body->GetMatrix());
				const ndVector bodyCom(matrix.TransformVector(body->GetCentreOfMass()));
				ndFloat32 mass = body->GetMassMatrix().m_w;
				totalMass += mass;
				com += matrix.TransformVector(body->GetCentreOfMass()).Scale(mass);
				bodies.PushBack(body);
			}
			com = com.Scale(1.0f / totalMass);
			com.m_w = 1.0f;

			ndMatrix referenceFrame(ndGetIdentityMatrix());
			referenceFrame.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
			referenceFrame.m_front = GetRoot()->m_body->GetMatrix().m_front;
			referenceFrame.m_right = referenceFrame.m_front.CrossProduct(referenceFrame.m_up).Normalize();
			referenceFrame.m_front = referenceFrame.m_up.CrossProduct(referenceFrame.m_right).Normalize();
			referenceFrame.m_posit = com;

			ndMatrix inetia(ndGetZeroMatrix());
			ndVector angularMomentum(ndVector::m_zero);

			const ndMatrix invReferenceFrame(referenceFrame.OrthoInverse());
			const ndVector gravity(ndFloat32(0.0f), DEMO_GRAVITY, ndFloat32(0.0f), ndFloat32(0.0f));
			for (ndInt32 i = 0; i < bodies.GetCount(); ++i)
			{
				const ndBodyKinematic* const body = bodies[i];
				const ndMatrix& bodyMatrix = body->GetMatrix();

				ndVector origin(bodyMatrix.TransformVector(body->GetCentreOfMass()));
				ndVector posit(referenceFrame.UntransformVector(origin) & ndVector::m_triplexMask);
				ndMatrix covariance(ndCovarianceMatrix(posit, posit));
				ndFloat32 mag2 = posit.DotProduct(posit).GetScalar();
				ndFloat32 mass(body->GetMassMatrix().m_w);

				const ndMatrix bodyInertia(body->CalculateInertiaMatrix());
				ndMatrix refInertia(referenceFrame * bodyInertia * invReferenceFrame);
				for (ndInt32 j = 0; j < 3; j++)
				{
					refInertia[j][j] += mass * mag2;
					refInertia[j] -= covariance[j].Scale(mass);
					inetia[j] += refInertia[j];
				}

				ndVector L(origin.CrossProduct(body->GetVelocity()) + bodyInertia.RotateVector(body->GetOmega()));
				angularMomentum += referenceFrame.UnrotateVector(L);
				//ndTrace(("%f %f\n", L.m_x, L.m_z));
			}
			inetia.m_posit.m_w = 1.0f;
			inetia = inetia.Inverse4x4();
			ndVector localOmega(inetia.RotateVector(angularMomentum));
			ndFloat32 omegaMag2 = localOmega.m_x * localOmega.m_x + localOmega.m_z * localOmega.m_z;
			ndFloat32 speedRewardProb = ndBrainFloat(ndExp(-ndBrainFloat(1.0f) * omegaMag2));

			const ndMatrix matrix(GetRoot()->m_body->GetMatrix());
			ndFloat32 upAngle = ndAcos(ndClamp(matrix.m_up.m_y, ndFloat32(-1.0f), ndFloat32(1.0f)));
			ndFloat32 angleRewardProb = ndBrainFloat(ndExp(-ndBrainFloat(100.0f) * upAngle * upAngle));

			//ndTrace(("s(%f) a(%f) angle(%f)\n", speedReward, angleReward, upAngle * ndRadToDegree));
			return angleRewardProb * speedRewardProb;
		}

		ndBrainFloat CalculateReward() const
		{
			//ndBrainFloat reward0 = CalculateZeroOmegaReward();
			ndBrainFloat reward1 = CalculateZeroMomentPointReward();
			//ndTrace(("zeroOmega(%f) ZeroMoment(%f)\n", reward0, reward1));
			return reward1;
		}

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			ndModelArticulation::Update(world, timestep);

			static int xxx; 
			ndTrace(("step(%d) reward(%f)\n", xxx++, CalculateReward()));
			
			m_timestep = timestep;
			m_agent->Step();
		}

		void PostUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			//ndFloat32 animSpeed = (m_control->m_animSpeed > 0.01f) ? (1.0f + 1.0f * m_control->m_animSpeed) : 0.0f;
			ndFloat32 animSpeed = 2.0f * m_control->m_animSpeed;
			m_animBlendTree->Update(timestep * animSpeed);
			ndModelArticulation::PostUpdate(world, timestep);
		}

		ndAnimationPose m_animPose;
		ndUIControlNode* m_control;
		ndAnimationSequencePlayer* m_poseGenerator;
		ndFixSizeArray<ndEffectorInfo, 4> m_effectorsInfo;
		ndSharedPtr<ndAnimationBlendTreeNode> m_animBlendTree;
		ndSharedPtr<ndBrainAgent> m_agent;
		ndFloat32 m_timestep;
		bool m_showDebug;
	};

	class ndModelUI : public ndUIEntity
	{
		public:
		ndModelUI(ndDemoEntityManager* const scene, const ndSharedPtr<ndModel>& quadruped)
			:ndUIEntity(scene)
			,m_model(quadruped)
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

			ndRobot* const model = (ndRobot*)*m_model;
			ndRobot::ndUIControlNode* const control = model->m_control;

			bool change = false;
			change = change || ImGui::SliderFloat("x", &control->m_x, -D_MAX_SWING_DIST_X, D_MAX_SWING_DIST_X);
			change = change || ImGui::SliderFloat("y", &control->m_y, -0.2f, 0.1f);
			change = change || ImGui::SliderFloat("z", &control->m_z, -D_MAX_SWING_DIST_Z, D_MAX_SWING_DIST_Z);
			change = change || ImGui::SliderFloat("pitch", &control->m_pitch, -15.0f, 15.0f);
			change = change || ImGui::SliderFloat("yaw", &control->m_yaw, -20.0f, 20.0f);
			change = change || ImGui::SliderFloat("roll", &control->m_roll, -15.0f, 15.0f);
			change = change || ImGui::SliderFloat("animSpeed", &control->m_animSpeed, 0.0f, 1.0f);
			change = change || ImGui::Checkbox("enable controller", &control->m_enableController);

			if (change)
			{
				ndBodyKinematic* const body = m_model->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
				body->SetSleepState(false);
			}
		}

		ndSharedPtr<ndModel> m_model;
	};

	ndModelArticulation* BuildModel(ndDemoEntityManager* const scene, const ndMatrix& matrixLocation, ndSharedPtr<ndBrainAgent> agent)
	{
		ndFloat32 mass = 20.0f;
		ndFloat32 radius = 0.25f;
		ndFloat32 limbMass = 0.25f;
		ndFloat32 limbLength = 0.3f;
		ndFloat32 limbRadios = 0.05f;

		ndRobot* const model = new ndRobot(agent);

		ndPhysicsWorld* const world = scene->GetWorld();
		//ndVector floor(FindFloor(*world, matrixLocation.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		ndVector floor(matrixLocation.m_posit);
		ndSharedPtr<ndBody> torso(world->GetBody(AddSphere(scene, matrixLocation, mass, radius, "smilli.png")));
		ndModelArticulation::ndNode* const modelRoot = model->AddRootBody(torso);
		
		ndMatrix location(matrixLocation);
		location.m_posit.m_y = floor.m_y + 0.5f;
		//location.m_posit.m_y += 1.5f;
		torso->SetMatrix(location);
		
		ndDemoEntity* const entity = (ndDemoEntity*)torso->GetNotifyCallback()->GetUserData();
		entity->SetMeshMatrix(ndYawMatrix(90.0f * ndDegreeToRad) * ndPitchMatrix(90.0f * ndDegreeToRad));
		
		ndMatrix matrix(ndRollMatrix(45.0f * ndDegreeToRad));
		matrix.m_posit.m_x = radius * 0.9f;
		matrix.m_posit.m_y = -radius * 0.5f;
		
		ndFloat32 angles[] = { 300.0f, 240.0f, 120.0f, 60.0f };
		ndFloat32 offset_x[] = { 0.2f, 0.2f, 0.2f, 0.2f };
		ndFloat32 offset_z[] = { -0.3f, 0.3f, -0.3f, 0.3f };
		ndFloat32 offset_y[] = { D_POSE_REST_POSITION_Y, D_POSE_REST_POSITION_Y, D_POSE_REST_POSITION_Y, D_POSE_REST_POSITION_Y };
		
		//ndFloat32 phase[] = { 0.0f, 0.25f, 0.5f, 0.75f };
		ndFloat32 phase[] = { 0.0f, 0.75f, 0.25f, 0.5f};
		ndSharedPtr<ndAnimationSequence> sequence(new ndRobot::ndPoseGenerator(phase));
		
		model->m_poseGenerator = new ndAnimationSequencePlayer(sequence);
		model->m_control = new ndRobot::ndUIControlNode(model->m_poseGenerator);
		model->m_animBlendTree = ndSharedPtr<ndAnimationBlendTreeNode>(model->m_control);
		
		ndRobot::ndPoseGenerator* const poseGenerator = (ndRobot::ndPoseGenerator*)*sequence;

		for (ndInt32 i = 0; i < 4; ++i)
		{
			ndMatrix limbPivotLocation(matrix * ndYawMatrix(angles[i] * ndDegreeToRad));
			limbPivotLocation.m_posit += torso->GetMatrix().m_posit;
			limbPivotLocation.m_posit.m_w = 1.0f;
		
			// add leg thigh
			const ndVector thighPivot(limbPivotLocation.m_posit);
		
			ndFloat32 workSpace = 0.0f;
			ndModelArticulation::ndNode* thighNode = nullptr;
			{
				ndMatrix bodyMatrix(limbPivotLocation);
				bodyMatrix.m_posit += limbPivotLocation.m_front.Scale(limbLength * 0.5f);
				ndSharedPtr<ndBody> thigh(world->GetBody(AddCapsule(scene, bodyMatrix, limbMass, limbRadios, limbRadios, limbLength)));
				thigh->SetMatrix(bodyMatrix);
				ndSharedPtr<ndJointBilateralConstraint> ballJoint(new ndIkJointSpherical(limbPivotLocation, thigh->GetAsBodyKinematic(), torso->GetAsBodyKinematic()));
				thighNode = model->AddLimb(modelRoot, thigh, ballJoint);
		
				limbPivotLocation.m_posit += limbPivotLocation.m_front.Scale(limbLength);
		
				workSpace += limbLength;
			}
		
			// add calf0
			ndModelArticulation::ndNode* calfNode = nullptr;
			{
				limbPivotLocation = ndRollMatrix(-90.0f * ndDegreeToRad) * limbPivotLocation;
		
				ndMatrix bodyMatrix(limbPivotLocation);
				bodyMatrix.m_posit += limbPivotLocation.m_front.Scale(limbLength * 0.5f);
				ndSharedPtr<ndBody> calf0(world->GetBody(AddCapsule(scene, bodyMatrix, limbMass * 0.25f, limbRadios, limbRadios, limbLength)));
				calf0->SetMatrix(bodyMatrix);
		
				ndMatrix caffPinAndPivotFrame(ndGetIdentityMatrix());
				ndFloat32 sign = angles[i] > 180.0f ? -1.0f : 1.0f;
				caffPinAndPivotFrame.m_front = limbPivotLocation.m_right.Scale(sign);
				caffPinAndPivotFrame.m_up = limbPivotLocation.m_front;
				caffPinAndPivotFrame.m_right = caffPinAndPivotFrame.m_front.CrossProduct(caffPinAndPivotFrame.m_up);
				caffPinAndPivotFrame.m_posit = limbPivotLocation.m_posit;
				ndSharedPtr<ndJointBilateralConstraint> hingeJoint(new ndIkJointHinge(caffPinAndPivotFrame, calf0->GetAsBodyKinematic(), thighNode->m_body->GetAsBodyKinematic()));
		
				// add joint limit to prevent knee from flipping
				ndIkJointHinge* const hinge = (ndIkJointHinge*)*hingeJoint;
				hinge->SetLimitState(true);
				hinge->SetLimits(-70.0f * ndDegreeToRad, 70.0f * ndDegreeToRad);
				calfNode = model->AddLimb(thighNode, calf0, hingeJoint);
		
				limbPivotLocation.m_posit += limbPivotLocation.m_front.Scale(limbLength);
				workSpace += limbLength;
			}
		
			// add calf1
			ndModelArticulation::ndNode* footNode = nullptr;
			{
				ndFloat32 lenght = limbLength * 0.5f;
				limbPivotLocation = ndRollMatrix(-45.0f * ndDegreeToRad) * limbPivotLocation;
				ndMatrix bodyMatrix(limbPivotLocation);
				bodyMatrix.m_posit += limbPivotLocation.m_front.Scale(lenght * 0.5f);
		
				ndSharedPtr<ndBody> foot(world->GetBody(AddCapsule(scene, bodyMatrix, limbMass * 0.25f, limbRadios, limbRadios, lenght)));
				foot->SetMatrix(bodyMatrix);
		
				// set a Material with zero restitution for the feet
				ndShapeInstance& instanceShape = foot->GetAsBodyDynamic()->GetCollisionShape();
				instanceShape.m_shapeMaterial.m_userId = ndDemoContactCallback::m_frictionTest;
		
				ndMatrix footPinAndPivotFrame(ndGetIdentityMatrix());
				footPinAndPivotFrame.m_front = limbPivotLocation.m_right;
				footPinAndPivotFrame.m_up = limbPivotLocation.m_front.Scale(-1.0f);
				footPinAndPivotFrame.m_right = footPinAndPivotFrame.m_front.CrossProduct(footPinAndPivotFrame.m_up);
				footPinAndPivotFrame.m_posit = limbPivotLocation.m_posit;
		
				// add joint limit to prevent knee from flipping
				ndJointHinge* const footHinge = new ndJointHinge(footPinAndPivotFrame, foot->GetAsBodyKinematic(), calfNode->m_body->GetAsBodyKinematic());
				//footHinge->SetLimitState(true);
				//footHinge->SetLimits(-20.0f * ndDegreeToRad, 20.0f * ndDegreeToRad);
				footHinge->SetAsSpringDamper(0.001f, 2000.0f, 50.0f);
				footNode = model->AddLimb(calfNode, foot, ndSharedPtr<ndJointBilateralConstraint>(footHinge));
		
				limbPivotLocation.m_posit += limbPivotLocation.m_front.Scale(lenght);
				workSpace += lenght;
			}
		
			// add leg effector
			{
				ndBodyKinematic* const targetBody = footNode->m_body->GetAsBodyKinematic();
		
				ndFloat32 angle(i < 2 ? -90.0f : 90.0f);
				ndMatrix effectorToeFrame(ndGetIdentityMatrix());
				ndMatrix effectorRefFrame(ndYawMatrix(angle * ndDegreeToRad));
				effectorRefFrame.m_posit = thighPivot;
				effectorToeFrame.m_posit = limbPivotLocation.m_posit;
		
				ndFloat32 regularizer = 0.001f;
				ndIkSwivelPositionEffector* const effector = new ndIkSwivelPositionEffector(effectorToeFrame.m_posit, effectorRefFrame, targetBody, torso->GetAsBodyKinematic());
				effector->SetLinearSpringDamper(regularizer, 4000.0f, 50.0f);
				effector->SetAngularSpringDamper(regularizer, 4000.0f, 50.0f);
				effector->SetWorkSpaceConstraints(0.0f, workSpace * 0.9f);
		
				ndRobot::ndEffectorInfo info(thighNode->m_joint, calfNode->m_joint, footNode->m_joint, ndSharedPtr<ndJointBilateralConstraint>(effector));
				model->m_effectorsInfo.PushBack(info);
				
				ndAnimKeyframe keyFrame;
				keyFrame.m_userData = &model->m_effectorsInfo[model->m_effectorsInfo.GetCount() - 1];
				model->m_animPose.PushBack(keyFrame);
				poseGenerator->AddTrack();
				poseGenerator->m_phase[i] = phase[i];
				poseGenerator->m_offset[i].m_x = offset_x[i];
				poseGenerator->m_offset[i].m_y = offset_y[i];
				poseGenerator->m_offset[i].m_z = offset_z[i];
			}
		}
		
		#ifdef ND_TRAIN_MODEL
			((ndRobot::ndControllerAgent_trainer*)*agent)->SetModel(model);
			((ndRobot::ndControllerAgent_trainer*)*agent)->ResetModel();
		#else
			((ndRobot::ndController*)*agent)->SetModel(model);
		#endif

		return model;
	}

	#ifdef ND_TRAIN_MODEL

	class TrainingUpdata : public ndDemoEntityManager::OnPostUpdate
	{
		public:
		TrainingUpdata(ndDemoEntityManager* const scene, const ndMatrix& matrix)
			:OnPostUpdate()
			,m_master()
			,m_bestActor()
			,m_outFile(nullptr)
			,m_timer(ndGetTimeInMicroseconds())
			,m_maxScore(ndFloat32 (-1.0e10f))
			,m_lastEpisode(-1)
			,m_stopTraining(200 * 1000000)
			,m_modelIsTrained(false)
		{
			ndWorld* const world = scene->GetWorld();
	
			m_outFile = fopen("quadruped_1-VPG.csv", "wb");
			fprintf(m_outFile, "vpg\n");

			ndInt32 countX = 6;
			ndInt32 countZ = 9;
			//countX = 0;
			//countZ = 0;

			ndBrainAgentContinuePolicyGradient_TrainerMaster<ND_AGENT_INPUT_SIZE, ND_AGENT_OUTPUT_SIZE>::HyperParameters hyperParameters;
			
			//hyperParameters.m_threadsCount = 1;
			//hyperParameters.m_sigma = ndReal(0.25f);
			hyperParameters.m_discountFactor = ndReal(0.99f);
			//hyperParameters.m_maxTrajectorySteps = 1024 * 6;
			hyperParameters.m_maxTrajectorySteps = 1024 * 8;

			m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster<ND_AGENT_INPUT_SIZE, ND_AGENT_OUTPUT_SIZE>> (new ndBrainAgentContinuePolicyGradient_TrainerMaster<ND_AGENT_INPUT_SIZE, ND_AGENT_OUTPUT_SIZE>(hyperParameters));
			m_bestActor = ndSharedPtr<ndBrain> (new ndBrain(*m_master->GetActor()));

			m_master->SetName(CONTROLLER_NAME);

			ndSharedPtr<ndBrainAgent> visualAgent(new ndRobot::ndControllerAgent_trainer(m_master));
			ndSharedPtr<ndModel> visualModel(BuildModel(scene, matrix, visualAgent));
			world->AddModel(visualModel);
			SetMaterial(visualModel);

			ndSharedPtr<ndUIEntity> quadrupedUI(new ndModelUI(scene, visualModel));
			scene->Set2DDisplayRenderFunction(quadrupedUI);

			// add a hidden battery of model to generate trajectories in parallel
			for (ndInt32 i = 0; i < countZ; ++i)
			{
				for (ndInt32 j = 0; j < countX; ++j)
				{
					ndMatrix location(matrix);
					location.m_posit.m_x += 6.0f * (ndRand() - 0.5f);
					location.m_posit.m_z += 6.0f * (ndRand() - 0.5f);
					ndSharedPtr<ndBrainAgent> agent(new ndRobot::ndControllerAgent_trainer(m_master));
					ndSharedPtr<ndModel> model(BuildModel(scene, location, agent));
					world->AddModel(model);
					m_models.Append(model);
					//HideModel(model);
					SetMaterial(model);
				}
			}
			scene->SetAcceleratedUpdate();
			((ndRobot*)*visualModel)->m_showDebug = true;
		}

		~TrainingUpdata()
		{
			if (m_outFile)
			{
				fclose(m_outFile);
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

		void OnDebug(ndDemoEntityManager* const, bool mode)
		{
			for (ndList<ndSharedPtr<ndModel>>::ndNode* node = m_models.GetFirst(); node; node = node->GetNext())
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
				TrainingRobotBodyNotify* const notify = new TrainingRobotBodyNotify((TrainingRobotBodyNotify*)body->GetNotifyCallback());
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

			if ((stopTraining >= m_stopTraining) || (m_master->GetAverageScore() > ndFloat32(99.5f)))
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

		ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster<ND_AGENT_INPUT_SIZE, ND_AGENT_OUTPUT_SIZE>> m_master;
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

	ndSharedPtr<ndBrainAgent> BuildAgent()
	{
		char fileName[1024];
		ndGetWorkingFileName(CONTROLLER_NAME, fileName);
		ndSharedPtr<ndBrain> actor(ndBrainLoad::Load(fileName));
		ndSharedPtr<ndBrainAgent> agent(new ndRobot::ndController(actor));
		return agent;
	}

	#endif
}

using namespace ndQuadruped_1;

void ndQuadrupedTest_1(ndDemoEntityManager* const scene)
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

	ndVector origin1(0.0f, 0.0f, 0.0f, 1.0f);
	ndMatrix matrix(ndYawMatrix(-0.0f * ndDegreeToRad));
	#ifdef ND_TRAIN_MODEL
		TrainingUpdata* const trainer = new TrainingUpdata(scene, matrix);
		scene->RegisterPostUpdate(trainer);
	#else
		ndWorld* const world = scene->GetWorld();
		ndSharedPtr<ndModel> model(BuildModel(scene, matrix, BuildAgent()));
		world->AddModel(model);
		((ndRobot*)(*model))->m_showDebug = true;

		ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(model->GetAsModelArticulation()->GetRoot()->m_body->GetMatrix(), model->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic(), world->GetSentinelBody()));
		//world->AddJoint(fixJoint);

		ndSharedPtr<ndUIEntity> quadrupedUI(new ndModelUI(scene, model));
		scene->Set2DDisplayRenderFunction(quadrupedUI);
	#endif
	
	matrix.m_posit.m_x -= 8.0f;
	matrix.m_posit.m_y += 1.5f;
	matrix.m_posit.m_z += 0.25f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);

	//ndFileFormatSave xxxx;
	//xxxx.SaveWorld(scene->GetWorld(), "xxxx.nd");
}
