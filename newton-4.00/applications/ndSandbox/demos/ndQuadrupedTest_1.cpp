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
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

namespace ndQuadruped_1
{
	#define ND_TRAIN_MODEL

	#define CONTROLLER_NAME "ndQuadruped_1VPG.dnn"

	#define D_MAX_SWING_DIST_X		ndReal(0.10f)
	#define D_MAX_SWING_DIST_Z		ndReal(0.15f)
	#define D_POSE_REST_POSITION_Y	ndReal (-0.3f)
	#define D_MIN_REWARD_ANGLE		ndReal(ndFloat32 (30.0f) * ndDegreeToRad)

	#define D_SWING_STEP			ndReal(0.005f)
	#define D_EFFECTOR_STEP			ndReal(0.1f)

#if 0
	enum ndActionSpace
	{
		m_leg0_action_posit_x,
		m_leg0_action_posit_y,
		m_leg0_action_posit_z,
		//m_leg0_action_posit_swivel,

		m_leg1_action_posit_x,
		m_leg1_action_posit_y,
		m_leg1_action_posit_z,
		//m_leg1_action_posit_swivel,
		
		m_leg2_action_posit_x,
		m_leg2_action_posit_y,
		m_leg2_action_posit_z,
		//m_leg2_action_posit_swivel,
		
		m_leg3_action_posit_x,
		m_leg3_action_posit_y,
		m_leg3_action_posit_z,
		//m_leg3_action_posit_swivel,

		m_actionsSize
	};

	enum ndObservationSpace
	{
		m_leg0_anim_posit_x,
		m_leg0_anim_posit_y,
		m_leg0_anim_posit_z,
		//m_leg0_anim_posit_swivel,
		m_leg0_posit_x,
		m_leg0_posit_y,
		m_leg0_posit_z,
		//m_leg0_posit_swivel,
		m_leg0_veloc_x,
		m_leg0_veloc_y,
		m_leg0_veloc_z,
		//m_leg0_veloc_swivel,
		
		m_leg1_anim_posit_x,
		m_leg1_anim_posit_y,
		m_leg1_anim_posit_z,
		//m_leg1_anim_posit_swivel,
		m_leg1_posit_x,
		m_leg1_posit_y,
		m_leg1_posit_z,
		//m_leg1_posit_swivel,
		m_leg1_veloc_x,
		m_leg1_veloc_y,
		m_leg1_veloc_z,
		//m_leg1_veloc_swivel,
		
		m_leg2_anim_posit_x,
		m_leg2_anim_posit_y,
		m_leg2_anim_posit_z,
		//m_leg2_anim_posit_swivel,
		m_leg2_posit_x,
		m_leg2_posit_y,
		m_leg2_posit_z,
		//m_leg2_posit_swivel,
		m_leg2_veloc_x,
		m_leg2_veloc_y,
		m_leg2_veloc_z,
		//m_leg2_veloc_swivel,
		
		m_leg3_anim_posit_x,
		m_leg3_anim_posit_y,
		m_leg3_anim_posit_z,
		//m_leg3_anim_posit_swivel,
		m_leg3_posit_x,
		m_leg3_posit_y,
		m_leg3_posit_z,
		//m_leg3_posit_swivel,
		m_leg3_veloc_x,
		m_leg3_veloc_y,
		m_leg3_veloc_z,
		//m_leg3_veloc_swivel,

		m_observationsSize
	};

	class ndRobot: public ndModelArticulation
	{
		public:
		class ndBodyState
		{
			public:
			ndBodyState()
				:m_com(ndGetZeroMatrix())
				,m_inertia(ndGetZeroMatrix())
				,m_linearMomentum(ndVector::m_zero)
				,m_angularMomentum(ndVector::m_zero)
				,m_mass(0.0f)
			{
				m_inertia[3][3] = ndFloat32(1.0f);
			}

			ndMatrix m_com;
			ndMatrix m_inertia;
			ndVector m_linearMomentum;
			ndVector m_angularMomentum;
			ndFloat32 m_mass;
		};

		class ndEffectorInfo
		{
			public:
			ndEffectorInfo()
				:m_footHinge(nullptr)
				,m_effector(nullptr)
			{
			}

			ndEffectorInfo(const ndSharedPtr<ndJointBilateralConstraint>& effector, ndJointHinge* const footHinge)
				:m_footHinge(footHinge)
				,m_effector(effector)
			{
			}

			ndJointHinge* m_footHinge;
			ndSharedPtr<ndJointBilateralConstraint> m_effector;
		};

		class ndPoseGenerator: public ndAnimationSequence
		{
			public:
			ndPoseGenerator(ndFloat32 gaitFraction, const ndFloat32* const phase)
				:ndAnimationSequence()
				,m_amp(0.27f)
				,m_gaitFraction(gaitFraction)
				,m_stanceMask(0)
			{
				m_currentPose.SetCount(0);
				m_duration = ndFloat32 (4.0f);
				for (ndInt32 i = 0; i < 4; i++)
				{
					m_phase[i] = phase[i];
					m_offset[i] = ndFloat32(0.0f);
					m_currentPose.PushBack(BasePose(i));
				}
			}

			ndVector GetTranslation(ndFloat32) const
			{
				return ndVector::m_zero;
			}

			ndInt32 GetStanceCode() const
			{
				return m_stanceMask;
			}

			ndVector BasePose(ndInt32 index) const 
			{
				ndVector base(ndVector::m_wOne);
				base.m_x = 0.4f;
				base.m_z = m_offset[index];
				base.m_y = D_POSE_REST_POSITION_Y;
				return base;
			}

			void CalculatePose(ndAnimationPose& output, ndFloat32 param) const
			{
				// generate a procedural in place march gait
				ndAssert(param >= ndFloat32(0.0f));
				ndAssert(param <= ndFloat32(1.0f));

				//param = 0.125f;
				
				m_stanceMask = 0x0f;
				ndFloat32 omega = ndPi / m_gaitFraction;
				for (ndInt32 i = 0; i < output.GetCount(); i++)
				{
					output[i].m_posit = BasePose(i);
					ndFloat32 t = ndMod (param - m_phase[i] + ndFloat32(1.0f), ndFloat32 (1.0f));
					t = m_gaitFraction / 2;
					if (t <= m_gaitFraction)
					{
						if (i == 0)
						//if ((i == 0) || (i == 3))
						//if ((i == 1) || (i == 2))
						{
							m_stanceMask = m_stanceMask ^ (1 << i);
							output[i].m_posit.m_y += m_amp * ndSin(omega * t);
						}
					}
					m_currentPose[i] = output[i].m_posit;
				}
			}

			ndFloat32 m_amp;
			ndFloat32 m_gaitFraction;
			ndFloat32 m_phase[4];
			ndFloat32 m_offset[4];
			mutable ndInt32 m_stanceMask;
			mutable ndFixSizeArray<ndVector, 4> m_currentPose;
		};

		// implement controller player
		class ndController : public ndBrainAgentContinueVPG<m_observationsSize, m_actionsSize>
		{
			public:
			ndController(ndSharedPtr<ndBrain>& actor)
				:ndBrainAgentContinueVPG<m_observationsSize, m_actionsSize>(actor)
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
				ndMemCpy(&m_currentTransition[0], observation, m_observationsSize);
			}

			virtual void ApplyActions(ndBrainFloat* const actions) const
			{
				m_model->ApplyActions(actions, &m_currentTransition[0]);
			}

			ndRobot* m_model;
			ndBrainFixSizeVector<m_observationsSize> m_currentTransition;
		};

		class ndControllerAgent_trainer : public ndBrainAgentContinueVPG_Trainer<m_observationsSize, m_actionsSize>
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

			ndControllerAgent_trainer(const HyperParameters& hyperParameters)
				:ndBrainAgentContinueVPG_Trainer<m_observationsSize, m_actionsSize>(hyperParameters)
				,m_bestActor(m_actor)
				,m_model(nullptr)
				,m_maxGain(-1.0e10f)
				,m_maxFrames(3000)
				,m_startTraning(0)
				,m_stopTraining(5000000)
				,m_timer(ndGetTimeInMicroseconds())
				,m_modelIsTrained(false)
			{
				SetName(CONTROLLER_NAME);
				m_outFile = fopen("quadruped_1-VPG.csv", "wb");
				fprintf(m_outFile, "vpg\n");
			}

			~ndControllerAgent_trainer()
			{
				if (m_outFile)
				{
					fclose(m_outFile);
				}
			}

			void SetModel(ndRobot* const model)
			{
				m_model = model;
				for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
				{
					ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();
					m_bodies.PushBack(body);
					m_basePose.PushBack(body);
				}
			}

			ndBrainFloat GetReward() const
			{
				if (IsTerminal())
				{
					return ndBrainFloat(-1.0f);
				}

				//ndInt32 contactMask = 0x0f;
				//const ndPoseGenerator* const poseGenerator = (ndPoseGenerator*)*m_model->m_poseGenerator->GetSequence();
				//for (ndInt32 i = 0; i < m_model->m_animPose.GetCount(); ++i)
				//{
				//	ndContact* const contact = m_model->FindContact(i);
				//	if (contact)
				//	{
				//		contactMask = contactMask ^ (1 << i);
				//	}
				//}
				//ndInt32 mask = contactMask & poseGenerator->GetStanceCode();
				//
				//ndReal reward = ndReal(0.0f);
				//switch (mask)
				//{
				//	case 0x07:
				//	case 0x0b:
				//	case 0x0d:
				//	case 0x0f:
				//	{
				//		reward = ndReal(1.0f);
				//	}
				//}
				//const ndUIControlNode* const control = m_model->m_control;
				//ndFloat32 control_x_reward = ndExp(-ndFloat32(230.0f) * control->m_x * control->m_x);
				//ndFloat32 control_z_reward = ndExp(-ndFloat32(102.0f) * control->m_z * control->m_z);
				//return ndReal((reward * ndFloat32(8.0f) + control_x_reward + control_z_reward) / ndFloat32(10.0f));




				ndInt32 stateIndex = 0;
				ndBrainFloat positReward = ndBrainFloat(0.0f);
				ndBrainFloat velocReward = ndBrainFloat(0.0f);
				const ndBrainFloat* const state = &m_currentTransition[0];
				
				for (ndInt32 i = 0; i < m_actionsSize / 3; ++i)
				{
					ndEffectorInfo* const info = &m_model->m_effectorsInfo[i];
					ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;
				
					ndVector targetPosit(effector->GetEffectorPosit());
					//ndVector targetPosit(effector->GetLocalTargetPosition()); very wrong option
					
					ndBrainFloat positError0 = state[stateIndex + m_leg0_anim_posit_x] - ndBrainFloat(targetPosit.m_x);
					ndBrainFloat positError1 = state[stateIndex + m_leg0_anim_posit_y] - ndBrainFloat(targetPosit.m_y);
					ndBrainFloat positError2 = state[stateIndex + m_leg0_anim_posit_z] - ndBrainFloat(targetPosit.m_z);
				
					ndBrainFloat d20 = ndMin(ndBrainFloat(100000.0f) * positError0 * positError0, ndBrainFloat(30.0f));
					positReward += ndBrainFloat(ndExp(-d20));
				
					ndBrainFloat d21 = ndMin(ndBrainFloat(100000.0f) * positError1 * positError1, ndBrainFloat(30.0f));
					positReward += ndBrainFloat(ndExp(-d21));
				
					ndBrainFloat d22 = ndMin(ndBrainFloat(100000.0f) * positError2 * positError2, ndBrainFloat(30.0f));
					positReward += ndBrainFloat(ndExp(-d22));
				
					ndVector posit;
					ndVector veloc;
					effector->GetDynamicState(posit, veloc);
					ndBrainFloat velocError0 = ndBrainFloat(veloc.m_x);
					ndBrainFloat velocError1 = ndBrainFloat(veloc.m_y);
					ndBrainFloat velocError2 = ndBrainFloat(veloc.m_z);
				
					ndBrainFloat d23 = ndMin(ndBrainFloat(200.0f) * velocError0 * velocError0, ndBrainFloat(30.0f));
					velocReward += ndBrainFloat(ndExp(-d23));
					
					ndBrainFloat d24 = ndMin(ndBrainFloat(200.0f) * velocError1 * velocError1, ndBrainFloat(30.0f));
					velocReward += ndBrainFloat(ndExp(-d24));
					
					ndBrainFloat d25 = ndMin(ndBrainFloat(200.0f) * velocError2 * velocError2, ndBrainFloat(30.0f));
					velocReward += ndBrainFloat(ndExp(-d25));
				
					stateIndex += 9;
				}

				ndBodyKinematic* const body = m_model->GetRoot()->m_body->GetAsBodyKinematic();
				const ndMatrix& matrix = body->GetMatrix();
				ndFloat32 sinAngle2 = matrix.m_up.m_x * matrix.m_up.m_x + matrix.m_up.m_z * matrix.m_up.m_z;
				//sinAngle = ndMin(sinAngle, ndFloat32(0.9f));

				ndBrainFloat orientationReward = (20 * m_actionsSize) * ndExp(-100.0f * sinAngle2);
				
				ndBrainFloat den = 2 * m_actionsSize + m_actionsSize + 20 * m_actionsSize;
				ndBrainFloat num = orientationReward + ndBrainFloat (2.0f) * positReward + velocReward;
				ndBrainFloat reward = num / den;
				if (reward > 0.85f) 
				{
					ndExpandTraceMessage("%d %f\n", GetFramesCount(), reward);
				}
				return reward;
			}

			virtual void ApplyActions(ndBrainFloat* const actions) const
			{
				m_model->ApplyActions(actions, &m_currentTransition[0]);
			}

			void GetObservation(ndBrainFloat* const observation)
			{
				m_model->GetObservation(observation);
				ndMemCpy(&m_currentTransition[0], observation, m_observationsSize);
			}

			bool IsTerminal() const
			{
				//ndBrainFloat maxVal = ndBrainFloat(0.0f);
				//const ndBrainFloat* const stateActions = &m_currentTransition.m_observation[0];
				//ndInt32 actionIndex = 0;
				//for (ndInt32 i = 0; i < m_model->m_animPose.GetCount(); ++i)
				//{
				//	ndBrainFloat error0 = stateActions[actionIndex + m_leg0_anim_posit_x] - stateActions[actionIndex + m_leg0_posit_x];
				//	ndBrainFloat error1 = stateActions[actionIndex + m_leg0_anim_posit_y] - stateActions[actionIndex + m_leg0_posit_y];
				//	ndBrainFloat error2 = stateActions[actionIndex + m_leg0_anim_posit_z] - stateActions[actionIndex + m_leg0_posit_z];
				//	maxVal = ndMax(maxVal, ndAbs(error0));
				//	maxVal = ndMax(maxVal, ndAbs(error1));
				//	maxVal = ndMax(maxVal, ndAbs(error2));
				//	actionIndex += (m_leg1_anim_posit_x - m_leg0_anim_posit_x);
				//  actionIndex += 9;
				//}
				//ndFloat32 dev = ndBrainFloat(0.5f) * D_EFFECTOR_STEP;
				//bool state = (maxVal > dev) ? true : false;
				//state = state && (m_startTraning >= 64);

				return m_model->IsTerminal();
			}

			void ResetModel() const
			{
				m_model->m_control->Reset();
				for (ndInt32 i = 0; i < m_basePose.GetCount(); i++)
				{
					m_basePose[i].SetPose();
				}

				ndUnsigned32 index = (ndRandInt() >> 1) % 4;
				ndFloat32 duration = m_model->m_poseGenerator->GetSequence()->GetDuration();
				m_model->m_poseGenerator->SetTime(duration * ndFloat32 (index) / 4.0f);
				m_model->m_poseGenerator->SetTime(duration * ndFloat32(index) / 4.0f);
				m_model->ApplyPoseGeneration();

				m_startTraning = 0;
			}

			void OptimizeStep()
			{
				ndInt32 stopTraining = GetFramesCount();
				if (stopTraining <= m_stopTraining)
				{
					ndInt32 episodeCount = GetEposideCount();
					ndBrainAgentContinueVPG_Trainer::OptimizeStep();

					episodeCount -= GetEposideCount();
					if (m_averageFramesPerEpisodes.GetAverage() >= ndFloat32(m_maxFrames))
					{
						if (m_averageQvalue.GetAverage() > m_maxGain)
						{
							m_bestActor.CopyFrom(m_actor);
							m_maxGain = m_averageQvalue.GetAverage();
							ndExpandTraceMessage("best actor episode: %d\taverageFrames: %f\taverageValue %f\n", GetEposideCount(), m_averageFramesPerEpisodes.GetAverage(), m_averageQvalue.GetAverage());
						}
					}

					if (episodeCount && !IsSampling())
					{
						ndExpandTraceMessage("step: %d\treward: %g\tframes: %g\n", GetFramesCount(), m_averageQvalue.GetAverage(), m_averageFramesPerEpisodes.GetAverage());
						if (m_outFile)
						{
							fprintf(m_outFile, "%g\n", m_averageQvalue.GetAverage());
							fflush(m_outFile);
						}
					}

					if (stopTraining == m_stopTraining)
					{
						char fileName[1024];
						m_modelIsTrained = true;
						m_actor.CopyFrom(m_bestActor);
						ndGetWorkingFileName(GetName().GetStr(), fileName);
						SaveToFile(fileName);
						ndExpandTraceMessage("saving to file: %s\n", fileName);
						ndExpandTraceMessage("training complete\n");
						ndUnsigned64 timer = ndGetTimeInMicroseconds() - m_timer;
						ndExpandTraceMessage("training time: %g\n seconds", ndFloat32(ndFloat64(timer) * ndFloat32(1.0e-6f)));
					}
				}

				//if (m_model->IsOutOfBounds())
				//{
				//	m_model->TelePort();
				//}
				m_startTraning ++;
			}

			FILE* m_outFile;
			ndBrain m_bestActor;
			ndRobot* m_model;
			ndFloat32 m_maxGain;
			ndInt32 m_maxFrames;
			mutable ndInt32 m_startTraning;
			ndInt32 m_stopTraining;
			ndUnsigned64 m_timer;
			bool m_modelIsTrained;
			ndFixSizeArray<ndBasePose, 32> m_basePose;
			ndFixSizeArray<ndBodyDynamic*, 32> m_bodies;
			ndBrainFixSizeVector<m_observationsSize> m_currentTransition;
		};

		class ndUIControlNode: public ndAnimationBlendTreeNode
		{
			public:
			ndUIControlNode(ndAnimationBlendTreeNode* const input)
				:ndAnimationBlendTreeNode(input)
				,m_x(ndReal(0.0f))
				,m_y(ndReal(0.0f))
				,m_z(ndReal(0.0f))
				,m_pitch(ndReal(0.0f))
				,m_yaw(ndReal(0.0f))
				,m_roll(ndReal(0.0f))
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
					const ndMatrix& localMatrix = effector->GetLocalMatrix1();
					ndVector p0(localMatrix.TransformVector(keyFrame.m_posit));
					ndVector p1(matrix.TransformVector(p0));
					ndVector p2(localMatrix.UntransformVector(p1));
					keyFrame.m_posit = p2;
				}
			}

			ndReal m_x;
			ndReal m_y;
			ndReal m_z;
			ndReal m_pitch;
			ndReal m_yaw;
			ndReal m_roll;
			ndReal m_animSpeed;
			bool m_enableController;
		};
		
		ndRobot(ndSharedPtr<ndBrainAgent>& agent)
			:ndModelArticulation()
			,m_agent(agent)
		{
		}

		ndContact* FindContact (ndInt32 index) const 
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

		void Debug(ndConstraintDebugCallback& context) const
		{
			ndBodyKinematic* const rootBody = GetRoot()->m_body->GetAsBodyKinematic();
			ndSkeletonContainer* const skeleton = rootBody->GetSkeleton();
			if (!skeleton)
			{
				return;
			}

			//ndBodyKinematic* const rootBody = GetRoot()->m_body->GetAsBodyKinematic();
			//context.DrawFrame(rootBody->GetMatrix());
			//for (ndInt32 i = 0; i < 4; ++i)
			//{
			//	const ndEffectorInfo& info = m_effectorsInfo[i];
			//	info.m_footHinge->DebugJoint(context);
			//}
			//ndVector upVector(rootBody->GetMatrix().m_up);
			//for (ndInt32 i = 0; i < 4; ++i)
			//{
			//	const ndEffectorInfo& info = m_effectorsInfo[i];
			//	const ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info.m_effector;
			//	effector->DebugJoint(context);
			//
			//	//ndMatrix lookAtMatrix0;
			//	//ndMatrix lookAtMatrix1;
			//	//ndJointBilateralConstraint* const footJoint = info.m_footHinge;
			//	//footJoint->DebugJoint(context);
			//	//footJoint->CalculateGlobalMatrix(lookAtMatrix0, lookAtMatrix1);
			//	//ndVector updir(lookAtMatrix1.m_posit + upVector.Scale(-1.0f));
			//	//context.DrawLine(lookAtMatrix1.m_posit, updir, ndVector(0.0f, 0.0f, 0.0f, 1.0f));
			//}

			//auto FindContact = [this](ndInt32 index)
			//{
			//	const ndAnimKeyframe& keyFrame = m_animPose[index];
			//	ndEffectorInfo* const info = (ndEffectorInfo*)keyFrame.m_userData;
			//	ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;
			//
			//	ndBodyKinematic* const body = effector->GetBody0();
			//	ndBodyKinematic::ndContactMap& contacts = body->GetContactMap();
			//	ndBodyKinematic::ndContactMap::Iterator it(contacts);
			//	for (it.Begin(); it; it++)
			//	{
			//		ndContact* const contact = *it;
			//		if (contact->IsActive())
			//		{
			//			return contact;
			//		}
			//	}
			//	return (ndContact*)nullptr;
			//};

			
			ndFixSizeArray<ndJointBilateralConstraint*, 32> effectors;
			for (ndInt32 i = 0; i < m_actionsSize / 3; ++i)
			{
				ndEffectorInfo* const info = (ndEffectorInfo*)m_animPose[i].m_userData;
				ndAssert(info == &m_effectorsInfo[i]);
				effectors.PushBack (*info->m_effector);
			
				ndContact* const contact = FindContact(i);
				if (contact)
				{
					m_world->CalculateJointContacts(contact);
				}
			}

			ndIkSolver* const invDynamicsSolver = (ndIkSolver*)&m_invDynamicsSolver;
			invDynamicsSolver->SolverBegin(skeleton, &effectors[0], effectors.GetCount(), m_world, m_timestep);
			invDynamicsSolver->Solve();

			//a) Mt = sum(m(i))
			//b) cg = sum(p(i) * m(i)) / Mt
			//c) Vcg = sum(v(i) * m(i)) / Mt
			//d) Icg = sum(I(i) +  m(i) * (Identity * ((p(i) - cg) * transpose (p(i) - cg)) - covarianMatrix(p(i) - cg))
			//e) T0 = sum[w(i) x (I(i) * w(i)) - Vcg x (m(i) * v(i))]
			//f) T1 = sum[(p(i) - cg) x Fext(i) + Text(i)]
			//g) Bcg = (Icg ^ -1) * (T0 + T1)

			ndBodyState state;
			ndFixSizeArray<const ndBodyKinematic*, 32> bodies;

			for (ndModelArticulation::ndNode* node = GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				const ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
				const ndMatrix matrix(body->GetMatrix());
				ndFloat32 mass = body->GetMassMatrix().m_w;
				state.m_mass += mass;
				state.m_com.m_posit += matrix.TransformVector(body->GetCentreOfMass()).Scale(mass);
				bodies.PushBack(body);
			}
			ndFloat32 invMass = 1.0f / state.m_mass;
			state.m_com.m_posit = state.m_com.m_posit.Scale(invMass);
			state.m_com.m_posit.m_w = ndFloat32 (1.0f);

			ndMatrix comMatrix(m_rootNode->m_body->GetAsBodyKinematic()->GetMatrix());
			comMatrix.m_posit = state.m_com.m_posit;
			context.DrawFrame(comMatrix);
			ndVector comLineOfAction(comMatrix.m_posit);
			comLineOfAction.m_y -= 0.5f;
			context.DrawLine(comMatrix.m_posit, comLineOfAction, ndVector::m_zero);
			comLineOfAction.m_y = comMatrix.m_posit.m_y - 0.38f;
			context.DrawPoint(comLineOfAction, ndVector(0.0f, 0.0f, 1.0f, 1.0f), 10);

			ndVector netTorque(ndVector::m_zero);
			ndVector netTorque1(ndVector::m_zero);
			for (ndInt32 i = 0; i < bodies.GetCount(); ++i)
			{
				const ndBodyKinematic* const body = bodies[i];
				const ndMatrix matrix(body->GetMatrix());
				const ndVector comDist((matrix.TransformVector(body->GetCentreOfMass()) - state.m_com.m_posit) & ndVector::m_triplexMask);

				const ndVector omega(body->GetOmega());
				const ndMatrix bodyInertia(body->CalculateInertiaMatrix());

				//ndVector force(invDynamicsSolver->GetBodyForce(body));
				//ndVector torque (invDynamicsSolver->GetBodyTorque(body));
				ndVector torque(bodyInertia.RotateVector(body->GetAlpha()));
				ndVector force(body->GetAccel().Scale(body->GetMassMatrix().m_w));
				torque += comDist.CrossProduct(force);
				torque += omega.CrossProduct(bodyInertia.RotateVector(omega));

				ndJacobian force1(body->CalculateNetForce());
				force1.m_angular += comDist.CrossProduct(force1.m_linear);
				force1.m_angular += omega.CrossProduct(bodyInertia.RotateVector(omega));

				netTorque += torque;
				netTorque1 += force1.m_angular;
			}
			invDynamicsSolver->SolverEnd();
			ndVector weight(ndVector::m_zMask);
			weight.m_y = -state.m_mass * DEMO_GRAVITY;
			ndVector zmpLineOfAction(comLineOfAction);
			//zmpLineOfAction.m_z += netTorque.m_x / weight.m_y;
			//zmpLineOfAction.m_x += -netTorque.m_z / weight.m_y;
			zmpLineOfAction.m_z += netTorque1.m_x / weight.m_y;
			zmpLineOfAction.m_x += -netTorque1.m_z / weight.m_y;
			context.DrawPoint(zmpLineOfAction, ndVector(1.0f, 0.0f, 0.0f, 1.0f), 6);

			const ndPoseGenerator* const poseGenerator = (ndPoseGenerator*)*m_poseGenerator->GetSequence();
			ndInt32 stanceMask = poseGenerator->GetStanceCode();

			ndFixSizeArray<ndVector, 4> contactPoints;
			for (ndInt32 i = 0; i < m_actionsSize / 3; ++i)
			{
				const ndAnimKeyframe& keyFrame = m_animPose[i];
				//if (keyFrame.m_rotation.m_w > ndFloat32(0.0f))
				if (stanceMask & (1<<i))
				{
					ndEffectorInfo* const info = (ndEffectorInfo*)keyFrame.m_userData;
					ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;
					ndBodyKinematic* const body = effector->GetBody0();
					contactPoints.PushBack(body->GetMatrix().TransformVector(effector->GetLocalMatrix0().m_posit));
				}
			}

			ndVector supportColor(0.0f, 1.0f, 1.0f, 1.0f);
			if (contactPoints.GetCount() >= 3)
			{
				ndMatrix rotation(ndPitchMatrix(90.0f * ndDegreeToRad));
				rotation.TransformTriplex(&contactPoints[0].m_x, sizeof(ndVector), &contactPoints[0].m_x, sizeof(ndVector), contactPoints.GetCount());
				ndInt32 supportCount = ndConvexHull2d(&contactPoints[0], contactPoints.GetCount());
				rotation.OrthoInverse().TransformTriplex(&contactPoints[0].m_x, sizeof(ndVector), &contactPoints[0].m_x, sizeof(ndVector), contactPoints.GetCount());
				ndVector p0(contactPoints[supportCount - 1]);
				ndBigVector bigPolygon[16];
				for (ndInt32 i = 0; i < supportCount; ++i)
				{
					bigPolygon[i] = contactPoints[i];
					context.DrawLine(contactPoints[i], p0, supportColor);
					p0 = contactPoints[i];
				}
			
				//ndBigVector p0Out;
				//ndBigVector p1Out;
				//ndBigVector ray_p0(comMatrix.m_posit);
				//ndBigVector ray_p1(comMatrix.m_posit);
				//ray_p1.m_y -= 1.0f;
				//ndRayToPolygonDistance(ray_p0, ray_p1, bigPolygon, supportCount, p0Out, p1Out);
				//context.DrawPoint(p0Out, ndVector(1.0f, 0.0f, 0.0f, 1.0f), 3);
				//context.DrawPoint(p1Out, ndVector(0.0f, 1.0f, 0.0f, 1.0f), 3);
			}
			else if (contactPoints.GetCount() == 2)
			{
				context.DrawLine(contactPoints[0], contactPoints[1], supportColor);
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

		ndInt32 HasContact(ndInt32 effectorIndex) const
		{
			const ndAnimKeyframe& keyFrame = m_animPose[effectorIndex];
			ndEffectorInfo* const info = (ndEffectorInfo*)keyFrame.m_userData;
			ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;

			ndBodyKinematic* const body = effector->GetBody0();
			const ndBodyKinematic::ndContactMap& contacts = body->GetContactMap();
			ndBodyKinematic::ndContactMap::Iterator it(contacts);
			for (it.Begin(); it; it++)
			{
				const ndContact* const contact = *it;
				if (contact->IsActive())
				{
					return 1;
				}
			}
			return 0;
		}

		void ApplyPoseGeneration()
		{
			ndVector veloc;
			m_animBlendTree->Evaluate(m_animPose, veloc);

			ndInt32 actionIndex = 0;
			ndBrainFloat actions[m_actionsSize * 2];
			for (ndInt32 i = 0; i < m_actionsSize / 3; ++i)
			{
				ndVector posit(m_animPose[i].m_posit);

				actions[actionIndex + m_leg0_action_posit_x] = ndBrainFloat(posit.m_x);
				actions[actionIndex + m_leg0_action_posit_y] = ndBrainFloat(posit.m_y);
				actions[actionIndex + m_leg0_action_posit_z] = ndBrainFloat(posit.m_z);
				//actions[actionIndex + m_leg0_action_posit_swivel] = posit.m_w;

				//actionIndex += (m_leg1_action_posit_x - m_leg0_action_posit_x);
				actionIndex += 3;
			}

			ApplyPoseGeneration(actions);
		}

		void ApplyPoseGeneration(ndBrainFloat* const actions)
		{
			ndBodyKinematic* const rootBody = GetRoot()->m_body->GetAsBodyKinematic();
			ndSkeletonContainer* const skeleton = rootBody->GetSkeleton();
			ndAssert(skeleton);
			ndJointBilateralConstraint* effectors[32];
			ndVector upVector(rootBody->GetMatrix().m_up);

			ndInt32 actionIndex = 0;
			ndInt32 effectorsCount = 0;
			for (ndInt32 i = 0; i < m_actionsSize / 3; ++i)
			{
				ndEffectorInfo* const info = &m_effectorsInfo[i];
				effectors[i] = *info->m_effector;

				ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;

				//ndVector posit(m_animPose[i].m_posit);
				ndVector posit(actions[actionIndex + m_leg0_action_posit_x], 
							   actions[actionIndex + m_leg0_action_posit_y], 
							   actions[actionIndex + m_leg0_action_posit_z], ndBrainFloat(1.0f));
				effector->SetLocalTargetPosition(posit);
				//effector->SetSwivelAngle(actions[actionIndex + m_leg0_action_posit_swivel]);
				effector->SetSwivelAngle(0.0f);
				
				// calculate lookAt angle
				ndMatrix lookAtMatrix0;
				ndMatrix lookAtMatrix1;
				info->m_footHinge->CalculateGlobalMatrix(lookAtMatrix0, lookAtMatrix1);

				ndMatrix upMatrix(ndGetIdentityMatrix());
				upMatrix.m_front = lookAtMatrix1.m_front;
				upMatrix.m_right = (upMatrix.m_front.CrossProduct(upVector) & ndVector::m_triplexMask).Normalize();
				upMatrix.m_up = upMatrix.m_right.CrossProduct(upMatrix.m_front);
				upMatrix = upMatrix * lookAtMatrix0.OrthoInverse();
				const ndFloat32 angle = ndAtan2(upMatrix.m_up.m_z, upMatrix.m_up.m_y);
				info->m_footHinge->SetTargetAngle(angle);

				effectorsCount++;
				actionIndex += 3;
			}

			m_invDynamicsSolver.SolverBegin(skeleton, effectors, effectorsCount, m_world, m_timestep);
			m_invDynamicsSolver.Solve();
			m_invDynamicsSolver.SolverEnd();
		}

		bool IsTerminal() const
		{
			ndBodyKinematic* const body = GetRoot()->m_body->GetAsBodyKinematic();
			const ndMatrix& matrix = body->GetMatrix();
			ndFloat32 sinAngle = ndSqrt(matrix.m_up.m_x * matrix.m_up.m_x + matrix.m_up.m_z * matrix.m_up.m_z);
			sinAngle = ndMin(sinAngle, ndFloat32(0.9f));
			bool fail = ndAbs(ndAsin(sinAngle)) > D_MIN_REWARD_ANGLE;
			return fail;
		}

		ndReal GetReward()
		{
			if (IsTerminal())
			{
				return ndReal(-1.0f);
			}

#if 0
			ndBodyKinematic* const rootBody = GetRoot()->m_body->GetAsBodyKinematic();
			ndSkeletonContainer* const skeleton = rootBody->GetSkeleton();
			ndAssert(skeleton);
			ndJointBilateralConstraint* joint[4];
			ndVector upVector(rootBody->GetMatrix().m_up);
			for (ndInt32 i = 0; i < 4; ++i)
			{
				ndEffectorInfo* const info = (ndEffectorInfo*)m_animPose[i].m_userData;
				ndAssert(info == &m_effectorsInfo[i]);
				joint[i] = *info->m_effector;
			}

			m_invDynamicsSolver.SolverBegin(skeleton, joint, 4, m_world, m_timestep);
			m_invDynamicsSolver.Solve();

			//a) Mt = sum(m(i))
			//b) cg = sum(p(i) * m(i)) / Mt
			//c) Vcg = sum(v(i) * m(i)) / Mt
			//d) Icg = sum(I(i) +  m(i) * (Identity * ((p(i) - cg) * transpose (p(i) - cg)) - covarianMatrix(p(i) - cg))
			//e) T0 = sum[w(i) x (I(i) * w(i)) - Vcg x (m(i) * v(i))]
			//f) T1 = sum[(p(i) - cg) x Fext(i) + Text(i)]
			//g) Bcg = (Icg ^ -1) * (T0 + T1)

			ndBodyState state;
			ndFixSizeArray<ndVector, 32> bodyMomentum;
			ndFixSizeArray<const ndBodyKinematic*, 32> bodies;
			for (ndModelArticulation::ndNode* node = GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				const ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
				const ndMatrix matrix(body->GetMatrix());
				ndFloat32 mass = body->GetMassMatrix().m_w;
				state.m_mass += mass;
				ndVector momentum(body->GetVelocity().Scale(mass));
				state.m_linearMomentum += momentum;
				state.m_com.m_posit += matrix.TransformVector(body->GetCentreOfMass()).Scale(mass);

				bodies.PushBack(body);
				bodyMomentum.PushBack(momentum);
			}
			//ndFloat32 invMass = 1.0f / state.m_mass;
			ndVector torque (ndVector::m_zero);
			for (ndInt32 i = 0; i < bodies.GetCount(); ++i)
			{
				const ndBodyKinematic* const body = bodies[i];
				const ndMatrix matrix(body->GetMatrix());
				const ndVector comDist((matrix.TransformVector(body->GetCentreOfMass()) - state.m_com.m_posit) & ndVector::m_wOne);
			
				const ndMatrix bodyInertia(body->CalculateInertiaMatrix());
				const ndVector omega(body->GetOmega());

				torque += m_invDynamicsSolver.GetBodyTorque(body);
				torque += comDist.CrossProduct(m_invDynamicsSolver.GetBodyForce(body));
				torque += omega.CrossProduct(bodyInertia.RotateVector(omega));

				// note: the sum below should add to zero.
				//ndVector angularMomentumSum(ndVector::m_zero);
				//for (ndInt32 j = 0; j < bodies.GetCount(); ++j)
				//{
				//	if (j != i)
				//	{
				//		angularMomentumSum += bodyMomentum[j].CrossProduct(bodyMomentum[i]);
				//	}
				//}
				//torque += angularMomentumSum.Scale(invMass);

				ndFloat32 mass = body->GetMassMatrix().m_w;
				ndFloat32 massDist2 = comDist.DotProduct(comDist).GetScalar() * mass;
				const ndMatrix covariance(ndCovarianceMatrix(comDist, comDist));
				state.m_inertia[0][0] += massDist2;
				state.m_inertia[1][1] += massDist2;
				state.m_inertia[2][2] += massDist2;
				state.m_inertia[0] += (bodyInertia[0] - covariance[0].Scale(mass));
				state.m_inertia[1] += (bodyInertia[1] - covariance[1].Scale(mass));
				state.m_inertia[2] += (bodyInertia[2] - covariance[2].Scale(mass));
				ndAssert(state.m_inertia[0][0] > ndFloat32(0.0f));
				ndAssert(state.m_inertia[1][1] > ndFloat32(0.0f));
				ndAssert(state.m_inertia[2][2] > ndFloat32(0.0f));
			}
			m_invDynamicsSolver.SolverEnd();

			ndMatrix invInertia(state.m_inertia.Inverse4x4());
			ndVector alpha(invInertia.RotateVector(torque));

			const ndMatrix& rootMatrix = GetRoot()->m_body->GetMatrix();
			ndMatrix matrix(ndGetIdentityMatrix());
			matrix.m_up = ndVector::m_zero;
			matrix.m_up.m_y = ndFloat32(1.0f);
			matrix.m_right = (rootMatrix.m_front.CrossProduct(matrix.m_up)).Normalize();
			matrix.m_front = matrix.m_up.CrossProduct(matrix.m_right);
			torque = matrix.UnrotateVector(torque);

			//ndTrace(("%d torque(%f %f %f) alpha(%f %f %f)\n", xxx, torque.m_x, torque.m_y, torque.m_z, alpha.m_x, alpha.m_y, alpha.m_z));

			//ndFloat32 xxx0 = torque.m_x * torque.m_x + torque.m_z * torque.m_z;
			ndFloat32 accel = alpha.m_x * alpha.m_x + alpha.m_z * alpha.m_z;
			//ndTrace(("%d reward(torque)=%f reward(alpha)=%f\n", xxx, xxx0, xxx1));

			ndFloat32 reward = ndPow(ndEXP, -ndFloat32(0.05f) * accel);
			ndTrace(("alpha^2=%f reward=%f\n", accel, reward));

			//ndFloat32 sinAngle = ndSqrt(matrix.m_up.m_x * matrix.m_up.m_x + matrix.m_up.m_z * matrix.m_up.m_z);
			//ndFloat32 reward = ndReal(ndPow(ndEXP, -ndFloat32(10000.0f) * sinAngle * sinAngle));
			return ndReal(reward);
#endif
			
			ndInt32 contactMask = 0;
			ndInt32 keyFramerMask = ((ndPoseGenerator*) *m_poseGenerator->GetSequence())->GetStanceCode();
			for (ndInt32 i = 0; i < m_actionsSize / 3; ++i)
			{
				contactMask = contactMask | (HasContact(i) << i);
			}

			ndFloat32 animReward = ndReal(0.0f);
			if (contactMask == 0x0f)
			{
				animReward = ndFloat32(0.1f);
			}
			else if (keyFramerMask == contactMask)
			{
				animReward = ndFloat32(1.0f);
			}

			ndFloat32 control_x_reward = ndExp(-ndFloat32(230.0f) * m_control->m_x * m_control->m_x);
			ndFloat32 control_z_reward = ndExp(-ndFloat32(102.0f) * m_control->m_z * m_control->m_z);
			return ndReal ((animReward * ndFloat32(4.0f) + control_x_reward + control_z_reward) / ndFloat32 (6.0f));
		}

		ndBodyState CalculateFullBodyState() const
		{
			ndBodyState state;
			const ndMatrix& rootMatrix = GetRoot()->m_body->GetMatrix();
			state.m_com.m_up = ndVector::m_zero;
			state.m_com.m_up.m_y = ndFloat32 (1.0f);
			state.m_com.m_right = (rootMatrix.m_front.CrossProduct(state.m_com.m_up)).Normalize();
			state.m_com.m_front = state.m_com.m_up.CrossProduct(state.m_com.m_right);
			state.m_com.m_posit = ndVector::m_zero;
			for (ndModelArticulation::ndNode* node = GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				const ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
				const ndMatrix matrix(body->GetMatrix());
				ndFloat32 mass = body->GetMassMatrix().m_w;
				state.m_mass += mass;
				state.m_com.m_posit += matrix.TransformVector(body->GetCentreOfMass()).Scale(mass);
			}
			state.m_com.m_posit = state.m_com.m_posit.Scale(1.0f / state.m_mass);
			state.m_com.m_posit.m_w = 1.0f;

			for (ndModelArticulation::ndNode* node = GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				const ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();

				const ndMatrix matrix(body->GetMatrix());
			 	ndVector comDist (matrix.TransformVector(body->GetCentreOfMass()) - state.m_com.m_posit);
				comDist = comDist & ndVector::m_triplexMask;

				ndFloat32 mass = body->GetMassMatrix().m_w;
				//ndMatrix bodyInertia(body->CalculateInertiaMatrix());
				//ndMatrix covariance(ndCovarianceMatrix(comDist, comDist));
				
				//ndVector linearMomentum(body->GetVelocity().Scale(mass));
				//state.m_veloc += linearMomentum;
				//state.m_omega += comDist.CrossProduct(linearMomentum);
				//state.m_omega += bodyInertia.RotateVector(body->GetOmega());
				//
				//ndFloat32 massDist2 = comDist.DotProduct(comDist).GetScalar() * mass;
				//
				//state.m_inertia[0] += (bodyInertia[0] - covariance[0].Scale(mass));
				//state.m_inertia[1] += (bodyInertia[1] - covariance[1].Scale(mass));
				//state.m_inertia[2] += (bodyInertia[2] - covariance[2].Scale(mass));
				//
				//state.m_inertia[0][0] += massDist2;
				//state.m_inertia[1][1] += massDist2;
				//state.m_inertia[2][2] += massDist2;
				//ndAssert(state.m_inertia[0][0] > ndFloat32(0.0f));
				//ndAssert(state.m_inertia[1][1] > ndFloat32(0.0f));
				//ndAssert(state.m_inertia[2][2] > ndFloat32(0.0f));

				ndVector linearMomentum(body->GetVelocity().Scale(mass));
				state.m_linearMomentum += linearMomentum;

				ndMatrix bodyInertia(body->CalculateInertiaMatrix());
				state.m_angularMomentum += bodyInertia.RotateVector(body->GetOmega());
				state.m_angularMomentum += comDist.CrossProduct(linearMomentum);
			}

			//state.m_inertia.m_posit = ndVector::m_wOne;
			//ndMatrix invInertia(state.m_inertia.Inverse4x4());
			//state.m_omega = invInertia.RotateVector(state.m_omega);
			//state.m_veloc = state.m_veloc.Scale(1.0f / state.m_mass);

			return state;
		}

		void CheckTrainingCompleted()
		{
			#ifdef ND_TRAIN_MODEL
			if (m_agent->IsTrainer())
			{
				ndControllerAgent_trainer* const agent = (ndControllerAgent_trainer*)*m_agent;
				if (agent->m_modelIsTrained)
				{
					char fileName[1024];
					ndGetWorkingFileName(agent->GetName().GetStr(), fileName);
					ndSharedPtr<ndBrain> actor(ndBrainLoad::Load(fileName));
					m_agent = ndSharedPtr<ndBrainAgent>(new ndRobot::ndController(actor));
					((ndRobot::ndController*)*m_agent)->SetModel(this);
					//ResetModel();
					((ndPhysicsWorld*)m_world)->NormalUpdates();
				}
			}
			#endif
		}


		// ************************************************
		// new quad environment
		// ************************************************
		void ApplyActions(ndBrainFloat* const actions, const ndBrainFloat* const currentState)
		{
			//m_control->m_animSpeed = 4.0f;
			//m_control->m_animSpeed = 2.0f;
			//m_control->m_animSpeed = 1.0f;
			//m_control->m_animSpeed = 0.5f;
			m_control->m_animSpeed = 0.25f;
			//m_control->m_animSpeed = 0.1f;
			//if (m_control->m_enableController)
			//{
			//	//m_control->m_x = ndClamp(ndReal(m_control->m_x + actions[m_bodySwing_x] * D_SWING_STEP), -D_MAX_SWING_DIST_X, D_MAX_SWING_DIST_X);
			//	//m_control->m_z = ndClamp(ndReal(m_control->m_z + actions[m_bodySwing_z] * D_SWING_STEP), -D_MAX_SWING_DIST_Z, D_MAX_SWING_DIST_Z);
			//}

			ndInt32 stateIndex = 0;
			ndInt32 actionIndex = 0;
			ndBrainFloat combinedActions[m_actionsSize * 2];
			for (ndInt32 i = 0; i < m_actionsSize / 3; ++i)
			{
				combinedActions[actionIndex + m_leg0_action_posit_x] = currentState[stateIndex + m_leg0_anim_posit_x] + actions[actionIndex + m_leg0_action_posit_x] * D_EFFECTOR_STEP;
				combinedActions[actionIndex + m_leg0_action_posit_y] = currentState[stateIndex + m_leg0_anim_posit_y] + actions[actionIndex + m_leg0_action_posit_y] * D_EFFECTOR_STEP;
				combinedActions[actionIndex + m_leg0_action_posit_z] = currentState[stateIndex + m_leg0_anim_posit_z] + actions[actionIndex + m_leg0_action_posit_z] * D_EFFECTOR_STEP;
				//actions[actionIndex + m_leg0_action_posit_swivel] = m_currentTransition.m_observation[stateIndex + m_leg0_anim_posit_swivel] + actions[actionIndex + m_leg0_action_posit_swivel] * D_SWING_STEP;

				actionIndex += 3;
				stateIndex += 9;
			}

			ApplyPoseGeneration(combinedActions);
		}

		void GetObservation(ndBrainFloat* const observation)
		{
			//ndInt32 contactMask = 0x0f;
			//const ndPoseGenerator* const poseGenerator = (ndPoseGenerator*)*m_model->m_poseGenerator->GetSequence();
			//ndInt32 stanceMask = poseGenerator->GetStanceCode();
			//for (ndInt32 i = 0; i < m_model->m_animPose.GetCount(); ++i)
			//{
			//	ndContact* const contact = m_model->FindContact(i);
			//	if (contact)
			//	{
			//		contactMask = contactMask ^ (1 << i);
			//	}
			//	ndInt32 input = contactMask & stanceMask & (1 << i);
			//	state[m_leg0_y0 + i] = input ? ndReal (1.0f) : ndReal(0.0f);
			//}

			ndVector veloc;
			m_animBlendTree->Evaluate(m_animPose, veloc);

			ndInt32 startIndex = 0;
			for (ndInt32 i = 0; i < m_actionsSize / 3; ++i)
			{
				ndEffectorInfo* const info = &m_effectorsInfo[i];
				ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;

				ndVector effectPositState;
				ndVector effectVelocState;
				effector->GetDynamicState(effectPositState, effectVelocState);

				ndVector posit(m_animPose[i].m_posit);

				observation[startIndex + m_leg0_anim_posit_x] = ndBrainFloat(posit.m_x);
				observation[startIndex + m_leg0_anim_posit_y] = ndBrainFloat(posit.m_y);
				observation[startIndex + m_leg0_anim_posit_z] = ndBrainFloat(posit.m_z);
				//observation[startIndex + m_leg0_anim_posit_swivel] = ndBrainFloat(posit.m_w);

				observation[startIndex + m_leg0_posit_x] = ndBrainFloat(effectPositState.m_x);
				observation[startIndex + m_leg0_posit_y] = ndBrainFloat(effectPositState.m_y);
				observation[startIndex + m_leg0_posit_z] = ndBrainFloat(effectPositState.m_z);
				//observation[startIndex + m_leg0_posit_swivel] = ndBrainFloat(effectPositState.m_w);

				observation[startIndex + m_leg0_veloc_x] = ndBrainFloat(effectVelocState.m_x);
				observation[startIndex + m_leg0_veloc_y] = ndBrainFloat(effectVelocState.m_y);
				observation[startIndex + m_leg0_veloc_z] = ndBrainFloat(effectVelocState.m_z);
				//observation[startIndex + m_leg0_veloc_swivel] = ndBrainFloat(effectVelocState.m_w);

				startIndex += 9;
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
			m_animBlendTree->Update(timestep * m_control->m_animSpeed);
			ndModelArticulation::PostUpdate(world, timestep);
			m_agent->OptimizeStep();
			CheckTrainingCompleted();
		}

		ndAnimationPose m_animPose;
		ndFixSizeArray<ndEffectorInfo, 4> m_effectorsInfo;
		ndSharedPtr<ndAnimationBlendTreeNode> m_animBlendTree;
		
		ndUIControlNode* m_control;
		ndAnimationSequencePlayer* m_poseGenerator;

		ndFloat32 m_timestep;
		ndSharedPtr<ndBrainAgent> m_agent;
	};

	class ndModelUI: public ndUIEntity
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
			change = change || ImGui::SliderFloat("posit x", &control->m_x, -D_MAX_SWING_DIST_X, D_MAX_SWING_DIST_X);
			change = change || ImGui::SliderFloat("posit y", &control->m_y, -0.2f, 0.1f);
			change = change || ImGui::SliderFloat("posit z", &control->m_z, -D_MAX_SWING_DIST_Z, D_MAX_SWING_DIST_Z);
			change = change || ImGui::SliderFloat("pitch", &control->m_pitch, -15.0f, 15.0f);
			change = change || ImGui::SliderFloat("yaw", &control->m_yaw, -20.0f, 20.0f);
			change = change || ImGui::SliderFloat("roll", &control->m_roll, -15.0f, 15.0f);
			change = change || ImGui::SliderFloat("animSpeed", &control->m_animSpeed, 0.0f, 4.0f);
			change = change || ImGui::Checkbox("enable controller", &control->m_enableController);

			if (change)
			{
				ndBodyKinematic* const body = m_model->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
				body->SetSleepState(false);
			}
		}

		ndSharedPtr<ndModel> m_model;
	};

	ndSharedPtr<ndBrainAgent> BuildAgent()
	{
		#ifdef ND_TRAIN_MODEL
			// add a reinforcement learning controller 
			ndBrainAgentContinueVPG_Trainer<m_observationsSize, m_actionsSize>::HyperParameters hyperParameters;
			//hyperParameters.m_sigma = ndReal(0.25f);
			hyperParameters.m_discountFactor = ndReal(0.99f);
			hyperParameters.m_extraTrajectorySteps = 6000;
			ndSharedPtr<ndBrainAgent> agent(new ndRobot::ndControllerAgent_trainer(hyperParameters));
		#else
			char fileName[1024];
			ndGetWorkingFileName(CONTROLLER_NAME, fileName);
			ndSharedPtr<ndBrain> actor(ndBrainLoad::Load(fileName));
			ndSharedPtr<ndBrainAgent> agent(new ndRobot::ndController(actor));
		#endif
		return agent;
	}

	ndModelArticulation* BuildModel(ndDemoEntityManager* const scene, const ndMatrix& matrixLocation)
	{
		ndFloat32 mass = 20.0f;
		ndFloat32 radius = 0.25f;
		ndFloat32 limbMass = 0.25f;
		ndFloat32 limbLength = 0.3f;
		ndFloat32 limbRadios = 0.05f;

		ndSharedPtr<ndBrainAgent> agent (BuildAgent());
		ndRobot* const model = new ndRobot(agent);

		ndPhysicsWorld* const world = scene->GetWorld();
		ndVector floor(FindFloor(*world, matrixLocation.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		ndSharedPtr<ndBody> torso (world->GetBody(AddSphere(scene, matrixLocation, mass, radius, "smilli.tga")));
		ndModelArticulation::ndNode* const modelRoot = model->AddRootBody(torso);
		
		ndMatrix location(matrixLocation);
		location.m_posit.m_y = floor.m_y + 0.5f;
//location.m_posit.m_y += 0.5f;
		torso->SetMatrix(location);
		
		ndDemoEntity* const entity = (ndDemoEntity*)torso->GetNotifyCallback()->GetUserData();
		entity->SetMeshMatrix(ndYawMatrix(90.0f * ndDegreeToRad) * ndPitchMatrix(90.0f * ndDegreeToRad));
		
		ndMatrix matrix(ndRollMatrix(45.0f * ndDegreeToRad));
		matrix.m_posit.m_x = radius * 0.9f;
		matrix.m_posit.m_y = -radius * 0.5f;
		
		ndFloat32 angles[] = {300.0f, 240.0f, 120.0f, 60.0f};
		ndFloat32 offset[] = {-0.3f, 0.3f, -0.3f, 0.3f};
		//ndFloat32 phase[] = { 0.0f, 0.5f, 0.0f, 0.5f};
		//ndSharedPtr<ndAnimationSequence> sequence(new ndRobot::ndPoseGenerator(0.47f, phase));

		ndFloat32 phase[] = {0.0f, 0.5f, 0.25f, 0.75f};
		ndSharedPtr<ndAnimationSequence> sequence(new ndRobot::ndPoseGenerator(0.24f, phase));

		model->m_poseGenerator = new ndAnimationSequencePlayer(sequence);
		model->m_control = new ndRobot::ndUIControlNode(model->m_poseGenerator);
		model->m_animBlendTree = ndSharedPtr<ndAnimationBlendTreeNode>(model->m_control);

		ndRobot::ndPoseGenerator* const poseGenerator = (ndRobot::ndPoseGenerator*)*sequence;
		const ndVector upDir(location.m_up);
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
				ndSharedPtr<ndBody> thigh (world->GetBody(AddCapsule(scene, bodyMatrix, limbMass, limbRadios, limbRadios, limbLength)));
				thigh->SetMatrix(bodyMatrix);
				ndSharedPtr<ndJointBilateralConstraint> ballJoint (new ndIkJointSpherical(limbPivotLocation, thigh->GetAsBodyKinematic(), torso->GetAsBodyKinematic()));
				thighNode = model->AddLimb(modelRoot, thigh, ballJoint);
		
				limbPivotLocation.m_posit += limbPivotLocation.m_front.Scale(limbLength);
		
				workSpace += limbLength;
			}
		
			// add calf0
			ndModelArticulation::ndNode* calf0Node = nullptr;
			{
				limbPivotLocation = ndRollMatrix(-90.0f * ndDegreeToRad) * limbPivotLocation;
		
				ndMatrix bodyMatrix(limbPivotLocation);
				bodyMatrix.m_posit += limbPivotLocation.m_front.Scale(limbLength * 0.5f);
				ndSharedPtr<ndBody> calf0 (world->GetBody(AddCapsule(scene, bodyMatrix, limbMass, limbRadios, limbRadios, limbLength)));
				calf0->SetMatrix(bodyMatrix);
		
				ndMatrix caffPinAndPivotFrame(ndGetIdentityMatrix());
				ndFloat32 sign = angles[i] > 180.0f ? -1.0f : 1.0f;
				caffPinAndPivotFrame.m_front = limbPivotLocation.m_right.Scale(sign);
				caffPinAndPivotFrame.m_up = limbPivotLocation.m_front;
				caffPinAndPivotFrame.m_right = caffPinAndPivotFrame.m_front.CrossProduct(caffPinAndPivotFrame.m_up);
				caffPinAndPivotFrame.m_posit = limbPivotLocation.m_posit;
				ndSharedPtr<ndJointBilateralConstraint> hingeJoint (new ndIkJointHinge(caffPinAndPivotFrame, calf0->GetAsBodyKinematic(), thighNode->m_body->GetAsBodyKinematic()));
		
				// add joint limit to prevent knee from flipping
				 ndIkJointHinge* const hinge = (ndIkJointHinge*)*hingeJoint;
				hinge->SetLimitState(true);
				hinge->SetLimits(-70.0f * ndDegreeToRad, 70.0f * ndDegreeToRad);
				calf0Node = model->AddLimb(thighNode, calf0, hingeJoint);
		
				limbPivotLocation.m_posit += limbPivotLocation.m_front.Scale(limbLength);
				workSpace += limbLength;
			}
		
			// add calf1
			ndJointHinge* footHinge = nullptr;
			ndModelArticulation::ndNode* footNode = nullptr;
			{
				ndFloat32 lenght = limbLength * 0.5f;
				limbPivotLocation = ndRollMatrix(-45.0f * ndDegreeToRad) * limbPivotLocation;
				ndMatrix bodyMatrix(limbPivotLocation);
				bodyMatrix.m_posit += limbPivotLocation.m_front.Scale(lenght * 0.5f);
		
				ndSharedPtr<ndBody> foot (world->GetBody(AddCapsule(scene, bodyMatrix, limbMass * 0.5f, limbRadios, limbRadios, lenght)));
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
				footHinge = new ndJointHinge(footPinAndPivotFrame, foot->GetAsBodyKinematic(), calf0Node->m_body->GetAsBodyKinematic());
				//footHinge->SetLimitState(true);
				//footHinge->SetLimits(-20.0f * ndDegreeToRad, 20.0f * ndDegreeToRad);
				footHinge->SetAsSpringDamper(0.001f, 2000.0f, 50.0f);
		
				//ndSharedPtr<ndJointBilateralConstraint> hinge(footHinge);
				footNode = model->AddLimb(calf0Node, foot, ndSharedPtr<ndJointBilateralConstraint>(footHinge));
		
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
				effector->SetLinearSpringDamper(regularizer, 2000.0f, 50.0f);
				effector->SetAngularSpringDamper(regularizer, 2000.0f, 50.0f);
				effector->SetWorkSpaceConstraints(0.0f, workSpace * 0.9f);

				ndRobot::ndEffectorInfo info(ndSharedPtr<ndJointBilateralConstraint> (effector), footHinge);
				model->m_effectorsInfo.PushBack(info);

				ndAnimKeyframe keyFrame;
				keyFrame.m_userData = &model->m_effectorsInfo[model->m_effectorsInfo.GetCount() - 1];
				model->m_animPose.PushBack(keyFrame);
				poseGenerator->AddTrack();
				poseGenerator->m_phase[i] = phase[i];
				poseGenerator->m_offset[i] = offset[i];
			}
		}

		#ifdef ND_TRAIN_MODEL
			((ndRobot::ndControllerAgent_trainer*)*agent)->SetModel(model);
			scene->SetAcceleratedUpdate();
		#else
			((ndRobot::ndController*)*agent)->SetModel(model);
		#endif
		
		return model;
	}
#else

	enum ndActionSpace
	{
		m_move_x,
		m_move_z,
		m_actionsSize
	};

	enum ndObsevationSpace
	{
		posit_x,
		posit_y,
		posit_z,
		veloc_x,
		veloc_y,
		veloc_z,
		support,
		desired_support,
		m_observationsSize
	};

	class ndRobot : public ndModelArticulation
	{
		public:
		class ndEffectorInfo
		{
			public:
			ndEffectorInfo()
				:m_footHinge(nullptr)
				,m_effector(nullptr)
			{
			}

			ndEffectorInfo(const ndSharedPtr<ndJointBilateralConstraint>& effector, ndJointHinge* const footHinge)
				:m_footHinge(footHinge)
				,m_effector(effector)
			{
			}

			ndJointHinge* m_footHinge;
			ndSharedPtr<ndJointBilateralConstraint> m_effector;
		};

		class ndPoseGenerator : public ndAnimationSequence
		{
			public:
			ndPoseGenerator(ndFloat32 gaitFraction, const ndFloat32* const phase)
				:ndAnimationSequence()
				,m_amp(0.27f)
				,m_gaitFraction(gaitFraction)
			{
				m_currentPose.SetCount(0);
				m_duration = ndFloat32(4.0f);
				for (ndInt32 i = 0; i < 4; i++)
				{
					m_phase[i] = phase[i];
					m_offset[i] = ndFloat32(0.0f);
					m_currentPose.PushBack(BasePose(i));
				}
			}

			ndVector GetTranslation(ndFloat32) const
			{
				return ndVector::m_zero;
			}

			ndVector BasePose(ndInt32 index) const
			{
				ndVector base(ndVector::m_wOne);
				base.m_x = 0.4f;
				base.m_z = m_offset[index];
				base.m_y = D_POSE_REST_POSITION_Y;
				return base;
			}

			void CalculatePose(ndAnimationPose& output, ndFloat32 param) const
			{
				// generate a procedural in place march gait
				ndAssert(param >= ndFloat32(0.0f));
				ndAssert(param <= ndFloat32(1.0f));

				//param = 0.125f;
				//m_stanceMask = 0x0f;

				ndFloat32 omega = ndPi / m_gaitFraction;
				for (ndInt32 i = 0; i < output.GetCount(); i++)
				{
					output[i].m_userParamInt = 1;
					output[i].m_posit = BasePose(i);
					ndFloat32 t = ndMod(param - m_phase[i] + ndFloat32(1.0f), ndFloat32(1.0f));
					t = m_gaitFraction / 2;
					if (t <= m_gaitFraction)
					{
						if (i == 0)
							//if ((i == 0) || (i == 3))
							//if ((i == 1) || (i == 2))
						{
							//m_stanceMask = m_stanceMask ^ (1 << i);
							output[i].m_userParamInt = 0;
							output[i].m_posit.m_y += m_amp * ndSin(omega * t);
						}
					}
					m_currentPose[i] = output[i].m_posit;
				}
			}

			ndFloat32 m_amp;
			ndFloat32 m_gaitFraction;
			ndFloat32 m_phase[4];
			ndFloat32 m_offset[4];
			mutable ndFixSizeArray<ndVector, 4> m_currentPose;
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
					const ndMatrix& localMatrix = effector->GetLocalMatrix1();
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

		class ndControllerAgent_trainer : public ndBrainAgentContinueVPG_Trainer<m_observationsSize * 4, m_actionsSize>
		{
			public:
			//class ndBasePose
			//{
			//	public:
			//	ndBasePose()
			//		:m_body(nullptr)
			//	{
			//	}
			//
			//	ndBasePose(ndBodyDynamic* const body)
			//		:m_veloc(body->GetVelocity())
			//		,m_omega(body->GetOmega())
			//		,m_posit(body->GetPosition())
			//		,m_rotation(body->GetRotation())
			//		,m_body(body)
			//	{
			//	}
			//
			//	void SetPose() const
			//	{
			//		ndAssert(0);
			//		//m_body->SetMatrix(ndCalculateMatrix(m_rotation, m_posit));
			//		//m_body->SetOmega(m_omega);
			//		//m_body->SetVelocity(m_veloc);
			//	}
			//
			//	ndVector m_veloc;
			//	ndVector m_omega;
			//	ndVector m_posit;
			//	ndQuaternion m_rotation;
			//	ndBodyDynamic* m_body;
			//};

			ndControllerAgent_trainer(const HyperParameters& hyperParameters)
				:ndBrainAgentContinueVPG_Trainer<m_observationsSize * 4, m_actionsSize>(hyperParameters)
				,m_bestActor(m_actor)
				,m_model(nullptr)
				,m_maxGain(-1.0e10f)
				,m_maxFrames(3000)
				,m_startTraning(0)
				,m_stopTraining(5000000)
				,m_timer(ndGetTimeInMicroseconds())
				,m_modelIsTrained(false)
			{
				SetName(CONTROLLER_NAME);
				m_outFile = fopen("quadruped_1-VPG.csv", "wb");
				fprintf(m_outFile, "vpg\n");
			}

			~ndControllerAgent_trainer()
			{
				if (m_outFile)
				{
					fclose(m_outFile);
				}
			}

			void SetModel(ndRobot* const model)
			{
				m_model = model;
				for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
				{
					ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();
					m_bodies.PushBack(body);
					//m_basePose.PushBack(body);
				}
			}

			ndBrainFloat GetReward() const
			{
				//if (IsTerminal())
				//{
				//	return ndBrainFloat(-1.0f);
				//}
				//
				////ndInt32 contactMask = 0x0f;
				////const ndPoseGenerator* const poseGenerator = (ndPoseGenerator*)*m_model->m_poseGenerator->GetSequence();
				////for (ndInt32 i = 0; i < m_model->m_animPose.GetCount(); ++i)
				////{
				////	ndContact* const contact = m_model->FindContact(i);
				////	if (contact)
				////	{
				////		contactMask = contactMask ^ (1 << i);
				////	}
				////}
				////ndInt32 mask = contactMask & poseGenerator->GetStanceCode();
				////
				////ndReal reward = ndReal(0.0f);
				////switch (mask)
				////{
				////	case 0x07:
				////	case 0x0b:
				////	case 0x0d:
				////	case 0x0f:
				////	{
				////		reward = ndReal(1.0f);
				////	}
				////}
				////const ndUIControlNode* const control = m_model->m_control;
				////ndFloat32 control_x_reward = ndExp(-ndFloat32(230.0f) * control->m_x * control->m_x);
				////ndFloat32 control_z_reward = ndExp(-ndFloat32(102.0f) * control->m_z * control->m_z);
				////return ndReal((reward * ndFloat32(8.0f) + control_x_reward + control_z_reward) / ndFloat32(10.0f));
				//
				//
				//
				//
				//ndInt32 stateIndex = 0;
				//ndBrainFloat positReward = ndBrainFloat(0.0f);
				//ndBrainFloat velocReward = ndBrainFloat(0.0f);
				//const ndBrainFloat* const state = &m_currentTransition[0];
				//
				//for (ndInt32 i = 0; i < m_actionsSize / 3; ++i)
				//{
				//	ndEffectorInfo* const info = &m_model->m_effectorsInfo[i];
				//	ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;
				//
				//	ndVector targetPosit(effector->GetEffectorPosit());
				//	//ndVector targetPosit(effector->GetLocalTargetPosition()); very wrong option
				//
				//	ndBrainFloat positError0 = state[stateIndex + m_leg0_anim_posit_x] - ndBrainFloat(targetPosit.m_x);
				//	ndBrainFloat positError1 = state[stateIndex + m_leg0_anim_posit_y] - ndBrainFloat(targetPosit.m_y);
				//	ndBrainFloat positError2 = state[stateIndex + m_leg0_anim_posit_z] - ndBrainFloat(targetPosit.m_z);
				//
				//	ndBrainFloat d20 = ndMin(ndBrainFloat(100000.0f) * positError0 * positError0, ndBrainFloat(30.0f));
				//	positReward += ndBrainFloat(ndExp(-d20));
				//
				//	ndBrainFloat d21 = ndMin(ndBrainFloat(100000.0f) * positError1 * positError1, ndBrainFloat(30.0f));
				//	positReward += ndBrainFloat(ndExp(-d21));
				//
				//	ndBrainFloat d22 = ndMin(ndBrainFloat(100000.0f) * positError2 * positError2, ndBrainFloat(30.0f));
				//	positReward += ndBrainFloat(ndExp(-d22));
				//
				//	ndVector posit;
				//	ndVector veloc;
				//	effector->GetDynamicState(posit, veloc);
				//	ndBrainFloat velocError0 = ndBrainFloat(veloc.m_x);
				//	ndBrainFloat velocError1 = ndBrainFloat(veloc.m_y);
				//	ndBrainFloat velocError2 = ndBrainFloat(veloc.m_z);
				//
				//	ndBrainFloat d23 = ndMin(ndBrainFloat(200.0f) * velocError0 * velocError0, ndBrainFloat(30.0f));
				//	velocReward += ndBrainFloat(ndExp(-d23));
				//
				//	ndBrainFloat d24 = ndMin(ndBrainFloat(200.0f) * velocError1 * velocError1, ndBrainFloat(30.0f));
				//	velocReward += ndBrainFloat(ndExp(-d24));
				//
				//	ndBrainFloat d25 = ndMin(ndBrainFloat(200.0f) * velocError2 * velocError2, ndBrainFloat(30.0f));
				//	velocReward += ndBrainFloat(ndExp(-d25));
				//
				//	stateIndex += 9;
				//}
				//
				//ndBodyKinematic* const body = m_model->GetRoot()->m_body->GetAsBodyKinematic();
				//const ndMatrix& matrix = body->GetMatrix();
				//ndFloat32 sinAngle2 = matrix.m_up.m_x * matrix.m_up.m_x + matrix.m_up.m_z * matrix.m_up.m_z;
				////sinAngle = ndMin(sinAngle, ndFloat32(0.9f));
				//
				//ndBrainFloat orientationReward = (20 * m_actionsSize) * ndExp(-100.0f * sinAngle2);
				//
				//ndBrainFloat den = 2 * m_actionsSize + m_actionsSize + 20 * m_actionsSize;
				//ndBrainFloat num = orientationReward + ndBrainFloat(2.0f) * positReward + velocReward;
				//ndBrainFloat reward = num / den;
				//if (reward > 0.85f)
				//{
				//	ndExpandTraceMessage("%d %f\n", GetFramesCount(), reward);
				//}
				//return reward;
				return 0;
			}

			virtual void ApplyActions(ndBrainFloat* const actions) const
			{
				//m_model->ApplyActions(actions, &m_currentTransition[0]);
				m_model->ApplyActions(actions);
			}

			void GetObservation(ndBrainFloat* const observation)
			{
				m_model->GetObservation(observation);
				//ndMemCpy(&m_currentTransition[0], observation, m_observationsSize);
			}

			bool IsTerminal() const
			{
				ndAssert(0);
				////ndBrainFloat maxVal = ndBrainFloat(0.0f);
				////const ndBrainFloat* const stateActions = &m_currentTransition.m_observation[0];
				////ndInt32 actionIndex = 0;
				////for (ndInt32 i = 0; i < m_model->m_animPose.GetCount(); ++i)
				////{
				////	ndBrainFloat error0 = stateActions[actionIndex + m_leg0_anim_posit_x] - stateActions[actionIndex + m_leg0_posit_x];
				////	ndBrainFloat error1 = stateActions[actionIndex + m_leg0_anim_posit_y] - stateActions[actionIndex + m_leg0_posit_y];
				////	ndBrainFloat error2 = stateActions[actionIndex + m_leg0_anim_posit_z] - stateActions[actionIndex + m_leg0_posit_z];
				////	maxVal = ndMax(maxVal, ndAbs(error0));
				////	maxVal = ndMax(maxVal, ndAbs(error1));
				////	maxVal = ndMax(maxVal, ndAbs(error2));
				////	actionIndex += (m_leg1_anim_posit_x - m_leg0_anim_posit_x);
				////  actionIndex += 9;
				////}
				////ndFloat32 dev = ndBrainFloat(0.5f) * D_EFFECTOR_STEP;
				////bool state = (maxVal > dev) ? true : false;
				////state = state && (m_startTraning >= 64);
				//
				//return m_model->IsTerminal();
				return false;
			}

			void ResetModel() const
			{
				ndAssert(0);
				//m_model->m_control->Reset();
				//for (ndInt32 i = 0; i < m_basePose.GetCount(); i++)
				//{
				//	m_basePose[i].SetPose();
				//}
				//
				//ndUnsigned32 index = (ndRandInt() >> 1) % 4;
				//ndFloat32 duration = m_model->m_poseGenerator->GetSequence()->GetDuration();
				//m_model->m_poseGenerator->SetTime(duration * ndFloat32(index) / 4.0f);
				//m_model->m_poseGenerator->SetTime(duration * ndFloat32(index) / 4.0f);
				//m_model->ApplyPoseGeneration();
				//
				//m_startTraning = 0;
			}

			void OptimizeStep()
			{
				ndAssert(0);
				//ndInt32 stopTraining = GetFramesCount();
				//if (stopTraining <= m_stopTraining)
				//{
				//	ndInt32 episodeCount = GetEposideCount();
				//	ndBrainAgentContinueVPG_Trainer::OptimizeStep();
				//
				//	episodeCount -= GetEposideCount();
				//	if (m_averageFramesPerEpisodes.GetAverage() >= ndFloat32(m_maxFrames))
				//	{
				//		if (m_averageQvalue.GetAverage() > m_maxGain)
				//		{
				//			m_bestActor.CopyFrom(m_actor);
				//			m_maxGain = m_averageQvalue.GetAverage();
				//			ndExpandTraceMessage("best actor episode: %d\taverageFrames: %f\taverageValue %f\n", GetEposideCount(), m_averageFramesPerEpisodes.GetAverage(), m_averageQvalue.GetAverage());
				//		}
				//	}
				//
				//	if (episodeCount && !IsSampling())
				//	{
				//		ndExpandTraceMessage("step: %d\treward: %g\tframes: %g\n", GetFramesCount(), m_averageQvalue.GetAverage(), m_averageFramesPerEpisodes.GetAverage());
				//		if (m_outFile)
				//		{
				//			fprintf(m_outFile, "%g\n", m_averageQvalue.GetAverage());
				//			fflush(m_outFile);
				//		}
				//	}
				//
				//	if (stopTraining == m_stopTraining)
				//	{
				//		char fileName[1024];
				//		m_modelIsTrained = true;
				//		m_actor.CopyFrom(m_bestActor);
				//		ndGetWorkingFileName(GetName().GetStr(), fileName);
				//		SaveToFile(fileName);
				//		ndExpandTraceMessage("saving to file: %s\n", fileName);
				//		ndExpandTraceMessage("training complete\n");
				//		ndUnsigned64 timer = ndGetTimeInMicroseconds() - m_timer;
				//		ndExpandTraceMessage("training time: %g\n seconds", ndFloat32(ndFloat64(timer) * ndFloat32(1.0e-6f)));
				//	}
				//}
				//
				////if (m_model->IsOutOfBounds())
				////{
				////	m_model->TelePort();
				////}
				//m_startTraning++;
			}

			FILE* m_outFile;
			ndBrain m_bestActor;
			ndRobot* m_model;
			ndFloat32 m_maxGain;
			ndInt32 m_maxFrames;
			mutable ndInt32 m_startTraning;
			ndInt32 m_stopTraining;
			ndUnsigned64 m_timer;
			bool m_modelIsTrained;
			//ndFixSizeArray<ndBasePose, 32> m_basePose;
			ndFixSizeArray<ndBodyDynamic*, 32> m_bodies;
			//ndBrainFixSizeVector<m_observationsSize> m_currentTransition;
		};

		ndRobot(ndSharedPtr<ndBrainAgent>& agent)
			:ndModelArticulation()
			,m_animPose()
			,m_control(nullptr)
			,m_poseGenerator(nullptr)
			,m_effectorsInfo()
			,m_animBlendTree()
			,m_agent(agent)
		{
		}

		ndVector CalculateZeroMomentPoint() const
		{
			ndFixSizeArray<ndVector, 32> r;
			ndFixSizeArray<const ndBodyKinematic*, 32> bodies;

			//ndVector centerOfPresure;

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
			com = com.Scale(ndFloat32 (1.0f) / totalMass);

			ndVector force(ndVector::m_zero);
			ndVector torque(ndVector::m_zero);
			const ndVector gravity(ndFloat32(0.0f), DEMO_GRAVITY, ndFloat32(0.0f), ndFloat32(0.0f));
			for (ndInt32 i = 0; i < bodies.GetCount(); ++i)
			{
				const ndVector centerOfMass (r[i] - com);
				const ndBodyKinematic* const body = bodies[i];
				const ndMatrix bodyInertia(body->CalculateInertiaMatrix());
				const ndVector bodyForce((body->GetAccel() - gravity).Scale (body->GetMassMatrix().m_w));

				force += bodyForce;
				torque += centerOfMass.CrossProduct(bodyForce);
				torque += bodyInertia.RotateVector(body->GetAlpha());
			}
			// remember to clamp the values values before calculating xZmp and zZmp
			if (ndAbs(force.m_y) > ndFloat32(0.0f))
			{
				ndAssert(ndAbs(force.m_y) > ndFloat32(0.0f));
				ndFloat32 zZmp = torque.m_z / force.m_y;
				ndFloat32 xZmp = -torque.m_x / force.m_y;
				//ndTrace(("x=%f z=%f\n", xZmp, zZmp));

				com.m_x += xZmp;
				com.m_z += zZmp;
			}
			return com;
		}

		void Debug(ndConstraintDebugCallback& context) const
		{
			ndBodyKinematic* const rootBody = GetRoot()->m_body->GetAsBodyKinematic();
			//ndSkeletonContainer* const skeleton = rootBody->GetSkeleton();
			//if (!skeleton)
			//{
			//	return;
			//}

			//ndFixSizeArray<ndJointBilateralConstraint*, 32> effectors;
			//for (ndInt32 i = 0; i < m_actionsSize / 3; ++i)
			//{
			//	ndEffectorInfo* const info = (ndEffectorInfo*)m_animPose[i].m_userData;
			//	ndAssert(info == &m_effectorsInfo[i]);
			//	effectors.PushBack(*info->m_effector);
			//
			//	ndContact* const contact = FindContact(i);
			//	if (contact)
			//	{
			//		m_world->CalculateJointContacts(contact);
			//	}
			//}
			//
			//ndIkSolver* const invDynamicsSolver = (ndIkSolver*)&m_invDynamicsSolver;
			//invDynamicsSolver->SolverBegin(skeleton, &effectors[0], effectors.GetCount(), m_world, m_timestep);
			//invDynamicsSolver->Solve();
			//
			////a) Mt = sum(m(i))
			////b) cg = sum(p(i) * m(i)) / Mt
			////c) Vcg = sum(v(i) * m(i)) / Mt
			////d) Icg = sum(I(i) +  m(i) * (Identity * ((p(i) - cg) * transpose (p(i) - cg)) - covarianMatrix(p(i) - cg))
			////e) T0 = sum[w(i) x (I(i) * w(i)) - Vcg x (m(i) * v(i))]
			////f) T1 = sum[(p(i) - cg) x Fext(i) + Text(i)]
			////g) Bcg = (Icg ^ -1) * (T0 + T1)
			//
			//ndBodyState state;
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

			//state.m_com.m_posit = state.m_com.m_posit.Scale(invMass);
			//state.m_com.m_posit.m_w = ndFloat32(1.0f);
			//ndMatrix comMatrix(m_rootNode->m_body->GetAsBodyKinematic()->GetMatrix());
			//comMatrix.m_posit = state.m_com.m_posit;
			//context.DrawFrame(comMatrix);

			ndVector comLineOfAction(centerOfMass);
			comLineOfAction.m_y -= ndFloat32(0.5f);
			context.DrawLine(centerOfMass, comLineOfAction, ndVector::m_zero);
			//comLineOfAction.m_y = comMatrix.m_posit.m_y - 0.38f;
			//context.DrawPoint(comLineOfAction, ndVector(0.0f, 0.0f, 1.0f, 1.0f), 10);
			
			//ndVector netTorque(ndVector::m_zero);
			//ndVector netTorque1(ndVector::m_zero);
			//const ndVector gravity(ndFloat32(0.0f), DEMO_GRAVITY, ndFloat32(0.0f), ndFloat32(0.0f));
			//for (ndInt32 i = 0; i < bodies.GetCount(); ++i)
			//{
			//	const ndBodyKinematic* const body = bodies[i];
			//	const ndMatrix matrix(body->GetMatrix());
			//	const ndVector comDist((matrix.TransformVector(body->GetCentreOfMass()) - centerOfMass) & ndVector::m_triplexMask);
			//
			//	const ndVector omega(body->GetOmega());
			//	const ndVector accel(body->GetAccel());
			//	const ndMatrix bodyInertia(body->CalculateInertiaMatrix());
			//
			//	const ndVector angleTorque (bodyInertia.RotateVector(body->GetAlpha()));
			//	const ndVector gyroTorque(omega.CrossProduct(bodyInertia.RotateVector(omega)));
			//	
			//
			////	//ndVector force(invDynamicsSolver->GetBodyForce(body));
			////	//ndVector torque (invDynamicsSolver->GetBodyTorque(body));
			////	ndVector torque(bodyInertia.RotateVector(body->GetAlpha()));
			////	ndVector force(body->GetAccel().Scale(body->GetMassMatrix().m_w));
			////	torque += comDist.CrossProduct(force);
			////	torque += omega.CrossProduct(bodyInertia.RotateVector(omega));
			////
			////	ndJacobian force1(body->CalculateNetForce());
			////	force1.m_angular += comDist.CrossProduct(force1.m_linear);
			////	force1.m_angular += omega.CrossProduct(bodyInertia.RotateVector(omega));
			////
			////	netTorque += torque;
			////	netTorque1 += force1.m_angular;
			//}

			//invDynamicsSolver->SolverEnd();
			//ndVector weight(ndVector::m_zMask);
			//weight.m_y = -state.m_mass * DEMO_GRAVITY;
			//ndVector zmpLineOfAction(comLineOfAction);
			////zmpLineOfAction.m_z += netTorque.m_x / weight.m_y;
			////zmpLineOfAction.m_x += -netTorque.m_z / weight.m_y;
			//zmpLineOfAction.m_z += netTorque1.m_x / weight.m_y;
			//zmpLineOfAction.m_x += -netTorque1.m_z / weight.m_y;
			//context.DrawPoint(zmpLineOfAction, ndVector(1.0f, 0.0f, 0.0f, 1.0f), 6);
			//const ndPoseGenerator* const poseGenerator = (ndPoseGenerator*)*m_poseGenerator->GetSequence();
			
			const ndVector upVector(rootBody->GetMatrix().m_up);
			ndFixSizeArray<ndVector, 4> desiredSupportPoint;
			for (ndInt32 i = 0; i < m_animPose.GetCount(); ++i)
			{
				const ndAnimKeyframe& keyFrame = m_animPose[i];
				ndEffectorInfo* const info = (ndEffectorInfo*)keyFrame.m_userData;
				ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;

				if (i == 0)
				{
					effector->DebugJoint(context);
				}

				if (keyFrame.m_userParamInt)
				{
					ndBodyKinematic* const body = effector->GetBody0();
					desiredSupportPoint.PushBack(body->GetMatrix().TransformVector(effector->GetLocalMatrix0().m_posit));
				}
			}
			
			ndVector supportColor(0.0f, 1.0f, 1.0f, 1.0f);
			if (desiredSupportPoint.GetCount() >= 3)
			{
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
			else if (desiredSupportPoint.GetCount() == 2)
			{
				ndAssert(0);
				context.DrawLine(desiredSupportPoint[0], desiredSupportPoint[1], supportColor);
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

		void UpdatePose(ndFloat32 timestep)
		{
			ndBodyKinematic* const rootBody = GetRoot()->m_body->GetAsBodyKinematic();
			ndSkeletonContainer* const skeleton = rootBody->GetSkeleton();
			ndAssert(skeleton);
			ndFixSizeArray<ndJointBilateralConstraint*, 32> effectors;

			ndVector veloc;
			m_animBlendTree->Evaluate(m_animPose, veloc);

			//const ndVector upVector(0.0f, 1.0f, 0.0f, 0.0f);
			const ndVector upVector(rootBody->GetMatrix().m_up);

			for (ndInt32 i = 0; i < m_animPose.GetCount(); ++i)
			{
				ndEffectorInfo* const info = &m_effectorsInfo[i];
				effectors.PushBack(*info->m_effector);

				ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;

				ndVector posit(m_animPose[i].m_posit);
				effector->SetLocalTargetPosition(posit);

				//effector->SetSwivelAngle(actions[actionIndex + m_leg0_action_posit_swivel]);
				ndFloat32 swivelAngle = effector->CalculateLookAtSwivelAngle(upVector);
				effector->SetSwivelAngle(swivelAngle);
				//effector->SetSwivelAngle(0.0f);

				// calculate lookAt angle
				ndMatrix lookAtMatrix0;
				ndMatrix lookAtMatrix1;
				info->m_footHinge->CalculateGlobalMatrix(lookAtMatrix0, lookAtMatrix1);

				ndMatrix upMatrix(ndGetIdentityMatrix());
				upMatrix.m_front = lookAtMatrix1.m_front;
				upMatrix.m_right = (upMatrix.m_front.CrossProduct(upVector) & ndVector::m_triplexMask).Normalize();
				upMatrix.m_up = upMatrix.m_right.CrossProduct(upMatrix.m_front);
				upMatrix = upMatrix * lookAtMatrix0.OrthoInverse();

				const ndFloat32 angle = ndAtan2(upMatrix.m_up.m_z, upMatrix.m_up.m_y);
				info->m_footHinge->SetTargetAngle(angle);
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

		//void ApplyActions(ndBrainFloat* const actions, const ndBrainFloat* const currentState)
		void ApplyActions(ndBrainFloat* const actions)
		{
			////m_control->m_animSpeed = 4.0f;
			////m_control->m_animSpeed = 2.0f;
			////m_control->m_animSpeed = 1.0f;
			////m_control->m_animSpeed = 0.5f;
			//m_control->m_animSpeed = 0.25f;
			////m_control->m_animSpeed = 0.1f;
			////if (m_control->m_enableController)
			////{
			////	//m_control->m_x = ndClamp(ndReal(m_control->m_x + actions[m_bodySwing_x] * D_SWING_STEP), -D_MAX_SWING_DIST_X, D_MAX_SWING_DIST_X);
			////	//m_control->m_z = ndClamp(ndReal(m_control->m_z + actions[m_bodySwing_z] * D_SWING_STEP), -D_MAX_SWING_DIST_Z, D_MAX_SWING_DIST_Z);
			////}
			//
			//ndInt32 stateIndex = 0;
			//ndInt32 actionIndex = 0;
			//ndBrainFloat combinedActions[m_actionsSize * 2];
			//for (ndInt32 i = 0; i < m_actionsSize / 3; ++i)
			//{
			//	combinedActions[actionIndex + m_leg0_action_posit_x] = currentState[stateIndex + m_leg0_anim_posit_x] + actions[actionIndex + m_leg0_action_posit_x] * D_EFFECTOR_STEP;
			//	combinedActions[actionIndex + m_leg0_action_posit_y] = currentState[stateIndex + m_leg0_anim_posit_y] + actions[actionIndex + m_leg0_action_posit_y] * D_EFFECTOR_STEP;
			//	combinedActions[actionIndex + m_leg0_action_posit_z] = currentState[stateIndex + m_leg0_anim_posit_z] + actions[actionIndex + m_leg0_action_posit_z] * D_EFFECTOR_STEP;
			//	//actions[actionIndex + m_leg0_action_posit_swivel] = m_currentTransition.m_observation[stateIndex + m_leg0_anim_posit_swivel] + actions[actionIndex + m_leg0_action_posit_swivel] * D_SWING_STEP;
			//
			//	actionIndex += 3;
			//	stateIndex += 9;
			//}
			//
			//ApplyPoseGeneration(combinedActions);

			m_control->m_x = ndClamp(ndReal(m_control->m_x + actions[m_move_x] * D_SWING_STEP), -D_MAX_SWING_DIST_X, D_MAX_SWING_DIST_X);
			m_control->m_z = ndClamp(ndReal(m_control->m_z + actions[m_move_z] * D_SWING_STEP), -D_MAX_SWING_DIST_Z, D_MAX_SWING_DIST_Z);

			UpdatePose(m_timestep);
		}

		void GetObservation(ndBrainFloat* const observation)
		{
			for (ndInt32 i = 0; i < m_animPose.GetCount(); ++i)
			{
				const ndAnimKeyframe& keyFrame = m_animPose[i];
				const ndEffectorInfo* const info = (ndEffectorInfo*)keyFrame.m_userData;
				const ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;

				ndVector effectPositState;
				ndVector effectVelocState;
				effector->GetDynamicState(effectPositState, effectVelocState);

				ndBrainFloat* const input = &observation[i * m_observationsSize];
				input[posit_x] = ndBrainFloat(effectPositState.m_x);
				input[posit_y] = ndBrainFloat(effectPositState.m_y);
				input[posit_z] = ndBrainFloat(effectPositState.m_z);
				input[veloc_x] = ndBrainFloat(effectVelocState.m_x);
				input[veloc_y] = ndBrainFloat(effectVelocState.m_y);
				input[veloc_z] = ndBrainFloat(effectVelocState.m_z);
				input[support] = FindContact(i) ? ndFloat32(1.0f) : ndFloat32(0.0f);
				input[desired_support] = keyFrame.m_userParamInt ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
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
			//m_animBlendTree->Update(timestep * m_control->m_animSpeed);
			ndModelArticulation::PostUpdate(world, timestep);
			//m_agent->OptimizeStep();
			//CheckTrainingCompleted();
		}

		ndAnimationPose m_animPose;
		ndUIControlNode* m_control;
		ndAnimationSequencePlayer* m_poseGenerator;
		ndFixSizeArray<ndEffectorInfo, 4> m_effectorsInfo;
		ndSharedPtr<ndAnimationBlendTreeNode> m_animBlendTree;
		ndSharedPtr<ndBrainAgent> m_agent;
		ndFloat32 m_timestep;
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
			change = change || ImGui::SliderFloat("posit x", &control->m_x, -D_MAX_SWING_DIST_X, D_MAX_SWING_DIST_X);
			change = change || ImGui::SliderFloat("posit y", &control->m_y, -0.2f, 0.1f);
			change = change || ImGui::SliderFloat("posit z", &control->m_z, -D_MAX_SWING_DIST_Z, D_MAX_SWING_DIST_Z);
			change = change || ImGui::SliderFloat("pitch", &control->m_pitch, -15.0f, 15.0f);
			change = change || ImGui::SliderFloat("yaw", &control->m_yaw, -20.0f, 20.0f);
			change = change || ImGui::SliderFloat("roll", &control->m_roll, -15.0f, 15.0f);
			change = change || ImGui::SliderFloat("animSpeed", &control->m_animSpeed, 0.0f, 4.0f);
			change = change || ImGui::Checkbox("enable controller", &control->m_enableController);

			if (change)
			{
				ndBodyKinematic* const body = m_model->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
				body->SetSleepState(false);
			}
		}

		ndSharedPtr<ndModel> m_model;
	};

	ndSharedPtr<ndBrainAgent> BuildAgent()
	{
		#ifdef ND_TRAIN_MODEL
		// add a reinforcement learning controller 
		ndBrainAgentContinueVPG_Trainer<m_observationsSize * 4, m_actionsSize>::HyperParameters hyperParameters;
		//hyperParameters.m_sigma = ndReal(0.25f);
		hyperParameters.m_discountFactor = ndReal(0.99f);
		hyperParameters.m_extraTrajectorySteps = 6000;
		ndSharedPtr<ndBrainAgent> agent(new ndRobot::ndControllerAgent_trainer(hyperParameters));
		#else
		char fileName[1024];
		ndGetWorkingFileName(CONTROLLER_NAME, fileName);
		ndSharedPtr<ndBrain> actor(ndBrainLoad::Load(fileName));
		ndSharedPtr<ndBrainAgent> agent(new ndRobot::ndController(actor));
		#endif
		return agent;
	}

	ndModelArticulation* BuildModel(ndDemoEntityManager* const scene, const ndMatrix& matrixLocation)
	{
		ndFloat32 mass = 20.0f;
		ndFloat32 radius = 0.25f;
		ndFloat32 limbMass = 0.25f;
		ndFloat32 limbLength = 0.3f;
		ndFloat32 limbRadios = 0.05f;

		ndSharedPtr<ndBrainAgent> agent(BuildAgent());
		ndRobot* const model = new ndRobot(agent);

		ndPhysicsWorld* const world = scene->GetWorld();
		ndVector floor(FindFloor(*world, matrixLocation.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		ndSharedPtr<ndBody> torso(world->GetBody(AddSphere(scene, matrixLocation, mass, radius, "smilli.tga")));
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
		ndFloat32 offset[] = { -0.3f, 0.3f, -0.3f, 0.3f };
		////ndFloat32 phase[] = { 0.0f, 0.5f, 0.0f, 0.5f};
		////ndSharedPtr<ndAnimationSequence> sequence(new ndRobot::ndPoseGenerator(0.47f, phase));
		
		ndFloat32 phase[] = { 0.0f, 0.5f, 0.25f, 0.75f };
		ndSharedPtr<ndAnimationSequence> sequence(new ndRobot::ndPoseGenerator(0.24f, phase));
		
		model->m_poseGenerator = new ndAnimationSequencePlayer(sequence);
		model->m_control = new ndRobot::ndUIControlNode(model->m_poseGenerator);
		model->m_animBlendTree = ndSharedPtr<ndAnimationBlendTreeNode>(model->m_control);
		
		ndRobot::ndPoseGenerator* const poseGenerator = (ndRobot::ndPoseGenerator*)*sequence;
		//const ndVector upDir(location.m_up);
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
			ndModelArticulation::ndNode* calf0Node = nullptr;
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
				calf0Node = model->AddLimb(thighNode, calf0, hingeJoint);
		
				limbPivotLocation.m_posit += limbPivotLocation.m_front.Scale(limbLength);
				workSpace += limbLength;
			}
		
			// add calf1
			ndJointHinge* footHinge = nullptr;
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
				footHinge = new ndJointHinge(footPinAndPivotFrame, foot->GetAsBodyKinematic(), calf0Node->m_body->GetAsBodyKinematic());
				//footHinge->SetLimitState(true);
				//footHinge->SetLimits(-20.0f * ndDegreeToRad, 20.0f * ndDegreeToRad);
				footHinge->SetAsSpringDamper(0.001f, 2000.0f, 50.0f);
		
				//ndSharedPtr<ndJointBilateralConstraint> hinge(footHinge);
				footNode = model->AddLimb(calf0Node, foot, ndSharedPtr<ndJointBilateralConstraint>(footHinge));
		
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
				effector->SetLinearSpringDamper(regularizer, 2000.0f, 50.0f);
				effector->SetAngularSpringDamper(regularizer, 2000.0f, 50.0f);
				effector->SetWorkSpaceConstraints(0.0f, workSpace * 0.9f);
		
				ndRobot::ndEffectorInfo info(ndSharedPtr<ndJointBilateralConstraint>(effector), footHinge);
				model->m_effectorsInfo.PushBack(info);
		
				ndAnimKeyframe keyFrame;
				keyFrame.m_userData = &model->m_effectorsInfo[model->m_effectorsInfo.GetCount() - 1];
				model->m_animPose.PushBack(keyFrame);
				poseGenerator->AddTrack();
				poseGenerator->m_phase[i] = phase[i];
				poseGenerator->m_offset[i] = offset[i];
			}
		}
		
		#ifdef ND_TRAIN_MODEL
			((ndRobot::ndControllerAgent_trainer*)*agent)->SetModel(model);
			scene->SetAcceleratedUpdate();
		#else
			((ndRobot::ndController*)*agent)->SetModel(model);
		#endif

		return model;
	}

#endif
}

using namespace ndQuadruped_1;

void ndQuadrupedTest_1(ndDemoEntityManager* const scene)
{
	// build a floor
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
	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(ndYawMatrix(-0.0f * ndDegreeToRad));
	
	ndSharedPtr<ndModel> model(BuildModel(scene, matrix));
	world->AddModel(model);

	ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(model->GetAsModelArticulation()->GetRoot()->m_body->GetMatrix(), model->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic(), world->GetSentinelBody()));
	//world->AddJoint(fixJoint);

	ndSharedPtr<ndUIEntity> quadrupedUI (new ndModelUI(scene, model));
	scene->Set2DDisplayRenderFunction(quadrupedUI);
	
	matrix.m_posit.m_x -= 4.0f;
	matrix.m_posit.m_y += 1.5f;
	matrix.m_posit.m_z += 0.25f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);

	//ndFileFormatSave xxxx;
	//xxxx.SaveWorld(scene->GetWorld(), "xxxx.nd");
}
