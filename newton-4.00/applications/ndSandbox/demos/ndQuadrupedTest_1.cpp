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
	#define D_SWING_STEP			ndFloat32 (0.01f)
	#define D_MAX_SWING_DIST		ndFloat32 (0.15f)
	#define D_MIN_REWARD_ANGLE		(ndFloat32 (20.0f) * ndDegreeToRad)

	#define D_POSE_REST_POSITION_Y	ndFloat32 (-0.3f)

	enum ndActionSpace
	{
		m_bodySwing,
		m_actionsSize
	};

	enum ndStateSpace
	{
		m_leg0_y,
		m_leg1_y,
		m_leg2_y,
		m_leg3_y,
		m_body_swing,
		m_body_omega_x,
		m_body_omega_z,
		m_stateSize
	};

	class ndModelQuadruped: public ndModelArticulation
	{
		public:
		class ndBodyState
		{
			public:
			ndBodyState()
				:m_com(ndGetZeroMatrix())
				,m_inertia(ndGetZeroMatrix())
				,m_veloc(ndVector::m_zero)
				,m_omega(ndVector::m_zero)
				,m_mass(0.0f)
			{
			}

			ndMatrix m_com;
			ndMatrix m_inertia;
			ndVector m_veloc;
			ndVector m_omega;
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
			ndPoseGenerator()
				:ndAnimationSequence()
			{
				m_duration = ndFloat32 (4.0f);
				for (ndInt32 i = 0; i < 4; i++)
				{
					m_phase[i] = ndFloat32 (0.0f);
					m_offset[i] = ndFloat32(0.0f);
				}
			}

			ndVector GetTranslation(ndFloat32) const
			{
				return ndVector::m_zero;
			}

			void CalculatePose(ndAnimationPose& output, ndFloat32 param) const
			{
				// generate a procedural in place march gait
				const ndFloat32 gaitFraction = 0.47f;
				ndFloat32 amp = 0.27f;
				ndFloat32 omega = ndPi / gaitFraction;

				ndVector base (ndVector::m_wOne);
				base.m_y = D_POSE_REST_POSITION_Y;
				base.m_x = 0.4f;
				for (ndInt32 i = 0; i < 4; i++)
				{
					output[i].m_posit = base;
					output[i].m_posit.m_z = m_offset[i];
					ndFloat32 t = ndMod (param - m_phase[i] + ndFloat32(1.0f), ndFloat32 (1.0f));
					if (t <= gaitFraction)
					{
						output[i].m_posit.m_y += amp * ndSin(omega * t);
					}
					output[i].m_rotation.m_w = ((t > ndFloat32(0.0f)) && (t < gaitFraction)) ? ndFloat32(0.0f) : ndFloat32(1.0f);
				}
			}

			ndFloat32 m_phase[4];
			ndFloat32 m_offset[4];
		};

		// implement controller player
		class ndControllerAgent : public ndBrainAgentDDPG<m_stateSize, m_actionsSize>
		{
			public:
			ndControllerAgent(ndSharedPtr<ndBrain>& actor)
				:ndBrainAgentDDPG<m_stateSize, m_actionsSize>(actor)
				,m_model(nullptr)
			{
			}

			void SetModel(ndModelQuadruped* const model)
			{
				m_model = model;
			}

			void GetObservation(ndReal* const state) const
			{
				m_model->GetObservation(state);
			}

			virtual void ApplyActions(ndReal* const actions) const
			{
				//if (!m_model->HasSupportContact())
				//{
				//	for (ndInt32 i = 0; i < m_actionsSize; ++i)
				//	{
				//		actions[i] = ndReal(0.0f);
				//	}
				//}
				m_model->ApplyActions(actions);
			}

			ndModelQuadruped* m_model;
		};

		class ndControllerAgent_trainer: public ndBrainAgentTD3_Trainer<m_stateSize, m_actionsSize>
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

			ndControllerAgent_trainer(ndSharedPtr<ndBrain>& actor, ndSharedPtr<ndBrain>& critic)
				:ndBrainAgentTD3_Trainer<m_stateSize, m_actionsSize>(actor, critic)
				,m_model(nullptr)
				,m_maxGain(-1.0e10f)
				,m_maxFrames(300)
				,m_stopTraining(3000000)
				//, m_stopTraining(2000)
				,m_averageQValue()
				,m_averageFramesPerEpisodes()
			{
				//SetActionNoise(ndReal(0.15f));
				SetActionNoise(ndReal(0.20f));
				SetLearnRate(ndReal(1.0e-3f));

				m_outFile = fopen("traingPerf-TD3.csv", "wb");
				fprintf(m_outFile, "td3\n");
				m_timer = ndGetTimeInMicroseconds();
			}

			~ndControllerAgent_trainer()
			{
				if (m_outFile)
				{
					fclose(m_outFile);
				}
			}

			void SetModel(ndModelQuadruped* const model)
			{
				m_model = model;
				for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
				{
					ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();
					m_bodies.PushBack(body);
					m_basePose.PushBack(body);
				}
			}

			ndReal GetReward() const
			{
				return m_model->GetReward();
			}

			virtual void ApplyActions(ndReal* const actions) const
			{
				if (GetEpisodeFrames() >= 10000)
				{
					const ndRandom& random = GetRandomGenerator(0);
					for (ndInt32 i = 0; i < m_actionsSize; ++i)
					{
						ndReal gaussianNoise = ndReal(random.GetGaussianRandom(ndFloat32(actions[i]), ndFloat32(1.0f)));
						ndReal clippiedNoisyAction = ndClamp(gaussianNoise, ndReal(-1.0f), ndReal(1.0f));
						actions[i] = clippiedNoisyAction;
					}
				}
				m_model->ApplyActions(actions);
			}

			void GetObservation(ndReal* const state) const
			{
				m_model->GetObservation(state);
			}

			bool IsTerminal() const
			{
				bool state = m_model->IsTerminal();
				if (!IsSampling())
				{
					if (GetEpisodeFrames() >= 15000)
					{
						state = true;
					}
					m_averageQValue.Update(GetCurrentValue());
					if (state)
					{
						m_averageFramesPerEpisodes.Update(ndReal(GetEpisodeFrames()));
					}
				}
				return state;
			}

			void ResetModel() const
			{
				m_model->m_control->m_z = ndReal(0.0f);
				for (ndInt32 i = 0; i < m_basePose.GetCount(); i++)
				{
					m_basePose[i].SetPose();
				}
			}

			void OptimizeStep()
			{
				ndInt32 stopTraining = GetFramesCount();
				if (stopTraining <= m_stopTraining)
				{
					ndInt32 episodeCount = GetEposideCount();
					ndBrainAgentDDPG_Trainer::OptimizeStep();
				
					episodeCount -= GetEposideCount();
					if (m_averageFramesPerEpisodes.GetAverage() >= ndFloat32(m_maxFrames))
					{
						if (m_averageQValue.GetAverage() > m_maxGain)
						{
							char fileName[1024];
							ndGetWorkingFileName(GetName().GetStr(), fileName);
							SaveToFile(fileName);

							ndExpandTraceMessage("saving to file: %s\n", fileName);
							ndExpandTraceMessage("episode: %d\taverageFrames: %f\taverageValue %f\n\n", GetEposideCount(), m_averageFramesPerEpisodes.GetAverage(), m_averageQValue.GetAverage());
							m_maxGain = m_averageQValue.GetAverage();
						}
					}
				
					if (episodeCount && !IsSampling())
					{
						ndExpandTraceMessage("%g %g\n", m_averageQValue.GetAverage(), m_averageFramesPerEpisodes.GetAverage());
						if (m_outFile)
						{
							fprintf(m_outFile, "%g\n", m_averageQValue.GetAverage());
							fflush(m_outFile);
						}
					}
				
					if (stopTraining == m_stopTraining)
					{
						ndExpandTraceMessage("\n");
						ndExpandTraceMessage("training complete\n");
						ndUnsigned64 timer = ndGetTimeInMicroseconds() - m_timer;
						ndExpandTraceMessage("time training: %f\n", ndFloat32(ndFloat64(timer) * ndFloat32(1.0e-6f)));
					}
				}
				//if (m_model->IsOutOfBounds())
				//{
				//	m_model->TelePort();
				//}
			}

			FILE* m_outFile;
			ndModelQuadruped* m_model;
			ndFloat32 m_maxGain;
			ndInt32 m_maxFrames;
			ndInt32 m_stopTraining;
			ndUnsigned64 m_timer;
			ndFixSizeArray<ndBasePose, 32> m_basePose;
			ndFixSizeArray<ndBodyDynamic*, 32> m_bodies;
			mutable ndMovingAverage<128> m_averageQValue;
			mutable ndMovingAverage<128> m_averageFramesPerEpisodes;
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
			{
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
		};
		
		ndModelQuadruped(ndSharedPtr<ndBrainAgent>& agent)
			:ndModelArticulation()
			,m_agent(agent)
		{
		}

		void Debug(ndConstraintDebugCallback& context) const
		{
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

			ndFixSizeArray<ndVector, 4> contactPoints;
			for (ndInt32 i = 0; i < m_animPose.GetCount(); ++i)
			{
				const ndAnimKeyframe& keyFrame = m_animPose[i];
				if (keyFrame.m_rotation.m_w > ndFloat32(0.0f))
				{
					ndEffectorInfo* const info = (ndEffectorInfo*)keyFrame.m_userData;
					ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;
					ndBodyKinematic* const body = effector->GetBody0();
					contactPoints.PushBack(body->GetMatrix().TransformVector(effector->GetLocalMatrix0().m_posit));
				}
			}
		
			ndBodyState bodyState(CalculateFullBodyState());
			ndMatrix comMatrix(m_rootNode->m_body->GetAsBodyKinematic()->GetMatrix());
			context.DrawFrame(bodyState.m_com);
			
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
					context.DrawLine(contactPoints[i], p0, ndVector::m_zero);
					p0 = contactPoints[i];
				}
			
				ndBigVector p0Out;
				ndBigVector p1Out;
				ndBigVector ray_p0(comMatrix.m_posit);
				ndBigVector ray_p1(comMatrix.m_posit);
				ray_p1.m_y -= 1.0f;
				
				ndRayToPolygonDistance(ray_p0, ray_p1, bigPolygon, supportCount, p0Out, p1Out);
				context.DrawPoint(p0Out, ndVector(1.0f, 0.0f, 0.0f, 1.0f), 3);
				context.DrawPoint(p1Out, ndVector(0.0f, 1.0f, 0.0f, 1.0f), 3);
			}
			else if (contactPoints.GetCount() == 2)
			{
				context.DrawLine(contactPoints[0], contactPoints[1], ndVector::m_zero);
			
				ndBigVector p0Out;
				ndBigVector p1Out;
				ndBigVector ray_p0(comMatrix.m_posit);
				ndBigVector ray_p1(comMatrix.m_posit);
				ray_p1.m_y -= 1.0f;
			
				ndRayToRayDistance(ray_p0, ray_p1, contactPoints[0], contactPoints[1], p0Out, p1Out);
				context.DrawPoint(p0Out, ndVector(1.0f, 0.0f, 0.0f, 1.0f), 3);
				context.DrawPoint(p1Out, ndVector(0.0f, 1.0f, 0.0f, 1.0f), 3);
			}
		}

		void ApplyPoseGeneration()
		{
			ndVector veloc;
			m_animBlendTree->Update(m_timestep * m_control->m_animSpeed);
			m_animBlendTree->Evaluate(m_animPose, veloc);

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

				ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;
				ndVector posit(m_animPose[i].m_posit);
				effector->SetLocalTargetPosition(posit);
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
			}

			m_invDynamicsSolver.SolverBegin(skeleton, joint, 4, m_world, m_timestep);
			m_invDynamicsSolver.Solve();
			m_invDynamicsSolver.SolverEnd();
		}

		void GetObservation(ndReal* const state)
		{
			for (ndInt32 i = 0; i < m_animPose.GetCount(); ++i)
			{
				const ndAnimKeyframe& keyFrame = m_animPose[i];
				ndEffectorInfo* const info = (ndEffectorInfo*)keyFrame.m_userData;
				ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;

				ndVector posit(effector->GetLocalTargetPosition());
				state[i] = posit.m_y - D_POSE_REST_POSITION_Y;
				//if (i==0) ndTrace(("%d %f %f %f\n", i, posit.m_x, state[i], posit.m_z));
			}

			//ndBodyState bodyState(CalculateFullBodyState());
			//ndVector omega(bodyState.m_com.UnrotateVector(bodyState.m_omega));

			ndBodyKinematic* const rootBody = GetRoot()->m_body->GetAsBodyKinematic();
			const ndMatrix& rootMatrix = rootBody->GetMatrix();
			//ndMatrix matrix(ndGetIdentityMatrix());
			//matrix.m_up = ndVector::m_zero;
			//matrix.m_up.m_y = ndFloat32(1.0f);
			//matrix.m_right = (rootMatrix.m_front.CrossProduct(matrix.m_up)).Normalize();
			//matrix.m_front = matrix.m_up.CrossProduct(matrix.m_right);
			//ndVector omega(matrix.UnrotateVector(rootBody->GetOmega()));
			ndVector omega(rootMatrix.UnrotateVector(rootBody->GetOmega()));

			state[m_body_omega_x] = omega.m_x;
			state[m_body_omega_z] = omega.m_z;

			state[m_body_swing] = m_control->m_z;
		}

		void ApplyActions(ndReal* const actions)
		{
			m_control->m_animSpeed = 0.25f;
			m_control->m_z = ndClamp(m_control->m_z + actions[m_bodySwing] * D_SWING_STEP, -D_MAX_SWING_DIST, D_MAX_SWING_DIST);
			ApplyPoseGeneration();
		}

		bool IsTerminal() const
		{
			ndBodyKinematic* const body = GetRoot()->m_body->GetAsBodyKinematic();
			const ndMatrix& matrix = body->GetMatrix();
			ndFloat32 sinAngle = ndSqrt(matrix.m_up.m_x * matrix.m_up.m_x + matrix.m_up.m_z * matrix.m_up.m_z);
			sinAngle = ndMin(sinAngle, ndFloat32(0.9f));
			bool fail = ndAbs(ndAsin(sinAngle)) > D_MIN_REWARD_ANGLE;
			//ndTrace(("%f\n", ndAsin(sinAngle) * ndRadToDegree));
			return fail;
		}

		ndReal GetReward() const
		{
			if (IsTerminal())
			{
				return ndReal(0.0f);
			}

			ndBodyKinematic* const boxBody = GetRoot()->m_body->GetAsBodyKinematic();
			const ndMatrix& matrix = boxBody->GetMatrix();
			ndFloat32 sinAngle = ndSqrt(matrix.m_up.m_x * matrix.m_up.m_x + matrix.m_up.m_z * matrix.m_up.m_z);
			ndFloat32 reward = ndReal(ndPow(ndEXP, -ndFloat32(10000.0f) * sinAngle * sinAngle));
			return ndReal(reward);
		}

		ndBodyState CalculateFullBodyState() const
		{
			ndBodyState state;
			//const ndMatrix& rootMatrix = GetRoot()->m_body->GetMatrix();
			//state.m_com.m_up = ndVector::m_zero;
			//state.m_com.m_up.m_y = ndFloat32 (1.0f);
			//state.m_com.m_right = (rootMatrix.m_front.CrossProduct(state.m_com.m_up)).Normalize();
			//state.m_com.m_front = state.m_com.m_up.CrossProduct(state.m_com.m_right);
			state.m_com = GetRoot()->m_body->GetMatrix();
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

				ndMatrix bodyInertia(body->CalculateInertiaMatrix());
				ndMatrix covariance(ndCovarianceMatrix(comDist, comDist));

				ndFloat32 mass = body->GetMassMatrix().m_w;
				ndVector linearMomentum(body->GetVelocity().Scale(mass));
				state.m_veloc += linearMomentum;
				state.m_omega += comDist.CrossProduct(linearMomentum);
				state.m_omega += bodyInertia.RotateVector(body->GetOmega());

				ndFloat32 massDist2 = comDist.DotProduct(comDist).GetScalar() * mass;
				
				state.m_inertia[0] += (bodyInertia[0] - covariance[0].Scale(mass));
				state.m_inertia[1] += (bodyInertia[1] - covariance[1].Scale(mass));
				state.m_inertia[2] += (bodyInertia[2] - covariance[2].Scale(mass));
				
				state.m_inertia[0][0] += massDist2;
				state.m_inertia[1][1] += massDist2;
				state.m_inertia[2][2] += massDist2;
				ndAssert(state.m_inertia[0][0] > ndFloat32(0.0f));
				ndAssert(state.m_inertia[1][1] > ndFloat32(0.0f));
				ndAssert(state.m_inertia[2][2] > ndFloat32(0.0f));
			}

			state.m_inertia.m_posit = ndVector::m_wOne;
			ndMatrix invInertia(state.m_inertia.Inverse4x4());
			state.m_omega = invInertia.RotateVector(state.m_omega);
			state.m_veloc = state.m_veloc.Scale(1.0f / state.m_mass);

			return state;
		}

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			m_world = world;
			m_timestep = timestep;
			ndModelArticulation::Update(world, timestep);
			m_agent->Step();
		}

		void PostUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			ndModelArticulation::PostUpdate(world, timestep);
			m_agent->OptimizeStep();
		}

		ndAnimationPose m_animPose;
		ndFixSizeArray<ndEffectorInfo, 4> m_effectorsInfo;
		ndSharedPtr<ndAnimationBlendTreeNode> m_animBlendTree;

		ndUIControlNode* m_control;
		ndAnimationSequencePlayer* m_poseGenerator;

		ndWorld* m_world;
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
			m_scene->Print(color, "Control panel");
		
			ndModelQuadruped* const model = (ndModelQuadruped*)*m_model;
			ndModelQuadruped::ndUIControlNode* const control = model->m_control;
			
			bool change = false;
			ImGui::Text("position x");
			change = change || ImGui::SliderFloat("##x", &control->m_x, -0.1f, 0.1f);
			ImGui::Text("position y");
			change = change || ImGui::SliderFloat("##y", &control->m_y, -0.2f, 0.1f);
			ImGui::Text("position z");
			change = change || ImGui::SliderFloat("##z", &control->m_z, -D_MAX_SWING_DIST, D_MAX_SWING_DIST);

			ImGui::Text("pitch");
			change = change || ImGui::SliderFloat("##pitch", &control->m_pitch, -15.0f, 15.0f);
			ImGui::Text("yaw");
			change = change || ImGui::SliderFloat("##yaw", &control->m_yaw, -20.0f, 20.0f);
			ImGui::Text("roll");
			change = change || ImGui::SliderFloat("##roll", &control->m_roll, -15.0f, 15.0f);

			ImGui::Text("animSpeed");
			change = change || ImGui::SliderFloat("##animSpeed", &control->m_animSpeed, 0.0f, 1.0f);

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
			ndInt32 layerSize = 64;
			ndSharedPtr<ndBrain> actor(new ndBrain());
			ndBrainLayer* const layer0 = new ndBrainLayer(m_stateSize, layerSize, m_tanh);
			ndBrainLayer* const layer1 = new ndBrainLayer(layer0->GetOuputSize(), layerSize, m_tanh);
			ndBrainLayer* const layer2 = new ndBrainLayer(layer1->GetOuputSize(), layerSize, m_tanh);
			ndBrainLayer* const ouputLayer = new ndBrainLayer(layer2->GetOuputSize(), m_actionsSize, m_tanh);
			actor->BeginAddLayer();
			actor->AddLayer(layer0);
			actor->AddLayer(layer1);
			actor->AddLayer(layer2);
			actor->AddLayer(ouputLayer);
			actor->EndAddLayer();
			actor->InitGaussianWeights(ndReal(0.25f));

			// the critic is more complex since is deal with more complex inputs
			ndSharedPtr<ndBrain> critic(new ndBrain());
			ndBrainLayer* const criticLayer0 = new ndBrainLayer(m_stateSize + m_actionsSize, layerSize * 2, m_tanh);
			ndBrainLayer* const criticLayer1 = new ndBrainLayer(criticLayer0->GetOuputSize(), layerSize * 2, m_tanh);
			ndBrainLayer* const criticLayer2 = new ndBrainLayer(criticLayer1->GetOuputSize(), layerSize * 2, m_tanh);
			ndBrainLayer* const criticOuputLayer = new ndBrainLayer(criticLayer2->GetOuputSize(), 1, m_lineal);
			critic->BeginAddLayer();
			critic->AddLayer(criticLayer0);
			critic->AddLayer(criticLayer1);
			critic->AddLayer(criticLayer2);
			critic->AddLayer(criticOuputLayer);
			critic->EndAddLayer();
			critic->InitGaussianWeights(ndReal(0.25f));

			// add a reinforcement learning controller 
			ndSharedPtr<ndBrainAgent> agent(new ndModelQuadruped::ndControllerAgent_trainer(actor, critic));
			agent->SetName("quadruped_1.nn");
		#else
			char fileName[1024];
			ndGetWorkingFileName("quadruped_1.nn", fileName);
			ndSharedPtr<ndBrain> actor(ndBrainLoad::Load(fileName));
			ndSharedPtr<ndBrainAgent> agent(new ndModelQuadruped::ndControllerAgent(actor));
		#endif
		return agent;
	}

	ndModelArticulation* BuildModel(ndDemoEntityManager* const scene, const ndMatrix& matrixLocation)
	{
		ndFloat32 mass = 10.0f;
		ndFloat32 radius = 0.25f;
		ndFloat32 limbMass = 0.5f;
		ndFloat32 limbLength = 0.3f;
		ndFloat32 limbRadios = 0.06f;

		ndSharedPtr<ndBrainAgent> agent (BuildAgent());
		ndModelQuadruped* const model = new ndModelQuadruped(agent);

		ndPhysicsWorld* const world = scene->GetWorld();
		ndVector floor(FindFloor(*world, matrixLocation.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		ndSharedPtr<ndBody> torso (world->GetBody(AddSphere(scene, matrixLocation, mass, radius, "smilli.tga")));
		ndModelArticulation::ndNode* const modelRoot = model->AddRootBody(torso);
		
		ndMatrix location(matrixLocation);
		location.m_posit.m_y = floor.m_y + 0.5f;
		torso->SetMatrix(location);
		
		ndDemoEntity* const entity = (ndDemoEntity*)torso->GetNotifyCallback()->GetUserData();
		entity->SetMeshMatrix(ndYawMatrix(90.0f * ndDegreeToRad) * ndPitchMatrix(90.0f * ndDegreeToRad));
		
		ndMatrix matrix(ndRollMatrix(45.0f * ndDegreeToRad));
		matrix.m_posit.m_x = radius * 0.9f;
		matrix.m_posit.m_y = -radius * 0.5f;
		
		ndFloat32 angles[] = { 300.0f, 240.0f, 120.0f, 60.0f };
		ndFloat32 offset[] = { -0.3f, 0.3f, -0.3f, 0.3f };
		//ndFloat32 phase[] = { 0.0f, 0.75f, 0.25f, 0.5f };
		ndFloat32 phase[] = { 0.0f, 0.5f, 0.0f, 0.5f };

		ndSharedPtr<ndAnimationSequence> sequence(new ndModelQuadruped::ndPoseGenerator());
		model->m_poseGenerator = new ndAnimationSequencePlayer(sequence);
		model->m_control = new ndModelQuadruped::ndUIControlNode(model->m_poseGenerator);
		model->m_animBlendTree = ndSharedPtr<ndAnimationBlendTreeNode>(model->m_control);

		ndModelQuadruped::ndPoseGenerator* const poseGenerator = (ndModelQuadruped::ndPoseGenerator*)*sequence;
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

				ndModelQuadruped::ndEffectorInfo info(ndSharedPtr<ndJointBilateralConstraint> (effector), footHinge);
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
			((ndModelQuadruped::ndControllerAgent_trainer*)*agent)->SetModel(model);
			scene->SetAcceleratedUpdate();
		#else
			((ndModelQuadruped::ndControllerAgent*)*agent)->SetModel(model);
		#endif
		return model;
	}
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
