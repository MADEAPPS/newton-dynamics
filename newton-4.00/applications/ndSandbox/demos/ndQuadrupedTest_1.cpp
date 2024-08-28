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
		ndBrainFloat m_torso_x;
		ndBrainFloat m_torso_z;
	};

	class ndObservationVector
	{
		public:
		ndLegObservation n_legs[4];
		ndBrainFloat m_torso_x;
		ndBrainFloat m_torso_z;
		ndBrainFloat m_animSpeed;
	};

	#define ND_AGENT_OUTPUT_SIZE	(sizeof (ndActionVector) / sizeof (ndBrainFloat))
	#define ND_AGENT_INPUT_SIZE		(sizeof (ndObservationVector) / sizeof (ndBrainFloat))

	#define D_MAX_SWING_DIST_X		ndReal(0.10f)
	#define D_MAX_SWING_DIST_Z		ndReal(0.15f)
	#define D_POSE_REST_POSITION_Y	ndReal(-0.3f)

	//#define D_SWING_STEP			ndReal(0.01f)
	#define D_SWING_STEP			ndReal(0.005f)
	#define D_MODEL_DEAD_ANGLE		ndReal(0.2f)
	#define D_MIN_TRAIN_ANIM_SPEED	ndReal(0.1f)

	void ExportUrdfModel(ndDemoEntityManager* const scene)
	{
		ndFloat32 mass = 20.0f;
		ndFloat32 radius = 0.25f;
		ndFloat32 limbMass = 0.25f;
		ndFloat32 limbLength = 0.3f;
		ndFloat32 limbRadios = 0.05f;

		ndSharedPtr<ndModelArticulation> model(new ndModelArticulation);

		ndBodyKinematic* const torso = CreateSphere(scene, ndGetIdentityMatrix(), mass, radius, "smilli.png");
		ndModelArticulation::ndNode* const modelRoot = model->AddRootBody(torso);
		torso->SetMatrix(ndGetIdentityMatrix());

		ndMatrix matrix(ndRollMatrix(45.0f * ndDegreeToRad));
		matrix.m_posit.m_x = radius * 0.9f;
		matrix.m_posit.m_y = -radius * 0.5f;

		ndFloat32 angles[] = { 300.0f, 240.0f, 120.0f, 60.0f };

		//for (ndInt32 i = 0; i < 1; ++i)
		for (ndInt32 i = 0; i < 4; ++i)
		{
			ndMatrix limbPivotLocation(matrix * ndYawMatrix(angles[i] * ndDegreeToRad));

			ndModelArticulation::ndNode* thighNode = nullptr;
			{
				ndMatrix bodyMatrix(limbPivotLocation);
				bodyMatrix.m_posit += limbPivotLocation.m_front.Scale(limbLength * 0.5f);

				ndBodyKinematic* const thigh = CreateCapsule(scene, bodyMatrix, limbMass, limbRadios, limbRadios, limbLength);
				thigh->SetMatrix(bodyMatrix);
				ndJointBilateralConstraint* const ballJoint = new ndIkJointSpherical(limbPivotLocation, thigh, torso);
				thighNode = model->AddLimb(modelRoot, thigh, ballJoint);
				thighNode->m_name = ndString ("thigh_") + i;

				limbPivotLocation.m_posit += limbPivotLocation.m_front.Scale(limbLength);
			}

			// add calf0
			ndModelArticulation::ndNode* calfNode = nullptr;
			{
				limbPivotLocation = ndRollMatrix(-90.0f * ndDegreeToRad) * limbPivotLocation;
			
				ndMatrix bodyMatrix(limbPivotLocation);
				bodyMatrix.m_posit += limbPivotLocation.m_front.Scale(limbLength * 0.5f);
				ndBodyKinematic* const calf0 = CreateCapsule(scene, bodyMatrix, limbMass * 0.25f, limbRadios, limbRadios, limbLength);
				calf0->SetMatrix(bodyMatrix);
			
				ndMatrix caffPinAndPivotFrame(ndGetIdentityMatrix());
				ndFloat32 sign = angles[i] > 180.0f ? -1.0f : 1.0f;
				caffPinAndPivotFrame.m_front = limbPivotLocation.m_right.Scale(sign);
				caffPinAndPivotFrame.m_up = limbPivotLocation.m_front;
				caffPinAndPivotFrame.m_right = caffPinAndPivotFrame.m_front.CrossProduct(caffPinAndPivotFrame.m_up);
				caffPinAndPivotFrame.m_posit = limbPivotLocation.m_posit;
				ndIkJointHinge* const hinge = new ndIkJointHinge(caffPinAndPivotFrame, calf0->GetAsBodyKinematic(), thighNode->m_body->GetAsBodyKinematic());
			
				hinge->SetLimitState(true);
				//hinge->SetLimits(-60.0f * ndDegreeToRad, 60.0f * ndDegreeToRad);
				hinge->SetLimits(-70.0f * ndDegreeToRad, 70.0f * ndDegreeToRad);
				calfNode = model->AddLimb(thighNode, calf0, hinge);
				calfNode->m_name = ndString("calf_") + i;
			
				limbPivotLocation.m_posit += limbPivotLocation.m_front.Scale(limbLength);
			}
			
			// add calf1
			{
				ndFloat32 lenght = limbLength * 0.5f;
				limbPivotLocation = ndRollMatrix(-45.0f * ndDegreeToRad) * limbPivotLocation;
				ndMatrix bodyMatrix(limbPivotLocation);
				bodyMatrix.m_posit += limbPivotLocation.m_front.Scale(lenght * 0.5f);
			
				ndBodyKinematic* const foot = CreateCapsule(scene, bodyMatrix, limbMass * 0.25f, limbRadios, limbRadios, lenght);
				foot->SetMatrix(bodyMatrix);
			
				ndMatrix footPinAndPivotFrame(ndGetIdentityMatrix());
				footPinAndPivotFrame.m_front = limbPivotLocation.m_right;
				footPinAndPivotFrame.m_up = limbPivotLocation.m_front.Scale(-1.0f);
				footPinAndPivotFrame.m_right = footPinAndPivotFrame.m_front.CrossProduct(footPinAndPivotFrame.m_up);
				footPinAndPivotFrame.m_posit = limbPivotLocation.m_posit;
			
				// add joint limit to prevent knee from flipping
				ndJointHinge* const footHinge = new ndJointHinge(footPinAndPivotFrame, foot->GetAsBodyKinematic(), calfNode->m_body->GetAsBodyKinematic());
				footHinge->SetAsSpringDamper(0.001f, 2000.0f, 50.0f);
				ndModelArticulation::ndNode* const footNode = model->AddLimb(calfNode, foot, footHinge);

				footNode->m_name = ndString("foot_") + i;
			}
		}

		ndUrdfFile urdf;
		char fileName[256];
		ndGetWorkingFileName("quadSpider.urdf", fileName);
		urdf.Export(fileName, *model);
	}

	ndModelArticulation* CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location)
	{
		ndUrdfFile urdf;
		char fileName[256];
		ndGetWorkingFileName("quadSpider.urdf", fileName);
		ndModelArticulation* const model = urdf.Import(fileName);

		SetModelVisualMesh(scene, model);
		ndMatrix matrix(model->GetRoot()->m_body->GetMatrix() * location);
		matrix.m_posit = location.m_posit;
		model->SetTransform(matrix);

		ndFloat32 totalVolume = 0.0f;
		ndFixSizeArray<ndBodyKinematic*, 256> bodyArray;
		for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
		{
			ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
			ndFloat32 volume = body->GetCollisionShape().GetVolume();
			if (node->m_name != "base_link")
			{
				volume *= 4.0f;
			}
			totalVolume += volume;
			bodyArray.PushBack(body);
		}

		ndFloat32 totalMass = 25.0f;
		ndFloat32 density = totalMass / totalVolume;
		for (ndInt32 i = 0; i < bodyArray.GetCount(); ++i)
		{
			ndBodyKinematic* const body = bodyArray[i];
			ndFloat32 volume = body->GetCollisionShape().GetVolume();
			ndFloat32 mass = density * volume;
			body->SetMassMatrix(mass, body->GetCollisionShape());
			ndVector inertia(body->GetMassMatrix());
			ndFloat32 maxInertia = ndMax(ndMax(inertia.m_x, inertia.m_y), inertia.m_z);
			ndFloat32 minInertia = ndMin(ndMin(inertia.m_x, inertia.m_y), inertia.m_z);
			if (minInertia < maxInertia * 0.125f)
			{
				minInertia = maxInertia * 0.125f;
				for (ndInt32 j = 0; j < 3; ++j)
				{
					if (inertia[j] < minInertia)
					{
						inertia[j] = minInertia;
					}
				}
			}
			body->SetMassMatrix(inertia);
		}

		return model;
	}

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
				:ndBrainAgentContinuePolicyGradient(src.m_actor)
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

			void SaveTrajectory()
			{
				//ndInt32 stepsCount = 0;
				//// if the model is dead, just skip this trajectory, not need to train on a dead model.
				//for (ndInt32 i = 0; i < m_trajectory.GetCount(); ++i)
				//{
				//	if (m_trajectory.GetReward(i) > ndReal(0.05f))
				//	{
				//		// model is alive, break loop.
				//		stepsCount = m_trajectory.GetCount();
				//		break;
				//	}
				//}
				//m_trajectory.SetCount(stepsCount);
				ndBrainAgentContinuePolicyGradient_Trainer::SaveTrajectory();
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

		//RobotModelNotify(const RobotModelNotify& src)
		//	:ndModelNotify(src)
		//	,m_controller(src.m_controller)
		//{
		//	//Init(robot);
		//	ndAssert(0);
		//}

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
				ndModelArticulation::ndNode* const footNode = robot->FindByName(effectorNames[i]);
				if (footNode)
				{
					ndModelArticulation::ndNode* const calfNode = footNode->GetParent();
					ndModelArticulation::ndNode* const thighNode = calfNode->GetParent();

					ndBodyKinematic* const footBody = footNode->m_body->GetAsBodyKinematic();

					const ndShapeInstance& footCollision = footBody->GetCollisionShape();
					ndMatrix effectorMatrix(footCollision.GetLocalMatrix() * footBody->GetMatrix());

					ndShapeInfo shapeInfo (footCollision.GetShapeInfo());
					effectorMatrix.m_posit += effectorMatrix.m_front.Scale(shapeInfo.m_capsule.m_height * 0.5f);

					ndMatrix effectorRefFrame(ndYawMatrix(angles[i] * ndDegreeToRad));
					effectorRefFrame.m_posit = rootMatrix.TransformVector(thighNode->m_joint->GetLocalMatrix1().m_posit);

					ndFloat32 regularizer = 0.001f;
					ndIkSwivelPositionEffector* const effector = new ndIkSwivelPositionEffector(effectorMatrix.m_posit, effectorRefFrame, footBody, robot->GetRoot()->m_body->GetAsBodyKinematic());
					effector->SetLinearSpringDamper(regularizer, 4000.0f, 50.0f);
					effector->SetAngularSpringDamper(regularizer, 4000.0f, 50.0f);
					//effector->SetWorkSpaceConstraints(0.0f, workSpace * 0.9f);
					effector->SetWorkSpaceConstraints(0.0f, 0.75f * 0.9f);
					effector->SetMaxForce(effectorStrength);
					effector->SetMaxTorque(effectorStrength);

					ndEffectorInfo info(thighNode->m_joint, calfNode->m_joint, footNode->m_joint, ndSharedPtr<ndJointBilateralConstraint>(effector));
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

		//#pragma optimize( "", off )
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
				ndAssert(0);
			}

			return reward;
		}

		//#pragma optimize( "", off )
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

		//#pragma optimize( "", off )
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

		void GetObservation(ndBrainFloat* const inputObservations)
		{
			ndMemSet(inputObservations, 0.0f, ND_AGENT_INPUT_SIZE);
			ndObservationVector& observation = *((ndObservationVector*)inputObservations);
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
				//observation.n_legs[i].m_animSequence = ndBrainFloat(keyFrame.m_userParamInt ? 1.0f : 0.0f);
				observation.n_legs[i].m_animSequence = ndBrainFloat(keyFrame.m_userParamFloat);
			}

			observation.m_torso_x = m_control->m_x;
			observation.m_torso_z = m_control->m_z;
			observation.m_animSpeed = m_control->m_animSpeed;
		}

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

		void Update(ndWorld* const world, ndFloat32 timestep)
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

		void PostUpdate(ndWorld* const, ndFloat32 timestep)
		{
			//ndFloat32 animSpeed = (m_control->m_animSpeed > 0.01f) ? (1.0f + 1.0f * m_control->m_animSpeed) : 0.0f;
			ndFloat32 animSpeed = 2.0f * m_control->m_animSpeed;
			m_animBlendTree->Update(timestep * animSpeed);
		}

		void PostTransformUpdate(ndWorld* const, ndFloat32)
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
		TrainingUpdata(ndDemoEntityManager* const scene, const ndMatrix& matrix)
			:OnPostUpdate()
			,m_master()
			,m_bestActor()
			,m_outFile(nullptr)
			,m_timer(ndGetTimeInMicroseconds())
			,m_maxScore(ndFloat32(-1.0e10f))
			,m_discountFactor(0.995f)
			,m_horizon(ndFloat32(1.0f) / (ndFloat32(1.0f) - m_discountFactor))
			,m_lastEpisode(-1)
			,m_stopTraining(500 * 1000000)
			,m_modelIsTrained(false)
		{
			//ndWorld* const world = scene->GetWorld();
			m_outFile = fopen("quadruped_1-vpg.csv", "wb");
			fprintf(m_outFile, "vpg\n");
			
			ndBrainAgentContinuePolicyGradient_TrainerMaster::HyperParameters hyperParameters;
			
			//hyperParameters.m_threadsCount = 1;
			hyperParameters.m_maxTrajectorySteps = 1024 * 8;
			hyperParameters.m_extraTrajectorySteps = 1024 * 2;
			//hyperParameters.m_bashTrajectoryCount = 1000;
			hyperParameters.m_discountFactor = ndReal(m_discountFactor);
			hyperParameters.m_numberOfActions = ND_AGENT_OUTPUT_SIZE;
			hyperParameters.m_numberOfObservations = ND_AGENT_INPUT_SIZE;
			
			m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>(new ndBrainAgentContinuePolicyGradient_TrainerMaster(hyperParameters));
			m_bestActor = ndSharedPtr<ndBrain>(new ndBrain(*m_master->GetActor()));
			m_master->SetName(CONTROLLER_NAME);

			auto SpawnModel = [this, scene](const ndMatrix& matrix, bool debug)
			{
				ndWorld* const world = scene->GetWorld();
				ndModelArticulation* const model = CreateModel(scene, matrix);
				SetMaterial(model);
				model->SetNotifyCallback(new RobotModelNotify(m_master, model, debug));
				model->AddToWorld(world);
				((RobotModelNotify*)*model->GetNotifyCallback())->ResetModel();
				return model;
			};

			ndModelArticulation* const visualModel = SpawnModel(matrix, true);
			ndSharedPtr<ndUIEntity> quadrupedUI(new ndModelUI(scene, (RobotModelNotify*)*visualModel->GetNotifyCallback()));
			scene->Set2DDisplayRenderFunction(quadrupedUI);

			ndInt32 countX = 22;
			ndInt32 countZ = 23;
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

					ndModelArticulation* const model = SpawnModel(location, false);
					m_models.Append(model);
				}
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
				void* const useData = originalNotify->m_entity;
				originalNotify->m_entity = nullptr;
				TrainingRobotBodyNotify* const notify = new TrainingRobotBodyNotify((TrainingRobotBodyNotify*)body->GetNotifyCallback());
				body->SetNotifyCallback(notify);
				notify->m_entity = (ndDemoEntity*)useData;
			
				for (ndModelArticulation::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
				{
					stack.PushBack(child);
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

			if ((stopTraining >= m_stopTraining) || (100.0f * m_master->GetAverageScore() / m_horizon > 96.0f))
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

		ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster> m_master;
		ndSharedPtr<ndBrain> m_bestActor;
		ndList<ndModelArticulation*> m_models;
		FILE* m_outFile;
		ndUnsigned64 m_timer;
		ndFloat32 m_maxScore;
		ndFloat32 m_discountFactor;
		ndFloat32 m_horizon;
		ndInt32 m_lastEpisode;
		ndInt32 m_stopTraining;
		bool m_modelIsTrained;
	};
}

using namespace ndQuadruped_1;

void ndQuadrupedTest_1(ndDemoEntityManager* const scene)
{
	// build a floor
	ndSetRandSeed(94157);

	BuildFloorBox(scene, ndGetIdentityMatrix());
	//BuildFlatPlane(scene, true);

//	ExportUrdfModel(scene);

	// register a zero restitution and high friction material for the feet
	ndApplicationMaterial material;
	material.m_restitution = 0.0f;
	material.m_staticFriction0 = 0.8f;
	material.m_staticFriction1 = 0.8f;
	material.m_dynamicFriction0 = 0.8f;
	material.m_dynamicFriction1 = 0.8f;
	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
	callback->RegisterMaterial(material, ndDemoContactCallback::m_frictionTest, ndDemoContactCallback::m_default);

	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit.m_y = 0.6f;

	#ifdef ND_TRAIN_MODEL
		scene->RegisterPostUpdate(new TrainingUpdata(scene, matrix));
	#else
		ndWorld* const world = scene->GetWorld();
		ndModelArticulation* const referenceModel = CreateModel(scene, matrix);
		referenceModel->AddToWorld(world);

		ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(referenceModel->GetAsModelArticulation()->GetRoot()->m_body->GetMatrix(), referenceModel->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic(), world->GetSentinelBody()));
		//world->AddJoint(fixJoint);

		char fileName[256];
		ndGetWorkingFileName(CONTROLLER_NAME, fileName);
		ndSharedPtr<ndBrain> brain(ndBrainLoad::Load(fileName));
		referenceModel->SetNotifyCallback(new RobotModelNotify(brain, referenceModel, true));

		ndSharedPtr<ndUIEntity> quadrupedUI(new ndModelUI(scene, (RobotModelNotify*)*referenceModel->GetNotifyCallback()));
		scene->Set2DDisplayRenderFunction(quadrupedUI);

		matrix.m_posit.m_z += 1.5f;

		ndInt32 countZ = 5;
		ndInt32 countX = 5;

		//countZ = 0;
		//countX = 0;
		for (ndInt32 i = 0; i < countZ; ++i)
		{
			for (ndInt32 j = 0; j < countX; ++j)
			{
				ndMatrix location(matrix);
				location.m_posit.m_x += 3.0f * ndFloat32 (j - countX/2);
				location.m_posit.m_z += 3.0f * ndFloat32 (i - countZ/2);
				ndModelArticulation* const model = CreateModel(scene, location);
				model->SetNotifyCallback(new RobotModelNotify(brain, model, false));
				model->AddToWorld(world);
				//m_models.Append(model);
				//SetMaterial(model);
			}
		}

	#endif
	
	matrix.m_posit.m_x -= 8.0f;
	matrix.m_posit.m_y += 1.5f;
	matrix.m_posit.m_z += 0.25f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
