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
#include "ndDemoMesh.h"
#include "ndUIEntity.h"
#include "ndMeshLoader.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndContactCallback.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

namespace ndQuadruped_3
{
	#define ND_TRAIN_MODEL
	#define CONTROLLER_NAME "ndSpot-VPG.dnn"

	class ndLegObservation
	{
		public:
		ndBrainFloat m_posit_x;
		ndBrainFloat m_posit_y;
		ndBrainFloat m_posit_z;
		ndBrainFloat m_veloc_x;
		ndBrainFloat m_veloc_y;
		ndBrainFloat m_veloc_z;
		ndBrainFloat m_isLegLifted;
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
		ndFloat32 m_animTime;
	};
	#define ND_AGENT_OUTPUT_SIZE	(sizeof (ndActionVector) / sizeof (ndBrainFloat))
	#define ND_AGENT_INPUT_SIZE		(sizeof (ndObservationVector) / sizeof (ndBrainFloat))

	#define D_MAX_SWING_DIST_X		ndReal(0.40f)
	#define D_MAX_SWING_DIST_Z		ndReal(0.17f)
	#define D_POSE_REST_POSITION_Y	ndReal (-0.3f)

	//#define D_SWING_STEP			ndReal(0.01f)
	#define D_SWING_STEP			ndReal(0.005f)

	class ndDefinition
	{
		public:
		enum ndJointType
		{
			m_root,
			m_hinge,
			m_effector
		};

		char m_boneName[32];
		ndJointType m_type;
		ndFloat32 m_minAngle;
		ndFloat32 m_maxAngle;
		ndFloat32 m_walkPhase;
		ndFloat32 m_weightDistribution;
	};

	static ndDefinition jointsDefinition[] =
	{
		{ "spot_body", ndDefinition::m_root, 0.0f, 0.0f, 0.0f, 1.25f},

		{ "spot_shoulder_FR", ndDefinition::m_hinge, -90.0f, 90.0f, 0.0f, 4.0f },
		{ "spot_up_arm_FR", ndDefinition::m_hinge, -130.0f, 130.0f, 0.0f, 0.8f  },
		{ "spot_arm_FR", ndDefinition::m_hinge, -90.0f, 45.0f, 0.0f, 1.0f  },
		{ "spot_arm_FR_effector", ndDefinition::m_effector, 0.0f, 0.0f, 0.0f, 0.0f  },
		
		{ "spot_shoulder_FL", ndDefinition::m_hinge, -90.0f, 90.0f, 0.0f, 4.0f},
		{ "spot_up_arm_FL", ndDefinition::m_hinge, -130.0f, 130.0f, 0.0f, 0.8f},
		{ "spot_arm_FL", ndDefinition::m_hinge, -90.0f, 45.0f, 0.0f, 1.0f},
		{ "spot_arm_FL_effector", ndDefinition::m_effector, 0.0f, 0.0f, 0.0f, 0.0f },
		
		{ "spot_shoulder_BR", ndDefinition::m_hinge, -90.0f, 90.0f, 0.0f, 4.0f},
		{ "spot_up_arm_BR", ndDefinition::m_hinge, -130.0f, 130.0f, 0.0f, 0.8f},
		{ "spot_arm_BR", ndDefinition::m_hinge, -90.0f, 45.0f, 0.0f, 1.0f },
		{ "spot_arm_BR_effector", ndDefinition::m_effector, 0.0f, 0.0f, 0.0f, 0.0f },
		
		{ "spot_shoulder_BL", ndDefinition::m_hinge, -90.0f, 90.0f, 0.0f, 4.0f},
		{ "spot_up_arm_BL", ndDefinition::m_hinge, -130.0f, 130.0f, 0.0f, 0.8f},
		{ "spot_arm_BL", ndDefinition::m_hinge, -90.0f, 45.0f, 0.0f, 1.0f},
		{ "spot_arm_BL_effector", ndDefinition::m_effector, 0.0f, 0.0f, 0.0f, 0.0f },
	};

	class ndQuadrupedMaterial: public ndApplicationMaterial
	{
		public:
		ndQuadrupedMaterial()
			:ndApplicationMaterial()
		{
		}

		ndQuadrupedMaterial(const ndQuadrupedMaterial& src)
			:ndApplicationMaterial(src)
		{
		}

		ndApplicationMaterial* Clone() const
		{
			return new ndQuadrupedMaterial(*this);
		}

		bool OnAabbOverlap(const ndContact* const, ndFloat32, const ndShapeInstance& instanceShape0, const ndShapeInstance& instanceShape1) const
		{
			// filter self collision when the contact is with in the same model
			const ndShapeMaterial& material0 = instanceShape0.GetMaterial();
			const ndShapeMaterial& material1 = instanceShape1.GetMaterial();

			ndUnsigned64 pointer0 = material0.m_userParam[ndDemoContactCallback::m_modelPointer].m_intData;
			ndUnsigned64 pointer1 = material1.m_userParam[ndDemoContactCallback::m_modelPointer].m_intData;
			if (pointer0 == pointer1)
			{
				// here we know the part are from the same model.
				// we can apply some more filtering by for now we just disable all self model collisions. 
				return false;
			}
			return true;
		}
	};

	class ndRobot : public ndModelArticulation
	{
		public:
		class ndEffectorInfo
		{
			public:
			ndEffectorInfo()
				:m_basePosition(ndVector::m_wOne)
				,m_effector(nullptr)
			{
			}

			ndEffectorInfo(const ndSharedPtr<ndJointBilateralConstraint>& effector)
				:m_basePosition(((ndIkSwivelPositionEffector*)*effector)->GetLocalTargetPosition())
				,m_effector(effector)
			{
			}

			ndVector m_basePosition;
			ndSharedPtr<ndJointBilateralConstraint> m_effector;
		};

		class ndPoseGenerator : public ndAnimationSequence
		{
			public:
			ndVector m_offset[4];
			mutable ndVector m_currentPose[4];
			ndFloat32 m_amp;
			ndFloat32 m_gaitFraction;
			ndFloat32 m_phase[4];
		};

		class ndCrawlPoseGenerator : public ndPoseGenerator
		{
			public:
			ndCrawlPoseGenerator(ndFloat32 gaitFraction, const ndFloat32* const phase)
				:ndPoseGenerator()
			{
				m_amp = 0.4f;
				m_gaitFraction = gaitFraction;
				m_duration = ndFloat32(4.0f);
				for (ndInt32 i = 0; i < 4; i++)
				{
					m_phase[i] = phase[i];
					m_offset[i] = ndVector::m_zero;
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

				ndFloat32 gaitGuard = m_gaitFraction * 0.05f;
				ndFloat32 omega = ndPi / (m_gaitFraction - ndFloat32(2.0f) * gaitGuard);

				ndFloat32 ycontact = D_POSE_REST_POSITION_Y + m_amp / 2.0f;
				for (ndInt32 i = 0; i < output.GetCount(); i++)
				{
					output[i].m_userParamInt = 0;
					output[i].m_posit = m_offset[i];
					ndFloat32 t = ndMod(param - m_phase[i] + ndFloat32(1.0f), ndFloat32(1.0f));
					if (t <= m_gaitFraction)
					{
						if ((t >= gaitGuard) && (t <= (m_gaitFraction - gaitGuard)))
						{
							output[i].m_posit.m_x -= m_amp * ndSin(omega * (t - gaitGuard));
							output[i].m_userParamInt = output[i].m_posit.m_y < ycontact ? -1 : 1;
						}
					}

					m_currentPose[i] = output[i].m_posit;
				}
			}
		};

		class ndWalkPoseGenerator : public ndPoseGenerator
		{
			public:
			ndWalkPoseGenerator(ndFloat32 gaitFraction, const ndFloat32* const phase)
				:ndPoseGenerator()
			{
				m_amp = 0.4f;
				m_gaitFraction = gaitFraction;
				m_duration = ndFloat32(4.0f);
				for (ndInt32 i = 0; i < 4; i++)
				{
					m_phase[i] = phase[i];
					m_offset[i] = ndVector::m_zero;
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

				ndFloat32 gaitGuard = m_gaitFraction * 0.05f;
				ndFloat32 omega = ndPi / (m_gaitFraction - ndFloat32(2.0f) * gaitGuard);

				ndFloat32 ycontact = D_POSE_REST_POSITION_Y + m_amp / 2.0f;
				for (ndInt32 i = 0; i < output.GetCount(); i++)
				{
					output[i].m_userParamInt = 0;
					output[i].m_posit = m_offset[i];
					ndFloat32 t = ndMod(param - m_phase[i] + ndFloat32(1.0f), ndFloat32(1.0f));
					if (t <= m_gaitFraction)
					{
						if ((t >= gaitGuard) && (t <= (m_gaitFraction - gaitGuard)))
						{
							output[i].m_posit.m_x -= m_amp * ndSin(omega * (t - gaitGuard));
							output[i].m_userParamInt = output[i].m_posit.m_y < ycontact ? -1 : 1;
						}
					}

					m_currentPose[i] = output[i].m_posit;
				}
			}
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

				void SetPose(ndFloat32 x0, ndFloat32 z0) const
				{
					ndMatrix matrix(ndCalculateMatrix(m_rotation, m_posit));
					matrix.m_posit.m_x += x0;
					matrix.m_posit.m_z += z0;

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
				return m_model->CalculateReward();
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
				ndInt32 count = 0;
				ndInt32 isGround = 4;

				bool airborneLeg[4];
				bool sequenceAirborne[4];

				ndMatrix matrix (m_model->GetRoot()->m_body->GetMatrix());
				if (matrix.m_up.m_y < 0.5f)
				{
					return true;
				}

				for (ndInt32 i = 0; i < m_model->m_animPose.GetCount(); ++i)
				{
					ndContact* const contact = m_model->FindContact(i);
					bool isAirborne = !(contact && contact->IsActive());
					isGround -= ndInt32(isAirborne);

					const ndAnimKeyframe& keyFrame = m_model->m_animPose[i];
					if (keyFrame.m_userParamInt != 0)
					{
						airborneLeg[count] = isAirborne;
						if (keyFrame.m_userParamInt == 0)
						{
							isAirborne = false;
						}
						else if (keyFrame.m_userParamInt == 1)
						{
							isAirborne = true;
						}
						sequenceAirborne[count] = isAirborne;

						count++;
					}
				}
				if (isGround < 2)
				{
					return false;
				}

				return false;
			}

			#pragma optimize( "", off )
			void ResetModel()
			{
				m_model->m_control->Reset();

				//ndFloat32 x0 = 10.0f * (ndRand() - 0.5f);
				//ndFloat32 z0 = 10.0f * (ndRand() - 0.5f);
				ndFloat32 x0 = 0.0f;
				ndFloat32 z0 = 0.0f;
				for (ndInt32 i = 0; i < m_basePose.GetCount(); i++)
				{
					m_basePose[i].SetPose(x0, z0);
				}
				ndFloat32 duration = m_model->m_poseGenerator->GetSequence()->GetDuration();
				ndUnsigned32 randomeStart = ndRandInt() % 2;
				//ndUnsigned32 index = randomeStart * 2 + 1;
				ndUnsigned32 index = ndUnsigned32(randomeStart ? 1 : 3);
				m_model->m_animBlendTree->SetTime(ndFloat32(index) * duration * 0.25f);
				//m_model->m_animBlendTree->SetTime(0.0f);
				
				ndFloat32 randVar = ndRand();
				randVar = randVar * randVar * randVar;
				ndFloat32 speed0 = ndFloat32(0.125f);
				ndFloat32 speed1 = ndFloat32(1.0f);
				ndFloat32 animationSpeed = speed0 + (speed1 - speed0) * randVar;
				m_model->m_control->m_animSpeed = animationSpeed;
			}

			ndFixSizeArray<ndBasePose, 32> m_basePose;
			ndRobot* m_model;
		};

		D_CLASS_REFLECTION(ndQuadruped_3::ndRobot, ndModel)

		class RobotBodyNotify : public ndDemoEntityNotify
		{
			public:
			RobotBodyNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity, ndBodyKinematic* const parentBody)
				:ndDemoEntityNotify(manager, entity, parentBody)
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

		ndRobot(ndSharedPtr<ndBrainAgent>& agent)
			:ndModelArticulation()
			,m_effectorsInfo()
			,m_effectorsJoints()
			,m_animPose()
			,m_control(nullptr)
			,m_poseGenerator(nullptr)
			,m_animBlendTree()
			,m_agent(agent)
			,m_timestep(0.0f)
			,m_showDebug(false)
		{
		}

		~ndRobot()
		{
		}

		void NormalizeMassDistribution(ndFloat32 totalMass, ndInt32 bodyCount, ndBodyKinematic** const bodyArray, ndFloat32* const weightFactor) const
		{
			ndFloat32 totalVolume = 0.0f;
			for (ndInt32 i = 0; i < bodyCount; ++i)
			{
				ndBodyKinematic* const body = bodyArray[i];
				ndFloat32 volume = body->GetCollisionShape().GetVolume() * weightFactor[i];
				totalVolume += volume;
			}
			
			ndFloat32 density = totalMass / totalVolume;
			for (ndInt32 i = 0; i < bodyCount; ++i)
			{
				ndBodyKinematic* const body = bodyArray[i];
				ndFloat32 volume = body->GetCollisionShape().GetVolume() * weightFactor[i];
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
				//ndTrace(("%f ", mass));
			}
			//ndTrace(("\n"));
		}

		ndSharedPtr<ndBody> CreateBodyPart(ndDemoEntityManager* const scene, ndDemoEntity* const entityPart, ndFloat32 mass, ndBodyDynamic* const parentBone)
		{
			ndSharedPtr<ndShapeInstance> shapePtr(entityPart->CreateCollisionFromChildren());
			ndShapeInstance* const shape = *shapePtr;
			ndAssert(shape);

			// create the rigid body that will make this body
			ndMatrix matrix(entityPart->CalculateGlobalMatrix());

			ndBodyKinematic* const body = new ndBodyDynamic();
			body->SetMatrix(matrix);
			body->SetCollisionShape(*shape);
			body->SetMassMatrix(mass, *shape);
			//body->SetNotifyCallback(new ndDemoEntityNotify(scene, entityPart, parentBone));
			body->SetNotifyCallback(new RobotBodyNotify(scene, entityPart, parentBone));

			ndShapeInstance& instanceShape = body->GetCollisionShape();
			instanceShape.m_shapeMaterial.m_userId = ndDemoContactCallback::m_modelPart;
			instanceShape.m_shapeMaterial.m_userParam[ndDemoContactCallback::m_modelPointer].m_ptrData = this;

			// add body to the world
			ndSharedPtr<ndBody> bodyPtr(body);
			scene->GetWorld()->AddBody(bodyPtr);
			return bodyPtr;
		}

		void Debug(ndConstraintDebugCallback& context) const
		{
			if (!m_showDebug)
			{
				return;
			}
			ndMatrix comMatrix(GetRoot()->m_body->GetMatrix());
			comMatrix.m_posit = CalculateCenterOfMass();
			context.DrawFrame(comMatrix);

			ndVector zeroMomentPoint(CalculateZeroMomentPoint());
			zeroMomentPoint.m_y = comMatrix.m_posit.m_y;
			
			ndFixSizeArray<ndVector, 16> contactPoints;
			for (ndInt32 i = 0; i < m_effectorsInfo.GetCount(); ++i)
			{
				const ndEffectorInfo& info = m_effectorsInfo[i];
				const ndAnimKeyframe& keyFrame = m_animPose[i];
				if (keyFrame.m_userParamInt == 0)
				{
					const ndJointBilateralConstraint* const joint = *info.m_effector;
					ndBodyKinematic* const body = joint->GetBody0();
					contactPoints.PushBack(body->GetMatrix().TransformVector(info.m_effector->GetLocalMatrix0().m_posit));
				}
			}
			
			const ndVector supportPolygonColor(1.0f, 1.0f, 0.0f, 1.0f);
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
					context.DrawLine(contactPoints[i], p0, supportPolygonColor);
					p0 = contactPoints[i];
				}
			
				ndBigVector p0Out;
				ndBigVector p1Out;
				//ndBigVector ray_p0(comMatrix.m_posit);
				//ndBigVector ray_p1(comMatrix.m_posit);
				ndBigVector ray_p0(zeroMomentPoint);
				ndBigVector ray_p1(zeroMomentPoint);
				ray_p1.m_y -= 1.2f;
				
				ndRayToPolygonDistance(ray_p0, ray_p1, bigPolygon, supportCount, p0Out, p1Out);
				
				context.DrawPoint(p0Out, ndVector(1.0f, 0.0f, 0.0f, 1.0f), 3);
				context.DrawPoint(p1Out, ndVector(0.0f, 1.0f, 0.0f, 1.0f), 3);
			}
		}

		ndVector CalculateCenterOfMass() const
		{
			ndVector com(ndVector::m_zero);
			ndFloat32 totalMass = ndFloat32(0.0f);
			ndFixSizeArray<const ndBodyKinematic*, 32> bodies;
			for (ndModelArticulation::ndNode* node = GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				const ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();

				const ndMatrix matrix(body->GetMatrix());
				const ndVector bodyCom(matrix.TransformVector(body->GetCentreOfMass()));
				ndFloat32 mass = body->GetMassMatrix().m_w;
				totalMass += mass;
				com += matrix.TransformVector(body->GetCentreOfMass()).Scale(mass);
			}
			com = com.Scale(ndFloat32(1.0f) / totalMass);
			com.m_w = 1.0f;
			return com;
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
			if (ndAbs(force.m_y) > ndFloat32(1.0e-4f))
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

		void GetObservation(ndBrainFloat* const observationInput)
		{
			ndObservationVector& observation = *((ndObservationVector*)observationInput);
			for (ndInt32 i = 0; i < m_animPose.GetCount(); ++i)
			{
				const ndAnimKeyframe& keyFrame = m_animPose[i];
				const ndEffectorInfo* const info = (ndEffectorInfo*)keyFrame.m_userData;
				const ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;

				ndVector effectPositState;
				ndVector effectVelocState;
				effector->GetDynamicState(effectPositState, effectVelocState);

				observation.n_legs[i].m_posit_x = ndBrainFloat(effectPositState.m_x);
				observation.n_legs[i].m_posit_y = ndBrainFloat(effectPositState.m_y);
				observation.n_legs[i].m_posit_z = ndBrainFloat(effectPositState.m_z);
				observation.n_legs[i].m_veloc_x = ndBrainFloat(effectVelocState.m_x);
				observation.n_legs[i].m_veloc_y = ndBrainFloat(effectVelocState.m_y);
				observation.n_legs[i].m_veloc_z = ndBrainFloat(effectVelocState.m_z);
				observation.n_legs[i].m_isLegLifted = FindContact(i) ? ndBrainFloat(0.0f) : ndBrainFloat(1.0f);;
			}

			observation.m_torso.m_x = m_control->m_x;
			observation.m_torso.m_z = m_control->m_z;
			observation.m_animTime = m_poseGenerator->GetTime();
			//ndTrace(("%f\n", observation.m_animTime));
		}

		void ApplyActions(ndBrainFloat* const actions)
		{
			//m_control->m_animSpeed = 4.0f;
			//m_control->m_animSpeed = 2.0f;
			//m_control->m_animSpeed = 1.0f;
			//m_control->m_animSpeed = 0.5f;
			//m_control->m_animSpeed = 0.25f;
			//m_control->m_animSpeed = 0.1f;

			//m_control->m_enableController = 0;

			if (m_control->m_enableController)
			{
				const ndActionVector& actionVector = *((ndActionVector*)actions);
				m_control->m_x = ndClamp(ndReal(m_control->m_x + actionVector.m_x * D_SWING_STEP), -D_MAX_SWING_DIST_X, D_MAX_SWING_DIST_X);
				m_control->m_z = ndClamp(ndReal(m_control->m_z + actionVector.m_z * D_SWING_STEP), -D_MAX_SWING_DIST_Z, D_MAX_SWING_DIST_Z);
			}

			UpdatePose(m_timestep);
		}

		ndBrainFloat CalculateReward()
		{
			#if 1
			ndFixSizeArray<ndBigVector, 4> desiredSupportPoint;
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
				#if 1
					ndRayToPolygonDistance(ray_p0, ray_p1, &desiredSupportPoint[0], desiredSupportPoint.GetCount(), p0Out, p1Out);
				#else
					ndVector center(ndVector::m_zero);
					for (ndInt32 i = desiredSupportPoint.GetCount() - 1; i >= 0; --i)
					{
						center += desiredSupportPoint[i];
					}
					center = center.Scale(1.0f / ndFloat32(desiredSupportPoint.GetCount()));
					p0Out = ndBigVector(center);
					p1Out = ndPointToRayDistance(ndBigVector(p0Out), ray_p0, ray_p1);
				#endif	
				const ndBigVector error((p0Out - p1Out) & ndBigVector::m_triplexMask);
				ndFloat32 dist2 = ndFloat32(error.DotProduct(error).GetScalar());
				reward = ndBrainFloat(ndExp(-ndBrainFloat(1000.0f) * dist2));
				//ndTrace(("d2(% f) r(% f)\n", dist2, reward));
			}
			else
			{
				ndAssert(0);
			}
			#else
			const ndMatrix matrix(GetRoot()->m_body->GetMatrix());
			ndFloat32 upAngle = ndAcos(matrix.m_up.m_y);
			ndFloat32 reward = ndBrainFloat(ndExp(-ndBrainFloat(100.0f) * upAngle * upAngle));

			#endif
			return reward;
		}

		void PostTransformUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			ndModelArticulation::PostTransformUpdate(world, timestep);
		}

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			ndModelArticulation::Update(world, timestep);

			m_timestep = timestep;
			m_agent->Step();
			//UpdatePose(m_timestep);
		}

		void PostUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			m_animBlendTree->Update(timestep * m_control->m_animSpeed);
			ndModelArticulation::PostUpdate(world, timestep);
		}

		ndFixSizeArray<ndEffectorInfo, 4> m_effectorsInfo;
		ndFixSizeArray<ndSharedPtr<ndJointBilateralConstraint>, 8> m_effectorsJoints;

		ndAnimationPose m_animPose;
		ndUIControlNode* m_control;
		ndAnimationSequencePlayer* m_poseGenerator;
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
			//ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			//m_scene->Print(color, "Control panel");
			//
			//bool change = false;
			//ImGui::Text("position x");
			//change = change | ImGui::SliderFloat("##x", &m_model->m_param_x0, -1.0f, 1.0f);
			////ImGui::Text("position y");
			////change = change | ImGui::SliderFloat("##y", &info.m_y, -1.0f, 1.0f);
			////ImGui::Text("position z");
			////change = change | ImGui::SliderFloat("##z", &info.m_z, -1.0f, 1.0f);
			//
			//if (change)
			//{
			//	m_model->m_bodyArray[0]->SetSleepState(false);
			//}

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

	ndModelArticulation* BuildModel(ndDemoEntityManager* const scene, ndSharedPtr<ndDemoEntity> modelMesh, const ndMatrix& matrixLocation, ndSharedPtr<ndBrainAgent> agent)
	{
		ndDemoEntity* const entity = modelMesh->CreateClone();
		scene->AddEntity(entity);
		
		ndRobot* const model = new ndRobot(agent);
		ndDemoEntity* const rootEntity = entity->Find(jointsDefinition[0].m_boneName);
		
		// find the floor location 
		ndMatrix matrix(rootEntity->CalculateGlobalMatrix() * matrixLocation);
		//ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		//matrix.m_posit.m_y = floor.m_y;
		
		matrix.m_posit.m_y += 0.1f;
		rootEntity->ResetMatrix(matrix);
		
		// add the root body
		ndSharedPtr<ndBody> rootBody (model->CreateBodyPart(scene, rootEntity, 1.0f, nullptr));
		
		ndModelArticulation::ndNode* const modelRoot = model->AddRootBody(rootBody);
		ndFixSizeArray<ndDemoEntity*, 32> childEntities;
		ndFixSizeArray<ndModelArticulation::ndNode*, 32> parentBone;

		ndInt32 bodyCount = 0;
		ndFloat32 weightFactor[32];
		ndBodyKinematic* bodyArray[32];

		bodyArray[bodyCount] = rootBody->GetAsBodyKinematic();
		weightFactor[bodyCount] = jointsDefinition[0].m_weightDistribution;
		bodyCount++;

		ndInt32 stack = 0;
		for (ndDemoEntity* child = rootEntity->GetFirstChild(); child; child = child->GetNext())
		{
			childEntities[stack] = child;
			parentBone[stack] = modelRoot;
			stack++;
		}
		
		//ndFloat32 phase[] = { 0.0f, 0.75f, 0.25f, 0.5f }; // crawling foward
		ndFloat32 phase[] = { 0.0f, 0.25f, 0.75f, 0.5f };	// crawling backward
		ndSharedPtr<ndAnimationSequence> sequence(new ndRobot::ndCrawlPoseGenerator(0.24f, phase));

		model->m_poseGenerator = new ndAnimationSequencePlayer(sequence);
		model->m_control = new ndRobot::ndUIControlNode(model->m_poseGenerator);
		model->m_animBlendTree = ndSharedPtr<ndAnimationBlendTreeNode>(model->m_control);

		ndRobot::ndPoseGenerator* const poseGenerator = (ndRobot::ndPoseGenerator*)*sequence;

		ndInt32 effectorsCount = 0;
		const ndInt32 definitionCount = ndInt32(sizeof(jointsDefinition) / sizeof(jointsDefinition[0]));
		while (stack)
		{
			stack--;
			ndDemoEntity* const childEntity = childEntities[stack];
			ndModelArticulation::ndNode* parentNode = parentBone[stack];
		
			const char* const name = childEntity->GetName().GetStr();
			for (ndInt32 i = 0; i < definitionCount; ++i)
			{
				const ndDefinition& definition = jointsDefinition[i];
				if (!strcmp(definition.m_boneName, name))
				{
					//dTrace(("name: %s\n", name));
					if (definition.m_type == ndDefinition::m_hinge)
					{
						ndSharedPtr<ndBody> childBody(model->CreateBodyPart(scene, childEntity, 1.0f, parentNode->m_body->GetAsBodyDynamic()));

						bodyArray[bodyCount] = childBody->GetAsBodyKinematic();
						weightFactor[bodyCount] = definition.m_weightDistribution;
						bodyCount++;

						const ndMatrix pivotMatrix(childBody->GetMatrix());
						ndIkJointHinge* const hinge = new ndIkJointHinge(pivotMatrix, childBody->GetAsBodyDynamic(), parentNode->m_body->GetAsBodyDynamic());
						hinge->SetLimitState(true);
						hinge->SetLimits(definition.m_minAngle * ndDegreeToRad, definition.m_maxAngle * ndDegreeToRad);
						
						ndSharedPtr<ndJointBilateralConstraint> hingePtr(hinge);
						parentNode = model->AddLimb(parentNode, childBody, hingePtr);
					}
					else
					{
						ndDemoEntityNotify* notify = (ndDemoEntityNotify*)parentNode->m_body->GetNotifyCallback();
						
						notify = (ndDemoEntityNotify*)notify->m_parentBody->GetNotifyCallback();
						notify = (ndDemoEntityNotify*)notify->m_parentBody->GetNotifyCallback();
		
						ndMatrix effectorFrame(rootBody->GetMatrix());
						ndMatrix pivotFrame(ndRollMatrix(-90.0f * ndDegreeToRad) * rootBody->GetMatrix());
						pivotFrame.m_posit = notify->GetBody()->GetMatrix().m_posit;
						effectorFrame.m_posit = childEntity->CalculateGlobalMatrix().m_posit;
		
						ndFloat32 regularizer = 0.001f;
						ndIkSwivelPositionEffector* const effector = new ndIkSwivelPositionEffector(effectorFrame.m_posit, pivotFrame, parentNode->m_body->GetAsBodyDynamic(), modelRoot->m_body->GetAsBodyDynamic());
		
						effector->SetSwivelMode(false);
						effector->SetLinearSpringDamper(regularizer, 5000.0f, 50.0f);
						//effector->SetAngularSpringDamper(regularizer, 5000.0f, 50.0f);
		
						const ndVector elbowPoint(childEntity->GetParent()->CalculateGlobalMatrix().m_posit);
						const ndVector dist0(effectorFrame.m_posit - elbowPoint);
						const ndVector dist1(elbowPoint - pivotFrame.m_posit);
						const ndFloat32 workSpace = ndSqrt(dist0.DotProduct(dist0).GetScalar()) + ndSqrt(dist1.DotProduct(dist1).GetScalar());
						effector->SetWorkSpaceConstraints(0.0f, workSpace * 0.95f);

						ndSharedPtr<ndJointBilateralConstraint> effectotPtr(effector);
						ndRobot::ndEffectorInfo info(effectotPtr);
						model->m_effectorsInfo.PushBack(info);

						ndAnimKeyframe keyFrame;
						keyFrame.m_userData = &model->m_effectorsInfo[model->m_effectorsInfo.GetCount() - 1];
						model->m_animPose.PushBack(keyFrame);
						poseGenerator->AddTrack();
						poseGenerator->m_phase[effectorsCount] = phase[effectorsCount];
						poseGenerator->m_offset[effectorsCount] = info.m_basePosition;
						effectorsCount++;
					}
					break;
				}
			}
		
			for (ndDemoEntity* child = childEntity->GetFirstChild(); child; child = child->GetNext())
			{
				childEntities[stack] = child;
				parentBone[stack] = parentNode;
				stack++;
			}
		}

		model->NormalizeMassDistribution(25.0f, bodyCount, bodyArray, weightFactor);
		
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
		TrainingUpdata(ndDemoEntityManager* const scene, const ndMatrix& matrix, ndSharedPtr<ndDemoEntity>& modelMesh)
			:OnPostUpdate()
			,m_master()
			,m_bestActor()
			,m_outFile(nullptr)
			,m_timer(ndGetTimeInMicroseconds())
			,m_maxScore(ndFloat32(-1.0e10f))
			,m_maxFrames(1000)
			,m_lastEpisode(-1)
			,m_stopTraining(200 * 1000000)
			,m_modelIsTrained(false)
		{
			ndWorld* const world = scene->GetWorld();
			
			m_outFile = fopen("quadruped_3-VPG.csv", "wb");
			fprintf(m_outFile, "vpg\n");
			
			//const ndInt32 countX = 0;
			//const ndInt32 countZ = 0;
			const ndInt32 countX = 6;
			const ndInt32 countZ = 9;
			
			ndBrainAgentContinuePolicyGradient_TrainerMaster<ND_AGENT_INPUT_SIZE, ND_AGENT_OUTPUT_SIZE>::HyperParameters hyperParameters;
			
			//hyperParameters.m_threadsCount = 4;
			hyperParameters.m_discountFactor = ndReal(0.995f);
			hyperParameters.m_maxTrajectorySteps = 1024 * 8;
			
			m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster<ND_AGENT_INPUT_SIZE, ND_AGENT_OUTPUT_SIZE>>(new ndBrainAgentContinuePolicyGradient_TrainerMaster<ND_AGENT_INPUT_SIZE, ND_AGENT_OUTPUT_SIZE>(hyperParameters));
			m_bestActor = ndSharedPtr< ndBrain>(new ndBrain(*m_master->GetActor()));
			
			m_master->SetName(CONTROLLER_NAME);
			
			ndSharedPtr<ndBrainAgent> visualAgent(new ndRobot::ndControllerAgent_trainer(m_master));
			ndSharedPtr<ndModel> visualModel(BuildModel(scene, modelMesh, matrix, visualAgent));
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
					location.m_posit.m_x += 20.0f * (ndRand() - 0.5f);
					location.m_posit.m_z += 20.0f * (ndRand() - 0.5f);
					ndSharedPtr<ndBrainAgent> agent(new ndRobot::ndControllerAgent_trainer(m_master));
					ndSharedPtr<ndModel> model(BuildModel(scene, modelMesh, location, agent));
					world->AddModel(model);
					m_models.Append(model);
					//HideModel(model);
					SetMaterial(model);
				}
			}

			ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(visualModel->GetAsModelArticulation()->GetRoot()->m_body->GetMatrix(), visualModel->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic(), world->GetSentinelBody()));
			//world->AddJoint(fixJoint);
			((ndRobot*)*visualModel)->m_showDebug = true;

			scene->SetAcceleratedUpdate();
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

		void OnDebug(ndDemoEntityManager* const, bool mode)
		{
			for (ndList<ndSharedPtr<ndModel>>::ndNode* node = m_models.GetFirst(); node; node = node->GetNext())
			{
				HideModel(node->GetInfo(), mode);
			}
		}

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
				if (m_master->GetAverageFrames() >= ndFloat32(m_maxFrames))
				{
					if (m_master->GetAverageScore() > m_maxScore)
					{
						if (m_lastEpisode != m_master->GetEposideCount())
						{
							m_bestActor->CopyFrom(*m_master->GetActor());
							m_maxScore = m_master->GetAverageScore();
							ndExpandTraceMessage("best actor episode: %d\taverageFrames: %f\taverageValue %f\n", m_master->GetEposideCount(), m_master->GetAverageFrames(), m_master->GetAverageScore());
							m_lastEpisode = m_master->GetEposideCount();
						}
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
		
			if (stopTraining >= m_stopTraining)
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
		ndInt32 m_maxFrames;
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

using namespace ndQuadruped_3;
void ndQuadrupedTest_3(ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFlatPlane(scene, true);
	//BuildFloorBox(scene, ndGetIdentityMatrix());
	
	ndVector origin1(0.0f, 0.0f, 0.0f, 1.0f);
	ndMeshLoader loader;
	ndSharedPtr<ndDemoEntity> modelMesh (loader.LoadEntity("spotBoston.fbx", scene));

	ndMatrix matrix(ndYawMatrix(-0.0f * ndDegreeToRad));
	
	// register a material for filtering self collisions 
	ndQuadrupedMaterial material;
	material.m_restitution = 0.0f;
	material.m_staticFriction0 = 0.9f;
	material.m_staticFriction1 = 0.9f;
	material.m_dynamicFriction0 = 0.9f;
	material.m_dynamicFriction1 = 0.9f;
	
	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
	callback->RegisterMaterial(material, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_default);

#ifdef ND_TRAIN_MODEL
	TrainingUpdata* const trainer = new TrainingUpdata(scene, matrix, modelMesh);
	scene->RegisterPostUpdate(trainer);
#else
	ndWorld* const world = scene->GetWorld();
	ndSharedPtr<ndModel> model(BuildModel(scene, modelMesh, matrix, BuildAgent()));
	world->AddModel(model);
	scene->SetSelectedModel(*model);

	ndSharedPtr<ndUIEntity> quadrupedUI(new ndModelUI(scene, model));
	scene->Set2DDisplayRenderFunction(quadrupedUI);

	ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(model->GetAsModelArticulation()->GetRoot()->m_body->GetMatrix(), model->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic(), world->GetSentinelBody()));
	//world->AddJoint(fixJoint);

	//ndVector posit(matrix.m_posit);
	//posit.m_x += 1.5f;
	//posit.m_z += 1.5f;
	//AddBox(scene, posit, 2.0f, 0.3f, 0.4f, 0.7f);
	//AddBox(scene, posit, 1.0f, 0.3f, 0.4f, 0.7f);

	//posit.m_x += 0.6f;
	//posit.m_z += 0.2f;
	//AddBox(scene, posit, 8.0f, 0.3f, 0.4f, 0.7f);
	//AddBox(scene, posit, 4.0f, 0.3f, 0.4f, 0.7f);
#endif

	matrix.m_posit.m_x -= 20.0f;
	matrix.m_posit.m_y += 1.5f;
	matrix.m_posit.m_z += 0.25f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
