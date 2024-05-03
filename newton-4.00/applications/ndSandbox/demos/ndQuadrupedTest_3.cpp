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
	};
	#define ND_AGENT_OUTPUT_SIZE	(sizeof (ndActionVector) / sizeof (ndBrainFloat))
	#define ND_AGENT_INPUT_SIZE		(sizeof (ndObservationVector) / sizeof (ndBrainFloat))


	#define D_POSE_REST_POSITION_Y	ndReal (-0.3f)

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
	};

	static ndDefinition jointsDefinition[] =
	{
		{ "spot_body", ndDefinition::m_root, 0.0f, 0.0f, 0.0f},

		{ "spot_shoulder_FR", ndDefinition::m_hinge, -90.0f, 90.0f, 0.0f },
		{ "spot_up_arm_FR", ndDefinition::m_hinge, -130.0f, 130.0f, 0.0f },
		{ "spot_arm_FR", ndDefinition::m_hinge, -90.0f, 45.0f, 0.0f },
		{ "spot_arm_FR_effector", ndDefinition::m_effector, 0.0f, 0.0f, 0.0f },
		
		{ "spot_shoulder_FL", ndDefinition::m_hinge, -90.0f, 90.0f, 0.0f },
		{ "spot_up_arm_FL", ndDefinition::m_hinge, -130.0f, 130.0f, 0.0f },
		{ "spot_arm_FL", ndDefinition::m_hinge, -90.0f, 45.0f, 0.0f },
		{ "spot_arm_FL_effector", ndDefinition::m_effector, 0.0f, 0.0f, 0.0f },
		
		{ "spot_shoulder_BR", ndDefinition::m_hinge, -90.0f, 90.0f, 0.0f },
		{ "spot_up_arm_BR", ndDefinition::m_hinge, -130.0f, 130.0f, 0.0f },
		{ "spot_arm_BR", ndDefinition::m_hinge, -90.0f, 45.0f, 0.0f },
		{ "spot_arm_BR_effector", ndDefinition::m_effector, 0.0f, 0.0f, 0.0f },
		
		{ "spot_shoulder_BL", ndDefinition::m_hinge, -90.0f, 90.0f, 0.0f },
		{ "spot_up_arm_BL", ndDefinition::m_hinge, -130.0f, 130.0f, 0.0f },
		{ "spot_arm_BL", ndDefinition::m_hinge, -90.0f, 45.0f, 0.0f },
		{ "spot_arm_BL_effector", ndDefinition::m_effector, 0.0f, 0.0f, 0.0f },
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

	class ndWalkSequence : public ndAnimationSequence
	{
		public:
		class ndSegment
		{
			public:
			ndSegment()
				:m_a(ndVector::m_zero)
				,m_b(ndVector::m_zero)
				,m_t0(0.0f)
				,m_perimeter(0.0f)
			{
			}

			void Init(const ndVector& p0, const ndVector& p1, ndFloat32 t0, ndFloat32 t1)
			{
				ndFloat32 den = t1 - t0;
				m_a.m_x = (p1.m_x - p0.m_x) / den;
				m_b.m_x = (p0.m_x * t1 - p1.m_x * t0) / den;

				m_b.m_y = 0.0f;
				m_a.m_y = p1.m_y;

				m_t0 = t0;
				m_perimeter = ndPi / den;
			}

			ndVector Interpolate(ndFloat32 t) const
			{
				ndVector point(m_b);
				point.m_x += m_a.m_x * t;
				point.m_y += m_a.m_y * ndSin(m_perimeter * (t - m_t0));
				return point;
			}

			ndVector m_a;
			ndVector m_b;
			ndFloat32 m_t0;
			ndFloat32 m_perimeter;
		};

		ndWalkSequence(ndFloat32 midParam)
			:ndAnimationSequence()
			,m_segment0()
			,m_segment1()
			,m_xBias(0.11f)
			,m_xStride(1.0f)
			,m_midParam(midParam)
			,m_offsets()
			,m_isGrounded()
		{
			ndFloat32 walkStride = 0.3f;
			const ndVector p0(-walkStride, 0.0f, 0.0f, 0.0f);
			const ndVector p1(walkStride, 0.0f, 0.0f, 0.0f);
			const ndVector p2(-walkStride, 0.1f, 0.0f, 0.0f);
			m_segment0.Init(p0, p1, 0.0f, m_midParam);
			m_segment1.Init(p1, p2, m_midParam, 1.0f);

			m_isGrounded.SetCount(4);
			m_offsets.PushBack(0.0f);
			m_offsets.PushBack(0.0f);
			m_offsets.PushBack(0.0f);
			m_offsets.PushBack(0.0f);

			if (m_midParam > 0.5f)
			{
				// set walk sequence gait offset

				// front gait, left leg is 1/2 offset form the right leg
				m_offsets[1] = 0.5f; // front right leg
				m_offsets[2] = 0.0f; // front left leg

				// rear gait is a 3/4 delay form the front gait
				m_offsets[3] = 0.25f; // rear right leg
				m_offsets[0] = 0.75f; // rear left leg
			}
			else
			{
				// set trot sequence offset

				// front gait, left leg is 1/2 offset form the right leg
				m_offsets[1] = 0.5f; // front right leg
				m_offsets[2] = 0.0f; // front left leg

				// rear gait is a 1/2 delay form the front gait
				m_offsets[3] = 0.0f; // rear right leg
				m_offsets[0] = 0.5f; // rear left leg
			}
		}

		~ndWalkSequence()
		{
		}

		void InitParam(ndFloat32& b, ndFloat32& a, ndFloat32 x0, ndFloat32 t0, ndFloat32 x1, ndFloat32 t1) const
		{
			ndFloat32 den = t1 - t0;
			a = (x1 - x0) / den;
			b = (x0 * t1 - x1 * t0) / den;
		}

		void CalculatePose(ndAnimationPose& output, ndFloat32 param) const
		{
			ndAssert(output.GetCount() == m_offsets.GetCount());
			for (ndInt32 i = 0; i < m_offsets.GetCount(); ++i)
			{
				ndAnimKeyframe& keyFrame = output[i];

				const ndFloat32 t = ndMod(param + m_offsets[i], ndFloat32(1.0f));
				m_isGrounded[i] = t <= m_midParam;
				ndVector posit(m_isGrounded[i] ? m_segment0.Interpolate(t) : m_segment1.Interpolate(t));
				posit.m_x = posit.m_x * m_xStride - m_xBias;
				keyFrame.m_posit = posit;
				keyFrame.m_rotation = ndQuaternion();
			}
		}

		ndSegment m_segment0;
		ndSegment m_segment1;
		ndFloat32 m_xBias;
		ndFloat32 m_xStride;
		ndFloat32 m_midParam;
		ndFixSizeArray<ndFloat32, 4> m_offsets;
		mutable ndFixSizeArray<bool, 4> m_isGrounded;
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

				//ndFloat32 omega = ndPi / m_gaitFraction;

				ndFloat32 gaitGuard = m_gaitFraction * 0.05f;
				ndFloat32 omega = ndPi / (m_gaitFraction - ndFloat32(2.0f) * gaitGuard);

				ndFloat32 ycontact = D_POSE_REST_POSITION_Y + m_amp / 2.0f;
				for (ndInt32 i = 0; i < output.GetCount(); i++)
				{
					output[i].m_userParamInt = 0;
					output[i].m_posit = BasePose(i);
					ndFloat32 t = ndMod(param - m_phase[i] + ndFloat32(1.0f), ndFloat32(1.0f));
					if (t <= m_gaitFraction)
					{
						if ((t >= gaitGuard) && (t <= (m_gaitFraction - gaitGuard)))
						{
							//output[i].m_posit.m_y += m_amp * ndSin(omega * t);
							output[i].m_posit.m_y += m_amp * ndSin(omega * (t - gaitGuard));
							output[i].m_userParamInt = output[i].m_posit.m_y < ycontact ? -1 : 1;
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
				ndAssert(0);
				return 0;
				//ndBrainFloat reward = m_model->CalculateReward();
				//return reward;
			}

			//virtual void ApplyActions(ndBrainFloat* const actions)
			virtual void ApplyActions(ndBrainFloat* const)
			{
				ndAssert(0);
				//m_model->ApplyActions(actions);
			}

			//void GetObservation(ndBrainFloat* const observation)
			void GetObservation(ndBrainFloat* const)
			{
				ndAssert(0);
				//m_model->GetObservation(observation);
			}

			bool IsTerminal() const
			{
				ndAssert(0);
				return false;
				//ndInt32 count = 0;
				//ndInt32 isGround = 4;
				//
				//bool airborneLeg[4];
				//bool sequenceAirborne[4];
				//
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
				//
				//return false;
			}

			void ResetModel()
			{
				ndAssert(0);
				//m_model->m_control->Reset();
				//for (ndInt32 i = 0; i < m_basePose.GetCount(); i++)
				//{
				//	m_basePose[i].SetPose();
				//}
				//m_model->m_animBlendTree->SetTime(0.0f);
				//
				//ndFloat32 randVar = ndRand();
				//randVar = randVar * randVar * randVar;
				//ndFloat32 speed0 = ndFloat32(0.5f);
				//ndFloat32 speed1 = ndFloat32(4.0f);
				//ndFloat32 animationSpeed = speed0 + (speed1 - speed0) * randVar;
				//m_model->m_control->m_animSpeed = animationSpeed;
			}

			void Step()
			{
				ndBrainAgentContinuePolicyGradient_Trainer::Step();
			}

			ndFixSizeArray<ndBasePose, 32> m_basePose;
			ndRobot* m_model;
		};

		D_CLASS_REFLECTION(ndQuadruped_3::ndRobot, ndModel)


		ndRobot()
			:ndModelArticulation()
			,m_effectorsInfo()
			,m_effectorsJoints()
			,m_animPose()
			,m_control(nullptr)
			,m_poseGenerator(nullptr)
			,m_animBlendTree()
			,m_timerstep(0.0f)
		{
		}

		~ndRobot()
		{
		}

		void NormalizeMassDistribution(ndFloat32 mass) const
		{
			ndFloat32 volumeRatio = 0.02f;
			ndFloat32 maxVolume = -1.0e10f;
			for (ndNode* node = GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
				ndFloat32 volume = body->GetCollisionShape().GetVolume();
				maxVolume = ndMax(maxVolume, volume);
			}
			
			ndFloat32 totalVolume = 0.0f;
			for (ndNode* node = GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
				ndFloat32 volume = body->GetCollisionShape().GetVolume();
				if (volume < volumeRatio * maxVolume)
				{
					volume = volumeRatio * maxVolume;
				}
				totalVolume += volume;
			}
			
			ndFloat32 density = mass / totalVolume;
			for (ndNode* node = GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
				ndFloat32 volume = body->GetCollisionShape().GetVolume();
				if (volume < volumeRatio * maxVolume)
				{
					volume = volumeRatio * maxVolume;
				}
				ndFloat32 normalMass = density * volume;
				body->SetMassMatrix(normalMass, body->GetCollisionShape());
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
			body->SetNotifyCallback(new ndDemoEntityNotify(scene, entityPart, parentBone));

			ndShapeInstance& instanceShape = body->GetCollisionShape();
			instanceShape.m_shapeMaterial.m_userId = ndDemoContactCallback::m_modelPart;
			instanceShape.m_shapeMaterial.m_userParam[ndDemoContactCallback::m_modelPointer].m_ptrData = this;

			// add body to the world
			ndSharedPtr<ndBody> bodyPtr(body);
			scene->GetWorld()->AddBody(bodyPtr);
			return bodyPtr;
		}

		//ndBodyDynamic* GetRoot() const
		//{
		//	return m_bodyArray[0];
		//}

		ndVector CalculateCenterOfMass() const
		{
			ndAssert(0);
			return ndVector::m_zero;
			//ndFloat32 toltalMass = 0.0f;
			//ndVector com(ndVector::m_zero);
			//for (ndInt32 i = 0; i < m_bodyArray.GetCount(); ++i)
			//{
			//	ndBodyDynamic* const body = m_bodyArray[i];
			//	ndFloat32 mass = body->GetMassMatrix().m_w;
			//	ndVector comMass(body->GetMatrix().TransformVector(body->GetCentreOfMass()));
			//	com += comMass.Scale(mass);
			//	toltalMass += mass;
			//}
			//com = com.Scale(1.0f / toltalMass);
			//com.m_w = 1.0f;
			//return com;
		}

		//void Debug(ndConstraintDebugCallback& context) const
		void Debug(ndConstraintDebugCallback&) const
		{
			////ndMatrix comMatrix(m_bodyArray[0]->GetMatrix());
			////comMatrix.m_posit = CalculateCenterOfMass();
			////context.DrawFrame(comMatrix);
			////
			////ndFixSizeArray<ndVector, 16> contactPoints;
			////for (ndInt32 i = 0; i < m_effectors.GetCount(); ++i)
			////{
			////	//const ndEffectorInfo& info = m_effectors[i];
			////	//ndJointBilateralConstraint* const joint = info.m_effector;
			////	//ndBodyKinematic* const body = joint->GetBody0();
			////	//const ndBodyKinematic::ndContactMap& contacts = body->GetContactMap();
			////	//ndBodyKinematic::ndContactMap::Iterator it(contacts);
			////	//for (it.Begin(); it; it++)
			////	//{
			////	//	const ndContact* const contact = *it;
			////	//	if (contact->IsActive())
			////	//	{
			////	//		//const ndContactPointList& contactMap = contact->GetContactPoints();
			////	//		//contactPoints.PushBack(contactMap.GetFirst()->GetInfo().m_point);
			////	//		contactPoints.PushBack(body->GetMatrix().TransformVector(info.m_effector->GetLocalMatrix0().m_posit));
			////	//	}
			////	//}
			////	if (m_walkCycle.m_isGrounded[i])
			////	{
			////		const ndEffectorInfo& info = m_effectors[i];
			////		ndJointBilateralConstraint* const joint = info.m_effector;
			////		ndBodyKinematic* const body = joint->GetBody0();
			////		contactPoints.PushBack(body->GetMatrix().TransformVector(info.m_effector->GetLocalMatrix0().m_posit));
			////	}
			////
			////	//	joint->DebugJoint(context);
			////}
			////
			////if (contactPoints.GetCount() >= 3)
			////{
			////	ndMatrix rotation(ndPitchMatrix(90.0f * ndDegreeToRad));
			////	rotation.TransformTriplex(&contactPoints[0].m_x, sizeof(ndVector), &contactPoints[0].m_x, sizeof(ndVector), contactPoints.GetCount());
			////	ndInt32 supportCount = ndConvexHull2d(&contactPoints[0], contactPoints.GetCount());
			////	rotation.Inverse().TransformTriplex(&contactPoints[0].m_x, sizeof(ndVector), &contactPoints[0].m_x, sizeof(ndVector), contactPoints.GetCount());
			////	ndVector p0(contactPoints[supportCount - 1]);
			////	ndBigVector bigPolygon[16];
			////	for (ndInt32 i = 0; i < supportCount; ++i)
			////	{
			////		bigPolygon[i] = contactPoints[i];
			////		context.DrawLine(contactPoints[i], p0, ndVector::m_zero);
			////		p0 = contactPoints[i];
			////	}
			////
			////	ndBigVector p0Out;
			////	ndBigVector p1Out;
			////	ndBigVector ray_p0(comMatrix.m_posit);
			////	ndBigVector ray_p1(comMatrix.m_posit);
			////	ray_p1.m_y -= 1.0f;
			////
			////	ndRayToPolygonDistance(ray_p0, ray_p1, bigPolygon, supportCount, p0Out, p1Out);
			////
			////	context.DrawPoint(p0Out, ndVector(1.0f, 0.0f, 0.0f, 1.0f), 3);
			////	context.DrawPoint(p1Out, ndVector(0.0f, 1.0f, 0.0f, 1.0f), 3);
			////}
			//
			//for (ndInt32 i = 0; i < m_effectorsInfo.GetCount(); ++i)
			//{
			//	const ndEffectorInfo& info = m_effectorsInfo[i];
			//	ndJointBilateralConstraint* const joint = info.m_effector;
			//	joint->DebugJoint(context);
			//}
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
				//effector->SetLocalTargetPosition(posit);
			}

			m_invDynamicsSolver.SolverBegin(skeleton, &effectors[0], effectors.GetCount(), m_world, timestep);
			m_invDynamicsSolver.Solve();
			m_invDynamicsSolver.SolverEnd();
		}

		void PostTransformUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			ndModelArticulation::PostTransformUpdate(world, timestep);
		}

		void PostUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			//ndVector veloc;
			//m_animBlendTree->Evaluate(m_animPose, veloc);

			m_animBlendTree->Update(timestep * m_control->m_animSpeed);
			ndModelArticulation::PostUpdate(world, timestep);
		}

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			ndModelArticulation::Update(world, timestep);

			m_timerstep = timestep;
			//m_agent->Step();

			UpdatePose(timestep);


			//m_bodyArray[0]->SetSleepState(false);
			//ndFloat32 animSpeed = m_param_xxxx.Interpolate(m_param_x0);
			//m_timer = ndMod(m_timer + timestep * animSpeed, ndFloat32(1.0f));
			//
			////ndAssert(0);
			////m_walk->SetParam(1.0f - m_timer);
			////m_animBlendTree->Evaluate(m_output);
			////for (ndInt32 i = 0; i < m_effectorsInfo.GetCount(); i++)
			////{
			////	ndEffectorInfo& info = m_effectorsInfo[i];
			////	const ndAnimKeyframe& keyFrame = m_output[i];
			////	ndVector posit(info.m_basePosition);
			////	posit.m_x += keyFrame.m_posit.m_x;
			////	posit.m_y += keyFrame.m_posit.m_y;
			////	posit.m_z += keyFrame.m_posit.m_z;
			////	info.m_effector->SetLocalTargetPosition(posit);
			////}
			//
			//ndSkeletonContainer* const skeleton = m_bodyArray[0]->GetSkeleton();
			//ndAssert(skeleton);
			//
			////m_invDynamicsSolver.SetMaxIterations(4);
			//if (m_effectorsJoints.GetCount() && !m_invDynamicsSolver.IsSleeping(skeleton))
			//{
			//	ndFixSizeArray<ndJointBilateralConstraint*, 8> effectors;
			//	for (ndInt32 i = 0; i < m_effectorsJoints.GetCount(); ++i)
			//	{
			//		effectors.PushBack(*m_effectorsJoints[i]);
			//	}
			//
			//	m_invDynamicsSolver.SolverBegin(skeleton, &effectors[0], effectors.GetCount(), world, timestep);
			//	m_invDynamicsSolver.Solve();
			//	m_invDynamicsSolver.SolverEnd();
			//}
		}

		//ndIkSolver m_invDynamicsSolver;
		//ndAnimationSequencePlayer* m_walk;
		//ndAnimationBlendTreeNode* m_animBlendTree;
		//ndAnimationPose m_output;
		//ndWalkSequence m_walkCycle;
		//ndWalkSequence m_trotCycle;
		//ndFixSizeArray<ndBodyDynamic*, 16> m_bodyArray;

		ndFixSizeArray<ndEffectorInfo, 4> m_effectorsInfo;
		ndFixSizeArray<ndSharedPtr<ndJointBilateralConstraint>, 8> m_effectorsJoints;

		ndAnimationPose m_animPose;
		ndUIControlNode* m_control;
		ndAnimationSequencePlayer* m_poseGenerator;
		ndSharedPtr<ndAnimationBlendTreeNode> m_animBlendTree;
		ndFloat32 m_timerstep;
		//ndReal m_param_x0;
		//ndParamMapper m_param_xxxx;
	};

	class ndModelUI : public ndUIEntity
	{
		public:
		ndModelUI(ndDemoEntityManager* const scene, ndRobot* const quadruped)
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
		}

		ndRobot* m_model;
	};

	ndModelArticulation* BuildModel(ndDemoEntityManager* const scene, ndSharedPtr<ndDemoEntity> modelMesh, const ndMatrix& matrixLocation, ndSharedPtr<ndBrainAgent> agent)
	{
		ndDemoEntity* const entity = modelMesh->CreateClone();
		scene->AddEntity(entity);
		//ndWorld* const world = scene->GetWorld();
		
		ndRobot* const model = new ndRobot();
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

		ndInt32 stack = 0;
		for (ndDemoEntity* child = rootEntity->GetFirstChild(); child; child = child->GetNext())
		{
			childEntities[stack] = child;
			parentBone[stack] = modelRoot;
			stack++;
		}
		
		ndFloat32 phase[] = { 0.0f, 0.75f, 0.25f, 0.5f };
		ndSharedPtr<ndAnimationSequence> sequence(new ndRobot::ndPoseGenerator(0.24f, phase));

		model->m_poseGenerator = new ndAnimationSequencePlayer(sequence);
		model->m_control = new ndRobot::ndUIControlNode(model->m_poseGenerator);
		model->m_animBlendTree = ndSharedPtr<ndAnimationBlendTreeNode>(model->m_control);

		ndRobot::ndPoseGenerator* const poseGenerator = (ndRobot::ndPoseGenerator*)*sequence;

		ndFloat32 offset[] = { -0.3f, 0.3f, -0.3f, 0.3f };

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
						effector->SetAngularSpringDamper(regularizer, 5000.0f, 50.0f);
		
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
						poseGenerator->m_phase[i] = phase[i];
						poseGenerator->m_offset[i] = offset[i];
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
		
		model->NormalizeMassDistribution(100.0f);
		
		//m_timer = 0.0f;
		//m_param_x0 = -1.0f;
		//m_param_xxxx = ndParamMapper(0.0, 0.75f);
		//m_output.SetCount(4);

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
			,m_maxFrames(6000)
			,m_lastEpisode(-1)
			,m_stopTraining(200 * 1000000)
			,m_modelIsTrained(false)
		{
			ndWorld* const world = scene->GetWorld();
			
			m_outFile = fopen("quadruped_3-VPG.csv", "wb");
			fprintf(m_outFile, "vpg\n");
			
			const ndInt32 countX = 0;
			const ndInt32 countZ = 0;
			//const ndInt32 countX = 6;
			//const ndInt32 countZ = 9;
			
			ndBrainAgentContinuePolicyGradient_TrainerMaster<ND_AGENT_INPUT_SIZE, ND_AGENT_OUTPUT_SIZE>::HyperParameters hyperParameters;
			
			//hyperParameters.m_threadsCount = 1;
			//hyperParameters.m_sigma = ndReal(0.25f);
			hyperParameters.m_discountFactor = ndReal(0.99f);
			//hyperParameters.m_maxTrajectorySteps = 1024 * 6;
			hyperParameters.m_maxTrajectorySteps = 1024 * 8;
			
			m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster<ND_AGENT_INPUT_SIZE, ND_AGENT_OUTPUT_SIZE>>(new ndBrainAgentContinuePolicyGradient_TrainerMaster<ND_AGENT_INPUT_SIZE, ND_AGENT_OUTPUT_SIZE>(hyperParameters));
			m_bestActor = ndSharedPtr< ndBrain>(new ndBrain(*m_master->GetActor()));
			
			m_master->SetName(CONTROLLER_NAME);
			
			ndSharedPtr<ndBrainAgent> visualAgent(new ndRobot::ndControllerAgent_trainer(m_master));
			ndSharedPtr<ndModel> visualModel(BuildModel(scene, modelMesh, matrix, visualAgent));
			world->AddModel(visualModel);
			//SetMaterial(visualModel);

			ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(visualModel->GetAsModelArticulation()->GetRoot()->m_body->GetMatrix(), visualModel->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic(), world->GetSentinelBody()));
			//world->AddJoint(fixJoint);

			//ndSharedPtr<ndUIEntity> quadrupedUI(new ndModelUI(scene, visualModel));
			//scene->Set2DDisplayRenderFunction(quadrupedUI);
			
			// add a hidden battery of model to generate trajectories in parallel
			for (ndInt32 i = 0; i < countZ; ++i)
			{
				ndAssert(0);
				for (ndInt32 j = 0; j < countX; ++j)
				{
					ndMatrix location(matrix);
					location.m_posit.m_x += 6.0f * (ndRand() - 0.5f);
					location.m_posit.m_z += 6.0f * (ndRand() - 0.5f);
					ndSharedPtr<ndBrainAgent> agent(new ndRobot::ndControllerAgent_trainer(m_master));
					ndSharedPtr<ndModel> model(BuildModel(scene, modelMesh, location, agent));
					world->AddModel(model);
					m_models.Append(model);
					//HideModel(model);
					//SetMaterial(model);
				}
			}
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
		
		//class InvisibleBodyNotify : public ndDemoEntityNotify
		//{
		//public:
		//	InvisibleBodyNotify(const ndDemoEntityNotify* const src)
		//		:ndDemoEntityNotify(*src)
		//	{
		//	}
		//
		//	virtual bool OnSceneAabbOverlap(const ndBody* const otherBody) const
		//	{
		//		const ndBodyKinematic* const body0 = ((ndBody*)GetBody())->GetAsBodyKinematic();
		//		const ndBodyKinematic* const body1 = ((ndBody*)otherBody)->GetAsBodyKinematic();
		//		const ndShapeInstance& instanceShape0 = body0->GetCollisionShape();
		//		const ndShapeInstance& instanceShape1 = body1->GetCollisionShape();
		//		return instanceShape0.m_shapeMaterial.m_userId != instanceShape1.m_shapeMaterial.m_userId;
		//	}
		//};
		//
		//void SetMaterial(ndSharedPtr<ndModel>& model) const
		//{
		//	ndRobot* const robot = (ndRobot*)*model;
		//
		//	ndModelArticulation::ndNode* stackMem[128];
		//	ndInt32 stack = 1;
		//	stackMem[0] = robot->GetRoot();
		//	while (stack)
		//	{
		//		stack--;
		//		ndModelArticulation::ndNode* const node = stackMem[stack];
		//		ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
		//
		//		ndShapeInstance& instanceShape = body->GetCollisionShape();
		//		instanceShape.m_shapeMaterial.m_userId = ndDemoContactCallback::m_modelPart;
		//
		//		ndDemoEntityNotify* const originalNotify = (ndDemoEntityNotify*)body->GetNotifyCallback();
		//		void* const useData = originalNotify->m_entity;
		//		originalNotify->m_entity = nullptr;
		//		InvisibleBodyNotify* const notify = new InvisibleBodyNotify((InvisibleBodyNotify*)body->GetNotifyCallback());
		//		body->SetNotifyCallback(notify);
		//		notify->m_entity = (ndDemoEntity*)useData;
		//
		//		for (ndModelArticulation::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
		//		{
		//			stackMem[stack] = child;
		//			stack++;
		//		}
		//	}
		//}
		//
		//void OnDebug(ndDemoEntityManager* const, bool mode)
		//{
		//	for (ndList<ndSharedPtr<ndModel>>::ndNode* node = m_models.GetFirst(); node; node = node->GetNext())
		//	{
		//		HideModel(node->GetInfo(), mode);
		//	}
		//}
		
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
	material.m_restitution = 0.1f;
	material.m_staticFriction0 = 0.9f;
	material.m_staticFriction1 = 0.9f;
	material.m_dynamicFriction0 = 0.9f;
	material.m_dynamicFriction1 = 0.9f;
	
	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
	callback->RegisterMaterial(material, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_default);
	callback->RegisterMaterial(material, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_modelPart);

#ifdef ND_TRAIN_MODEL
	TrainingUpdata* const trainer = new TrainingUpdata(scene, matrix, modelMesh);
	scene->RegisterPostUpdate(trainer);
#else
	ndWorld* const world = scene->GetWorld();
	ndSharedPtr<ndModel> model(BuildModel(scene, modelMesh, matrix));
	world->AddModel(model);
	scene->SetSelectedModel(*model);

	//ndModelUI* const quadrupedUI = new ndModelUI(scene, robot);
	ndModelUI* const quadrupedUI = new ndModelUI(scene, (ndRobot*)*model);
	ndSharedPtr<ndUIEntity> quadrupedUIPtr(quadrupedUI);
	scene->Set2DDisplayRenderFunction(quadrupedUIPtr);

	ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(model->GetAsModelArticulation()->GetRoot()->m_body->GetMatrix(), model->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic(), world->GetSentinelBody()));
	world->AddJoint(fixJoint);

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

	matrix.m_posit.m_x -= 5.0f;
	matrix.m_posit.m_y += 1.5f;
	matrix.m_posit.m_z += 0.25f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
