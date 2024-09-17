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

namespace ndAdvancedRobot
{
	//#define ND_TRAIN_MODEL
	#define CONTROLLER_NAME "ndRobotArmReach"

	//#define CONTROLLER_RESUME_TRAINING

	#define ND_USE_EULERS

	class ndActionVector
	{
		public:
		//ndBrainFloat m_actions[6];
		//ndBrainFloat m_actions[3];
		ndBrainFloat m_actions[1];
	};

	class ndObservationVector
	{
		public:
		// joint state
		ndBrainFloat m_jointPosit[6];
		ndBrainFloat m_jointVeloc[6];

		ndBrainFloat m_collided;

		// distance to target error.
		ndBrainFloat m_delta_x;
		ndBrainFloat m_delta_y;
		ndBrainFloat m_deltaAzimuth;

		#ifdef ND_USE_EULERS
			ndBrainFloat m_deltaYaw;
			ndBrainFloat m_deltaRoll;
			ndBrainFloat m_deltaPitch;
		#else
			ndBrainFloat m_rotation[4];
		#endif
	};

	class ndControlParameters
	{
		public:
		ndControlParameters()
			:m_x(0.0f)
			,m_y(0.0f)
			,m_azimuth(0.0f)
			,m_gripperPosit(0.0f)
		{
			#ifdef ND_USE_EULERS
				m_yaw = 0.0f;
				m_roll = 0.0f;
				m_pitch = 0.0f;
			#else
				m_headRotation = ndQuaternion();
			#endif
		}

		ndReal m_x;
		ndReal m_y;
		ndReal m_azimuth;
		ndReal m_gripperPosit;

		#ifdef ND_USE_EULERS
			ndReal m_yaw;
			ndReal m_roll;
			ndReal m_pitch;
		#else
			ndQuaternion m_headRotation;
		#endif
	};

	#define ND_AGENT_OUTPUT_SIZE	(sizeof (ndActionVector) / sizeof (ndBrainFloat))
	#define ND_AGENT_INPUT_SIZE		(sizeof (ndObservationVector) / sizeof (ndBrainFloat))

	#define ND_MIN_X_SPAND			ndReal (-1.5f)
	#define ND_MAX_X_SPAND			ndReal ( 1.5f)
	#define ND_MIN_Y_SPAND			ndReal (-2.2f)
	#define ND_MAX_Y_SPAND			ndReal ( 1.5f)
	#define ND_ACTION_SENSITIVITY	ndReal ( 0.05f)

	#define ND_DEAD_PENALTY			ndReal (-10.0f)

	class ndDefinition
	{
		public:
		enum ndJointType
		{
			m_root,
			m_hinge,
			m_slider,
			m_effector
		};

		char m_boneName[32];
		ndJointType m_type;
		ndFloat32 m_mass;
		ndFloat32 m_minLimit;
		ndFloat32 m_maxLimit;
		ndFloat32 m_maxStrength;
	};

	static ndDefinition jointsDefinition[] =
	{
		{ "base", ndDefinition::m_root, 100.0f, 0.0f, 0.0f},
		{ "base_rotator", ndDefinition::m_hinge, 50.0f, -1.0e10f, 1.0e10f},
		{ "arm_0", ndDefinition::m_hinge , 5.0f, -120.0f * ndDegreeToRad, 60.0f * ndDegreeToRad},
		{ "arm_1", ndDefinition::m_hinge , 5.0f, -90.0f * ndDegreeToRad, 60.0f * ndDegreeToRad},
		{ "arm_2", ndDefinition::m_hinge , 5.0f, -1.0e10f, 1.0e10f},
		{ "arm_3", ndDefinition::m_hinge , 3.0f, -1.0e10f, 1.0e10f},
		{ "arm_4", ndDefinition::m_hinge , 2.0f, -1.0e10f, 1.0e10f},
		{ "gripperLeft", ndDefinition::m_slider  , 1.0f, -0.2f, 0.03f},
		{ "gripperRight", ndDefinition::m_slider , 1.0f, -0.2f, 0.03f},
		//{ "effector", ndDefinition::m_effector , 0.0f, 0.0f, 0.0f},
	};

	class ndRobotBodyNotify : public ndDemoEntityNotify
	{
		public:
		ndRobotBodyNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity, ndBodyKinematic* const parentBody = nullptr)
			:ndDemoEntityNotify(manager, entity, parentBody)
		{
		}

		#pragma optimize( "", off )
		virtual bool OnSceneAabbOverlap(const ndBody* const otherBody) const
		{
			const ndBodyKinematic* const body0 = ((ndBody*)GetBody())->GetAsBodyKinematic();
			const ndBodyKinematic* const body1 = ((ndBody*)otherBody)->GetAsBodyKinematic();
			const ndShapeInstance& instanceShape0 = body0->GetCollisionShape();
			const ndShapeInstance& instanceShape1 = body1->GetCollisionShape();

			if (instanceShape0.m_shapeMaterial.m_userParam[0].m_ptrData != instanceShape1.m_shapeMaterial.m_userParam[0].m_ptrData)
			{
				if (instanceShape0.m_shapeMaterial.m_userParam[0].m_ptrData && instanceShape1.m_shapeMaterial.m_userParam[0].m_ptrData)
				{
					return false;
				}
			}

			//if ((instanceShape0.m_shapeMaterial.m_userId == ndDemoContactCallback::m_modelPart) &&
			//	(instanceShape1.m_shapeMaterial.m_userId == ndDemoContactCallback::m_modelPart))
			//{
			//	return false;
			//}
			return true;
		}
	};

	class RobotModelNotify : public ndModelNotify
	{
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
				m_body->SetOmega(m_omega);
				m_body->SetVelocity(m_veloc);
				m_body->SetMatrix(matrix);
			}

			ndVector m_veloc;
			ndVector m_omega;
			ndVector m_posit;
			ndQuaternion m_rotation;
			ndBodyDynamic* m_body;
		};

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

			//#pragma optimize( "", off )
			void SaveTrajectory()
			{
				if (IsTerminal())
				{
					ndInt32 index = m_trajectory.GetCount() - 1;
					if (m_trajectory.GetReward(index) != ND_DEAD_PENALTY)
					{
						m_trajectory.SetReward(index, ND_DEAD_PENALTY);
					}
				}
				ndBrainAgentContinuePolicyGradient_Trainer::SaveTrajectory();
			}

			void GetObservation(ndBrainFloat* const observation)
			{
				m_robot->GetObservation(observation);
			}

			virtual void ApplyActions(ndBrainFloat* const actions)
			{
				m_robot->ApplyActions(actions);
				m_robot->CheckModelStability();
			}

			void ResetModel()
			{
				m_robot->ResetModel();
			}

			RobotModelNotify* m_robot;
			ndReal m_rewardsMemories[32];
		};

		public:
		RobotModelNotify(ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>& master, ndModelArticulation* const robot)
			:ndModelNotify()
			,m_effectorLocalBase(ndGetIdentityMatrix())
			,m_effectorLocalTarget(ndGetIdentityMatrix())
			,m_effectorReference(ndGetIdentityMatrix())
			,m_controller(nullptr)
			,m_controllerTrainer(nullptr)
			,m_world(nullptr)
			,m_arm_0(nullptr)
			,m_arm_1(nullptr)
			,m_arm_2(nullptr)
			,m_arm_3(nullptr)
			,m_arm_4(nullptr)
			,m_base_rotator(nullptr)
			,m_leftGripper(nullptr)
			,m_rightGripper(nullptr)
			,m_targetLocation()
			,m_timestep(ndFloat32(0.0f))
			,m_modelAlive(true)
			,m_showDebug(false)
		{
			m_controllerTrainer = new ndControllerTrainer(master);
			m_controllerTrainer->m_robot = this;
			Init(robot);
		}

		RobotModelNotify(const ndSharedPtr<ndBrain>& brain, ndModelArticulation* const robot, bool showDebug)
			:ndModelNotify()
			,m_effectorLocalBase(ndGetIdentityMatrix())
			,m_effectorLocalTarget(ndGetIdentityMatrix())
			,m_effectorReference(ndGetIdentityMatrix())
			,m_controller(nullptr)
			,m_controllerTrainer(nullptr)
			,m_world(nullptr)
			,m_arm_0(nullptr)
			,m_arm_1(nullptr)
			,m_arm_2(nullptr)
			,m_arm_3(nullptr)
			,m_arm_4(nullptr)
			,m_base_rotator(nullptr)
			,m_leftGripper(nullptr)
			,m_rightGripper(nullptr)
			,m_targetLocation()
			,m_timestep(ndFloat32(0.0f))
			,m_modelAlive(true)
			,m_showDebug(showDebug)
		{
			m_controller = new ndController(brain);
			m_controller->m_robot = this;
			Init(robot);
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
			m_modelId = ndInt32 (robot->GetRoot()->m_body->GetId());
			m_arm_0 = (ndJointHinge*)*robot->FindByName("arm_0")->m_joint;
			m_arm_1 = (ndJointHinge*)*robot->FindByName("arm_1")->m_joint;
			m_arm_2 = (ndJointHinge*)*robot->FindByName("arm_2")->m_joint;
			m_arm_3 = (ndJointHinge*)*robot->FindByName("arm_3")->m_joint;
			m_arm_4 = (ndJointHinge*)*robot->FindByName("arm_4")->m_joint;
			m_base_rotator = (ndJointHinge*)* robot->FindByName("base_rotator")->m_joint;
			m_leftGripper = (ndJointSlider*)*robot->FindByName("gripperLeft")->m_joint;
			m_rightGripper = (ndJointSlider*)*robot->FindByName("gripperRight")->m_joint;

			m_armJoints.PushBack(m_arm_0);
			m_armJoints.PushBack(m_arm_1);
			m_armJoints.PushBack(m_arm_2);
			m_armJoints.PushBack(m_arm_3);
			m_armJoints.PushBack(m_arm_4);
			m_armJoints.PushBack(m_base_rotator);
			for (ndModelArticulation::ndNode* node = robot->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				m_basePose.PushBack(node->m_body->GetAsBodyDynamic());
			}

			//ndBodyDynamic* const body = m_arm_4->GetBody0()->GetAsBodyDynamic();
			//ndDemoEntity* const entity = (ndDemoEntity*)body->GetNotifyCallback()->GetUserData();
			//ndDemoEntity* const effectoEntity = entity->Find("effector");
			//
			//const ndMatrix alignMatrix(ndRollMatrix(-ndPi * 0.5f));
			//const ndMatrix effectorLocalMatrix(effectoEntity->GetCurrentMatrix());
			////m_effectorMatrixOffset = alignMatrix * effectorLocalMatrix;
			//m_effectorMatrixOffset = effectorLocalMatrix;
			//const ndMatrix baseMatrix(m_base_rotator->CalculateGlobalMatrix1());
			//const ndMatrix effectorMatrix(m_effectorMatrixOffset * m_arm_4->CalculateGlobalMatrix0());
			//m_effectorPositOffset = baseMatrix.UntransformVector(effectorMatrix.m_posit);

			ndBodyDynamic* const rootBody = robot->GetRoot()->m_body->GetAsBodyDynamic();
			ndDemoEntity* const rootEntity = (ndDemoEntity*)rootBody->GetNotifyCallback()->GetUserData();
			ndDemoEntity* const effectorEntity = rootEntity->Find("effector");

			const ndMatrix referenceFrame(rootEntity->Find("referenceFrame")->CalculateGlobalMatrix());
			const ndMatrix effectorFrame(effectorEntity->CalculateGlobalMatrix());

			ndAssert(rootBody == m_base_rotator->GetBody1());
			m_effectorLocalBase = referenceFrame * rootBody->GetMatrix().OrthoInverse();
			m_effectorLocalTarget = effectorFrame * m_arm_4->GetBody0()->GetMatrix().OrthoInverse();;
			m_effectorReference = effectorFrame * referenceFrame.OrthoInverse();
		}

		bool ModelCollided() const 
		{
			const ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
			for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				const ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();
				const ndBodyKinematic::ndContactMap& contacts = body->GetContactMap();
				ndBodyKinematic::ndContactMap::Iterator it(contacts);
				for (it.Begin(); it; it++)
				{
					ndContact* const contact = *it;
					if (contact->IsActive())
					{
						return true;
					}
				}
			}
			return false;
		}

		#pragma optimize( "", off )
		bool IsTerminal() const
		{
			//ndAssert(0);
			if (!m_modelAlive)
			{
				return true;
			}
			
			auto ChechHingeLimits = [this](const ndJointHinge* const joint)
			{
				ndFloat32 minLimit;
				ndFloat32 maxLimit;
				ndFloat32 angle = joint->GetAngle();
				joint->GetLimits(minLimit, maxLimit);
				return (angle >= minLimit) && (angle <= maxLimit);
			};

			for (ndInt32 i = 0; i < m_armJoints.GetCount(); ++i)
			{
				if (!ChechHingeLimits(m_armJoints[i]))
				{
					return true;
				}
			}

			auto ChechGripperLimits = [this](const ndJointSlider* const joint)
			{
				ndFloat32 minLimit;
				ndFloat32 maxLimit;
				ndFloat32 posit = joint->GetPosit();
				joint->GetLimits(minLimit, maxLimit);
				return (posit >= minLimit) && (posit <= maxLimit);
			};

			if (!ChechGripperLimits(m_leftGripper))
			{
				return true;
			}

			if (!ChechGripperLimits(m_rightGripper))
			{
				return true;
			}

			const ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
			for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			{
				const ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();
			
				const ndVector omega(body->GetOmega());
				const ndVector veloc(body->GetVelocity());
				ndFloat32 vMag2 = veloc.DotProduct(veloc).GetScalar();
				if (vMag2 > 400.0f)
				{
					return true;
				}
			
				ndFloat32 wMag2 = omega.DotProduct(omega).GetScalar();
				if (wMag2 > 250.0f)
				{
					return true;
				}
			}

			if (ModelCollided())
			{
				return true;
			}

			return false;
		}

		#pragma optimize( "", off )
		ndReal GetReward() const
		{
			if (IsTerminal())
			{
				return ND_DEAD_PENALTY;
			}

			ndTrace(("xxxxxxxxxxxxxx\n"));
			return 0;
			//const ndMatrix invBaseMatrix(m_base_rotator->CalculateGlobalMatrix1().OrthoInverse());
			//const ndMatrix effectorMatrix(m_effectorMatrixOffset * m_arm_4->CalculateGlobalMatrix0() * invBaseMatrix);
			//
			//ndFloat32 azimuth = 0.0f;
			//const ndVector& posit = effectorMatrix.m_posit;
			//if ((posit.m_y * posit.m_y + posit.m_z * posit.m_z) > 1.0e-3f)
			//{
			//	azimuth = ndAtan2(posit.m_z, posit.m_y);
			//}
			//const ndMatrix aximuthMatrix(ndPitchMatrix(azimuth));
			//const ndVector currenPosit(aximuthMatrix.UnrotateVector(posit) - m_effectorPositOffset);
			//
			//ndFloat32 dx = m_targetLocation.m_x - currenPosit.m_x;
			//ndFloat32 dy = m_targetLocation.m_y - currenPosit.m_y;
			//ndFloat32 dAzimuth = ndAnglesSub(m_targetLocation.m_azimuth, azimuth);
			//
			//#ifdef ND_USE_EULERS 
			//	ndVector euler1;
			//	ndVector euler(effectorMatrix.CalcPitchYawRoll(euler1));
			//	ndFloat32 deltaYaw = ndAnglesSub(euler.m_y, m_targetLocation.m_yaw);
			//	ndFloat32 deltaRoll = ndAnglesSub(euler.m_z, m_targetLocation.m_roll);
			//	ndFloat32 deltaPitch = ndAnglesSub(euler.m_x, m_targetLocation.m_pitch);
			//
			//	ndFloat32 posit_xReward = ndExp(-200.0f * dx * dx);
			//	ndFloat32 posit_yReward = ndExp(-200.0f * dy * dy);
			//	ndFloat32 azimuthReward = ndExp(-200.0f * dAzimuth * dAzimuth);
			//	ndFloat32 yawReward = ndExp(-200.0f * deltaYaw * deltaYaw);
			//	ndFloat32 rollReward = ndExp(-200.0f * deltaRoll * deltaRoll);
			//	ndFloat32 pitchReward = ndExp(-200.0f * deltaPitch * deltaPitch);
			//
			//	//const ndFloat32 rewardWeight = 1.0 / 8.0f;
			//	//return rewardWeight * (posit_xReward + posit_yReward + yawReward + rollReward + pitchReward + 3.0f * azimuthReward);
			//	return (posit_xReward + posit_yReward + azimuthReward) / 3.0f;
			//
			//#else
			//	ndQuaternion effectorRotation(effectorMatrix);
			//	ndFloat32 dRotation = effectorRotation.DotProduct(m_targetLocation.m_headRotation).GetScalar();
			//	if (dRotation < 0.0f)
			//	{
			//		dRotation = effectorRotation.DotProduct(m_targetLocation.m_headRotation.Scale(-1.0f)).GetScalar();
			//	}
			//	dRotation = 1.0f - dRotation;
			//	ndFloat32 angleError2 = dRotation * dRotation;
			//
			//	ndFloat32 positReward = ndExp(-100.0f * positError2);
			//	ndFloat32 azimuthReward = ndExp(-100.0f * azimuth2);
			//	ndFloat32 rotationReward = ndExp(-50.0f * angleError2);
			//	return azimuthReward * 0.4f + positReward * 0.3f + rotationReward * 0.3f;
			//#endif
		}

		#pragma optimize( "", off )
		void GetObservation(ndBrainFloat* const inputObservations)
		{
			ndObservationVector* const observation = (ndObservationVector*)inputObservations;
			for (ndInt32 i = m_armJoints.GetCount() - 1; i >= 0; --i)
			{ 
				const ndJointBilateralConstraint* const joint = m_armJoints[i];
			
				ndJointBilateralConstraint::ndKinematicState kinematicState;
				joint->GetKinematicState(&kinematicState);
				observation->m_jointPosit[i] = ndBrainFloat(kinematicState.m_posit);
				observation->m_jointVeloc[i] = ndBrainFloat(kinematicState.m_velocity);
			}

			observation->m_collided = ndBrainFloat(ModelCollided() ? 1.0f : 0.0f);
			ndTrace(("xxxxxxx\n"));

			observation->m_deltaYaw = 0.0f;
			observation->m_deltaRoll = 0.0f;
			observation->m_deltaPitch = 0.0f;
			observation->m_delta_x = 0.0f;
			observation->m_delta_y = 0.0f;
			observation->m_deltaAzimuth = 0.0f;

			//const ndMatrix invBaseMatrix(m_base_rotator->CalculateGlobalMatrix1().OrthoInverse());
			//const ndMatrix effectorMatrix(m_effectorMatrixOffset * m_arm_4->CalculateGlobalMatrix0() * invBaseMatrix);
			//
			//#ifdef ND_USE_EULERS
			//	ndVector euler1;
			//	ndVector euler(effectorMatrix.CalcPitchYawRoll(euler1));
			//	ndFloat32 deltaYaw = ndAnglesSub(euler.m_y, m_targetLocation.m_yaw);
			//	ndFloat32 deltaRoll = ndAnglesSub(euler.m_z, m_targetLocation.m_roll);
			//	ndFloat32 deltaPitch = ndAnglesSub(euler.m_x, m_targetLocation.m_pitch);
			//
			//	observation->m_deltaYaw = ndBrainFloat(deltaYaw);
			//	observation->m_deltaRoll = ndBrainFloat(deltaRoll);
			//	observation->m_deltaPitch = ndBrainFloat(deltaPitch);
			//#else
			//	ndQuaternion effectorRotation(effectorMatrix);
			//	if (effectorRotation.DotProduct(m_targetLocation.m_headRotation).GetScalar() < 0.0f)
			//	{
			//		effectorRotation = effectorRotation.Scale(-1.0f);
			//	}
			//	const ndQuaternion rotation(effectorRotation * m_targetLocation.m_headRotation.Inverse());
			//	observation->m_rotation[0] = ndBrainFloat(rotation.m_x);
			//	observation->m_rotation[1] = ndBrainFloat(rotation.m_y);
			//	observation->m_rotation[2] = ndBrainFloat(rotation.m_z);
			//	observation->m_rotation[3] = ndBrainFloat(rotation.m_w);
			//#endif
			//
			//ndFloat32 azimuth = 0.0f;
			//const ndVector& posit = effectorMatrix.m_posit;
			//if ((posit.m_y * posit.m_y + posit.m_z * posit.m_z) > 1.0e-3f)
			//{
			//	azimuth = ndAtan2(posit.m_z, posit.m_y);
			//}
			//const ndMatrix aximuthMatrix(ndPitchMatrix(azimuth));
			//const ndVector currenPosit(aximuthMatrix.UnrotateVector(posit) - m_effectorPositOffset);
			//
			//observation->m_delta_x = ndBrainFloat(m_targetLocation.m_x - currenPosit.m_x);
			//observation->m_delta_y = ndBrainFloat(m_targetLocation.m_y - currenPosit.m_y);
			//observation->m_deltaAzimuth = ndBrainFloat(ndAnglesSub(m_targetLocation.m_azimuth, azimuth));
			//
			////static int xxxx = 0;
			////if (xxxx)
			////{
			////	const ndMatrix invBaseMatrix(m_base_rotator->CalculateGlobalMatrix1().OrthoInverse());
			////	const ndMatrix effectorMatrix(m_effectorMatrixOffset * m_arm_4->CalculateGlobalMatrix0() * invBaseMatrix);
			////
			////	ndFloat32 azimuth1 = 0.0f;
			////	const ndVector& posit = effectorMatrix.m_posit;
			////	if ((posit.m_y * posit.m_y + posit.m_z * posit.m_z) > 1.0e-3f)
			////	{
			////		azimuth1 = ndAtan2(posit.m_z, posit.m_y);
			////	}
			////	const ndMatrix aximuthMatrix(ndPitchMatrix(azimuth1));
			////	const ndVector currenPosit(aximuthMatrix.UnrotateVector(posit) - m_effectorPositOffset);
			////
			////	ndFloat32 dx = m_targetLocation.m_x - currenPosit.m_x;
			////	ndFloat32 dy = m_targetLocation.m_y - currenPosit.m_y;
			////	ndFloat32 dAzimuth = ndAnglesSub(m_targetLocation.m_azimuth, azimuth);
			////
			////	ndAssert(ndAbs(dx - observation->m_delta_x) < 1.0e-3f);
			////	ndAssert(ndAbs(dy - observation->m_delta_y) < 1.0e-3f);
			////	ndAssert(ndAbs(dAzimuth - observation->m_deltaAzimuth) < 1.0e-3f);
			////}
		}

		//#pragma optimize( "", off )
		void ApplyActions(ndBrainFloat* const actions)
		{
			auto SetParamter = [this, actions](ndJointHinge* const hinge, ndInt32 index)
			{
				ndFloat32 angle = hinge->GetAngle();
				ndFloat32 deltaAngle = actions[index] * ND_ACTION_SENSITIVITY;
				ndFloat32 targetAngle = ndAnglesAdd (angle, deltaAngle);
				ndAssert(ndAbs(targetAngle - (angle + deltaAngle)) < 1.0e-4f);
				hinge->SetTargetAngle(targetAngle);
			};

			GetReward();

			//SetParamter(m_arm_0, 0);
			//SetParamter(m_arm_1, 1);
			//SetParamter(m_arm_2, 2);
			//SetParamter(m_arm_3, 3);
			//SetParamter(m_arm_4, 4);
			//SetParamter(m_base_rotator, 5);
			//SetParamter(m_base_rotator, 2);
			//SetParamter(m_base_rotator, 0);
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

		//#pragma optimize( "", off )
		void ResetModel()
		{
			m_modelAlive = true;
			for (ndInt32 i = 0; i < m_basePose.GetCount(); i++)
			{
				m_basePose[i].SetPose();
			}

			for (ndInt32 i = 0; i < m_armJoints.GetCount(); ++i)
			{
				m_armJoints[i]->SetTargetAngle(0.0f);
			}

			GetModel()->GetAsModelArticulation()->ClearMemory();

			m_leftGripper->SetOffsetPosit(0.0f);
			m_rightGripper->SetOffsetPosit(0.0f);

			m_targetLocation.m_x = ND_MIN_X_SPAND + ndRand() * (ND_MAX_X_SPAND - ND_MIN_X_SPAND);
			m_targetLocation.m_y = ND_MIN_Y_SPAND + ndRand() * (ND_MAX_Y_SPAND - ND_MIN_Y_SPAND);
			m_targetLocation.m_azimuth = (2.0f * ndRand() - 1.0f) * ndPi;

			m_targetLocation.m_x = ndClamp(m_targetLocation.m_x, ndReal(ND_MIN_X_SPAND + 0.05f), ndReal(ND_MAX_X_SPAND - 0.05f));
			m_targetLocation.m_y = ndClamp(m_targetLocation.m_y, ndReal(ND_MIN_Y_SPAND + 0.05f), ndReal(ND_MAX_Y_SPAND - 0.05f));
			m_targetLocation.m_azimuth = ndClamp(m_targetLocation.m_azimuth, ndReal(-ndPi + 0.09f), ndReal(ndPi - 0.09f));

			ndFloat32 yaw = ndFloat32((1.0f * ndRand() - 0.5f) * ndPi);
			ndFloat32 pitch = ndFloat32((2.0f * ndRand() - 1.0f) * ndPi);
			ndFloat32 roll = ndFloat32(-ndPi * 0.35f + ndRand() * (ndPi * 0.9f - (-ndPi * 0.35f)));

			m_targetLocation.m_x = 0.0f;
			m_targetLocation.m_y = 0.0f;
			//m_targetLocation.m_azimuth = 0.0f;
			yaw = 0.0f * ndDegreeToRad;
			roll = 0.0f * ndDegreeToRad;
			pitch = 0.0f * ndDegreeToRad;
			#ifdef ND_USE_EULERS
				m_targetLocation.m_yaw = yaw;
				m_targetLocation.m_roll = roll;
				m_targetLocation.m_pitch = pitch;
			#else
				m_targetLocation.m_headRotation = ndQuaternion(ndPitchMatrix(pitch) * ndYawMatrix(yaw) * ndRollMatrix(roll));
			#endif
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

		void PostUpdate(ndWorld* const, ndFloat32)
		{
		}

		void PostTransformUpdate(ndWorld* const, ndFloat32)
		{
		}

		ndMatrix CalculateTargetMatrix() const
		{
			#ifdef ND_USE_EULERS
				ndMatrix targetMatrix(ndPitchMatrix(m_targetLocation.m_pitch) * ndYawMatrix(m_targetLocation.m_yaw) * ndRollMatrix(m_targetLocation.m_roll));
			#else
				ndMatrix targetMatrix(ndCalculateMatrix(m_targetLocation.m_headRotation));
			#endif

			return m_effectorReference * m_effectorLocalBase;
			//ndFloat32 x = m_targetLocation.m_x;
			//ndFloat32 y = m_targetLocation.m_y;
			//const ndVector localPosit(x, y, ndFloat32(0.0f), ndFloat32(0.0f));
			//const ndMatrix aximuthMatrix(ndPitchMatrix(m_targetLocation.m_azimuth));
			//const ndVector offsetPosit(localPosit + m_effectorPositOffset);
			//targetMatrix.m_posit = aximuthMatrix.TransformVector(offsetPosit);
			//return targetMatrix;
		}

		void Debug(ndConstraintDebugCallback& context) const
		{
			if (!m_showDebug)
			{
				//return;
			}
			
			const ndVector color(1.0f, 0.0f, 0.0f, 1.0f);

			const ndMatrix baseMatrix(m_effectorLocalBase * m_base_rotator->GetBody1()->GetMatrix());
			context.DrawFrame(baseMatrix);
			context.DrawPoint(baseMatrix.m_posit, color, ndFloat32(5.0f));

			const ndMatrix effectorMatrix(m_effectorLocalTarget * m_arm_4->GetBody0()->GetMatrix());
			context.DrawFrame(effectorMatrix);
			context.DrawPoint(effectorMatrix.m_posit, color, ndFloat32(5.0f));

			const ndMatrix targetMatrix(CalculateTargetMatrix() * m_base_rotator->GetBody1()->GetMatrix());
			context.DrawFrame(targetMatrix);
			context.DrawPoint(targetMatrix.m_posit, color, ndFloat32(5.0f));
		}

		ndMatrix m_effectorLocalBase;
		ndMatrix m_effectorLocalTarget;
		ndMatrix m_effectorReference;

		ndController* m_controller;
		ndControllerTrainer* m_controllerTrainer;
		ndWorld* m_world;
		ndJointHinge* m_arm_0;
		ndJointHinge* m_arm_1;
		ndJointHinge* m_arm_2;
		ndJointHinge* m_arm_3;
		ndJointHinge* m_arm_4;
		ndJointHinge* m_base_rotator;
		ndJointSlider* m_leftGripper;
		ndJointSlider* m_rightGripper;

		ndFixSizeArray<ndBasePose, 16> m_basePose;
		ndFixSizeArray<ndJointHinge*, 16> m_armJoints;

		ndControlParameters m_targetLocation;
		ndFloat32 m_timestep;
		ndInt32 m_modelId;
		bool m_modelAlive;
		bool m_showDebug;

		friend class ndRobotUI;
	};

	class ndRobotUI : public ndUIEntity
	{
		public:
		ndRobotUI(ndDemoEntityManager* const scene, RobotModelNotify* const robot)
			:ndUIEntity(scene)
			,m_robot(robot)
		{
		}

		~ndRobotUI()
		{
		}

		virtual void RenderUI()
		{
		}

		virtual void RenderHelp()
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			m_scene->Print(color, "Control panel");

			ndInt8 change = 0;
			
			#ifdef ND_USE_EULERS
				ndReal yaw = m_robot->m_targetLocation.m_yaw;
				ndReal roll = m_robot->m_targetLocation.m_roll;
				ndReal pitch = m_robot->m_targetLocation.m_pitch;
			#else
				ndVector euler1;
				ndVector euler(m_robot->m_targetLocation.m_headRotation.GetEulerAngles(euler1));
				ndReal pitch = ndReal(euler.m_x);
				ndReal yaw = ndReal(euler.m_y);
				ndReal roll = ndReal(euler.m_z);
			#endif
			
			change = change | ndInt8 (ImGui::SliderFloat("x", &m_robot->m_targetLocation.m_x, ND_MIN_Y_SPAND, ND_MAX_Y_SPAND));
			change = change | ndInt8(ImGui::SliderFloat("y", &m_robot->m_targetLocation.m_y, ND_MIN_X_SPAND, ND_MAX_X_SPAND));
			change = change | ndInt8 (ImGui::SliderFloat("azimuth", &m_robot->m_targetLocation.m_azimuth, -ndPi, ndPi));
			change = change | ndInt8(ImGui::SliderFloat("pitch", &pitch, -ndPi, ndPi));
			change = change | ndInt8(ImGui::SliderFloat("yaw", &yaw, -ndPi * 0.5f, ndPi * 0.5f));
			change = change | ndInt8(ImGui::SliderFloat("roll", &roll, -ndPi * 0.35f, ndPi * 0.9f));
			change = change | ndInt8 (ImGui::SliderFloat("gripper", &m_robot->m_targetLocation.m_gripperPosit, -0.2f, 0.4f));
			
			bool newTarget = ndInt8(ImGui::Button("random target"));
			if (newTarget)
			{
				change = 1;
				pitch = ndReal((2.0f * ndRand() - 1.0f) * ndPi);
				yaw = ndReal((2.0f * ndRand() - 1.0f) * ndPi * 0.5f);
				roll = ndReal(-ndPi * 0.35f + ndRand() * (ndPi * 0.9f - (-ndPi * 0.35f)));
			
				m_robot->m_targetLocation.m_azimuth = ndReal((2.0f * ndRand() - 1.0f) * ndPi);
				m_robot->m_targetLocation.m_x = ndReal(ND_MIN_X_SPAND + ndRand() * (ND_MAX_X_SPAND - ND_MIN_X_SPAND));
				m_robot->m_targetLocation.m_y = ndReal(ND_MIN_Y_SPAND + ndRand() * (ND_MAX_Y_SPAND - ND_MIN_Y_SPAND));

				yaw = 0.0f;
				roll = 0.0f;
				pitch = 0.0f;
				m_robot->m_targetLocation.m_x = 0.0f;
				m_robot->m_targetLocation.m_y = 0.0f;
				//m_robot->m_targetLocation.m_azimuth = ndReal((2.0f * ndRand() - 1.0f) * ndPi);
			}
			#ifdef ND_USE_EULERS
				m_robot->m_targetLocation.m_yaw = yaw;
				m_robot->m_targetLocation.m_roll = roll;
				m_robot->m_targetLocation.m_pitch = pitch;
			#else
				m_robot->m_targetLocation.m_headRotation = ndQuaternion (ndPitchMatrix(pitch) * ndYawMatrix(yaw) * ndRollMatrix(roll));
				//if (m_robot->m_targetLocation.m_headRotation.DotProduct(m_robot->m_location.m_headRotation).GetScalar() < 0.0f)
				//{
				//	m_robot->m_targetLocation.m_headRotation = m_robot->m_targetLocation.m_headRotation.Scale(-1.0f);
				//}
			#endif
			
			if (change)
			{
				m_robot->GetModel()->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic()->SetSleepState(false);
			}
		}

		RobotModelNotify* m_robot;
	};

	ndBodyDynamic* CreateBodyPart(ndDemoEntityManager* const scene, ndDemoEntity* const entityPart, ndFloat32 mass, ndBodyDynamic* const parentBone, ndModelArticulation* const model)
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
		body->SetNotifyCallback(new ndRobotBodyNotify(scene, entityPart, parentBone));

		ndShapeInstance& instanceShape = body->GetCollisionShape();
		instanceShape.m_shapeMaterial.m_userId = ndDemoContactCallback::m_modelPart;
		instanceShape.m_shapeMaterial.m_userParam[0].m_ptrData = model;

		return body->GetAsBodyDynamic();
	}

	void NormalizeInertia(ndModelArticulation* const model)
	{
		for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
		{
			ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();

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

	ndModelArticulation* CreateModel(ndDemoEntityManager* const scene, ndDemoEntity* const modelMesh, const ndMatrix& location)
	{
		// make a clone of the mesh and add it to the scene
		ndModelArticulation* const model = new ndModelArticulation();

		ndDemoEntity* const entity = modelMesh->CreateClone();
		scene->AddEntity(entity);

		ndDemoEntity* const rootEntity = (ndDemoEntity*)entity->Find(jointsDefinition[0].m_boneName);
		ndMatrix matrix(rootEntity->CalculateGlobalMatrix() * location);
		rootEntity->ResetMatrix(matrix);

		// add the root body
		ndSharedPtr<ndBody> rootBody(CreateBodyPart(scene, rootEntity, jointsDefinition[0].m_mass, nullptr, model));

		rootBody->SetMatrix(rootEntity->CalculateGlobalMatrix());

		// add the root body to the model
		ndModelArticulation::ndNode* const modelNode = model->AddRootBody(rootBody);
		modelNode->m_name = jointsDefinition[0].m_boneName;

		ndFixSizeArray<ndDemoEntity*, 32> childEntities;
		ndFixSizeArray<ndModelArticulation::ndNode*, 32> parentBones;
		for (ndDemoEntity* child = rootEntity->GetFirstChild(); child; child = child->GetNext())
		{
			childEntities.PushBack(child);
			parentBones.PushBack(modelNode);
		}

		const ndInt32 definitionCount = ndInt32(sizeof(jointsDefinition) / sizeof(jointsDefinition[0]));
		while (parentBones.GetCount())
		{
			ndDemoEntity* const childEntity = childEntities.Pop();
			ndModelArticulation::ndNode* parentBone = parentBones.Pop();

			const char* const name = childEntity->GetName().GetStr();
			for (ndInt32 i = 0; i < definitionCount; ++i)
			{
				const ndDefinition& definition = jointsDefinition[i];
				if (!strcmp(definition.m_boneName, name))
				{
					//ndTrace(("name: %s\n", name));
					if (definition.m_type == ndDefinition::m_hinge)
					{
						ndSharedPtr<ndBody> childBody(CreateBodyPart(scene, childEntity, definition.m_mass, parentBone->m_body->GetAsBodyDynamic(), model));
						const ndMatrix pivotMatrix(childBody->GetMatrix());
						ndSharedPtr<ndJointBilateralConstraint> hinge(new ndJointHinge(pivotMatrix, childBody->GetAsBodyKinematic(), parentBone->m_body->GetAsBodyKinematic()));

						ndJointHinge* const hingeJoint = (ndJointHinge*)*hinge;

						hingeJoint->SetAsSpringDamper(1.0e-3f, 3000.0f, 100.0f);
						hingeJoint->SetLimits(definition.m_minLimit, definition.m_maxLimit);
						if ((definition.m_minLimit > -1000.0f) && (definition.m_maxLimit < 1000.0f))
						{
							hingeJoint->SetLimitState(true);
						}

						parentBone = model->AddLimb(parentBone, childBody, hinge);
						parentBone->m_name = name;
					}
					else if (definition.m_type == ndDefinition::m_slider)
					{
						ndSharedPtr<ndBody> childBody(CreateBodyPart(scene, childEntity, definition.m_mass, parentBone->m_body->GetAsBodyDynamic(), model));

						const ndMatrix pivotMatrix(childBody->GetMatrix());
						ndSharedPtr<ndJointBilateralConstraint> slider(new ndJointSlider(pivotMatrix, childBody->GetAsBodyKinematic(), parentBone->m_body->GetAsBodyKinematic()));

						ndJointSlider* const sliderJoint = (ndJointSlider*)*slider;
						sliderJoint->SetLimits(definition.m_minLimit, definition.m_maxLimit);
						sliderJoint->SetLimitState(true);
						sliderJoint->SetAsSpringDamper(0.001f, 2000.0f, 100.0f);
						parentBone = model->AddLimb(parentBone, childBody, slider);
						parentBone->m_name = definition.m_boneName;
					}
					break;
				}
			}

			for (ndDemoEntity* child = childEntity->GetFirstChild(); child; child = child->GetNext())
			{
				childEntities.PushBack(child);
				parentBones.PushBack(parentBone);
			}
		}

		NormalizeInertia(model);
		return model;
	}

	void AddBackgroundScene(ndDemoEntityManager* const scene, const ndMatrix& matrix)
	{
		ndMatrix location(matrix);
		location.m_posit.m_x += 1.5f;
		location.m_posit.m_z += 1.5f;
		AddBox(scene, location, 2.0f, 0.3f, 0.4f, 0.7f);
		AddBox(scene, location, 1.0f, 0.3f, 0.4f, 0.7f);

		location = ndYawMatrix(ndPi * 0.5f) * location;
		location.m_posit.m_x += 1.0f;
		location.m_posit.m_z += 0.5f;
		AddBox(scene, location, 8.0f, 0.3f, 0.4f, 0.7f);
		AddBox(scene, location, 4.0f, 0.3f, 0.4f, 0.7f);
	}

	class TrainingUpdata : public ndDemoEntityManager::OnPostUpdate
	{
		public:
		TrainingUpdata(ndDemoEntityManager* const scene, const ndMatrix& matrix, ndSharedPtr<ndDemoEntity>& visualMesh, ndBodyKinematic* const floor)
			:OnPostUpdate()
			,m_master()
			,m_bestActor()
			,m_outFile(nullptr)
			,m_timer(ndGetTimeInMicroseconds())
			//,m_horizon(ndFloat32(1.0f) / (ndFloat32(1.0f) - m_discountFactor))
			,m_maxScore(ndFloat32(-1.0e10f))
			,m_saveScore(m_maxScore)
			,m_discountFactor(0.99f)
			,m_lastEpisode(0xffffffff)
			,m_stopTraining(ndUnsigned32(2000)* ndUnsigned32(1000000))
			,m_modelIsTrained(false)
		{
			char name[256];
			m_horizon = ndFloat32(1.0f) / (ndFloat32(1.0f) - m_discountFactor);
			snprintf(name, sizeof(name), "%s-vpg.csv", CONTROLLER_NAME);
			m_outFile = fopen(name, "wb");
			fprintf(m_outFile, "vpg\n");

			ndBrainAgentContinuePolicyGradient_TrainerMaster::HyperParameters hyperParameters;

			//float xxx = 0.0f;
			//int xxxxx = 0; 
			//while (xxx < 99)
			//{
			//	xxxxx++;
			//	xxx = 1.0 + xxx * 0.99;
			//}

			//hyperParameters.m_threadsCount = 1;
			hyperParameters.m_maxTrajectorySteps = 1024 * 4;
			hyperParameters.m_extraTrajectorySteps = 512;
			hyperParameters.m_bashTrajectoryCount = 1000;
			hyperParameters.m_discountFactor = ndReal(m_discountFactor);
			hyperParameters.m_numberOfActions = ND_AGENT_OUTPUT_SIZE;
			hyperParameters.m_numberOfObservations = ND_AGENT_INPUT_SIZE;

			m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>(new ndBrainAgentContinuePolicyGradient_TrainerMaster(hyperParameters));
			m_bestActor = ndSharedPtr<ndBrain>(new ndBrain(*m_master->GetActor()));

			snprintf(name, sizeof(name), "%s.dnn", CONTROLLER_NAME);
			m_master->SetName(name);

			#ifdef CONTROLLER_RESUME_TRAINING
				char fileName[256];
				snprintf(name, sizeof(name), "%s_critic.dnn", CONTROLLER_NAME);
				ndGetWorkingFileName(name, fileName);
				ndSharedPtr<ndBrain> critic(ndBrainLoad::Load(fileName));
				m_master->GetCritic()->CopyFrom(**critic);

				snprintf(name, sizeof(name), "%s_actor.dnn", CONTROLLER_NAME);
				ndGetWorkingFileName(name, fileName);
				ndSharedPtr<ndBrain> actor(ndBrainLoad::Load(fileName));
				m_master->GetActor()->CopyFrom(**actor);
			#endif

			auto SpawnModel = [this, scene, &visualMesh, floor](const ndMatrix& matrix)
			{
				ndWorld* const world = scene->GetWorld();
				ndModelArticulation* const model = CreateModel(scene, *visualMesh, matrix);

				model->SetNotifyCallback(new RobotModelNotify(m_master, model));
				model->AddToWorld(world);
				((RobotModelNotify*)*model->GetNotifyCallback())->ResetModel();

				ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(model->GetRoot()->m_body->GetMatrix(), model->GetRoot()->m_body->GetAsBodyKinematic(), floor));
				world->AddJoint(fixJoint);
				return model;
			};

			ndInt32 countX = 22;
			ndInt32 countZ = 23;
			//countX = 10;
			//countZ = 10;

			// add a hidden battery of model to generate trajectories in parallel
			for (ndInt32 i = 0; i < countZ; ++i)
			{
				for (ndInt32 j = 0; j < countX; ++j)
				{
					ndMatrix location(matrix);
					location.m_posit.m_x += 10.0f * ndFloat32(j - countX/2);
					location.m_posit.m_z += 10.0f * ndFloat32(i - countZ/2);

					if ((i == countZ / 2) && (j == countX / 2))
					{
						//AddBackgroundScene(scene, location);
					}

					ndModelArticulation* const model = SpawnModel(location);
					if ((i == countZ/2) && (j == countX/2))
					{
						ndSharedPtr<ndUIEntity> robotUI(new ndRobotUI(scene, (RobotModelNotify*)*model->GetNotifyCallback()));
						scene->Set2DDisplayRenderFunction(robotUI);

						RobotModelNotify* const notify = (RobotModelNotify*)*model->GetNotifyCallback();
						notify->m_showDebug = true;
					}
					else
					{
						m_models.Append(model);
					}
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
			ndUnsigned32 stopTraining = m_master->GetFramesCount();
			if (stopTraining <= m_stopTraining)
			{
				ndUnsigned32 episodeCount = m_master->GetEposideCount();
				m_master->OptimizeStep();
			
				episodeCount -= m_master->GetEposideCount();
				ndFloat32 rewardTrajectory = m_master->GetAverageFrames() * m_master->GetAverageScore();
				if (rewardTrajectory >= m_maxScore)
				{
					if (m_lastEpisode != m_master->GetEposideCount())
					{
						m_maxScore = rewardTrajectory;
						m_bestActor->CopyFrom(*m_master->GetActor());
						ndExpandTraceMessage("best actor episode: %u\treward %f\ttrajectoryFrames: %f\n", m_master->GetEposideCount(), 100.0f * m_master->GetAverageScore() / m_horizon, m_master->GetAverageFrames());
						m_lastEpisode = m_master->GetEposideCount();
					}
				}
			
				if (episodeCount && !m_master->IsSampling())
				{
					ndExpandTraceMessage("steps: %u\treward: %g\t  trajectoryFrames: %g\n", m_master->GetFramesCount(), 100.0f * m_master->GetAverageScore() / m_horizon, m_master->GetAverageFrames());
					if (m_outFile)
					{
						fprintf(m_outFile, "%g\n", m_master->GetAverageScore());
						fflush(m_outFile);
					}
				}

				if (rewardTrajectory > m_saveScore)
				{
					char fileName[256];
					m_saveScore = ndFloor(rewardTrajectory) + 2.0f;

					// save partial controller in case of crash 
					ndBrain* const actor = m_master->GetActor();
					char name[256];
					snprintf(name, sizeof(name), "%s_actor.dnn", CONTROLLER_NAME);
					ndGetWorkingFileName(name, fileName);
					actor->SaveToFile(fileName);

					ndBrain* const critic = m_master->GetCritic();
					snprintf(name, sizeof(name), "%s_critic.dnn", CONTROLLER_NAME);
					ndGetWorkingFileName(name, fileName);
					critic->SaveToFile(fileName);
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
		ndFloat32 m_horizon;
		ndFloat32 m_maxScore;
		ndFloat32 m_saveScore;
		ndFloat32 m_discountFactor;
		ndUnsigned32 m_lastEpisode;
		ndUnsigned32 m_stopTraining;
		bool m_modelIsTrained;
	};
}

using namespace ndAdvancedRobot;
void ndAdvancedIndustrialRobot(ndDemoEntityManager* const scene)
{
	// build a floor
	ndBodyKinematic* const floor = BuildFloorBox(scene, ndGetIdentityMatrix());
	//BuildFloorBox(scene, ndGetIdentityMatrix());

	ndVector origin1(0.0f, 0.0f, 0.0f, 1.0f);
	ndMeshLoader loader;
	ndSharedPtr<ndDemoEntity> modelMesh(loader.LoadEntity("robot.fbx", scene));
	ndMatrix matrix(ndYawMatrix(-ndPi * 0.5f));

#ifdef ND_TRAIN_MODEL
	scene->RegisterPostUpdate(new TrainingUpdata(scene, matrix, modelMesh, floor));
#else

	AddBackgroundScene(scene, matrix);

	ndWorld* const world = scene->GetWorld();
	ndModelArticulation* const model = CreateModel(scene, *modelMesh, matrix);
	model->AddToWorld(world);

	ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(model->GetRoot()->m_body->GetMatrix(), model->GetRoot()->m_body->GetAsBodyKinematic(), floor));
	world->AddJoint(fixJoint);

	char name[256];
	char fileName[256];
	snprintf(name, sizeof(name), "%s.dnn", CONTROLLER_NAME);
	ndGetWorkingFileName(name, fileName);
	ndSharedPtr<ndBrain> brain(ndBrainLoad::Load(fileName));
	model->SetNotifyCallback(new RobotModelNotify(brain, model, true));
	
	ndSharedPtr<ndUIEntity> robotUI(new ndRobotUI(scene, (RobotModelNotify*)*model->GetNotifyCallback()));
	scene->Set2DDisplayRenderFunction(robotUI);

	matrix.m_posit.m_z += 1.5f;
	ndInt32 countZ = 5;
	ndInt32 countX = 5;
	
	//countZ = 0;
	//countX = 0;
	for (ndInt32 i = 0; i < countZ; ++i)
	{
		for (ndInt32 j = 0; j < countX; ++j)
		{
			//ndMatrix location(matrix);
			//location.m_posit.m_x += 3.0f * ndFloat32(j - countX / 2);
			//location.m_posit.m_z += 3.0f * ndFloat32(i - countZ / 2);
			//ndModelArticulation* const model = CreateModel(scene, location);
			//model->SetNotifyCallback(new RobotModelNotify(brain, model, false));
			//model->AddToWorld(world);
			////m_models.Append(model);
			////SetMaterial(model);
		}
	}
#endif
	
	matrix.m_posit.m_x -= 7.0f;
	matrix.m_posit.m_y += 2.5f;
	matrix.m_posit.m_z += 0.25f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), ndPi * 0.0f);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
