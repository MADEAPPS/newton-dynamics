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

	#define USE_DDPG

	#ifdef USE_DDPG
		#define CONTROLLER_NAME "ndQuadruped_2-sac.dnn"
	#else	
		#define CONTROLLER_NAME "ndQuadruped_2-ppo.dnn"
	#endif

	enum Actions
	{
		m_actionPosit_x,
		m_actionPosit_z,
		m_actionsSize
	};

	enum Observations
	{
		m_posePosit_x,
		m_posePosit_y,
		m_posePosit_z,
		m_poseVeloc_x,
		m_poseVeloc_y,
		m_poseVeloc_z,
		m_effectorPosit_x,
		m_effectorPosit_y,
		m_effectorPosit_z,
		m_effectorVeloc_x,
		m_effectorVeloc_y,
		m_effectorVeloc_z,

		m_contactDistance,
		m_observationSize
	};

	#define D_CYCLE_PERIOD			ndFloat32(4.0f)
	#define D_CYCLE_STRIDE_X		ndFloat32(0.3f)
	#define D_CYCLE_STRIDE_Z		ndFloat32(0.3f)
	#define D_CYCLE_AMPLITUDE		ndFloat32(0.27f)
	#define D_POSE_REST_POSITION_Y	ndFloat32(-0.3f)

	#define D_ACTION_SENSITIVITY	ndReal(0.002f)

	#define D_TILT_MAX_TILL_ANGLE	ndFloat32(25.0f * ndDegreeToRad)

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
		
			ndVector GetTranslation(ndFloat32) const override
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
					
					stride_x = 0.0f;
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
				,m_softContact(nullptr)
				,m_effector(nullptr)
			{
			}

			ndEffectorInfo(
				ndJointSpherical* const thigh,
				ndJointHinge* const calf,
				ndJointHinge* const heel,
				ndJointSlider* softContact,
				ndIkSwivelPositionEffector* const effector)
				:m_calf(calf)
				,m_heel(heel)
				,m_thigh(thigh)
				,m_softContact(softContact)
				,m_effector(effector)
			{
			}

			ndJointHinge* m_calf;
			ndJointHinge* m_heel;
			ndJointSpherical* m_thigh;
			ndJointSlider* m_softContact;
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
		
#ifdef USE_DDPG
		class ndControllerTrainer : public ndBrainAgentOffPolicyGradient_Agent
#else
		class ndControllerTrainer : public ndBrainAgentContinuePolicyGradient_Agent
#endif
		{
			public:
#ifdef USE_DDPG
			ndControllerTrainer(const ndSharedPtr<ndBrainAgentOffPolicyGradient_Trainer>& master, RobotModelNotify* const robot)
				:ndBrainAgentOffPolicyGradient_Agent(master)
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

		class ndRayCastFloor : public ndRayCastClosestHitCallback
		{
			public:
			ndRayCastFloor(ndWorld* const world, const ndEffectorInfo& leg)
				:ndRayCastClosestHitCallback()
			{
				ndMatrix origin(leg.m_effector->CalculateGlobalMatrix0());
				ndVector dest(origin.m_posit);
				dest.m_y -= D_CYCLE_AMPLITUDE;
				world->RayCast(*this, origin.m_posit, dest);
			}

			ndFloat32 OnRayCastAction(const ndContactPoint& contact, ndFloat32 intersetParam)
			{
				if (contact.m_body1->GetInvMass() != ndFloat32(0.0f))
				{
					return 1.2f;
				}
				return ndRayCastClosestHitCallback::OnRayCastAction(contact, intersetParam);
			}
		};

		public:
		RobotModelNotify(ndModelArticulation* const robot)
			:ndModelNotify()
			,m_solver()
			,m_animPose0()
			,m_animPose1()
			,m_poseGenerator()
			,m_animBlendTree()
			,m_basePose()
			,m_legs()
			,m_controller(nullptr)
			,m_controllerTrainer(nullptr)
			,m_comX(ndFloat32(0.0f))
			,m_comZ(ndFloat32(0.0f))
			,m_timestep(ndFloat32(0.0f))
			,m_modelEnum(0)
		{
			SetModel(robot);
		}

		~RobotModelNotify()
		{
		}

		#ifdef USE_DDPG
		void SetControllerTrainer(const ndSharedPtr<ndBrainAgentOffPolicyGradient_Trainer>& master)
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

		//#pragma optimize( "", off )
		bool IsTerminal() const
		{
			const ndModelArticulation::ndNode* const rootNode = GetModel()->GetAsModelArticulation()->GetRoot();
			const ndVector upDir (rootNode->m_body->GetMatrix().m_up);
			if (upDir.m_y < ndCos(D_TILT_MAX_TILL_ANGLE * ndFloat32 (1.5f)))
			{
				return true;
			}

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

			return false;
		}

		ndModelArticulation::ndCenterOfMassDynamics CalculateDynamics(ndFloat32 timestep) const
		{
			ndModelArticulation::ndNode* const rootNode = GetModel()->GetAsModelArticulation()->GetRoot();
			ndMatrix referenceFrame(rootNode->m_body->GetMatrix());
			referenceFrame.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
			referenceFrame.m_right = referenceFrame.m_front.CrossProduct(referenceFrame.m_up).Normalize();
			referenceFrame.m_front = referenceFrame.m_up.CrossProduct(referenceFrame.m_right).Normalize();
			ndFixSizeArray<ndJointBilateralConstraint*, 64> extraJoint;
			return GetModel()->GetAsModelArticulation()->CalculateCentreOfMassDynamics(m_solver, referenceFrame, extraJoint,timestep);
		}

		ndFloat32 CalculateSupportDistance(const ndModelArticulation::ndCenterOfMassDynamics& comDynamics) const
		{
			ndFixSizeArray<ndVector, 16> supportPolygon;
			CaculateSupportPolygon(supportPolygon);
			// add a penalty for not not having a support polygon
			if (supportPolygon.GetCount() == 0)
			{
				return 0.0f;
			}
			if (supportPolygon.GetCount() == 1)
			{
				return ndBrainFloat(1.0f);
			}

			if (supportPolygon.GetCount() == 2)
			{
				ndFloat32 xAlpha = comDynamics.m_alpha.m_z / DEMO_GRAVITY;
				ndFloat32 zAlpha = -comDynamics.m_alpha.m_x / DEMO_GRAVITY;
				const ndVector surrogateLocalZmpPoint(xAlpha, ndFloat32(0.0f), zAlpha, ndFloat32(1.0f));
				ndVector scaledSurrogateLocalZmpPoint(surrogateLocalZmpPoint.Scale(ndFloat32(0.25f)));
				scaledSurrogateLocalZmpPoint.m_w = ndFloat32(1.0f);

				const ndVector surrogateZmpPoint(comDynamics.m_centerOfMass.TransformVector(scaledSurrogateLocalZmpPoint));
				ndBigVector ray_p0(surrogateZmpPoint);
				ndBigVector ray_p1(surrogateZmpPoint + ndVector(0.0f, -0.5f, 0.0f, 0.0f));

				ndBigVector ray_q0(supportPolygon[0]);
				ndBigVector ray_q1(supportPolygon[1]);

				ndBigVector p0;
				ndBigVector p1;
				ndRayToRayDistance(ray_p0, ray_p1, ray_q0, ray_q1, p0, p1);

				ndVector dist(p1 - p0);
				ndFloat32 dist2 = dist.DotProduct(dist).GetScalar();
				return ndSqrt(dist2);
			}
			else
			{
				ndFloat32 xAlpha = comDynamics.m_alpha.m_z / DEMO_GRAVITY;
				ndFloat32 zAlpha = -comDynamics.m_alpha.m_x / DEMO_GRAVITY;
				const ndVector surrogateLocalZmpPoint(xAlpha, ndFloat32(0.0f), zAlpha, ndFloat32(1.0f));
				ndVector scaledSurrogateLocalZmpPoint(surrogateLocalZmpPoint.Scale(ndFloat32(0.25f)));
				scaledSurrogateLocalZmpPoint.m_w = ndFloat32(1.0f);

				const ndVector surrogateZmpPoint(comDynamics.m_centerOfMass.TransformVector(scaledSurrogateLocalZmpPoint));
				ndBigVector ray_p0(surrogateZmpPoint);
				ndBigVector ray_p1(surrogateZmpPoint + ndVector(0.0f, -0.5f, 0.0f, 0.0f));

				ndFixSizeArray<ndBigVector, 16> polygon;
				for (ndInt32 i = 0; i < supportPolygon.GetCount(); ++i)
				{
					polygon.PushBack(supportPolygon[i]);
				}

				ndBigVector p0;
				ndBigVector p1;
				ndRayToPolygonDistance(ray_p0, ray_p1, &polygon[0], supportPolygon.GetCount(), p0, p1);

				ndVector dist(p1 - p0);
				ndFloat32 dist2 = dist.DotProduct(dist).GetScalar();
				return ndSqrt(dist2);
			}
		}

		ndFloat32 CalculateFloorDistance(const ndEffectorInfo& leg) const
		{
			ndWorld* const world = GetModel()->GetWorld();
			const ndRayCastFloor caster(world, leg);
			ndFloat32 hitDist = ndClamp(caster.m_param, ndFloat32(0.1284f), (D_CYCLE_AMPLITUDE));
			hitDist = (hitDist - ndFloat32(0.1284f)) / D_CYCLE_AMPLITUDE;
			return hitDist;
		}

		//#pragma optimize( "", off )
		ndBrainFloat CalculateReward() const
		{
			if (IsTerminal())
			{
				return ndBrainFloat (-50.0f);
			}

#if 0
			auto PolynomialOmegaReward = [](ndFloat32 omega, ndFloat32 maxValue)
			{
				omega = ndClamp(omega, -maxValue, maxValue);
				ndFloat32 r = ndFloat32(1.0f) - ndAbs(omega) / maxValue;
				ndFloat32 reward = r * r * r * r;
				return reward;
			};

			auto PolynomialAccelerationReward = [](ndFloat32 alpha, ndFloat32 maxValue)
			{
				alpha = ndClamp(alpha, -maxValue, maxValue);
				ndFloat32 r = ndFloat32(1.0f) - ndAbs(alpha) / maxValue;
				ndFloat32 reward = r * r;
				return reward;
			};

			// calculate a surrogate zero moment point
			const ndModelArticulation::ndCenterOfMassDynamics comDynamics(CalculateDynamics(m_timestep));

			ndFloat32 xAlpha = comDynamics.m_alpha.m_z / DEMO_GRAVITY;
			ndFloat32 zAlpha = -comDynamics.m_alpha.m_x / DEMO_GRAVITY;
			const ndVector surrogateLocalZmpPoint(xAlpha, ndFloat32(0.0f), zAlpha, ndFloat32(1.0f));
			ndVector scaledSurrogateLocalZmpPoint(surrogateLocalZmpPoint.Scale(ndFloat32 (0.25f)));
			scaledSurrogateLocalZmpPoint.m_w = ndFloat32 (1.0f);

			static float xxxxx = 0.0f;
			static float zzzzz = 0.0f;
			if ((ndAbs(scaledSurrogateLocalZmpPoint.m_x) > xxxxx) || (ndAbs(scaledSurrogateLocalZmpPoint.m_z) > zzzzz))
			{
				xxxxx = ndMax(xxxxx, ndAbs(scaledSurrogateLocalZmpPoint.m_x));
				zzzzz = ndMax(zzzzz, ndAbs(scaledSurrogateLocalZmpPoint.m_z));
				ndTrace(("zmp(%f %f)\n", xxxxx, zzzzz));
			}

			ndFloat32 dist = CalculateSupportDistance(comDynamics);
			if (dist < 0.001f)
			{
				return ndFloat32(1.0f);
			}

			ndFloat32 alphaReward_x = PolynomialAccelerationReward(scaledSurrogateLocalZmpPoint.m_x, 4.0f);
			ndFloat32 alphaReward_z = PolynomialAccelerationReward(scaledSurrogateLocalZmpPoint.m_z, 2.0f);
			return ndFloat32(0.5f) * (alphaReward_x + alphaReward_z);
#endif

			ndFloat32 contacSlideSpeed = 0.0f;
			for (ndInt32 i = 0; i < m_legs.GetCount(); ++i)
			{
				const ndEffectorInfo& leg = m_legs[i];
				ndIkSwivelPositionEffector* const effector = leg.m_effector;
				const ndContact* const contact = GetContact(effector);
				if (contact)
				{
					ndBodyKinematic* const body = effector->GetBody0();
					const ndContactPointList& contactPoints = contact->GetContactPoints();
					for (ndContactPointList::ndNode* contactPointsNode = contactPoints.GetFirst(); contactPointsNode; contactPointsNode = contactPointsNode->GetNext())
					{
						ndContactMaterial& contactPoint = contactPointsNode->GetInfo();
						ndVector contactVeloc(body->GetVelocityAtPoint(contactPoint.m_point));
						contactVeloc = contactPoint.m_normal.Scale(contactVeloc.DotProduct(contactPoint.m_normal).GetScalar());
						ndFloat32 hSpeed = ndSqrt(contactVeloc.DotProduct(contactVeloc).GetScalar());
						if (hSpeed > contacSlideSpeed)
						{
							contacSlideSpeed = hSpeed;
						}
					}
				}
			}

			auto SparseReward = [](const ndFloat32 x, ndFloat32 maxParam)
			{
				ndFloat32 param = ndClamp(x, -maxParam, maxParam) / maxParam;
				return ndExp(ndFloat32(-100.0f) * param * param);
			};

			auto ContacSlidingReward = [SparseReward](const ndFloat32 slideSpeed)
			{
				return SparseReward(slideSpeed, ndFloat32(2.0f));
			};

			auto CalculateSparceTiltReward = [SparseReward](const ndFloat32 sinAngle)
			{
				ndFloat32 angle = ndAsin(ndClamp(sinAngle, ndFloat32(-1.0f), ndFloat32(1.0f)));
				return SparseReward(angle, D_TILT_MAX_TILL_ANGLE);
			};

			const ndModelArticulation::ndNode* const rootNode = GetModel()->GetAsModelArticulation()->GetRoot();
			ndMatrix referenceFrame(rootNode->m_body->GetMatrix());
			referenceFrame.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
			referenceFrame.m_right = referenceFrame.m_front.CrossProduct(referenceFrame.m_up).Normalize();
			referenceFrame.m_front = referenceFrame.m_up.CrossProduct(referenceFrame.m_right).Normalize();

			const ndVector upDir (referenceFrame.UnrotateVector(rootNode->m_body->GetMatrix().m_up));

			if (upDir.m_y < ndCos(D_TILT_MAX_TILL_ANGLE))
			{
				// add a penalty for few frames before dying
				return ndBrainFloat(-5.0f);
			}

			//ndFloat32 xxx0 = CalculateSparceTiltReward(0.0f);
			//ndFloat32 xxx1 = CalculateSparceTiltReward(0.01f);
			//ndFloat32 xxx2 = CalculateSparceTiltReward(0.1f);
			//ndFloat32 xxx3 = CalculateSparceTiltReward(0.5f);

			ndFloat32 slideReward = ContacSlidingReward(contacSlideSpeed);
			ndFloat32 tiltReward_x = CalculateSparceTiltReward(upDir.m_x);
			ndFloat32 tiltReward_z = CalculateSparceTiltReward(upDir.m_z);

			return tiltReward_x * 0.3f + tiltReward_z * 0.3f + slideReward * 0.4f;
		}

		//#pragma optimize( "", off )
		void ResetModel()
		{
			ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
			for (ndInt32 i = 0; i < m_basePose.GetCount(); i++)
			{
				m_basePose[i].SetPose();
			}
			
			model->ClearMemory();
			
			ndAnimationSequencePlayer* const animPlayer = (ndAnimationSequencePlayer*)*m_poseGenerator;
			ndAnimationSequence* const sequence = *animPlayer->GetSequence();
			ndFloat32 duration = sequence->GetDuration();

			ndFloat32 startQuadrant = ndFloat32(ndRandInt() % 4);
			//startQuadrant = 0.0f;
			ndFloat32 startTime = duration * startQuadrant / ndFloat32 (4.0f);
			m_animBlendTree->SetTime(startTime);

			m_comX = ndFloat32(0.0f);
			m_comZ = ndFloat32(0.0f);

			ndVector veloc;
			m_animBlendTree->Evaluate(m_animPose1, veloc);
		}

		//#pragma optimize( "", off )
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

				ndFloat32 floorDistance = CalculateFloorDistance(leg);

				ndInt32 base = m_observationSize * i;
				observations[base + m_posePosit_x] = ndBrainFloat(keyFramePosit0.m_x);
				observations[base + m_posePosit_y] = ndBrainFloat(keyFramePosit0.m_y);
				observations[base + m_posePosit_z] = ndBrainFloat(keyFramePosit0.m_z);
				observations[base + m_poseVeloc_x] = ndBrainFloat((keyFramePosit1.m_x - keyFramePosit0.m_x) * invTimestep);
				observations[base + m_poseVeloc_y] = ndBrainFloat((keyFramePosit1.m_y - keyFramePosit0.m_y) * invTimestep);
				observations[base + m_poseVeloc_z] = ndBrainFloat((keyFramePosit1.m_z - keyFramePosit0.m_z) * invTimestep);

				observations[base + m_effectorPosit_x] = ndBrainFloat(effectorPosit.m_x);
				observations[base + m_effectorPosit_y] = ndBrainFloat(effectorPosit.m_y);
				observations[base + m_effectorPosit_z] = ndBrainFloat(effectorPosit.m_z);
				observations[base + m_effectorVeloc_x] = ndBrainFloat(effectorVeloc.m_x);
				observations[base + m_effectorVeloc_y] = ndBrainFloat(effectorVeloc.m_y);
				observations[base + m_effectorVeloc_z] = ndBrainFloat(effectorVeloc.m_z);

				observations[base + m_contactDistance] = ndBrainFloat(floorDistance);
			}

			observations[m_observationSize * m_legs.GetCount() + 0] = m_comX / D_CYCLE_STRIDE_X;
			observations[m_observationSize * m_legs.GetCount() + 1] = m_comZ / D_CYCLE_STRIDE_Z;
		}

		void ApplyActions(ndBrainFloat* const actions)
		{
			ndBodyKinematic* const rootBody = GetModel()->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();

			const ndVector upVector(rootBody->GetMatrix().m_up);

			ndFloat32 y = ndFloat32(0.0f);
			m_comX = ndClamp(m_comX + actions[m_actionPosit_x] * D_ACTION_SENSITIVITY, -ndFloat32(0.25f) * D_CYCLE_STRIDE_X, ndFloat32(0.25f) * D_CYCLE_STRIDE_X);
			m_comZ = ndClamp(m_comZ + actions[m_actionPosit_z] * D_ACTION_SENSITIVITY, -ndFloat32(0.25f) * D_CYCLE_STRIDE_Z, ndFloat32(0.25f) * D_CYCLE_STRIDE_Z);

			//x = ndFloat32(0.25f) * 1.0f * D_CYCLE_STRIDE_X;
			//z = ndFloat32(0.25f) * 1.0f * D_CYCLE_STRIDE_Z;
			for (ndInt32 i = 0; i < m_legs.GetCount(); ++i)
			{
				ndEffectorInfo& leg = m_legs[i];
				ndIkSwivelPositionEffector* const effector = leg.m_effector;
				
				const ndAnimKeyframe keyFrame = m_animPose0[i];
				ndVector posit(keyFrame.m_posit);
				posit.m_x += m_comX;
				posit.m_y += y;
				posit.m_z += m_comZ;
			
				ndFloat32 swivelAngle = effector->CalculateLookAtSwivelAngle(upVector);

				ndFloat32 minAngle;
				ndFloat32 maxAngle;
				ndFloat32 kneeAngle = leg.m_calf->GetAngle();
				leg.m_calf->GetLimits(minAngle, maxAngle);
				ndFloat32 safeGuardAngle = ndFloat32(3.0f * ndDegreeToRad);
				maxAngle = ndMax(ndFloat32(0.0f), maxAngle - safeGuardAngle);
				minAngle = ndMin(ndFloat32(0.0f), minAngle + safeGuardAngle);
				if ((kneeAngle > maxAngle) || (kneeAngle < minAngle))
				{
					// project that target to the sphere of the current position
					leg.m_effector->SetAsReducedDof();
				}

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

		const ndContact* GetContact (ndIkSwivelPositionEffector* const effector) const
		{
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

		void CaculateSupportPolygon(ndFixSizeArray<ndVector, 16>& supportPolygon) const
		{
			supportPolygon.SetCount(0);
			ndFixSizeArray<ndBigVector, 16> supportPoints;
			for (ndInt32 i = 0; i < m_legs.GetCount(); ++i)
			{
				const ndEffectorInfo& leg = m_legs[i];
				ndIkSwivelPositionEffector* const effector = leg.m_effector;

				//auto HasContact = [effector]()
				//{
				//	ndBodyKinematic* const body = effector->GetBody0();
				//	ndBodyKinematic::ndContactMap& contacts = body->GetContactMap();
				//	ndBodyKinematic::ndContactMap::Iterator it(contacts);
				//	for (it.Begin(); it; it++)
				//	{
				//		ndContact* const contact = *it;
				//		if (contact->IsActive())
				//		{
				//			return true;
				//		}
				//	}
				//	return false;
				//};

				if (GetContact(effector))
				{
					supportPoints.PushBack(effector->CalculateGlobalMatrix0().m_posit);
				}
			}

			switch (supportPoints.GetCount())
			{
				case 1:
					supportPolygon.PushBack(supportPoints[0]);
					break;

				case 2:
					supportPolygon.PushBack(supportPoints[0]);
					supportPolygon.PushBack(supportPoints[1]);
					break;

				case 3:
				case 4:
				{
					ndBigVector origin(ndBigVector::m_zero);
					for (ndInt32 i = 0; i < supportPoints.GetCount(); ++i)
					{
						origin += supportPoints[i];
					}
					origin = origin.Scale(1.0f / ndFloat32(supportPoints.GetCount()));

					ndFloat32 scale = 1.0f;
					for (ndInt32 i = 0; i < supportPoints.GetCount(); ++i)
					{
						supportPoints[i] = origin + (supportPoints[i] - origin).Scale(scale);
					}

					for (ndInt32 i = 0; i < supportPoints.GetCount(); ++i)
					{
						supportPolygon.PushBack(supportPoints[i]);
					}

					ndMatrix rotation(ndPitchMatrix(90.0f * ndDegreeToRad));
					rotation.TransformTriplex(&supportPolygon[0].m_x, sizeof(ndVector), &supportPolygon[0].m_x, sizeof(ndVector), supportPolygon.GetCount());
					ndInt32 supportCount = ndConvexHull2d(&supportPolygon[0], supportPolygon.GetCount());
					rotation.OrthoInverse().TransformTriplex(&supportPolygon[0].m_x, sizeof(ndVector), &supportPolygon[0].m_x, sizeof(ndVector), supportCount);
					supportPolygon.SetCount(supportCount);
				}
				default:;
			}
		}

		virtual void Debug(ndConstraintDebugCallback& context) const
		{
			if (m_modelEnum != 0)
			{
				return;
			}

			ndVector blackColor(ndVector::m_wOne);
			for (ndInt32 i = 0; i < m_legs.GetCount(); ++i)
			{
				const ndEffectorInfo& leg = m_legs[i];
				//leg.m_heel->DebugJoint(context);
				//leg.m_effector->DebugJoint(context);
				ndFloat32 floorDistance = CalculateFloorDistance(leg);
				ndVector origin (leg.m_effector->CalculateGlobalMatrix0().m_posit);
				ndVector dest(origin);
				dest.m_y -= floorDistance * D_CYCLE_AMPLITUDE;
				context.DrawLine(origin, dest, blackColor);
			}

			ndFixSizeArray<ndVector, 16> supportPolygon;
			CaculateSupportPolygon(supportPolygon);

			ndVector supportColor(0.0f, 1.0f, 1.0f, 1.0f);
			if (supportPolygon.GetCount() > 1)
			{
				ndInt32 i0 = supportPolygon.GetCount() - 1;
				for (ndInt32 i1 = 0; i1 < supportPolygon.GetCount(); ++i1)
				{
					context.DrawLine(supportPolygon[i0], supportPolygon[i1], supportColor);
					i0 = i1;
				}
			}

			// calculate zmp
			ndModelArticulation::ndCenterOfMassDynamics dynamics(CalculateDynamics(m_timestep));

			ndFloat32 dist = CalculateSupportDistance(dynamics);
			if (dist < 0.001f)
			{
				dist *= 1;
			}

			// draw center of mass support defined as:
			// a point where a vertical line draw from the center of mass, intersect the support polygon plane.
			ndMatrix centerOfPresure(dynamics.m_centerOfMass);
			centerOfPresure.m_posit.m_y -= 0.28f;
			context.DrawPoint(centerOfPresure.m_posit, ndVector(0.0f, 0.0f, 1.0f, 1.0f), 4);

			// draw zero moment point define as: 
			// a point on the support polygon plane where a vertical 
			// force make the horizontal components of the com acceleration zero.
			ndFloat32 gravityForce = dynamics.m_mass * DEMO_GRAVITY + 0.001f;
			ndFloat32 x = dynamics.m_torque.m_z / gravityForce;
			ndFloat32 z = -dynamics.m_torque.m_x / gravityForce;
			const ndVector localZmp(x, ndFloat32(0.0f), z, ndFloat32(1.0f));
			ndVector scaledLocalZmp(localZmp.Scale(10.0f));
			scaledLocalZmp.m_w = ndFloat32(1.0f);
			const ndVector zmp(centerOfPresure.TransformVector(scaledLocalZmp));
			context.DrawPoint(zmp, ndVector(1.0f, 0.0f, 0.0f, 1.0f), 4);

			//// draw zero moment point surrogate point, define as: 
			//// a point propertinal to the center of mass angular acceleration
			//// projected onto the support polygon plane.
			//ndFloat32 xAlpha = dynamics.m_alpha.m_z / gravityForce;
			//ndFloat32 zAlpha = -dynamics.m_alpha.m_x / gravityForce;
			//const ndVector surrogateLocalZmpPoint(xAlpha, ndFloat32(0.0f), zAlpha, ndFloat32(1.0f));
			//ndVector scaledSurrogateLocalZmpPoint(surrogateLocalZmpPoint.Scale(5.0f));
			//scaledSurrogateLocalZmpPoint.m_w = ndFloat32(1.0f);
			//const ndVector surrogateZmpPoint(centerOfPresure.TransformVector(scaledSurrogateLocalZmpPoint));
			//context.DrawPoint(surrogateZmpPoint, ndVector(1.0f, 1.0f, 0.0f, 1.0f), 4);
		}

		void Update(ndFloat32 timestep)
		{
			m_timestep = timestep;

			ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
			ndBodyKinematic* const rootBody = model->GetRoot()->m_body->GetAsBodyKinematic();
			rootBody->SetSleepState(false);

			//ndFloat32 animSpeed = 2.0f * m_control->m_animSpeed;
			ndFloat32 animSpeed = 0.5f;
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
				m_controller->Step();
			}
		}

		mutable ndIkSolver m_solver;
		ndAnimationPose m_animPose0;
		ndAnimationPose m_animPose1;
		ndSharedPtr<ndAnimationBlendTreeNode> m_poseGenerator;
		ndSharedPtr<ndAnimationBlendTreeNode> m_animBlendTree;
		ndFixSizeArray<ndBasePose, 32> m_basePose;
		ndFixSizeArray<ndEffectorInfo, 4> m_legs;
		ndSharedPtr<ndController> m_controller;
		ndSharedPtr<ndControllerTrainer> m_controllerTrainer;

		ndFloat32 m_comX;
		ndFloat32 m_comZ;
		ndFloat32 m_timestep;
		ndInt32 m_modelEnum;
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

		// offset com so that the model is unstable
		ndVector com(rootBody->GetAsBodyKinematic()->GetCentreOfMass());
		//com.m_x -= 0.05f;
		com.m_x += 0.00f;
		rootBody->GetAsBodyKinematic()->SetCentreOfMass(com);

		// build all for legs
		ndModelArticulation::ndNode* const modelRootNode = model->AddRootBody(rootBody);
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
			((ndIkJointHinge*)*calfHinge)->SetLimits(-60.0f * ndDegreeToRad, 50.0f * ndDegreeToRad);

			// build heel
			ndSharedPtr<ndDemoEntity> heelEntity(calfEntity->GetChildren().GetFirst()->GetInfo());
			const ndMatrix heelMatrix(heelEntity->GetCurrentMatrix() * calfMatrix);
			ndSharedPtr<ndBody> heel(CreateRigidBody(heelEntity, heelMatrix, limbMass, calf->GetAsBodyDynamic()));

			ndSharedPtr<ndJointBilateralConstraint> heelHinge(new ndJointHinge(heelMatrix, heel->GetAsBodyKinematic(), calf->GetAsBodyKinematic()));
			ndModelArticulation::ndNode* const heelNode = model->AddLimb(calfNode, heel, heelHinge);
			((ndJointHinge*)*heelHinge)->SetAsSpringDamper(0.001f, 2000.0f, 50.0f);

			// build soft contact heel
			ndSharedPtr<ndDemoEntity> contactEntity(heelEntity->GetChildren().GetFirst()->GetInfo());
			const ndMatrix contactMatrix(contactEntity->GetCurrentMatrix() * heelMatrix);
			ndSharedPtr<ndBody> contact(CreateRigidBody(contactEntity, contactMatrix, limbMass * 0.5f, heel->GetAsBodyDynamic()));

			const ndMatrix contactAxis(ndRollMatrix(ndFloat32(90.0f) * ndDegreeToRad) * contactMatrix);
			ndSharedPtr<ndJointBilateralConstraint> softContact(new ndJointSlider(contactAxis, contact->GetAsBodyKinematic(), heel->GetAsBodyKinematic()));
			model->AddLimb(heelNode, contact, softContact);
			((ndJointSlider*)*softContact)->SetAsSpringDamper(0.01f, 2000.0f, 10.0f);

			// create effector
			ndSharedPtr<ndDemoEntity> footEntity(contactEntity->GetChildren().GetFirst()->GetInfo());
			ndMatrix footMatrix(matrix);
			footMatrix.m_posit = (footEntity->GetCurrentMatrix() * contactMatrix).m_posit;

			ndMatrix effectorRefFrame(footMatrix);
			effectorRefFrame.m_posit = thighMatrix.m_posit;

			ndFloat32 regularizer = 0.001f;
			ndFloat32 effectorStrength = 50000.0f;
			ndSharedPtr<ndJointBilateralConstraint> effector (new ndIkSwivelPositionEffector(effectorRefFrame, rootBody->GetAsBodyKinematic(), footMatrix.m_posit, contact->GetAsBodyKinematic()));
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
			leg.m_softContact = (ndJointSlider*)*softContact;
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
			,m_stopTraining(0)
			,m_modelIsTrained(false)
		{
			ndWorld* const world = scene->GetWorld();

			ndInt32 hiddenLayers = 4;
			ndInt32 hiddenLayersNeurons = 64;
			ndInt32 numberOfActions = m_actionsSize;
			ndInt32 numberOfObservations = m_observationSize * 4 + 2;

			#ifdef USE_DDPG
				m_outFile = fopen("ndQuadruped_2-sac.csv", "wb");
				fprintf(m_outFile, "sac\n");

				m_stopTraining = 1000000;
				ndBrainAgentOffPolicyGradient_Trainer::HyperParameters hyperParameters;

				//hyperParameters.m_usePerActionSigmas = true;
				hyperParameters.m_numberOfActions = numberOfActions;
				hyperParameters.m_numberOfObservations = numberOfObservations;
				hyperParameters.m_numberOfHiddenLayers = hiddenLayers;
				hyperParameters.m_hiddenLayersNumberOfNeurons = hiddenLayersNeurons;
				m_master = ndSharedPtr<ndBrainAgentOffPolicyGradient_Trainer>(new ndBrainAgentOffPolicyGradient_Trainer(hyperParameters));
			#else
				m_outFile = fopen("ndQuadruped_2-ppo.csv", "wb");
				fprintf(m_outFile, "ppo\n");

				m_stopTraining = 500 * 1000000;
				ndBrainAgentContinuePolicyGradient_TrainerMaster::HyperParameters hyperParameters;

				//hyperParameters.m_usePerActionSigmas = false;
				hyperParameters.m_numberOfActions = numberOfActions;
				hyperParameters.m_numberOfObservations = numberOfObservations;
				hyperParameters.m_numberOfHiddenLayers = hiddenLayers;
				hyperParameters.m_hiddenLayersNumberOfNeurons = hiddenLayersNeurons;
				m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>(new ndBrainAgentContinuePolicyGradient_TrainerMaster(hyperParameters));
			#endif

			m_bestActor = ndSharedPtr<ndBrain>(new ndBrain(*m_master->GetPolicyNetwork()));

			m_master->SetName(CONTROLLER_NAME);

			ndSharedPtr<ndModel>visualModel(CreateModel(scene, matrix, modelMesh));
			RobotModelNotify* const notify = (RobotModelNotify*)*visualModel->GetAsModel()->GetNotifyCallback();
			notify->SetControllerTrainer(m_master);
			notify->m_modelEnum = 0;
			
			SetMaterial(visualModel->GetAsModelArticulation());
			world->AddModel(visualModel);
			visualModel->AddBodiesAndJointsToWorld();

			ndBodyKinematic* const rootBody = visualModel->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
			ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(rootBody->GetMatrix(), rootBody, world->GetSentinelBody()));
			//world->AddJoint(fixJoint);

			// add a hidden battery of model to generate trajectories in parallel
#ifndef USE_DDPG
			ndInt32 countX = 10;
			ndInt32 countZ = 10;
			//countX = 0;
			//countZ = 0;
			
			ndInt32 enumeration = 1;
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
					notify1->m_modelEnum = enumeration;
					enumeration++;

					SetMaterial(model->GetAsModelArticulation());
					world->AddModel(model);
					model->AddBodiesAndJointsToWorld();

					m_models.Append(model->GetAsModelArticulation());

					ndBodyKinematic* const rootBody1 = model->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic();
					ndSharedPtr<ndJointBilateralConstraint> fixJoint1(new ndJointFix6dof(rootBody1->GetMatrix(), rootBody1, world->GetSentinelBody()));
					//world->AddJoint(fixJoint1);
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

				if (!m_master->IsSampling())
				{
					if (rewardTrajectory >= ndFloat32(m_maxScore))
					{
						if (m_lastEpisode != m_master->GetEposideCount())
						{
							m_maxScore = rewardTrajectory;
							m_bestActor->CopyFrom(*m_master->GetPolicyNetwork());
							ndExpandTraceMessage("   best actor episode: %d\treward %f\ttrajectoryFrames: %f\n", m_master->GetEposideCount(), 100.0f * m_master->GetAverageScore() / m_horizon, m_master->GetAverageFrames());
							m_lastEpisode = m_master->GetEposideCount();
						}
					}

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
			}
			
			ndFloat32 stopScore = 100.0f * ndFloat32(m_master->GetAverageFrames() * m_master->GetAverageScore()) / m_horizon;
			if ((stopTraining >= m_stopTraining) || (stopScore > 95.0f * ndFloat32(m_master->m_parameters.m_maxTrajectorySteps)))
			{
				char fileName[1024];
				m_modelIsTrained = true;
				ndGetWorkingFileName(m_master->GetName().GetStr(), fileName);
				//m_master->GetPolicyNetwork()->CopyFrom(*(*m_bestActor));
				//m_master->GetPolicyNetwork()->SaveToFile(fileName);
				m_bestActor->SaveToFile(fileName);
				ndExpandTraceMessage("saving to file: %s\n", fileName);
				ndExpandTraceMessage("training complete\n");
				ndUnsigned64 timer = ndGetTimeInMicroseconds() - m_timer;
				ndExpandTraceMessage("training time: %g seconds\n", ndFloat32(ndFloat64(timer) * ndFloat32(1.0e-6f)));
				manager->Terminate();
			}
		}

#ifdef USE_DDPG
		ndSharedPtr<ndBrainAgentOffPolicyGradient_Trainer> m_master;
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
	matrix.m_posit.m_y = 0.4f;

	#ifdef ND_TRAIN_MODEL
		scene->RegisterPostUpdate(new TrainingUpdata(scene, matrix, modelMesh));
	#else
		ndWorld* const world = scene->GetWorld();

		char fileName[256];
		ndGetWorkingFileName(CONTROLLER_NAME, fileName);
		ndSharedPtr<ndBrain> policy(ndBrainLoad::Load(fileName));

		ndSharedPtr<ndModel> referenceModel (CreateModel(scene, matrix, modelMesh));
		ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(referenceModel->GetAsModelArticulation()->GetRoot()->m_body->GetMatrix(), referenceModel->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic(), world->GetSentinelBody()));
		//world->AddJoint(fixJoint);
	
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
