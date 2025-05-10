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

// this model just plays the animation by injecting the poses to the end effectors
namespace ndQuadruped_1
{
	#define D_CYCLE_PERIOD		ndFloat32(4.0f)
	#define D_CYCLE_STRIDE_X	ndFloat32(0.3f)
	#define D_CYCLE_STRIDE_Z	ndFloat32(0.3f)
	#define D_CYCLE_AMPLITUDE	ndFloat32(0.27f)
	#define D_POSE_REST_POSITION_Y	ndReal(-0.3f)

	class RobotModelNotify : public ndModelNotify
	{
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

		class ndPoseGenerator : public ndAnimationSequence
		{
			public:
			ndPoseGenerator()
				:ndAnimationSequence()
				,m_amp(D_CYCLE_AMPLITUDE)
				,m_stride_x(D_CYCLE_STRIDE_X)
				,m_stride_z(D_CYCLE_STRIDE_Z)
			{
				m_duration = ndFloat32(4.0f);
				for (ndInt32 i = 0; i < 4; i++)
				{
					//m_currentPose[i] = ndVector::m_zero;
				}
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
					//break;
				}
			}

			ndFloat32 m_amp;
			ndFloat32 m_stride_x;
			ndFloat32 m_stride_z;
		};

		void Init()
		{
			ndSharedPtr<ndAnimationSequence> sequence(new ndPoseGenerator());
			
			m_poseGenerator = ndSharedPtr<ndAnimationBlendTreeNode>(new ndAnimationSequencePlayer(sequence));
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
				m_animPose.PushBack(keyFrame);
				poseGenerator->AddTrack();
			}
		}

		RobotModelNotify(ndModelArticulation* const robot)
			:ndModelNotify()
		{
			SetModel(robot);
		}

		void PostUpdate(ndFloat32)
		{
		}

		void PostTransformUpdate(ndFloat32)
		{
		}

		ndVector CalculateZeroMomentPoint(ndFloat32 timestep)
		{
			ndModelArticulation* const articulation = GetModel()->GetAsModelArticulation();
			ndBodyKinematic* const rootBody = articulation->GetRoot()->m_body->GetAsBodyKinematic();
			ndSkeletonContainer* const skeleton = rootBody->GetSkeleton();
			ndAssert(skeleton);

			m_solver.SolverBegin(skeleton, nullptr, 0, articulation->GetWorld(), timestep);
			m_solver.Solve();

			ndFixSizeArray<const ndBodyKinematic*, 32> bodies;
			auto CalculateReferenceFrame = [this, &bodies]()
			{
				ndVector com(ndVector::m_zero);
				ndFloat32 totalMass = ndFloat32(0.0f);
				ndModelArticulation::ndNode* const rootNode = GetModel()->GetAsModelArticulation()->GetRoot();
				for (ndModelArticulation::ndNode* iter = rootNode->GetFirstIterator(); iter; iter = iter->GetNextIterator())
				{
					const ndBodyKinematic* const body = iter->m_body->GetAsBodyKinematic();
					bodies.PushBack(body);
					const ndMatrix matrix(body->GetMatrix());
					const ndVector bodyCom(matrix.TransformVector(body->GetCentreOfMass()));
					ndFloat32 mass = body->GetMassMatrix().m_w;
					totalMass += mass;
					com += bodyCom.Scale(mass);
				}
			
				com = com.Scale(1.0f / totalMass);
				ndMatrix referenceFrame(rootNode->m_body->GetMatrix());
				referenceFrame.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
				referenceFrame.m_right = referenceFrame.m_front.CrossProduct(referenceFrame.m_up).Normalize();
				referenceFrame.m_front = referenceFrame.m_up.CrossProduct(referenceFrame.m_right).Normalize();
				referenceFrame.m_posit = com;
				referenceFrame.m_posit.m_w = 1.0f;
				return referenceFrame;
			};
			const ndMatrix comFrame(CalculateReferenceFrame());
			
			auto CalculateZmp = [this, &bodies, &comFrame]()
			{
				ndFloat32 totalMass = ndFloat32(0.0f);
				ndVector totalToque(ndVector::m_zero);
				const ndMatrix invComFrame(comFrame.OrthoInverse());
				for (ndInt32 i = bodies.GetCount() - 1; i >= 0; --i)
				{
					const ndBodyKinematic* const body = bodies[i];
					const ndMatrix bodyMatrix(body->GetMatrix());
				
					const ndVector origin(bodyMatrix.TransformVector(body->GetCentreOfMass()));
					const ndVector bodyCom(comFrame.UntransformVector(origin) & ndVector::m_triplexMask);
				
					ndFloat32 mass = body->GetMassMatrix().m_w;
				
					totalMass += mass;
					const ndMatrix bodyInertia(body->CalculateInertiaMatrix());
					const ndVector extForceTorque(bodyCom.CrossProduct(invComFrame.RotateVector(body->GetAccel().Scale(mass))));
					const ndVector extTorque(invComFrame.RotateVector(bodyInertia.RotateVector(body->GetAlpha())));
					const ndVector gyroTorque(invComFrame.RotateVector(body->GetOmega().CrossProduct(bodyInertia.RotateVector(body->GetOmega()))));
				
					totalToque += extTorque;
					totalToque += gyroTorque;
					totalToque += extForceTorque;
				}
				ndFloat32 gravityForce = totalMass * DEMO_GRAVITY + 0.1f;
				ndFloat32 x = totalToque.m_z / gravityForce;
				ndFloat32 z = -totalToque.m_x / gravityForce;
				const ndVector zmp(x, ndFloat32 (0.0f), z, ndFloat32(0.0f));
				return zmp.Scale (ndFloat32 (-0.1f));
			};
			const ndVector zmp(CalculateZmp());
			//ndTrace(("%f %f %f\n", zmp.m_x, zmp.m_y, zmp.m_z));
			m_solver.SolverEnd();
			
			return zmp;
		}

		ndJacobian CalculateDynamics(ndFloat32 timestep)
		{
			auto CalculateReferenceFrame = [this]()
			{
				ndVector com(ndVector::m_zero);
				ndFloat32 totalMass = ndFloat32(0.0f);
				ndModelArticulation::ndNode* const rootNode = GetModel()->GetAsModelArticulation()->GetRoot();
				for (ndModelArticulation::ndNode* iter = rootNode->GetFirstIterator(); iter; iter = iter->GetNextIterator())
				{
					const ndBodyKinematic* const body = iter->m_body->GetAsBodyKinematic();
					const ndMatrix matrix(body->GetMatrix());
					const ndVector bodyCom(matrix.TransformVector(body->GetCentreOfMass()));
					ndFloat32 mass = body->GetMassMatrix().m_w;
					totalMass += mass;
					com += bodyCom.Scale(mass);
				}

				com = com.Scale(1.0f / totalMass);
				ndMatrix referenceFrame(rootNode->m_body->GetMatrix());
				referenceFrame.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
				referenceFrame.m_right = referenceFrame.m_front.CrossProduct(referenceFrame.m_up).Normalize();
				referenceFrame.m_front = referenceFrame.m_up.CrossProduct(referenceFrame.m_right).Normalize();
				referenceFrame.m_posit = com;
				referenceFrame.m_posit.m_w = 1.0f;
				return referenceFrame;
			};
			const ndMatrix comFrame(CalculateReferenceFrame());
			return GetModel()->GetAsModelArticulation()->CalculateLocalAcceleration(m_solver, comFrame, timestep);
		}

		void Update(ndFloat32 timestep)
		{
			ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
			ndBodyKinematic* const rootBody = model->GetRoot()->m_body->GetAsBodyKinematic();
			rootBody->SetSleepState(false);

			//ndJacobian xxxx(model->CalculateTotalMomentum());
			//const ndVector zmp(CalculateZeroMomentPoint(timestep));
			//const ndVector zmp(ndVector::m_zero);
			const ndJacobian comOmegaAlpha(CalculateDynamics(timestep));
			const ndVector comOmega(comOmegaAlpha.m_linear);
			const ndVector comAlpha(comOmegaAlpha.m_angular);

			//ndFloat32 animSpeed = 2.0f * m_control->m_animSpeed;
			ndFloat32 animSpeed = 0.25f;
			m_animBlendTree->Update(timestep * animSpeed);

			ndVector veloc;
			m_animBlendTree->Evaluate(m_animPose, veloc);

			const ndVector upVector(rootBody->GetMatrix().m_up);
			for (ndInt32 i = 0; i < m_legs.GetCount(); ++ i)
			{
				ndEffectorInfo& leg = m_legs[i];
				ndIkSwivelPositionEffector* const effector = leg.m_effector;
				
				//ndVector posit(m_animPose[i].m_posit - zmp);
				ndVector posit(m_animPose[i].m_posit);
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
					// project that target to the sphere of the corrent position
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

		virtual void Debug(ndConstraintDebugCallback& context) const
		{
			ndFloat32 scale = context.GetScale();
			context.SetScale(scale * 0.75f);
			for (ndInt32 i = 0; i < m_legs.GetCount(); ++i)
			{
				const ndEffectorInfo& leg = m_legs[i];
				//leg.m_heel->DebugJoint(context);
				//leg.m_effector->DebugJoint(context);
				leg.m_calf->DebugJoint(context);
			}
			context.SetScale(scale);
		}

		ndIkSolver m_solver;
		ndAnimationPose m_animPose;
		ndFixSizeArray<ndEffectorInfo, 4> m_legs;
		ndSharedPtr<ndAnimationBlendTreeNode> m_poseGenerator;
		ndSharedPtr<ndAnimationBlendTreeNode> m_animBlendTree;
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
		ndFloat32 limbMass = 0.5f;
		ndMatrix matrix(entity->GetCurrentMatrix() * location);

		ndSharedPtr<ndBody> rootBody(CreateRigidBody(entity, matrix, mass, nullptr));
		ndModelArticulation::ndNode* const modelRootNode = model->AddRootBody(rootBody);
	
		// build all for legs
		for (ndList<ndSharedPtr<ndDemoEntity>>::ndNode* node = entity->GetChildren().GetFirst(); node; node = node->GetNext())
		{
			// build whig
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
			((ndIkJointHinge*)*calfHinge)->SetLimits(-50.0f * ndDegreeToRad, 40.0f * ndDegreeToRad);

			// build heel
			ndSharedPtr<ndDemoEntity> heelEntity(calfEntity->GetChildren().GetFirst()->GetInfo());
			const ndMatrix heelMatrix(heelEntity->GetCurrentMatrix() * calfMatrix);
			ndSharedPtr<ndBody> heel(CreateRigidBody(heelEntity, heelMatrix, limbMass, calf->GetAsBodyDynamic()));

			ndSharedPtr<ndJointBilateralConstraint> heelHinge(new ndJointHinge(heelMatrix, heel->GetAsBodyKinematic(), calf->GetAsBodyKinematic()));
			model->AddLimb(calfNode, heel, heelHinge);
			((ndJointHinge*)*heelHinge)->SetAsSpringDamper(0.001f, 2000.0f, 50.0f);

			// create effector
			ndSharedPtr<ndDemoEntity> footEntity(heelEntity->GetChildren().GetFirst()->GetInfo());
			ndMatrix footMatrix(matrix);
			footMatrix.m_posit = (footEntity->GetCurrentMatrix() * heelMatrix).m_posit;

			ndMatrix effectorRefFrame(footMatrix);
			effectorRefFrame.m_posit = thighMatrix.m_posit;

			ndFloat32 regularizer = 0.001f;
			ndFloat32 effectorStrength = 20.0f * 10.0f * 500.0f;
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
		notify->Init();
		return model;
	}
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

	ndMeshLoader loader;
	ndSharedPtr<ndDemoEntity> modelMesh(loader.LoadEntity("quadrupeSpider.fbx", scene));

	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit.m_y = 0.4f;

	//matrix.m_posit.m_y += 5.0f;
	//matrix.m_posit.m_y = 1.0f;
	ndWorld* const world = scene->GetWorld();

	ndSharedPtr<ndModel> referenceModel (CreateModel(scene, matrix, modelMesh));
	world->AddModel(referenceModel);
	referenceModel->AddBodiesAndJointsToWorld();
		
	ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(referenceModel->GetAsModelArticulation()->GetRoot()->m_body->GetMatrix(), referenceModel->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic(), world->GetSentinelBody()));
	//world->AddJoint(fixJoint);
	
	matrix.m_posit.m_x -= 4.0f;
	matrix.m_posit.m_y += 1.0f;
	matrix.m_posit.m_z += 0.25f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
