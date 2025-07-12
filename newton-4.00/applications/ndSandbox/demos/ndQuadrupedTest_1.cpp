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
	#define D_CYCLE_STRIDE_X	ndFloat32(0.4f)
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
				}
			}

			ndFloat32 m_amp;
			ndFloat32 m_stride_x;
			ndFloat32 m_stride_z;
		};

		void InitAnimation()
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
			,m_solver()
			,m_animPose()
			,m_legs()
			,m_poseGenerator()
			,m_animBlendTree()
			,m_timestep(ndFloat32 (0.0f))
		{
			SetModel(robot);
		}

		void PostUpdate(ndFloat32)
		{
		}

		void PostTransformUpdate(ndFloat32)
		{
		}

		ndModelArticulation::ndCenterOfMassDynamics CalculateDynamics(ndFloat32 timestep) const
		{
			ndModelArticulation::ndNode* const rootNode = GetModel()->GetAsModelArticulation()->GetRoot();

			ndMatrix referenceFrame(rootNode->m_body->GetMatrix());
			referenceFrame.m_up = ndVector(0.0f, 1.0f, 0.0f, 0.0f);
			referenceFrame.m_right = referenceFrame.m_front.CrossProduct(referenceFrame.m_up).Normalize();
			referenceFrame.m_front = referenceFrame.m_up.CrossProduct(referenceFrame.m_right).Normalize();

			ndFixSizeArray<ndJointBilateralConstraint*, 64> extraJoint;
			return GetModel()->GetAsModelArticulation()->CalculateCentreOfMassDynamics(m_solver, referenceFrame, extraJoint, timestep);
		}

		void Update(ndFloat32 timestep)
		{
			m_timestep = timestep;

			ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
			ndBodyKinematic* const rootBody = model->GetRoot()->m_body->GetAsBodyKinematic();
			rootBody->SetSleepState(false);

			const ndModelArticulation::ndCenterOfMassDynamics comDynamics(CalculateDynamics(timestep));
			const ndVector comOmega(comDynamics.m_omega);
			const ndVector comAlpha(comDynamics.m_alpha);

			//ndFloat32 animSpeed = 2.0f * m_control->m_animSpeed;
			ndFloat32 animSpeed = 0.5f;
			m_animBlendTree->Update(timestep * animSpeed);

			ndVector veloc;
			m_animBlendTree->Evaluate(m_animPose, veloc);

			const ndVector upVector(rootBody->GetMatrix().m_up);
			for (ndInt32 i = 0; i < m_legs.GetCount(); ++ i)
			{
				ndEffectorInfo& leg = m_legs[i];
				ndIkSwivelPositionEffector* const effector = leg.m_effector;
				
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

		void CaculateSupportPolygon(ndFixSizeArray<ndVector, 16>& supportPolygon) const
		{
			supportPolygon.SetCount(0);
			ndFixSizeArray<ndBigVector, 16> supportPoints;
			for (ndInt32 i = 0; i < m_legs.GetCount(); ++i)
			{
				const ndEffectorInfo& leg = m_legs[i];
				ndIkSwivelPositionEffector* const effector = leg.m_effector;

				auto HasContact = [effector]()
				{
					ndBodyKinematic* const body = effector->GetBody0();
					ndBodyKinematic::ndContactMap& contacts = body->GetContactMap();
					ndBodyKinematic::ndContactMap::Iterator it(contacts);
					for (it.Begin(); it; it++)
					{
						ndContact* const contact = *it;
						if (contact->IsActive())
						{
							return true;
						}
					}
					return false;
				};

				if (HasContact())
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

					//ndFixSizeArray<ndVector, 16> desiredSupportPoint;
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
			for (ndInt32 i = 0; i < m_legs.GetCount(); ++i)
			{
				ndFloat32 scale = context.GetScale();
				context.SetScale(scale * 0.75f);
				//const ndEffectorInfo& leg = m_legs[i];
				//leg.m_heel->DebugJoint(context);
				//leg.m_effector->DebugJoint(context);
				//leg.m_calf->DebugJoint(context);
				context.SetScale(scale);
			}

			ndVector supportColor(0.0f, 1.0f, 1.0f, 1.0f);

			// draw support feature ( a point, line or a polygon.
			ndFixSizeArray<ndVector, 16> supportPolygon;
			CaculateSupportPolygon(supportPolygon);

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

			// draw center of pressure define as
			// a point where a vertical line draw from the center of mass, intersect the support polygon plane.
			ndMatrix centerOfPresure(dynamics.m_centerOfMass);
			centerOfPresure.m_posit.m_y -= 0.28f;
			context.DrawPoint(centerOfPresure.m_posit, ndVector(0.0f, 0.0f, 1.0f, 1.0f), 4);

			// draw zero moment point define as: 
			// a point on the support polygon plane where a vertical force make the horizontal components of the com acceleration zero.
			ndFloat32 gravityForce = dynamics.m_mass * DEMO_GRAVITY + 0.001f;
			ndFloat32 x = dynamics.m_torque.m_z / gravityForce;
			ndFloat32 z = -dynamics.m_torque.m_x / gravityForce;
			const ndVector localZmp(x, ndFloat32(0.0f), z, ndFloat32(1.0f));
			ndVector scaledLocalZmp(localZmp.Scale(10.0f));
			scaledLocalZmp.m_w = ndFloat32 (1.0f);
			const ndVector zmp(centerOfPresure.TransformVector(scaledLocalZmp));
			context.DrawPoint(zmp, ndVector(1.0f, 0.0f, 0.0f, 1.0f), 4);

			// draw a the surrogate zmp (a point proportional the horizontal acceleration)
			ndFloat32 xAlpha = dynamics.m_alpha.m_z / DEMO_GRAVITY;
			ndFloat32 zAlpha = -dynamics.m_alpha.m_x / DEMO_GRAVITY;
			const ndVector surrogateLocalZmpPoint(xAlpha, ndFloat32(0.0f), zAlpha, ndFloat32(1.0f));
			ndVector scaledSurrogateLocalZmpPoint(surrogateLocalZmpPoint.Scale(ndFloat32(0.25f)));
			scaledSurrogateLocalZmpPoint.m_w = ndFloat32(1.0f);
			const ndVector surrogateZmpPoint(centerOfPresure.TransformVector(scaledSurrogateLocalZmpPoint));
			context.DrawPoint(surrogateZmpPoint, ndVector(1.0f, 1.0f, 0.0f, 1.0f), 4);

//ndAssert((surrogateLocalZmpPoint.m_x * scaledLocalZmp.m_x >= 0.0f) && (surrogateLocalZmpPoint.m_z * scaledLocalZmp.m_z >= 0.0f));

static int xxxxx;
ndTrace(("%d suppostpoints(%d) alpha(%f %f) zmp(%f %f)\n", xxxxx, supportPolygon.GetCount(), dynamics.m_alpha.m_x, dynamics.m_alpha.m_z, localZmp.m_x * 10.0f, localZmp.m_z * 10.0f));
//if (xxxxx >= 1127)
if (xxxxx >= 100)
	xxxxx *= 1;
xxxxx++;

		}

		mutable ndIkSolver m_solver;
		ndAnimationPose m_animPose;
		ndFixSizeArray<ndEffectorInfo, 4> m_legs;
		ndSharedPtr<ndAnimationBlendTreeNode> m_poseGenerator;
		ndSharedPtr<ndAnimationBlendTreeNode> m_animBlendTree;
		ndFloat32 m_timestep;
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
			// build thigh
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
			ndSharedPtr<ndBody> contact(CreateRigidBody(contactEntity, contactMatrix, limbMass, heel->GetAsBodyDynamic()));

			const ndMatrix contactAxis (ndRollMatrix(ndFloat32(90.0f) * ndDegreeToRad) * contactMatrix);
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
		notify->InitAnimation();
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
