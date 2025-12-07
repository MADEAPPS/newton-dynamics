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
#include "ndMeshLoader.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndQuadSpiderPlayer.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"

namespace ndQuadSpiderPlayer
{
	class ndAnimatedHelper : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "");
		}
	};

	class ndProdeduralGaitGenerator : public ndAnimationSequence
	{
		public:
		ndProdeduralGaitGenerator()
			:ndAnimationSequence()
			//,m_amp(D_CYCLE_AMPLITUDE)
			//,m_stride_x(D_CYCLE_STRIDE_X)
			//,m_stride_z(D_CYCLE_STRIDE_Z)
		{
			m_duration = ndFloat32(4.0f);
		}

		ndVector GetTranslation(ndFloat32) const override
		{
			return ndVector::m_zero;
		}

		void CalculatePose(ndAnimationPose& output, ndFloat32 param) override
		{
			//// generate a procedural in place march gait
			//ndAssert(param >= ndFloat32(0.0f));
			//ndAssert(param <= ndFloat32(1.0f));
			//
			//ndFloat32 gaitFraction = 0.25f;
			//ndFloat32 gaitGuard = gaitFraction * 0.25f;
			//ndFloat32 omega = ndPi / (gaitFraction - gaitGuard);
			//
			//for (ndInt32 i = 0; i < output.GetCount(); i++)
			//{
			//	const ndEffectorInfo& leg = *(ndEffectorInfo*)output[i].m_userData;;
			//	output[i].m_userParamFloat = 0.0f;
			//	output[i].m_posit = leg.m_effector->GetRestPosit();
			//}
			//
			//for (ndInt32 i = 0; i < output.GetCount(); i++)
			//{
			//	const ndEffectorInfo& leg = *(ndEffectorInfo*)output[i].m_userData;;
			//	const ndVector localPosit(leg.m_effector->GetRestPosit());
			//	ndFloat32 stride_x = m_stride_x;
			//	//ndFloat32 stride_z = m_stride_z;
			//	ndFloat32 phase = 0.0f;
			//	if (localPosit.m_x > 0.0f)
			//	{
			//		phase = (localPosit.m_z > 0.0f) ? 0.0f : 0.50f;
			//	}
			//	else
			//	{
			//		phase = (localPosit.m_z > 0.0f) ? 0.75f : 0.25f;
			//	}
			//
			//	stride_x = 0.0f;
			//	//stride_z = 0.0f;
			//
			//	ndFloat32 t = ndMod(param - phase + ndFloat32(1.0f), ndFloat32(1.0f));
			//	if ((t >= gaitGuard) && (t <= gaitFraction))
			//	{
			//		output[i].m_posit.m_y += m_amp * ndSin(omega * (t - gaitGuard));
			//		output[i].m_userParamFloat = 1.0f;
			//
			//		ndFloat32 num = t - gaitGuard;
			//		ndFloat32 den = gaitFraction - gaitGuard;
			//
			//		ndFloat32 t0 = num / den;
			//		output[i].m_posit.m_x += stride_x * t0 - stride_x * 0.5f;
			//		//output[i].m_posit.m_z += -(stride_z * t0 - stride_z * 0.5f);
			//	}
			//	else
			//	{
			//		if (t <= gaitGuard)
			//		{
			//			t += 1.0f;
			//			output[i].m_userParamFloat = 0.5f;
			//		}
			//
			//		ndFloat32 num = t - gaitFraction;
			//		ndFloat32 den = 1.0f - (gaitFraction - gaitGuard);
			//		ndFloat32 t0 = num / den;
			//		output[i].m_posit.m_x += -(stride_x * t0 - stride_x * 0.5f);
			//		//output[i].m_posit.m_z += (stride_z * t0 - stride_z * 0.5f);
			//	}
			//}
		}

		ndFloat32 m_amp;
		ndFloat32 m_stride_x;
		ndFloat32 m_stride_z;
	};

	ndController::ndController()
		:ndModelNotify()
		,m_timestep(0.0f)
	{
	}

	void ndController::Update(ndFloat32 timestep)
	{
		m_timestep = timestep;

		ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
		ndBodyKinematic* const rootBody = model->GetRoot()->m_body->GetAsBodyKinematic();
		rootBody->SetSleepState(false);

		//const ndModelArticulation::ndCenterOfMassDynamics comDynamics(CalculateDynamics(timestep));
		//const ndVector comOmega(comDynamics.m_omega);
		//const ndVector comAlpha(comDynamics.m_alpha);

		//ndFloat32 animSpeed = 2.0f * m_control->m_animSpeed;
		ndFloat32 animSpeed = 0.5f;
		m_animBlendTree->Update(timestep * animSpeed);

		ndVector veloc;
		m_animBlendTree->Evaluate(m_pose, veloc);

		const ndVector upVector(rootBody->GetMatrix().m_up);
		for (ndInt32 i = 0; i < m_legs.GetCount(); ++i)
		{
			ndEffectorInfo& leg = m_legs[i];
			ndIkSwivelPositionEffector* const effector = leg.m_effector;

			ndVector posit(m_pose[i].m_posit);
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
				ndAssert(0);
				// project that target to the sphere of the corrent position
				leg.m_effector->SetAsReducedDof();
			}

			effector->SetSwivelAngle(swivelAngle);
			effector->SetLocalTargetPosition(posit);

			//// calculate lookAt angle
			//ndMatrix lookAtMatrix0;
			//ndMatrix lookAtMatrix1;
			//ndJointHinge* const heelHinge = leg.m_heel;
			//heelHinge->CalculateGlobalMatrix(lookAtMatrix0, lookAtMatrix1);
			//
			//ndMatrix upMatrix(ndGetIdentityMatrix());
			//upMatrix.m_front = lookAtMatrix0.m_front;
			//upMatrix.m_right = (upVector.CrossProduct(upMatrix.m_front) & ndVector::m_triplexMask).Normalize();
			//upMatrix.m_up = upMatrix.m_right.CrossProduct(upMatrix.m_front);
			//upMatrix = upMatrix * lookAtMatrix0.OrthoInverse();
			//const ndFloat32 angle = ndAtan2(upMatrix.m_up.m_z, upMatrix.m_up.m_y);
			//heelHinge->SetTargetAngle(angle);
		}

	}

	void ndController::ResetModel()
	{
		ndAssert(0);
	}

	void ndController::CreateAnimationBlendTree()
	{
		ndSharedPtr<ndAnimationSequence> sequence(new ndProdeduralGaitGenerator());

		//m_poseGenerator = ndSharedPtr<ndAnimationBlendTreeNode>(new ndAnimationSequencePlayer(sequence));
		//m_animBlendTree = ndSharedPtr<ndAnimationBlendTreeNode>(m_poseGenerator);
		ndSharedPtr<ndAnimationBlendTreeNode> proceduralPoseGenerator (new ndAnimationSequencePlayer(sequence));
		m_animBlendTree = ndSharedPtr<ndAnimationBlendTreeNode>(proceduralPoseGenerator);
		
		////ndFloat32 duration = ((ndAnimationSequencePlayer*)*m_poseGenerator)->GetSequence()->GetDuration();
		////m_animBlendTree->SetTime(duration * ndRand());
		//m_animBlendTree->SetTime(0.0f);
		
		ndProdeduralGaitGenerator* const poseGenerator = (ndProdeduralGaitGenerator*)*sequence;
		for (ndInt32 i = 0; i < m_legs.GetCount(); ++i)
		{
			ndEffectorInfo& leg = m_legs[i];
			ndAnimKeyframe keyFrame;
			keyFrame.m_userData = &leg;
			m_pose.PushBack(keyFrame);
			poseGenerator->AddTrack();
		}
	}

	void ndController::CreateArticulatedModel(
		ndDemoEntityManager* const scene,
		ndModelArticulation* const model,
		ndSharedPtr<ndMesh> mesh,
		ndSharedPtr<ndRenderSceneNode> visualMesh)
	{
		auto CreateRigidBody = [scene](ndSharedPtr<ndMesh>& mesh, ndSharedPtr<ndRenderSceneNode>& visualMesh, ndFloat32 mass, ndBodyDynamic* const parentBody)
		{
			ndSharedPtr<ndShapeInstance> shape(mesh->CreateCollision());

			ndBodyKinematic* const body = new ndBodyDynamic();
			body->SetNotifyCallback(new ndDemoEntityNotify(scene, visualMesh, parentBody));
			body->SetMatrix(mesh->CalculateGlobalMatrix());
			body->SetCollisionShape(*(*shape));
			body->GetAsBodyDynamic()->SetMassMatrix(mass, *(*shape));
			return body;
		};

		// add the main upper body
		ndSharedPtr<ndBody> rootBody(CreateRigidBody(mesh, visualMesh, D_BODY_MASS, nullptr));
		ndModelArticulation::ndNode* const modelRootNode = model->AddRootBody(rootBody);
		
		// build all four legs
		for (ndList<ndSharedPtr<ndMesh>>::ndNode* node = mesh->GetChildren().GetFirst(); node; node = node->GetNext())
		{
			// build thigh
			ndSharedPtr<ndMesh> thighMesh(node->GetInfo());
			ndSharedPtr<ndRenderSceneNode> thighEntity(visualMesh->FindByName(thighMesh->GetName())->GetSharedPtr());
			ndSharedPtr<ndBody> thighBody(CreateRigidBody(thighMesh, thighEntity, D_LIMB_MASS, rootBody->GetAsBodyDynamic()));
			
			const ndMatrix thighMatrix(thighMesh->CalculateGlobalMatrix());
			ndSharedPtr<ndJointBilateralConstraint> ballJoint(new ndJointSpherical(thighMatrix, thighBody->GetAsBodyKinematic(), rootBody->GetAsBodyKinematic()));
			ndModelArticulation::ndNode* const thighNode = model->AddLimb(modelRootNode, thighBody, ballJoint);
		
			// build calf
			ndSharedPtr<ndMesh> calfMesh(thighMesh->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndRenderSceneNode> calfEntity(thighEntity->FindByName(calfMesh->GetName())->GetSharedPtr());
			ndSharedPtr<ndBody> calf(CreateRigidBody(calfMesh, calfEntity, D_LIMB_MASS, thighBody->GetAsBodyDynamic()));
			
			const ndMatrix calfMatrix(calfMesh->CalculateGlobalMatrix());
			ndSharedPtr<ndJointBilateralConstraint> calfHinge(new ndJointHinge(calfMatrix, calf->GetAsBodyKinematic(), thighBody->GetAsBodyKinematic()));
			ndModelArticulation::ndNode* const calfNode = model->AddLimb(thighNode, calf, calfHinge);
			
			((ndIkJointHinge*)*calfHinge)->SetLimitState(true);
			((ndIkJointHinge*)*calfHinge)->SetLimits(-60.0f * ndDegreeToRad, 50.0f * ndDegreeToRad);
			
			// build heel
			ndSharedPtr<ndMesh> heelMesh(calfMesh->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndRenderSceneNode> heelEntity(calfEntity->FindByName(heelMesh->GetName())->GetSharedPtr());
			ndSharedPtr<ndBody> heel(CreateRigidBody(heelMesh, heelEntity, D_LIMB_MASS * 0.5f, calf->GetAsBodyDynamic()));

			const ndMatrix heelMatrix(heelMesh->CalculateGlobalMatrix());
			ndSharedPtr<ndJointBilateralConstraint> heelHinge(new ndJointHinge(heelMatrix, heel->GetAsBodyKinematic(), calf->GetAsBodyKinematic()));
			ndModelArticulation::ndNode* const heelNode = model->AddLimb(calfNode, heel, heelHinge);
			((ndJointHinge*)*heelHinge)->SetAsSpringDamper(0.001f, 2000.0f, 50.0f);
			
			// build soft contact heel
			ndSharedPtr<ndMesh> contactMesh(heelMesh->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndRenderSceneNode> contactEntity(heelEntity->FindByName(contactMesh->GetName())->GetSharedPtr());
			ndSharedPtr<ndBody> contact(CreateRigidBody(contactMesh, contactEntity, D_LIMB_MASS * 0.5f, heel->GetAsBodyDynamic()));

			const ndMatrix contactMatrix(contactMesh->CalculateGlobalMatrix());
			const ndMatrix contactAxis(ndRollMatrix(ndFloat32(90.0f) * ndDegreeToRad) * contactMatrix);
			ndSharedPtr<ndJointBilateralConstraint> softContact(new ndJointSlider(contactAxis, contact->GetAsBodyKinematic(), heel->GetAsBodyKinematic()));
			model->AddLimb(heelNode, contact, softContact);
			((ndJointSlider*)*softContact)->SetAsSpringDamper(0.01f, 2000.0f, 10.0f);
			
			// create effector
			ndSharedPtr<ndMesh> footEntity(contactMesh->GetChildren().GetFirst()->GetInfo());
			ndMatrix footMatrix(footEntity->CalculateGlobalMatrix());
			ndMatrix effectorRefFrame(footMatrix);
			effectorRefFrame.m_posit = thighMatrix.m_posit;
			
			ndFloat32 regularizer = ndFloat32(0.001f);
			ndFloat32 effectorStrength = ndFloat32(20.0f * 10.0f * 500.0f);
			ndSharedPtr<ndJointBilateralConstraint> effector(new ndIkSwivelPositionEffector(effectorRefFrame, rootBody->GetAsBodyKinematic(), footMatrix.m_posit, contact->GetAsBodyKinematic()));
			((ndIkSwivelPositionEffector*)*effector)->SetLinearSpringDamper(regularizer, 4000.0f, 50.0f);
			((ndIkSwivelPositionEffector*)*effector)->SetAngularSpringDamper(regularizer, 4000.0f, 50.0f);
			((ndIkSwivelPositionEffector*)*effector)->SetWorkSpaceConstraints(0.0f, 0.75f * 0.9f);
			((ndIkSwivelPositionEffector*)*effector)->SetMaxForce(effectorStrength);
			((ndIkSwivelPositionEffector*)*effector)->SetMaxTorque(effectorStrength);
			model->AddCloseLoop(effector);
			
			ndEffectorInfo leg;
			leg.m_calf = (ndJointHinge*)*calfHinge;
			//leg.m_heel = (ndJointHinge*)*heelHinge;
			//leg.m_thigh = (ndJointSpherical*)*ballJoint;
			//leg.m_softContact = (ndJointSlider*)*softContact;
			leg.m_effector = (ndIkSwivelPositionEffector*)*effector;
			m_legs.PushBack(leg);
		}

		ndWorld* const world = scene->GetWorld();
		const ndMatrix upMatrix(rootBody->GetMatrix());
		ndSharedPtr<ndJointBilateralConstraint> upVector(new ndJointUpVector(upMatrix.m_up, rootBody->GetAsBodyKinematic(), world->GetSentinelBody()));
		model->AddCloseLoop(upVector);
	}

	ndSharedPtr<ndModel> ndController::CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, const ndRenderMeshLoader& loader)
	{
		ndMatrix matrix(location);
		matrix.m_posit = FindFloor(*scene->GetWorld(), matrix.m_posit, 200.0f);
		matrix.m_posit.m_y += ndFloat32(0.5f);
		loader.m_mesh->m_matrix = loader.m_mesh->m_matrix * matrix;
		
		ndSharedPtr<ndRenderSceneNode> visualMesh(loader.m_renderMesh->Clone());
		visualMesh->SetTransform(loader.m_mesh->m_matrix);
		visualMesh->SetTransform(loader.m_mesh->m_matrix);
		
		ndSharedPtr<ndModel> model (new ndModelArticulation());
		ndSharedPtr<ndModelNotify> controller(new ndController());
		model->SetNotifyCallback(controller);
		ndController* const playerController = (ndController*)(*controller);
		playerController->CreateArticulatedModel(scene, model->GetAsModelArticulation(), loader.m_mesh, visualMesh);

		// create animation blend tree
		playerController->CreateAnimationBlendTree();
		
		// add model a visual mesh to the scene and world
		ndWorld* const world = scene->GetWorld();
		world->AddModel(model);
		scene->AddEntity(visualMesh);
		model->AddBodiesAndJointsToWorld();
		return model;
	}

	//class ndPlaybackController : public ndController
	//{
	//	public:
	//	ndPlaybackController()
	//		:ndController()
	//	{
	//	}
	//
	//	//ndSharedPtr<ndBrainAgent> m_saveAgent;
	//	ndSharedPtr<ndAnimationBlendTreeNode> m_animBlendTree;
	//};

}

using namespace ndQuadSpiderPlayer;


void ndQuadSpiderAnimated(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));
	
	// add a help message
	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndAnimatedHelper());
	scene->SetDemoHelp(demoHelper);
	
	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("spider.nd"));
	ndController::CreateModel(scene, matrix, loader);
	
	matrix.m_posit.m_x -= 2.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 0.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}