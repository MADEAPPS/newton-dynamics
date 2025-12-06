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
	class ndHelpLegend_Sac : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "Cart Pole is the classic hello world of reinforcement learning");
			scene->Print(color, "it is use to test the correctness of an algorithm implementation.");
			scene->Print(color, "The model is trained using Soft Actor Critic(SAC).");
			scene->Print(color, "It consists of a pole attached by a hinge to a sliding cart.");
			scene->Print(color, "The objective goal was to train a neural network to keep");
			scene->Print(color, "the pole balanced in an upright position.");
			scene->Print(color, "You can interact with the simulation and try.");
			scene->Print(color, "to knock the pole over using the mouse.");
		}
	};
	
	class ndHelpLegend_Ppo : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "Cart Pole is the classic hello world of reinforcement learning");
			scene->Print(color, "It is used to test the correctness of an algorithm implementation.");
			scene->Print(color, "The model is trained using Proximal Policy Gradient (PPO).");
			scene->Print(color, "It consists of a pole attached by a hinge to a sliding cart.");
			scene->Print(color, "The objective goal was to train a neural network to keep");
			scene->Print(color, "the pole balanced in an upright position.");
			scene->Print(color, "You can interact with the simulation and try.");
			scene->Print(color, "to knock the pole over using the mouse.");
		}
	};

	class ndPlaybackController : public ndController
	{
		public:
		ndPlaybackController()
			:ndController()
		{
		}

		ndSharedPtr<ndBrainAgent> m_saveAgent;
	};

	ndController::ndController()
		:ndModelNotify()
		//,m_agent(nullptr)
		,m_timestep(0.0f)
	{
	}

	void ndController::Update(ndFloat32 timestep)
	{
		m_timestep = timestep;
		//if (m_agent)
		//{
		//	m_agent->Step();
		//}
	}

	void ndController::ResetModel()
	{
		ndAssert(0);
		//ndMatrix cartMatrix(ndGetIdentityMatrix());
		//cartMatrix.m_posit = m_cart->GetMatrix().m_posit;
		//cartMatrix.m_posit.m_x = ndFloat32(0.0f);
		////cartMatrix.m_posit.m_x = ndFloat32(10.0f) * (ndRand() - ndFloat32(0.5f));
		//cartMatrix.m_posit.m_y = ndFloat32(0.1f);
		//m_cart->SetMatrix(cartMatrix);
		//
		//const ndMatrix poleMatrix(m_poleHinge->CalculateGlobalMatrix1());
		//m_pole->SetMatrix(poleMatrix);
		//
		//m_pole->SetOmega(ndVector::m_zero);
		//m_pole->SetVelocity(ndVector::m_zero);
		//
		//m_cart->SetOmega(ndVector::m_zero);
		//m_cart->SetVelocity(ndVector::m_zero);
		//
		//GetModel()->GetAsModelArticulation()->ClearMemory();
	}

	#pragma optimize( "", off )
	bool ndController::IsTerminal() const
	{
		ndAssert(0);
		return true;
		//const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		//const ndJointSlider* const slider = (ndJointSlider*)*m_slider;
		//ndFloat32 angle = hinge->GetAngle();
		//ndFloat32 speed = slider->GetSpeed();
		//bool isdead = ndAbs(angle) > (REWARD_MIN_ANGLE * ndFloat32(2.0f));
		//isdead = isdead || (ndAbs(speed) > ndFloat32(3.0f));
		//return isdead;
	}

	#pragma optimize( "", off )
	ndBrainFloat ndController::CalculateReward() const
	{
		ndAssert(0);
		return true;

		//if (IsTerminal())
		//{
		//	// a terminal reward of zero should make for smoother MDPs. 
		//	// training small networks could be much harder with negative terminal rewards..
		//	// return ndBrainFloat(-1.0f);
		//	return ndBrainFloat(-1.0f);
		//}
		//
		//ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		//ndJointSlider* const slider = (ndJointSlider*)*m_slider;
		//
		//ndFloat32 angle = hinge->GetAngle();
		//ndFloat32 omega = hinge->GetOmega();
		//ndFloat32 speed = slider->GetSpeed();
		//
		//ndFloat32 invSigma2 = ndFloat32(50.0f);
		//ndFloat32 speedReward = ndExp(-invSigma2 * speed * speed);
		//ndFloat32 omegaReward = ndExp(-invSigma2 * omega * omega);
		//ndFloat32 angleReward = ndExp(-invSigma2 * angle * angle);
		//
		//// make sure the reward is never negative, to avoid the possibility of  
		//// MDP states with negative values.
		//ndFloat32 reward = ndFloat32(0.3f) * angleReward + ndFloat32(0.3f) * omegaReward + ndFloat32(0.4f) * speedReward;
		//return ndBrainFloat(reward);
	}

	void ndController::ApplyActions(ndBrainFloat* const actions)
	{
		ndAssert(0);
		//ndBrainFloat action = actions[0];
		//ndBrainFloat accel = PUSH_ACCEL * action;
		//ndFloat32 pushForce = accel * (m_cart->GetAsBodyDynamic()->GetMassMatrix().m_w);
		//
		//ndJointSlider* const slider = (ndJointSlider*)*m_slider;
		//const ndMatrix matrix(slider->CalculateGlobalMatrix0());
		//
		//ndVector force(m_cart->GetAsBodyDynamic()->GetForce() + matrix.m_front.Scale(pushForce));
		//m_cart->GetAsBodyDynamic()->SetForce(force);
	}

	void ndController::GetObservation(ndBrainFloat* const observation)
	{
		ndAssert(0);
		//const ndJointHinge* const hinge = (ndJointHinge*)*m_poleHinge;
		//const ndJointSlider* const slider = (ndJointSlider*)*m_slider;
		//
		//ndFloat32 omega = hinge->GetOmega();
		//ndFloat32 angle = hinge->GetAngle();
		//ndFloat32 speed = slider->GetSpeed();
		//
		//observation[m_poleAngle] = ndBrainFloat(angle);
		//observation[m_poleOmega] = ndBrainFloat(omega);
		//observation[m_cartSpeed] = ndBrainFloat(speed);
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

		// add the cart mesh and body
		ndFloat32 mass = 20.0f;
		ndFloat32 limbMass = 0.25f;
		ndSharedPtr<ndBody> rootBody(CreateRigidBody(mesh, visualMesh, mass, nullptr));
		ndModelArticulation::ndNode* const modelRootNode = model->AddRootBody(rootBody);
		
		// build all four legs
		for (ndList<ndSharedPtr<ndMesh>>::ndNode* node = mesh->GetChildren().GetFirst(); node; node = node->GetNext())
		{
			// build thigh
			ndSharedPtr<ndMesh> thighMesh(node->GetInfo());
			ndSharedPtr<ndRenderSceneNode> thighEntity(visualMesh->FindByName(thighMesh->GetName())->GetSharedPtr());
			ndSharedPtr<ndBody> thighBody(CreateRigidBody(thighMesh, thighEntity, limbMass, rootBody->GetAsBodyDynamic()));
			
			const ndMatrix thighMatrix(thighMesh->CalculateGlobalMatrix());
			ndSharedPtr<ndJointBilateralConstraint> ballJoint(new ndJointSpherical(thighMatrix, thighBody->GetAsBodyKinematic(), rootBody->GetAsBodyKinematic()));
			ndModelArticulation::ndNode* const thighNode = model->AddLimb(modelRootNode, thighBody, ballJoint);
		
			// build calf
			ndSharedPtr<ndMesh> calfMesh(thighMesh->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndRenderSceneNode> calfEntity(thighEntity->FindByName(calfMesh->GetName())->GetSharedPtr());
			ndSharedPtr<ndBody> calf(CreateRigidBody(calfMesh, calfEntity, limbMass, thighBody->GetAsBodyDynamic()));
			
			const ndMatrix calfMatrix(calfMesh->CalculateGlobalMatrix());
			ndSharedPtr<ndJointBilateralConstraint> calfHinge(new ndJointHinge(calfMatrix, calf->GetAsBodyKinematic(), thighBody->GetAsBodyKinematic()));
			ndModelArticulation::ndNode* const calfNode = model->AddLimb(thighNode, calf, calfHinge);
			
			((ndIkJointHinge*)*calfHinge)->SetLimitState(true);
			((ndIkJointHinge*)*calfHinge)->SetLimits(-60.0f * ndDegreeToRad, 50.0f * ndDegreeToRad);
			
			// build heel
			ndSharedPtr<ndMesh> heelMesh(calfMesh->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndRenderSceneNode> heelEntity(calfEntity->FindByName(heelMesh->GetName())->GetSharedPtr());
			ndSharedPtr<ndBody> heel(CreateRigidBody(heelMesh, heelEntity, limbMass * 0.5f, calf->GetAsBodyDynamic()));

			const ndMatrix heelMatrix(heelMesh->CalculateGlobalMatrix());
			ndSharedPtr<ndJointBilateralConstraint> heelHinge(new ndJointHinge(heelMatrix, heel->GetAsBodyKinematic(), calf->GetAsBodyKinematic()));
			ndModelArticulation::ndNode* const heelNode = model->AddLimb(calfNode, heel, heelHinge);
			((ndJointHinge*)*heelHinge)->SetAsSpringDamper(0.001f, 2000.0f, 50.0f);
			
			// build soft contact heel
			ndSharedPtr<ndMesh> contactMesh(heelMesh->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndRenderSceneNode> contactEntity(heelEntity->FindByName(contactMesh->GetName())->GetSharedPtr());
			ndSharedPtr<ndBody> contact(CreateRigidBody(contactMesh, contactEntity, limbMass * 0.5f, heel->GetAsBodyDynamic()));

			const ndMatrix contactMatrix(contactMesh->CalculateGlobalMatrix());
			const ndMatrix contactAxis(ndRollMatrix(ndFloat32(90.0f) * ndDegreeToRad) * contactMatrix);
			ndSharedPtr<ndJointBilateralConstraint> softContact(new ndJointSlider(contactAxis, contact->GetAsBodyKinematic(), heel->GetAsBodyKinematic()));
			model->AddLimb(heelNode, contact, softContact);
			((ndJointSlider*)*softContact)->SetAsSpringDamper(0.01f, 2000.0f, 10.0f);
			
			// create effector
			ndSharedPtr<ndMesh> footEntity(contactMesh->GetChildren().GetFirst()->GetInfo());
			//ndMatrix footMatrix(matrix);
			//footMatrix.m_posit = (footEntity->GetCurrentMatrix() * contactMatrix).m_posit;
			ndMatrix footMatrix(footEntity->CalculateGlobalMatrix());
			
			ndMatrix effectorRefFrame(footMatrix);
			effectorRefFrame.m_posit = thighMatrix.m_posit;
			
			ndFloat32 regularizer = 0.001f;
			ndFloat32 effectorStrength = 20.0f * 10.0f * 500.0f;
			ndSharedPtr<ndJointBilateralConstraint> effector(new ndIkSwivelPositionEffector(effectorRefFrame, rootBody->GetAsBodyKinematic(), footMatrix.m_posit, contact->GetAsBodyKinematic()));
			((ndIkSwivelPositionEffector*)*effector)->SetLinearSpringDamper(regularizer, 4000.0f, 50.0f);
			((ndIkSwivelPositionEffector*)*effector)->SetAngularSpringDamper(regularizer, 4000.0f, 50.0f);
			((ndIkSwivelPositionEffector*)*effector)->SetWorkSpaceConstraints(0.0f, 0.75f * 0.9f);
			((ndIkSwivelPositionEffector*)*effector)->SetMaxForce(effectorStrength);
			((ndIkSwivelPositionEffector*)*effector)->SetMaxTorque(effectorStrength);
			model->AddCloseLoop(effector);
			
			//RobotModelNotify::ndEffectorInfo leg;
			//leg.m_calf = (ndJointHinge*)*calfHinge;
			//leg.m_heel = (ndJointHinge*)*heelHinge;
			//leg.m_thigh = (ndJointSpherical*)*ballJoint;
			//leg.m_softContact = (ndJointSlider*)*softContact;
			//leg.m_effector = (ndIkSwivelPositionEffector*)*effector;
			//notify->m_legs.PushBack(leg);

			//break;
		}

		ndWorld* const world = scene->GetWorld();
		const ndMatrix upMatrix(rootBody->GetMatrix());
		ndSharedPtr<ndJointBilateralConstraint> upVector(new ndJointUpVector(upMatrix.m_up, rootBody->GetAsBodyKinematic(), world->GetSentinelBody()));
		model->AddCloseLoop(upVector);

		//notify->InitAnimation();
	}

	ndSharedPtr<ndModel> ndController::CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, const ndRenderMeshLoader& loader, const char* const)
	{
		ndMatrix matrix(location);
		matrix.m_posit = FindFloor(*scene->GetWorld(), matrix.m_posit, 200.0f);
		matrix.m_posit.m_y += ndFloat32(0.5f);
		loader.m_mesh->m_matrix = loader.m_mesh->m_matrix * matrix;
		
		ndSharedPtr<ndRenderSceneNode> visualMesh(loader.m_renderMesh->Clone());
		visualMesh->SetTransform(loader.m_mesh->m_matrix);
		visualMesh->SetTransform(loader.m_mesh->m_matrix);
		
		//ndModelArticulation* const model = new ndModelArticulation();
		ndSharedPtr<ndModel> model (new ndModelArticulation());
		ndSharedPtr<ndModelNotify> controller(new ndPlaybackController());
		model->SetNotifyCallback(controller);
		ndPlaybackController* const playerController = (ndPlaybackController*)(*controller);
		playerController->CreateArticulatedModel(scene, model->GetAsModelArticulation(), loader.m_mesh, visualMesh);
		
		// add model a visual mesh to the scene and world
		ndWorld* const world = scene->GetWorld();
		world->AddModel(model);
		scene->AddEntity(visualMesh);
		model->AddBodiesAndJointsToWorld();
		return model;
	}
}

using namespace ndQuadSpiderPlayer;

//void ndCartpolePlayer_PPO(ndDemoEntityManager* const scene)
//{
//	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));
//
//	// add a help message
//	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend_Ppo());
//	scene->SetDemoHelp(demoHelper);
//
//	ndMatrix matrix(ndGetIdentityMatrix());
//	ndRenderMeshLoader loader(*scene->GetRenderer());
//	loader.LoadMesh(ndGetWorkingFileName("cartpole.nd"));
//	ndController::CreateModel(scene, matrix, loader, CONTROLLER_NAME_PPO);
//
//	matrix.m_posit.m_x -= 0.0f;
//	matrix.m_posit.m_y += 0.5f;
//	matrix.m_posit.m_z += 2.0f;
//	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
//	scene->SetCameraMatrix(rotation, matrix.m_posit);
//}
//
//void ndCartpolePlayer_SAC(ndDemoEntityManager* const scene)
//{
//	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));
//
//	// add a help message
//	ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend_Sac());
//	scene->SetDemoHelp(demoHelper);
//
//	ndMatrix matrix(ndGetIdentityMatrix());
//	ndRenderMeshLoader loader(*scene->GetRenderer());
//	loader.LoadMesh(ndGetWorkingFileName("cartpole.nd"));
//	ndController::CreateModel(scene, matrix, loader, CONTROLLER_NAME_SAC);
//
//	matrix.m_posit.m_x -= 0.0f;
//	matrix.m_posit.m_y += 0.5f;
//	matrix.m_posit.m_z += 2.0f;
//	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
//	scene->SetCameraMatrix(rotation, matrix.m_posit);
//}


void ndQuadSpiderAnimated(ndDemoEntityManager* const scene)
{
	ndSharedPtr<ndBody> mapBody(BuildFloorBox(scene, ndGetIdentityMatrix(), "marbleCheckBoard.png", 0.1f, true));
	
	//// add a help message
	//ndSharedPtr<ndDemoEntityManager::ndDemoHelper> demoHelper(new ndHelpLegend_Sac());
	//scene->SetDemoHelp(demoHelper);
	
	ndMatrix matrix(ndGetIdentityMatrix());
	ndRenderMeshLoader loader(*scene->GetRenderer());
	loader.LoadMesh(ndGetWorkingFileName("spider.nd"));
	ndController::CreateModel(scene, matrix, loader, CONTROLLER_NAME_SAC);
	
	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}