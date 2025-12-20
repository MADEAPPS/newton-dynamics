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
#include "ndDemoCameraNodeFollow.h"

namespace ndQuadSpiderPlayer
{
	#define D_WALK_STRIDE		ndReal(0.3f)
	#define D_STRIDE_AMPLITUD	ndReal(0.35f)
	#define D_TURN_RATE			ndReal(0.25f)

	#define D_HIP_SWAY_X		ndReal(0.1f)
	#define D_HIP_SWAY_Z		ndReal(0.1f)

	ndBodySwingControl::ndBodySwingControl(const ndSharedPtr<ndAnimationBlendTreeNode>& input)
		:ndAnimationBlendTreeNode(input)
		,m_x(ndReal(0.0f))
		,m_y(ndReal(0.0f))
		,m_z(ndReal(0.0f))
		,m_yaw(ndReal(0.0f))
		,m_roll(ndReal(0.0f))
		,m_pitch(ndReal(0.0f))
	{
		Reset();
	}

	void ndBodySwingControl::Reset()
	{
		m_x = ndReal(0.0f);
		m_y = ndReal(0.0f);
		m_z = ndReal(0.0f);
		m_yaw = ndReal(0.0f);
		m_roll = ndReal(0.0f);
		m_pitch = ndReal(0.0f);
	}

	void ndBodySwingControl::Evaluate(ndAnimationPose& output, ndVector& veloc)
	{
		ndAnimationBlendTreeNode::Evaluate(output, veloc);

		ndMatrix matrix(ndPitchMatrix(m_pitch * ndDegreeToRad) * ndYawMatrix(m_yaw * ndDegreeToRad) * ndRollMatrix(m_roll * ndDegreeToRad));
		matrix.m_posit.m_x = -m_x;
		matrix.m_posit.m_y = m_y;
		matrix.m_posit.m_z = -m_z;
		for (ndInt32 i = 0; i < output.GetCount(); ++i)
		{
			ndAnimKeyframe& keyFrame = output[i];
			ndEffectorInfo* const info = (ndEffectorInfo*)keyFrame.m_userData;
			ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)info->m_effector;
		
			ndMatrix localMatrix(effector->GetLocalMatrix1());
			ndVector p0(localMatrix.TransformVector(keyFrame.m_posit));
			ndVector p1(matrix.TransformVector(p0));
			ndVector p2(localMatrix.UntransformVector(p1));
			keyFrame.m_posit = p2;
		}
	}

	ndProdeduralGaitGenerator::ndProdeduralGaitGenerator(ndController* const controller)
		:ndAnimationSequence()
		,m_owner(controller)
		,m_omega(ndFloat32(0.0f))
		,m_stride(ndFloat32(0.0f))
		,m_timeAcc(ndFloat32(0.0f))
	{
		m_duration = ndFloat32(1.0f);
		m_timeLine[0] = ndFloat32(0.0f);
		m_timeLine[1] = ndFloat32(0.0f);
		m_timeLine[2] = ndFloat32(m_duration);

		for (ndInt32 i = 0; i < 4; ++i)
		{
			ndEffectorInfo& leg = controller->m_legs[i];
			m_pose[i].m_base = leg.m_effector->GetEffectorPosit();
			m_pose[i].m_end = m_pose[i].m_base;
			m_pose[i].m_posit = m_pose[i].m_base;
			m_pose[i].m_start = m_pose[i].m_base;

			m_pose[i].m_a0 = m_pose[i].m_base.m_y;
			m_pose[i].m_a1 = ndFloat32 (0.0f);
			m_pose[i].m_a2 = ndFloat32(0.0f);

			m_pose[i].m_time = ndFloat32(0.0f);
			m_pose[i].m_maxTime = ndFloat32(1.0f);

			m_gaitSequence[i] = i;

			AddTrack();
		}
	}

	ndFloat32 ndProdeduralGaitGenerator::CalculateTime() const
	{
		ndFloat32 angle0 = m_timeAcc * ndFloat32(2.0f) * ndPi / m_duration;
		ndFloat32 deltaAngle = m_owner->m_timestep * ndFloat32(2.0f) * ndPi / m_duration;
		ndFloat32 angle1 = ndAnglesAdd(angle0, deltaAngle);
		if (angle1 < ndFloat32(0.0f))
		{
			angle1 += ndFloat32(2.0f) * ndPi;
		}
		ndFloat32 time = angle1 * m_duration / (ndFloat32(2.0f) * ndPi);
		return time;
	}

	ndProdeduralGaitGenerator::State ndProdeduralGaitGenerator::GetState(ndInt32 legIndex) const
	{
		// get the quadrant for this leg
		ndFloat32 sequenceOffset = ndFloat32(0.25f) * ndFloat32(m_gaitSequence[legIndex]);
		ndFloat32 t0 = ndMod(m_timeAcc + sequenceOffset * m_duration, m_duration);
		// get the next time
		ndFloat32 t1 = t0 + m_owner->m_timestep;

		// see if the step crosses a quadrant
		if (t0 <= m_timeLine[1])
		{
			if (t1 > m_timeLine[1])
			{
				return airToGround;
			}
			return onAir;
		}
		if (t0 <= m_timeLine[2])
		{
			if (t1 > m_timeLine[2])
			{
				return groundToAir;
			}
			return onGround;
		}
		ndAssert(0);
		return onGround;
	}

	void ndProdeduralGaitGenerator::IntegrateLeg(ndAnimationPose& output, ndInt32 legIndex)
	{
		output[legIndex].m_userParamFloat = ndFloat32(0.0f);

		State state = ndProdeduralGaitGenerator::GetState(legIndex);
//if (legIndex != 0)
//{
//	state = airToGround;
//}

		switch (state)
		{
			case groundToAir:
			{
				ndVector target(m_pose[legIndex].m_base);
				target.m_x += m_stride * ndFloat32(0.5f);

				ndFloat32 time = m_timeLine[1] - m_timeLine[0];
				m_pose[legIndex].m_end = target;
				m_pose[legIndex].m_start = m_pose[legIndex].m_posit;

				m_pose[legIndex].m_maxTime = time;
				m_pose[legIndex].m_time = ndFloat32(0.0f);

				// create a parabolic arch for the effector to follow
				ndFloat32 y0 = m_pose[legIndex].m_start.m_y;
				ndFloat32 y1 = m_pose[legIndex].m_end.m_y;

				ndFloat32 h = ndFloat32(0.5f) * (y1 + y0) + m_stride * D_STRIDE_AMPLITUD;

				ndFloat32 a0 = y0;
				ndFloat32 a1 = ndFloat32(4.0f) * (h - y0) - (y1 - y0);
				ndFloat32 a2 = y1 - y0 - a1;
				m_pose[legIndex].m_a0 = a0;
				m_pose[legIndex].m_a1 = a1;
				m_pose[legIndex].m_a2 = a2;

				output[legIndex].m_posit = m_pose[legIndex].m_posit;
				break;
			}

			case airToGround:
			{
				if (legIndex == 0)
				{
					// integrate last step
					m_pose[legIndex].m_time += m_owner->m_timestep;
					ndFloat32 param = m_pose[legIndex].m_time / m_pose[legIndex].m_maxTime;
					ndVector step(m_pose[legIndex].m_end - m_pose[legIndex].m_start);
					ndVector posit(m_pose[legIndex].m_start + step.Scale(param));

					ndFloat32 a0 = m_pose[legIndex].m_a0;
					ndFloat32 a1 = m_pose[legIndex].m_a1;
					ndFloat32 a2 = m_pose[legIndex].m_a2;
					posit.m_y = a0 + a1 * param + a2 * param * param;

					m_pose[legIndex].m_posit = posit;
				}

				ndVector target(m_pose[legIndex].m_base);
				target.m_x -= m_stride * ndFloat32 (0.5f);

				ndFloat32 time = m_timeLine[2] - m_timeLine[1];
				ndFloat32 angle = m_omega * time;
				ndMatrix matrix(ndYawMatrix(angle));
				target = matrix.RotateVector(target);

				m_pose[legIndex].m_start = m_pose[legIndex].m_posit;
				m_pose[legIndex].m_end = target;
				
				m_pose[legIndex].m_maxTime = time;
				m_pose[legIndex].m_time = ndFloat32(0.0f);

				output[legIndex].m_posit = m_pose[legIndex].m_posit;
				break;
			}

			case onAir:
			{
				m_pose[legIndex].m_time += m_owner->m_timestep;
				ndFloat32 param = m_pose[legIndex].m_time / m_pose[legIndex].m_maxTime;
				ndVector step(m_pose[legIndex].m_end - m_pose[legIndex].m_start);
				ndVector posit(m_pose[legIndex].m_start + step.Scale(param));

				ndFloat32 a0 = m_pose[legIndex].m_a0;
				ndFloat32 a1 = m_pose[legIndex].m_a1;
				ndFloat32 a2 = m_pose[legIndex].m_a2;
				posit.m_y = a0 + a1 * param + a2 * param * param;

				m_pose[legIndex].m_posit = posit;
				output[legIndex].m_posit = m_pose[legIndex].m_posit;
				break;
			}

			case onGround:
			default:
			{
				m_pose[legIndex].m_time += m_owner->m_timestep;
				ndFloat32 param = m_pose[legIndex].m_time / m_pose[legIndex].m_maxTime;
				ndVector step(m_pose[legIndex].m_end - m_pose[legIndex].m_start);
				m_pose[legIndex].m_posit = m_pose[legIndex].m_start + step.Scale(param);

				output[legIndex].m_posit = m_pose[legIndex].m_posit;
				break;
			}
		}
	}

	void ndProdeduralGaitGenerator::CalculatePose(ndAnimationPose& output, ndFloat32)
	{
		ndFloat32 nextTime = CalculateTime();
		ndAssert(nextTime < m_duration);
		ndAssert(nextTime >= ndFloat32(0.0f));

		for (ndInt32 i = 0; i < output.GetCount(); i++)
		{
			IntegrateLeg(output, i);
		}

		m_timeAcc = nextTime;
	}

	ndGeneratorWalkGait::ndGeneratorWalkGait(ndController* const controller)
		:ndProdeduralGaitGenerator(controller)
	{
		m_duration = ndFloat32(2.0f);
//m_duration = ndFloat32(5.0f);

		m_timeLine[0] = ndFloat32(0.00f) * m_duration;
		m_timeLine[1] = ndFloat32(0.25f) * m_duration;
		m_timeLine[2] = ndFloat32(1.00f) * m_duration;

		//m_stride = ndFloat32(0.0f);
		//m_omega = ndFloat32(0.0f);
		//m_stride = ndFloat32(0.4f);
		//m_omega = ndFloat32(0.3f);

		// gait one
		m_gaitSequence[0] = 3;
		m_gaitSequence[1] = 1;
		m_gaitSequence[2] = 2;
		m_gaitSequence[3] = 0;

		// gait two
		//m_gaitSequence[0] = 0;
		//m_gaitSequence[1] = 2;
		//m_gaitSequence[2] = 1;
		//m_gaitSequence[3] = 3;
	}

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
		m_animBlendTree->Update(timestep);

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
				// project that target to the sphere of the correct position
				leg.m_effector->SetAsReducedDof();
			}

			effector->SetSwivelAngle(swivelAngle);
			effector->SetLocalTargetPosition(posit);

			// the heel joint angle depend on the knee angle
			// I in theory no clip is need sine the knee angle 
			// is already with in limits, 
			// by I don't know until joint debug is enabled.
			ndJointHinge* const heelHinge = leg.m_heel;
			heelHinge->SetTargetAngle(-kneeAngle * ndFloat32(0.5f));
		}
	}

	void ndController::PostTransformUpdate(ndFloat32)
	{
		ndPhysicsWorld* const world = (ndPhysicsWorld*)GetModel()->GetWorld();
		ndDemoEntityManager* const scene = world->GetManager();

		ndRender* const renderer = *scene->GetRenderer();
		ndSharedPtr<ndRenderSceneNode> camera(renderer->GetCamera());
		if (camera == m_cameraNode)
		{
			m_walkGait->m_stride = 0.0f;
			//m_walkGait->m_stride = D_WALK_STRIDE;
			if (scene->GetKeyState(ImGuiKey_W))
			{
				m_walkGait->m_stride = D_WALK_STRIDE;
			}
			//else if (scene->GetKeyState(ImGuiKey_S))
			//{
			//	//turnSign = ndFloat32(-0.75f);
			//	//m_targetOmega = ND_EXCAVATOR_ENGINE_OMEGA;
			//	//engine->SetSleepState(false);
			//}

			m_walkGait->m_omega = 0.0f;
			if (scene->GetKeyState(ImGuiKey_A))
			{
				m_walkGait->m_omega = -D_TURN_RATE;
			}
			else if (scene->GetKeyState(ImGuiKey_D))
			{
				m_walkGait->m_omega = D_TURN_RATE;
			}
		}
	}

	void ndController::Debug(ndConstraintDebugCallback& callback) const
	{
		// just debug one leg
		const ndEffectorInfo& leg = m_legs[0];
		leg.m_calf->DebugJoint(callback);
		leg.m_effector->DebugJoint(callback);
	}

	void ndController::ResetModel()
	{
		ndAssert(0);
	}

	void ndController::CreateAnimationBlendTree()
	{
		m_walkGait = ndSharedPtr<ndGeneratorWalkGait>(new ndGeneratorWalkGait(this));
		ndSharedPtr<ndAnimationSequence> walkSequence = (ndSharedPtr<ndAnimationSequence>&)m_walkGait;

		ndSharedPtr<ndAnimationBlendTreeNode> proceduralPoseGenerator (new ndAnimationSequencePlayer(walkSequence));
		ndSharedPtr<ndAnimationBlendTreeNode> controlNode(new ndBodySwingControl(proceduralPoseGenerator));
		m_animBlendTree = ndSharedPtr<ndBodySwingControl> ((ndSharedPtr<ndBodySwingControl>&)controlNode);
		
		for (ndInt32 i = 0; i < m_legs.GetCount(); ++i)
		{
			ndEffectorInfo& leg = m_legs[i];
			ndAnimKeyframe keyFrame;
			keyFrame.m_userData = &leg;
			m_pose.PushBack(keyFrame);
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
			ndSharedPtr<ndBody> calfBody(CreateRigidBody(calfMesh, calfEntity, D_LIMB_MASS, thighBody->GetAsBodyDynamic()));
			
			const ndMatrix calfMatrix(calfMesh->CalculateGlobalMatrix());
			ndSharedPtr<ndJointBilateralConstraint> calfHinge(new ndJointHinge(calfMatrix, calfBody->GetAsBodyKinematic(), thighBody->GetAsBodyKinematic()));
			ndModelArticulation::ndNode* const calfNode = model->AddLimb(thighNode, calfBody, calfHinge);
			
			((ndIkJointHinge*)*calfHinge)->SetLimitState(true);
			((ndIkJointHinge*)*calfHinge)->SetLimits(-80.0f * ndDegreeToRad, 60.0f * ndDegreeToRad);
			
			// build heel
			ndSharedPtr<ndMesh> heelMesh(calfMesh->GetChildren().GetFirst()->GetInfo());
			ndSharedPtr<ndRenderSceneNode> heelEntity(calfEntity->FindByName(heelMesh->GetName())->GetSharedPtr());
			ndSharedPtr<ndBody> heelBody(CreateRigidBody(heelMesh, heelEntity, D_LIMB_MASS * 0.5f, calfBody->GetAsBodyDynamic()));

			const ndMatrix heelMatrix(heelMesh->CalculateGlobalMatrix());
			ndSharedPtr<ndJointBilateralConstraint> heelHinge(new ndJointHinge(heelMatrix, heelBody->GetAsBodyKinematic(), calfBody->GetAsBodyKinematic()));
			ndModelArticulation::ndNode* const heelNode = model->AddLimb(calfNode, heelBody, heelHinge);
			((ndJointHinge*)*heelHinge)->SetAsSpringDamper(0.001f, 2000.0f, 50.0f);

			// create effector
			ndAssert(heelMesh->FindByClosestMatch("-effector"));
			ndSharedPtr<ndMesh>footMesh(heelMesh->FindByClosestMatch("-effector")->GetSharedPtr());
			const ndVector effectOffset(footMesh->CalculateGlobalMatrix().m_posit);
			ndMatrix effectPivot(rootBody->GetMatrix());
			effectPivot.m_posit = thighMatrix.m_posit;
			
			ndFloat32 regularizer = ndFloat32(0.001f);
			ndFloat32 effectorStrength = ndAbs(ndFloat32(500.0f * D_LIMB_MASS * DEMO_GRAVITY));
			ndSharedPtr<ndJointBilateralConstraint> effector(new ndIkSwivelPositionEffector(effectPivot, rootBody->GetAsBodyKinematic(), effectOffset, heelBody->GetAsBodyKinematic()));
			((ndIkSwivelPositionEffector*)*effector)->SetLinearSpringDamper(regularizer, ndFloat32(4000.0f), ndFloat32(50.0f));
			((ndIkSwivelPositionEffector*)*effector)->SetAngularSpringDamper(regularizer, ndFloat32(4000.0f), ndFloat32(50.0f));
			((ndIkSwivelPositionEffector*)*effector)->SetWorkSpaceConstraints(ndFloat32(0.0f), ndFloat32(0.75f * 0.9f));
			((ndIkSwivelPositionEffector*)*effector)->SetMaxForce(effectorStrength);
			((ndIkSwivelPositionEffector*)*effector)->SetMaxTorque(effectorStrength);
			model->AddCloseLoop(effector);

			// build soft contact heel
			ndAssert(heelMesh->FindByClosestMatch("-capsule"));
			ndSharedPtr<ndMesh> contactMesh(heelMesh->FindByClosestMatch("-capsule")->GetSharedPtr());
			ndSharedPtr<ndRenderSceneNode> contactEntity(heelEntity->FindByName(contactMesh->GetName())->GetSharedPtr());
			ndSharedPtr<ndBody> contact(CreateRigidBody(contactMesh, contactEntity, D_LIMB_MASS * 0.5f, heelBody->GetAsBodyDynamic()));
			
			const ndMatrix contactMatrix(contactMesh->CalculateGlobalMatrix());
			const ndMatrix contactAxis(ndRollMatrix(ndFloat32(90.0f) * ndDegreeToRad) * contactMatrix);
			ndSharedPtr<ndJointBilateralConstraint> softContact(new ndJointSlider(contactAxis, contact->GetAsBodyKinematic(), heelBody->GetAsBodyKinematic()));
			model->AddLimb(heelNode, contact, softContact);
			((ndJointSlider*)*softContact)->SetLimitState(true);
			((ndJointSlider*)*softContact)->SetLimits(-ndFloat32(0.05f), ndFloat32(0.05f));
			((ndJointSlider*)*softContact)->SetAsSpringDamper(0.002f, 2000.0f, 100.0f);
			
			// save leg info
			ndEffectorInfo leg;
			leg.m_calf = (ndJointHinge*)*calfHinge;
			leg.m_heel = (ndJointHinge*)*heelHinge;
			leg.m_effector = (ndIkSwivelPositionEffector*)*effector;
			m_legs.PushBack(leg);
		}

		// add a camera node.
		const ndVector cameraPivot(0.0f, 0.5f, 0.0f, 0.0f);
		ndRender* const renderer = *scene->GetRenderer();
		m_cameraNode = ndSharedPtr<ndRenderSceneNode>(new ndDemoCameraNodeFollow(renderer, cameraPivot, -3.0f));
		visualMesh->AddChild(m_cameraNode);

		ndWorld* const world = scene->GetWorld();
		const ndMatrix upMatrix(rootBody->GetMatrix());
		ndSharedPtr<ndJointBilateralConstraint> upVector(new ndJointUpVector(upMatrix.m_up, rootBody->GetAsBodyKinematic(), world->GetSentinelBody()));
		//ndSharedPtr<ndJointBilateralConstraint> upVector(new ndJointFix6dof(upMatrix, rootBody->GetAsBodyKinematic(), world->GetSentinelBody()));
		//model->AddCloseLoop(upVector);
	}

	ndSharedPtr<ndModel> ndController::CreateModel(ndDemoEntityManager* const scene, const ndMatrix& location, const ndRenderMeshLoader& loader)
	{
		ndMatrix matrix(location);
		matrix.m_posit = FindFloor(*scene->GetWorld(), matrix.m_posit, 200.0f);
		matrix.m_posit.m_y += ndFloat32(0.5f);
		const ndMatrix saveMatrix(loader.m_mesh->m_matrix);
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

		loader.m_mesh->m_matrix = saveMatrix;
		return model;
	}

	class ndAnimatedHelper : public ndDemoEntityManager::ndDemoHelper
	{
		virtual void PresentHelp(ndDemoEntityManager* const scene) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "");
		}
	};

	class ndAnimatedControlPanel : public ndDemoEntityManager::ndDemoUIpanel
	{
		public:
		ndAnimatedControlPanel(ndSharedPtr<ndModel>& model)
			:ndDemoUIpanel()
		{
			m_playerController = (ndController*)(*model->GetNotifyCallback());
		}

		virtual void Update(ndDemoEntityManager* const) override
		{
			ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
			ndBodySwingControl* const swingControl = (ndBodySwingControl*)*m_playerController->m_animBlendTree;

			ImGui::SliderFloat("Swaying x", &swingControl->m_x, -D_HIP_SWAY_X, D_HIP_SWAY_X);
			ImGui::SliderFloat("Swaying z", &swingControl->m_z, -D_HIP_SWAY_Z, D_HIP_SWAY_Z);


			//ndModelArticulation* xxx = m_playerController->GetModel()->GetAsModelArticulation();;
			//const ndList<ndModelArticulation::ndNode>& list = xxx->GetCloseLoops();
			//ndJointUpVector* xxx1 = (ndJointUpVector*) *list.GetLast()->GetInfo().m_joint;
			//ndVector xxxxxx (xxx1->GetTorqueBody0());
			//ndTrace (("toque: %f %f %f\n", xxxxxx.m_x, xxxxxx.m_y, xxxxxx.m_z))
		}

		ndController* m_playerController;
	};
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
	ndSharedPtr<ndModel> model (ndController::CreateModel(scene, matrix, loader));

	ndSharedPtr<ndDemoEntityManager::ndDemoUIpanel> controlPanel(new ndAnimatedControlPanel(model));
	scene->SetDemoUIpanel(controlPanel);

#if 0
	ndMatrix matrix1(ndYawMatrix (45.0f * ndDegreeToRad) * matrix);
	matrix1.m_posit.m_x += 2.0f;
	matrix1.m_posit.m_z += 3.0f;
	ndController::CreateModel(scene, matrix1, loader);

	matrix1 = ndYawMatrix(-45.0f * ndDegreeToRad) * matrix;
	matrix1.m_posit.m_x += 2.0f;
	matrix1.m_posit.m_z -= 3.0f;
	//ndController::CreateModel(scene, matrix1, loader);

	matrix1 = ndYawMatrix(-45.0f * ndDegreeToRad) * matrix;
	matrix1.m_posit.m_x += 2.0f;
	matrix1.m_posit.m_z -= 0.0f;
	//ndController::CreateModel(scene, matrix1, loader);
#endif

	ndQuaternion rot(matrix);
	ndVector origin(-3.0f, 1.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);

	ndController* const controller = (ndController*)*model->GetNotifyCallback();
	ndRender* const renderer = *scene->GetRenderer();
	renderer->SetCamera(controller->m_cameraNode);
}