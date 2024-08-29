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
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

namespace ndSimpleRobot
{
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
	};

	static ndDefinition jointsDefinition[] =
	{
		{ "base", ndDefinition::m_root, 100.0f, 0.0f, 0.0f},
		{ "base_rotator", ndDefinition::m_hinge, 50.0f, -1.0e10f, 1.0e10f},
		{ "arm_0", ndDefinition::m_hinge , 5.0f, -140.0f * ndDegreeToRad, 1.0f * ndDegreeToRad},
		{ "arm_1", ndDefinition::m_hinge , 5.0f, -30.0f * ndDegreeToRad, 110.0f * ndDegreeToRad},
		{ "arm_2", ndDefinition::m_hinge , 5.0f, -1.0e10f, 1.0e10f},
		{ "arm_3", ndDefinition::m_hinge , 3.0f, -1.0e10f, 1.0e10f},
		{ "arm_4", ndDefinition::m_hinge , 2.0f, -1.0e10f, 1.0e10f},
		{ "gripperLeft", ndDefinition::m_slider , 1.0f, -0.2f, 0.03f},
		{ "gripperRight", ndDefinition::m_slider , 1.0f, -0.2f, 0.03f},
		{ "effector", ndDefinition::m_effector , 0.0f, 0.0f, 0.0f},
	};

	class RobotModelNotify : public ndModelNotify
	{
		public:
		RobotModelNotify(ndModelArticulation* const robot, bool showDebug)
			:ndModelNotify()
			,m_world(nullptr)
			,m_x(0.0f)
			,m_y(0.0f)
			,m_azimuth(0.0f)
			,m_yaw(0.0f)
			,m_pitch(0.0f)
			,m_roll(0.0f)
			,m_gripperPosit(0.0f)
			,m_timestep(ndFloat32(0.0f))
			,m_showDebug(showDebug)
		{
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
		}

		ndModelNotify* Clone() const
		{
			return new RobotModelNotify(*this);
		}

		void Init(ndModelArticulation* const robot)
		{
			m_rootBody = robot->GetRoot()->m_body->GetAsBodyDynamic();
			m_leftGripper = (ndJointSlider*)robot->FindByName("leftGripper")->m_joint->GetAsBilateral();
			m_rightGripper = (ndJointSlider*)robot->FindByName("rightGripper")->m_joint->GetAsBilateral();
			m_effector = (ndIk6DofEffector*)robot->FindLoopByName("effector")->m_joint->GetAsBilateral();
			m_effectorOffset = m_effector->GetOffsetMatrix().m_posit;
		}

		void ResetModel()
		{
			ndTrace(("Reset Model\n"));
		}

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			m_world = world;
			m_timestep = timestep;

			m_leftGripper->SetOffsetPosit(-m_gripperPosit * 0.5f);
			m_rightGripper->SetOffsetPosit(-m_gripperPosit * 0.5f);

			// apply target position collected by control panel
			ndMatrix targetMatrix(
				ndRollMatrix(90.0f * ndDegreeToRad) *
				ndPitchMatrix(m_pitch * ndDegreeToRad) *
				ndYawMatrix(m_yaw * ndDegreeToRad) *
				ndRollMatrix(m_roll * ndDegreeToRad) *
				ndRollMatrix(-90.0f * ndDegreeToRad));

			ndVector localPosit(m_x, m_y, 0.0f, 0.0f);
			const ndMatrix aximuthMatrix(ndYawMatrix(m_azimuth));
			targetMatrix.m_posit = aximuthMatrix.TransformVector(m_effectorOffset + localPosit);
			m_effector->SetOffsetMatrix(targetMatrix);

			//xxxx0 = m_effector->GetEffectorMatrix();
		}

		void PostUpdate(ndWorld* const, ndFloat32)
		{
			//xxxx1 = m_effector->GetEffectorMatrix();
			//
			//ndFloat32 azimuth = 0.0f;
			//const ndVector posit(xxxx1.m_posit);
			//if ((posit.m_x * posit.m_x + posit.m_z * posit.m_z) > 1.0e-3f)
			//{
			//	azimuth = ndAtan2(-posit.m_z, posit.m_x);
			//}
			//const ndMatrix aximuthMatrix(ndYawMatrix(azimuth));
			//const ndVector currenPosit(aximuthMatrix.UnrotateVector(posit) - m_effectorOffset);
			//ndAssert(currenPosit.m_x >= -1.0e-2f);
			//const ndVector currenPosit1(aximuthMatrix.UnrotateVector(posit) - m_effectorOffset);
		}

		void PostTransformUpdate(ndWorld* const, ndFloat32)
		{
		}

		//void Debug(ndConstraintDebugCallback& context) const
		void Debug(ndConstraintDebugCallback&) const
		{
			//ndTrace(("xxxxx\n"));
			//if (!m_showDebug)
			//{
			//	return;
			//}
			//
			//ndModelArticulation* const model = GetModel()->GetAsModelArticulation();
			//
			//ndFixSizeArray<const ndBodyKinematic*, 32> bodies;
			//
			//ndFloat32 totalMass = ndFloat32(0.0f);
			//ndVector centerOfMass(ndVector::m_zero);
			//for (ndModelArticulation::ndNode* node = model->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
			//{
			//	const ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
			//	const ndMatrix matrix(body->GetMatrix());
			//	ndFloat32 mass = body->GetMassMatrix().m_w;
			//	totalMass += mass;
			//	centerOfMass += matrix.TransformVector(body->GetCentreOfMass()).Scale(mass);
			//	bodies.PushBack(body);
			//}
			//ndFloat32 invMass = 1.0f / totalMass;
			//centerOfMass = centerOfMass.Scale(invMass);
			//
			//ndVector comLineOfAction(centerOfMass);
			//comLineOfAction.m_y -= ndFloat32(0.5f);
			//context.DrawLine(centerOfMass, comLineOfAction, ndVector::m_zero);
			//
			//ndBodyKinematic* const rootBody = model->GetRoot()->m_body->GetAsBodyKinematic();
			//const ndVector upVector(rootBody->GetMatrix().m_up);
			//ndFixSizeArray<ndBigVector, 16> supportPoint;
			//for (ndInt32 i = 0; i < m_animPose.GetCount(); ++i)
			//{
			//	const ndAnimKeyframe& keyFrame = m_animPose[i];
			//	ndEffectorInfo* const info = (ndEffectorInfo*)keyFrame.m_userData;
			//	ndIkSwivelPositionEffector* const effector = (ndIkSwivelPositionEffector*)*info->m_effector;
			//	if (i == 0)
			//	{
			//		effector->DebugJoint(context);
			//	}
			//
			//	if (keyFrame.m_userParamFloat < 1.0f)
			//	{
			//		ndBodyKinematic* const body = effector->GetBody0();
			//		supportPoint.PushBack(body->GetMatrix().TransformVector(effector->GetLocalMatrix0().m_posit));
			//	}
			//}
			//
			//ndVector supportColor(0.0f, 1.0f, 1.0f, 1.0f);
			//if (supportPoint.GetCount() >= 3)
			//{
			//	ScaleSupportShape(supportPoint);
			//	ndFixSizeArray<ndVector, 16> desiredSupportPoint;
			//	for (ndInt32 i = 0; i < supportPoint.GetCount(); ++i)
			//	{
			//		desiredSupportPoint.PushBack(supportPoint[i]);
			//	}
			//
			//	ndMatrix rotation(ndPitchMatrix(90.0f * ndDegreeToRad));
			//	rotation.TransformTriplex(&desiredSupportPoint[0].m_x, sizeof(ndVector), &desiredSupportPoint[0].m_x, sizeof(ndVector), desiredSupportPoint.GetCount());
			//	ndInt32 supportCount = ndConvexHull2d(&desiredSupportPoint[0], desiredSupportPoint.GetCount());
			//	rotation.OrthoInverse().TransformTriplex(&desiredSupportPoint[0].m_x, sizeof(ndVector), &desiredSupportPoint[0].m_x, sizeof(ndVector), desiredSupportPoint.GetCount());
			//	ndVector p0(desiredSupportPoint[supportCount - 1]);
			//	ndBigVector bigPolygon[16];
			//	for (ndInt32 i = 0; i < supportCount; ++i)
			//	{
			//		bigPolygon[i] = desiredSupportPoint[i];
			//		context.DrawLine(desiredSupportPoint[i], p0, supportColor);
			//		p0 = desiredSupportPoint[i];
			//	}
			//
			//	ndBigVector p0Out;
			//	ndBigVector p1Out;
			//	ndBigVector ray_p0(centerOfMass);
			//	ndBigVector ray_p1(comLineOfAction);
			//	ndRayToPolygonDistance(ray_p0, ray_p1, bigPolygon, supportCount, p0Out, p1Out);
			//
			//	const ndVector centerOfPresure(p0Out);
			//	context.DrawPoint(centerOfPresure, ndVector(0.0f, 0.0f, 1.0f, 1.0f), 5);
			//
			//	ndVector zmp(CalculateZeroMomentPoint());
			//	ray_p0 = zmp;
			//	ray_p1 = zmp;
			//	ray_p1.m_y -= ndFloat32(0.5f);
			//	ndRayToPolygonDistance(ray_p0, ray_p1, bigPolygon, supportCount, p0Out, p1Out);
			//	const ndVector zmpSupport(p0Out);
			//	context.DrawPoint(zmpSupport, ndVector(1.0f, 0.0f, 0.0f, 1.0f), 5);
			//}
			//else if (supportPoint.GetCount() == 2)
			//{
			//	ndTrace(("xxxxxxxxxx\n"));
			//	context.DrawLine(supportPoint[0], supportPoint[1], supportColor);
			//	//ndBigVector p0Out;
			//	//ndBigVector p1Out;
			//	//ndBigVector ray_p0(comMatrix.m_posit);
			//	//ndBigVector ray_p1(comMatrix.m_posit);
			//	//ray_p1.m_y -= 1.0f;
			//	//
			//	//ndRayToRayDistance(ray_p0, ray_p1, contactPoints[0], contactPoints[1], p0Out, p1Out);
			//	//context.DrawPoint(p0Out, ndVector(1.0f, 0.0f, 0.0f, 1.0f), 3);
			//	//context.DrawPoint(p1Out, ndVector(0.0f, 1.0f, 0.0f, 1.0f), 3);
			//}
		}

		//ndMatrix xxxx0;
		//ndMatrix xxxx1;
		ndVector m_effectorOffset;
		ndBodyDynamic* m_rootBody;
		ndJointSlider* m_leftGripper;
		ndJointSlider* m_rightGripper;
		ndIk6DofEffector* m_effector;
		ndWorld* m_world;

		ndReal m_x;
		ndReal m_y;
		ndReal m_azimuth;
		ndReal m_yaw;
		ndReal m_pitch;
		ndReal m_roll;
		ndReal m_gripperPosit;
		ndFloat32 m_timestep;
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
			change = change | ndInt8(ImGui::SliderFloat("x", &m_robot->m_x, 0.0f, 5.0f));
			change = change | ndInt8(ImGui::SliderFloat("y", &m_robot->m_y, -2.5f, 2.0f));
			change = change | ndInt8(ImGui::SliderFloat("azimuth", &m_robot->m_azimuth, -ndPi, ndPi));
			change = change | ndInt8(ImGui::SliderFloat("gripper", &m_robot->m_gripperPosit, -0.2f, 0.4f));
			change = change | ndInt8(ImGui::SliderFloat("pitch", &m_robot->m_pitch, -180.0f, 180.0f));
			change = change | ndInt8(ImGui::SliderFloat("yaw", &m_robot->m_yaw, -180.0f, 180.0f));
			change = change | ndInt8(ImGui::SliderFloat("roll", &m_robot->m_roll, -180.0f, 180.0f));

			if (change)
			{
				m_robot->GetModel()->GetAsModelArticulation()->GetRoot()->m_body->GetAsBodyKinematic()->SetSleepState(false);
			}
		}
	
		RobotModelNotify* m_robot;
	};

	ndBodyDynamic* CreateBodyPart(ndDemoEntityManager* const scene, ndDemoEntity* const entityPart, ndFloat32 mass, ndBodyDynamic* const parentBone)
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
		return body->GetAsBodyDynamic();
	}

	ndModelArticulation* CreateModel(ndDemoEntityManager* const scene, ndDemoEntity* const modelMesh, const ndMatrix& location)
	{
		// make a clone of the mesh and add it to the scene
		ndModelArticulation* const model = new ndModelArticulation();

		ndWorld* const world = scene->GetWorld();
		ndDemoEntity* const entity = modelMesh->CreateClone();
		scene->AddEntity(entity);

		ndDemoEntity* const rootEntity = (ndDemoEntity*)entity->Find(jointsDefinition[0].m_boneName);
		ndMatrix matrix(rootEntity->CalculateGlobalMatrix() * location);

		// find the floor location 
		ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		matrix.m_posit.m_y = floor.m_y;
				
		rootEntity->ResetMatrix(matrix);

		// add the root body
		ndSharedPtr<ndBody> rootBody(CreateBodyPart(scene, rootEntity, jointsDefinition[0].m_mass, nullptr));

		rootBody->SetMatrix(rootEntity->CalculateGlobalMatrix());

		// add the root body to the model
		ndModelArticulation::ndNode* const modelNode = model->AddRootBody(rootBody);
		
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
					ndTrace(("name: %s\n", name));
					if (definition.m_type == ndDefinition::m_hinge)
					{
						ndSharedPtr<ndBody> childBody (CreateBodyPart(scene, childEntity, definition.m_mass, parentBone->m_body->GetAsBodyDynamic()));
						const ndMatrix pivotMatrix(childBody->GetMatrix());
						ndSharedPtr<ndJointBilateralConstraint> hinge (new ndJointHinge(pivotMatrix, childBody->GetAsBodyKinematic(), parentBone->m_body->GetAsBodyKinematic()));

						ndJointHinge* const hingeJoint = (ndJointHinge*)*hinge;
						hingeJoint->SetLimits(definition.m_minLimit, definition.m_maxLimit);
						if ((definition.m_minLimit > -1000.0f) && (definition.m_maxLimit < 1000.0f))
						{
							hingeJoint->SetLimitState(true);
						}

						parentBone = model->AddLimb(parentBone, childBody, hinge);
					}
					else if (definition.m_type == ndDefinition::m_slider)
					{
						ndSharedPtr<ndBody> childBody(CreateBodyPart(scene, childEntity, definition.m_mass, parentBone->m_body->GetAsBodyDynamic()));
						
						const ndMatrix pivotMatrix(childBody->GetMatrix());
						ndSharedPtr<ndJointBilateralConstraint> slider (new ndJointSlider(pivotMatrix, childBody->GetAsBodyKinematic(), parentBone->m_body->GetAsBodyKinematic()));

						ndJointSlider* const sliderJoint = (ndJointSlider*)*slider;
						sliderJoint->SetLimits(definition.m_minLimit, definition.m_maxLimit);
						sliderJoint->SetAsSpringDamper(0.001f, 2000.0f, 100.0f);
						parentBone = model->AddLimb(parentBone, childBody, slider);

						if (!strstr(definition.m_boneName, "Left"))
						{
							parentBone->m_name = "leftGripper";
						}
						else
						{
							parentBone->m_name = "rightGripper";
						}
					}
					else if (definition.m_type == ndDefinition::m_effector)
					{
						ndBodyDynamic* const childBody = parentBone->m_body->GetAsBodyDynamic();
						
						const ndMatrix pivotFrame(rootEntity->Find("referenceFrame")->CalculateGlobalMatrix());
						const ndMatrix effectorFrame(childEntity->CalculateGlobalMatrix());
						
						ndSharedPtr<ndJointBilateralConstraint> effector (new ndIk6DofEffector(effectorFrame, pivotFrame, childBody, modelNode->m_body->GetAsBodyKinematic()));
						
						ndIk6DofEffector* const effectorJoint = (ndIk6DofEffector*)*effector;
						ndFloat32 relaxation = 1.0e-4f;
						effectorJoint->EnableRotationAxis(ndIk6DofEffector::m_shortestPath);
						effectorJoint->SetLinearSpringDamper(relaxation, 10000.0f, 500.0f);
						effectorJoint->SetAngularSpringDamper(relaxation, 10000.0f, 500.0f);
						effectorJoint->SetMaxForce(10000.0f);
						effectorJoint->SetMaxTorque(10000.0f);
						
						// the effector is part of the rig
						model->AddCloseLoop(effector, "effector");
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

		return model;
	}

	//class TrainingUpdata : public ndDemoEntityManager::OnPostUpdate
	//{
	//	public:
	//	TrainingUpdata(ndDemoEntityManager* const scene, const ndMatrix& matrix, ndSharedPtr<ndDemoEntity>& visualMesh, ndBodyKinematic* const floor)
	//		:OnPostUpdate()
	//		,m_master()
	//		,m_bestActor()
	//		,m_outFile(nullptr)
	//		,m_timer(ndGetTimeInMicroseconds())
	//		,m_maxScore(ndFloat32(-1.0e10f))
	//		,m_discountFactor(0.995f)
	//		,m_horizon(ndFloat32(1.0f) / (ndFloat32(1.0f) - m_discountFactor))
	//		,m_lastEpisode(-1)
	//		,m_stopTraining(100 * 1000000)
	//		,m_modelIsTrained(false)
	//	{
	//		//ndWorld* const world = scene->GetWorld();
	//		m_outFile = fopen("robotArmReach-vpg.csv", "wb");
	//		fprintf(m_outFile, "vpg\n");
	//		
	//		ndBrainAgentContinuePolicyGradient_TrainerMaster::HyperParameters hyperParameters;
	//		
	//		//hyperParameters.m_threadsCount = 1;
	//		hyperParameters.m_maxTrajectorySteps = 1024 * 8;
	//		hyperParameters.m_extraTrajectorySteps = 1024 * 2;
	//		hyperParameters.m_discountFactor = ndReal(m_discountFactor);
	//		hyperParameters.m_numberOfActions = ND_AGENT_OUTPUT_SIZE;
	//		hyperParameters.m_numberOfObservations = ND_AGENT_INPUT_SIZE;
	//		
	//		m_master = ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster>(new ndBrainAgentContinuePolicyGradient_TrainerMaster(hyperParameters));
	//		m_bestActor = ndSharedPtr<ndBrain>(new ndBrain(*m_master->GetActor()));
	//		m_master->SetName(CONTROLLER_NAME);
	//		
	//		auto SpawnModel = [this, scene, &visualMesh, floor](const ndMatrix& matrix)
	//		{
	//			ndWorld* const world = scene->GetWorld();
	//			ndModelArticulation* const model = CreateModel(scene, *visualMesh, matrix);
	//
	//			//SetMaterial(visualModel);
	//			model->SetNotifyCallback(new RobotModelNotify(m_master, model, true));
	//			model->AddToWorld(world);
	//			((RobotModelNotify*)*model->GetNotifyCallback())->ResetModel();
	//
	//			ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(model->GetRoot()->m_body->GetMatrix(), model->GetRoot()->m_body->GetAsBodyKinematic(), floor));
	//			world->AddJoint(fixJoint);
	//			return model;
	//		};
	//
	//		ndModelArticulation* const visualModel = SpawnModel(matrix);
	//		ndSharedPtr<ndUIEntity> robotUI(new ndRobotUI(scene, (RobotModelNotify*)*visualModel->GetNotifyCallback()));
	//		scene->Set2DDisplayRenderFunction(robotUI);
	//
	//		ndInt32 countX = 6;
	//		ndInt32 countZ = 9;
	//		countX = 0;
	//		countZ = 0;
	//		
	//		// add a hidden battery of model to generate trajectories in parallel
	//		for (ndInt32 i = 0; i < countZ; ++i)
	//		{
	//			for (ndInt32 j = 0; j < countX; ++j)
	//			{
	//		//		ndMatrix location(matrix);
	//		//		location.m_posit.m_x += 6.0f * (ndRand() - 0.5f);
	//		//		location.m_posit.m_z += 6.0f * (ndRand() - 0.5f);
	//		//
	//		//		ndModelArticulation* const model = CreateModel(scene, location);
	//		//		model->SetNotifyCallback(new RobotModelNotify(m_master, model, false));
	//		//		model->AddToWorld(world);
	//		//		m_models.Append(model);
	//		//		//HideModel(model);
	//		//		SetMaterial(model);
	//		//		((RobotModelNotify*)*model->GetNotifyCallback())->ResetModel();
	//			}
	//		}
	//		//scene->SetAcceleratedUpdate();
	//	}
	//
	//	~TrainingUpdata()
	//	{
	//		if (m_outFile)
	//		{
	//			fclose(m_outFile);
	//		}
	//	}
	//
	//	void HideModel(ndModelArticulation* const robot, bool mode) const
	//	{
	//		ndModelArticulation::ndNode* stackMem[128];
	//		ndInt32 stack = 1;
	//		stackMem[0] = robot->GetRoot();
	//		while (stack)
	//		{
	//			stack--;
	//			ndModelArticulation::ndNode* const node = stackMem[stack];
	//			ndBody* const body = *node->m_body;
	//			ndDemoEntityNotify* const userData = (ndDemoEntityNotify*)body->GetNotifyCallback();
	//			ndDemoEntity* const ent = userData->m_entity;
	//			mode ? ent->Hide() : ent->UnHide();
	//
	//			for (ndModelArticulation::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
	//			{
	//				stackMem[stack] = child;
	//				stack++;
	//			}
	//		}
	//	}
	//
	//	void OnDebug(ndDemoEntityManager* const, bool mode)
	//	{
	//		for (ndList<ndModelArticulation*>::ndNode* node = m_models.GetFirst(); node; node = node->GetNext())
	//		{
	//			HideModel(node->GetInfo(), mode);
	//		}
	//	}
	//
	//	class TrainingRobotBodyNotify : public ndDemoEntityNotify
	//	{
	//		public:
	//		TrainingRobotBodyNotify(const ndDemoEntityNotify* const src)
	//			:ndDemoEntityNotify(*src)
	//		{
	//		}
	//
	//		virtual bool OnSceneAabbOverlap(const ndBody* const otherBody) const
	//		{
	//			const ndBodyKinematic* const body0 = ((ndBody*)GetBody())->GetAsBodyKinematic();
	//			const ndBodyKinematic* const body1 = ((ndBody*)otherBody)->GetAsBodyKinematic();
	//			const ndShapeInstance& instanceShape0 = body0->GetCollisionShape();
	//			const ndShapeInstance& instanceShape1 = body1->GetCollisionShape();
	//			return instanceShape0.m_shapeMaterial.m_userId != instanceShape1.m_shapeMaterial.m_userId;
	//		}
	//	};
	//
	//	void SetMaterial(ndModelArticulation* const robot) const
	//	{
	//		ndFixSizeArray<ndModelArticulation::ndNode*, 256> stack;
	//
	//		stack.PushBack(robot->GetRoot());
	//		while (stack.GetCount())
	//		{
	//			ndModelArticulation::ndNode* const node = stack.Pop();
	//			ndBodyKinematic* const body = node->m_body->GetAsBodyKinematic();
	//
	//			ndShapeInstance& instanceShape = body->GetCollisionShape();
	//			instanceShape.m_shapeMaterial.m_userId = ndDemoContactCallback::m_modelPart;
	//
	//			ndDemoEntityNotify* const originalNotify = (ndDemoEntityNotify*)body->GetNotifyCallback();
	//			void* const useData = originalNotify->m_entity;
	//			originalNotify->m_entity = nullptr;
	//			TrainingRobotBodyNotify* const notify = new TrainingRobotBodyNotify((TrainingRobotBodyNotify*)body->GetNotifyCallback());
	//			body->SetNotifyCallback(notify);
	//			notify->m_entity = (ndDemoEntity*)useData;
	//
	//			for (ndModelArticulation::ndNode* child = node->GetFirstChild(); child; child = child->GetNext())
	//			{
	//				stack.PushBack(child);
	//			}
	//		}
	//	}
	//
	//	virtual void Update(ndDemoEntityManager* const manager, ndFloat32)
	//	{
	//		ndInt32 stopTraining = m_master->GetFramesCount();
	//		if (stopTraining <= m_stopTraining)
	//		{
	//			ndInt32 episodeCount = m_master->GetEposideCount();
	//			m_master->OptimizeStep();
	//
	//			episodeCount -= m_master->GetEposideCount();
	//			ndFloat32 rewardTrajectory = m_master->GetAverageFrames() * m_master->GetAverageScore();
	//			if (rewardTrajectory >= ndFloat32(m_maxScore))
	//			{
	//				if (m_lastEpisode != m_master->GetEposideCount())
	//				{
	//					m_maxScore = rewardTrajectory;
	//					m_bestActor->CopyFrom(*m_master->GetActor());
	//					ndExpandTraceMessage("best actor episode: %d\treward %f\ttrajectoryFrames: %f\n", m_master->GetEposideCount(), 100.0f * m_master->GetAverageScore() / m_horizon, m_master->GetAverageFrames());
	//					m_lastEpisode = m_master->GetEposideCount();
	//				}
	//			}
	//
	//			if (episodeCount && !m_master->IsSampling())
	//			{
	//				ndExpandTraceMessage("steps: %d\treward: %g\t  trajectoryFrames: %g\n", m_master->GetFramesCount(), 100.0f * m_master->GetAverageScore() / m_horizon, m_master->GetAverageFrames());
	//				if (m_outFile)
	//				{
	//					fprintf(m_outFile, "%g\n", m_master->GetAverageScore());
	//					fflush(m_outFile);
	//				}
	//			}
	//		}
	//
	//		if ((stopTraining >= m_stopTraining) || (100.0f * m_master->GetAverageScore() / m_horizon > 96.0f))
	//		{
	//			char fileName[1024];
	//			m_modelIsTrained = true;
	//			m_master->GetActor()->CopyFrom(*(*m_bestActor));
	//			ndGetWorkingFileName(m_master->GetName().GetStr(), fileName);
	//			m_master->GetActor()->SaveToFile(fileName);
	//			ndExpandTraceMessage("saving to file: %s\n", fileName);
	//			ndExpandTraceMessage("training complete\n");
	//			ndUnsigned64 timer = ndGetTimeInMicroseconds() - m_timer;
	//			ndExpandTraceMessage("training time: %g seconds\n", ndFloat32(ndFloat64(timer) * ndFloat32(1.0e-6f)));
	//			manager->Terminate();
	//		}
	//	}
	//
	//	ndSharedPtr<ndBrainAgentContinuePolicyGradient_TrainerMaster> m_master;
	//	ndSharedPtr<ndBrain> m_bestActor;
	//	ndList<ndModelArticulation*> m_models;
	//	FILE* m_outFile;
	//	ndUnsigned64 m_timer;
	//	ndFloat32 m_maxScore;
	//	ndFloat32 m_discountFactor;
	//	ndFloat32 m_horizon;
	//	ndInt32 m_lastEpisode;
	//	ndInt32 m_stopTraining;
	//	bool m_modelIsTrained;
	//};

	void AddBackgroundScene(ndDemoEntityManager* const scene, const ndMatrix& matrix)
	{
		ndMatrix location(matrix);
		location.m_posit.m_x += 1.5f;
		location.m_posit.m_z += 1.5f;
		AddBox(scene, location, 2.0f, 0.3f, 0.4f, 0.7f);
		AddBox(scene, location, 1.0f, 0.3f, 0.4f, 0.7f);
		
		location = ndYawMatrix(90.0f * ndDegreeToRad) * location;
		location.m_posit.m_x += 1.0f;
		location.m_posit.m_z += 0.5f;
		AddBox(scene, location, 8.0f, 0.3f, 0.4f, 0.7f);
		AddBox(scene, location, 4.0f, 0.3f, 0.4f, 0.7f);
	}
}

using namespace ndSimpleRobot;
void ndSimpleIndustrialRobot (ndDemoEntityManager* const scene)
{
	// build a floor
	ndBodyKinematic* const floor = BuildFloorBox(scene, ndGetIdentityMatrix());
	
	ndVector origin1(0.0f, 0.0f, 0.0f, 1.0f);
	ndMeshLoader loader;
	ndSharedPtr<ndDemoEntity> modelMesh(loader.LoadEntity("robot.fbx", scene));
	ndMatrix matrix(ndYawMatrix(-90.0f * ndDegreeToRad));

	AddBackgroundScene(scene, matrix);

	auto SpawnModel = [scene, &modelMesh, floor](const ndMatrix& matrix)
	{
		ndWorld* const world = scene->GetWorld();
		ndModelArticulation* const model = CreateModel(scene, *modelMesh, matrix);
		model->SetNotifyCallback(new RobotModelNotify(model, true));
		model->AddToWorld(world);
		((RobotModelNotify*)*model->GetNotifyCallback())->ResetModel();

		ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(model->GetRoot()->m_body->GetMatrix(), model->GetRoot()->m_body->GetAsBodyKinematic(), floor));
		world->AddJoint(fixJoint);
		return model;
	};

	ndModelArticulation* const visualModel = SpawnModel(matrix);
	ndSharedPtr<ndUIEntity> robotUI(new ndRobotUI(scene, (RobotModelNotify*)*visualModel->GetNotifyCallback()));
	scene->Set2DDisplayRenderFunction(robotUI);

	matrix.m_posit.m_x -= 5.0f;
	matrix.m_posit.m_y += 2.0f;
	matrix.m_posit.m_z += 5.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 45.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
