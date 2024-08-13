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

#if 0
	class ndIndustrialRobot : public ndModel
	{
		public:
		D_CLASS_REFLECTION(ndSimpleRobot::ndIndustrialRobot, ndModel)

		ndIndustrialRobot(ndDemoEntityManager* const scene, ndDemoEntity* const robotMesh, const ndMatrix& location)
			:ndModel()
			,m_rootBody(nullptr)
			,m_leftGripper(nullptr)
			,m_rightGripper(nullptr)
			,m_effector(nullptr)
			,m_effectorOffset(ndVector::m_wOne)
			,m_x(0.0f)
			,m_y(0.0f)
			,m_azimuth(0.0f)
			,m_gripperPosit(0.0f)
			,m_pitch(0.0f)
			,m_yaw(0.0f)
			,m_roll(0.0f)
		{
			// make a clone of the mesh and add it to the scene
			ndDemoEntity* const rootEntity = robotMesh->CreateClone();
			scene->AddEntity(rootEntity);
			ndWorld* const world = scene->GetWorld();
			
			// find the floor location 
			ndMatrix matrix(location);
			ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
			matrix.m_posit.m_y = floor.m_y;
			
			//matrix.m_posit.m_y += 1.0f;
			rootEntity->ResetMatrix(matrix);
			
			// add the root body
			m_rootBody = CreateBodyPart(scene, rootEntity, jointsDefinition[0].m_mass, nullptr);
			m_bodyArray.PushBack(m_rootBody);
			
			ndFixSizeArray<ndDemoEntity*, 32> childEntities;
			ndFixSizeArray<ndBodyDynamic*, 32> parentBone;
			
			ndInt32 stack = 0;
			for (ndDemoEntity* child = rootEntity->GetFirstChild(); child; child = child->GetNext())
			{
				childEntities[stack] = child;
				parentBone[stack] = m_rootBody;
				stack++;
			}
			
			const ndInt32 definitionCount = ndInt32(sizeof(jointsDefinition) / sizeof(jointsDefinition[0]));
			while (stack)
			{
				stack--;
				ndBodyDynamic* parentBody = parentBone[stack];
				ndDemoEntity* const childEntity = childEntities[stack];
			
				const char* const name = childEntity->GetName().GetStr();
				for (ndInt32 i = 0; i < definitionCount; ++i)
				{
					const ndDefinition& definition = jointsDefinition[i];
					if (!strcmp(definition.m_boneName, name))
					{
						ndTrace(("name: %s\n", name));
						if (definition.m_type == ndDefinition::m_hinge)
						{
							ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, definition.m_mass, parentBody);
							m_bodyArray.PushBack(childBody);
							const ndMatrix pivotMatrix(childBody->GetMatrix());
							ndJointHinge* const hinge = new ndJointHinge(pivotMatrix, childBody, parentBody);
							hinge->SetLimits(definition.m_minLimit, definition.m_maxLimit);
							if ((definition.m_minLimit > -1000.0f) && (definition.m_maxLimit < 1000.0f))
							{
								hinge->SetLimitState(true);
							}
							m_jointArray.PushBack(hinge);

							ndSharedPtr<ndJointBilateralConstraint> hingePtr(hinge);
							world->AddJoint(hingePtr);
							parentBody = childBody;
						}
						else if (definition.m_type == ndDefinition::m_slider)
						{
							ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, definition.m_mass, parentBody);
							m_bodyArray.PushBack(childBody);
			
							const ndMatrix pivotMatrix(childBody->GetMatrix());
							ndJointSlider* const slider = new ndJointSlider(pivotMatrix, childBody, parentBody);
							slider->SetLimits(definition.m_minLimit, definition.m_maxLimit);
							slider->SetAsSpringDamper(0.005f, 2000.0f, 200.0f);
			
							if (!strstr(definition.m_boneName, "Left"))
							{
								m_leftGripper = slider;
							}
							else
							{
								m_rightGripper = slider;
							}
							ndSharedPtr<ndJointBilateralConstraint> sliderPtr(slider);
							world->AddJoint(sliderPtr);
							
							parentBody = childBody;
						}
						else
						{
							ndBodyDynamic* const childBody = parentBody;
			
							const ndMatrix pivotFrame(rootEntity->Find("referenceFrame")->CalculateGlobalMatrix());
							const ndMatrix effectorFrame(childEntity->CalculateGlobalMatrix());
							m_effector = new ndIk6DofEffector(effectorFrame, pivotFrame, childBody, m_rootBody);
			
							m_effectorOffset = m_effector->GetOffsetMatrix().m_posit;
			
							ndFloat32 relaxation = 0.002f;
							m_effector->EnableRotationAxis(ndIk6DofEffector::m_shortestPath);
							m_effector->SetLinearSpringDamper(relaxation, 2000.0f, 200.0f);
							m_effector->SetAngularSpringDamper(relaxation, 2000.0f, 200.0f);
							m_effector->SetMaxForce(10000.0f);
							m_effector->SetMaxTorque(10000.0f);
			
							// the effector is part of the rig
							ndSharedPtr<ndJointBilateralConstraint> effectorPtr(m_effector);
							world->AddJoint(effectorPtr);
						}
						break;
					}
				}
			
				for (ndDemoEntity* child = childEntity->GetFirstChild(); child; child = child->GetNext())
				{
					childEntities[stack] = child;
					parentBone[stack] = parentBody;
					stack++;
				}
			}
		}

		~ndIndustrialRobot()
		{
		}

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

			// add body to the world
			ndSharedPtr<ndBody> bodyPtr(body);
			scene->GetWorld()->AddBody(bodyPtr);

			return body->GetAsBodyDynamic();
		}

		ndBodyDynamic* GetRoot() const
		{
			return m_rootBody;
		}

		void Debug(ndConstraintDebugCallback& context) const
		{
			if (m_effector)
			{
				((ndJointBilateralConstraint*)m_effector)->DebugJoint(context);
			}
		}

		void PostUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			ndModel::PostUpdate(world, timestep);
		}

		void PostTransformUpdate(ndWorld* const world, ndFloat32 timestep)
		{
			ndModel::PostTransformUpdate(world, timestep);
		}

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			ndModel::Update(world, timestep);
			if (m_effector)
			{
				// apply target position collected by control panel
				ndMatrix targetMatrix(
					ndRollMatrix(90.0f * ndDegreeToRad) *
					ndPitchMatrix(m_pitch * ndDegreeToRad) *
					ndYawMatrix(m_yaw * ndDegreeToRad) *
					ndRollMatrix(m_roll * ndDegreeToRad) *
					ndRollMatrix(-90.0f * ndDegreeToRad));

				ndVector localPosit(m_x, m_y, 0.0f, 0.0f);
				const ndMatrix aximuthMatrix(ndYawMatrix(m_azimuth * ndDegreeToRad));
				targetMatrix.m_posit = aximuthMatrix.TransformVector(m_effectorOffset + localPosit);

				m_effector->SetOffsetMatrix(targetMatrix);
				m_leftGripper->SetOffsetPosit(-m_gripperPosit * 0.5f);
				m_rightGripper->SetOffsetPosit(-m_gripperPosit * 0.5f);
			}
		}

		ndBodyDynamic* m_rootBody;
		ndJointSlider* m_leftGripper;
		ndJointSlider* m_rightGripper;
		ndIk6DofEffector* m_effector;
		ndFixSizeArray<ndBodyDynamic*, 16> m_bodyArray;
		ndFixSizeArray<ndJointBilateralConstraint*, 16> m_jointArray;
		ndVector m_effectorOffset;
		ndReal m_x;
		ndReal m_y;
		ndReal m_azimuth;
		ndReal m_gripperPosit;
		ndReal m_pitch;
		ndReal m_yaw;
		ndReal m_roll;
	};
	
	class ndRobotUI : public ndUIEntity
	{
		public:
		ndRobotUI(ndDemoEntityManager* const scene, ndIndustrialRobot* const robot)
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

			bool change = false;
			ImGui::Text("position x");
			change = change | ImGui::SliderFloat("##x", &m_robot->m_x, 0.0f, 5.0f);
			ImGui::Text("position y");
			change = change | ImGui::SliderFloat("##y", &m_robot->m_y, -2.5f, 2.0f);
			ImGui::Text("azimuth");
			change = change | ImGui::SliderFloat("##azimuth", &m_robot->m_azimuth, -180.0f, 180.0f);
			
			ImGui::Text("gripper");
			change = change | ImGui::SliderFloat("##gripper", &m_robot->m_gripperPosit, -0.2f, 0.4f);
			
			ImGui::Text("pitch");
			change = change | ImGui::SliderFloat("##pitch", &m_robot->m_pitch, -180.0f, 180.0f);
			ImGui::Text("yaw");
			change = change | ImGui::SliderFloat("##yaw", &m_robot->m_yaw, -180.0f, 180.0f);
			ImGui::Text("roll");
			change = change | ImGui::SliderFloat("##roll", &m_robot->m_roll, -180.0f, 180.0f);
			
			if (change)
			{
				m_robot->m_rootBody->SetSleepState(false);
			}
		}

		ndIndustrialRobot* m_robot;
	};
#endif

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

	ndModelArticulation* BuildModel(ndDemoEntityManager* const scene, ndDemoEntity* const modelMesh, const ndMatrix& location)
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
				
		////matrix.m_posit.m_y += 1.0f;
		rootEntity->ResetMatrix(matrix);

		//// add the root body
		ndSharedPtr<ndBody> rootBody(CreateBodyPart(scene, rootEntity, jointsDefinition[0].m_mass, nullptr));

		rootBody->SetMatrix(rootEntity->CalculateGlobalMatrix());

		// add body to the world
		//world->AddBody(rootBody);

		// add the root body to the model
		ndModelArticulation::ndNode* const modelNode = model->AddRootBody(rootBody);
		
		ndInt32 stack = 0;

		ndFixSizeArray<ndDemoEntity*, 32> childEntities;
		ndFixSizeArray<ndModelArticulation::ndNode*, 32> parentBones;
		for (ndDemoEntity* child = rootEntity->GetFirstChild(); child; child = child->GetNext())
		{
			childEntities[stack] = child;
			parentBones[stack] = modelNode;
			stack++;
		}
		
		const ndInt32 definitionCount = ndInt32(sizeof(jointsDefinition) / sizeof(jointsDefinition[0]));
		while (stack)
		{
			stack--;
			ndDemoEntity* const childEntity = childEntities[stack];
			ndModelArticulation::ndNode* parentBone = parentBones[stack];

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
						//world->AddBody(childBody);
						
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
						sliderJoint->SetAsSpringDamper(0.005f, 2000.0f, 200.0f);
						
						if (!strstr(definition.m_boneName, "Left"))
						{
							//m_leftGripper = slider;
						}
						else
						{
							//m_rightGripper = slider;
						}
						//world->AddJoint(jointPtr);
						parentBone = model->AddLimb(parentBone, childBody, slider);
					}
					else if (definition.m_type == ndDefinition::m_effector)
					{
						ndBodyDynamic* const childBody = parentBone->m_body->GetAsBodyDynamic();
						
						const ndMatrix pivotFrame(rootEntity->Find("referenceFrame")->CalculateGlobalMatrix());
						const ndMatrix effectorFrame(childEntity->CalculateGlobalMatrix());
						
						ndSharedPtr<ndJointBilateralConstraint> effector (new ndIk6DofEffector(effectorFrame, pivotFrame, childBody, modelNode->m_body->GetAsBodyKinematic()));
						
						ndIk6DofEffector* const effectorJoint = (ndIk6DofEffector*)*effector;
						ndFloat32 relaxation = 0.002f;
						//m_effectorOffset = m_effector->GetOffsetMatrix().m_posit;
						effectorJoint->EnableRotationAxis(ndIk6DofEffector::m_shortestPath);
						effectorJoint->SetLinearSpringDamper(relaxation, 2000.0f, 200.0f);
						effectorJoint->SetAngularSpringDamper(relaxation, 2000.0f, 200.0f);
						effectorJoint->SetMaxForce(10000.0f);
						effectorJoint->SetMaxTorque(10000.0f);
						
						// the effector is part of the rig
						model->AddCloseLoop(effector);
					}
					break;
				}
			}
		
			for (ndDemoEntity* child = childEntity->GetFirstChild(); child; child = child->GetNext())
			{
				childEntities[stack] = child;
				parentBones[stack] = parentBone;
				stack++;
			}
		}

		return model;
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
	
	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(ndYawMatrix(-90.0f * ndDegreeToRad));

	//ndIndustrialRobot* const robot = new ndIndustrialRobot(scene, *modelMesh, matrix);
	//scene->SetSelectedModel(robot);
	//
	//ndSharedPtr<ndModel> robotPtr(robot);
	//ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(robot->GetRoot()->GetMatrix(), robot->GetRoot(), floor));
	//world->AddModel(robotPtr);
	//world->AddJoint (fixJoint);
	//
	//ndRobotUI* const robotUI = new ndRobotUI(scene, robot);
	//ndSharedPtr<ndUIEntity> robotUIPtr(robotUI);
	//scene->Set2DDisplayRenderFunction(robotUIPtr);
	//
	//ndMatrix location(matrix * ndYawMatrix(90.0f * ndDegreeToRad));
	//location.m_posit.m_x += 1.5f;
	//location.m_posit.m_z += 1.5f;
	//AddBox(scene, location, 2.0f, 0.3f, 0.4f, 0.7f);
	//AddBox(scene, location, 1.0f, 0.3f, 0.4f, 0.7f);
	//
	//location = ndYawMatrix(90.0f * ndDegreeToRad) * location;
	//location.m_posit.m_x += 1.0f;
	//location.m_posit.m_z += 0.5f;
	//AddBox(scene, location, 8.0f, 0.3f, 0.4f, 0.7f);
	//AddBox(scene, location, 4.0f, 0.3f, 0.4f, 0.7f);

	ndSharedPtr<ndModel> model(BuildModel(scene, *modelMesh, matrix));
	ndModelArticulation* const robot = (ndModelArticulation*)*model;

	ndBodyKinematic* const baseBody = robot->GetRoot()->m_body->GetAsBodyKinematic();
	ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(baseBody->GetMatrix(), baseBody, floor));
	world->AddModel(model);
	world->AddJoint (fixJoint);

	matrix.m_posit.m_x -= 5.0f;
	matrix.m_posit.m_y += 2.0f;
	matrix.m_posit.m_z += 5.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 45.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
