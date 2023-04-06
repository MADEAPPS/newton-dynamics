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
#include "ndDemoCamera.h"
#include "ndLoadFbxMesh.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

namespace ndAdvancedRobot
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
		{ "arm_1", ndDefinition::m_hinge , 5.0f, -30.0f * ndDegreeToRad, 120.0f * ndDegreeToRad},
		{ "arm_2", ndDefinition::m_hinge , 5.0f, -1.0e10f, 1.0e10f},
		{ "arm_3", ndDefinition::m_hinge , 3.0f, -1.0e10f, 1.0e10f},
		{ "arm_4", ndDefinition::m_hinge , 2.0f, -1.0e10f, 1.0e10f},
		{ "gripperLeft", ndDefinition::m_slider  , 1.0f, -0.2f, 0.03f},
		{ "gripperRight", ndDefinition::m_slider , 1.0f, -0.2f, 0.03f},
		{ "effector", ndDefinition::m_effector , 0.0f, 0.0f, 0.0f},
	};

	class ndIndustrialRobot : public ndModel
	{
		public:
		D_CLASS_REFLECTION(ndAdvancedRobot::ndIndustrialRobot, ndBodyDynamic)

		ndIndustrialRobot(ndDemoEntityManager* const scene, ndDemoEntity* const robotMesh, const ndMatrix& location)
			:ndModel()
			,m_rootBody(nullptr)
			,m_leftGripper(nullptr)
			,m_rightGripper(nullptr)
			,m_effector()
			,m_invDynamicsSolver()
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
						//dTrace(("name: %s\n", name));
						if (definition.m_type == ndDefinition::m_hinge)
						{
							ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, definition.m_mass, parentBody);
							m_bodyArray.PushBack(childBody);
							const ndMatrix pivotMatrix(childBody->GetMatrix());
			
							// here we use a ik joints instead of a regular one.
							ndIkJointHinge* const hinge = new ndIkJointHinge(pivotMatrix, childBody, parentBody);
							hinge->SetLimits(definition.m_minLimit, definition.m_maxLimit);
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
							slider->SetAsSpringDamper(0.01f, 2000.0f, 100.0f);
			
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
							m_effector = ndSharedPtr<ndIk6DofEffector>(new ndIk6DofEffector(effectorFrame, pivotFrame, childBody, m_rootBody));
			
							m_effectorOffset = m_effector->GetOffsetMatrix().m_posit;
			
							ndFloat32 relaxation = 0.003f;
							m_effector->EnableRotationAxis(ndIk6DofEffector::m_shortestPath);
							m_effector->SetLinearSpringDamper(relaxation, 1500.0f, 200.0f);
							m_effector->SetAngularSpringDamper(relaxation, 1500.0f, 100.0f);
							m_effector->SetMaxForce(10000.0f);
							m_effector->SetMaxTorque(10000.0f);
			
							// the effector is not added to the world, 
							// because is use by the IK solver to calculate joint motors.
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

			ndBodyKinematic*const body = new ndBodyDynamic();
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
			if (*m_effector)
			{
				//((ndJointBilateralConstraint*)m_effector)->DebugJoint(context);
				ndJointBilateralConstraint* const effectJoint = (ndJointBilateralConstraint*)*m_effector;
				effectJoint->DebugJoint(context);
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

		void PlaceEffector()
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

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			ndModel::Update(world, timestep);

			ndSkeletonContainer* const skeleton = m_rootBody->GetSkeleton();
			ndAssert(skeleton);

			//m_invDynamicsSolver.SetMaxIterations(4);
			if (*m_effector && !m_invDynamicsSolver.IsSleeping(skeleton))
			{
				PlaceEffector();
				ndJointBilateralConstraint* const joint = *m_effector;
				m_invDynamicsSolver.SolverBegin(skeleton, &joint, 1, world, timestep);
				m_invDynamicsSolver.Solve();
				m_invDynamicsSolver.SolverEnd();
			}
		}

		ndBodyDynamic* m_rootBody;
		ndJointSlider* m_leftGripper;
		ndJointSlider* m_rightGripper;
		ndSharedPtr<ndIk6DofEffector> m_effector;
		ndIkSolver m_invDynamicsSolver;
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
			, m_robot(robot)
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
			//ImGui::Text("ik solver passes");

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
}

using namespace ndAdvancedRobot;
void ndAdvancedIndustrialRobot(ndDemoEntityManager* const scene)
{
	// build a floor
	ndBodyKinematic* const floor = BuildFloorBox(scene, ndGetIdentityMatrix());
	
	ndVector origin1(0.0f, 0.0f, 0.0f, 1.0f);
	//ndDemoEntity* const robotEntity = ndDemoEntity::LoadFbx("robot.fbx", scene);
	ndSharedPtr<ndDemoEntity> robotEntity(ndDemoEntity::LoadFbx("robot.fbx", scene));
	
	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(ndYawMatrix(-90.0f * ndDegreeToRad));
	ndIndustrialRobot* const robot = new ndIndustrialRobot(scene, *robotEntity, matrix);
	scene->SetSelectedModel(robot);

	ndSharedPtr<ndModel> robotPtr(robot);
	//ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(robot->GetRoot()->GetMatrix(), robot->GetRoot(), world->GetSentinelBody()));
	ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(robot->GetRoot()->GetMatrix(), robot->GetRoot(), floor));
	world->AddModel(robotPtr);
	world->AddJoint(fixJoint);
	
	ndRobotUI* const robotUI = new ndRobotUI(scene, robot);
	ndSharedPtr<ndUIEntity> robotUIPtr(robotUI);
	scene->Set2DDisplayRenderFunction(robotUIPtr);
	
	//matrix.m_posit.m_x += 2.0f;
	//matrix.m_posit.m_z -= 2.0f;
	//ndSharedPtr<ndUIEntity> robotUIPtr1(new ndIndustrialRobot(scene, robotEntity, matrix));
	//world->AddModel(robotUIPtr1);
	//delete robotEntity;
	
	ndMatrix location(matrix * ndYawMatrix(45.0f * ndDegreeToRad));
	location.m_posit.m_z += 1.75f;
	location.m_posit.m_x += 1.75f;
	AddBox(scene, location, 2.0f, 2.0f, 0.5f, 2.0f);

	//ndMatrix location(matrix * ndYawMatrix(0.0f * ndDegreeToRad));
	//location.m_posit.m_x += 1.5f;
	//location.m_posit.m_z += 1.5f;
	//AddBox(scene, location, 2.0f, 0.3f, 0.4f, 0.7f);
	//location = ndYawMatrix(60.0f * ndDegreeToRad) * location;
	//AddBox(scene, location, 1.0f, 0.3f, 0.4f, 0.7f);
	//
	//location = ndYawMatrix(60.0f * ndDegreeToRad) * location;
	//location.m_posit.m_x += 1.0f;
	//location.m_posit.m_z += 0.5f;
	//AddBox(scene, location, 8.0f, 0.3f, 0.4f, 0.7f);
	//location = ndYawMatrix(45.0f * ndDegreeToRad) * location;
	//AddBox(scene, location, 4.0f, 0.3f, 0.4f, 0.7f);
	
	matrix.m_posit.m_x -= 6.0f;
	matrix.m_posit.m_y += 2.0f;
	matrix.m_posit.m_z += 6.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 45.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
