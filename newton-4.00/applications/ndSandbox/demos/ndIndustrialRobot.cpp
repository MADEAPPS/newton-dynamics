/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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
#include "ndTargaToOpenGl.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndLoadFbxMesh.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

class dRobotDefinition
{
	public:
	char m_boneName[32];
};

static dRobotDefinition jointsDefinition[] =
{
	{ "base"},
};

class ndIndustrialRobot : public ndModel
{
	public:
	ndIndustrialRobot(ndDemoEntityManager* const scene, fbxDemoEntity* const robotMesh, const ndMatrix& location)
		:ndModel()
	{
		// make a clone of the mesh and add it to the scene
		ndDemoEntity* const entity = robotMesh->CreateClone();
		scene->AddEntity(entity);
		ndWorld* const world = scene->GetWorld();
		
		// find the floor location 
		ndMatrix matrix(location);
		ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		matrix.m_posit.m_y = floor.m_y;
		entity->ResetMatrix(matrix);
		
		// add the root body
		ndBodyDynamic* const rootBody = CreateBodyPart(scene, entity, nullptr);
		
		ndFixSizeArray<ndDemoEntity*, 32> childEntities;
		ndFixSizeArray<ndBodyDynamic*, 32> parentBone;

		ndInt32 stack = 0;
		for (ndDemoEntity* child = entity->GetChild(); child; child = child->GetSibling())
		{
			childEntities[stack] = child;
			parentBone[stack] = rootBody;
			stack++;
		}

		const ndInt32 definitionCount = ndInt32 (sizeof(jointsDefinition) / sizeof(jointsDefinition[0]));
		while (stack) 
		{
			stack--;
			ndBodyDynamic* const parentBody = parentBone[stack];
			ndDemoEntity* const childEntity = childEntities[stack];

			const char* const name = childEntity->GetName().GetStr();
			for (ndInt32 i = 0; i < definitionCount; i++) 
			{
				const dRobotDefinition& definition = jointsDefinition[i];
				//if (!strcmp(definition.m_boneName, name))
				//{
				//	dTrace(("name: %s\n", name));
				//	if (definition.m_limbType != dRobotDefinition::effector)
				//	{
				//		ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, parentBone->GetBody());
				//		bodyArray[bodyCount] = childBody;
				//		massWeight[bodyCount] = definition.m_massWeight;
				//		bodyCount++;
				//
				//		// connect this body part to its parentBody with a robot joint
				//		parentBone = ConnectBodyParts(childBody, parentBone, definition);
				//		parentBone->SetName(name);
				//
				//		if (strstr(name, "RightFoot"))
				//		{
				//			righFoot = parentBone;
				//			//bipedConfig.m_rightFootNode = parentBone;
				//		}
				//		else if (strstr(name, "LeftFoot"))
				//		{
				//			leftFoot = parentBone;
				//			//bipedConfig.m_leftFootNode = parentBone;
				//		}
				//	}
				//	else
				//	{
				//		dAssert(0);
				//		//ndMatrix effectorMatrix(childEntity->GetCurrentMatrix() * parentBone->GetBody()->GetMatrix());
				//		//ndCharacterEffectorNode* const effectorNode = CreateInverseDynamicEffector(effectorMatrix, parentBone);
				//		//effectorNode->SetName(name);
				//		//if (strcmp(effectorNode->GetJoint()->SubClassName(), "ndJointTwoBodyIK") == 0)
				//		//{
				//		//	//ndJointTwoBodyIK* const effectorJoint = (ndJointTwoBodyIK*)effectorNode->GetJoint();
				//		//	//effectorJoint->SetLinearSpringDamperRegularizer(definition.m_jointData.m_spring, definition.m_jointData.m_damper, definition.m_jointData.m_regularizer);
				//		//}
				//		//else
				//		//{
				//		//	dAssert(0);
				//		//}
				//		//
				//		//if (strstr(name, "right"))
				//		//{
				//		//	bipedConfig.m_rightFootEffector = effectorNode;
				//		//}
				//		//else if (strstr(name, "left"))
				//		//{
				//		//	bipedConfig.m_leftFootEffector = effectorNode;
				//		//}
				//	}
				//
				//	break;
				//}
			}

			for (ndDemoEntity* child = childEntity->GetChild(); child; child = child->GetSibling())
			{
				childEntities[stack] = child;
				//parentBones[stack] = parentBone;
				stack++;
			}
		}
	}

	~ndIndustrialRobot()
	{
	}
	
	ndBodyDynamic* CreateBodyPart(ndDemoEntityManager* const scene, ndDemoEntity* const entityPart, ndBodyDynamic* const parentBone)
	{
		ndShapeInstance* const shape = entityPart->CreateCollisionFromchildren();
		dAssert(shape);
		
		// create the rigid body that will make this body
		ndMatrix matrix(entityPart->CalculateGlobalMatrix());
		
		ndBodyDynamic* const body = new ndBodyDynamic();
		body->SetMatrix(matrix);
		body->SetCollisionShape(*shape);
		body->SetMassMatrix(1.0f, *shape);
		body->SetNotifyCallback(new ndDemoEntityNotify(scene, entityPart, parentBone));
		
		delete shape;

		// add body to the world
		scene->GetWorld()->AddBody(body);
		return body;
	}

	//ndCharacterNode* ConnectBodyParts(ndBodyDynamic* const childBody, ndCharacterNode* const parentNode, const dRobotDefinition& definition)
	//{
	//	ndMatrix matrix(childBody->GetMatrix());
	//	dRobotDefinition::dFrameMatrix frameAngle(definition.m_frameBasics);
	//	ndMatrix pinAndPivotInGlobalSpace(dPitchMatrix(frameAngle.m_pitch * ndDegreeToRad) * dYawMatrix(frameAngle.m_yaw * ndDegreeToRad) * dRollMatrix(frameAngle.m_roll * ndDegreeToRad) * matrix);
	//
	//	if (definition.m_limbType == dRobotDefinition::forwardKinematic)
	//	{
	//		ndCharacterForwardDynamicNode* const jointNode = CreateForwardDynamicLimb(pinAndPivotInGlobalSpace, childBody, parentNode);
	//
	//		dRobotDefinition::dJointLimit jointLimits(definition.m_jointLimits);
	//		ndJointPdActuator* const joint = (ndJointPdActuator*)jointNode->GetJoint();
	//
	//		joint->SetConeLimit(jointLimits.m_coneAngle * ndDegreeToRad);
	//		joint->SetTwistLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
	//		joint->SetConeAngleSpringDamperRegularizer(definition.m_coneSpringData.m_spring, definition.m_coneSpringData.m_damper, definition.m_coneSpringData.m_regularizer);
	//		joint->SetTwistAngleSpringDamperRegularizer(definition.m_twistSpringData.m_spring, definition.m_twistSpringData.m_damper, definition.m_twistSpringData.m_regularizer);
	//
	//		return jointNode;
	//	}
	//	else
	//	{
	//		ndCharacterInverseDynamicNode* const jointNode = CreateInverseDynamicLimb(pinAndPivotInGlobalSpace, childBody, parentNode);
	//
	//		dRobotDefinition::dJointLimit jointLimits(definition.m_jointLimits);
	//		ndJointBallAndSocket* const joint = (ndJointBallAndSocket*)jointNode->GetJoint();
	//
	//		//dTrace (("do not forget to delete this debug\n"))
	//		//joint->SetSolverModel(m_jointkinematicCloseLoop);
	//
	//		joint->SetConeLimit(jointLimits.m_coneAngle * ndDegreeToRad);
	//		joint->SetTwistLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
	//		joint->SetConeFriction(ndFloat32(0.0f), ndFloat32(0.0f));
	//		joint->SetTwistFriction(ndFloat32(0.0f), ndFloat32(0.0f));
	//
	//		return jointNode;
	//	}
	//}

	void Update(ndWorld* const world, ndFloat32 timestep) 
	{
		//dAssert(0);
		ndModel::Update(world, timestep);
	}

	void PostUpdate(ndWorld* const world, ndFloat32 timestep)
	{
		ndModel::PostUpdate(world, timestep);
	}

	void PostTransformUpdate(ndWorld* const world, ndFloat32 timestep)
	{
		ndModel::PostTransformUpdate(world, timestep);
	}
};

void ndInsdustrialRobot (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, dGetIdentityMatrix());

	ndVector origin1(0.0f, 0.0f, 0.0f, 0.0f);
	fbxDemoEntity* const robotEntity = scene->LoadFbxMesh("robot.fbx");

	ndMatrix matrix(dYawMatrix(-90.0f * ndDegreeToRad));
	ndIndustrialRobot* const robot = new ndIndustrialRobot(scene, robotEntity, matrix);
	//scene->SetSelectedModel(robot);
	scene->GetWorld()->AddModel(robot);
	
	matrix.m_posit.m_x += 2.0f;
	matrix.m_posit.m_z -= 2.0f;
	scene->GetWorld()->AddModel(new ndIndustrialRobot(scene, robotEntity, matrix));

	delete robotEntity;

	matrix.m_posit.m_x -= 8.0f;
	matrix.m_posit.m_y += 2.0f;
	scene->SetCameraMatrix(ndQuaternion(), matrix.m_posit);
}
