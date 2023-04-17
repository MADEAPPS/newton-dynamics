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
#include "ndDemoCamera.h"
#include "ndLoadFbxMesh.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

#include "ndAnimationPose.h"
#include "ndAnimationSequence.h"
#include "ndBasicPlayerCapsule.h"
#include "ndAnimationKeyframesTrack.h"
#include "ndAnimationSequencePlayer.h"

namespace ndRagdoll
{
	class ndDefinition
	{
		public:
		enum ndjointType
		{
			m_root,
			m_hinge,
			m_spherical,
			m_doubleHinge,
			m_effector
		};

		struct ndDampData
		{
			ndDampData()
				:m_spring(0.0f)
				,m_damper(0.25f)
				,m_regularizer(0.1f)
			{
			}

			ndDampData(ndFloat32 spring, ndFloat32 damper, ndFloat32 regularizer)
				:m_spring(spring)
				,m_damper(damper)
				,m_regularizer(regularizer)
			{
			}

			ndFloat32 m_spring;
			ndFloat32 m_damper;
			ndFloat32 m_regularizer;
		};

		struct ndJointLimits
		{
			ndFloat32 m_minTwistAngle;
			ndFloat32 m_maxTwistAngle;
			ndFloat32 m_coneAngle;
		};

		struct ndOffsetFrameMatrix
		{
			ndFloat32 m_pitch;
			ndFloat32 m_yaw;
			ndFloat32 m_roll;
		};

		char m_boneName[32];
		ndjointType m_limbType;
		ndFloat32 m_massWeight;
		ndJointLimits m_jointLimits;
		ndOffsetFrameMatrix m_frameBasics;
		ndDampData m_coneSpringData;
		ndDampData m_twistSpringData;
	};

	static ndDefinition ragdollDefinition[] =
	{
		{ "root", ndDefinition::m_root, 1.0f, {}, {} },
		{ "lowerback", ndDefinition::m_spherical, 1.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 0.0f } },
		{ "upperback", ndDefinition::m_spherical, 1.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 0.0f } },
		{ "lowerneck", ndDefinition::m_spherical, 1.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 0.0f } },
		{ "upperneck", ndDefinition::m_spherical, 1.0f,{ -60.0f, 60.0f, 30.0f },{ 0.0f, 0.0f, 0.0f } },
		
		{ "lclavicle", ndDefinition::m_spherical, 1.0f, { -60.0f, 60.0f, 80.0f }, { 0.0f, -60.0f, 0.0f } },
		{ "lhumerus", ndDefinition::m_hinge, 1.0f, { 0.5f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
		{ "lradius", ndDefinition::m_doubleHinge, 1.0f, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },
		
		{ "rclavicle", ndDefinition::m_spherical, 1.0f, { -60.0f, 60.0f, 80.0f }, { 0.0f, 60.0f, 0.0f } },
		{ "rhumerus", ndDefinition::m_hinge, 1.0f, { 0.5f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
		{ "rradius", ndDefinition::m_doubleHinge, 1.0f, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },
		
		{ "rhipjoint", ndDefinition::m_spherical, 1.0f, { -45.0f, 45.0f, 80.0f }, { 0.0f, -60.0f, 0.0f } },
		{ "rfemur", ndDefinition::m_hinge, 1.0f, { 0.5f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
		{ "rtibia", ndDefinition::m_doubleHinge, 1.0f, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },
		
		{ "lhipjoint", ndDefinition::m_spherical, 1.0f,{ -45.0f, 45.0f, 80.0f }, { 0.0f, 60.0f, 0.0f } },
		{ "lfemur", ndDefinition::m_hinge, 1.0f, { 0.5f, 120.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
		{ "ltibia", ndDefinition::m_doubleHinge, 1.0f, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },

		{ "", ndDefinition::m_root,{},{} },
	};

	ndBodyDynamic* CreateBodyPart(ndDemoEntityManager* const scene, ndDemoEntity* const entityPart, ndBodyDynamic* const parentBone)
	{
		ndSharedPtr<ndShapeInstance> shapePtr(entityPart->CreateCollisionFromChildren());
		ndShapeInstance* const shape = *shapePtr;
		ndAssert(shape);

		// create the rigid body that will make this body
		ndMatrix matrix(entityPart->CalculateGlobalMatrix());

		ndBodyDynamic* const body = new ndBodyDynamic();
		body->SetMatrix(matrix);
		body->SetCollisionShape(*shape);
		body->SetMassMatrix(1.0f, *shape);
		body->SetNotifyCallback(new ndBindingRagdollEntityNotify(scene, entityPart, parentBone, 100.0f));
		return body;
	}

	ndJointBilateralConstraint* ConnectBodyParts(ndBodyDynamic* const childBody, ndBodyDynamic* const parentBody, const ndDefinition& definition)
	{
		ndMatrix matrix(childBody->GetMatrix());
		ndDefinition::ndOffsetFrameMatrix frameAngle(definition.m_frameBasics);
		ndMatrix pinAndPivotInGlobalSpace(ndPitchMatrix(frameAngle.m_pitch * ndDegreeToRad) * ndYawMatrix(frameAngle.m_yaw * ndDegreeToRad) * ndRollMatrix(frameAngle.m_roll * ndDegreeToRad) * matrix);

		ndDefinition::ndJointLimits jointLimits(definition.m_jointLimits);

		switch (definition.m_limbType)
		{
			case ndDefinition::m_spherical:
			{
				ndJointSpherical* const joint = new ndJointSpherical(pinAndPivotInGlobalSpace, childBody, parentBody);
				joint->SetConeLimit(jointLimits.m_coneAngle * ndDegreeToRad);
				joint->SetTwistLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
				joint->SetAsSpringDamper(definition.m_coneSpringData.m_regularizer, definition.m_coneSpringData.m_spring, definition.m_coneSpringData.m_damper);
				return joint;
			}

			case ndDefinition::m_hinge:
			{
				ndJointHinge* const joint = new ndJointHinge(pinAndPivotInGlobalSpace, childBody, parentBody);
				joint->SetLimitState(true);
				joint->SetLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
				joint->SetAsSpringDamper(definition.m_coneSpringData.m_regularizer, definition.m_coneSpringData.m_spring, definition.m_coneSpringData.m_damper);
				return joint;
			}

			case ndDefinition::m_doubleHinge:
			{
				ndJointDoubleHinge* const joint = new ndJointDoubleHinge(pinAndPivotInGlobalSpace, childBody, parentBody);
				joint->SetLimits0(-30.0f * ndDegreeToRad, 30.0f * ndDegreeToRad);
				joint->SetLimits1(-45.0f * ndDegreeToRad, 45.0f * ndDegreeToRad);
				joint->SetAsSpringDamper0(definition.m_coneSpringData.m_regularizer, definition.m_coneSpringData.m_spring, definition.m_coneSpringData.m_damper);
				joint->SetAsSpringDamper1(definition.m_coneSpringData.m_regularizer, definition.m_coneSpringData.m_spring, definition.m_coneSpringData.m_damper);
				return joint;
			}

			default:
				ndAssert(0);
		}
		return nullptr;
	}

	ndModelPassiveRagdoll* BuildRagDoll(ndDemoEntityManager* const scene, ndDemoEntity* const modelMesh, const ndMatrix& location)
	{
		ndModelPassiveRagdoll* const ragdoll = new ndModelPassiveRagdoll();

		ndWorld* const world = scene->GetWorld();
		ndDemoEntity* const entity = modelMesh->CreateClone();
		scene->AddEntity(entity);

		ndDemoEntity* const rootEntity = (ndDemoEntity*)entity->Find(ragdollDefinition[0].m_boneName);
		ndMatrix matrix(rootEntity->CalculateGlobalMatrix() * location);

		// find the floor location 
		ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		matrix.m_posit.m_y = floor.m_y + 1.5f;

		ndSharedPtr<ndBody> rootBody(CreateBodyPart(scene, rootEntity, nullptr));

		// set the root transform matrix
		rootBody->GetNotifyCallback()->OnTransform(0, matrix);
		rootBody->GetNotifyCallback()->OnTransform(0, matrix);
		rootBody->SetMatrix(rootEntity->CalculateGlobalMatrix());

		// add body to the world
		world->AddBody(rootBody);

		// add the root body to the model
		ndModelPassiveRagdoll::ndRagdollNode* const ragDollNode = ragdoll->AddRootBody(rootBody);

		ndInt32 stack = 0;
		ndFixSizeArray<ndDemoEntity*, 32> childEntities;
		ndFixSizeArray<ndModelPassiveRagdoll::ndRagdollNode*, 32> parentBones;

		// parse the 3d model and add all the limb
		for (ndDemoEntity* child = rootEntity->GetFirstChild(); child; child = child->GetNext())
		{
			childEntities[stack] = child;
			parentBones[stack] = ragDollNode;
			stack++;
		}

		while (stack)
		{
			stack--;
			ndDemoEntity* const childEntity = childEntities[stack];
			ndModelPassiveRagdoll::ndRagdollNode* parentBone = parentBones[stack];
			const char* const name = childEntity->GetName().GetStr();
			//ndTrace(("name: %s\n", name));

			for (ndInt32 i = 0; ragdollDefinition[i].m_boneName[0]; ++i)
			{
				const ndDefinition& definition = ragdollDefinition[i];

				if (!strcmp(definition.m_boneName, name))
				{
					ndSharedPtr<ndBody> childBody (CreateBodyPart(scene, childEntity, parentBone->m_body));
					
					//connect this body part to its parentBody with a ragdoll joint
					ndSharedPtr<ndJointBilateralConstraint> joint = ConnectBodyParts(childBody->GetAsBodyDynamic(), parentBone->m_body, definition);
					world->AddJoint(joint);

					// add body to the world.
					world->AddBody(childBody);

					// add this child body to the radgoll model.
					parentBone = ragdoll->AddLimb(parentBone, childBody, joint);
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

		// make entire body weight 100.0 kg
		ragdoll->NormalizeMassDistribution(100.0f);
		return ragdoll;
	}
}

using namespace ndRagdoll;
void ndRagdollTest (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, ndGetIdentityMatrix());
	
	ndVector origin1(0.0f, 0.0f, 0.0f, 1.0f);
	ndSharedPtr<ndDemoEntity> modelMesh (ndDemoEntity::LoadFbx("walker.fbx", scene));
	
	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit.m_y = 0.5f;
	ndMatrix playerMatrix(matrix);

	ndModel* const model = BuildRagDoll(scene, *modelMesh, matrix);
	ndSharedPtr<ndModel> ragdoll(model);
	scene->GetWorld()->AddModel(ragdoll);
	
	//matrix.m_posit.m_x += 1.4f;
	//TestPlayerCapsuleInteraction(scene, matrix);
	//matrix.m_posit.m_x += 2.0f;
	//matrix.m_posit.m_y += 2.0f;
	//ndBodyKinematic* const reckingBall = AddSphere(scene, matrix.m_posit, 25.0f, 0.25f);
	//reckingBall->SetVelocity(ndVector(-5.0f, 0.0f, 0.0f, 0.0f));
	//matrix.m_posit.m_x += 2.0f;
	//matrix.m_posit.m_z -= 2.0f;
	//scene->GetWorld()->AddModel(new ndHumanoidModel(scene, modelMesh, matrix));
	//matrix.m_posit.m_z = 2.0f;
	//scene->GetWorld()->AddModel(new ndHumanoidModel(scene, modelMesh, matrix));
	//origin1.m_x += 20.0f;
	//AddCapsulesStacks(scene, origin1, 10.0f, 0.25f, 0.25f, 0.5f, 10, 10, 7);
	
	ndFloat32 angle = ndFloat32(90.0f * ndDegreeToRad);
	playerMatrix = ndYawMatrix(angle) * playerMatrix;
	ndVector origin(playerMatrix.m_posit + playerMatrix.m_front.Scale (-5.0f));
	origin.m_y += 1.0f;
	origin.m_z += -3.0f;
	scene->SetCameraMatrix(playerMatrix, origin);

	ndFileFormatSave xxxxSave;
	xxxxSave.SaveModels(scene->GetWorld(), "xxxx.nd");

	ndFileFormatLoad xxxxLoad;
	xxxxLoad.Load("xxxx.nd");

	// offset bodies positions for calibraion;
	const ndList<ndSharedPtr<ndBody>>& bodyList = xxxxLoad.GetBodyList();
	for (ndList<ndSharedPtr<ndBody>>::ndNode* node = bodyList.GetFirst(); node; node = node->GetNext())
	{
		ndSharedPtr<ndBody>& body = node->GetInfo();
		ndMatrix bodyMatrix (body->GetMatrix());
		bodyMatrix.m_posit.m_x += 0.5f;
		body->SetMatrix(bodyMatrix);
	}
	xxxxLoad.AddToWorld(scene->GetWorld());
}
