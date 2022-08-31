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
#include "ndTargaToOpenGl.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

#include "ndAnimationPose.h"
#include "ndAnimationSequence.h"
#include "ndBasicPlayerCapsule.h"
#include "ndAnimationKeyframesTrack.h"
#include "ndAnimationSequencePlayer.h"


class dActiveJointDefinition
{
	public:
	enum dLimbType
	{
		//forwardKinematic,
		//ballAndSocket,
		//effector,
		m_root,
		m_hinge,
		m_spherical,
		m_doubleHinge,
		m_effector
	};

	struct dJointPdData
	{
		dJointPdData()
			:m_spring(1500.0f)
			,m_damper(40.0f)
			,m_regularizer(0.001f)
		{
		}

		dJointPdData(ndFloat32 spring, ndFloat32 damper, ndFloat32 regularizer)
			:m_spring(spring)
			,m_damper(damper)
			,m_regularizer(regularizer)
		{
		}

		ndFloat32 m_spring;
		ndFloat32 m_damper;
		ndFloat32 m_regularizer;
	};

	struct dJointLimit
	{
		ndFloat32 m_minTwistAngle;
		ndFloat32 m_maxTwistAngle;
		ndFloat32 m_coneAngle;
	};

	struct dFrameMatrix
	{
		ndFloat32 m_pitch;
		ndFloat32 m_yaw;
		ndFloat32 m_roll;
	};

	char m_boneName[32];
	dLimbType m_limbType;
	ndFloat32 m_massWeight;
	dJointLimit m_jointLimits;
	dFrameMatrix m_frameBasics;
	dJointPdData m_coneSpringData;
	dJointPdData m_twistSpringData;
};

static dActiveJointDefinition jointsDefinition[] =
{
	{ "root", dActiveJointDefinition::m_root, 1.0f, {}, {} },

	{ "rhipjoint", dActiveJointDefinition::m_spherical, 1.0f, { -45.0f, 45.0f, 80.0f }, { 0.0f, 0.0f, 0.0f } },
	//{ "rfemur", dActiveJointDefinition::m_hinge, 1.0f, { 0.0f, 150.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
	//{ "rtibia", dActiveJointDefinition::m_doubleHinge, 1.0f, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },
	//{ "rightCalfEffector", dActiveJointDefinition::m_effector, 1.0f, { 0.0f, 0.0f, 60.0f }, { 0.0f, 0.0f, 0.0f } },
	//
	//{ "lhipjoint", dActiveJointDefinition::m_spherical, 1.0f, { -45.0f, 45.0f, 80.0f }, { 0.0f, 0.0f, 0.0f } },
	//{ "lfemur", dActiveJointDefinition::m_hinge, 1.0f, { 0.0f, 150.0f, 0.0f }, { 0.0f, 90.0f, 0.0f } },
	//{ "ltibia", dActiveJointDefinition::m_doubleHinge, 1.0f, { 0.0f, 0.0f, 60.0f }, { 90.0f, 0.0f, 90.0f } },
	//{ "leftCalfEffector", dActiveJointDefinition::m_effector, 1.0f, { 0.0f, 0.0f, 60.0f },{ 0.0f, 90.0f, 0.0f } },


	{ "", dActiveJointDefinition::m_root,{},{} },
};


class ndActiveRagdollEntityNotify : public ndDemoEntityNotify
{
	public:
	ndActiveRagdollEntityNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity, ndBodyDynamic* const parentBody)
		:ndDemoEntityNotify(manager, entity, parentBody)
		,m_bindMatrix(ndGetIdentityMatrix())
	{
		if (parentBody)
		{
			ndDemoEntity* const parentEntity = (ndDemoEntity*)(parentBody->GetNotifyCallback()->GetUserData());
			m_bindMatrix = entity->GetParent()->CalculateGlobalMatrix(parentEntity).Inverse();
		}
	}

	void OnTransform(ndInt32, const ndMatrix& matrix)
	{
		if (!m_parentBody)
		{
			const ndMatrix localMatrix(matrix * m_bindMatrix);
			const ndQuaternion rot(localMatrix);
			m_entity->SetMatrix(rot, localMatrix.m_posit);
		}
		else
		{
			const ndMatrix parentMatrix(m_parentBody->GetMatrix());
			const ndMatrix localMatrix(matrix * parentMatrix.Inverse() * m_bindMatrix);
			const ndQuaternion rot(localMatrix);
			m_entity->SetMatrix(rot, localMatrix.m_posit);
		}
	}

	void OnApplyExternalForce(ndInt32 thread, ndFloat32 timestep)
	{
		ndDemoEntityNotify::OnApplyExternalForce(thread, timestep);
		// remember to check and clamp huge angular velocities
	}

	ndMatrix m_bindMatrix;
};


class ndRagdollModel : public ndModel
{
	public:
	ndRagdollModel(ndDemoEntityManager* const scene, ndDemoEntity* const ragdollMesh, const ndMatrix& location)
		:ndModel()
		,m_rootBody(nullptr)
		//,m_animBlendTree(nullptr)
	{
		ndWorld* const world = scene->GetWorld();

		// make a clone of the mesh and add it to the scene
		ndDemoEntity* const entity = (ndDemoEntity*)ragdollMesh->CreateClone();
		scene->AddEntity(entity);

		ndDemoEntity* const rootEntity = (ndDemoEntity*)entity->Find(jointsDefinition[0].m_boneName);
		ndMatrix matrix(rootEntity->CalculateGlobalMatrix() * location);

		// find the floor location 
		ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		matrix.m_posit.m_y = floor.m_y + 1.03f;

		// add the root body
		ndBodyDynamic* const rootBody = CreateBodyPart(scene, rootEntity, nullptr);
		rootBody->SetMatrix(matrix);

		// set bindimg matrix;
		ndActiveRagdollEntityNotify* const notify = (ndActiveRagdollEntityNotify*)rootBody->GetNotifyCallback();
		notify->m_bindMatrix = matrix.Inverse() * rootEntity->CalculateGlobalMatrix(rootEntity->GetParent());

		m_rootBody = rootBody;
		

		ndInt32 stack = 0;
		ndFixSizeArray<ndFloat32, 64> massWeight;
		ndFixSizeArray<ndBodyDynamic*, 64> bodies;
		ndFixSizeArray<ndBodyDynamic*, 32> parentBones;
		ndFixSizeArray<ndDemoEntity*, 32> childEntities;

		parentBones.SetCount(32);
		childEntities.SetCount(32);
		for (ndDemoEntity* child = rootEntity->GetChild(); child; child = child->GetSibling())
		{
			childEntities[stack] = child;
			parentBones[stack] = rootBody;
			stack++;
		}

		bodies.PushBack(m_rootBody);
		massWeight.PushBack(jointsDefinition[0].m_massWeight);

		while (stack) 
		{
			stack--;
			ndBodyDynamic* parentBone = parentBones[stack];
			ndDemoEntity* const childEntity = childEntities[stack];
			const char* const name = childEntity->GetName().GetStr();
			//dTrace(("name: %s\n", name));
			for (ndInt32 i = 0; jointsDefinition[i].m_boneName[0]; ++i)
			{
				const dActiveJointDefinition& definition = jointsDefinition[i];
				if (!strcmp(definition.m_boneName, name))
				{
					ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, parentBone);
					bodies.PushBack(childBody);
					massWeight.PushBack(jointsDefinition[i].m_massWeight);

					//// connect this body part to its parentBody with a ragdoll joint
					//parentBone = ConnectBodyParts(childBody, parentBone, definition);
					//parentBone->SetName(name);

					parentBone = childBody;
					break;
				}
			}
		
			for (ndDemoEntity* child = childEntity->GetChild(); child; child = child->GetSibling())
			{
				childEntities[stack] = child;
				parentBones[stack] = parentBone;
				stack++;
			}
		}
		
		NormalizeMassDistribution(100.0f, bodies, massWeight);
		
		//if (1)
		//{
		//	ndBodyKinematic* testBody = m_rootNode->Find("mixamorig:Hips")->GetBody();
		//	//ndBodyKinematic* testBody = m_rootNode->Find("mixamorig:Spine1")->GetBody();
		//	ndJointFix6dof* const joint = new ndJointFix6dof(testBody->GetMatrix(), testBody, world->GetSentinelBody());
		//	world->AddJoint(joint);
		//	AddAttachment(joint);
		//}
		//
		//// initialize a biped controller and set to the model
		////m_bipedController.Init(this, bipedConfig);
		////SetController(&m_bipedController);
		//
		//if (righFoot)
		//{
		//	CreateKinematicChain(coronalFrame, righFoot);
		//}
		//
		//if (leftFoot)
		//{
		//	CreateKinematicChain(coronalFrame, leftFoot);
		//}
		//
		//SetAnimation(scene, entity);
	}

	~ndRagdollModel()
	{
		//if (m_animBlendTree)
		//{
		//	delete m_animBlendTree;
		//}
	}

	void SetAnimation(ndDemoEntityManager* const scene, const ndDemoEntity* const entity)
	{
		//ndAnimationSequence* const sequence = scene->GetAnimationSequence("whiteMan_idle.fbx");
		//const ndList<ndAnimationKeyFramesTrack>& tracks = sequence->GetTracks();
		//for (ndList<ndAnimationKeyFramesTrack>::ndNode* node = tracks.GetFirst(); node; node = node->GetNext())
		//{
		//	ndAnimationKeyFramesTrack& track = node->GetInfo();
		//	const char* const name = track.GetName().GetStr();
		//	ndCharacterNode* const skelNode = m_rootNode->Find(name);
		//	const ndDemoEntity* const ent = entity->Find(name);
		//	ndAssert(ent);
		//	ndAnimKeyframe keyFrame(ent->GetCurrentTransform());
		//	keyFrame.m_userData = skelNode;
		//	m_output.PushBack(keyFrame);
		//}
		//SetPose();
		//
		////ndAnimationSequence* const walkSequence = scene->GetAnimationSequence("whiteMan_idle.fbx");
		//ndAnimationSequence* const walkSequence = scene->GetAnimationSequence("whiteman_walk.fbx");
		//ndAnimationSequencePlayer* const walk = new ndAnimationSequencePlayer(walkSequence);
		//m_animBlendTree = walk;
	}

	void NormalizeMassDistribution(ndFloat32 mass, const ndFixSizeArray<ndBodyDynamic*, 64>& bodyArray, const ndFixSizeArray<ndFloat32, 64>& massWeight) const
	{
		ndFloat32 volume = 0.0f;
		for (ndInt32 i = 0; i < bodyArray.GetCount(); ++i)
		{
			volume += bodyArray[i]->GetCollisionShape().GetVolume() * massWeight[i];
		}
		ndFloat32 density = mass / volume;

		for (ndInt32 i = 0; i < bodyArray.GetCount(); ++i)
		{
			ndBodyDynamic* const body = bodyArray[i];
			ndFloat32 scale = density * body->GetCollisionShape().GetVolume() * massWeight[i];
			ndVector inertia(body->GetMassMatrix().Scale (scale));
			body->SetMassMatrix(inertia);
		}
	}
	
	ndBodyDynamic* CreateBodyPart(ndDemoEntityManager* const scene, ndDemoEntity* const entityPart, ndBodyDynamic* const parentBone)
	{
		ndShapeInstance* const shape = entityPart->CreateCollisionFromChildren();
		ndAssert(shape);

		// create the rigid body that will make this body
		ndMatrix matrix(entityPart->CalculateGlobalMatrix());

		ndBodyDynamic* const body = new ndBodyDynamic();
		body->SetMatrix(matrix);
		body->SetCollisionShape(*shape);
		body->SetMassMatrix(1.0f, *shape);
		body->SetNotifyCallback(new ndActiveRagdollEntityNotify(scene, entityPart, parentBone));

		scene->GetWorld()->AddBody(body);
		delete shape;
		return body;
	}

	ndCharacterNode* ConnectBodyParts(ndBodyDynamic* const childBody, ndCharacterNode* const parentNode, const dActiveJointDefinition& definition)
	{
		//ndMatrix matrix(childBody->GetMatrix());
		//dActiveJointDefinition::dFrameMatrix frameAngle(definition.m_frameBasics);
		//ndMatrix pinAndPivotInGlobalSpace(ndPitchMatrix(frameAngle.m_pitch * ndDegreeToRad) * ndYawMatrix(frameAngle.m_yaw * ndDegreeToRad) * ndRollMatrix(frameAngle.m_roll * ndDegreeToRad) * matrix);
		//
		//if (definition.m_limbType == dActiveJointDefinition::forwardKinematic)
		//{
		//	ndAssert(0);
		//	return nullptr;
		//	//ndCharacterForwardDynamicNode* const jointNode = CreateForwardDynamicLimb(pinAndPivotInGlobalSpace, childBody, parentNode);
		//	//
		//	//dActiveJointDefinition::dJointLimit jointLimits(definition.m_jointLimits);
		//	//ndJointSpherical* const joint = (ndJointSpherical*)jointNode->GetJoint();
		//	//
		//	//ndAssert(0);
		//	//joint->SetConeLimit(jointLimits.m_coneAngle * ndDegreeToRad);
		//	//joint->SetTwistLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
		//	//joint->SetSpringDamper(definition.m_coneSpringData.m_regularizer, definition.m_coneSpringData.m_spring, definition.m_coneSpringData.m_damper);
		//	//return jointNode;
		//}
		//else
		//{
		//	ndCharacterInverseDynamicNode* const jointNode = CreateInverseDynamicLimb(pinAndPivotInGlobalSpace, childBody, parentNode);
		//
		//	dActiveJointDefinition::dJointLimit jointLimits(definition.m_jointLimits);
		//	ndJointSpherical* const joint = (ndJointSpherical*)jointNode->GetJoint();
		//
		//	//dTrace (("do not forget to delete this debug\n"))
		//	//joint->SetSolverModel(m_jointkinematicCloseLoop);
		//
		//	ndAssert(0);
		//	joint->SetConeLimit(jointLimits.m_coneAngle * ndDegreeToRad);
		//	joint->SetTwistLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
		//	joint->SetAsSpringDamper(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		//	return jointNode;
		//}
	}

	//void Update(ndWorld* const world, ndFloat32 timestep) 
	void Update(ndWorld* const, ndFloat32)
	{
		//ndAssert(0);
		////m_animBlendTree->Evaluate(m_output, timestep);
		//m_animBlendTree->Evaluate(m_output, timestep * 0.05f);
		////m_animBlendTree->Evaluate(m_output, 0.0f);
		//for (ndInt32 i = 0; i < m_output.GetCount(); ++i)
		//{
		//	const ndAnimKeyframe& keyFrame = m_output[i];
		//	ndCharacterNode* const skelNode = (ndCharacterNode*)keyFrame.m_userData;
		//	if (skelNode)
		//	{
		//		skelNode->SetLocalPose(ndMatrix(keyFrame.m_rotation, keyFrame.m_posit));
		//	}
		//}
		//SetPose();
		//
		//ndCharacter::Update(world, timestep);
	}

	void PostUpdate(ndWorld* const world, ndFloat32 timestep)
	{
		//ndCharacter::PostUpdate(world, timestep);
	}

	void PostTransformUpdate(ndWorld* const world, ndFloat32 timestep)
	{
		//ndCharacter::PostTransformUpdate(world, timestep);
	}

	ndBodyDynamic* m_rootBody;
	//ndAnimationPose m_output;
	//ndAnimationBlendTreeNode* m_animBlendTree;
	//ndCharacterBipedPoseController m_bipedController;
};

//static void TestPlayerCapsuleInteraction(ndDemoEntityManager* const scene, const ndMatrix& location)
//{
//	ndMatrix localAxis(ndGetIdentityMatrix());
//	localAxis[0] = ndVector(0.0, 1.0f, 0.0f, 0.0f);
//	localAxis[1] = ndVector(1.0, 0.0f, 0.0f, 0.0f);
//	localAxis[2] = localAxis[0].CrossProduct(localAxis[1]);
//
//	ndFloat32 height = 1.9f;
//	ndFloat32 radio = 0.5f;
//	ndFloat32 mass = 100.0f;
//	ndDemoEntity* const entity = scene->LoadFbxMesh("walker.fbx");
//	ndBasicPlayerCapsule* const player = new ndBasicPlayerCapsule(scene, entity, localAxis, location, mass, radio, height, height / 4.0f);
//	player->GetNotifyCallback()->SetGravity(ndVector::m_zero);
//	ndMatrix matrix(player->GetMatrix());
//	matrix.m_posit.m_y += 0.5f;
//	player->SetMatrix(matrix);
//	delete entity;
//}

void ndRagdoll (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, ndGetIdentityMatrix());

	ndVector origin1(0.0f, 0.0f, 0.0f, 1.0f);
	fbxDemoEntity* const ragdollMesh = scene->LoadFbxMesh("walker.fbx");

	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit.m_y = 0.5f;
	ndMatrix playerMatrix(matrix);
	ndRagdollModel* const ragdoll = new ndRagdollModel(scene, ragdollMesh, matrix);
	scene->SetSelectedModel(ragdoll);
	scene->GetWorld()->AddModel(ragdoll);

	matrix.m_posit.m_x += 1.4f;
	//TestPlayerCapsuleInteraction(scene, matrix);

	matrix.m_posit.m_x += 2.0f;
	matrix.m_posit.m_y += 2.0f;
	//ndBodyKinematic* const reckingBall = AddSphere(scene, matrix.m_posit, 25.0f, 0.25f);
	//reckingBall->SetVelocity(ndVector(-5.0f, 0.0f, 0.0f, 0.0f));

	matrix.m_posit.m_x += 2.0f;
	matrix.m_posit.m_z -= 2.0f;
	//scene->GetWorld()->AddModel(new ndRagdollModel(scene, ragdollMesh, matrix));

	matrix.m_posit.m_z = 2.0f;
	//scene->GetWorld()->AddModel(new ndRagdollModel(scene, ragdollMesh, matrix));
	delete ragdollMesh;

	origin1.m_x += 20.0f;
	//AddCapsulesStacks(scene, origin1, 10.0f, 0.25f, 0.25f, 0.5f, 10, 10, 7);

	ndFloat32 angle = ndFloat32(90.0f * ndDegreeToRad);
	playerMatrix = ndYawMatrix(angle) * playerMatrix;
	ndVector origin(playerMatrix.m_posit + playerMatrix.m_front.Scale (-5.0f));
	origin.m_y += 1.0f;
	origin.m_z -= 2.0f;
	scene->SetCameraMatrix(playerMatrix, origin);

	//ndLoadSave loadScene;
	//loadScene.SaveModel("xxxxxx", ragdoll);
}
