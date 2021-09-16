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

class dJointDefinition
{
	public:
	enum dCollsionMask
	{
		m_type0,
		m_type1,
		m_type2,
		m_type3,
	};

	struct dJointLimit
	{
		dFloat32 m_minTwistAngle;
		dFloat32 m_maxTwistAngle;
		dFloat32 m_coneAngle;
	};

	struct dFrameMatrix
	{
		dFloat32 m_pitch;
		dFloat32 m_yaw;
		dFloat32 m_roll;
	};

	char m_boneName[32];
	dInt32 m_type;
	dInt32 m_typeMask;
	dFloat32 m_friction;
	dJointLimit m_jointLimits;
	dFrameMatrix m_frameBasics;
};

class ndRagdollEntityNotify : public ndDemoEntityNotify
{
	public:
	ndRagdollEntityNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity, ndBodyDynamic* const parentBody)
		:ndDemoEntityNotify(manager, entity, parentBody)
		,m_bindMatrix(dGetIdentityMatrix())
	{
		if (parentBody)
		{
			ndDemoEntity* const parentEntity = (ndDemoEntity*)(parentBody->GetNotifyCallback()->GetUserData());
			m_bindMatrix = entity->GetParent()->CalculateGlobalMatrix(parentEntity).Inverse();
		}
	}

	void OnTransform(dInt32 thread, const dMatrix& matrix)
	{
		if (!m_parentBody)
		{
			ndDemoEntityNotify::OnTransform(thread, matrix);
		}
		else
		{
			const dMatrix parentMatrix(m_parentBody->GetMatrix());
			const dMatrix localMatrix(matrix * parentMatrix.Inverse() * m_bindMatrix);
			const dQuaternion rot(localMatrix);
			m_entity->SetMatrix(rot, localMatrix.m_posit);
		}
	}

	void OnApplyExternalForce(dInt32 thread, dFloat32 timestep)
	{
		ndDemoEntityNotify::OnApplyExternalForce(thread, timestep);
		// remember to check and clamp huge angular velocities
	}

	dMatrix m_bindMatrix;
};

static dJointDefinition jointsDefinition[] =
{
	{ "mixamorig:Hips", 1, 16, 0, {}, {}},
	
	{ "mixamorig:Spine", 2, 16, 10.0f, { -15.0f, 15.0f,  30.0f }, { 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:Spine1", 4, 16, 10.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:Spine2", 8, 16, 10.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:Neck", 16, 31, 10.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 180.0f } },
	
	{ "mixamorig:RightUpLeg", 16, 31, 10.0f, { -45.0f, 45.0f, 120.0f }, { 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:RightLeg", 16, 31, 10.0f, { -140.0f, 10.0f, 0.0f }, { 0.0f, 90.0f, 90.0f } },
	{ "mixamorig:RightFoot", 16, 31, 10.0f,{ 0.0f, 0.0f, 60.0f },{ 0.0f, 0.0f, 180.0f } },
	
	{ "mixamorig:LeftUpLeg", 16, 31, 10.0f, { -45.0f, 45.0f, 120.0f }, { 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:LeftLeg", 16, 31, 10.0f, { -140.0f, 10.0f, 0.0f }, { 0.0f, 90.0f, 90.0f } },
	{ "mixamorig:LeftFoot", 16, 31, 10.0f,{ 0.0f, 0.0f, 60.0f },{ 0.0f, 0.0f, 180.0f } },
	
	{ "mixamorig:RightArm", 16, 27, 10.0f, { -45.0f, 45.0f, 80.0f }, { 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:RightForeArm", 16, 31, 10.0f, { -140.0f, 10.0f, 0.0f }, { 0.0f, 00.0f, 90.0f } },
	
	{ "mixamorig:LeftArm", 16, 27, 10.0f, { -45.0f, 45.0f, 80.0f }, { 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:LeftForeArm", 16, 31, 10.0f, { -140.0f, 10.0f, 0.0f }, { 0.0f, 0.0f, -90.0f } },
};

//class ndRagdollModel : public ndModel
class ndRagdollModel : public ndCharacter
{
	public:
	ndRagdollModel(ndDemoEntityManager* const scene, fbxDemoEntity* const ragdollMesh, const dMatrix& location)
		:ndCharacter()
	{
		// make a clone of the mesh and add it to the scene
		ndDemoEntity* const entity = ragdollMesh->CreateClone();
		scene->AddEntity(entity);
		ndWorld* const world = scene->GetWorld();
		
		// find the floor location 
		dMatrix matrix(location);
		dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		matrix.m_posit.m_y = floor.m_y + 1.0f;

		// add the root body
		ndDemoEntity* const rootEntity = (ndDemoEntity*)entity->Find(jointsDefinition[0].m_boneName);
		rootEntity->ResetMatrix(rootEntity->GetCurrentMatrix() * matrix);
		ndBodyDynamic* const rootBody = CreateBodyPart(scene, rootEntity, nullptr);
		ndCharacterRootNode* const rootNode = CreateRoot(rootBody);

		dInt32 stack = 0;
		const int definitionCount = sizeof(jointsDefinition) / sizeof(jointsDefinition[0]);
		
		ndDemoEntity* childEntities[32];
		ndCharacterLimbNode* parentBones[32];
		for (ndDemoEntity* child = rootEntity->GetChild(); child; child = child->GetSibling()) 
		{
			childEntities[stack] = child;
			parentBones[stack] = rootNode;
			stack++;
		}
		
		dInt32 bodyCount = 1;
		ndBodyDynamic* bodyArray[1024];
		bodyArray[0] = rootBody;
		
		// walk model hierarchic adding all children designed as rigid body bones. 
		while (stack) 
		{
			stack--;
			ndCharacterLimbNode* parentBone = parentBones[stack];
			ndDemoEntity* const childEntity = childEntities[stack];
			const char* const name = childEntity->GetName().GetStr();
			//dTrace(("name: %s\n", name));
			for (dInt32 i = 0; i < definitionCount; i++) 
			{
				if (!strcmp(jointsDefinition[i].m_boneName, name)) 
				{
					ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, parentBone->GetBody());
					bodyArray[bodyCount] = childBody;
					bodyCount++;
		
					// connect this body part to its parentBody with a ragdoll joint
					parentBone = ConnectBodyParts(childBody, parentBone, jointsDefinition[i]);
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
		
		SetModelMass(100.0f, bodyCount, bodyArray);
		
		for (dInt32 i = 0; i < bodyCount; i++)
		{
			ndDemoEntity* ent = (ndDemoEntity*)bodyArray[i]->GetNotifyCallback()->GetUserData();
			if (ent->GetName() == "mixamorig:Neck") 
			{
				//world->AddJoint(new ndJointFix6dof(bodyArray[i]->GetMatrix(), bodyArray[i], world->GetSentinelBody()));
				break;
			}
		}
	}

	void SetModelMass(dFloat32 mass, int bodyCount, ndBodyDynamic** const bodyArray) const
	{
		dFloat32 volume = 0.0f;
		for (int i = 0; i < bodyCount; i++) 
		{
			volume += bodyArray[i]->GetCollisionShape().GetVolume();
		}
		dFloat32 density = mass / volume;

		for (int i = 0; i < bodyCount; i++) 
		{
			ndBodyDynamic* const body = bodyArray[i];
			dFloat32 scale = density * body->GetCollisionShape().GetVolume();
			dVector inertia(body->GetMassMatrix().Scale (scale));
			body->SetMassMatrix(inertia);
		}
	}
	
	ndBodyDynamic* CreateBodyPart(ndDemoEntityManager* const scene, ndDemoEntity* const entityPart, ndBodyDynamic* const parentBone)
	{
		ndShapeInstance* const shape = entityPart->CreateCollisionFromchildren();
		dAssert(shape);

		// create the rigid body that will make this body
		dMatrix matrix(entityPart->CalculateGlobalMatrix());

		ndBodyDynamic* const body = new ndBodyDynamic();
		body->SetMatrix(matrix);
		body->SetCollisionShape(*shape);
		body->SetMassMatrix(1.0f, *shape);
		body->SetNotifyCallback(new ndRagdollEntityNotify(scene, entityPart, parentBone));

		delete shape;
		return body;
	}

	ndCharacterLimbNode* ConnectBodyParts(ndBodyDynamic* const childBody, ndCharacterLimbNode* const parentBone, const dJointDefinition& definition)
	{
		dMatrix matrix(childBody->GetMatrix());
		dJointDefinition::dFrameMatrix frameAngle(definition.m_frameBasics);
		dMatrix pinAndPivotInGlobalSpace(dPitchMatrix(frameAngle.m_pitch * dDegreeToRad) * dYawMatrix(frameAngle.m_yaw * dDegreeToRad) * dRollMatrix(frameAngle.m_roll * dDegreeToRad) * matrix);

		ndCharacterForwardDynamicNode* const jointNode = CreateForwardDynamicLimb(pinAndPivotInGlobalSpace, childBody, parentBone);
		ndJointPdActuator* const joint = (ndJointPdActuator*)jointNode->GetJoint();
		
		dJointDefinition::dJointLimit jointLimits(definition.m_jointLimits);
		joint->SetConeLimit(jointLimits.m_coneAngle * dDegreeToRad);
		joint->SetTwistLimits(jointLimits.m_minTwistAngle * dDegreeToRad, jointLimits.m_maxTwistAngle * dDegreeToRad);
		joint->SetConeAngleSpringDamperRegularizer(0.0f, definition.m_friction, 0.005f);

		return jointNode;
	}

	void Update(ndWorld* const, dFloat32) 
	{
	}

	//void PostUpdate(ndWorld* const world, dFloat32)
	void PostUpdate(ndWorld* const, dFloat32)
	{
	}

	//void PostTransformUpdate(ndWorld* const world, dFloat32 timestep)
	void PostTransformUpdate(ndWorld* const, dFloat32)
	{
	}
};

void ndBasicRagdoll (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, dGetIdentityMatrix());

	dVector origin1(0.0f, 0.0f, 0.0f, 0.0f);
	fbxDemoEntity* const ragdollMesh = scene->LoadFbxMesh("whiteMan.fbx");

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit.m_y = 0.5f;
	ndRagdollModel* const ragdoll = new ndRagdollModel(scene, ragdollMesh, matrix);
	scene->SetSelectedModel(ragdoll);
	scene->GetWorld()->AddModel(ragdoll);

	matrix.m_posit.m_x += 2.0f;
	matrix.m_posit.m_z -= 2.0f;
	scene->GetWorld()->AddModel(new ndRagdollModel(scene, ragdollMesh, matrix));

	matrix.m_posit.m_z = 2.0f;
	scene->GetWorld()->AddModel(new ndRagdollModel(scene, ragdollMesh, matrix));

	origin1.m_x += 20.0f;
	AddCapsulesStacks(scene, origin1, 10.0f, 0.25f, 0.25f, 0.5f, 10, 10, 7);

	delete ragdollMesh;
	dQuaternion rot;
	dVector origin(-5.0f, 1.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
