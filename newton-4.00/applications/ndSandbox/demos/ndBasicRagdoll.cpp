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

	dMatrix m_bindMatrix;
};

static dJointDefinition jointsDefinition[] =
{
	{ "mixamorig:Hips", 1, 16 },
	
	{ "mixamorig:Spine", 2, 16, 100.0f,{ -15.0f, 15.0f,  30.0f },{ 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:Spine1", 4, 16, 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:Spine2", 8, 16, 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:Neck", 16, 31, 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 180.0f } },

	{ "mixamorig:LeftUpLeg", 16, 31, 100.0f,{ -45.0f, 45.0f, 120.0f },{ 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:LeftLeg", 16, 31, 50.0f,{ -140.0f, 10.0f, 0.0f },{ 0.0f, 90.0f, 90.0f } },
	
	{ "mixamorig:RightUpLeg", 16, 31, 100.0f,{ -45.0f, 45.0f, 120.0f },{ 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:RightLeg", 16, 31, 50.0f,{ -140.0f, 10.0f, 0.0f },{ 0.0f, 90.0f, 90.0f } },

	{ "mixamorig:LeftArm", 16, 27, 100.0f,{ -45.0f, 45.0f, 80.0f },{ 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:LeftForeArm", 16, 31, 50.0f,{ -140.0f, 10.0f, 0.0f },{ 0.0f, 0.0f, -90.0f } },
	
	{ "mixamorig:RightArm", 16, 27, 100.0f,{ -45.0f, 45.0f, 80.0f },{ 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:RightForeArm", 16, 31, 50.0f,{ -140.0f, 10.0f, 0.0f },{ 0.0f, 00.0f, 90.0f } },
};

class ndRagDollModel : public ndModel
{
	public:
	ndRagDollModel(ndDemoEntityManager* const scene, fbxDemoEntity* const ragdollMesh, const dMatrix& location)
	{
		// make a clone of the mesh and add it to the scene
		ndDemoEntity* const entity = ragdollMesh->CreateClone();
		scene->AddEntity(entity);
		
		ndWorld* const world = scene->GetWorld();
		const int definitionCount = sizeof(jointsDefinition) / sizeof(jointsDefinition[0]);
		
		// find the floor location 
		dMatrix matrix(location);
		dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		matrix.m_posit.m_y = floor.m_y + 1.0f;

		// add the root bone
		ndDemoEntity* const rootEntity = (ndDemoEntity*)entity->Find(jointsDefinition[0].m_boneName);
		rootEntity->ResetMatrix(rootEntity->GetCurrentMatrix() * matrix);
		ndBodyDynamic* const rootBone = CreateBodyPart(scene, rootEntity, nullptr);

		//NewtonCollisionMaterial collisionMaterial;
		//NewtonCollisionGetMaterial(NewtonBodyGetCollision(rootBone), &collisionMaterial);
		//collisionMaterial.m_userData.m_ptr = rootBone;
		//collisionMaterial.m_userParam[0].m_int = jointsDefinition[0].m_type;
		//collisionMaterial.m_userParam[1].m_int = jointsDefinition[0].m_typeMask;
		//NewtonCollisionSetMaterial(NewtonBodyGetCollision(rootBone), &collisionMaterial);

		int stack = 0;
		ndBodyDynamic* parentBones[32];
		ndDemoEntity* childEntities[32];
		for (ndDemoEntity* child = rootEntity->GetChild(); child; child = child->GetSibling()) 
		{
			childEntities[stack] = child;
			parentBones[stack] = rootBone;
			stack++;
		}
		
		dInt32 bodyCount = 1;
		ndBodyDynamic* bodyArray[1024];
		bodyArray[0] = rootBone;

		// walk model hierarchic adding all children designed as rigid body bones. 
		while (stack) 
		{
			stack--;
			ndBodyDynamic* parentBone = parentBones[stack];
			ndDemoEntity* const childEntity = childEntities[stack];
			const char* const name = childEntity->GetName().GetStr();
			//dTrace(("name: %s\n", name));
			for (dInt32 i = 0; i < definitionCount; i++) 
			{
				if (!strcmp(jointsDefinition[i].m_boneName, name)) 
				{
					ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, parentBone);
					bodyArray[bodyCount] = childBody;
					bodyCount++;

					// connect this body part to its parentBody with a ragdoll joint
					ConnectBodyParts(world, childBody, parentBone, jointsDefinition[i]);

					//// save the controller as the collision user data, for collision culling
					//NewtonCollisionMaterial collisionMaterial;
					//NewtonCollisionGetMaterial(NewtonBodyGetCollision(childBody), &collisionMaterial);
					//collisionMaterial.m_userData.m_ptr = rootBone;
					//collisionMaterial.m_userParam[0].m_int = jointsDefinition[i].m_type;
					//collisionMaterial.m_userParam[1].m_int = jointsDefinition[i].m_typeMask;
					//NewtonCollisionSetMaterial(NewtonBodyGetCollision(childBody), &collisionMaterial);

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
		
		SetModelMass(100.0f, bodyCount, bodyArray);
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
		ndWorld* const world = scene->GetWorld();
		ndBodyDynamic* const bone = new ndBodyDynamic();
		ndShapeInstance* const shape = entityPart->CreateCollisionFromchildren(world);
		
		dAssert(shape);

		// create the rigid body that will make this bone
		dMatrix matrix(entityPart->CalculateGlobalMatrix());
		bone->SetMatrix(matrix);
		bone->SetCollisionShape(*shape);
		bone->SetMassMatrix(1.0f, *shape);
		bone->SetNotifyCallback(new ndRagdollEntityNotify(scene, entityPart, parentBone));

		world->AddBody(bone);
		delete shape;

		//// assign the material for early collision culling
		//NewtonBodySetMaterialGroupID(bone, m_material);
		//
		//// set the bod part force and torque call back to the gravity force, skip the transform callback
		////NewtonBodySetForceAndTorqueCallback (bone, PhysicsApplyGravityForce);
		//NewtonBodySetForceAndTorqueCallback(bone, ClampAngularVelocity);

		return bone;
	}

	void ConnectBodyParts(ndWorld* const world, ndBodyDynamic* const childBody, ndBodyDynamic* const parentBody, const dJointDefinition& definition) const
	{
		ndJointFix6dof* const joint = new ndJointFix6dof(childBody, parentBody);
		world->AddJoint(joint);

		//dMatrix matrix;
		//NewtonBodyGetMatrix(childBody, &matrix[0][0]);
		//
		//dJointDefinition::dFrameMatrix frameAngle(definition.m_frameBasics);
		//dMatrix pinAndPivotInGlobalSpace(dPitchMatrix(frameAngle.m_pitch * dDegreeToRad) * dYawMatrix(frameAngle.m_yaw * dDegreeToRad) * dRollMatrix(frameAngle.m_roll * dDegreeToRad));
		//pinAndPivotInGlobalSpace = pinAndPivotInGlobalSpace * matrix;
		//
		//dMatrix parentRollMatrix(dGetIdentityMatrix() * pinAndPivotInGlobalSpace);
		//
		//dJointDefinition::dJointLimit jointLimits(definition.m_jointLimits);
		//dCustomBallAndSocket* const joint = new dCustomBallAndSocket(pinAndPivotInGlobalSpace, parentRollMatrix, childBody, parentBody);
		//
		//dFloat32 friction = definition.m_friction * 0.25f;
		//joint->EnableCone(true);
		//joint->SetConeFriction(friction);
		//joint->SetConeLimits(jointLimits.m_coneAngle * dDegreeToRad);
		//
		//joint->EnableTwist(true);
		//joint->SetTwistFriction(friction);
		//joint->SetTwistLimits(jointLimits.m_minTwistAngle * dDegreeToRad, jointLimits.m_maxTwistAngle * dDegreeToRad);
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

	ndDemoEntityManager::ndKeyTrigger m_changeVehicle;
};

void ndBasicRagdoll (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, dGetIdentityMatrix());

	dVector origin1(0.0f, 0.0f, 0.0f, 0.0f);
	fbxDemoEntity* const ragdollMesh = scene->LoadFbxMesh("whiteMan.fbx");

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit.m_y = 0.5f;
	ndRagDollModel* const ragdoll = new ndRagDollModel(scene, ragdollMesh, matrix);
	scene->GetWorld()->AddModel(ragdoll);

	//AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 10, 10, 7);

	delete ragdollMesh;
	dQuaternion rot;
	dVector origin(-10.0f, 1.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);
}
