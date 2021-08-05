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

class dActiveJointDefinition
{
	public:
	enum dLimbType
	{
		fowardKinematic,
		inverseKinematic,
		effector,
	};

	struct dJointPidData
	{
		dJointPidData()
			:m_spring(1000.0f)
			,m_damper(40.0f)
			,m_regularizer(0.005f)
		{
		}
		dFloat32 m_spring;
		dFloat32 m_damper;
		dFloat32 m_regularizer;
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
	dLimbType m_limbType;
	dJointLimit m_jointLimits;
	dFrameMatrix m_frameBasics;
	dJointPidData m_jointData;
};

class ndActiveRagdollEntityNotify : public ndDemoEntityNotify
{
	public:
	ndActiveRagdollEntityNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity, ndBodyDynamic* const parentBody)
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

static dActiveJointDefinition jointsDefinition[] =
{
	{ "mixamorig:Hips", dActiveJointDefinition::fowardKinematic},
	
	{ "mixamorig:Spine", dActiveJointDefinition::fowardKinematic, { -15.0f, 15.0f,  30.0f }, { 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:Spine1", dActiveJointDefinition::fowardKinematic, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:Spine2", dActiveJointDefinition::fowardKinematic, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:Neck", dActiveJointDefinition::fowardKinematic, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 180.0f } },
	
	{ "mixamorig:RightArm", dActiveJointDefinition::fowardKinematic, { -45.0f, 45.0f, 80.0f }, { 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:RightForeArm", dActiveJointDefinition::fowardKinematic, { -140.0f, 10.0f, 0.0f }, { 0.0f, 00.0f, 90.0f } },
	{ "mixamorig:RightHand", dActiveJointDefinition::fowardKinematic, { 0.0f, 0.0f, 60.0f }, { 0.0f, 0.0f, 180.0f } },
	
	{ "mixamorig:LeftArm", dActiveJointDefinition::fowardKinematic, { -45.0f, 45.0f, 80.0f }, { 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:LeftForeArm", dActiveJointDefinition::fowardKinematic, { -140.0f, 10.0f, 0.0f }, { 0.0f, 0.0f, -90.0f } },
	{ "mixamorig:LeftHand", dActiveJointDefinition::fowardKinematic, { 0.0f, 0.0f, 60.0f }, { 0.0f, 0.0f, 180.0f } },
	
	{ "mixamorig:RightUpLeg", dActiveJointDefinition::inverseKinematic, { -45.0f, 45.0f, 120.0f }, { 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:RightLeg", dActiveJointDefinition::inverseKinematic, { -140.0f, 10.0f, 0.0f }, { 0.0f, 90.0f, 90.0f } },
	{ "mixamorig:RightFoot", dActiveJointDefinition::inverseKinematic, { 0.0f, 0.0f, 60.0f }, { 0.0f, 0.0f, 180.0f } },
	{ "rightFoot_effector", dActiveJointDefinition::effector },
	
	{ "mixamorig:LeftUpLeg", dActiveJointDefinition::inverseKinematic, { -45.0f, 45.0f, 120.0f }, { 0.0f, 0.0f, 180.0f } },
	{ "mixamorig:LeftLeg", dActiveJointDefinition::inverseKinematic, { -140.0f, 10.0f, 0.0f }, { 0.0f, 90.0f, 90.0f } },
	{ "mixamorig:LeftFoot", dActiveJointDefinition::inverseKinematic, { 0.0f, 0.0f, 60.0f }, { 0.0f, 0.0f, 180.0f } },
	{ "leftFoot_effector", dActiveJointDefinition::effector },
};

class ndActiveRagdollModel : public ndCharacter
{
	public:
	ndActiveRagdollModel(ndDemoEntityManager* const scene, fbxDemoEntity* const ragdollMesh, const dMatrix& location)
	{
		// make a clone of the mesh and add it to the scene
		ndDemoEntity* const entity = ragdollMesh->CreateClone();
		scene->AddEntity(entity);
		ndWorld* const world = scene->GetWorld();
		
		// find the floor location 
		dMatrix matrix(location);
		dVector floor(FindFloor(*world, matrix.m_posit + dVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		matrix.m_posit.m_y = floor.m_y + 0.1f;

		// add the root body
		ndDemoEntity* const rootEntity = (ndDemoEntity*)entity->Find(jointsDefinition[0].m_boneName);
		rootEntity->ResetMatrix(rootEntity->GetCurrentMatrix() * matrix);
		ndCharacterRootNode* const rootNode = CreateRoot(CreateBodyPart(scene, rootEntity, nullptr));

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
		bodyArray[0] = rootNode->GetBody();
		
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
					if (jointsDefinition[i].m_limbType != dActiveJointDefinition::effector)
					{
						ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, parentBone->GetBody());
						bodyArray[bodyCount] = childBody;
						bodyCount++;

						// connect this body part to its parentBody with a ragdoll joint
						parentBone = ConnectBodyParts(world, childBody, parentBone, jointsDefinition[i]);
					}
					else
					{
						ndCharacterLimbNode* const referenceNode = parentBone->GetParent()->GetParent()->GetParent();
						dMatrix effectorMatrix(childEntity->GetCurrentMatrix() * parentBone->GetBody()->GetMatrix());
						ndCharacterEffectorNode* const effectorNode = CreateInverseDynamicEffector(effectorMatrix, parentBone, referenceNode);
						ndJointBilateralConstraint* const effectorJoint = effectorNode->GetJoint();
						world->AddJoint(effectorJoint);
					}
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
			if (ent->GetName() == "mixamorig:Hips") 
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
		ndWorld* const world = scene->GetWorld();
		ndShapeInstance* const shape = entityPart->CreateCollisionFromchildren(world);
		dAssert(shape);

		// create the rigid body that will make this body
		dMatrix matrix(entityPart->CalculateGlobalMatrix());

		ndBodyDynamic* const body = new ndBodyDynamic();
		body->SetMatrix(matrix);
		body->SetCollisionShape(*shape);
		body->SetMassMatrix(1.0f, *shape);
		body->SetNotifyCallback(new ndActiveRagdollEntityNotify(scene, entityPart, parentBone));
		world->AddBody(body);

		delete shape;
		return body;
	}

	ndCharacterLimbNode* ConnectBodyParts(ndWorld* const world, ndBodyDynamic* const childBody, ndCharacterLimbNode* const parentNode, const dActiveJointDefinition& definition)
	{
		dMatrix matrix(childBody->GetMatrix());
		dActiveJointDefinition::dFrameMatrix frameAngle(definition.m_frameBasics);
		dMatrix pinAndPivotInGlobalSpace(dPitchMatrix(frameAngle.m_pitch * dDegreeToRad) * dYawMatrix(frameAngle.m_yaw * dDegreeToRad) * dRollMatrix(frameAngle.m_roll * dDegreeToRad) * matrix);

		if (definition.m_limbType == dActiveJointDefinition::fowardKinematic)
		{
			ndCharacterFowardDynamicNode* const jointNode = CreateFowardDynamicLimb(pinAndPivotInGlobalSpace, childBody, parentNode);

			dActiveJointDefinition::dJointLimit jointLimits(definition.m_jointLimits);
			ndJointPid3dofActuator* const joint = (ndJointPid3dofActuator*)jointNode->GetJoint();

			joint->SetConeLimit(jointLimits.m_coneAngle * dDegreeToRad);
			//joint->SetConeLimit(0.0f);
			//joint->SetConeFriction(0.05f, definition.m_friction);

			joint->SetTwistLimits(jointLimits.m_minTwistAngle * dDegreeToRad, jointLimits.m_maxTwistAngle * dDegreeToRad);
			//joint->SetTwistFriction(0.05f, definition.m_friction);

			world->AddJoint(joint);
			return jointNode;
		}
		else
		{
			ndCharacterInverseDynamicNode* const jointNode = CreateInverseDynamicLimb(pinAndPivotInGlobalSpace, childBody, parentNode);

			dActiveJointDefinition::dJointLimit jointLimits(definition.m_jointLimits);
			ndJointBallAndSocket* const joint = (ndJointBallAndSocket*)jointNode->GetJoint();

			joint->SetConeLimit(jointLimits.m_coneAngle * dDegreeToRad);
			//joint->SetConeLimit(0.0f);
			//joint->SetConeFriction(0.05f, definition.m_friction);

			joint->SetTwistLimits(jointLimits.m_minTwistAngle * dDegreeToRad, jointLimits.m_maxTwistAngle * dDegreeToRad);
			//joint->SetTwistFriction(0.05f, definition.m_friction);

			world->AddJoint(joint);
			return jointNode;
		}
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

void ndActiveRagdoll (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, dGetIdentityMatrix());

	dVector origin1(0.0f, 0.0f, 0.0f, 0.0f);
	fbxDemoEntity* const ragdollMesh = scene->LoadFbxMesh("whiteMan.fbx");

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit.m_y = 0.5f;
	dMatrix playerMatrix(matrix);
	ndActiveRagdollModel* const ragdoll = new ndActiveRagdollModel(scene, ragdollMesh, matrix);
	scene->GetWorld()->AddModel(ragdoll);

	matrix.m_posit.m_x += 2.0f;
	matrix.m_posit.m_z -= 2.0f;
	//scene->GetWorld()->AddModel(new ndActiveRagdollModel(scene, ragdollMesh, matrix));

	matrix.m_posit.m_z = 2.0f;
	//scene->GetWorld()->AddModel(new ndActiveRagdollModel(scene, ragdollMesh, matrix));

	origin1.m_x += 20.0f;
	//AddCapsulesStacks(scene, origin1, 10.0f, 0.25f, 0.25f, 0.5f, 10, 10, 7);

	delete ragdollMesh;
	dQuaternion rot(dYawMatrix (90.0f * dDegreeToRad) * playerMatrix);
	dVector origin(playerMatrix.m_posit + dVector (0.0f, 1.0f, 2.5f, 0.0f));
	scene->SetCameraMatrix(rot, origin);
}
