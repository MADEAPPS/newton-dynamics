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
	enum jointType
	{
		m_root,
		m_hinge,
		m_spherical,
		m_effector
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
	jointType m_type;
	dJointLimit m_jointLimits;
	dFrameMatrix m_frameBasics;
};

static dJointDefinition mannequinDefinition[] =
{
	{ "pelvis", dJointDefinition::m_root, {}, {} },
	//{ "Bip001 Spine1", 1.0f,{ -30.0f, 30.0f, 60.0f },{ 0.0f, 90.0f, 0.0f } },
	//{ "Bip001 Head", 1.0f,{ -60.0f, 60.0f, 60.0f },{ 0.0f, 90.0f, 0.0f } },

	{ "rightLeg", dJointDefinition::m_spherical, { -45.0f, 45.0f, 80.0f }, { 0.0f, 90.0f, 0.0f } },
	{ "rightCalf", dJointDefinition::m_hinge, { 0.0f, 120.0f, 0.0f }, { 0.0f, 0.0f, -90.0f } },
	//{ "Bip001 L Foot", 1.0f,{ 0.0f, 0.0f, 60.0f },{ 0.0f, 90.0f, 0.0f } },

	//{ "leftLeg", dJointDefinition::m_spherical, { -45.0f, 45.0f, 80.0f }, { 0.0f, 90.0f, 0.0f } },
	//{ "leftCalf", dJointDefinition::m_hinge, { 0.0f, 120.0f, 0.0f }, { 0.0f, 0.0f, -90.0f } },
	//{ "Bip001 R Foot", 1.0f,{ 0.0f, 0.0f, 60.0f },{ 0.0f, 90.0f, 0.0f } },

	{ "", dJointDefinition::m_root,{ 0.0f, 0.0f, 0.0f },{ 0.0f, 0.0f, 0.0f } },
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

	void OnTransform(ndInt32 thread, const ndMatrix& matrix)
	{
		if (!m_parentBody)
		{
			ndDemoEntityNotify::OnTransform(thread, matrix);
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
	ndRagdollModel(ndDemoEntityManager* const scene, fbxDemoEntity* const ragdollMesh, const ndMatrix& location, dJointDefinition* const definition)
		:ndModel()
	{
		// make a clone of the mesh and add it to the scene
		ndDemoEntity* const rootEntity = (ndDemoEntity*)ragdollMesh->CreateClone();
		scene->AddEntity(rootEntity);
		ndWorld* const world = scene->GetWorld();

		ndMatrix matrix(rootEntity->CalculateGlobalMatrix() * location);
		rootEntity->ResetMatrix(dGetIdentityMatrix());

		// find the floor location 
		ndVector floor(FindFloor(*world, matrix.m_posit + ndVector(0.0f, 100.0f, 0.0f, 0.0f), 200.0f));
		matrix.m_posit.m_y = floor.m_y + 1.0f;
		rootEntity->ResetMatrix(matrix);

		ndFixSizeArray<ndBodyDynamic*, 64> bodies;
		// add the root body
		ndBodyDynamic* const rootBody = CreateBodyPart(scene, rootEntity, nullptr);
		m_rootBody = rootBody;
		bodies.PushBack(m_rootBody);

		ndInt32 stack = 0;
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
		
		// walk model hierarchic adding all children designed as rigid body bones. 
		while (stack) 
		{
			stack--;
			ndBodyDynamic* parentBone = parentBones[stack];
			ndDemoEntity* const childEntity = childEntities[stack];
			const char* const name = childEntity->GetName().GetStr();
			//dTrace(("name: %s\n", name));
			for (ndInt32 i = 0; definition[i].m_boneName[0]; ++i)
			{
				if (!strcmp(definition[i].m_boneName, name))
				{
					ndBodyDynamic* const childBody = CreateBodyPart(scene, childEntity, parentBone);
					bodies.PushBack(childBody);
		
					// connect this body part to its parentBody with a ragdoll joint
					ndJointBilateralConstraint* joint = ConnectBodyParts(childBody, parentBone, definition[i]);
					world->AddJoint(joint);
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
		
		SetModelMass(bodies, 100.0f);
	}

	void SetModelMass(const ndFixSizeArray<ndBodyDynamic*, 64>& bodies, ndFloat32 mass) const
	{
		ndFloat32 volume = 0.0f;
		for (int i = 0; i < bodies.GetCount(); ++i) 
		{
			volume += bodies[i]->GetCollisionShape().GetVolume();
		}
		ndFloat32 density = mass / volume;

		for (ndInt32 i = 0; i < bodies.GetCount(); ++i)
		{
			ndBodyDynamic* const body = bodies[i];
			ndFloat32 normalMass = density * body->GetCollisionShape().GetVolume();
			body->SetMassMatrix(normalMass, body->GetCollisionShape());
			ndVector inertia(body->GetMassMatrix());
			ndFloat32 maxInertia = ndMax(ndMax(inertia.m_x, inertia.m_y), inertia.m_z);
			ndFloat32 minInertia = ndMin(ndMin(inertia.m_x, inertia.m_y), inertia.m_z);
			if (minInertia < maxInertia * 0.125f)
			{
				minInertia = maxInertia * 0.125f;
				for (ndInt32 j = 0; j < 3; j++)
				{
					if (inertia[j] < minInertia)
					{
						inertia[j] = minInertia;
					}
				}
			}
			body->SetMassMatrix(inertia);
		}
	}
	
	ndBodyDynamic* CreateBodyPart(ndDemoEntityManager* const scene, ndDemoEntity* const entityPart, ndBodyDynamic* const parentBone)
	{
		ndShapeInstance* const shape = entityPart->CreateCollisionFromChildren();
		dAssert(shape);

		// create the rigid body that will make this body
		ndMatrix matrix(entityPart->CalculateGlobalMatrix());

		ndBodyDynamic* const body = new ndBodyDynamic();
		body->SetMatrix(matrix);
		body->SetCollisionShape(*shape);
		body->SetMassMatrix(1.0f, *shape);
		body->SetNotifyCallback(new ndRagdollEntityNotify(scene, entityPart, parentBone));

		scene->GetWorld()->AddBody(body);
		delete shape;
		return body;
	}

	ndJointBilateralConstraint* ConnectBodyParts(ndBodyDynamic* const childBody, ndBodyDynamic* const parentBone, const dJointDefinition& definition)
	{
		ndMatrix matrix(childBody->GetMatrix());
		dJointDefinition::dFrameMatrix frameAngle(definition.m_frameBasics);
		ndMatrix pinAndPivotInGlobalSpace(dPitchMatrix(frameAngle.m_pitch * ndDegreeToRad) * dYawMatrix(frameAngle.m_yaw * ndDegreeToRad) * dRollMatrix(frameAngle.m_roll * ndDegreeToRad) * matrix);

		switch (definition.m_type)
		{
			case dJointDefinition::m_spherical:
			{
				ndIkJointSpherical* const joint = new ndIkJointSpherical(pinAndPivotInGlobalSpace, childBody, parentBone);

				dJointDefinition::dJointLimit jointLimits(definition.m_jointLimits);
				//joint->SetConeLimit(jointLimits.m_coneAngle * ndDegreeToRad);
				//joint->SetTwistLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
				return joint;
			}

			case dJointDefinition::m_hinge:
			{
				ndIkJointHinge* const joint = new ndIkJointHinge(pinAndPivotInGlobalSpace, childBody, parentBone);

				dJointDefinition::dJointLimit jointLimits(definition.m_jointLimits);
				joint->SetLimitState(true);
				joint->SetLimits(jointLimits.m_minTwistAngle * ndDegreeToRad, jointLimits.m_maxTwistAngle * ndDegreeToRad);
				return joint;
			}

			default:
				dAssert(0);
		}
		return nullptr;
	}

	void Update(ndWorld* const world, ndFloat32 timestep) 
	{
		ndModel::Update(world, timestep);

		//bool needProjection = false;
		//ndFloat32 invtimestep2 = 1.0f / (timestep * timestep);
		//for (ndInt32 i = 0; (i < m_bodies.GetCount()) && !needProjection; ++i)
		//{
		//	ndBodyDynamic* const body = m_bodies[i];
		//	const ndVector veloc(body->GetOmega());
		//	const ndVector omega(body->GetVelocity());
		//	ndFloat32 maxVeloc2 = body->GetMaxLinearStep() * body->GetMaxLinearStep() * invtimestep2;
		//	ndFloat32 maxOmega2 = body->GetMaxAngularStep() * body->GetMaxAngularStep() * invtimestep2;
		//	needProjection = needProjection || (omega.DotProduct(omega).GetScalar() > maxOmega2);
		//	needProjection = needProjection || (veloc.DotProduct(veloc).GetScalar() > maxVeloc2);
		//}
		//if (needProjection)
		//{
		//	ndSkeletonContainer* const skeleton = m_bodies[0]->GetSkeleton();
		//	m_solver.ProjectVelocities(skeleton, world, timestep);
		//}
	}

	//void PostUpdate(ndWorld* const world, ndFloat32)
	void PostUpdate(ndWorld* const, ndFloat32)
	{
	}

	//void PostTransformUpdate(ndWorld* const world, ndFloat32 timestep)
	void PostTransformUpdate(ndWorld* const, ndFloat32)
	{
		//ndInt32 sleepCount = 0;
		//for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
		//{
		//	ndBodyDynamic* const body = m_bodies[i];
		//	sleepCount += body->GetSleepState() ? 1 : 0;
		//}
		//if ((sleepCount > 0) && (sleepCount < m_bodies.GetCount()))
		//{
		//	for (ndInt32 i = 0; i < m_bodies.GetCount(); ++i)
		//	{
		//		ndBodyDynamic* const body = m_bodies[i];
		//		if (body->GetSleepState() == true) 
		//		{
		//			ndBodyNotify* const notify = body->GetNotifyCallback();
		//			dAssert(notify);
		//			notify->OnTransform(0, body->GetMatrix());
		//		}
		//	}
		//}
	}

	ndBodyDynamic* m_rootBody;
};

void BuildMannequin(ndDemoEntityManager* const scene, const ndVector& origin)
{
	ndMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;

	matrix.m_posit.m_y = 0.5f;
	fbxDemoEntity* const ragdollMesh = scene->LoadFbxMesh("mannequin.fbx");

	ndMatrix entMatrix(dYawMatrix(-90.0f * ndDegreeToRad) * ragdollMesh->GetRenderMatrix());
	ragdollMesh->ResetMatrix(entMatrix);

	ndWorld* const world = scene->GetWorld();
	ndRagdollModel* const ragdoll = new ndRagdollModel(scene, ragdollMesh, matrix, mannequinDefinition);
	world->AddModel(ragdoll);
	world->AddJoint(new ndJointFix6dof(ragdoll->m_rootBody->GetMatrix(), ragdoll->m_rootBody, world->GetSentinelBody()));

	//matrix.m_posit.m_x += 2.0f;
	//matrix.m_posit.m_z -= 2.0f;
	//scene->GetWorld()->AddModel(new ndRagdollModel(scene, ragdollMesh, matrix, mannequinDefinition));
	//
	//matrix.m_posit.m_z = 2.0f;
	//scene->GetWorld()->AddModel(new ndRagdollModel(scene, ragdollMesh, matrix, mannequinDefinition));

	delete ragdollMesh;
}

void ndBasicRagdoll (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, dGetIdentityMatrix());

	ndMatrix origin(dGetIdentityMatrix());
	origin.m_posit.m_x += 20.0f;
	//AddCapsulesStacks(scene, origin, 10.0f, 0.25f, 0.25f, 0.5f, 10, 10, 7);

	origin.m_posit.m_x -= 20.0f;
	BuildMannequin(scene, origin.m_posit); 

	ndQuaternion rot;
	origin.m_posit.m_x -= 5.0f;
	origin.m_posit.m_y = 2.0f;
	scene->SetCameraMatrix(rot, origin.m_posit);
}
