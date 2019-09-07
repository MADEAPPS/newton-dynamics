/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "toolbox_stdafx.h"
#include "SkyBox.h"
#include "DemoMesh.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "TargaToOpenGl.h"
#include "DemoEntityManager.h"
#include "DebugDisplay.h"
#include "HeightFieldPrimitive.h"


class dDynamicsRagdoll: public dAnimationRagdollRoot
{
	public:
	dDynamicsRagdoll(NewtonBody* const body, const dMatrix& bindMarix)
		:dAnimationRagdollRoot(body, bindMarix)
		,m_hipEffector(NULL)
		,m_leftFootEffector(NULL)
	{
	}

	~dDynamicsRagdoll()
	{
	}

	void PreUpdate(dFloat timestep)
	{
		dAssert(0);
/*
		// do most of the control here, so that the is no need do subclass  
		//dDynamicsRagdoll* const ragDoll = (dDynamicsRagdoll*)model;
		NewtonBody* const rootBody = GetBody();
		NewtonBodySetSleepState(rootBody, 0);

		// calculate the center of mass
		dVector com(CalculateCenterOfMass());

		// get pivot 
		dMatrix rootMatrix;
		dMatrix pivotMatrix;
		NewtonBodyGetMatrix(GetBody(), &rootMatrix[0][0]);
		NewtonBodyGetMatrix(m_leftFootEffector->GetBody(), &pivotMatrix[0][0]);
		
		dVector comRadius (com - pivotMatrix.m_posit);
		dVector bodyRadius (rootMatrix.m_posit - pivotMatrix.m_posit);

		dFloat r0 = dSqrt(comRadius.DotProduct3(comRadius));
		dFloat r1 = dSqrt(bodyRadius.DotProduct3(bodyRadius));

		dVector updir (0.0f, 1.0f, 0.0f, 0.0f);
		dVector p1 (pivotMatrix.m_posit + updir.Scale (r0));
		dVector targtePosit (rootMatrix.m_posit + (p1 - com).Scale (r1/r0));
		targtePosit.m_w = 1.0f;
		
		dMatrix matrix(dPitchMatrix(0.0f * dDegreeToRad) * dYawMatrix(0.0f * dDegreeToRad) * dRollMatrix(0.0f * dDegreeToRad));
		//com.m_y = rootMatrix.m_posit.m_y;
		//com.m_w = 1.0f;
		//matrix.m_posit = rootMatrix.m_posit;
		//matrix.m_posit = com;

		matrix.m_posit = targtePosit;
		m_hipEffector->SetTarget(matrix);

dVector error(matrix.m_posit - rootMatrix.m_posit);
dTrace(("%f %f %f\n", error[0], error[1], error[2]));
*/
		dAnimationRagdollRoot::PreUpdate(timestep);
	}

	void SetHipEffector(dAnimationJoint* const hip)
	{
		m_hipEffector = new dAnimationRagDollEffector(hip);
		m_loopJoints.Append(m_hipEffector);
	}

	void SetFootEffector(dAnimationJoint* const joint)
	{
		m_leftFootEffector = new dAnimationRagDollEffector(joint);
		m_loopJoints.Append(m_leftFootEffector);
	}

	dAnimationRagDollEffector* m_hipEffector;
	dAnimationRagDollEffector* m_leftFootEffector;
};


class dDynamicRagdollManager: public dAnimationModelManager
{
	class dJointDefinition
	{
		public:
		struct dJointLimit
		{
			dFloat m_minTwistAngle;
			dFloat m_maxTwistAngle;
			dFloat m_coneAngle;
		};

		struct dFrameMatrix
		{
			dFloat m_pitch;
			dFloat m_yaw;
			dFloat m_roll;
		};

		char m_boneName[32];
		dFloat m_friction;
		dJointLimit m_jointLimits;
		dFrameMatrix m_frameBasics;
		dAnimationRagdollJoint::dRagdollMotorType m_type;
	};
	
	public:

	dDynamicRagdollManager(DemoEntityManager* const scene)
		:dAnimationModelManager(scene->GetNewton())
//		, m_currentRig(NULL)
	{
//		scene->Set2DDisplayRenderFunction(RenderHelpMenu, NULL, this);
	}

	~dDynamicRagdollManager()
	{
	}

	static void ClampAngularVelocity(const NewtonBody* body, dFloat timestep, int threadIndex)
	{
		dVector omega;
		NewtonBodyGetOmega(body, &omega[0]);
		omega.m_w = 0.0f;
		dFloat mag2 = omega.DotProduct3(omega);
		if (mag2 > (100.0f * 100.0f)) {
			omega = omega.Normalize().Scale(100.0f);
			NewtonBodySetOmega(body, &omega[0]);
		}

		//PhysicsApplyGravityForce(body, timestep, threadIndex);
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		dFloat mass;

		dFloat gravity = -0.0f;
		NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
		dVector dir(0.0f, gravity, 0.0f);
		dVector force(dir.Scale(mass));
		NewtonBodySetForce(body, &force.m_x);
	}

	NewtonBody* CreateBodyPart(DemoEntity* const bodyPart)
	{
		NewtonWorld* const world = GetWorld();
		NewtonCollision* const shape = bodyPart->CreateCollisionFromchildren(world);
		dAssert(shape);

		// calculate the bone matrix
		dMatrix matrix(bodyPart->CalculateGlobalMatrix());

		// create the rigid body that will make this bone
		NewtonBody* const bone = NewtonCreateDynamicBody(world, shape, &matrix[0][0]);

		// calculate the moment of inertia and the relative center of mass of the solid
		//NewtonBodySetMassProperties (bone, definition.m_mass, shape);
		NewtonBodySetMassProperties(bone, 1.0f, shape);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(bone, bodyPart);

		// assign the material for early collision culling
		//NewtonBodySetMaterialGroupID(bone, m_material);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		//NewtonBodySetForceAndTorqueCallback (bone, PhysicsApplyGravityForce);
		NewtonBodySetForceAndTorqueCallback(bone, ClampAngularVelocity);

		// destroy the collision helper shape 
		NewtonDestroyCollision(shape);
		return bone;
	}

	void SetModelMass( dFloat mass, dAnimationJointRoot* const rootNode) const
	{
		dFloat volume = 0.0f;
		for (dAnimationJoint* joint = GetFirstJoint(rootNode); joint; joint = GetNextJoint(joint)) {
			volume += NewtonConvexCollisionCalculateVolume(NewtonBodyGetCollision(joint->GetBody()));
		}
		dFloat density = mass / volume;

		for (dAnimationJoint* joint = GetFirstJoint(rootNode); joint; joint = GetNextJoint(joint)) {
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;
			NewtonBody* const body = joint->GetBody();
			NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
			dFloat scale = density * NewtonConvexCollisionCalculateVolume(NewtonBodyGetCollision(body));
			mass *= scale;
			Ixx *= scale;
			Iyy *= scale;
			Izz *= scale;
			NewtonBodySetMassMatrix(body, mass, Ixx, Iyy, Izz);
		}
	}

	dAnimationJoint* CreateChildNode(NewtonBody* const boneBody, dAnimationJoint* const parent, const dJointDefinition& definition) const
	{
		dMatrix matrix;
		NewtonBodyGetMatrix(boneBody, &matrix[0][0]);

		dJointDefinition::dFrameMatrix frameAngle(definition.m_frameBasics);
		dMatrix pinAndPivotInGlobalSpace(dPitchMatrix(frameAngle.m_pitch * dDegreeToRad) * dYawMatrix(frameAngle.m_yaw * dDegreeToRad) * dRollMatrix(frameAngle.m_roll * dDegreeToRad));
		pinAndPivotInGlobalSpace = pinAndPivotInGlobalSpace * matrix;

		//dMatrix bindMatrix(entity->GetParent()->CalculateGlobalMatrix((DemoEntity*)NewtonBodyGetUserData(parentBody)).Inverse());
		dMatrix bindMatrix(dGetIdentityMatrix());
		dAnimationJoint* const joint = new dAnimationRagdollJoint(definition.m_type, pinAndPivotInGlobalSpace, boneBody, bindMatrix, parent);
		return joint;
	}

	
	void CreateRagdollExperiment_0(const dMatrix& location)
	{
		static dJointDefinition jointsDefinition[] =
		{
			{ "body" },
			{ "leg", 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 180.0f }, dAnimationRagdollJoint::m_threeDof },
			{ "foot", 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 90.0f, 0.0f }, dAnimationRagdollJoint::m_twoDof },
		};
		const int definitionCount = sizeof (jointsDefinition)/sizeof (jointsDefinition[0]);

		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		DemoEntity* const modelEntity = DemoEntity::LoadNGD_mesh("selfbalance_01.ngd", GetWorld(), scene->GetShaderCache());

		dMatrix matrix0(modelEntity->GetCurrentMatrix());
		//matrix0.m_posit = location;
		//modelEntity->ResetMatrix(*scene, matrix0);
		scene->Append(modelEntity);

		// add the root childBody
		NewtonBody* const rootBody = CreateBodyPart(modelEntity);

		// build the rag doll with rigid bodies connected by joints
		dDynamicsRagdoll* const dynamicRagdoll = new dDynamicsRagdoll(rootBody, dGetIdentityMatrix());
		AddModel(dynamicRagdoll);
		dynamicRagdoll->SetCalculateLocalTransforms(true);

		// save the controller as the collision user data, for collision culling
		NewtonCollisionSetUserData(NewtonBodyGetCollision(rootBody), dynamicRagdoll);

		int stackIndex = 0;
		DemoEntity* childEntities[32];
		dAnimationJoint* parentBones[32];

		for (DemoEntity* child = modelEntity->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = dynamicRagdoll;
			childEntities[stackIndex] = child;
			stackIndex++;
		}

		// walk model hierarchic adding all children designed as rigid body bones. 
		while (stackIndex) {
			stackIndex--;
			DemoEntity* const entity = childEntities[stackIndex];
			dAnimationJoint* parentNode = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			for (int i = 0; i < definitionCount; i++) {
				if (!strcmp(jointsDefinition[i].m_boneName, name)) {
					NewtonBody* const childBody = CreateBodyPart(entity);
					// connect this body part to its parent with a rag doll joint
					parentNode = CreateChildNode(childBody, parentNode, jointsDefinition[i]);

					NewtonCollisionSetUserData(NewtonBodyGetCollision(childBody), parentNode);
					break;
				}
			}

			for (DemoEntity* child = entity->GetChild(); child; child = child->GetSibling()) {
				parentBones[stackIndex] = parentNode;
				childEntities[stackIndex] = child;
				stackIndex++;
			}
		}

		SetModelMass (100.0f, dynamicRagdoll);

		// set the collision mask
		// note this container work best with a material call back for setting bit field 
		//dAssert(0);
		//controller->SetDefaultSelfCollisionMask();

		// transform the entire contraction to its location
		dMatrix worldMatrix(modelEntity->GetCurrentMatrix() * location);
		worldMatrix.m_posit = location.m_posit;
		NewtonBodySetMatrixRecursive(rootBody, &worldMatrix[0][0]);

		// attach effectors here
		for (dAnimationJoint* joint = GetFirstJoint(dynamicRagdoll); joint; joint = GetNextJoint(joint)) {
			if (joint->GetAsRoot()) {
				dAssert(dynamicRagdoll == joint);
				dynamicRagdoll->SetHipEffector(joint);
			} else if (joint->GetAsLeaf()) {
				dynamicRagdoll->SetFootEffector(joint);
			}
		}

		dynamicRagdoll->Finalize();
		//return controller;
	}

	void OnPostUpdate(dAnimationJointRoot* const model, dFloat timestep)
	{
		// do nothing for now
	}

	virtual void OnUpdateTransform(const dAnimationJoint* const bone, const dMatrix& localMatrix) const
	{
		// calculate the local transform for this player body
		NewtonBody* const body = bone->GetBody();
		DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(body);
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));

		dQuaternion rot(localMatrix);
		ent->SetMatrix(*scene, rot, localMatrix.m_posit);
	}

	void OnPreUpdate(dAnimationJointRoot* const model, dFloat timestep)
	{
		// call the solver 
		dAnimationModelManager::OnPreUpdate(model, timestep);
	}
};


void DynamicRagdoll(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	CreateLevelMesh(scene, "flatPlane.ngd", true);


	dDynamicRagdollManager* const manager = new dDynamicRagdollManager(scene);
	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID(world);
	NewtonMaterialSetDefaultFriction(world, defaultMaterialID, defaultMaterialID, 1.0f, 1.0f);
	NewtonMaterialSetDefaultElasticity(world, defaultMaterialID, defaultMaterialID, 0.0f);

	dMatrix origin (dYawMatrix(0.0f * dDegreeToRad));
	origin.m_posit.m_y = 2.0f;
	manager->CreateRagdollExperiment_0(origin);
/*
//	int count = 10;
//	count = 1;
//origin = dGetIdentityMatrix();
	origin.m_posit.m_x = 2.0f;
//	origin.m_posit.m_y = 2.1f;
	origin.m_posit.m_y = 3.0f;
	for (int i = 0; i < count; i++) {
		manager->CreateRagDoll(scene, origin);
		//manager->CreateRagDoll (scene, origin1);
		origin.m_posit.m_x += 1.0f;
	}
*/
	origin.m_posit.m_x = -3.0f;
	origin.m_posit.m_y = 1.0f;
	origin.m_posit.m_z = 0.0f;
	scene->SetCameraMatrix(dGetIdentityMatrix(), origin.m_posit);
}




