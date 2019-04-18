/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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


class dAnimationEndEffectorTest: public dAnimationRagDollEffector, public dCustomJoint
{
	public:
	dAnimationEndEffectorTest(dAnimationJointRoot* const root)
		:dAnimationRagDollEffector(root)
		,dCustomJoint(6, root->GetBody(), NULL)
	{
		SetSolverModel(3);
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		const dMatrix& matrix1 = m_targetMatrix;
		dMatrix matrix0;

		NewtonBodyGetMatrix(GetBody0(), &matrix0[0][0]);
		matrix0 = m_localMatrix * matrix0;

		const dVector relPosit(matrix1.UnrotateVector(matrix1.m_posit - matrix0.m_posit));
		const dVector relPositDir = relPosit.Scale(1.0f / dSqrt(relPosit.DotProduct3(relPosit) + 1.0e-6f));

		dAssert(m_maxLinearSpeed >= 0.0f);
		const dFloat invTimestep = 1.0f / timestep;
		const dFloat linearStep = m_maxLinearSpeed * timestep;
		for (int i = 0; i < 3; i++) {
			NewtonUserJointAddLinearRow(m_joint, &matrix0.m_posit[0], &matrix0.m_posit[0], &matrix1[i][0]);
			const dFloat stopAccel = NewtonUserJointCalculateRowZeroAccelaration(m_joint);
			dFloat speed = ClipParam(relPosit[i], linearStep * dAbs(relPositDir[i])) * invTimestep;
			dFloat relAccel = speed * invTimestep + stopAccel;
			NewtonUserJointSetRowAcceleration(m_joint, relAccel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_linearFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_linearFriction);
		}

		const dVector& coneDir0 = matrix0.m_front;
		const dVector& coneDir1 = matrix1.m_front;

		const dFloat angleStep = m_maxAngularSpeed * timestep;
		dFloat cosAngleCos = coneDir1.DotProduct3(coneDir0);
		if (cosAngleCos < 0.9999f) {
			dMatrix coneRotation(dGetIdentityMatrix());
			dVector lateralDir(coneDir1.CrossProduct(coneDir0));
			dFloat mag2 = lateralDir.DotProduct3(lateralDir);
			if (mag2 > 1.0e-4f) {
				lateralDir = lateralDir.Scale(1.0f / dSqrt(mag2));
				coneRotation = dMatrix(dQuaternion(lateralDir, dAcos(dClamp(cosAngleCos, dFloat(-1.0f), dFloat(1.0f)))), matrix1.m_posit);
			} else {
				lateralDir = matrix0.m_up.Scale(-1.0f);
				coneRotation = dMatrix(dQuaternion(matrix0.m_up, dFloat(180.0f) * dDegreeToRad), matrix1.m_posit);
			}

			dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
			dAssert(dAbs(pitchMatrix[0][0] - dFloat(1.0f)) < dFloat(1.0e-3f));
			dAssert(dAbs(pitchMatrix[0][1]) < dFloat(1.0e-3f));
			dAssert(dAbs(pitchMatrix[0][2]) < dFloat(1.0e-3f));

			dFloat pitchAngle = dAtan2(pitchMatrix[1][2], pitchMatrix[1][1]);
			dFloat coneAngle = -dAcos(dClamp(cosAngleCos, dFloat(-1.0f), dFloat(1.0f)));
			//dTrace(("cone:%f pitch:%f\n", coneAngle * dRadToDegree, pitchAngle * dRadToDegree));

			dVector angleDir(pitchAngle, coneAngle, dFloat(0.0f), dFloat(0.0f));
			angleDir = angleDir.Scale(1.0f / dSqrt(angleDir.DotProduct3(angleDir) + 1.0e-6f));

			{
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0[0][0]);
				const dFloat stopAlpha = NewtonUserJointCalculateRowZeroAccelaration(m_joint);
				dFloat omega = ClipParam(pitchAngle, angleStep * dAbs(angleDir[0])) * invTimestep;
				dFloat relAlpha = omega * invTimestep + stopAlpha;
				NewtonUserJointSetRowAcceleration(m_joint, relAlpha);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
			}

			{
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
				const dFloat stopAlpha = NewtonUserJointCalculateRowZeroAccelaration(m_joint);
				dFloat omega = ClipParam(coneAngle, angleStep * dAbs(angleDir[1])) * invTimestep;
				dFloat relAlpha = omega * invTimestep + stopAlpha;
				NewtonUserJointSetRowAcceleration(m_joint, relAlpha);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
			}

			{
				dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
				const dFloat stopAlpha = NewtonUserJointCalculateRowZeroAccelaration(m_joint);
				NewtonUserJointSetRowAcceleration(m_joint, stopAlpha);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
			}

		} else {

			// using small angular aproximation to get the joint angle;
			{
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0[0][0]);
				const dFloat stopAlpha = NewtonUserJointCalculateRowZeroAccelaration(m_joint);
				dFloat angle = dCustomJoint::CalculateAngle(&matrix0[1][0], &matrix1[1][0], &matrix0[0][0]);
				dFloat omega = ClipParam(angle, angleStep) * invTimestep;
				dFloat relAlpha = omega * invTimestep + stopAlpha;
				NewtonUserJointSetRowAcceleration(m_joint, relAlpha);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
			}

			{
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1[1][0]);
				const dFloat stopAlpha = NewtonUserJointCalculateRowZeroAccelaration(m_joint);
				dFloat angle = matrix1[2].DotProduct3(matrix1[0]);
				dFloat omega = ClipParam(angle, angleStep) * invTimestep;
				dFloat relAlpha = omega * invTimestep + stopAlpha;
				NewtonUserJointSetRowAcceleration(m_joint, relAlpha);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
			}

			{
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1[2][0]);
				const dFloat stopAlpha = NewtonUserJointCalculateRowZeroAccelaration(m_joint);
				dFloat angle = matrix1[1].DotProduct3(matrix1[0]);
				dFloat omega = ClipParam(angle, angleStep) * invTimestep;
				dFloat relAlpha = omega * invTimestep + stopAlpha;
				NewtonUserJointSetRowAcceleration(m_joint, relAlpha);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
			}
		}
	}
};

class dDynamicsRagdollTest: public dAnimationRagdollRoot
{
	public:
	dDynamicsRagdollTest(NewtonBody* const body, const dMatrix& bindMarix)
		:dAnimationRagdollRoot(body, bindMarix)
		,m_hipEffector(NULL)
	{
	}

	~dDynamicsRagdollTest()
	{
	}

	void PreUpdate(dFloat timestep)
	{
		NewtonBodySetSleepState(GetBody(), 0);
		m_hipEffector->SetTarget();
		//dAnimationRagdollRoot::PreUpdate(timestep);
	}

	dAnimationRagDollEffector* m_hipEffector;
};


class dDynamicsRagdoll: public dAnimationRagdollRoot
{
	public:
	dDynamicsRagdoll(NewtonBody* const body, const dMatrix& bindMarix)
		:dAnimationRagdollRoot(body, bindMarix)
		,m_xxxxx(NULL)
		,m_hipEffector(NULL)
	{
	}

	~dDynamicsRagdoll()
	{
	}

	void PreUpdate(dFloat timestep)
	{
		NewtonBodySetSleepState(GetBody(), 0);
		m_hipEffector->SetTarget();
		dAnimationRagdollRoot::PreUpdate(timestep);
	}

	void SetHipEffector(dAnimationJoint* const hip)
	{
		m_hipEffector = new dAnimationRagDollEffector(hip);
		m_loopJoints.Append(m_hipEffector);
	}

	void SetFootEffector(dAnimationJoint* const joint)
	{
		m_xxxxx = joint;
	}

	dAnimationJoint* m_xxxxx;
	dAnimationRagDollEffector* m_hipEffector;
};


class DynamicRagdollManager: public dAnimationModelManager
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

	DynamicRagdollManager(DemoEntityManager* const scene)
		:dAnimationModelManager(scene->GetNewton())
//		, m_currentRig(NULL)
	{
//		scene->Set2DDisplayRenderFunction(RenderHelpMenu, NULL, this);
	}

	~DynamicRagdollManager()
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

		dFloat gravity = -10.0f;
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

	void DynamicsRagdollExperiment_0(const dMatrix& location)
	{
		static dJointDefinition jointsDefinition[] =
		{
			{ "body" },
			{ "leg", 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 180.0f }, dAnimationRagdollJoint::m_threeDof },
			{ "foot", 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 90.0f, 0.0f }, dAnimationRagdollJoint::m_twoDof },
		};
		const int definitionCount = sizeof(jointsDefinition) / sizeof(jointsDefinition[0]);

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
		dDynamicsRagdollTest* const dynamicRagdoll = new dDynamicsRagdollTest(rootBody, dGetIdentityMatrix());
		AddModel(dynamicRagdoll);
		dynamicRagdoll->SetCalculateLocalTransforms(true);

		// attach a Hip effector to the root body
		dynamicRagdoll->m_hipEffector = new dAnimationEndEffectorTest(dynamicRagdoll);
		dynamicRagdoll->GetLoops().Append(dynamicRagdoll->m_hipEffector);

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

		SetModelMass(100.0f, dynamicRagdoll);

		// set the collision mask
		// note this container work best with a material call back for setting bit field 
		//dAssert(0);
		//controller->SetDefaultSelfCollisionMask();

		// transform the entire contraction to its location
		dMatrix worldMatrix(modelEntity->GetCurrentMatrix() * location);
		worldMatrix.m_posit = location.m_posit;
		NewtonBodySetMatrixRecursive(rootBody, &worldMatrix[0][0]);

		dynamicRagdoll->Finalize();
		//return controller;
	}


	void DynamicsRagdollExperiment_1(const dMatrix& location)
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
		// do most fo the controll here, so that teh is no need do subclass  

		// calculate the center of mass
		dVector com(0.0f);
		dFloat totalMass = 0.0f;
		for (dAnimationJoint* joint = GetFirstJoint(model); joint; joint = GetNextJoint(joint)) {
			dMatrix matrix;
			dVector localCom;
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;
			dFloat mass;

			NewtonBody* const body = joint->GetBody();
			NewtonBodyGetMatrix(body, &matrix[0][0]);
			NewtonBodyGetCentreOfMass(body, &localCom[0]);
			NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);

			totalMass += mass;
			com += matrix.TransformVector(localCom).Scale(mass);
		}
		com = com.Scale(1.0f / totalMass);


		dDynamicsRagdoll* const ragDoll = (dDynamicsRagdoll*)model;
		NewtonBody* const rootBody = ragDoll->GetBody();
		NewtonBodySetSleepState(rootBody, 0);
		ragDoll->m_hipEffector->SetTarget();

		// call the solver 
		dAnimationModelManager::OnPreUpdate(model, timestep);
	}
};


void DynamicRagDoll(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	CreateLevelMesh(scene, "flatPlane.ngd", true);


	DynamicRagdollManager* const manager = new DynamicRagdollManager(scene);
	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID(world);
	NewtonMaterialSetDefaultFriction(world, defaultMaterialID, defaultMaterialID, 1.0f, 1.0f);
	NewtonMaterialSetDefaultElasticity(world, defaultMaterialID, defaultMaterialID, 0.0f);

	dMatrix origin (dYawMatrix(0.0f * dDegreeToRad));
	origin.m_posit.m_y = 2.0f;
//	manager->DynamicsRagdollExperiment_0(origin);
	manager->DynamicsRagdollExperiment_1(origin);
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




