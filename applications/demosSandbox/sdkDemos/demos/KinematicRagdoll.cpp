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

//#define OLD_KINEMATIC_RAGDOLL

#ifdef OLD_KINEMATIC_RAGDOLL

class dKinematicBoneDefinition
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

static dKinematicBoneDefinition tredDefinition[] =
{
	{ "bone_pelvis" },

	{ "bone_rightLeg", 100.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, 90.0f, 0.0f }, dAnimationRagdollJoint::m_threeDof },
	{ "bone_righCalf", 100.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 90.0f }, dAnimationRagdollJoint::m_oneDof },
	{ "bone_rightAnkle", 100.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 90.0f }, dAnimationRagdollJoint::m_zeroDof },
	{ "bone_rightFoot", 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, -90.0f, 0.0f }, dAnimationRagdollJoint::m_twoDof },

	{ "bone_leftLeg", 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 90.0f, 0.0f }, dAnimationRagdollJoint::m_threeDof },
	{ "bone_leftCalf", 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 90.0f }, dAnimationRagdollJoint::m_oneDof },
	{ "bone_leftAnkle", 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 90.0f }, dAnimationRagdollJoint::m_zeroDof },
	{ "bone_leftFoot", 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, -90.0f, 0.0f }, dAnimationRagdollJoint::m_twoDof },
};

class dKinematicEndEffector: public dAnimationRagDollEffector, public dCustomJoint
{
	public:
	dKinematicEndEffector(dAnimationJoint* const joint)
		:dAnimationRagDollEffector(joint)
		,dCustomJoint(6, joint->GetBody(), NULL)
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
			const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
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
				const dFloat stopAlpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
				dFloat omega = ClipParam(pitchAngle, angleStep * dAbs(angleDir[0])) * invTimestep;
				dFloat relAlpha = omega * invTimestep + stopAlpha;
				NewtonUserJointSetRowAcceleration(m_joint, relAlpha);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
			}

			{
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &lateralDir[0]);
				const dFloat stopAlpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
				dFloat omega = ClipParam(coneAngle, angleStep * dAbs(angleDir[1])) * invTimestep;
				dFloat relAlpha = omega * invTimestep + stopAlpha;
				NewtonUserJointSetRowAcceleration(m_joint, relAlpha);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
			}

			{
				dVector sideDir(lateralDir.CrossProduct(matrix0.m_front));
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &sideDir[0]);
				const dFloat stopAlpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
				NewtonUserJointSetRowAcceleration(m_joint, stopAlpha);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
			}

		} else {

			// using small angular aproximation to get the joint angle;
			{
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0[0][0]);
				const dFloat stopAlpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
				dFloat angle = dCustomJoint::CalculateAngle(&matrix0[1][0], &matrix1[1][0], &matrix0[0][0]);
				dFloat omega = ClipParam(angle, angleStep) * invTimestep;
				dFloat relAlpha = omega * invTimestep + stopAlpha;
				NewtonUserJointSetRowAcceleration(m_joint, relAlpha);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
			}

			{
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1[1][0]);
				const dFloat stopAlpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
				dFloat angle = matrix1[2].DotProduct3(matrix1[0]);
				dFloat omega = ClipParam(angle, angleStep) * invTimestep;
				dFloat relAlpha = omega * invTimestep + stopAlpha;
				NewtonUserJointSetRowAcceleration(m_joint, relAlpha);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
			}

			{
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1[2][0]);
				const dFloat stopAlpha = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
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

class dKinematicRagdoll: public dAnimationRagdollRoot
{
	public:
	dKinematicRagdoll(NewtonBody* const body, const dMatrix& bindMarix)
		:dAnimationRagdollRoot(body, bindMarix)
		,m_hipEffector(NULL)
		,m_leftFootEffector__(NULL)
		,m_rightFootEffector__(NULL)
	{
	}

	~dKinematicRagdoll()
	{
	}

	void SetHipEffector(dAnimationJoint* const hip)
	{
		m_hipEffector = new dKinematicEndEffector(hip);
		m_loopJoints.Append(m_hipEffector);
	}

	void SetLeftFootEffector(dAnimationJoint* const joint)
	{
		m_leftFootEffector__ = new dKinematicEndEffector(joint);
		m_loopJoints.Append(m_leftFootEffector__);
	}

	void SetRightFootEffector(dAnimationJoint* const joint)
	{
		m_rightFootEffector__ = new dKinematicEndEffector(joint);
		m_loopJoints.Append(m_rightFootEffector__);
	}

	dVector CalculateEffectorOrigin(dAnimationRagDollEffector* const hipEffector) const
	{
		return dVector(0.0f);
	}

	void SelftBalance()
	{
	}

	void PreUpdate(dFloat timestep)
	{
		// do most of the control here, so that the is no need do subclass  
		//dDynamicsRagdoll* const ragDoll = (dDynamicsRagdoll*)model;
		NewtonBody* const rootBody = GetBody();
		NewtonBodySetSleepState(rootBody, 0);

		// get pivot 
		dMatrix rootMatrix;
//		dMatrix pivotMatrix;
		NewtonBodyGetMatrix(GetBody(), &rootMatrix[0][0]);

		dVector com(CalculateCenterOfMass());

		if (m_leftFootEffector__) {
			dMatrix footMatrix (m_leftFootEffector__->GetMatrix());
			m_leftFootEffector__->SetTarget(footMatrix);
		}

		if (m_rightFootEffector__) {
			dMatrix footMatrix(m_rightFootEffector__->GetMatrix());
			m_rightFootEffector__->SetTarget(footMatrix);
		}

/*
		NewtonBodyGetMatrix(ragDoll->m_leftFootEffector->GetBody(), &pivotMatrix[0][0]);
		
		dVector comRadius (com - pivotMatrix.m_posit);
		dVector bodyRadius (rootMatrix.m_posit - pivotMatrix.m_posit);

		dFloat r0 = dSqrt(comRadius.DotProduct3(comRadius));
		dFloat r1 = dSqrt(bodyRadius.DotProduct3(bodyRadius));

		dVector updir (0.0f, 1.0f, 0.0f, 0.0f);
		dVector p1 (pivotMatrix.m_posit + updir.Scale (r0));
		dVector targtePosit (rootMatrix.m_posit + (p1 - com).Scale (r1/r0));
		targtePosit.m_w = 1.0f;
*/		
		dMatrix matrix(dPitchMatrix(0.0f * dDegreeToRad) * dYawMatrix(0.0f * dDegreeToRad) * dRollMatrix(0.0f * dDegreeToRad));
		//com.m_y = rootMatrix.m_posit.m_y;
		//com.m_w = 1.0f;
		//matrix.m_posit = rootMatrix.m_posit;
		//matrix.m_posit = com;

		matrix.m_posit = rootMatrix.m_posit;
		m_hipEffector->SetTarget(matrix);

//dVector error(matrix.m_posit - rootMatrix.m_posit);
//dTrace(("%f %f %f\n", error[0], error[1], error[2]));

	}

	dAnimationRagDollEffector* m_hipEffector;
	dAnimationRagDollEffector* m_leftFootEffector__;
	dAnimationRagDollEffector* m_rightFootEffector__;
};


class dKinematicRagdollManager: public dAnimationModelManager
{
	
	public:
	dKinematicRagdollManager(DemoEntityManager* const scene)
		:dAnimationModelManager(scene->GetNewton())
//		, m_currentRig(NULL)
	{
//		scene->Set2DDisplayRenderFunction(RenderHelpMenu, NULL, this);
	}

	~dKinematicRagdollManager()
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

	dAnimationJoint* CreateChildNode(NewtonBody* const boneBody, dAnimationJoint* const parent, const dKinematicBoneDefinition& definition) const
	{
		dMatrix matrix;
		NewtonBodyGetMatrix(boneBody, &matrix[0][0]);

		dKinematicBoneDefinition::dFrameMatrix frameAngle(definition.m_frameBasics);
		dMatrix pinAndPivotInGlobalSpace(dPitchMatrix(frameAngle.m_pitch * dDegreeToRad) * dYawMatrix(frameAngle.m_yaw * dDegreeToRad) * dRollMatrix(frameAngle.m_roll * dDegreeToRad));
		pinAndPivotInGlobalSpace = pinAndPivotInGlobalSpace * matrix;

		//dMatrix bindMatrix(entity->GetParent()->CalculateGlobalMatrix((DemoEntity*)NewtonBodyGetUserData(parentBody)).Inverse());
		dMatrix bindMatrix(dGetIdentityMatrix());
		dAnimationJoint* const joint = new dAnimationRagdollJoint(definition.m_type, pinAndPivotInGlobalSpace, boneBody, bindMatrix, parent);
		return joint;
	}

	void CreateKinematicModel(const char* const modelName, const dMatrix& location, dKinematicBoneDefinition* const defintion, int size)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		DemoEntity* const modelEntity = DemoEntity::LoadNGD_mesh(modelName, world, scene->GetShaderCache());
		scene->Append(modelEntity);

		// add the root childBody
		NewtonBody* const rootBody = CreateBodyPart(modelEntity);

		// build the rag doll with rigid bodies connected by joints
		dKinematicRagdoll* const dynamicRagdoll = new dKinematicRagdoll(rootBody, dGetIdentityMatrix());
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
			const char* const name = entity->GetName().GetStr();
dTrace(("%s\n", name));
			dAnimationJoint* parentNode = parentBones[stackIndex];

			for (int i = 0; i < size; i++) {
				if (!strcmp(defintion[i].m_boneName, name)) {
					NewtonBody* const childBody = CreateBodyPart(entity);

					// connect this body part to its parent with a rag doll joint
					parentNode = CreateChildNode(childBody, parentNode, defintion[i]);

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
				NewtonBody* const body = joint->GetBody();
				DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(body);
				if (entity->GetName().Find("left") != -1) {
					dynamicRagdoll->SetLeftFootEffector(joint);
				} else if (entity->GetName().Find("right") != -1) {
					dynamicRagdoll->SetRightFootEffector(joint);
				}
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

#else


class dKinematicBoneDefinition
{
	public:
	enum jointType
	{
		m_none,
		m_1dof,
		m_2dof,
		m_3dof,
		m_effector,
	};

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

	char* m_boneName;
	char* m_parentNoneName;
	dFloat m_massFraction;
	jointType m_type;
	dJointLimit m_jointLimits;
	dFrameMatrix m_frameBasics;
};

static dKinematicBoneDefinition tredDefinition[] =
{
	{ "bone_pelvis", NULL, 1.0f, dKinematicBoneDefinition::m_none },

	{ "bone_rightLeg", NULL, 0.3f, dKinematicBoneDefinition::m_3dof, {60.0f, 60.0f, 70.0f}, { 0.0f, 90.0f, 0.0f }},
	{ "bone_righCalf", NULL, 0.3f, dKinematicBoneDefinition::m_1dof, {-80.0f, 30.0f, 0.0f}, { 0.0f, 0.0f, 90.0f }},
	//{ "bone_rightAnkle", 100.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 90.0f }, dAnimationRagdollJoint::m_zeroDof },
	//{ "bone_rightFoot", 100.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, -90.0f, 0.0f }, dAnimationRagdollJoint::m_twoDof },
	
	//{ "bone_leftLeg", NULL, 0.3f, dKinematicBoneDefinition::m_3dof, { 60.0f, 60.0f, 70.0f }, { 0.0f, 90.0f, 0.0f } },
	//{ "bone_leftCalf", NULL, 0.3f, dKinematicBoneDefinition::m_1dof, { -80.0f, 30.0f, 0.0f }, { 0.0f, 0.0f, 90.0f } },
	//{ "bone_leftAnkle", 100.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 90.0f }, dAnimationRagdollJoint::m_zeroDof },
	//{ "bone_leftFoot", 100.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, -90.0f, 0.0f }, dAnimationRagdollJoint::m_twoDof },
	{ NULL},
};

class dModelDescritor
{
	public:
	char* m_filename;
	dKinematicBoneDefinition* m_skeletonDefinition;
};

static dModelDescritor tred = {"tred.ngd", tredDefinition};


class dKinematicRagdoll: public dModelRootNode
{
	public:
	dKinematicRagdoll(NewtonBody* const rootBody)
		:dModelRootNode(rootBody, dGetIdentityMatrix())
	{
	}
};


class dKinematicRagdollManager: public dModelManager
{
	public:
	dKinematicRagdollManager(DemoEntityManager* const scene)
		:dModelManager(scene->GetNewton())
		//,m_currentController(NULL)
		//,m_azimuth(0.0f)
		//,m_posit_x(0.0f)
		//,m_posit_y(0.0f)
		//,m_gripper_pitch(0.0f)
		//,m_gripper_yaw(0.0f)
		//,m_gripper_roll(0.0f)
	{
//		scene->Set2DDisplayRenderFunction(RenderHelpMenu, NULL, this);
	}

	~dKinematicRagdollManager()
	{
	}


	static void RenderHelpMenu(DemoEntityManager* const scene, void* const context)
	{
		dAssert (0);
		//dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		//dKinematicRagdollManager* const me = (dKinematicRagdollManager*)context;
		//scene->Print(color, "linear degrees of freedom");
		//ImGui::SliderFloat("Azimuth", &me->m_azimuth, -180.0f, 180.0f);
		//ImGui::SliderFloat("posit_x", &me->m_posit_x, -1.4f, 0.2f);
		//ImGui::SliderFloat("posit_y", &me->m_posit_y, -1.2f, 0.4f);
		//
		//ImGui::Separator();
		//scene->Print(color, "angular degrees of freedom");
		//ImGui::SliderFloat("pitch", &me->m_gripper_pitch, -180.0f, 180.0f);
		//ImGui::SliderFloat("yaw", &me->m_gripper_yaw, -80.0f, 80.0f);
		//ImGui::SliderFloat("roll", &me->m_gripper_roll, -180.0f, 180.0f);
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

//		PhysicsApplyGravityForce(body, timestep, threadIndex);
	}

	NewtonBody* CreateBodyPart(DemoEntity* const bodyPart, const dKinematicBoneDefinition& definition)
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
		NewtonBodySetMassProperties(bone, definition.m_massFraction, shape);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(bone, bodyPart);

		// assign the material for early collision culling
		//NewtonBodySetMaterialGroupID(bone, m_material);
		NewtonBodySetMaterialGroupID(bone, 0);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		//NewtonBodySetForceAndTorqueCallback (bone, PhysicsApplyGravityForce);
		NewtonBodySetForceAndTorqueCallback(bone, ClampAngularVelocity);

		// destroy the collision helper shape 
		NewtonDestroyCollision(shape);
		return bone;
	}

/*
	dCustomKinematicController* ConnectWithEffectoJoint(NewtonBody* const effectorReferenceBody, DemoEntity* const effectorNode, NewtonBody* const body, const dSixAxisJointDefinition& definition)
	{
		dMatrix matrix(effectorNode->CalculateGlobalMatrix());
#ifdef USE_OLD_KINEMATICS
		dCustomKinematicController* const effector = new dCustomKinematicController (body, matrix);
#else
		dCustomKinematicController* const effector = new dCustomKinematicController(body, matrix, effectorReferenceBody);
#endif
		effector->SetSolverModel(1);
		effector->SetMaxLinearFriction(SIZE_ROBOT_MASS * DEMO_GRAVITY * 50.0f);
		effector->SetMaxAngularFriction(SIZE_ROBOT_MASS * 50.0f);
		return effector;
	}
*/

	void ConnectLimb(NewtonBody* const bone, NewtonBody* const parent, const dKinematicBoneDefinition& definition)
	{
		dMatrix matrix;
		NewtonBodyGetMatrix(bone, &matrix[0][0]);

		dKinematicBoneDefinition::dFrameMatrix frameAngle(definition.m_frameBasics);
		dMatrix pinAndPivotInGlobalSpace(dPitchMatrix(frameAngle.m_pitch * dDegreeToRad) * dYawMatrix(frameAngle.m_yaw * dDegreeToRad) * dRollMatrix(frameAngle.m_roll * dDegreeToRad));
		pinAndPivotInGlobalSpace = pinAndPivotInGlobalSpace * matrix;

		//dMatrix pinAndPivotInGlobalSpace(dRollMatrix(90.0f * dDegreeToRad) * matrix);

		switch (definition.m_type)
		{
			case dKinematicBoneDefinition::m_1dof:
			{
				dCustomHinge* const joint = new dCustomHinge(pinAndPivotInGlobalSpace, bone, parent);
				joint->EnableLimits(true);
				joint->SetLimits(definition.m_jointLimits.m_minTwistAngle * dDegreeToRad, definition.m_jointLimits.m_maxTwistAngle * dDegreeToRad);
				break;
			}

			case dKinematicBoneDefinition::m_3dof:
			{
				dCustomBallAndSocket* const joint = new dCustomBallAndSocket(pinAndPivotInGlobalSpace, bone, parent);
				joint->EnableTwist(true);
				joint->SetTwistLimits(definition.m_jointLimits.m_minTwistAngle * dDegreeToRad, definition.m_jointLimits.m_maxTwistAngle * dDegreeToRad);

				joint->EnableCone(true);
				joint->SetConeLimits(definition.m_jointLimits.m_coneAngle * dDegreeToRad);
				break;
			}

			default:
				dAssert (0);
		}
	}

	void NormalizeMassAndInertia(dModelRootNode* const model, dFloat modelMass) const
	{
		dAssert (0);
/*
		int stack = 1;
		int bodyCount = 0;
		NewtonBody* bodyArray[1024];
		dModelNode* stackBuffer[32];

		stackBuffer[0] = model;
		while (stack) {
			stack--;
			dModelNode* const root = stackBuffer[stack];
			bodyArray[bodyCount] = root->GetBody();
			bodyCount++;
			const dModelChildrenList& children = root->GetChildren();
			for (dModelChildrenList::dListNode* node = children.GetFirst(); node; node = node->GetNext()) {
				stackBuffer[stack] = node->GetInfo().GetData();
				stack++;
			}
		}

		dFloat totalMass = 0.0f;
		for (int i = 0; i < bodyCount; i++) {
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;
			dFloat mass;

			NewtonBody* const body = bodyArray[i];
			NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
			totalMass += mass;
		}

		dFloat massNormalize = modelMass / totalMass;
		for (int i = 0; i < bodyCount; i++) {
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;
			dFloat mass;

			NewtonBody* const body = bodyArray[i];
			NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);

			mass *= massNormalize;
			Ixx *= massNormalize;
			Iyy *= massNormalize;
			Izz *= massNormalize;

			dFloat minInertia = dMin(Ixx, dMin(Iyy, Izz));
			if (minInertia < 10.0f) {
				dFloat maxInertia = dMax(dFloat(10.0f), dMax(Ixx, dMax(Iyy, Izz)));
				Ixx = maxInertia;
				Iyy = maxInertia;
				Izz = maxInertia;
			}

			NewtonBodySetMassMatrix(body, mass, Ixx, Iyy, Izz);
		}
*/
	}


	void CreateKinematicModel(dModelDescritor& descriptor, const dMatrix& location) 
	{
		//dKinematicRagdoll* const dynamicRagdoll = new dKinematicRagdoll(rootBody, dGetIdentityMatrix());
		//AddModel(dynamicRagdoll);
		//dynamicRagdoll->SetCalculateLocalTransforms(true);
		//
		//// save the controller as the collision user data, for collision culling
		//NewtonCollisionSetUserData(NewtonBodyGetCollision(rootBody), dynamicRagdoll);
		//
		//// walk model hierarchic adding all children designed as rigid body bones. 
		//while (stackIndex) {
		//	stackIndex--;
		//	DemoEntity* const entity = childEntities[stackIndex];
		//	const char* const name = entity->GetName().GetStr();
		//	dTrace(("%s\n", name));
		//	dAnimationJoint* parentNode = parentBones[stackIndex];
		//
		//	for (int i = 0; i < size; i++) {
		//		if (!strcmp(defintion[i].m_boneName, name)) {
		//			NewtonBody* const childBody = CreateBodyPart(entity);
		//
		//			// connect this body part to its parent with a rag doll joint
		//			parentNode = CreateChildNode(childBody, parentNode, defintion[i]);
		//
		//			NewtonCollisionSetUserData(NewtonBodyGetCollision(childBody), parentNode);
		//			break;
		//		}
		//	}
		//
		//	for (DemoEntity* child = entity->GetChild(); child; child = child->GetSibling()) {
		//		parentBones[stackIndex] = parentNode;
		//		childEntities[stackIndex] = child;
		//		stackIndex++;
		//	}
		//}
		//
		//SetModelMass(100.0f, dynamicRagdoll);
		//
		//// transform the entire contraction to its location
		//dMatrix worldMatrix(modelEntity->GetCurrentMatrix() * location);
		//worldMatrix.m_posit = location.m_posit;
		//NewtonBodySetMatrixRecursive(rootBody, &worldMatrix[0][0]);
		//
		//
		//// attach effectors here
		//for (dAnimationJoint* joint = GetFirstJoint(dynamicRagdoll); joint; joint = GetNextJoint(joint)) {
		//	if (joint->GetAsRoot()) {
		//		dAssert(dynamicRagdoll == joint);
		//		dynamicRagdoll->SetHipEffector(joint);
		//	}
		//	else if (joint->GetAsLeaf()) {
		//		NewtonBody* const body = joint->GetBody();
		//		DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(body);
		//		if (entity->GetName().Find("left") != -1) {
		//			dynamicRagdoll->SetLeftFootEffector(joint);
		//		}
		//		else if (entity->GetName().Find("right") != -1) {
		//			dynamicRagdoll->SetRightFootEffector(joint);
		//		}
		//	}
		//}
		//
		//dynamicRagdoll->Finalize();

		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		DemoEntity* const entityModel = DemoEntity::LoadNGD_mesh(descriptor.m_filename, world, scene->GetShaderCache());
		scene->Append(entityModel);
		dMatrix matrix0(entityModel->GetCurrentMatrix());
		entityModel->ResetMatrix(*scene, matrix0 * location);

		// create the root body, do not set the transform call back 
		NewtonBody* const rootBody = CreateBodyPart(entityModel, descriptor.m_skeletonDefinition[0]);

		// make a kinematic controlled model.
		dKinematicRagdoll* const model = new dKinematicRagdoll(rootBody);

		// add the model to the manager
		AddRoot(model);

		// the the model to calculate the local transformation
		model->SetTranformMode(true);

		// save the controller as the collision user data, for collision culling
		//NewtonCollisionSetUserData(NewtonBodyGetCollision(rootBone), controller);

		int stackIndex = 0;
		DemoEntity* childEntities[32];
		dModelNode* parentBones[32];
		for (DemoEntity* child = entityModel->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = model;
			childEntities[stackIndex] = child;
			stackIndex++;
		}

		// walk model hierarchic adding all children designed as rigid body bones. 
		while (stackIndex) {
			stackIndex--;

			DemoEntity* const entity = childEntities[stackIndex];
			dModelNode* parentBone = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			dTrace(("name: %s\n", name));
			for (int i = 1; descriptor.m_skeletonDefinition[i].m_boneName; i++) {
				if (!strcmp(descriptor.m_skeletonDefinition[i].m_boneName, name)) {
					NewtonBody* const parentBody = parentBone->GetBody();
					if (descriptor.m_skeletonDefinition[i].m_type == dKinematicBoneDefinition::m_effector) {
						dAssert (0);
						//robot->m_effector = ConnectEffector(rootBody, entity, parentBody, definition[i]);
					} else {
						NewtonBody* const childBody = CreateBodyPart(entity, descriptor.m_skeletonDefinition[i]);
						ConnectLimb(childBody, parentBody, descriptor.m_skeletonDefinition[i]);

						dMatrix bindMatrix(entity->GetParent()->CalculateGlobalMatrix((DemoEntity*)NewtonBodyGetUserData(parentBody)).Inverse());
						dModelNode* const bone = new dModelNode(childBody, bindMatrix, parentBone);

						for (DemoEntity* child = entity->GetChild(); child; child = child->GetSibling()) {
							parentBones[stackIndex] = bone;
							childEntities[stackIndex] = child;
							stackIndex++;
						}
					}
					break;
				}
			}
		}

		// set mass distribution by density and volume
		//NormalizeMassAndInertia(SIZE_ROBOT_MASS, bodyCount, bodyArray);

		NewtonBodySetMassMatrix(rootBody, 0.0f, 0.0f, 0.0f, 0.0f);

		//m_currentController = robot;
		//robot->SetPivotMatrix();
	}

	virtual void OnDebug(dModelRootNode* const model, dCustomJoint::dDebugDisplay* const debugContext) 
	{
		//dKinematicRagdoll* const ragdoll = (dKinematicRagdoll*)model;
		//if (robot->m_effector) {
		//	robot->m_effector->Debug(debugContext);
		//}
	}

	virtual void OnUpdateTransform(const dModelNode* const bone, const dMatrix& localMatrix) const
	{
		NewtonBody* const body = bone->GetBody();
		DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(body);
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));
		
		dQuaternion rot(localMatrix);
		ent->SetMatrix(*scene, rot, localMatrix.m_posit);
	}

	virtual void OnPreUpdate(dModelRootNode* const model, dFloat timestep) const 
	{
		//if (model == m_currentController) {
		//	m_currentController->UpdateEffectors (m_azimuth, m_posit_x, m_posit_y, 
		//										  m_gripper_pitch, m_gripper_yaw, m_gripper_roll);
		//}
	}
	
	//dSixAxisRobot* m_currentController;
	//dFloat32 m_azimuth;
	//dFloat32 m_posit_x;
	//dFloat32 m_posit_y;
	//dFloat32 m_gripper_pitch;
	//dFloat32 m_gripper_yaw;
	//dFloat32 m_gripper_roll;
};


#endif


void KinematicRagdoll(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	CreateLevelMesh(scene, "flatPlane.ngd", true);

	dKinematicRagdollManager* const manager = new dKinematicRagdollManager(scene);
	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID(world);
	NewtonMaterialSetDefaultFriction(world, defaultMaterialID, defaultMaterialID, 1.0f, 1.0f);
	NewtonMaterialSetDefaultElasticity(world, defaultMaterialID, defaultMaterialID, 0.0f);

	dMatrix origin (dYawMatrix(90.0f * dDegreeToRad));
	
#ifdef OLD_KINEMATIC_RAGDOLL
	origin.m_posit.m_y = 2.0f;
	const int definitionCount = sizeof(tredDefinition) / sizeof(tredDefinition[0]);
	manager->CreateKinematicModel("tred.ngd", origin, tredDefinition, definitionCount);
#else
	manager->CreateKinematicModel(tred, origin);
#endif

	origin.m_posit.m_x = -8.0f;
	origin.m_posit.m_y = 1.0f;
	origin.m_posit.m_z = 0.0f;
	scene->SetCameraMatrix(dGetIdentityMatrix(), origin.m_posit);
}




