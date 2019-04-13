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


class dAnimationHipEffector: public dAnimationLoopJoint
	, public dCustomJoint
{
	public:
	dAnimationHipEffector(dAnimationJointRoot* const root)
		:dAnimationLoopJoint(root->GetProxyBody(), root->GetStaticWorld())
		,dCustomJoint(6, root->GetBody(), NULL)
		,m_localMatrix(dGetIdentityMatrix())
		,m_targetMatrix(dGetIdentityMatrix())
		,m_maxLinearSpeed(0.1f * 120.0f)
		,m_maxAngularSpeed(3.0f * dDegreeToRad * 120.0f)
		,m_linearFriction(100000.0f)
		,m_angularFriction(10000.0f)
	{
		dMatrix matrix;
		NewtonBody* const body = root->GetBody();
		NewtonBodyGetMatrix(body, &matrix[0][0]);

		m_localMatrix.m_posit = matrix.m_posit;
		m_localMatrix = m_localMatrix * matrix.Inverse();

		SetSolverModel(3);
	}

	void SetTarget()
	{
		dMatrix matrix(m_localMatrix * m_state0->GetMatrix());
		m_targetMatrix.m_posit = matrix.m_posit;

		static dVector xxx = m_targetMatrix.m_posit + dVector (1.0f, 1.0f, 0.0f, 0.0f);
		m_targetMatrix.m_posit = xxx;
	}

	virtual int GetMaxDof() const
	{
		return 6;
	}

	dFloat ClipParam (dFloat value, dFloat maxValue) const
	{
		if (value > maxValue) {
			value = maxValue;
		} else if (value < -maxValue) {
			value = -maxValue;
		} else {
			value *= 0.3f;
		}
		return value;
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		const dMatrix& matrix1 = m_targetMatrix;
		dMatrix matrix0(m_localMatrix * m_state0->GetMatrix());

		//const dVector& omega0 = m_state0->GetOmega();
		//const dVector& omega1 = m_state1->GetOmega();
		//const dVector& veloc0 = m_state0->GetVelocity();
		//const dVector& veloc1 = m_state1->GetVelocity();
		const dVector relPosit(matrix1.m_posit - matrix0.m_posit);

		dAssert(m_maxLinearSpeed >= 0.0f);
		const dFloat invTimestep = 1.0f/ timestep;
		const dFloat linearStep = m_maxLinearSpeed * timestep;
		for (int i = 0; i < 3; i++) {
			NewtonUserJointAddLinearRow (m_joint, &matrix0.m_posit[0], &matrix0.m_posit[0], &matrix1[i][0]);
			const dFloat stopAccel = NewtonUserJointCalculateRowZeroAccelaration(m_joint);
			dFloat speed = ClipParam(relPosit.DotProduct3(matrix1[i]), linearStep) * invTimestep;
			dFloat relAccel = speed * invTimestep + stopAccel;
			NewtonUserJointSetRowAcceleration(m_joint, relAccel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_linearFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_linearFriction);
		}

		const dVector& coneDir0 = matrix0.m_front;
		const dVector& coneDir1 = matrix1.m_front;
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
			dAssert(0);
		} else {

			const dFloat angularStep = m_maxAngularSpeed * timestep;
			// using small angular aproximation to get the joint angle;
			{
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1[1][0]);
				const dFloat stopAlpha = NewtonUserJointCalculateRowZeroAccelaration(m_joint);
				dFloat omega = ClipParam(matrix1[2].DotProduct3(matrix1[0]), angularStep) * invTimestep;
				dFloat relAlpha = omega * invTimestep + stopAlpha;
				NewtonUserJointSetRowAcceleration(m_joint, relAlpha);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
			}

			{
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1[2][0]);
				const dFloat stopAlpha = NewtonUserJointCalculateRowZeroAccelaration(m_joint);
				dFloat omega = ClipParam(matrix1[1].DotProduct3(matrix1[0]), angularStep) * invTimestep;
				dFloat relAlpha = omega * invTimestep + stopAlpha;
				NewtonUserJointSetRowAcceleration(m_joint, relAlpha);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
			}
		}


	}

	virtual void JacobianDerivative(dComplementaritySolver::dParamInfo* const constraintParams)
	{
		const dMatrix& matrix1 = m_targetMatrix;
		dMatrix matrix0(m_localMatrix * m_state0->GetMatrix());
		
		const dVector& omega0 = m_state0->GetOmega();
		const dVector& omega1 = m_state1->GetOmega();
		const dVector& veloc0 = m_state0->GetVelocity();
		const dVector& veloc1 = m_state1->GetVelocity();
		const dVector relPosit(matrix1.m_posit - matrix0.m_posit);

		int dofCount = 0;
		dAssert(m_maxLinearSpeed >= 0.0f);
		const dFloat invTimestep = constraintParams->m_timestepInv;
		const dFloat linearStep = m_maxLinearSpeed * constraintParams->m_timestep;
		for (int i = 0; i < 3; i++) {
			AddLinearRowJacobian(constraintParams, matrix0.m_posit, matrix1[i]);

			dVector stopVeloc(constraintParams->m_jacobians[i].m_jacobian_J01.m_linear * veloc0 +
							  constraintParams->m_jacobians[i].m_jacobian_J01.m_angular * omega0 +
							  constraintParams->m_jacobians[i].m_jacobian_J10.m_linear * veloc1 +
							  constraintParams->m_jacobians[i].m_jacobian_J10.m_angular * omega1);

			dFloat dist = ClipParam (relPosit.DotProduct3(matrix1[i]), linearStep);
			dFloat currentSpeed = dist * invTimestep - (stopVeloc.m_x + stopVeloc.m_y + stopVeloc.m_z);
			constraintParams->m_jointLowFrictionCoef[i] = -m_linearFriction;
			constraintParams->m_jointHighFrictionCoef[i] = m_linearFriction;
			constraintParams->m_jointAccel[i] = currentSpeed * invTimestep;
			constraintParams->m_normalIndex[i] = 0;

			dofCount ++;
		}

/*
		const dVector& coneDir0 = matrix0.m_front;
		const dVector& coneDir1 = matrix1.m_front;
		dFloat cosAngleCos = coneDir1.DotProduct3(coneDir0);
		if (cosAngleCos < 0.9999f) {
			dMatrix coneRotation(dGetIdentityMatrix());
			dVector lateralDir (coneDir1.CrossProduct(coneDir0));
			dFloat mag2 = lateralDir.DotProduct3(lateralDir);
			if (mag2 > 1.0e-4f) {
				lateralDir = lateralDir.Scale(1.0f / dSqrt(mag2));
				coneRotation = dMatrix(dQuaternion(lateralDir, dAcos(dClamp(cosAngleCos, dFloat(-1.0f), dFloat(1.0f)))), matrix1.m_posit);
			} else {
				lateralDir = matrix0.m_up.Scale(-1.0f);
				coneRotation = dMatrix(dQuaternion(matrix0.m_up, dFloat(180.0f) * dDegreeToRad), matrix1.m_posit);
			}
			//dMatrix xxxx(matrix1 * coneRotation);
			//dMatrix xxxx1(xxxx * matrix0.Inverse());

			dMatrix pitchMatrix(matrix1 * coneRotation * matrix0.Inverse());
			dAssert(dAbs(pitchMatrix[0][0] - dFloat(1.0f)) < dFloat(1.0e-3f));
			dAssert(dAbs(pitchMatrix[0][1]) < dFloat(1.0e-3f));
			dAssert(dAbs(pitchMatrix[0][2]) < dFloat(1.0e-3f));
			//dTrace(("%f %f %f\n", pitchMatrix[0][0], pitchMatrix[0][1], pitchMatrix[0][2]));

		} else {
			// using small angular aproximation to get the joint angle;
			{
				AddAngularRowJacobian (constraintParams, matrix1[1], 0.0f);
				dVector stopOmega(constraintParams->m_jacobians[dofCount].m_jacobian_J01.m_angular * omega0 + constraintParams->m_jacobians[dofCount].m_jacobian_J10.m_angular * omega1);
				dFloat angle = ClipParam (matrix1[2].DotProduct3(matrix1[0]), m_maxAngle);
				dFloat angleSpeed = angle * invTimestep - (stopOmega.m_x + stopOmega.m_y + stopOmega.m_z);
				constraintParams->m_jointLowFrictionCoef[dofCount] = -m_angularFriction;
				constraintParams->m_jointHighFrictionCoef[dofCount] = m_angularFriction;
				constraintParams->m_jointAccel[dofCount] = angleSpeed * invTimestep;
				constraintParams->m_normalIndex[dofCount] = 0;
				dofCount ++;
			}

			{
				AddAngularRowJacobian(constraintParams, matrix1[2], 0.0f);
				dVector stopOmega(constraintParams->m_jacobians[dofCount].m_jacobian_J01.m_angular * omega0 + constraintParams->m_jacobians[dofCount].m_jacobian_J10.m_angular * omega1);
				dFloat angle = ClipParam(matrix1[1].DotProduct3(matrix1[0]), m_maxAngle);
				dFloat angleSpeed = angle * invTimestep - (stopOmega.m_x + stopOmega.m_y + stopOmega.m_z);
				constraintParams->m_jointLowFrictionCoef[dofCount] = -m_angularFriction;
				constraintParams->m_jointHighFrictionCoef[dofCount] = m_angularFriction;
				constraintParams->m_jointAccel[dofCount] = angleSpeed * invTimestep;
				constraintParams->m_normalIndex[dofCount] = 0;
				dofCount++;
			}
		}
*/
		m_dof = dofCount;
		m_count = dofCount;
		constraintParams->m_count = dofCount;
	}

	virtual void UpdateSolverForces(const dComplementaritySolver::dJacobianPair* const jacobians) const
	{
		dAssert(0);
	}

	dMatrix m_localMatrix;
	dMatrix m_targetMatrix;
	dFloat m_maxLinearSpeed;
	dFloat m_maxAngularSpeed;
	dFloat m_linearFriction;
	dFloat m_angularFriction;
};

class dDynamicsRagdoll: public dAnimationJointRoot
{
	public:
	dDynamicsRagdoll(NewtonBody* const body, const dMatrix& bindMarix)
		:dAnimationJointRoot(body, bindMarix)
		,m_hipEffector(NULL)
	{
	}

	~dDynamicsRagdoll()
	{
	}

	void PreUpdate(dFloat timestep)
	{
		dAnimationJointRoot::RigidBodyToStates();
		m_proxyBody.SetForce(dVector (0.0f));
		m_proxyBody.SetTorque(dVector(0.0f));

		NewtonBodySetSleepState(GetBody(), 0);
		m_hipEffector->SetTarget();

		//if (m_animationTree) {
		//	m_animationTree->Update(timestep);
		//}
		//m_solver.Update(timestep);
		//UpdateJointAcceleration();
	}

	dAnimationHipEffector* m_hipEffector;
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

	void SetModelMass(dFloat mass, int bodyCount, NewtonBody** const bodyArray) const
	{
		dFloat volume = 0.0f;
		for (int i = 0; i < bodyCount; i++) {
			volume += NewtonConvexCollisionCalculateVolume(NewtonBodyGetCollision(bodyArray[i]));
		}
		dFloat density = mass / volume;

		for (int i = 0; i < bodyCount; i++) {
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;

			NewtonBody* const body = bodyArray[i];
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
		dAnimationJoint* const joint = new dAnimationJointRagdoll(pinAndPivotInGlobalSpace, boneBody, bindMatrix, parent);
		return joint;
	}

	void DynamicsRagdollExperiment_1(const dMatrix& location)
	{
		static dJointDefinition jointsDefinition[] =
		{
			{ "body" },
			{ "leg", 100.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 180.0f } },
			//{ "foot", 100.0f, { -15.0f, 15.0f, 30.0f }, { 0.0f, 0.0f, 180.0f } },
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

		// attach a Hip effector to the root body
		dynamicRagdoll->m_hipEffector = new dAnimationHipEffector(dynamicRagdoll);
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

		int bodyCount = 1;
		NewtonBody* bodyArray[1024];
		bodyArray[0] = rootBody;

		// walk model hierarchic adding all children designed as rigid body bones. 
		while (stackIndex) {
			stackIndex--;
			DemoEntity* const entity = childEntities[stackIndex];
			dAnimationJoint* parentNode = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			for (int i = 0; i < definitionCount; i++) {
				if (!strcmp(jointsDefinition[i].m_boneName, name)) {
					NewtonBody* const childBody = CreateBodyPart(entity);

					bodyArray[bodyCount] = childBody;
					bodyCount++;

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

		SetModelMass (100.0f, bodyCount, bodyArray);

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
		// position the effector
		//const dAnimationLoopJointList& effectors = model->GetLoops();
		//for (dAnimationLoopJointList::dListNode* node = effectors.GetFirst(); node; node = node->GetNext()) {
		//	dAnimationHipEffector* const effector = (dAnimationHipEffector*)node->GetInfo();
		//	effector->SetTarget();
		//}

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
	origin.m_posit.m_y = 1.2f;
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




