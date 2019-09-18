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
#include "DebugDisplay.h"
#include "PhysicsUtils.h"
#include "TargaToOpenGl.h"
#include "DemoEntityManager.h"

#define SIZE_ROBOT_MASS 500.0f

class dSixAxisJointDefinition
{
	public:
	struct dJointLimit
	{
		dFloat m_minTwistAngle;
		dFloat m_maxTwistAngle;
	};

	char* m_boneName;
	char* m_parentNoneName;
	dJointLimit m_jointLimits;
	dFloat m_massFraction;
};

static dSixAxisJointDefinition robot1[] =
{
	{ "bone_base000", NULL,{ -1000.0f, 1000.0f }, 1.0f },
	{ "bone_base001", NULL, { -1000.0f, 1000.0f }, 0.4f },
	{ "bone_base002", NULL, { -120.0f, 45.0f }, 0.3f },
	{ "bone_base003", NULL, { -120.0f, 15.0f }, 0.2f },
	{ "bone_base004", NULL, { -1000.0f, 1000.0f }, 0.1f },
	{ "bone_base005", NULL, { -225.0f, 45.0f }, 0.06f },
	{ "bone_base006", NULL, { -1000.0f, 1000.0f }, 0.06f },
	{ "effector", "bone_base001", { -1000.0f, 1000.0f }},
	{ NULL, NULL},
};

class dSixAxisRobot: public dModelRootNode
{
	public:
	dSixAxisRobot(NewtonBody* const rootBody, const dMatrix& bindMatrix)
		:dModelRootNode(rootBody, bindMatrix)
		,m_pivotMatrix(dGetIdentityMatrix())
		,m_gripperMatrix(dGetIdentityMatrix())
		,m_effector(NULL)
		,m_azimuth(0.0f)
		,m_posit_x(0.0f)
		,m_posit_y(0.0f)
		,m_pitch(0.0f)
		,m_yaw(0.0f)
		,m_roll(0.0f)
	{
	}

	void SetPivotMatrix()
	{
		dMatrix baseMatrix;
		dMatrix pivotMatrix;
		dModelNode* const referenceNode = GetChildren().GetFirst()->GetInfo().GetData();
		NewtonBodyGetMatrix(GetBody(), &baseMatrix[0][0]);
		NewtonBodyGetMatrix(referenceNode->GetBody(), &pivotMatrix[0][0]);
		m_pivotMatrix = pivotMatrix * baseMatrix.Inverse();
		m_gripperMatrix = m_effector->GetTargetMatrix() * m_pivotMatrix.Inverse();
	}

	void UpdateEffectors (dFloat azimuth, dFloat posit_x, dFloat posit_y, dFloat pitch, dFloat yaw, dFloat roll)
	{
		m_azimuth = azimuth;
		m_posit_x = posit_x;
		m_posit_y = posit_y;
		m_pitch = pitch;
		m_yaw = yaw;
		m_roll = roll;

		dMatrix yawMatrix (dYawMatrix(m_azimuth * dDegreeToRad));
		dMatrix gripperMatrix (dPitchMatrix(m_pitch * dDegreeToRad) * dYawMatrix(m_yaw * dDegreeToRad) * dRollMatrix(m_roll * dDegreeToRad) * m_gripperMatrix);

		gripperMatrix.m_posit.m_x += m_posit_x;
		gripperMatrix.m_posit.m_y += m_posit_y;
		m_effector->SetTargetMatrix (gripperMatrix * yawMatrix * m_pivotMatrix);
	}
	
	dMatrix m_pivotMatrix;
	dMatrix m_gripperMatrix;
	dCustomKinematicController* m_effector;
	dFloat m_azimuth;
	dFloat m_posit_x;
	dFloat m_posit_y;
	dFloat m_pitch;
	dFloat m_yaw;
	dFloat m_roll;
};

class dSixAxisManager: public dModelManager
{
	public:
	dSixAxisManager(DemoEntityManager* const scene)
		:dModelManager(scene->GetNewton())
		,m_currentController(NULL)
		,m_azimuth(0.0f)
		,m_posit_x(0.0f)
		,m_posit_y(0.0f)
		,m_gripper_pitch(0.0f)
		,m_gripper_yaw(0.0f)
		,m_gripper_roll(0.0f)
	{
		scene->Set2DDisplayRenderFunction(RenderHelpMenu, NULL, this);
	}

	~dSixAxisManager()
	{
	}

	static void RenderHelpMenu(DemoEntityManager* const scene, void* const context)
	{
		dSixAxisManager* const me = (dSixAxisManager*)context;

		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "linear degrees of freedom");
		ImGui::SliderFloat("Azimuth", &me->m_azimuth, -180.0f, 180.0f);
		ImGui::SliderFloat("posit_x", &me->m_posit_x, -1.4f, 0.2f);
		ImGui::SliderFloat("posit_y", &me->m_posit_y, -1.2f, 0.4f);

		ImGui::Separator();
		scene->Print(color, "angular degrees of freedom");
		ImGui::SliderFloat("pitch", &me->m_gripper_pitch, -180.0f, 180.0f);
		ImGui::SliderFloat("yaw", &me->m_gripper_yaw, -80.0f, 80.0f);
		ImGui::SliderFloat("roll", &me->m_gripper_roll, -180.0f, 180.0f);
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

		PhysicsApplyGravityForce(body, timestep, threadIndex);
	}

	NewtonBody* CreateBodyPart(DemoEntity* const bodyPart, const dSixAxisJointDefinition& definition)
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

	dCustomKinematicController* ConnectWithEffectoJoint(NewtonBody* const effectorReferenceBody, DemoEntity* const effectorNode, NewtonBody* const parent, const dSixAxisJointDefinition& definition)
	{
		dMatrix matrix(effectorNode->CalculateGlobalMatrix());
#ifdef USE_OLD_KINEMATICS
		dCustomKinematicController* const effector = new dCustomKinematicController (parent, matrix);
#else
		dCustomKinematicController* const effector = new dCustomKinematicController(parent, matrix, effectorReferenceBody);
#endif
		effector->SetSolverModel(1);
		effector->SetMaxLinearFriction(SIZE_ROBOT_MASS * DEMO_GRAVITY * 50.0f);
		effector->SetMaxAngularFriction(SIZE_ROBOT_MASS * 50.0f);
		return effector;
	}

	void ConnectWithHingeJoint(NewtonBody* const bone, NewtonBody* const parent, const dSixAxisJointDefinition& definition)
	{
		dMatrix matrix;
		NewtonBodyGetMatrix(bone, &matrix[0][0]);

		dMatrix pinAndPivotInGlobalSpace(dRollMatrix(90.0f * dDegreeToRad) * matrix);

		dCustomHinge* const joint = new dCustomHinge(pinAndPivotInGlobalSpace, bone, parent);
		if (definition.m_jointLimits.m_maxTwistAngle < 360.0f) {
			joint->EnableLimits(true);
			joint->SetLimits(definition.m_jointLimits.m_minTwistAngle * dDegreeToRad, definition.m_jointLimits.m_maxTwistAngle * dDegreeToRad);
		}
	}

	void SetModelMass(dFloat modelMass, int bodyCount, NewtonBody** const bodyArray) const
	{
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
	}

	void MakeSixAxisRobot(DemoEntityManager* const scene, const dMatrix& origin, dSixAxisJointDefinition* const definition)
	{
		DemoEntity* const model = DemoEntity::LoadNGD_mesh("robot1.ngd", scene->GetNewton(), scene->GetShaderCache());
		scene->Append(model);
		model->ResetMatrix(*scene, origin);

		// create the root body, do not set the transform call back 
		NewtonBody* const rootBody = CreateBodyPart(model, definition[0]);

		// make a kinematic controlled model.
		dSixAxisRobot* const robot = new dSixAxisRobot(rootBody, dGetIdentityMatrix());

		// add the model to the manager
		AddRoot(robot);

		// the the model to calculate the local transformation
		robot->SetTranformMode(true);

		// save the controller as the collision user data, for collision culling
		//NewtonCollisionSetUserData(NewtonBodyGetCollision(rootBone), controller);

		int stackIndex = 0;
		DemoEntity* childEntities[32];
		dModelNode* parentBones[32];
		for (DemoEntity* child = model->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = robot;
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
			dModelNode* parentBone = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			dTrace(("name: %s\n", name));
			for (int i = 0; definition[i].m_boneName; i++) {
				if (!strcmp(definition[i].m_boneName, name)) {
					if (strstr (name, "bone")) {
						NewtonBody* const childBody = CreateBodyPart(entity, definition[i]);
						bodyArray[bodyCount] = childBody;
						bodyCount++;

						// connect this body part to its parent with a rag doll joint
						NewtonBody* const parentBody = parentBone->GetBody();
						ConnectWithHingeJoint(childBody, parentBody, definition[i]);
					
						dMatrix bindMatrix(entity->GetParent()->CalculateGlobalMatrix((DemoEntity*)NewtonBodyGetUserData(parentBody)).Inverse());
						parentBone = new dModelNode(childBody, bindMatrix, parentBone);
						//// save the controller as the collision user data, for collision culling
						//NewtonCollisionSetUserData(NewtonBodyGetCollision(childBody), parentBone);

						for (DemoEntity* child = entity->GetChild(); child; child = child->GetSibling()) {
							parentBones[stackIndex] = parentBone;
							childEntities[stackIndex] = child;
							stackIndex++;
						}
					} else if (strstr (name, "effector")) {	
						NewtonBody* const parentBody = parentBone->GetBody();
						dCustomKinematicController* effector = ConnectWithEffectoJoint(rootBody, entity, parentBody, definition[i]);
						robot->m_effector = effector;
					}
					break;
				}
			}
		}

		// set mass distribution by density and volume
		SetModelMass(SIZE_ROBOT_MASS, bodyCount, bodyArray);

#ifdef USE_OLD_KINEMATICS
		// make root body static
		NewtonBodySetMassMatrix(rootBody, 0.0f, 0.0f, 0.0f, 0.0f);
#else
		NewtonBodySetMassMatrix(rootBody, 0.0f, 0.0f, 0.0f, 0.0f);
#endif

		m_currentController = robot;
		robot->SetPivotMatrix();


/*
		// set the collision mask
		// note this container work best with a material call back for setting bit field 
		//dAssert(0);
		//controller->SetDefaultSelfCollisionMask();

		// transform the entire contraction to its location
		dMatrix worldMatrix(rootEntity->GetCurrentMatrix() * location);
		NewtonBodySetMatrixRecursive(rootBone, &worldMatrix[0][0]);
*/
	}

	virtual void OnDebug(dModelRootNode* const model, dCustomJoint::dDebugDisplay* const debugContext) 
	{
		dSixAxisRobot* const robot = (dSixAxisRobot*)model;
		if (robot->m_effector) {
			robot->m_effector->Debug(debugContext);
		}
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
		if (model == m_currentController) {
			m_currentController->UpdateEffectors (m_azimuth, m_posit_x, m_posit_y, 
												  m_gripper_pitch, m_gripper_yaw, m_gripper_roll);
		}
	}
	
	dSixAxisRobot* m_currentController;

	dFloat32 m_azimuth;
	dFloat32 m_posit_x;
	dFloat32 m_posit_y;
	dFloat32 m_gripper_pitch;
	dFloat32 m_gripper_yaw;
	dFloat32 m_gripper_roll;
};


void SixAxisManipulators(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	CreateLevelMesh (scene, "flatPlane.ngd", true);
	dSixAxisManager* const robotManager = new dSixAxisManager(scene);

	dMatrix origin(dYawMatrix(0.0f * dDegreeToRad));
	dMatrix origin1(dYawMatrix(180.0f * dDegreeToRad));
	origin.m_posit.m_z = -1.0f;
	origin1.m_posit.m_z =  1.0f;

	int count = 10;
count = 1;
origin = dGetIdentityMatrix();
origin.m_posit.m_x = 0.0f;
	for (int i = 0; i < count; i ++) {
		robotManager->MakeSixAxisRobot (scene, origin, robot1);
		origin.m_posit.m_x += 1.0f;
		origin1.m_posit.m_x += 1.0f;
	}

	origin.m_posit = dVector(0.0f, 1.0f, 3.0f, 1.0f);
	dMatrix rotation (dYawMatrix(90.0f * dDegreeToRad));
	scene->SetCameraMatrix(rotation, origin.m_posit);
}

