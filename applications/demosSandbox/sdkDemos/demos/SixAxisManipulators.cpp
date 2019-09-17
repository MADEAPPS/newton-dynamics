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
};

static dSixAxisJointDefinition robot1[] =
{
	{ "bone_base001", NULL, { -1000.0f, 1000.0f } },
	{ "bone_base002", NULL, { -120.0f, 45.0f } },
	{ "bone_base003", NULL, { -120.0f, 15.0f } },
	{ "bone_base004", NULL, { -1000.0f, 1000.0f } },
	{ "bone_base005", NULL, { -225.0f, 45.0f } },
	{ "bone_base006", NULL, { -1000.0f, 1000.0f } },
	{ "effector", "bone_base001", { -1000.0f, 1000.0f } },
	{ NULL, NULL},
};

class dSixAxisRobot: public dModelRootNode
{
	public:
	dSixAxisRobot(NewtonBody* const rootBody, const dMatrix& bindMatrix)
		:dModelRootNode(rootBody, bindMatrix)
		,m_effector(NULL)
	{
	}

	dCustomKinematicController* m_effector;
};

class dSixAxisManager: public dModelManager
{
	public:
	dSixAxisManager(DemoEntityManager* const scene)
		:dModelManager(scene->GetNewton())
//		,m_currentController(NULL)
//		,m_azimuth(0.0f)
//		,m_posit_x(0.0f)
//		,m_posit_y(0.0f)
//		,m_gripper_roll(0.0f)
//		,m_gripper_pitch(0.0f)
	{
//			scene->Set2DDisplayRenderFunction(RenderHelpMenu, NULL, this);
	}

	~dSixAxisManager()
	{
	}

/*
	static void RenderHelpMenu(DemoEntityManager* const scene, void* const context)
	{
		dSixAxisManager* const me = (dSixAxisManager*)context;

		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "Use sliders to manipulate robot");
		ImGui::SliderFloat("Azimuth", &me->m_azimuth, -150.0f, 150.0f);
		ImGui::SliderFloat("posit_x", &me->m_posit_x, -1.0f, 1.0f);
		ImGui::SliderFloat("posit_y", &me->m_posit_y, -1.0f, 1.0f);

		//		ImGui::Separator();
		//		ImGui::Separator();
		//		ImGui::SliderFloat("eff_roll", &me->m_gripper_roll, -360.0f, 360.0f);
		//		ImGui::SliderFloat("eff_pitch", &me->m_gripper_pitch, -60.0f, 60.0f);

		for (dListNode* node = me->GetFirst(); node; node = node->GetNext()) {
			dSixAxisController* const controller = &node->GetInfo();
			controller->SetTarget(me->m_posit_x, me->m_posit_y, me->m_azimuth * dDegreeToRad, me->m_gripper_pitch * dDegreeToRad, me->m_gripper_roll * dDegreeToRad);
		}
	}

	virtual dSixAxisController* CreateController()
	{
		return (dSixAxisController*)dCustomControllerManager<dSixAxisController>::CreateController();
	}

	dSixAxisController* MakeSixAxisRobot(DemoEntityManager* const scene, const dMatrix& origin)
	{
		DemoEntity* const model = DemoEntity::LoadNGD_mesh("robotArm.ngd", scene->GetNewton(), scene->GetShaderCache());

		scene->Append(model);
		model->ResetMatrix(*scene, origin);

		DemoEntity* const model1 = DemoEntity::LoadNGD_mesh("robot2.ngd", scene->GetNewton(), scene->GetShaderCache());
		scene->Append(model1);
		model1->ResetMatrix(*scene, origin);

		dSixAxisController* const controller = (dSixAxisController*)CreateController();
		controller->MakeSixAxisRobot(scene, model);
		m_currentController = controller;
		return controller;
	}

	void OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
	{
		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			dSixAxisController* const controller = &node->GetInfo();
			controller->Debug(debugContext);
		}
	}
*/

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
		//effector->SetSolverModel(2);
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

	void SetModelMass(dFloat mass, int bodyCount, NewtonBody** const bodyArray) const
	{

		dFloat maxVolume = 0.0f;
		for (int i = 0; i < bodyCount; i++) {
			maxVolume = dMax(NewtonConvexCollisionCalculateVolume(NewtonBodyGetCollision(bodyArray[i])), maxVolume);
		}

		maxVolume /= 20.0f;
		dFloat volume = 0.0f;
		for (int i = 0; i < bodyCount; i++) {
			dFloat vol = NewtonConvexCollisionCalculateVolume(NewtonBodyGetCollision(bodyArray[i]));
			if (vol < maxVolume) {
				vol = maxVolume;
			}
			volume += vol;
		}

		dFloat density = mass / volume;

		for (int i = 0; i < bodyCount; i++) {
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;

			NewtonBody* const body = bodyArray[i];
			NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
			dFloat vol = NewtonConvexCollisionCalculateVolume(NewtonBodyGetCollision(bodyArray[i]));
			bool isSmall = false;
			if (vol < maxVolume) {
				isSmall = true;
				vol = maxVolume;
			}
			dFloat scale = density * vol;
			mass *= scale;
			Ixx *= scale;
			Iyy *= scale;
			Izz *= scale;
			if (isSmall) {
				dFloat maxInertia = 4.0f* dMax(Ixx, dMax(Iyy, Izz));
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
		NewtonBody* const rootBody = CreateBodyPart(model);

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
						NewtonBody* const childBody = CreateBodyPart(entity);
						bodyArray[bodyCount] = childBody;
						bodyCount++;

						// connect this body part to its parent with a rag doll joint
						NewtonBody* const parentBody = parentBone->GetBody();
						if (
							strstr(name, "base002") ||
							strstr(name, "base003") ||
							strstr(name, "base004") ||
							strstr(name, "base005") ||
							strstr(name, "base006")){
							ConnectWithHingeJoint(childBody, parentBody, definition[i]);
						}
					
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
		//dVector veloc(0.0f, 4.0f, 0.0f, 0.0f);
		//NewtonBodySetVelocity(rootBody, &veloc[0]);
		//NewtonBodySetOmega(rootBody, &veloc[0]);
#endif

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


	
/*
	dSixAxisController* m_currentController;
	dFloat32 m_azimuth;
	dFloat32 m_posit_x;
	dFloat32 m_posit_y;
	dFloat32 m_gripper_roll;
	dFloat32 m_gripper_pitch;
*/
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

