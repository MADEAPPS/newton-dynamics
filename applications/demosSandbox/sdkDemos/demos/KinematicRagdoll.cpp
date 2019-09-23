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


class dKinematicBoneDefinition
{
	public:
	enum jointType
	{
		m_none,
		m_0dof,
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
	jointType m_type;
	dFloat m_massFraction;
	dJointLimit m_jointLimits;
	dFrameMatrix m_frameBasics;
};

static dKinematicBoneDefinition tredDefinition[] =
{
	{ "bone_pelvis", dKinematicBoneDefinition::m_none, 1.0f },

	{ "bone_rightLeg", dKinematicBoneDefinition::m_3dof, 0.3f, {60.0f, 60.0f, 70.0f}, { 0.0f, 90.0f, 0.0f }},
	{ "bone_righCalf", dKinematicBoneDefinition::m_1dof, 0.2f, {-80.0f, 30.0f, 0.0f}, { 0.0f, 0.0f, 90.0f }},
	{ "bone_rightAnkle", dKinematicBoneDefinition::m_0dof, 0.2f, {0.0f, 0.0f, 0.0f}, { 0.0f, 0.0f, 0.0f }},
	{ "bone_rightFoot", dKinematicBoneDefinition::m_3dof, 0.2f, {0.0f, 0.0f, 30.0f}, { 0.0f, 0.0f, 0.0f }},
	{ "effector_rightAnkle", dKinematicBoneDefinition::m_effector},

	//{ "bone_leftLeg", dKinematicBoneDefinition::m_3dof, 0.3f, { 60.0f, 60.0f, 70.0f }, { 0.0f, 90.0f, 0.0f } },
	//{ "bone_leftCalf", dKinematicBoneDefinition::m_1dof, 0.2f, { -80.0f, 30.0f, 0.0f }, { 0.0f, 0.0f, 90.0f } },
	//{ "bone_leftAnkle", dKinematicBoneDefinition::m_0dof, 0.2f, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f } },
	//{ "bone_leftFoot", dKinematicBoneDefinition::m_3dof, 0.2f, { 0.0f, 0.0f, 30.0f }, { 0.0f, 0.0f, 0.0f } },
	//{ "effector_leftAnkle", dKinematicBoneDefinition::m_effector },

	{ NULL},
};

class dModelDescritor
{
	public:
	char* m_filename;
	dFloat m_mass;
	dKinematicBoneDefinition* m_skeletonDefinition;
};

static dModelDescritor tred = {"tred.ngd", 500.0f, tredDefinition};


class dModelAnimTreeSelfBalanceCalculator: public dModelAnimTree
{
	public:
	dModelAnimTreeSelfBalanceCalculator(dModelRootNode* const model, dModelAnimTree* const child, dCustomKinematicController* const footEffector0)
		:dModelAnimTree(model)
		,m_child(child)
		,m_rootEffector0(footEffector0)
	{
	}

	~dModelAnimTreeSelfBalanceCalculator()
	{
		delete m_child;
	}

	virtual void Debug(dCustomJoint::dDebugDisplay* const debugContext) const
	{
		dMatrix matrix;
		NewtonBodyGetMatrix(GetRoot()->GetBody(), &matrix[0][0]);
		matrix.m_posit = CalculateCenterOfMass();
		debugContext->DrawFrame(matrix);
		m_child->Debug(debugContext);
	}

	dVector CalculateCenterOfMass() const
	{
		dMatrix matrix;
		dVector com(0.0f);
		dVector localCom;
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		dFloat mass;
		dFloat totalMass = 0.0f;

		int stack = 1;
		int bodyCount = 0;
		const dModelNode* stackBuffer[32];

		stackBuffer[0] = GetRoot();
		while (stack) {
			stack--;
			const dModelNode* const root = stackBuffer[stack];

			NewtonBody* const body = root->GetBody();
			NewtonBodyGetMatrix(body, &matrix[0][0]);
			NewtonBodyGetCentreOfMass(body, &localCom[0]);
			NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);

			totalMass += mass;
			com += matrix.TransformVector(localCom).Scale(mass);

			bodyCount++;
			const dModelChildrenList& children = root->GetChildren();
			for (const dModelChildrenList::dListNode* node = children.GetFirst(); node; node = node->GetNext()) {
				stackBuffer[stack] = node->GetInfo().GetData();
				stack++;
			}
		}

		com = com.Scale(1.0f / totalMass);
		com.m_w = 1.0f;
		return com;
	}

	void GeneratePose(dFloat timestep, dModelKeyFramePose& output)
	{
		m_child->GeneratePose(timestep, output);

		dMatrix pivotMatrix (m_rootEffector0->GetBodyMatrix());
		dVector com (CalculateCenterOfMass());

		dVector error (pivotMatrix.m_posit - com);
		//error.m_x = 0.0f;
		error.m_y = 0.0f;
		//error.m_z = 0.0f;

		pivotMatrix.m_posit -= error.Scale (0.35f);
		dTrace (("(%f %f %f) (%f %f %f)\n", com.m_x, com.m_y, com.m_z
										  , pivotMatrix.m_posit.m_x, pivotMatrix.m_posit.m_y, pivotMatrix.m_posit.m_z));

		dMatrix rootMatrix;
		NewtonBodyGetMatrix (m_rootEffector0->GetBody1(), &rootMatrix[0][0]);
		
		dMatrix effectorMatrix (pivotMatrix * rootMatrix.Inverse());
		for (dModelKeyFramePose::dListNode* node = output.GetFirst(); node; node = node->GetNext()) {
			dModelKeyFrame& transform = node->GetInfo();
			transform.m_posit = effectorMatrix.m_posit;
			//transform.SetMatrix(effectorMatrix);
		}
	}

	dModelAnimTree* m_child;
	dCustomKinematicController* m_rootEffector0;
};


class dKinematicRagdoll: public dModelRootNode
{
	public:
	dKinematicRagdoll(NewtonBody* const rootBody)
		:dModelRootNode(rootBody, dGetIdentityMatrix())
		,m_pose()
		,m_animtree(NULL)
	{
	}

	~dKinematicRagdoll()
	{
		if (m_animtree) {
			delete m_animtree;
		}
	}

	void SetAnimTree(dCustomKinematicController* const rootEffector0)
	{
		dModelAnimTreePose* const idlePose = new dModelAnimTreePose(this, m_pose);
		dModelAnimTreeSelfBalanceCalculator* const balanceCalculator = new dModelAnimTreeSelfBalanceCalculator(this, idlePose, rootEffector0);
		m_animtree = balanceCalculator;
	}

	void OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
	{
		dFloat scale = debugContext->GetScale();
		debugContext->SetScale(0.5f);
		for (dModelKeyFramePose::dListNode* node = m_pose.GetFirst(); node; node = node->GetNext()) {
			const dModelKeyFrame& keyFrame = node->GetInfo();
			keyFrame.m_effector->Debug(debugContext);
		}

		if (m_animtree) {
			m_animtree->Debug(debugContext);
		}

		debugContext->SetScale(scale);
	}

	void ApplyControls (dFloat timestep)
	{
		m_animtree->GeneratePose(timestep, m_pose);

		for (dModelKeyFramePose::dListNode* node = m_pose.GetFirst(); node; node = node->GetNext()) {
			dModelKeyFrame& transform = node->GetInfo();
			transform.m_effector->SetTargetMatrix(dMatrix(transform.m_rotation, transform.m_posit));
		}
	}

	dModelKeyFramePose m_pose;
	dModelAnimTree* m_animtree;
};


class dKinematicRagdollManager: public dModelManager
{
	public:
	dKinematicRagdollManager(DemoEntityManager* const scene)
		:dModelManager(scene->GetNewton())
		//,m_currentController(NULL)
	{
		//scene->Set2DDisplayRenderFunction(RenderHelpMenu, NULL, this);
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

		PhysicsApplyGravityForce(body, timestep, threadIndex);
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

	dCustomKinematicController* ConnectEffector(dModelNode* const effectorNode, const dMatrix& effectorMatrix, const dFloat modelMass)
	{
		dCustomKinematicController* const effector = new dCustomKinematicController(effectorNode->GetBody(), effectorMatrix, effectorNode->GetRoot()->GetBody());
		effector->SetSolverModel(1);
		effector->SetMaxLinearFriction(modelMass * 9.8f * 10.0f);
		effector->SetMaxAngularFriction(modelMass * 100.0f);
		return effector;
	}

	void ConnectLimb(NewtonBody* const bone, NewtonBody* const parent, const dKinematicBoneDefinition& definition)
	{
		dMatrix matrix;
		NewtonBodyGetMatrix(bone, &matrix[0][0]);

		dKinematicBoneDefinition::dFrameMatrix frameAngle(definition.m_frameBasics);
		dMatrix pinAndPivotInGlobalSpace(dPitchMatrix(frameAngle.m_pitch * dDegreeToRad) * dYawMatrix(frameAngle.m_yaw * dDegreeToRad) * dRollMatrix(frameAngle.m_roll * dDegreeToRad));
		pinAndPivotInGlobalSpace = pinAndPivotInGlobalSpace * matrix;

		switch (definition.m_type)
		{
			case dKinematicBoneDefinition::m_0dof:
			{
				dCustomHinge* const joint = new dCustomHinge(pinAndPivotInGlobalSpace, bone, parent);
				joint->EnableLimits(true);
				joint->SetLimits(0.0f, 0.0f);
				break;
			}

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
	}

	void CreateKinematicModel(dModelDescritor& descriptor, const dMatrix& location) 
	{
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
		dModelNode* parentBones[32];
		DemoEntity* childEntities[32];
		for (DemoEntity* child = entityModel->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = model;
			childEntities[stackIndex] = child;
			stackIndex++;
			const char* const name = child->GetName().GetStr();
			dTrace(("name: %s\n", name));
		}

		// walk model hierarchic adding all children designed as rigid body bones. 

		dCustomKinematicController* rootEffector0;
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
						dModelKeyFrame effectorPose;
						dMatrix effectorMatrix(entity->CalculateGlobalMatrix());
						effectorPose.m_effector = ConnectEffector(parentBone, effectorMatrix, descriptor.m_mass);

						effectorPose.SetMatrix (effectorPose.m_effector->GetTargetMatrix());
						model->m_pose.Append(effectorPose);

						rootEffector0 = effectorPose.m_effector;

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
		NormalizeMassAndInertia(model, descriptor.m_mass);

#if 1
		dCustomHinge* fixToWorld = new dCustomHinge (matrix0 * location, model->GetBody(), NULL);
		fixToWorld->EnableLimits(true);
		fixToWorld->SetLimits(0.0f, 0.0f);
#endif

		// setup the pose generator 
		model->SetAnimTree(rootEffector0);

		//m_currentController = robot;
	}

	virtual void OnDebug(dModelRootNode* const root, dCustomJoint::dDebugDisplay* const debugContext)
	{
		dKinematicRagdoll* const model = (dKinematicRagdoll*)root;
		model->OnDebug(debugContext);
	}

	virtual void OnUpdateTransform(const dModelNode* const bone, const dMatrix& localMatrix) const
	{
		NewtonBody* const body = bone->GetBody();
		DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(body);
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));
		
		dQuaternion rot(localMatrix);
		ent->SetMatrix(*scene, rot, localMatrix.m_posit);
	}

	virtual void OnPreUpdate(dModelRootNode* const root, dFloat timestep) const 
	{
		dKinematicRagdoll* const model = (dKinematicRagdoll*)root;
		model->ApplyControls (timestep);
	}
	
	//dSixAxisRobot* m_currentController;
};



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
	origin.m_posit.m_y += 0.5f;
	
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




