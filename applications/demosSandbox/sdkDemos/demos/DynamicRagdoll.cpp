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
#include "dCustomBallAndSocket.h"
#include "DebugDisplay.h"
#include "HeightFieldPrimitive.h"


enum JointType
{
	one_dof,
	two_dof,
	three_dof,
};

struct dBalancingDummyDefinition
{
	char m_boneName[32];
	char m_shapeType[32];

	JointType m_type;

	dFloat m_shapePitch;
	dFloat m_shapeYaw;
	dFloat m_shapeRoll;

	dFloat m_shape_x;
	dFloat m_shape_y;
	dFloat m_shape_z;

	dFloat m_radio;
	dFloat m_height;
	dFloat m_mass;

	dFloat m_minTwistAngle;
	dFloat m_maxTwistAngle;
	dFloat m_coneAngle;

	dFloat m_framePitch;
	dFloat m_frameYaw;
	dFloat m_frameRoll;

	dFloat m_frictionTorque;
};


static dBalancingDummyDefinition skeletonRagDoll[] =
{
	{ "Bip01_Pelvis", "capsule", three_dof, 0.0f, 0.0f, -90.0f, 0.0f, 0.0f, 0.01f, 0.07f, 0.16f, 30.0f, 0.0f, -0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },

	{ "Bip01_L_Thigh", "capsule", three_dof, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f, 0.19f, 0.05f, 0.34f, 14.0f, -45.0f, 45.0f, 120.0f, 0.0f, -90.0f, -0.0f, 100.0f },
	{ "Bip01_L_Calf", "capsule", one_dof, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f, 0.19f, 0.05f, 0.34f, 10.0f, -140.0f, 10.0f, 0.0f, 90.0f, 0.0f, 90.0f, 50.0f },
//	{ "Bip01_L_Foot", "convexhull", two_dof, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 75.0f, 0.0f, -90.0f, -0.0f, 50.0f },

	{ "Bip01_R_Thigh", "capsule", three_dof, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f, 0.19f, 0.05f, 0.34f, 14.0f, -45.0f, 45.0f, 120.0f, 0.0f, -90.0f, -0.0f, 100.0f },
	{ "Bip01_R_Calf", "capsule", one_dof, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f, 0.19f, 0.05f, 0.34f, 10.0f, -140.0f, 10.0f, 0.0f, 90.0f, 0.0f, 90.0f, 50.0f },
//	{ "Bip01_R_Foot", "convexhull", two_dof, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 75.0f, 0.0f, -90.0f, -0.0f, 50.0f },

//	{ "Bip01_Spine", "capsule", three_dof, 0.0f, 0.0f, -90.0f, 0.0f, 0.0f, 0.06f, 0.07f, 0.14f, 20.0f, 30.0f, -30.0f, 30.0f, 0.0f, -90.0f, 0.0f, 100.0f },
//	{ "Bip01_Spine1", "capsule", three_dof, 0.0f, 0.0f, -90.0f, 0.0f, 0.0f, 0.06f, 0.07f, 0.12f, 20.0f, 30.0f, -30.0f, 30.0f, 0.0f, -90.0f, 0.0f, 100.0f },
//	{ "Bip01_Spine2", "capsule", three_dof, 0.0f, 0.0f, -90.0f, 0.0f, 0.0f, 0.06f, 0.07f, 0.08f, 20.0f, 30.0f, -30.0f, 30.0f, 0.0f, -90.0f, 0.0f, 100.0f },

//	{ "Bip01_Neck", "capsule", two_dof, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f, 0.05f, 0.03f, 0.04f, 5.0f, 30.0f, -30.0f, 30.0f, 0.0f, -90.0f, 0.0f, 100.0f },
//	{ "Bip01_Head", "sphere", three_dof, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f, 0.09f, 0.09f, 0.0f, 5.0f, 30.0f, -60.0f, 60.0f, 0.0f, -90.0f, 0.0f, 100.0f },

//	{ "Bip01_L_UpperArm", "capsule", three_dof, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f, 0.12f, 0.03f, 0.23f, 10.0f, 80.0f, 30.0f, 120.0f, 0.0f, -90.0f, 0.0f, 100.0f },
//	{ "Bip01_L_Forearm", "capsule", one_dof, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f, 0.12f, 0.03f, 0.23f, 7.0f, -150.0f, 0.0f, 0.0f, 0.0f, 0.0f, 90.0f, 50.0f },
//	{ "Bip01_L_Hand", "convexhull", two_dof, 0.0f, 00.0f, 0.0f, 0.0f, 0.0f, 0.00f, 0.00f, 0.00f, 2.0f, 0.0f, -45.0f, 45.0f, 0.0f, 0.0f, 90.0f, 10.0f },

//	{ "Bip01_R_UpperArm", "capsule", three_dof, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f, 0.12f, 0.03f, 0.23f, 10.0f, 80.0f, 30.0f, 120.0f, 0.0f, -90.0f, 0.0f, 100.0f },
//	{ "Bip01_R_Forearm", "capsule", one_dof, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f, 0.12f, 0.03f, 0.23f, 7.0f, 0.0f, 150.0f, 0.0f, 0.0f, 0.0f, -90.0f, 50.0f },
//	{ "Bip01_R_Hand", "convexhull", two_dof, 0.0f, 00.0f, 0.0f, 0.0f, 0.0f, 0.00f, 0.00f, 0.00f, 2.0f, 0.0f, -45.0f, 45.0f, 0.0f, 0.0f, -90.0f, 10.0f },
};

class BalancingDummyManager: public dCustomArticulaledTransformManager
{
	public:
	class dBiped: public dCustomAlloc
	{
		public:
		dBiped(dCustomArticulatedTransformController* const controller)
			:m_kinematicSolver(NULL)
			,m_controller(controller)
		{
			NewtonWorld* const workd = controller->GetManager()->GetWorld();

			// make the kinematic solver
			m_kinematicSolver = NewtonCreateInverseDynamics(workd);

			// build the articulated body hierarchy 
			BuildInverseDynamics();

			// finalize inverse dynamics solver
			NewtonInverseDynamicsEndBuild(m_kinematicSolver);
		
		}

		~dBiped()
		{
			NewtonInverseDynamicsDestroy(m_kinematicSolver);
		}


		void BuildInverseDynamics()
		{
			void* rootNode[128];
			dCustomArticulatedTransformController::dSkeletonBone* stackPool[128];

			int stack = 1;
			rootNode[0] = NULL;
			stackPool[0] = m_controller->GetRoot();
			while (stack) {
				stack--;
				void* parentNode = rootNode[stack];
				dCustomArticulatedTransformController::dSkeletonBone* const bone = stackPool[stack];

				void* ikNode = NULL;
				if (parentNode) {
					NewtonBody* const parentBody = bone->m_parent->m_body;
					for (NewtonJoint* joint = NewtonBodyGetFirstJoint(bone->m_body); joint; joint = NewtonBodyGetNextJoint (bone->m_body, joint)) {
						dCustomJoint* const customJoint = (dCustomJoint*) NewtonJointGetUserData(joint);
						if ((customJoint->GetBody0() == parentBody) || (customJoint->GetBody1() == parentBody)) {
							ikNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, parentNode, joint);
							break;
						}
					}
					dAssert (ikNode);
				} else {
					ikNode = NewtonInverseDynamicsAddRoot(m_kinematicSolver, bone->m_body);
				}

				//DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(bone->m_body);
				//const dString& name = entity->GetName();
				//if (name == "Bip01_L_Foot") {
				//	dAssert(0);
				//} else if (name == "Bip01_R_Foot") {
				//	dAssert(0);
				//}

				for (dList<dCustomArticulatedTransformController::dSkeletonBone>::dListNode* node = bone->GetFirst(); node; node = node->GetNext()) {
					rootNode[stack] = ikNode; 
					stackPool[stack] = &node->GetInfo();
					stack++;
				}
			}
		}

		void Update(dFloat timestep, int threadIndex)
		{
//			NewtonInverseDynamicsUpdate(m_kinematicSolver, timestep, threadIndex);
		}

		NewtonInverseDynamics* m_kinematicSolver;
		dCustomArticulatedTransformController* m_controller;
	};


	BalancingDummyManager(DemoEntityManager* const scene)
		:dCustomArticulaledTransformManager(scene->GetNewton())
	{
		// create a material for early collision culling
		m_material = NewtonMaterialCreateGroupID(scene->GetNewton());
		NewtonMaterialSetCallbackUserData(scene->GetNewton(), m_material, m_material, this);
		NewtonMaterialSetCollisionCallback(scene->GetNewton(), m_material, m_material, OnBoneAABBOverlap, NULL);
	}

	static int OnBoneAABBOverlap(const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex)
	{
		NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
		NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);
		dCustomArticulatedTransformController::dSkeletonBone* const bone0 = (dCustomArticulatedTransformController::dSkeletonBone*)NewtonCollisionGetUserData(collision0);
		dCustomArticulatedTransformController::dSkeletonBone* const bone1 = (dCustomArticulatedTransformController::dSkeletonBone*)NewtonCollisionGetUserData(collision1);

		dAssert(bone0);
		dAssert(bone1);
		if (bone0->m_controller && bone1->m_controller) {
			return bone0->m_controller->SelfCollisionTest(bone0, bone1) ? 1 : 0;
		}

		return 1;
	}

	void GetDimentions(DemoEntity* const bodyPart, dVector& origin, dVector& size) const
	{
		DemoMesh* const mesh = (DemoMesh*)bodyPart->GetMesh();
		dAssert(mesh->IsType(DemoMesh::GetRttiType()));

		dFloat* const array = mesh->m_vertex;
		dVector pmin(1.0e20f, 1.0e20f, 1.0e20f, 0.0f);
		dVector pmax(-1.0e20f, -1.0e20f, -1.0e20f, 0.0f);

		for (int i = 0; i < mesh->m_vertexCount; i++) {
			dFloat x = array[i * 3 + 0];
			dFloat y = array[i * 3 + 1];
			dFloat z = array[i * 3 + 2];

			pmin.m_x = x < pmin.m_x ? x : pmin.m_x;
			pmin.m_y = y < pmin.m_y ? y : pmin.m_y;
			pmin.m_z = z < pmin.m_z ? z : pmin.m_z;

			pmax.m_x = x > pmax.m_x ? x : pmax.m_x;
			pmax.m_y = y > pmax.m_y ? y : pmax.m_y;
			pmax.m_z = z > pmax.m_z ? z : pmax.m_z;
		}

		size = (pmax - pmin).Scale(0.5f);
		origin = (pmax + pmin).Scale(0.5f);
		origin.m_w = 1.0f;
	}

	NewtonCollision* MakeSphere(DemoEntity* const bodyPart, const dBalancingDummyDefinition& definition) const
	{
		dVector size(0.0f);
		dVector origin(0.0f);
		dMatrix matrix(dGetIdentityMatrix());

		matrix.m_posit.m_x = definition.m_shape_x;
		matrix.m_posit.m_y = definition.m_shape_y;
		matrix.m_posit.m_z = definition.m_shape_z;
		return NewtonCreateSphere(GetWorld(), definition.m_radio, 0, &matrix[0][0]);
	}

	NewtonCollision* MakeCapsule(DemoEntity* const bodyPart, const dBalancingDummyDefinition& definition) const
	{
		dVector size(0.0f);
		dVector origin(0.0f);
		dMatrix matrix(dPitchMatrix(definition.m_shapePitch * 3.141592f / 180.0f) * dYawMatrix(definition.m_shapeYaw * 3.141592f / 180.0f) * dRollMatrix(definition.m_shapeRoll * 3.141592f / 180.0f));

		matrix.m_posit.m_x = definition.m_shape_x;
		matrix.m_posit.m_y = definition.m_shape_y;
		matrix.m_posit.m_z = definition.m_shape_z;
		return NewtonCreateCapsule(GetWorld(), definition.m_radio, definition.m_radio, definition.m_height, 0, &matrix[0][0]);
	}

	NewtonCollision* MakeBox(DemoEntity* const bodyPart) const
	{
		dAssert(0);
		return NULL;
	}

	NewtonCollision* MakeConvexHull(DemoEntity* const bodyPart) const
	{
		dFloat points[1024 * 16][3];

		DemoMesh* const mesh = (DemoMesh*)bodyPart->GetMesh();
		dAssert(mesh->IsType(DemoMesh::GetRttiType()));
		dAssert(mesh->m_vertexCount && (mesh->m_vertexCount < int(sizeof(points) / sizeof(points[0]))));

		// go over the vertex array and find and collect all vertices's weighted by this bone.
		dFloat* const array = mesh->m_vertex;
		for (int i = 0; i < mesh->m_vertexCount; i++) {
			points[i][0] = array[i * 3 + 0];
			points[i][1] = array[i * 3 + 1];
			points[i][2] = array[i * 3 + 2];
		}

		return NewtonCreateConvexHull(GetWorld(), mesh->m_vertexCount, &points[0][0], 3 * sizeof(dFloat), 1.0e-3f, 0, NULL);
	}

	NewtonBody* CreateRagDollBodyPart(DemoEntity* const bodyPart, const dBalancingDummyDefinition& definition)
	{
		NewtonCollision* shape = NULL;
		if (!strcmp(definition.m_shapeType, "sphere")) {
			shape = MakeSphere(bodyPart, definition);
		} else if (!strcmp(definition.m_shapeType, "capsule")) {
			shape = MakeCapsule(bodyPart, definition);
		} else if (!strcmp(definition.m_shapeType, "box")) {
			shape = MakeBox(bodyPart);
		} else {
			shape = MakeConvexHull(bodyPart);
		}

		// calculate the bone matrix
		dMatrix matrix(bodyPart->CalculateGlobalMatrix());
		NewtonWorld* const world = GetWorld();

		// create the rigid body that will make this bone
		NewtonBody* const bone = NewtonCreateDynamicBody(world, shape, &matrix[0][0]);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonBodySetMassProperties(bone, definition.m_mass, shape);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(bone, bodyPart);

		// assign the material for early collision culling
		NewtonBodySetMaterialGroupID(bone, m_material);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(bone, PhysicsApplyGravityForce);

		// destroy the collision helper shape 
		NewtonDestroyCollision(shape);
		return bone;
	}

	void ConnectBodyParts(NewtonBody* const bone, NewtonBody* const parent, const dBalancingDummyDefinition& definition) const
	{
		dMatrix matrix;
		NewtonBodyGetMatrix(bone, &matrix[0][0]);

		dMatrix pinAndPivotInGlobalSpace(dPitchMatrix(definition.m_framePitch * 3.141592f / 180.0f) * dYawMatrix(definition.m_frameYaw * 3.141592f / 180.0f) * dRollMatrix(definition.m_frameRoll * 3.141592f / 180.0f));
		pinAndPivotInGlobalSpace = pinAndPivotInGlobalSpace * matrix;

		switch (definition.m_type) 
		{
			case one_dof:
			{
				dCustomRagdollMotor_1dof* const joint = new dCustomRagdollMotor_1dof(pinAndPivotInGlobalSpace, bone, parent);
				//joint->DisableMotor();
				joint->SetJointTorque(definition.m_frictionTorque);
				joint->SetTwistAngle(definition.m_minTwistAngle * 3.141592f / 180.0f, definition.m_maxTwistAngle * 3.141592f / 180.0f);
				break;
			}

			case two_dof:
			{
				dCustomRagdollMotor_2dof* const joint = new dCustomRagdollMotor_2dof(pinAndPivotInGlobalSpace, bone, parent);
				//joint->DisableMotor();
				joint->SetJointTorque(definition.m_frictionTorque);
				joint->SetConeAngle(definition.m_coneAngle * 3.141592f / 180.0f);
				break;
			}

			case three_dof:
			{
				//dCustomRagdollMotor_3dof* const joint = new dCustomRagdollMotor_3dof(pinAndPivotInGlobalSpace, bone, parent);
				dCustomRagdollMotor_2dof* const joint = new dCustomRagdollMotor_2dof(pinAndPivotInGlobalSpace, bone, parent);
				//joint->DisableMotor();
				joint->SetJointTorque(definition.m_frictionTorque);
				joint->SetConeAngle(definition.m_coneAngle * 3.141592f / 180.0f);
				//joint->SetTwistAngle(definition.m_minTwistAngle * 3.141592f / 180.0f, definition.m_maxTwistAngle * 3.141592f / 180.0f);
				break;
			}
		}
	}

	dCustomJoint* FindJoint(NewtonBody* const child, NewtonBody* const parent)
	{
		for (NewtonJoint* joint = NewtonBodyGetFirstJoint(child); joint; joint = NewtonBodyGetNextJoint(child, joint)) {
			dCustomJoint* const cJoint = (dCustomJoint*)NewtonJointGetUserData(joint);
			if (((child == cJoint->GetBody0()) && (parent == cJoint->GetBody1())) ||
				((child == cJoint->GetBody1()) && (parent == cJoint->GetBody0()))) {
				return cJoint;
			}
		}
		dAssert(0);
		return NULL;
	}


	virtual void OnUpdateTransform(const dCustomArticulatedTransformController::dSkeletonBone* const bone, const dMatrix& localMatrix) const
	{
		DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(bone->m_body);
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(bone->m_body));

		dQuaternion rot(localMatrix);
		ent->SetMatrix(*scene, rot, localMatrix.m_posit);
	}

	void DestroyController (dCustomArticulatedTransformController* const controller)
	{
		dBiped* const balancingModule = (dBiped*)controller->GetUserData();
		delete balancingModule;
		dCustomArticulaledTransformManager::DestroyController (controller);
	}

	void CreateRagDoll(const dMatrix& location, const DemoEntity* const model, dBalancingDummyDefinition* const definition, int defintionCount)
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		// make a clone of the mesh 
		DemoEntity* const ragDollEntity = (DemoEntity*)model->CreateClone();
		scene->Append(ragDollEntity);

		// build the rag doll with rigid bodies connected by joints
		dCustomArticulatedTransformController* const controller = CreateTransformController();
		controller->SetCalculateLocalTransforms(true);

		// add the root bone
		DemoEntity* const rootEntity = (DemoEntity*)ragDollEntity->Find(definition[0].m_boneName);
		NewtonBody* const rootBone = CreateRagDollBodyPart(rootEntity, definition[0]);
		// for debugging
// xxx
NewtonBodySetMassMatrix(rootBone, 0.0f, 0.0f, 0.0f, 0.0f);

		dCustomArticulatedTransformController::dSkeletonBone* const bone0 = controller->AddRoot(rootBone, dGetIdentityMatrix());
		// save the controller as the collision user data, for collision culling
		NewtonCollisionSetUserData(NewtonBodyGetCollision(rootBone), bone0);

		int stackIndex = 0;
		DemoEntity* childEntities[32];
		dCustomArticulatedTransformController::dSkeletonBone* parentBones[32];
		for (DemoEntity* child = rootEntity->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = bone0;
			childEntities[stackIndex] = child;
			stackIndex++;
		}

		// walk model hierarchic adding all children designed as rigid body bones. 
		while (stackIndex) {
			stackIndex--;
			DemoEntity* const entity = childEntities[stackIndex];
			dCustomArticulatedTransformController::dSkeletonBone* parentBone = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			for (int i = 0; i < defintionCount; i++) {
				if (!strcmp(definition[i].m_boneName, name)) {
					NewtonBody* const bone = CreateRagDollBodyPart(entity, definition[i]);

					// connect this body part to its parent with a ragdoll joint
					ConnectBodyParts(bone, parentBone->m_body, definition[i]);

					dMatrix bindMatrix(entity->GetParent()->CalculateGlobalMatrix((DemoEntity*)NewtonBodyGetUserData(parentBone->m_body)).Inverse());
					parentBone = controller->AddBone(bone, bindMatrix, parentBone);
					// save the controller as the collision user data, for collision culling
					NewtonCollisionSetUserData(NewtonBodyGetCollision(bone), parentBone);
					break;
				}
			}

			for (DemoEntity* child = entity->GetChild(); child; child = child->GetSibling()) {
				parentBones[stackIndex] = parentBone;
				childEntities[stackIndex] = child;
				stackIndex++;
			}
		}

		// set the collision mask
		// note this container work best with a material call back for setting bit field 
		controller->SetDefaultSelfCollisionMask();

		// transform the entire contraction to its location
		dMatrix worldMatrix(rootEntity->GetCurrentMatrix() * location);
		NewtonBodySetMatrixRecursive(rootBone, &worldMatrix[0][0]);

		// add up the active balancing support
		dBiped* const balancingModule = new dBiped(controller);
		controller->SetUserData(balancingModule);
	}

	void OnPreUpdate(dCustomArticulatedTransformController* const controller, dFloat timestep, int threadIndex) const
	{
		dBiped* const biped = (dBiped*)controller->GetUserData();
		biped->Update(timestep, threadIndex);
	}


	int m_material;
};


void DynamicRagDoll (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	CreateLevelMesh (scene, "flatPlane.ngd", true);
	//CreateHeightFieldTerrain(scene, HEIGHTFIELD_DEFAULT_SIZE, HEIGHTFIELD_DEFAULT_CELLSIZE, 1.5f, 0.2f, 200.0f, -50.0f);

	// load a skeleton mesh 
	DemoEntity ragDollModel(dGetIdentityMatrix(), NULL);
	ragDollModel.LoadNGD_mesh("skeleton.ngd", scene->GetNewton());

	{
		// prepare model for rag doll
		DemoEntity* limb;
		dMatrix limbMatrix;

		limb = ragDollModel.Find("Bip01_L_UpperArm");
		limbMatrix = dPitchMatrix(-40.0f * 3.141592f / 180.0f) * limb->GetCurrentMatrix();
		limb->ResetMatrix(*scene, limbMatrix);

		limb = ragDollModel.Find("Bip01_R_UpperArm");
		limbMatrix = dPitchMatrix(40.0f * 3.141592f / 180.0f) * limb->GetCurrentMatrix();
		limb->ResetMatrix(*scene, limbMatrix);

		limb = ragDollModel.Find("Bip01_L_Thigh");
		limbMatrix = dPitchMatrix(40.0f * 3.141592f / 180.0f) * limb->GetCurrentMatrix();
		limb->ResetMatrix(*scene, limbMatrix);

		limb = ragDollModel.Find("Bip01_R_Thigh");
		limbMatrix = dPitchMatrix(-40.0f * 3.141592f / 180.0f) * limb->GetCurrentMatrix();
		limb->ResetMatrix(*scene, limbMatrix);
	}

	//  create a skeletal transform controller for controlling rag doll
	BalancingDummyManager* const manager = new BalancingDummyManager(scene);

	NewtonWorld* const world = scene->GetNewton();
	dMatrix matrix(dGetIdentityMatrix());

	//	dVector origin (-10.0f, 1.0f, 0.0f, 1.0f);
	dVector origin(FindFloor(world, dVector(-10.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));

	int count = 1;
	for (int x = 0; x < count; x++) {
		for (int z = 0; z < count; z++) {
			dVector p(origin + dVector((x - count / 2) * 3.0f - count / 2, 0.0f, (z - count / 2) * 3.0f, 0.0f));
			matrix.m_posit = FindFloor(world, p, 100.0f);
			//matrix.m_posit.m_y += 0.f;
			manager->CreateRagDoll(matrix, &ragDollModel, skeletonRagDoll, sizeof(skeletonRagDoll) / sizeof(skeletonRagDoll[0]));
		}
	}

	//	const int defaultMaterialID = NewtonMaterialGetDefaultGroupID(scene->GetNewton());
	const dVector location(origin);
	const dVector size(0.25f, 0.25f, 0.375f, 0.0f);
	//	const int count1 = 5;
	const dMatrix shapeOffsetMatrix(dGetIdentityMatrix());
	//AddPrimitiveArray(scene, 10.0f, location, size, count1, count1, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);

	origin.m_x -= 1.0f;
	origin.m_x -= 2.0f;
	origin.m_y += 1.0f;
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}




