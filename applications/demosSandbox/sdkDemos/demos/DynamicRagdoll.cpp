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

#if 0
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

	{ "Bip01_L_Thigh", "capsule", three_dof, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f, 0.19f, 0.05f, 0.34f, 14.0f, -45.0f, 45.0f, 120.0f, 0.0f, -90.0f, -0.0f, 10000.0f },
	{ "Bip01_L_Calf", "capsule", one_dof, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f, 0.19f, 0.05f, 0.34f, 10.0f, -140.0f, 10.0f, 0.0f, 90.0f, 0.0f, 90.0f, 5000.0f },
	{ "Bip01_L_Foot", "convexhull", two_dof, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 60.0f, 0.0f, -90.0f, -0.0f, 5000.0f },

//	{ "Bip01_R_Thigh", "capsule", three_dof, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f, 0.19f, 0.05f, 0.34f, 14.0f, -45.0f, 45.0f, 120.0f, 0.0f, -90.0f, -0.0f, 10000.0f },
//	{ "Bip01_R_Calf", "capsule", one_dof, 0.0f, 90.0f, 0.0f, 0.0f, 0.0f, 0.19f, 0.05f, 0.34f, 10.0f, -140.0f, 10.0f, 0.0f, 90.0f, 0.0f, 90.0f, 5000.0f },
//	{ "Bip01_R_Foot", "convexhull", two_dof, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 60.0f, 0.0f, -90.0f, -0.0f, 5000.0f },

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
	class dBiped1d: public dCustomRagdollMotor_1dof
	{
		public:
		dBiped1d(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
			:dCustomRagdollMotor_1dof(pinAndPivotFrame, child, parent)
		{
		}

		void Debug(dDebugDisplay* const debugDisplay) const
		{
			//dCustomRagdollMotor_1dof::Debug(debugDisplay);
		}
	};

	class dBiped2d: public dCustomRagdollMotor_2dof
	{
		public:
		dBiped2d (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
			:dCustomRagdollMotor_2dof(pinAndPivotFrame, child, parent)
		{
		}

		void Debug(dDebugDisplay* const debugDisplay) const
		{
			dCustomRagdollMotor_2dof::Debug(debugDisplay);
		}
	};

	class dBipedEffector: public dCustomRagdollMotor_EndEffector
	{
		public:
		dBipedEffector(NewtonInverseDynamics* const invDynSolver, void* const invDynNode, NewtonBody* const referenceBody, const dMatrix& attachmentMatrixInGlobalSpace)
			:dCustomRagdollMotor_EndEffector(invDynSolver, invDynNode, referenceBody, attachmentMatrixInGlobalSpace)
		{
			SetLinearSpeed(1.0f);
		}
	};

	class dBiped: public dCustomAlloc
	{
		public:
		dBiped(dCustomArticulatedTransformController* const controller)
			:m_animTreeNode(NULL)
			,m_kinematicSolver(NULL)
			,m_controller(controller)
			,m_leftLegEffector(NULL)
			,m_rightLegEffector(NULL)
		{
			NewtonWorld* const workd = controller->GetManager()->GetWorld();

			// make the kinematic solver
			m_kinematicSolver = NewtonCreateInverseDynamics(workd);

			// build the articulated body hierarchy 
			BuildInverseDynamics();

			// finalize inverse dynamics solver
			NewtonInverseDynamicsEndBuild(m_kinematicSolver);

			// make sure innerta are too skewed
			NormalizeInertia();

			// create an animation tree
			NewtonBody* const rootBody = NewtonInverseDynamicsGetBody(m_kinematicSolver, NewtonInverseDynamicsGetRoot(m_kinematicSolver));
			dEffectorTreeFixPose* const idlePose = new dEffectorTreeFixPose(rootBody);

			//m_animTreeNode = new dEffectorTreeRoot(rootBody, m_postureModifier);
			m_animTreeNode = new dEffectorTreeRoot(rootBody, idlePose);

			dMatrix invRootMatrix;
			NewtonBodyGetMatrix(rootBody, &invRootMatrix[0][0]);
			invRootMatrix = invRootMatrix.Inverse();

			{
				dMatrix poseMatrix;
				dMatrix effectorMatrix;
				dEffectorTreeInterface::dEffectorTransform frame;
				dCustomRagdollMotor_EndEffector* effector;

				effector = m_leftLegEffector;
				if (effector) {
					// set left foot effector local position
					effectorMatrix = effector->GetBodyMatrix();
				effectorMatrix.m_posit.m_z -= 0.4f;
				effectorMatrix.m_posit.m_y -= 0.1f;

					poseMatrix = effectorMatrix * invRootMatrix;
					frame.m_effector = effector;
					frame.m_posit = poseMatrix.m_posit;
					frame.m_rotation = dQuaternion(poseMatrix);
					idlePose->GetPose().Append(frame);
					//walkPoseGenerator->GetPose().Append(frame);
					m_animTreeNode->GetPose().Append(frame);
				}

				effector = m_rightLegEffector;
				if (effector) {
					// set right foot effector local position
					effectorMatrix = effector->GetBodyMatrix();
				effectorMatrix.m_posit.m_z += 0.4f;
				effectorMatrix.m_posit.m_y -= 0.1f;

					poseMatrix = effectorMatrix * invRootMatrix;
					frame.m_effector = effector;
					frame.m_posit = poseMatrix.m_posit;
					frame.m_rotation = dQuaternion(poseMatrix);
					idlePose->GetPose().Append(frame);
					//walkPoseGenerator->GetPose().Append(frame);
					m_animTreeNode->GetPose().Append(frame);
				}
			}

// xxx
NewtonBodySetMassMatrix(rootBody, 0.0f, 0.0f, 0.0f, 0.0f);

		}

		~dBiped()
		{
			NewtonInverseDynamicsDestroy(m_kinematicSolver);
		}


		void BuildInverseDynamics()
		{
			dMatrix rootMatrix;
			void* rootNode[128];
			dCustomArticulatedTransformController::dSkeletonBone* stackPool[128];

			int stack = 1;
			rootNode[0] = NULL;
			stackPool[0] = m_controller->GetRoot();
			NewtonBody* const rootbody = stackPool[0]->m_body;
			NewtonBodyGetMatrix(rootbody, &rootMatrix[0][0]);

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

				DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(bone->m_body);
				const dString& name = entity->GetName();
				if (name == "Bip01_L_Foot") {
					dMatrix matrix0;
					dMatrix matrix1;
					dCustomJoint* const customJoint = (dCustomJoint*)NewtonJointGetUserData(NewtonInverseDynamicsGetJoint(m_kinematicSolver, ikNode));
					dAssert(customJoint->GetBody0() == bone->m_body);
					customJoint->CalculateGlobalMatrix(matrix0, matrix1);
					m_leftLegEffector = new dBipedEffector(m_kinematicSolver, ikNode, rootbody, matrix0);

				} else if (name == "Bip01_R_Foot") {
					dMatrix matrix0;
					dMatrix matrix1;
					dCustomJoint* const customJoint = (dCustomJoint*) NewtonJointGetUserData(NewtonInverseDynamicsGetJoint(m_kinematicSolver, ikNode));
					dAssert (customJoint->GetBody0() == bone->m_body);
					customJoint->CalculateGlobalMatrix (matrix0, matrix1);
					m_rightLegEffector = new dBipedEffector(m_kinematicSolver, ikNode, rootbody, matrix0);
				}

				for (dList<dCustomArticulatedTransformController::dSkeletonBone>::dListNode* node = bone->GetFirst(); node; node = node->GetNext()) {
					rootNode[stack] = ikNode; 
					stackPool[stack] = &node->GetInfo();
					stack++;
				}
			}
		}


		void NormalizeInertia()
		{
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;
			dFloat mass;
			dCustomArticulatedTransformController::dSkeletonBone* stackPool[128];

			int stack = 1;
			stackPool[0] = m_controller->GetRoot();
			NewtonBody* const rootbody = stackPool[0]->m_body;
			NewtonBodyGetMass(rootbody, &mass, &Ixx, &Iyy, &Izz);
			dFloat inertialScale = dMax(dMax(Ixx, Iyy), Izz) / mass;

			while (stack) {
				stack--;
				dCustomArticulatedTransformController::dSkeletonBone* const bone = stackPool[stack];
				NewtonBodyGetMass(bone->m_body, &mass, &Ixx, &Iyy, &Izz);
				NewtonBodySetMassMatrix(bone->m_body, mass, mass * inertialScale, mass * inertialScale, mass * inertialScale);

				for (dList<dCustomArticulatedTransformController::dSkeletonBone>::dListNode* node = bone->GetFirst(); node; node = node->GetNext()) {
					stackPool[stack] = &node->GetInfo();
					stack++;
				}
			}
		}


		void Debug(dCustomJoint::dDebugDisplay* const debugContext) const
		{
			//const dEffectorTreeInterface::dEffectorPose& pose = m_animTreeNode->GetPose();
			if (m_leftLegEffector) {
				m_leftLegEffector->Debug(debugContext);
			}
			if (m_rightLegEffector) {
				m_rightLegEffector->Debug(debugContext);
			}
		}

		void Update(dFloat timestep, int threadIndex)
		{
			//m_animTreeNode->Update(timestep);
			//NewtonInverseDynamicsUpdate(m_kinematicSolver, timestep, threadIndex);
		}

		dEffectorTreeRoot* m_animTreeNode;
		NewtonInverseDynamics* m_kinematicSolver;
		
		// these objects are no to be deleted
		dCustomArticulatedTransformController* m_controller;
		dCustomRagdollMotor_EndEffector* m_leftLegEffector;
		dCustomRagdollMotor_EndEffector* m_rightLegEffector;
		
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
		dMatrix matrix(dPitchMatrix(definition.m_shapePitch * dDegreeToRad) * dYawMatrix(definition.m_shapeYaw * dDegreeToRad) * dRollMatrix(definition.m_shapeRoll * dDegreeToRad));

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

		dMatrix pinAndPivotInGlobalSpace(dPitchMatrix(definition.m_framePitch * dDegreeToRad) * dYawMatrix(definition.m_frameYaw * dDegreeToRad) * dRollMatrix(definition.m_frameRoll * dDegreeToRad));
		pinAndPivotInGlobalSpace = pinAndPivotInGlobalSpace * matrix;

		switch (definition.m_type) 
		{
			case one_dof:
			{
				//dBiped1d* const joint = new dBiped1d(pinAndPivotInGlobalSpace, bone, parent);
				//joint->SetJointTorque(definition.m_frictionTorque);
				//joint->SetTwistAngle(definition.m_minTwistAngle * dDegreeToRad, definition.m_maxTwistAngle * dDegreeToRad);

				dCustomHinge* const joint = new dCustomHinge(pinAndPivotInGlobalSpace, bone, parent);
				joint->EnableLimits(true);
				joint->SetLimits(0.0f, 0.0f);
				break;
			}

			case two_dof:
			{
				//dBiped2d* const joint = new dBiped2d(pinAndPivotInGlobalSpace, bone, parent);
				//joint->DisableMotor();
				//joint->SetJointTorque(definition.m_frictionTorque);
				//joint->SetConeAngle(definition.m_coneAngle * dDegreeToRad);
				dCustomHinge* const joint = new dCustomHinge (pinAndPivotInGlobalSpace, bone, parent);
				joint->EnableLimits(true);
				joint->SetLimits(0.0f, 0.0f);
				break;
			}

			case three_dof:
			{
				//dBiped3d* const joint = new dBiped2d(pinAndPivotInGlobalSpace, bone, parent);
				dBiped2d* const joint = new dBiped2d(pinAndPivotInGlobalSpace, bone, parent);
				//joint->DisableMotor();
				joint->SetJointTorque(definition.m_frictionTorque);
				joint->SetConeAngle(definition.m_coneAngle * dDegreeToRad);
				joint->SetConeAngle(60.0f * dDegreeToRad);
				//joint->SetTwistAngle(definition.m_minTwistAngle * dDegreeToRad, definition.m_maxTwistAngle * dDegreeToRad);
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

		// add the root boneBody
		DemoEntity* const rootEntity = (DemoEntity*)ragDollEntity->Find(definition[0].m_boneName);
		NewtonBody* const rootBone = CreateRagDollBodyPart(rootEntity, definition[0]);
		// for debugging

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
					NewtonBody* const boneBody = CreateRagDollBodyPart(entity, definition[i]);

					// connect this body part to its parent with a ragdoll joint
					ConnectBodyParts(boneBody, parentBone->m_body, definition[i]);

					dMatrix bindMatrix(entity->GetParent()->CalculateGlobalMatrix((DemoEntity*)NewtonBodyGetUserData(parentBone->m_body)).Inverse());
					parentBone = controller->AddBone(boneBody, bindMatrix, parentBone);
					// save the controller as the collision user data, for collision culling
					NewtonCollisionSetUserData(NewtonBodyGetCollision(boneBody), parentBone);
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

	void OnDebug (dCustomJoint::dDebugDisplay* const debugContext)
	{
		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			dCustomArticulatedTransformController* const controller = &node->GetInfo();
			dBiped* const biped = (dBiped*)controller->GetUserData();
			biped->Debug(debugContext);
		}
	}

	int m_material;
};



void DynamicRagDoll (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	dAssert (0);

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
		limbMatrix = dPitchMatrix(-40.0f * dDegreeToRad) * limb->GetCurrentMatrix();
		limb->ResetMatrix(*scene, limbMatrix);

		limb = ragDollModel.Find("Bip01_R_UpperArm");
		limbMatrix = dPitchMatrix(40.0f * dDegreeToRad) * limb->GetCurrentMatrix();
		limb->ResetMatrix(*scene, limbMatrix);

		limb = ragDollModel.Find("Bip01_L_Thigh");
		//limbMatrix = dPitchMatrix(40.0f * dDegreeToRad) * limb->GetCurrentMatrix();
		//limb->ResetMatrix(*scene, limbMatrix);

		limb = ragDollModel.Find("Bip01_R_Thigh");
		limbMatrix = dPitchMatrix(-40.0f * dDegreeToRad) * limb->GetCurrentMatrix();
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
			matrix.m_posit.m_y += 0.2f;
			//matrix.m_posit.m_y += 3.0f;
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
#endif


struct dRagDollConfig
{
	char m_partName[32];
	dFloat m_mass;
	dFloat m_minLimit;
	dFloat m_maxLimit;
};

static dRagDollConfig ragDollConfig[] =
{
	{ "bone_spine0", 210.0f, -1000.0f, 1000.0f },
//	{ "bone_base1", 200.0f, -1000.0f, 1000.0f },
//	{ "bone_arm0", 180.0f, -45.0f, 135.0f },
//	{ "bone_arm1", 160.0f, -90.0f, 35.0f },
//	{ "effector_arm", 1000.0f, 0.0f, 0.0f }
};


class BalancingDummyManager : public dAnimationCharacterRigManager
{
	public:
	BalancingDummyManager(DemoEntityManager* const scene)
		:dAnimationCharacterRigManager(scene->GetNewton())
		//,m_currentController(NULL)
		//,m_azimuth(0.0f)
		//,m_posit_x(0.0f)
		//,m_posit_y(0.0f)
		//,m_gripper_roll(0.0f)
		//,m_gripper_pitch(0.0f)
	{
		//scene->Set2DDisplayRenderFunction(RenderHelpMenu, NULL, this);
	}

	~BalancingDummyManager()
	{
	}

/*
	static void RenderHelpMenu(DemoEntityManager* const scene, void* const context)
	{
		BalancingDummyManager* const me = (BalancingDummyManager*)context;

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

	void OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
	{
		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			dSixAxisController* const controller = &node->GetInfo();
			controller->Debug(debugContext);
		}
	}
*/

	DemoEntity* FindMesh(const DemoEntity* const bodyPart) const
	{
		for (DemoEntity* child = bodyPart->GetChild(); child; child = child->GetSibling()) {
			if (child->GetMesh()) {
				return child;
			}
		}
		dAssert(0);
		return NULL;
	}

	NewtonCollision* MakeConvexHull(const DemoEntity* const bodyPart) const
	{
		dVector points[1024 * 16];

		const DemoEntity* const meshEntity = FindMesh(bodyPart);

		DemoMesh* const mesh = (DemoMesh*)meshEntity->GetMesh();
		dAssert(mesh->IsType(DemoMesh::GetRttiType()));
		dAssert(mesh->m_vertexCount && (mesh->m_vertexCount < int(sizeof(points) / sizeof(points[0]))));

		// go over the vertex array and find and collect all vertices's weighted by this bone.
		const dFloat* const array = mesh->m_vertex;
		for (int i = 0; i < mesh->m_vertexCount; i++) {
			points[i][0] = array[i * 3 + 0];
			points[i][1] = array[i * 3 + 1];
			points[i][2] = array[i * 3 + 2];
			points[i][3] = 0.0f;
		}
		dMatrix matrix(meshEntity->GetMeshMatrix());
		matrix = matrix * meshEntity->GetCurrentMatrix();
		//matrix = matrix * bodyPart->GetParent()->GetCurrentMatrix();
		matrix.TransformTriplex(&points[0][0], sizeof(dVector), &points[0][0], sizeof(dVector), mesh->m_vertexCount);
		//return NewtonCreateConvexHull(GetWorld(), mesh->m_vertexCount, &points[0][0], sizeof(dVector), 1.0e-3f, SERVO_VEHICLE_DEFINITION::m_bodyPart, NULL);
		return NewtonCreateConvexHull(GetWorld(), mesh->m_vertexCount, &points[0][0], sizeof(dVector), 1.0e-3f, 0, NULL);
	}

	NewtonBody* CreateBodyPart(DemoEntity* const bodyPart, const dRagDollConfig& definition)
	{
		NewtonCollision* const shape = MakeConvexHull(bodyPart);

		// calculate the bone matrix
		dMatrix matrix(bodyPart->CalculateGlobalMatrix());

		NewtonWorld* const world = GetWorld();

		// create the rigid body that will make this bone
		NewtonBody* const body = NewtonCreateDynamicBody(world, shape, &matrix[0][0]);

		// destroy the collision helper shape 
		NewtonDestroyCollision(shape);

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(body);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonBodySetMassProperties(body, definition.m_mass, collision);
		//NewtonBodySetMassProperties(body, 0.0f, collision);

		// save the user lifterData with the bone body (usually the visual geometry)
		NewtonBodySetUserData(body, bodyPart);

		// assign a body part id
		//NewtonCollisionSetUserID(collision, definition.m_bodyPartID);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);
		return body;
	}

	dAnimationCharacterRig* CreateRagDoll(DemoEntityManager* const scene, const dMatrix& origin)
	{
		DemoEntity* const model = DemoEntity::LoadNGD_mesh("tred_2.ngd", scene->GetNewton());
		scene->Append(model);

		dMatrix modelMatrix(model->GetCurrentMatrix());
		modelMatrix.m_posit = dVector(0.0f);
		modelMatrix.m_posit.m_w = 1.0f;

		model->ResetMatrix(*scene, modelMatrix * origin);

		NewtonBody* const rootBody = CreateBodyPart(model, ragDollConfig[0]);
		NewtonBodySetMassMatrix(rootBody, 0.0f, 0.0f, 0.0f, 0.0f);
		dAnimationCharacterRig* const rig = CreateCharacterRig(rootBody);

		int stackIndex = 0;
		DemoEntity* childEntities[32];
		dAnimationRigJoint* parentBones[32];
		for (DemoEntity* child = model->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = rig;
			childEntities[stackIndex] = child;
			stackIndex++;
		}

		const int partCount = sizeof(ragDollConfig) / sizeof(ragDollConfig[0]);
		while (stackIndex) {
			stackIndex--;
			DemoEntity* const entity = childEntities[stackIndex];
			dAnimationRigJoint* const parentJoint = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			for (int i = 0; i < partCount; i++) {
				if (!strcmp(ragDollConfig[i].m_partName, name)) {

					if (strstr(name, "bone")) {
						// add a bone and all it children
						dAssert(0);
						NewtonBody* const limbBody = CreateBodyPart(entity, ragDollConfig[i]);
						dAnimationRigHinge* const limbJoint = new dAnimationRigHinge(parentJoint, limbBody);

						limbJoint->SetFriction(ragDollConfig[i].m_mass * DEMO_GRAVITY * 50.0f);
						limbJoint->SetLimits(ragDollConfig[i].m_minLimit * dDegreeToRad, ragDollConfig[i].m_maxLimit * dDegreeToRad);

						for (DemoEntity* child = entity->GetChild(); child; child = child->GetSibling()) {
							parentBones[stackIndex] = limbJoint;
							childEntities[stackIndex] = child;
							stackIndex++;
						}
					} else if (strstr(name, "effector")) {
						dAssert(0);
						// add an end effector (end effector can't have children)
						dMatrix pivot(entity->CalculateGlobalMatrix());
						dAnimationRigEffector* const effector = new dAnimationRigEffector(parentJoint->GetAsRigLimb(), pivot);
						effector->SetLinearSpeed(2.0f);
						effector->SetMaxLinearFriction(ragDollConfig[i].m_mass * DEMO_GRAVITY * 50.0f);
					}
					break;
				}
			}
		}

		rig->Finalize();
		return rig;
	}

	void OnUpdateTransform(const dAnimationRigJoint* const bone, const dMatrix& localMatrix) const
	{
/*
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());
		NewtonBody* const newtonBody = bone->GetNewtonBody();
		DemoEntity* const meshEntity = (DemoEntity*)NewtonBodyGetUserData(newtonBody);

		dQuaternion rot(localMatrix);
		meshEntity->SetMatrix(*scene, rot, localMatrix.m_posit);
*/
	}

//	dSixAxisController* m_currentController;
//	dFloat32 m_azimuth;
//	dFloat32 m_posit_x;
//	dFloat32 m_posit_y;
//	dFloat32 m_gripper_roll;
//	dFloat32 m_gripper_pitch;
};

void DynamicRagDoll(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	CreateLevelMesh(scene, "flatPlane.ngd", true);
	BalancingDummyManager* const robotManager = new BalancingDummyManager(scene);

	int count = 10;
	count = 1;

	dMatrix origin (dYawMatrix(-90.0f * dDegreeToRad));
	origin.m_posit.m_x = 2.0f;
	origin.m_posit.m_y = 2.1f;
	for (int i = 0; i < count; i++) {
		robotManager->CreateRagDoll(scene, origin);
		//robotManager->CreateRagDoll (scene, origin1);
		origin.m_posit.m_x += 1.0f;
	}

	origin.m_posit = dVector(-4.0f, 1.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(dGetIdentityMatrix(), origin.m_posit);
}




