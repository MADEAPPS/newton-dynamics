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
#include "DebugDisplay.h"
#include "PhysicsUtils.h"
#include "TargaToOpenGl.h"
#include "DemoEntityManager.h"
#include "dCustomBallAndSocket.h"
#include "HeightFieldPrimitive.h"


class dHaxapodController: public dCustomControllerBase
{
	public:
	class dHexapodMotor: public dCustomHinge
	{
		public:
		dHexapodMotor(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent, dFloat minAngle, dFloat maxAngle)
			:dCustomHinge(pinAndPivotFrame, child, parent)
			,m_torque(1000.0f)
		{
			m_maxAngle = dAbs (minAngle);
			m_minAngle = - dAbs (minAngle);
			EnableLimits(false);
		}

		void SubmitConstraintsFreeDof(dFloat timestep, const dMatrix& matrix0, const dMatrix& matrix1)
		{
/*
			dFloat angle = m_curJointAngle.GetAngle();
			if (angle < m_minAngle) {
				dFloat relAngle = angle - m_minAngle;
				NewtonUserJointAddAngularRow(m_joint, -relAngle, &matrix1.m_front[0]);
				NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
			} else if (angle > m_maxAngle) {
				dFloat relAngle = angle - m_maxAngle;
				NewtonUserJointAddAngularRow(m_joint, -relAngle, &matrix1.m_front[0]);
				NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
			} else {
				NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_front[0]);
				dFloat accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
				NewtonUserJointSetRowAcceleration(m_joint, accel);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
			}
*/
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_front[0]);
			dFloat accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
		}

		void Debug(dDebugDisplay* const debugDisplay) const
		{
			dCustomHinge::Debug(debugDisplay);
		}

		dFloat m_torque;
	};


	class dHexapodEffector: public dCustomRagdollMotor_EndEffector
	{
		public:
		dHexapodEffector (NewtonInverseDynamics* const invDynSolver, void* const invDynNode, const dMatrix& attachmentMatrixInGlobalSpace)
			:dCustomRagdollMotor_EndEffector (invDynSolver, invDynNode, attachmentMatrixInGlobalSpace)
		{
		}
	};

	class dHexapodNode
	{
		public:
		dHexapodNode(dHexapodNode* const parent)
			:m_matrix(dGetIdentityMatrix())
			,m_worldMatrix(dGetIdentityMatrix())
			,m_parent(parent)
			,m_effector(NULL)
			,m_children()
		{
		}

		virtual ~dHexapodNode()
		{
			for (dList<dHexapodNode*>::dListNode* node = m_children.GetFirst(); node; node = node->GetNext()) {
				delete node->GetInfo();
			}
		}

		virtual void UpdateTranform(const dMatrix& parentMatrix)
		{
			m_worldMatrix = m_matrix * parentMatrix;
			for (dList<dHexapodNode*>::dListNode* nodePtr = m_children.GetFirst(); nodePtr; nodePtr = nodePtr->GetNext()) {
				dHexapodNode* const node = nodePtr->GetInfo();
				node->UpdateTranform(m_worldMatrix);
			}
		}

		virtual void UpdateEffectors(dFloat timestep)
		{
			for (dList<dHexapodNode*>::dListNode* nodePtr = m_children.GetFirst(); nodePtr; nodePtr = nodePtr->GetNext()) {
				dHexapodNode* const node = nodePtr->GetInfo();
				node->UpdateEffectors(timestep);
			}
		}

		void Debug(dCustomJoint::dDebugDisplay* const debugContext) const
		{
			m_effector->Debug(debugContext);
			for (dList<dHexapodNode*>::dListNode* nodePtr = m_children.GetFirst(); nodePtr; nodePtr = nodePtr->GetNext()) {
				dHexapodNode* const node = nodePtr->GetInfo();
				node->Debug(debugContext);
			}
		}

		dMatrix m_matrix;
		dMatrix m_worldMatrix;
		dHexapodNode* m_parent;
		dHexapodEffector* m_effector;
		dList<dHexapodNode*> m_children;
	};

	class dHexapodLimb: public dHexapodNode
	{
		public:
		dHexapodLimb(dHexapodNode* const parent, dHexapodEffector* const effector, dFloat footRadio)
			:dHexapodNode(parent)
			,m_radiusHeight(footRadio)
		{
			m_effector = effector;
			dAssert (0);
//			m_effector->SetPickMode(0);
			m_effector->SetMaxLinearFriction(1000.0f);

			if (parent) {
				parent->m_children.Append (this);
			}
		}

		virtual void UpdateEffectors(dFloat timestep)
		{
//			m_effector->SetTargetMatrix(m_worldMatrix);
			dHexapodNode::UpdateEffectors(timestep);
		}

		dFloat m_radiusHeight;
	};

	class dHexapodRoot: public dHexapodNode
	{
		public:
		dHexapodRoot(DemoEntityManager* const scene, const dMatrix& origin)
			:dHexapodNode(NULL)
			,m_kinematicSolver(NULL)
		{
			dFloat mass = 30.0f;
			// make the kinematic solver
			m_kinematicSolver = NewtonCreateInverseDynamics(scene->GetNewton());

			// make the root body
			dMatrix baseMatrix(dGetIdentityMatrix());
//			baseMatrix.m_posit.m_y += 0.35f;
			baseMatrix.m_posit.m_y += 0.75f;
			dVector size (1.3f, 0.31f, 0.5f, 0.0f);
			NewtonBody* const hexaBody = CreateBox(scene, baseMatrix, size, mass, 1.0f);
			void* const hexaBodyNode = NewtonInverseDynamicsAddRoot(m_kinematicSolver, hexaBody);
			m_effector = new dHexapodEffector(m_kinematicSolver, hexaBodyNode, baseMatrix * origin);
			//m_effector = new dHexapodEffector(hexaBody, baseMatrix * origin);
			m_effector->SetMaxLinearFriction(mass * DEMO_GRAVITY * 10.0f);
			m_effector->SetMaxAngularFriction(mass * DEMO_GRAVITY * 4.0f);

			baseMatrix.m_posit.m_y -= 0.06f;
			// make the hexapod six limbs
			for (int i = 0; i < 3; i ++) {
//if (i != 1)
{
				dMatrix rightLocation (baseMatrix);
				rightLocation.m_posit += rightLocation.m_right.Scale (size.m_z * 0.65f);
				rightLocation.m_posit += rightLocation.m_front.Scale (size.m_x * 0.3f - size.m_x * i / 3.0f);
				AddLimb (scene, hexaBodyNode, rightLocation, mass * 0.1f, 0.3f);

				dMatrix similarTransform (dGetIdentityMatrix());
				similarTransform.m_posit.m_x = rightLocation.m_posit.m_x;
				similarTransform.m_posit.m_y = rightLocation.m_posit.m_y;
				dMatrix leftLocation (rightLocation * similarTransform.Inverse() * dYawMatrix(3.141592f) * similarTransform);
				AddLimb (scene, hexaBodyNode, leftLocation, mass * 0.1f, 0.3f);
}
			}

			// finalize inverse dynamics solver
			NewtonInverseDynamicsEndBuild(m_kinematicSolver);
		}

		void AddLimb(DemoEntityManager* const scene, void* const rootNode, const dMatrix& location, dFloat partMass, dFloat limbLenght)
		{
#if 0
			NewtonBody* const parent = NewtonInverseDynamicsGetBody(m_kinematicSolver, rootNode);
			dMatrix baseMatrix (dRollMatrix(3.141592f * 0.5f));

			// make limb base
			dMatrix cylinderMatrix (baseMatrix * location);
			NewtonBody* const base = CreateCylinder(scene, cylinderMatrix, partMass, 10.0f, 0.2f, 0.1f);
			dHexapodMotor* const baseHinge = new dHexapodMotor(cylinderMatrix, base, parent, -3.141592f * 0.5f, 3.141592f * 0.5f);
			//void* const baseHingeNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, rootNode, baseHinge->GetJoint());

			//make limb forward arm
			dMatrix forwardArmMatrix (dPitchMatrix(-30.0f * 3.141592f / 180.0f));
			dVector forwardArmSize (limbLenght * 0.25f, limbLenght * 0.25f, limbLenght, 0.0f);
			forwardArmMatrix.m_posit += forwardArmMatrix.m_right.Scale (forwardArmSize.m_z * 0.5f);
			NewtonBody* const forwardArm = CreateBox(scene, forwardArmMatrix * location, forwardArmSize, partMass, 10.0f);
			dMatrix forwardArmPivot (forwardArmMatrix);
			forwardArmPivot.m_posit -= forwardArmMatrix.m_right.Scale (forwardArmSize.m_z * 0.5f);
			dHexapodMotor* const forwardArmHinge = new dHexapodMotor(forwardArmPivot * location, forwardArm, base, -3.141592f * 0.5f, 3.141592f * 0.5f);
			//void* const forwardArmHingeNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, baseHingeNode, forwardArmHinge->GetJoint());

			//make limb forward arm
			dMatrix armMatrix(dPitchMatrix(-90.0f * 3.141592f / 180.0f));
			dFloat armSize = limbLenght * 1.25f;
			armMatrix.m_posit += forwardArmMatrix.m_right.Scale(limbLenght);
			armMatrix.m_posit.m_y -= armSize * 0.5f;
			NewtonBody* const arm = CreateCapsule(scene, armMatrix * location, partMass, 10.0f, armSize * 0.2f, armSize);
			dMatrix armPivot(armMatrix);
			armPivot.m_posit.m_y += armSize * 0.5f;
			dHexapodMotor* const armHinge = new dHexapodMotor(armPivot * location, arm, forwardArm, -3.141592f * 0.5f, 3.141592f * 0.5f);
			//void* const armHingeNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, forwardArmHingeNode, armHinge->GetJoint());

			dMatrix effectorMatrix(dGetIdentityMatrix());
			effectorMatrix.m_posit = armPivot.m_posit;
			effectorMatrix.m_posit.m_y -= armSize;
#if 1
dHexapodEffector* const effector = new dHexapodEffector(arm, effectorMatrix * location);
effector->SetPickMode(1);
effector->SetMaxLinearFriction(1000.0f);
effector->SetMaxAngularFriction(1000.0f);
#else
			dHexapodEffector* const effector = new dHexapodEffector(m_kinematicSolver, armHingeNode, effectorMatrix * location);
			new dHexapodLimb (this, effector, armSize * 0.2f);
#endif

#else


			NewtonBody* const parent = NewtonInverseDynamicsGetBody(m_kinematicSolver, rootNode);

			dFloat inertiaScale = 4.0f;
			// make limb base
			dMatrix baseMatrix(dRollMatrix(3.141592f * 0.5f));
			dMatrix cylinderMatrix(baseMatrix * location);
			NewtonBody* const base = CreateCylinder(scene, cylinderMatrix, partMass, inertiaScale, 0.2f, 0.1f);
			dHexapodMotor* const baseHinge = new dHexapodMotor(cylinderMatrix, base, parent, -3.141592f * 0.5f, 3.141592f * 0.5f);
			void* const baseHingeNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, rootNode, baseHinge->GetJoint());

			//make limb forward arm
			dMatrix forwardArmMatrix(dPitchMatrix(-30.0f * 3.141592f / 180.0f));
			dVector forwardArmSize(limbLenght * 0.25f, limbLenght * 0.25f, limbLenght, 0.0f);
			forwardArmMatrix.m_posit += forwardArmMatrix.m_right.Scale(forwardArmSize.m_z * 0.5f);
			NewtonBody* const forwardArm = CreateBox(scene, forwardArmMatrix * location, forwardArmSize, partMass, inertiaScale);
			dMatrix forwardArmPivot(forwardArmMatrix);
			forwardArmPivot.m_posit -= forwardArmMatrix.m_right.Scale(forwardArmSize.m_z * 0.5f);
			dHexapodMotor* const forwardArmHinge = new dHexapodMotor(forwardArmPivot * location, forwardArm, base, -3.141592f * 0.5f, 3.141592f * 0.5f);
			void* const forwardArmHingeNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, baseHingeNode, forwardArmHinge->GetJoint());

			//make limb forward arm
			dMatrix armMatrix(dPitchMatrix(-90.0f * 3.141592f / 180.0f));
			dFloat armSize = limbLenght * 1.25f;
			armMatrix.m_posit += forwardArmMatrix.m_right.Scale(limbLenght);
			armMatrix.m_posit.m_y -= armSize * 0.5f;
			NewtonBody* const arm = CreateCapsule(scene, armMatrix * location, partMass, inertiaScale, armSize * 0.2f, armSize);
			dMatrix armPivot(armMatrix);
			armPivot.m_posit.m_y += armSize * 0.5f;
			dHexapodMotor* const armHinge = new dHexapodMotor(armPivot * location, arm, forwardArm, -3.141592f * 0.5f, 3.141592f * 0.5f);
			//void* const armHingeNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, forwardArmHingeNode, armHinge->GetJoint());
			NewtonInverseDynamicsAddChildNode(m_kinematicSolver, forwardArmHingeNode, armHinge->GetJoint());

			dMatrix effectorMatrix(dGetIdentityMatrix());
			effectorMatrix.m_posit = armPivot.m_posit;
			effectorMatrix.m_posit.m_y -= armSize;
			//dHexapodEffector* const effector = new dHexapodEffector(arm, effectorMatrix * location);
			dCustomKinematicController* const effector = new dCustomKinematicController(arm, effectorMatrix * location);
			NewtonInverseDynamicsAddLoopJoint(m_kinematicSolver, effector->GetJoint());
			effector->SetPickMode(0);
			effector->SetMaxLinearFriction(10000.0f);
			effector->SetMaxAngularFriction(10000.0f);
#endif

		}

		~dHexapodRoot()
		{
			if (m_kinematicSolver) {
				NewtonInverseDynamicsDestroy (m_kinematicSolver);
			}
		}

		void SetTarget (dFloat z, dFloat y, dFloat azimuth, dFloat pitch, dFloat roll)
		{
/*
			// set base matrix
			m_matrix = dYawMatrix(azimuth);
			m_matrix.m_posit = m_matrix.TransformVector(m_referencePosit);

			// set effector matrix
			m_effector->m_matrix = dRollMatrix(roll) * dPitchMatrix(pitch);
			m_effector->m_matrix.m_posit = dVector(0.0f, y, z, 1.0f);

			// calculate global matrices
			UpdateTranform(dGetIdentityMatrix());
*/
		}

		virtual void UpdateEffectors(dFloat timestep)
		{
			if (m_kinematicSolver) {
				//dVector xxx (m_effector->GetBodyMatrix().m_posit);
				dVector xxx (m_effector->GetTargetMatrix().m_posit);
				xxx.m_y = 0.75f;
				m_effector->SetTargetPosit(xxx);

				dHexapodNode::UpdateEffectors(timestep);
				NewtonInverseDynamicsUpdate(m_kinematicSolver, timestep, 0);
			}
		}

		void ScaleIntertia(NewtonBody* const body, dFloat factor) const
		{
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;
			dFloat mass;
			NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
			NewtonBodySetMassMatrix(body, mass, Ixx * factor, Iyy * factor, Izz * factor);
		}

		NewtonBody* CreateBox(DemoEntityManager* const scene, const dMatrix& location, const dVector& size, dFloat mass, dFloat inertiaScale) const
		{
			NewtonWorld* const world = scene->GetNewton();
			int materialID = NewtonMaterialGetDefaultGroupID(world);
			NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);
			DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");
			NewtonBody* const body = CreateSimpleSolid(scene, geometry, mass, location, collision, materialID);
			ScaleIntertia(body, inertiaScale);

			geometry->Release();
			NewtonDestroyCollision(collision);
			return body;
		}

		NewtonBody* CreateCapsule(DemoEntityManager* const scene, const dMatrix& location, dFloat mass, dFloat inertiaScale, dFloat radius, dFloat height) const
		{
			NewtonWorld* const world = scene->GetNewton();
			int materialID = NewtonMaterialGetDefaultGroupID(world);
			dVector size(radius, height, radius, 0.0f);
			dMatrix align (dYawMatrix(3.141592f * 0.5f));
			NewtonCollision* const collision = CreateConvexCollision(world, align, size, _CAPSULE_PRIMITIVE, 0);
			DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");

			NewtonBody* const body = CreateSimpleSolid(scene, geometry, mass, location, collision, materialID);
			ScaleIntertia(body, inertiaScale);

			geometry->Release();
			NewtonDestroyCollision(collision);
			return body;
		}

		NewtonBody* CreateCylinder(DemoEntityManager* const scene, const dMatrix& location, dFloat mass, dFloat inertiaScale, dFloat radius, dFloat height) const
		{
			NewtonWorld* const world = scene->GetNewton();
			int materialID = NewtonMaterialGetDefaultGroupID(world);
			dVector size(radius, height, radius, 0.0f);
			NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _CYLINDER_PRIMITIVE, 0);
			DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");

			NewtonBody* const body = CreateSimpleSolid(scene, geometry, mass, location, collision, materialID);
			ScaleIntertia(body, inertiaScale);

			geometry->Release();
			NewtonDestroyCollision(collision);
			return body;
		}

		NewtonInverseDynamics* m_kinematicSolver;
	};

	dHaxapodController()
		:m_robot (NULL)
	{
	}

	~dHaxapodController()
	{
		if (m_robot) {
			delete m_robot;
		}
	}

	void SetTarget (dFloat x, dFloat y, dFloat azimuth, dFloat pitch, dFloat roll)
	{
		if (m_robot) {
			m_robot->SetTarget(x, y, azimuth, pitch, roll);
		}
	}

	void MakeHexapod(DemoEntityManager* const scene, const dMatrix& location)
	{
		m_robot = new dHexapodRoot(scene, location);
	}

	void PostUpdate(dFloat timestep, int threadIndex) 
	{
	}

	void PreUpdate(dFloat timestep, int threadIndex)
	{
Sleep (10);
		if (m_robot) {
			m_robot->UpdateEffectors(timestep);
		}
	}

	void Debug(dCustomJoint::dDebugDisplay* const debugContext) const
	{
		if (m_robot) {
			m_robot->Debug(debugContext);
		}
	}

	dHexapodRoot* m_robot;
};

class dHexapodManager: public dCustomControllerManager<dHaxapodController>
{
	public:
	dHexapodManager(DemoEntityManager* const scene)
		:dCustomControllerManager<dHaxapodController>(scene->GetNewton(), "sixAxisManipulator")
		,m_currentController(NULL)
		,m_azimuth(0.0f)
		,m_posit_x(0.0f)
		,m_posit_y(0.0f)
		,m_gripper_roll(0.0f)
		,m_gripper_pitch(0.0f)
	{
//		scene->Set2DDisplayRenderFunction(RenderHelpMenu, NULL, this);
	}

	~dHexapodManager()
	{
	}

	static void RenderHelpMenu(DemoEntityManager* const scene, void* const context)
	{
		dHexapodManager* const me = (dHexapodManager*)context;

		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "Hexapod controller");
		ImGui::SliderFloat("Azimuth", &me->m_azimuth, -360.0f, 360.0f);
		ImGui::SliderFloat("posit_x", &me->m_posit_x, -1.0f, 1.0f);
		ImGui::SliderFloat("posit_y", &me->m_posit_y, -1.0f, 1.0f);
/*
		ImGui::Separator();
		ImGui::Separator();
		ImGui::SliderFloat("eff_roll", &me->m_gripper_roll, -360.0f, 360.0f);
		ImGui::SliderFloat("eff_pitch", &me->m_gripper_pitch, -60.0f, 60.0f);
		for (dListNode* node = me->GetFirst(); node; node = node->GetNext()) {
			dHaxapodController* const controller = &node->GetInfo();
			controller->SetTarget (me->m_posit_x, me->m_posit_y, me->m_azimuth * 3.141592f / 180.0f, me->m_gripper_pitch * 3.141592f / 180.0f, me->m_gripper_roll * 3.141592f / 180.0f);
		}
*/
	}

	virtual dHaxapodController* CreateController()
	{
		return (dHaxapodController*)dCustomControllerManager<dHaxapodController>::CreateController();
	}

	dHaxapodController* MakeHexapod(DemoEntityManager* const scene, const dMatrix& location)
	{
		dHaxapodController* const controller = (dHaxapodController*)CreateController();
		controller->MakeHexapod(scene, location);
		m_currentController = controller;
		return controller;
	}

	void OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
	{
		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			dHaxapodController* const controller = &node->GetInfo();
			controller->Debug(debugContext);
		}
	}

	dHaxapodController* m_currentController;
	dFloat m_azimuth;
	dFloat m_posit_x;
	dFloat m_posit_y;
	dFloat m_gripper_roll;
	dFloat m_gripper_pitch;
};


void Hexapod(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	CreateLevelMesh (scene, "flatPlane.ngd", true);
	dHexapodManager* const robotManager = new dHexapodManager(scene);

	dMatrix location (dGetIdentityMatrix());
	robotManager->MakeHexapod (scene, location);

	dVector origin(0.0f);
	origin.m_x = -2.0f;
	origin.m_y  = 0.5f;
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}

