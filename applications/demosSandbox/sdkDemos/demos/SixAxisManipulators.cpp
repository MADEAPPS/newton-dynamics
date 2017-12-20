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


class dSixAxisController: public dCustomControllerBase
{
	public:
	class dKukaServoMotor1: public dCustomHinge
	{
		public:
		dKukaServoMotor1(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent, dFloat minAngle, dFloat maxAngle)
			:dCustomHinge(pinAndPivotFrame, child, parent)
			,m_torque(1000.0f)
		{
			m_maxAngle = dAbs (minAngle);
			m_minAngle = - dAbs (minAngle);
			EnableLimits(false);
		}

		void SubmitConstraintsFreeDof(dFloat timestep, const dMatrix& matrix0, const dMatrix& matrix1)
		{
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
		}

		void Debug(dDebugDisplay* const debugDisplay) const
		{
		}

		dFloat m_torque;
	};

	class dKukaServoMotor2: public dCustomUniversal
	{
		public:
		dKukaServoMotor2(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
			:dCustomUniversal(pinAndPivotFrame, child, parent)
			,m_torque(1000.0f)
		{
			EnableLimit_0(false);
			EnableLimit_1(false);
		}

		void Debug(dDebugDisplay* const debugDisplay) const
		{
		}

		void SubmitConstraints(dFloat timestep, int threadIndex)
		{
			dMatrix matrix0;
			dMatrix matrix1;
			dFloat accel;

			dCustomUniversal::SubmitConstraints(timestep, threadIndex);

			// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
			CalculateGlobalMatrix(matrix0, matrix1);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix0.m_front[0]);
			accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_up[0]);
			accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
		}

		dFloat m_torque;
	};

#if 0
	class dKukaEffector: public dCustomRagdollMotor_EndEffector
	{
		public:
		dKukaEffector(NewtonInverseDynamics* const invDynSolver, void* const invDynNode, const dMatrix& attachmentMatrixInGlobalSpace)
			:dCustomRagdollMotor_EndEffector(invDynSolver, invDynNode, attachmentMatrixInGlobalSpace)
		{
		}
	};
#else
	class dKukaEffector: public dCustomKinematicController
	{
		public:
		dKukaEffector(NewtonInverseDynamics* const invDynSolver, void* const invDynNode, const dMatrix& attachmentMatrixInGlobalSpace)
			:dCustomKinematicController(invDynSolver, invDynNode, attachmentMatrixInGlobalSpace)
		{
		}

		void SetAsThreedof()
		{
			SetPickMode(0);
		}
	};
#endif

	class dSixAxisNode
	{
		public:
		dSixAxisNode(dSixAxisNode* const parent)
			:m_matrix(dGetIdentityMatrix())
			,m_parent(parent)
			,m_children()
		{
		}

		virtual ~dSixAxisNode()
		{
			for (dList<dSixAxisNode*>::dListNode* node = m_children.GetFirst(); node; node = node->GetNext()) {
				delete node->GetInfo();
			}
		}

		virtual void UpdateTranform(const dMatrix& parentMatrix)
		{
			m_worldMatrix = m_matrix * parentMatrix;
			for (dList<dSixAxisNode*>::dListNode* nodePtr = m_children.GetFirst(); nodePtr; nodePtr = nodePtr->GetNext()) {
				dSixAxisNode* const node = nodePtr->GetInfo();
				node->UpdateTranform(m_worldMatrix);
			}
		}

		virtual void UpdateEffectors(dFloat timestep)
		{
			for (dList<dSixAxisNode*>::dListNode* nodePtr = m_children.GetFirst(); nodePtr; nodePtr = nodePtr->GetNext()) {
				dSixAxisNode* const node = nodePtr->GetInfo();
				node->UpdateEffectors(timestep);
			}
		}

		dMatrix m_matrix;
		dMatrix m_worldMatrix;
		dSixAxisNode* m_parent;
		dList<dSixAxisNode*> m_children;
	};

	class dSixAxisEffector: public dSixAxisNode
	{
		public:
		dSixAxisEffector(dSixAxisNode* const parent, dKukaEffector* const effector)
			:dSixAxisNode(parent)
			,m_effector(effector)
		{
			m_effector->SetAsThreedof();
			m_effector->SetMaxLinearFriction(1000.0f);
			if (parent) {
				parent->m_children.Append (this);
			}
		}

		virtual void UpdateEffectors(dFloat timestep)
		{
			m_effector->SetTargetMatrix (m_worldMatrix);
			dSixAxisNode::UpdateEffectors(timestep);
		}

		dKukaEffector* m_effector;
	};

	class dSixAxisRoot: public dSixAxisNode
	{
		public:
		dSixAxisRoot(DemoEntityManager* const scene, const dVector& origin)
			:dSixAxisNode(NULL)
			,m_kinematicSolver(NULL)
			,m_effector(NULL)
			,m_referencePosit (0.0f)
		{
			m_matrix = dGetIdentityMatrix();
			m_matrix.m_posit = origin;
			m_matrix.m_posit.m_w = 1.0f;

			m_kinematicSolver = NewtonCreateInverseDynamics(scene->GetNewton());

			dMatrix location(dRollMatrix(90.0f * 3.141592f / 180.0f));
			location.m_posit = origin;
			location.m_posit.m_y += 0.125f * 0.5f;

			// add Robot Base
			NewtonBody* const parentBody = CreateCylinder(scene, location, 0.35f, 0.125f);
			dMatrix parentMatrix(dGrammSchmidt(dVector(0.0f, 1.0f, 0.0f)));
			parentMatrix.m_posit = location.m_posit;
			dCustomHinge* const fixHinge = new dCustomHinge(parentMatrix, parentBody, NULL);
			fixHinge->EnableLimits(true);
			fixHinge->SetLimits(0.0f, 0.0f);
			// lock the root body to the world
			void* const rootNode = NewtonInverseDynamicsAddRoot(m_kinematicSolver, parentBody);
			NewtonInverseDynamicsAddLoopJoint(m_kinematicSolver, fixHinge->GetJoint());

			// add Robot rotating platform
			dMatrix baseMatrix(dGetIdentityMatrix());
			baseMatrix.m_posit = location.m_posit;
			baseMatrix.m_posit.m_y += 0.125f * 0.5f + 0.11f;
			baseMatrix.m_posit.m_z += 0.125f * 0.5f;
			NewtonBody* const base = CreateBox(scene, baseMatrix, dVector(0.125f, 0.2f, 0.25f));

			dMatrix baseSpin(dGrammSchmidt(dVector(0.0f, 1.0f, 0.0f)));
			baseSpin.m_posit = location.m_posit;
			dKukaServoMotor1* const baseHinge = new dKukaServoMotor1(baseSpin, base, parentBody, -3.141592f * 2.0f, 3.141592f * 2.0f);
			void* const baseHingeNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, rootNode, baseHinge->GetJoint());

			// add Robot Arm
			dMatrix armMatrix0(dPitchMatrix(45.0f * 3.141592f / 180.0f));
			armMatrix0.m_posit = baseMatrix.m_posit;
			armMatrix0.m_posit.m_y += 0.30f;
			armMatrix0.m_posit.m_x += 0.09f;
			armMatrix0.m_posit.m_z -= 0.125f;
			NewtonBody* const armBody0 = CreateBox(scene, armMatrix0, dVector(0.05f, 0.1f, 0.75f));
			dMatrix armHingeMatrix0(dGrammSchmidt(dVector(1.0f, 0.0f, 0.0f)));
			armHingeMatrix0.m_posit = armMatrix0.m_posit + armMatrix0.RotateVector(dVector(0.0f, 0.0f, 0.3f));
			dKukaServoMotor1* const armJoint0 = new dKukaServoMotor1(armHingeMatrix0, armBody0, base, -3.141592f * 2.0f, 3.141592f * 2.0f);
			void* const armJointNode0 = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, baseHingeNode, armJoint0->GetJoint());

			dMatrix armMatrix1(armMatrix0 * dYawMatrix(3.141592f));
			armMatrix1.m_posit = armMatrix0.m_posit;
			armMatrix1.m_posit.m_y += 0.4f;
			armMatrix1.m_posit.m_x -= 0.05f;
			armMatrix1.m_posit.m_z -= 0.1f;
			dFloat armAngleLimit = 80.0f * 3.141592f / 180.0f;
//			dFloat armAngleLimit = 10.0f * 3.141592f / 180.0f;
			NewtonBody* const armBody1 = CreateBox(scene, armMatrix1, dVector(0.05f, 0.1f, 0.5f));
			dMatrix armHingeMatrix1(dGrammSchmidt(dVector(1.0f, 0.0f, 0.0f)));
			armHingeMatrix1.m_posit = armMatrix1.m_posit + armMatrix1.RotateVector(dVector(0.0f, 0.0f, 0.2f));
			dKukaServoMotor1* const armJoint1 = new dKukaServoMotor1(armHingeMatrix1, armBody1, armBody0, -armAngleLimit, armAngleLimit);
			void* const armJointNode1 = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, armJointNode0, armJoint1->GetJoint());
#if 0
			// Robot gripper base
			dMatrix gripperMatrix(dYawMatrix(90.0f * 3.141592f / 180.0f));
			gripperMatrix.m_posit = armMatrix1.m_posit + armMatrix1.m_right.Scale(-0.25f) + gripperMatrix.m_front.Scale(-0.06f);
			NewtonBody* const gripperBase = CreateCylinder(scene, gripperMatrix, 0.1f, -0.15f);
			dMatrix gripperEffectMatrix(dGetIdentityMatrix());
			gripperEffectMatrix.m_up = dVector(1.0f, 0.0f, 0.0f, 0.0f);
			gripperEffectMatrix.m_front = gripperMatrix.m_front;
			gripperEffectMatrix.m_right = gripperEffectMatrix.m_front.CrossProduct(gripperEffectMatrix.m_up);
			gripperEffectMatrix.m_posit = gripperMatrix.m_posit + gripperMatrix.m_front.Scale(0.065f);
			dKukaServoMotor2* const gripperJoint = new dKukaServoMotor2(gripperEffectMatrix, gripperBase, armBody1);
			void* const gripperJointNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, armJointNode1, gripperJoint->GetJoint());

			// add the inverse dynamics end effector
			//dKukaEndEffector* const effector = new dKukaEndEffector(m_kinematicSolver, gripperJointNode, gripperEffectMatrix);
			dKukaEffector* const effector = new dKukaEffector(m_kinematicSolver, gripperJointNode, gripperEffectMatrix);
#else

			dMatrix gripperMatrix(dYawMatrix(90.0f * 3.141592f / 180.0f));
			gripperMatrix.m_posit = armMatrix1.m_posit + armMatrix1.m_right.Scale(-0.25f) + gripperMatrix.m_front.Scale(-0.06f);
			dMatrix gripperEffectMatrix(dGetIdentityMatrix());
			gripperEffectMatrix.m_up = dVector(1.0f, 0.0f, 0.0f, 0.0f);
			gripperEffectMatrix.m_front = gripperMatrix.m_front;
			gripperEffectMatrix.m_right = gripperEffectMatrix.m_front.CrossProduct(gripperEffectMatrix.m_up);
			gripperEffectMatrix.m_posit = gripperMatrix.m_posit + gripperMatrix.m_front.Scale(0.065f);
			//dKukaEndEffector* const effector = new dKukaEndEffector(m_kinematicSolver, armJointNode1, gripperEffectMatrix);
			dKukaEffector* const effector = new dKukaEffector(m_kinematicSolver, armJointNode1, gripperEffectMatrix);
#endif
			m_effector = new dSixAxisEffector(this, effector);

			// save the tip reference point
			m_referencePosit = gripperEffectMatrix.m_posit - origin;
			m_referencePosit.m_w = 1.0f;

			// complete the ik
			NewtonInverseDynamicsEndBuild(m_kinematicSolver);
		}

		~dSixAxisRoot()
		{
			NewtonInverseDynamicsDestroy (m_kinematicSolver);
		}

		void SetTarget (dFloat z, dFloat y, dFloat azimuth, dFloat pitch, dFloat roll)
		{
			// set base matrix
			m_matrix = dYawMatrix(azimuth);
			m_matrix.m_posit = m_matrix.TransformVector(m_referencePosit);

			// set effector matrix
			m_effector->m_matrix = dRollMatrix(roll) * dPitchMatrix(pitch);
			m_effector->m_matrix.m_posit = dVector(0.0f, y, z, 1.0f);

			// calculate global matrices
			UpdateTranform(dGetIdentityMatrix());
		}

		virtual void UpdateEffectors(dFloat timestep)
		{
			dSixAxisNode::UpdateEffectors(timestep);
			NewtonInverseDynamicsUpdate(m_kinematicSolver, timestep, 0);
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

		NewtonBody* CreateBox(DemoEntityManager* const scene, const dMatrix& location, const dVector& size) const
		{
			NewtonWorld* const world = scene->GetNewton();
			int materialID = NewtonMaterialGetDefaultGroupID(world);
			NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);
			DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");

			dFloat mass = 1.0f;
			NewtonBody* const body = CreateSimpleSolid(scene, geometry, mass, location, collision, materialID);
			ScaleIntertia(body, 10.0f);

			geometry->Release();
			NewtonDestroyCollision(collision);
			return body;
		}

		NewtonBody* CreateCylinder(DemoEntityManager* const scene, const dMatrix& location, dFloat radius, dFloat height) const
		{
			NewtonWorld* const world = scene->GetNewton();
			int materialID = NewtonMaterialGetDefaultGroupID(world);
			dVector size(radius, height, radius, 0.0f);
			NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _CYLINDER_PRIMITIVE, 0);
			DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");

			dFloat mass = 1.0f;
			NewtonBody* const body = CreateSimpleSolid(scene, geometry, mass, location, collision, materialID);
			ScaleIntertia(body, 10.0f);

			geometry->Release();
			NewtonDestroyCollision(collision);
			return body;
		}

		NewtonInverseDynamics* m_kinematicSolver;
		dSixAxisEffector* m_effector;
		dVector m_referencePosit;
	};

	dSixAxisController()
		:m_robot (NULL)
	{
	}

	~dSixAxisController()
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

	void MakeKukaRobot_IK(DemoEntityManager* const scene, const dVector& origin)
	{
		m_robot = new dSixAxisRoot(scene, origin);
	}

	void PostUpdate(dFloat timestep, int threadIndex) 
	{
	}

	void PreUpdate(dFloat timestep, int threadIndex)
	{
		if (m_robot) {
			m_robot->UpdateEffectors(timestep);
		}
	}

	void Debug(dCustomJoint::dDebugDisplay* const debugContext) const
	{
		if (m_robot) {
			m_robot->m_effector->m_effector->Debug(debugContext);
		}
	}

	dSixAxisRoot* m_robot;
};

class dSixAxisManager: public dCustomControllerManager<dSixAxisController>
{
	public:
	dSixAxisManager(DemoEntityManager* const scene)
		:dCustomControllerManager<dSixAxisController>(scene->GetNewton(), "sixAxisManipulator")
		,m_currentController(NULL)
		,m_azimuth(0.0f)
		,m_posit_x(0.0f)
		,m_posit_y(0.0f)
		,m_gripper_roll(0.0f)
		,m_gripper_pitch(0.0f)
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
		scene->Print(color, "Use sliders to manipulate robot");
		ImGui::SliderFloat("Azimuth", &me->m_azimuth, -360.0f, 360.0f);
		ImGui::SliderFloat("posit_x", &me->m_posit_x, -1.0f, 1.0f);
		ImGui::SliderFloat("posit_y", &me->m_posit_y, -1.0f, 1.0f);

		ImGui::Separator();
		ImGui::Separator();
		ImGui::SliderFloat("eff_roll", &me->m_gripper_roll, -360.0f, 360.0f);
		ImGui::SliderFloat("eff_pitch", &me->m_gripper_pitch, -60.0f, 60.0f);

//me->m_posit_y = 0.25f;

		for (dListNode* node = me->GetFirst(); node; node = node->GetNext()) {
			dSixAxisController* const controller = &node->GetInfo();
			controller->SetTarget (me->m_posit_x, me->m_posit_y, me->m_azimuth * 3.141592f / 180.0f, me->m_gripper_pitch * 3.141592f / 180.0f, me->m_gripper_roll * 3.141592f / 180.0f);
		}
	}

	virtual dSixAxisController* CreateController()
	{
		return (dSixAxisController*)dCustomControllerManager<dSixAxisController>::CreateController();
	}

	dSixAxisController* MakeKukaRobot_IK(DemoEntityManager* const scene, const dVector& origin)
	{
		dSixAxisController* const controller = (dSixAxisController*)CreateController();
		controller->MakeKukaRobot_IK(scene, origin);
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

	dSixAxisController* m_currentController;
	dFloat m_azimuth;
	dFloat m_posit_x;
	dFloat m_posit_y;
	dFloat m_gripper_roll;
	dFloat m_gripper_pitch;
};


void SixAxisManipulators(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	CreateLevelMesh (scene, "flatPlane.ngd", true);
	dSixAxisManager* const robotManager = new dSixAxisManager(scene);

	robotManager->MakeKukaRobot_IK (scene, dVector (0.0f, 0.0f, 0.0f, 1.0f));

	dVector origin(0.0f);
	origin.m_x = -2.0f;
	origin.m_y  = 0.5f;
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}

