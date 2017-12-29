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
	class dKukaServoMotor1: public dCustomRagdollMotor_1dof
	{
		public:
		dKukaServoMotor1(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent, dFloat minAngle, dFloat maxAngle, bool isIK)
			:dCustomRagdollMotor_1dof(pinAndPivotFrame, child, parent)
			,m_isIK(isIK)
		{
			SetJointTorque (1000.0f);
			SetTwistAngle (minAngle, maxAngle);
		}

		void SubmitConstraints(dFloat timestep, int threadIndex)
		{
			if (m_isIK) {
				dCustomRagdollMotor_1dof::SubmitConstraints(timestep, threadIndex);
			} else {
				// for debug purpose
				dMatrix matrix0;
				dMatrix matrix1;

				CalculateGlobalMatrix(matrix0, matrix1);
				dCustomRagdollMotor::SubmitConstraints(timestep, threadIndex);

				// two rows to restrict rotation around around the parent coordinate system
				NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up), &matrix1.m_up[0]);
				NewtonUserJointAddAngularRow(m_joint, CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right), &matrix1.m_right[0]);

				dFloat angle = CalculateAngle(matrix1.m_up, matrix0.m_up, matrix1.m_front);
				if (angle < m_minTwistAngle) {
					dFloat relAngle = angle - m_minTwistAngle;
					NewtonUserJointAddAngularRow(m_joint, -relAngle, &matrix1.m_front[0]);
					NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
				} else if (angle > m_maxTwistAngle) {
					dFloat relAngle = angle - m_maxTwistAngle;
					NewtonUserJointAddAngularRow(m_joint, -relAngle, &matrix1.m_front[0]);

					NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
				}
			}
		}

		void Debug(dDebugDisplay* const debugDisplay) const
		{
			dCustomRagdollMotor_1dof::Debug(debugDisplay);
		}

		bool m_isIK;
	};

/*
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
			dAssert (0);

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
*/

	class dKukaEffector: public dCustomRagdollMotor_EndEffector
	{
		public:
		dKukaEffector(NewtonInverseDynamics* const invDynSolver, void* const invDynNode, const dMatrix& attachmentMatrixInGlobalSpace)
			:dCustomRagdollMotor_EndEffector(invDynSolver, invDynNode, attachmentMatrixInGlobalSpace)
		{
			SetLinearSpeed(0.4f);
			SetMaxLinearFriction (5000.0f);
		}

		dKukaEffector(NewtonBody* const body, const dMatrix& attachmentMatrixInGlobalSpace)
			:dCustomRagdollMotor_EndEffector(body, attachmentMatrixInGlobalSpace)
		{
			SetLinearSpeed(0.4f);
			SetMaxLinearFriction (5000.0f);
		}
	};


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
/*
		dSixAxisRoot(DemoEntityManager* const scene, const dMatrix& location)
			:dSixAxisNode(NULL)
			,m_kinematicSolver(NewtonCreateInverseDynamics(scene->GetNewton()))
			,m_effector(NULL)
			,m_referenceMatrix (dGetIdentityMatrix())
		{
			dFloat size = 1.0f;
			dFloat r = size / 4.0f;
			dFloat h = size / 8.0f;

#if 0
			// add Robot Base Frame locked the root body to the world
			dMatrix baseFrameMatrix(dRollMatrix(90.0f * 3.141592f / 180.0f));
			baseFrameMatrix.m_posit.m_y = h * 0.5f;
			NewtonBody* const baseFrameBody = CreateCylinder(scene, baseFrameMatrix * origin, r, h);
			dCustomHinge* const baseFrameFixHinge = new dCustomHinge(baseFrameMatrix * origin, baseFrameBody, NULL);
			baseFrameFixHinge->EnableLimits(true);
			baseFrameFixHinge->SetLimits(0.0f, 0.0f);
			void* const baseFrameNode = NewtonInverseDynamicsAddRoot(m_kinematicSolver, baseFrameBody);
			NewtonInverseDynamicsAddLoopJoint(m_kinematicSolver, baseFrameFixHinge->GetJoint());

			// add Robot rotating column
			dMatrix rotatingColumnMatrix(dGetIdentityMatrix());
			rotatingColumnMatrix.m_posit.m_y = h + h * 0.5f;
			NewtonBody* const rotatingColumnBody = CreateBox(scene, rotatingColumnMatrix * origin, dVector(r * 0.5f, h, r * 0.75f));
			rotatingColumnMatrix = dRollMatrix(3.141592f * 0.5f) * rotatingColumnMatrix;
			dKukaServoMotor1* const rotatingColumnHinge = new dKukaServoMotor1(rotatingColumnMatrix * origin, rotatingColumnBody, baseFrameBody, -3.141592f * 2.0f, 3.141592f * 2.0f);
			void* const rotatingColumnNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, baseFrameNode, rotatingColumnHinge->GetJoint());

			// add Robot link arm
			dFloat l = size * 0.75f;
			dMatrix linkArmMatrix(dGetIdentityMatrix());
			linkArmMatrix.m_posit = rotatingColumnMatrix.m_posit;
			linkArmMatrix.m_posit.m_y += l * 0.5f;
			NewtonBody* const linkArmBody = CreateBox(scene, linkArmMatrix * origin, dVector(l * 0.125f, l, l * 0.125f));
			linkArmMatrix.m_posit.m_y -= l * 0.5f;
			dKukaServoMotor1* const linkArmJoint = new dKukaServoMotor1(linkArmMatrix * origin, linkArmBody, rotatingColumnBody, -3.141592f * 2.0f, 3.141592f * 2.0f);
			void* const linkArmNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, rotatingColumnNode, linkArmJoint->GetJoint());

			// add Robot arm
			dFloat l1 = l * 0.5f;
			dMatrix armMatrix(linkArmMatrix);
			armMatrix.m_posit.m_y += l;
			armMatrix.m_posit.m_z += l1 * 0.5f;
			dFloat armAngleLimit = 80.0f * 3.141592f / 180.0f;
//			dFloat armAngleLimit = 10.0f * 3.141592f / 180.0f;
			NewtonBody* const armBody = CreateBox(scene, armMatrix * origin, dVector(l * 0.125f, l * 0.125f, l1));
			armMatrix.m_posit.m_z -= l1 * 0.5f;
			dKukaServoMotor1* const armJoint = new dKukaServoMotor1(armMatrix * origin, armBody, linkArmBody, -armAngleLimit, armAngleLimit);
//			dKukaServoMotor1* const armJoint = new dKukaServoMotor1_xxxx(armMatrix * origin, armBody, linkArmBody, -armAngleLimit, armAngleLimit);
			void* const armNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, linkArmNode, armJoint->GetJoint());

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

			dMatrix gripperMatrix(armMatrix);
			gripperMatrix.m_posit.m_z += l1;
			dKukaEffector* const effector = new dKukaEffector(m_kinematicSolver, armNode, gripperMatrix * location);
#endif
			m_effector = new dSixAxisEffector(this, effector);


#else

			// add Robot rotating column
			dMatrix rotatingColumnMatrix(dGetIdentityMatrix());
			rotatingColumnMatrix.m_posit.m_y = h * 0.5f;
			NewtonBody* const rotatingColumnBody = CreateBox(scene, rotatingColumnMatrix * location, dVector(r * 0.5f, h, r * 0.75f));
			rotatingColumnMatrix = dRollMatrix(3.141592f * 0.5f) * rotatingColumnMatrix;
			dCustomHinge* const rotatingColumnHinge = new dCustomHinge(rotatingColumnMatrix * location, rotatingColumnBody, NULL);
			rotatingColumnHinge->EnableLimits(true);
			rotatingColumnHinge->SetLimits(0.0f, 0.0f);
			void* const rotatingColumnNode = NewtonInverseDynamicsAddRoot(m_kinematicSolver, rotatingColumnBody);
			NewtonInverseDynamicsAddLoopJoint(m_kinematicSolver, rotatingColumnHinge->GetJoint());

			// add Robot link arm
			dFloat l = size * 0.75f;
			dMatrix linkArmMatrix(dGetIdentityMatrix());
			linkArmMatrix.m_posit = rotatingColumnMatrix.m_posit;
			linkArmMatrix.m_posit.m_y += l * 0.5f;
			NewtonBody* const linkArmBody = CreateBox(scene, linkArmMatrix * location, dVector(l * 0.125f, l, l * 0.125f));
			linkArmMatrix.m_posit.m_y -= l * 0.5f;
			dFloat lowLimit = - 90.0f * 3.141592f / 180.0f;
			dFloat highLimit =   1.0f * 3.141592f / 180.0f;
			dKukaServoMotor1* const linkArmJoint = new dKukaServoMotor1_xxxx(linkArmMatrix * location, linkArmBody, rotatingColumnBody, lowLimit, highLimit);
			void* const linkArmNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, rotatingColumnNode, linkArmJoint->GetJoint());

			dMatrix gripperMatrix(linkArmMatrix);
			gripperMatrix.m_posit.m_y += l;
//			dKukaEffector* const effector = new dKukaEffector(m_kinematicSolver, linkArmNode, gripperMatrix * location);
			dKukaEffector* const effector = new dKukaEffector(linkArmBody, gripperMatrix * location);
			m_effector = new dSixAxisEffector(this, effector);
#endif

			// save the tip reference point
			m_referenceMatrix = gripperMatrix * location;

			// complete the ik
			NewtonInverseDynamicsEndBuild(m_kinematicSolver);
		}
*/

		dSixAxisRoot(DemoEntityManager* const scene, const dMatrix& location, bool isIK)
			:dSixAxisNode(NULL)
			,m_kinematicSolver(NULL)
			,m_effector(NULL)
			,m_referenceMatrix(dGetIdentityMatrix())
		{
			dFloat size = 1.0f;
			dFloat r = size / 4.0f;
			dFloat h = size / 8.0f;

			dFloat lowLimit = -90.0f * 3.141592f / 180.0f;
			dFloat highLimit = 30.0f * 3.141592f / 180.0f;

			if (isIK) {
				m_kinematicSolver = NewtonCreateInverseDynamics(scene->GetNewton());

				// add Robot rotating column
				dMatrix rotatingColumnMatrix(dGetIdentityMatrix());
				rotatingColumnMatrix.m_posit.m_y = h * 0.5f;
				NewtonBody* const rotatingColumnBody = CreateBox(scene, rotatingColumnMatrix * location, dVector(r * 0.5f, h, r * 0.75f));
				rotatingColumnMatrix = dRollMatrix(3.141592f * 0.5f) * rotatingColumnMatrix;
				dCustomHinge* const rotatingColumnHinge = new dCustomHinge(rotatingColumnMatrix * location, rotatingColumnBody, NULL);
				rotatingColumnHinge->EnableLimits(true);
				rotatingColumnHinge->SetLimits(0.0f, 0.0f);
				void* const rotatingColumnNode = NewtonInverseDynamicsAddRoot(m_kinematicSolver, rotatingColumnBody);
				NewtonInverseDynamicsAddLoopJoint(m_kinematicSolver, rotatingColumnHinge->GetJoint());

				// add Robot link arm
				dFloat l = size * 0.75f;
				dMatrix linkArmMatrix(dGetIdentityMatrix());
				linkArmMatrix.m_posit = rotatingColumnMatrix.m_posit;
				linkArmMatrix.m_posit.m_y += l * 0.5f;
				NewtonBody* const linkArmBody = CreateBox(scene, linkArmMatrix * location, dVector(l * 0.125f, l, l * 0.125f));
				linkArmMatrix.m_posit.m_y -= l * 0.5f;
				dKukaServoMotor1* const linkArmJoint = new dKukaServoMotor1(linkArmMatrix * location, linkArmBody, rotatingColumnBody, lowLimit, highLimit, true);
				void* const linkArmNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, rotatingColumnNode, linkArmJoint->GetJoint());

				dMatrix gripperMatrix(linkArmMatrix);
				gripperMatrix.m_posit.m_y += l;
				dKukaEffector* const effector = new dKukaEffector(m_kinematicSolver, linkArmNode, gripperMatrix * location);
				m_effector = new dSixAxisEffector(this, effector);

				// save the tip reference point
				m_referenceMatrix = gripperMatrix * location;

				// complete the ik
				NewtonInverseDynamicsEndBuild(m_kinematicSolver);
			} 
			else
			{
				// add Robot rotating column
				dMatrix rotatingColumnMatrix(dGetIdentityMatrix());
				rotatingColumnMatrix.m_posit.m_y = h * 0.5f;
				NewtonBody* const rotatingColumnBody = CreateBox(scene, rotatingColumnMatrix * location, dVector(r * 0.5f, h, r * 0.75f));
				rotatingColumnMatrix = dRollMatrix(3.141592f * 0.5f) * rotatingColumnMatrix;
				dCustomHinge* const rotatingColumnHinge = new dCustomHinge(rotatingColumnMatrix * location, rotatingColumnBody, NULL);
				rotatingColumnHinge->EnableLimits(true);
				rotatingColumnHinge->SetLimits(0.0f, 0.0f);

				// add Robot link arm
				dFloat l = size * 0.75f;
				dMatrix linkArmMatrix(dGetIdentityMatrix());
				linkArmMatrix.m_posit = rotatingColumnMatrix.m_posit;
				linkArmMatrix.m_posit.m_y += l * 0.5f;
				NewtonBody* const linkArmBody = CreateBox(scene, linkArmMatrix * location, dVector(l * 0.125f, l, l * 0.125f));
				linkArmMatrix.m_posit.m_y -= l * 0.5f;
				new dKukaServoMotor1(linkArmMatrix * location, linkArmBody, rotatingColumnBody, lowLimit, highLimit, false);

				dMatrix gripperMatrix(linkArmMatrix);
				gripperMatrix.m_posit.m_y += l;
				dKukaEffector* const effector = new dKukaEffector(linkArmBody, gripperMatrix * location);
				m_effector = new dSixAxisEffector(this, effector);

				// save the tip reference point
				m_referenceMatrix = gripperMatrix * location;
			}
		}

		~dSixAxisRoot()
		{
			if(m_kinematicSolver) {
				NewtonInverseDynamicsDestroy (m_kinematicSolver);
			}
		}

		void SetTarget (dFloat z, dFloat y, dFloat azimuth, dFloat pitch, dFloat roll)
		{
			// set base matrix
			m_matrix = dYawMatrix(azimuth) * m_referenceMatrix;

			// set effector matrix
			m_effector->m_matrix = dRollMatrix(roll) * dPitchMatrix(pitch);
			m_effector->m_matrix.m_posit = dVector(0.0f, y, z, 1.0f);

			// calculate global matrices
			UpdateTranform(dGetIdentityMatrix());
		}

		virtual void UpdateEffectors(dFloat timestep)
		{
			dSixAxisNode::UpdateEffectors(timestep);
			if (m_kinematicSolver) {
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
		dMatrix m_referenceMatrix;
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
		if (m_robot && m_robot->m_effector) {
			m_robot->SetTarget(x, y, azimuth, pitch, roll);
		}
	}

	void MakeKukaRobot_IK(DemoEntityManager* const scene, const dMatrix& origin)
	{
		m_robot = new dSixAxisRoot(scene, origin, true);
	}

	void MakeKukaRobot_FD(DemoEntityManager* const scene, const dMatrix& origin)
	{
		m_robot = new dSixAxisRoot(scene, origin, false);
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
		if (m_robot && m_robot->m_effector) {
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

//me->m_posit_x = 1.0f;

		for (dListNode* node = me->GetFirst(); node; node = node->GetNext()) {
			dSixAxisController* const controller = &node->GetInfo();
			controller->SetTarget (me->m_posit_x, me->m_posit_y, me->m_azimuth * 3.141592f / 180.0f, me->m_gripper_pitch * 3.141592f / 180.0f, me->m_gripper_roll * 3.141592f / 180.0f);
		}
	}

	virtual dSixAxisController* CreateController()
	{
		return (dSixAxisController*)dCustomControllerManager<dSixAxisController>::CreateController();
	}

	dSixAxisController* MakeKukaRobot_IK(DemoEntityManager* const scene, const dMatrix& origin)
	{
		dSixAxisController* const controller = (dSixAxisController*)CreateController();
		controller->MakeKukaRobot_IK(scene, origin);
		m_currentController = controller;
		return controller;
	}

	dSixAxisController* MakeKukaRobot_FD(DemoEntityManager* const scene, const dMatrix& origin)
	{
		dSixAxisController* const controller = (dSixAxisController*)CreateController();
		controller->MakeKukaRobot_FD(scene, origin);
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
	dFloat32 m_azimuth;
	dFloat32 m_posit_x;
	dFloat32 m_posit_y;
	dFloat32 m_gripper_roll;
	dFloat32 m_gripper_pitch;
};


void SixAxisManipulators(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	CreateLevelMesh (scene, "flatPlane.ngd", true);
	dSixAxisManager* const robotManager = new dSixAxisManager(scene);

	dMatrix origin(dYawMatrix(0.0f * 3.141693f));
	origin.m_posit.m_z = -0.5f;
	robotManager->MakeKukaRobot_IK (scene, origin);

	origin.m_posit.m_z = 0.5f;
	robotManager->MakeKukaRobot_FD(scene, origin);
	
	origin.m_posit = dVector (-2.0f, 0.5f, 0.0f, 1.0f);
	scene->SetCameraMatrix(dGetIdentityMatrix(), origin.m_posit);
}

