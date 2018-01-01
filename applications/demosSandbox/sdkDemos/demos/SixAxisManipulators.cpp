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
	class dEffectorTreeInputModifier: public dEffectorTreeInterface
	{
		public:
		dEffectorTreeInputModifier(dEffectorTreePose* const poseGenerator, const dVector& azimuthAxis, const dVector& planeAxis)
			:dEffectorTreeInterface(poseGenerator->m_rootBody)
			,m_poseGenerator(poseGenerator)
			,m_euler(0.0f)
			,m_position(0.0f)
			,m_planeAxis(planeAxis)
			,m_azimuthAxis(azimuthAxis)
		{
		}

		~dEffectorTreeInputModifier()
		{
			delete m_poseGenerator;
		}

		void SetTarget(dFloat z, dFloat y, dFloat azimuth, dFloat pitch, dFloat roll)
		{
			m_position.m_y = y;
			m_position.m_z = z;

//m_position.m_y = -0.5f;
			m_euler.m_x = pitch;
			m_euler.m_y = azimuth;
			m_euler.m_z = roll;
		}

		virtual void Evaluate(dEffectorPose& output)
		{
			m_poseGenerator->Evaluate(output);

			dEffectorTransform& transform = output.GetFirst()->GetInfo();
			dQuaternion yawRotation (m_azimuthAxis, m_euler.m_y);

			transform.m_posit += m_azimuthAxis.Scale (m_position.m_y);
			transform.m_posit += m_planeAxis.Scale (m_position.m_z);

			transform.m_rotation = transform.m_rotation * yawRotation;
			transform.m_posit = yawRotation.RotateVector(transform.m_posit);
			transform.m_posit.m_w = 1.0f;
		}

		dEffectorTreePose* m_poseGenerator;
		dVector m_euler;
		dVector m_position;
		dVector m_planeAxis;
		dVector m_azimuthAxis;
	};


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
				dCustomRagdollMotor::SubmitConstraints(timestep, threadIndex);

				// two rows to restrict rotation around around the parent coordinate system
				CalculateGlobalMatrix(matrix0, matrix1);
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
		dKukaEffector(NewtonInverseDynamics* const invDynSolver, void* const invDynNode, NewtonBody* const referenceBody, const dMatrix& attachmentMatrixInGlobalSpace)
			:dCustomRagdollMotor_EndEffector(invDynSolver, invDynNode, referenceBody, attachmentMatrixInGlobalSpace)
		{
			SetLinearSpeed(0.4f);
			SetMaxLinearFriction (5000.0f);
		}

		dKukaEffector(NewtonBody* const body, NewtonBody* const referenceBody, const dMatrix& attachmentMatrixInGlobalSpace)
			:dCustomRagdollMotor_EndEffector(body, referenceBody, attachmentMatrixInGlobalSpace)
		{
			SetLinearSpeed(2.0f);
			SetMaxLinearFriction (5000.0f);
		}
	};

	dSixAxisController()
		:m_kinematicSolver(NULL)
		,m_animTreeNode(NULL)
		,m_inputModifier(NULL)
	{
	}

	~dSixAxisController()
	{
		if (m_kinematicSolver) {
			NewtonInverseDynamicsDestroy (m_kinematicSolver);
		}

		if (m_animTreeNode) {
			delete m_animTreeNode;
		}
	}


	void SetTarget (dFloat x, dFloat y, dFloat azimuth, dFloat pitch, dFloat roll)
	{
		m_inputModifier->SetTarget(x, y, azimuth, pitch, roll);
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

	NewtonBody* CreateCylinder(DemoEntityManager* const scene, const dMatrix& location, dFloat mass, dFloat radius, dFloat height) const
	{
		NewtonWorld* const world = scene->GetNewton();
		int materialID = NewtonMaterialGetDefaultGroupID(world);
		dVector size(radius, height, radius, 0.0f);
		NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _CYLINDER_PRIMITIVE, 0);
		DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");

		NewtonBody* const body = CreateSimpleSolid(scene, geometry, mass, location, collision, materialID);
		ScaleIntertia(body, 10.0f);

		geometry->Release();
		NewtonDestroyCollision(collision);
		return body;
	}


	void MakeKukaRobot_IK(DemoEntityManager* const scene, const dMatrix& location)
	{
		dFloat size = 1.0f;
		dFloat r = size / 4.0f;
		dFloat h = size / 8.0f;

		dFloat lowLimit = -88.0f * 3.141592f / 180.0f;
		dFloat highLimit = 60.0f * 3.141592f / 180.0f;

		m_kinematicSolver = NewtonCreateInverseDynamics(scene->GetNewton());

		// add Robot Base Frame locked the root body to the world
		dMatrix baseFrameMatrix(dRollMatrix(90.0f * 3.141592f / 180.0f));
		baseFrameMatrix.m_posit.m_y = h * 0.5f;
		NewtonBody* const baseFrameBody = CreateCylinder(scene, baseFrameMatrix * location, 20.0f, r, h);
		void* const baseFrameNode = NewtonInverseDynamicsAddRoot(m_kinematicSolver, baseFrameBody);
		//NewtonBodySetMassMatrix(baseFrameBody, 0.0f, 0.0f, 0.0f, 0.0f);

		// add Robot rotating column
		dMatrix rotatingColumnMatrix(dGetIdentityMatrix());
		rotatingColumnMatrix.m_posit.m_y = h + h * 0.5f;
		NewtonBody* const rotatingColumnBody = CreateBox(scene, rotatingColumnMatrix * location, dVector(r * 0.5f, h, r * 0.75f));
		rotatingColumnMatrix = dRollMatrix(3.141592f * 0.5f) * rotatingColumnMatrix;
		dKukaServoMotor1* const rotatingColumnHinge = new dKukaServoMotor1(rotatingColumnMatrix * location, rotatingColumnBody, baseFrameBody, -2.0f * 3.141592f, 2.0f * 3.141592f, true);
		void* const rotatingColumnNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, baseFrameNode, rotatingColumnHinge->GetJoint());

		// add Robot link arm
		dFloat l = size * 0.75f;
		dMatrix linkArmMatrix(dGetIdentityMatrix());
		linkArmMatrix.m_posit = rotatingColumnMatrix.m_posit;
		linkArmMatrix.m_posit.m_y += l * 0.5f;
		NewtonBody* const linkArmBody = CreateBox(scene, linkArmMatrix * location, dVector(l * 0.125f, l, l * 0.125f));
		linkArmMatrix.m_posit.m_y -= l * 0.5f;
		dKukaServoMotor1* const linkArmJoint = new dKukaServoMotor1(linkArmMatrix * location, linkArmBody, rotatingColumnBody, -0.5f * 3.141592f, 0.5f * 3.141592f, true);
		void* const linkArmNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, rotatingColumnNode, linkArmJoint->GetJoint());

		// add Robot arm
		dFloat l1 = l * 0.5f;
		dMatrix armMatrix(linkArmMatrix);
		armMatrix.m_posit.m_y += l;
		armMatrix.m_posit.m_z += l1 * 0.5f;
		NewtonBody* const armBody = CreateBox(scene, armMatrix * location, dVector(l * 0.125f, l * 0.125f, l1));
		armMatrix.m_posit.m_z -= l1 * 0.5f;
		dKukaServoMotor1* const armJoint = new dKukaServoMotor1(armMatrix * location, armBody, linkArmBody, lowLimit, highLimit, true);
		void* const armNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, linkArmNode, armJoint->GetJoint());

		// add effector
		dMatrix gripperMatrix(armMatrix);
		gripperMatrix.m_posit.m_z += l1;
		dKukaEffector* const effector = new dKukaEffector(m_kinematicSolver, armNode, baseFrameBody, gripperMatrix * location);
		//m_effector = new dSixAxisEffector(this, effector);
		effector->SetAsThreedof();
		effector->SetMaxLinearFriction(1000.0f);

/*
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
*/


		// create the Animation tree for manipulation 
		dMatrix effectorLocalMatrix(gripperMatrix * baseFrameMatrix.Inverse());
		dVector upAxis(baseFrameMatrix.UnrotateVector(dVector(0.0f, 1.0f, 0.0f, 1.0f)));
		dVector planeAxis(baseFrameMatrix.UnrotateVector(dVector(0.0f, 0.0f, 1.0f, 1.0f)));

		dEffectorTreeFixPose* const fixPose = new dEffectorTreeFixPose(baseFrameBody);
		m_inputModifier = new dEffectorTreeInputModifier(fixPose, upAxis, planeAxis);
		m_animTreeNode = new dEffectorTreeRoot(baseFrameBody, m_inputModifier);

		// set base pose
		dEffectorTreeInterface::dEffectorTransform frame;
		frame.m_effector = effector;
		frame.m_posit = effectorLocalMatrix.m_posit;
		frame.m_rotation = dQuaternion(effectorLocalMatrix);

		fixPose->GetPose().Append(frame);
		m_animTreeNode->GetPose().Append(frame);

		NewtonInverseDynamicsEndBuild(m_kinematicSolver);
	}

	void MakeKukaRobot_FD(DemoEntityManager* const scene, const dMatrix& location)
	{
		dFloat size = 1.0f;
		dFloat r = size / 4.0f;
		dFloat h = size / 8.0f;

		dFloat lowLimit = -88.0f * 3.141592f / 180.0f;
		dFloat highLimit = 60.0f * 3.141592f / 180.0f;

		// add Robot Base Frame locked the root body to the world
		dMatrix baseFrameMatrix(dRollMatrix(90.0f * 3.141592f / 180.0f));
		baseFrameMatrix.m_posit.m_y = h * 0.5f;
		NewtonBody* const baseFrameBody = CreateCylinder(scene, baseFrameMatrix * location, 20.0f, r, h);
		NewtonBodySetMassMatrix(baseFrameBody, 0.0f, 0.0f, 0.0f, 0.0f);
		
		// add Robot rotating column
		dMatrix rotatingColumnMatrix(dGetIdentityMatrix());
		rotatingColumnMatrix.m_posit.m_y = h + h * 0.5f;
		NewtonBody* const rotatingColumnBody = CreateBox(scene, rotatingColumnMatrix * location, dVector(r * 0.5f, h, r * 0.75f));
		rotatingColumnMatrix = dRollMatrix(3.141592f * 0.5f) * rotatingColumnMatrix;
		new dKukaServoMotor1(rotatingColumnMatrix * location, rotatingColumnBody, baseFrameBody, -2.0f * 3.141592f, 2.0f * 3.141592f, false);

		// add Robot link arm
		dFloat l = size * 0.75f;
		dMatrix linkArmMatrix(dGetIdentityMatrix());
		linkArmMatrix.m_posit = rotatingColumnMatrix.m_posit;
		linkArmMatrix.m_posit.m_y += l * 0.5f;
		NewtonBody* const linkArmBody = CreateBox(scene, linkArmMatrix * location, dVector(l * 0.125f, l, l * 0.125f));
		linkArmMatrix.m_posit.m_y -= l * 0.5f;
		new dKukaServoMotor1(linkArmMatrix * location, linkArmBody, rotatingColumnBody, -0.5f * 3.141592f, 0.5f * 3.141592f, false);

		// add Robot arm
		dFloat l1 = l * 0.5f;
		dMatrix armMatrix(linkArmMatrix);
		armMatrix.m_posit.m_y += l;
		armMatrix.m_posit.m_z += l1 * 0.5f;
		NewtonBody* const armBody = CreateBox(scene, armMatrix * location, dVector(l * 0.125f, l * 0.125f, l1));
		armMatrix.m_posit.m_z -= l1 * 0.5f;
		new dKukaServoMotor1(armMatrix * location, armBody, linkArmBody, lowLimit, highLimit, false);

		// add effector
		dMatrix gripperMatrix(armMatrix);
		gripperMatrix.m_posit.m_z += l1;
		dKukaEffector* const effector = new dKukaEffector(armBody, baseFrameBody, gripperMatrix * location);
		effector->SetAsThreedof();
		effector->SetMaxLinearFriction(1000.0f);

		// add effector node 
		dMatrix effectorLocalMatrix (gripperMatrix * baseFrameMatrix.Inverse());
		dVector upAxis (baseFrameMatrix.UnrotateVector(dVector (0.0f, 1.0f, 0.0f, 1.0f)));
		dVector planeAxis (baseFrameMatrix.UnrotateVector(dVector (0.0f, 0.0f, 1.0f, 1.0f)));

		dEffectorTreeFixPose* const fixPose = new dEffectorTreeFixPose(baseFrameBody);
		m_inputModifier = new dEffectorTreeInputModifier(fixPose, upAxis, planeAxis);
		m_animTreeNode = new dEffectorTreeRoot(baseFrameBody, m_inputModifier);

		// set base pose
		dEffectorTreeInterface::dEffectorTransform frame;
		frame.m_effector = effector;
		frame.m_posit = effectorLocalMatrix.m_posit;
		frame.m_rotation = dQuaternion(effectorLocalMatrix);

		fixPose->GetPose().Append(frame);
		m_animTreeNode->GetPose().Append(frame);

//dMatrix xxxxxx (baseFrameMatrix * location * dYawMatrix(0.0f * 3.141592f));
//xxxxxx.m_posit.m_y += 2.0f;
//NewtonBodySetMatrixRecursive(baseFrameBody, &xxxxxx[0][0]);

	}

	void PostUpdate(dFloat timestep, int threadIndex) 
	{
	}

	void PreUpdate(dFloat timestep, int threadIndex)
	{
		m_animTreeNode->Update();
		if (m_kinematicSolver) {
			NewtonInverseDynamicsUpdate(m_kinematicSolver, timestep, 0);
		}
	}

	void Debug(dCustomJoint::dDebugDisplay* const debugContext) const
	{
		const dEffectorTreeInterface::dEffectorPose& pose = m_animTreeNode->GetPose();
		for (dEffectorTreeInterface::dEffectorPose::dListNode* node = pose.GetFirst(); node; node = node->GetNext()) {
			dCustomRagdollMotor_EndEffector* const effector = node->GetInfo().m_effector;
			effector->Debug(debugContext);
		}
	}

	NewtonInverseDynamics* m_kinematicSolver;
	dEffectorTreeRoot* m_animTreeNode;
	dEffectorTreeInputModifier* m_inputModifier;
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
		ImGui::SliderFloat("Azimuth", &me->m_azimuth, -150.0f, 150.0f);
		ImGui::SliderFloat("posit_x", &me->m_posit_x, -1.0f, 1.0f);
		ImGui::SliderFloat("posit_y", &me->m_posit_y, -1.0f, 1.0f);

//		ImGui::Separator();
//		ImGui::Separator();
//		ImGui::SliderFloat("eff_roll", &me->m_gripper_roll, -360.0f, 360.0f);
//		ImGui::SliderFloat("eff_pitch", &me->m_gripper_pitch, -60.0f, 60.0f);

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
	origin.m_posit.m_z = -0.75f;
	robotManager->MakeKukaRobot_IK (scene, origin);

	origin.m_posit.m_z = 0.75f;
	robotManager->MakeKukaRobot_FD(scene, origin);
	
	origin.m_posit = dVector (-3.0f, 0.5f, 0.0f, 1.0f);
	scene->SetCameraMatrix(dGetIdentityMatrix(), origin.m_posit);
}

