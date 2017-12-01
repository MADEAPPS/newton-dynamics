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
		dKukaServoMotor1(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
			:dCustomHinge(pinAndPivotFrame, child, parent)
			,m_torque(1000.0f)
		{
			EnableLimits(false);
		}

		void SubmitConstraintsFreeDof(dFloat timestep, const dMatrix& matrix0, const dMatrix& matrix1)
		{
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_front[0]);
			dFloat accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
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

		void SubmitConstraints(dFloat timestep, int threadIndex)
		{
			dCustomUniversal::SubmitConstraints(timestep, threadIndex);
/*
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix1.m_front[0]);
			dFloat accel = NewtonUserJointGetRowInverseDynamicsAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, accel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_torque);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_torque);
*/
		}

		dFloat m_torque;
	};


	dSixAxisController()
		:m_kinematicSolver(NULL)
		,m_azimuth(0.0f)
		,m_posit_x(0.0f)
		,m_posit_y(0.0f)
	{
	}

	void Init()
	{
		m_kinematicSolver = NewtonCreateInverseDynamics(GetManager()->GetWorld());
	}

	void DrawHelp(DemoEntityManager* const scene)
	{
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "Use sliders to manipulate robot");
		ImGui::SliderFloat("Azimuth", &m_azimuth, -360.0f, 360.0f);
		ImGui::SliderFloat("posit_x", &m_posit_x, -1.0f, 1.0f);
		ImGui::SliderFloat("posit_y", &m_posit_y, -1.0f, 1.0f);
	}

	void PostUpdate(dFloat timestep, int threadIndex)
	{
	}


	void PreUpdate(dFloat timestep, int threadIndex)
	{
	}

	void MakeKukaRobot(DemoEntityManager* const scene, const dVector& origin)
	{
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
		void* const rootNode = NewtonInverseDynamicsAddRoot(m_kinematicSolver, parentBody);

		// add Robot rotating platform
		dMatrix baseMatrix(dGetIdentityMatrix());
		baseMatrix.m_posit = location.m_posit;
		baseMatrix.m_posit.m_y += 0.125f * 0.5f + 0.11f;
		baseMatrix.m_posit.m_z += 0.125f * 0.5f;
		NewtonBody* const base = CreateBox(scene, baseMatrix, dVector(0.125f, 0.2f, 0.25f));
		dMatrix baseSpin(dGrammSchmidt(dVector(0.0f, 1.0f, 0.0f)));
		baseSpin.m_posit = location.m_posit;
		dKukaServoMotor1* const baseHinge = new dKukaServoMotor1(baseSpin, base, parentBody);
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
		dKukaServoMotor1* const armJoint0 = new dKukaServoMotor1(armHingeMatrix0, armBody0, base);
		void* const armJointNode0 = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, baseHingeNode, armJoint0->GetJoint());

		dMatrix armMatrix1(armMatrix0 * dYawMatrix(3.141592f));
		armMatrix1.m_posit = armMatrix0.m_posit;
		armMatrix1.m_posit.m_y += 0.4f;
		armMatrix1.m_posit.m_x -= 0.05f;
		armMatrix1.m_posit.m_z -= 0.1f;
		NewtonBody* const armBody1 = CreateBox(scene, armMatrix1, dVector(0.05f, 0.1f, 0.5f));
		dMatrix armHingeMatrix1(dGrammSchmidt(dVector(1.0f, 0.0f, 0.0f)));
		armHingeMatrix1.m_posit = armMatrix1.m_posit + armMatrix1.RotateVector(dVector(0.0f, 0.0f, 0.2f));
		dKukaServoMotor1* const armJoint1 = new dKukaServoMotor1(armHingeMatrix1, armBody1, armBody0);
		void* const armJointNode1 = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, armJointNode0, armJoint1->GetJoint());

		// Robot gripper base
		dMatrix gripperMatrix(dYawMatrix(90.0f * 3.141592f / 180.0f) * armMatrix1);
		gripperMatrix.m_posit += gripperMatrix.m_front.Scale(0.325f);
		NewtonBody* const gripperBase = CreateCylinder(scene, gripperMatrix, 0.1f, -0.15f);
		dMatrix gripperEffectMatrix(dGetIdentityMatrix());
		gripperEffectMatrix.m_up = gripperMatrix.m_front;
		gripperEffectMatrix.m_front = dVector(1.0f, 0.0f, 0.0f, 0.0f);
		gripperEffectMatrix.m_right = gripperEffectMatrix.m_front.CrossProduct(gripperEffectMatrix.m_up);
		gripperEffectMatrix.m_posit = gripperMatrix.m_posit - gripperMatrix.m_front.Scale(0.05f);
		dKukaServoMotor2* const gripperJoint = new dKukaServoMotor2(gripperEffectMatrix, gripperBase, armBody1);
//		void* const arm1HingeNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, arm0HingeNode, arm1Hinge->GetJoint());


		NewtonInverseDynamicsEndBuild(m_kinematicSolver);
	}

	private:
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
	dFloat32 m_azimuth;
	dFloat32 m_posit_x;
	dFloat32 m_posit_y;
};

class dSixAxisManager: public dCustomControllerManager<dSixAxisController>
{
	public:
	dSixAxisManager(DemoEntityManager* const scene)
		:dCustomControllerManager<dSixAxisController>(scene->GetNewton(), "sixAxisManipulator")
		,m_currentController(NULL)
	{
		scene->Set2DDisplayRenderFunction(RenderHelpMenu, NULL, this);
	}

	~dSixAxisManager()
	{
	}

	static void RenderHelpMenu(DemoEntityManager* const scene, void* const context)
	{
		dSixAxisManager* const me = (dSixAxisManager*)context;
		if (me->m_currentController) {
			me->m_currentController->DrawHelp(scene);
		}
	}

	virtual dSixAxisController* CreateController()
	{
		dSixAxisController* const controller = (dSixAxisController*)dCustomControllerManager<dSixAxisController>::CreateController();
		controller->Init();
		return controller;
	}

	dSixAxisController* MakeKukaRobot(DemoEntityManager* const scene, const dVector& origin)
	{
		dSixAxisController* const controller = (dSixAxisController*)CreateController();
		controller->MakeKukaRobot(scene, origin);
		m_currentController = controller;
		return controller;
	}

	dSixAxisController* m_currentController;
};


void SixAxisManipulators(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	CreateLevelMesh (scene, "flatPlane.ngd", true);
	dSixAxisManager* const robotManager = new dSixAxisManager(scene);

	robotManager->MakeKukaRobot (scene, dVector (0.0f, 0.0f, 0.0f));
//	robotManager->MakeKukaRobot (scene, dVector (0.0f, 0.0f, 2.0f));
	dVector origin(0.0f);
	origin.m_x = -2.0f;
	origin.m_y  = 0.5f;
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}

