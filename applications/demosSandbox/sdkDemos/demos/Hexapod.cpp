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


class dEffectorTreePostureGenerator: public dEffectorTreeInterface
{
	public:
	dEffectorTreePostureGenerator(dEffectorTreePose* const poseGenerator)
		:dEffectorTreeInterface(poseGenerator->m_rootBody)
		,m_euler(0.0f)
		,m_position(0.0f)
		,m_poseGenerator(poseGenerator)
	{
		m_position.m_w = 1.0f;
	}

	~dEffectorTreePostureGenerator()
	{
		delete m_poseGenerator;
	}

	void SetTarget(dFloat z, dFloat y, dFloat pitch, dFloat yaw, dFloat roll)
	{
		m_position.m_y = y;
		m_position.m_z = z;
		m_euler.m_x = pitch;
		m_euler.m_y = yaw;
		m_euler.m_z = roll;
	}

	virtual void Evaluate(dEffectorPose& output)
	{
		m_poseGenerator->Evaluate(output);

		dQuaternion rotation (dPitchMatrix(m_euler.m_x) * dYawMatrix(m_euler.m_y) * dRollMatrix(m_euler.m_z));
		for (dEffectorPose::dListNode* node = output.GetFirst(); node; node = node->GetNext()) {
			dEffectorTransform& transform = node->GetInfo();
			transform.m_rotation = transform.m_rotation * rotation;
			transform.m_posit = m_position + rotation.RotateVector(transform.m_posit);
		}
	}

	dVector m_euler;
	dVector m_position;
	dEffectorTreePose* m_poseGenerator;
};



class dHaxapodController: public dCustomControllerBase
{
	public:
	class dHexapodMotor: public dCustomRagdollMotor_1dof
	{
		public:
		dHexapodMotor(const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent, dFloat minAngle, dFloat maxAngle)
			:dCustomRagdollMotor_1dof(pinAndPivotFrame, child, parent)
		{
			SetJointTorque (1000.0f);
			SetTwistAngle (minAngle, maxAngle);
		}

		void Debug(dDebugDisplay* const debugDisplay) const
		{
			//dCustomRagdollMotor_1dof::Debug(debugDisplay);
		}
	};

	class dHexapodEffector: public dCustomRagdollMotor_EndEffector
	{
		public:
		dHexapodEffector (NewtonInverseDynamics* const invDynSolver, void* const invDynNode, NewtonBody* const referenceBody, const dMatrix& attachmentMatrixInGlobalSpace)
			:dCustomRagdollMotor_EndEffector (invDynSolver, invDynNode, referenceBody, attachmentMatrixInGlobalSpace)
		{
			SetLinearSpeed(1.0f);
		}
	};

	dHaxapodController()
		:m_animTreeNode(NULL)
		,m_kinematicSolver(NULL)
	{
	}

	~dHaxapodController()
	{
		if (m_kinematicSolver) {
			NewtonInverseDynamicsDestroy(m_kinematicSolver);
		}
	}

	void SetTarget (dFloat x, dFloat y, dFloat pitch, dFloat yaw, dFloat roll)
	{
		m_postureModifier->SetTarget(x, y, pitch, yaw, roll);
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

	NewtonBody* CreateCapsule(DemoEntityManager* const scene, const dMatrix& location, dFloat mass, dFloat inertiaScale, dFloat radius, dFloat height) const
	{
		NewtonWorld* const world = scene->GetNewton();
		int materialID = NewtonMaterialGetDefaultGroupID(world);
		dVector size(radius, height, radius, 0.0f);
		dMatrix align(dYawMatrix(3.141592f * 0.5f));
		NewtonCollision* const collision = CreateConvexCollision(world, align, size, _CAPSULE_PRIMITIVE, 0);
		DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");

		NewtonBody* const body = CreateSimpleSolid(scene, geometry, mass, location, collision, materialID);
		ScaleIntertia(body, inertiaScale);

		geometry->Release();
		NewtonDestroyCollision(collision);
		return body;
	}

	dCustomRagdollMotor_EndEffector* AddLeg(DemoEntityManager* const scene, void* const rootNode, const dMatrix& matrix, dFloat partMass, dFloat limbLenght)
	{
		NewtonBody* const parent = NewtonInverseDynamicsGetBody(m_kinematicSolver, rootNode);

		dFloat inertiaScale = 4.0f;
		// make limb base
		dMatrix baseMatrix(dRollMatrix(3.141592f * 0.5f));
		dMatrix cylinderMatrix(baseMatrix * matrix);
		NewtonBody* const base = CreateCylinder(scene, cylinderMatrix, partMass, inertiaScale, 0.2f, 0.1f);
		dHexapodMotor* const baseHinge = new dHexapodMotor(cylinderMatrix, base, parent, -3.141592f * 0.5f, 3.141592f * 0.5f);
		void* const baseHingeNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, rootNode, baseHinge->GetJoint());

		//make limb forward arm
		dMatrix forwardArmMatrix(dPitchMatrix(-30.0f * 3.141592f / 180.0f));
		dVector forwardArmSize(limbLenght * 0.25f, limbLenght * 0.25f, limbLenght, 0.0f);
		forwardArmMatrix.m_posit += forwardArmMatrix.m_right.Scale(forwardArmSize.m_z * 0.5f);
		NewtonBody* const forwardArm = CreateBox(scene, forwardArmMatrix * matrix, forwardArmSize, partMass, inertiaScale);
		dMatrix forwardArmPivot(forwardArmMatrix);
		forwardArmPivot.m_posit -= forwardArmMatrix.m_right.Scale(forwardArmSize.m_z * 0.5f);
		dHexapodMotor* const forwardArmHinge = new dHexapodMotor(forwardArmPivot * matrix, forwardArm, base, -3.141592f * 0.5f, 3.141592f * 0.5f);
		void* const forwardArmHingeNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, baseHingeNode, forwardArmHinge->GetJoint());

		//make limb forward arm
		dMatrix armMatrix(dPitchMatrix(-90.0f * 3.141592f / 180.0f));
		dFloat armSize = limbLenght * 1.25f;
		armMatrix.m_posit += forwardArmMatrix.m_right.Scale(limbLenght);
		armMatrix.m_posit.m_y -= armSize * 0.5f;
		NewtonBody* const arm = CreateCapsule(scene, armMatrix * matrix, partMass, inertiaScale, armSize * 0.2f, armSize);
		dMatrix armPivot(armMatrix);
		armPivot.m_posit.m_y += armSize * 0.5f;
		dHexapodMotor* const armHinge = new dHexapodMotor(armPivot * matrix, arm, forwardArm, -3.141592f * 0.5f, 3.141592f * 0.5f);
		void* const armHingeNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, forwardArmHingeNode, armHinge->GetJoint());

		dMatrix effectorMatrix(dGetIdentityMatrix());
		effectorMatrix.m_posit = armPivot.m_posit;
		effectorMatrix.m_posit.m_y -= armSize;
		dHexapodEffector* const effector = new dHexapodEffector(m_kinematicSolver, armHingeNode, parent, effectorMatrix * matrix);
		effector->SetAsThreedof();
		effector->SetMaxLinearFriction(partMass * DEMO_GRAVITY * 10.0f);
		effector->SetMaxAngularFriction(partMass * DEMO_GRAVITY * 10.0f);

		return effector;
	}

	void MakeHexapod(DemoEntityManager* const scene, const dMatrix& location)
	{
		dFloat mass = 30.0f;
		// make the kinematic solver
		m_kinematicSolver = NewtonCreateInverseDynamics(scene->GetNewton());

		// make the root body
		dMatrix baseMatrix(dGetIdentityMatrix());
		baseMatrix.m_posit.m_y += 0.35f;
		dVector size (1.3f, 0.31f, 0.5f, 0.0f);
		NewtonBody* const hexaBody = CreateBox(scene, baseMatrix * location, size, mass, 1.0f);
		void* const hexaBodyNode = NewtonInverseDynamicsAddRoot(m_kinematicSolver, hexaBody);

		int legEffectorCount = 0;
		dCustomRagdollMotor_EndEffector* legEffectors[32];

		baseMatrix.m_posit.m_y -= 0.06f;
		// make the hexapod six limbs
		for (int i = 0; i < 3; i ++) {
			dMatrix rightLocation (baseMatrix);
			rightLocation.m_posit += rightLocation.m_right.Scale (size.m_z * 0.65f);
			rightLocation.m_posit += rightLocation.m_front.Scale (size.m_x * 0.3f - size.m_x * i / 3.0f);
			legEffectors[legEffectorCount] = AddLeg (scene, hexaBodyNode, rightLocation * location, mass * 0.1f, 0.3f);
			legEffectorCount ++;

			dMatrix similarTransform (dGetIdentityMatrix());
			similarTransform.m_posit.m_x = rightLocation.m_posit.m_x;
			similarTransform.m_posit.m_y = rightLocation.m_posit.m_y;
			dMatrix leftLocation (rightLocation * similarTransform.Inverse() * dYawMatrix(3.141592f) * similarTransform);
			legEffectors[legEffectorCount] = AddLeg (scene, hexaBodyNode, leftLocation * location, mass * 0.1f, 0.3f);
			legEffectorCount ++;
		}

		// finalize inverse dynamics solver
		NewtonInverseDynamicsEndBuild(m_kinematicSolver);
		
		// create a fix pose frame generator
		dEffectorTreeFixPose* const idlePose = new dEffectorTreeFixPose(hexaBody);
		m_postureModifier = new dEffectorTreePostureGenerator(idlePose);
		m_animTreeNode = new dEffectorTreeRoot(hexaBody, m_postureModifier);

		dMatrix rootMatrix;
		NewtonBodyGetMatrix (hexaBody, &rootMatrix[0][0]);
		rootMatrix = rootMatrix.Inverse();
		for (int i = 0; i < legEffectorCount; i++) {
			dEffectorTreeInterface::dEffectorTransform frame;
			dCustomRagdollMotor_EndEffector* const effector = legEffectors[i];
			dMatrix effectorMatrix(effector->GetBodyMatrix());

			dMatrix poseMatrix(effectorMatrix * rootMatrix);
			
			frame.m_effector = effector;
			frame.m_posit = poseMatrix.m_posit;
			frame.m_rotation = dQuaternion(poseMatrix);

			idlePose->GetPose().Append(frame);
			m_animTreeNode->GetPose().Append(frame);
		}
	}

	void PostUpdate(dFloat timestep, int threadIndex) 
	{
	}

	void PreUpdate(dFloat timestep, int threadIndex)
	{
		m_animTreeNode->Update();
		NewtonInverseDynamicsUpdate(m_kinematicSolver, timestep, threadIndex);
	}

	void Debug(dCustomJoint::dDebugDisplay* const debugContext) const
	{
		const dEffectorTreeInterface::dEffectorPose& pose = m_animTreeNode->GetPose();
		for (dEffectorTreeInterface::dEffectorPose::dListNode* node = pose.GetFirst(); node; node = node->GetNext()) {
			dCustomRagdollMotor_EndEffector* const effector = node->GetInfo().m_effector;
			effector->Debug(debugContext);
		}
	}

	dEffectorTreeRoot* m_animTreeNode;
	NewtonInverseDynamics* m_kinematicSolver;
	dEffectorTreePostureGenerator* m_postureModifier; // do not delete 
};

class dHexapodManager: public dCustomControllerManager<dHaxapodController>
{
	public:
	dHexapodManager(DemoEntityManager* const scene)
		:dCustomControllerManager<dHaxapodController>(scene->GetNewton(), "sixAxisManipulator")
		,m_currentController(NULL)
		,m_yaw(0.0f)
		,m_roll(0.0f)
		,m_pitch(0.0f)
		,m_posit_x(0.0f)
		,m_posit_y(0.0f)
	{
		scene->Set2DDisplayRenderFunction(RenderHelpMenu, NULL, this);
	}

	~dHexapodManager()
	{
	}

	static void RenderHelpMenu(DemoEntityManager* const scene, void* const context)
	{
		dHexapodManager* const me = (dHexapodManager*)context;

		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "Hexapod controller");

		ImGui::Separator();
//		scene->Print(color, "control mode");
//		ImGui::RadioButton("rotate body", &xxxx, 1);
//		ImGui::RadioButton("translate body", &xxxx, 0);
//		ImGui::Separator();

		ImGui::SliderFloat("pitch", &me->m_pitch, -10.0f, 10.0f);
		ImGui::SliderFloat("yaw", &me->m_yaw, -10.0f, 10.0f);
		ImGui::SliderFloat("roll", &me->m_roll, -10.0f, 10.0f);
		ImGui::SliderFloat("posit_x", &me->m_posit_x, -0.1f, 0.1f);
		ImGui::SliderFloat("posit_y", &me->m_posit_y, -0.4f, 0.4f);

		for (dListNode* node = me->GetFirst(); node; node = node->GetNext()) {
			dHaxapodController* const controller = &node->GetInfo();
			controller->SetTarget (me->m_posit_x, me->m_posit_y, me->m_pitch * 3.141592f / 180.0f, me->m_yaw * 3.141592f / 180.0f, me->m_roll * 3.141592f / 180.0f);
		}
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
	dFloat32 m_yaw;
	dFloat32 m_roll;
	dFloat32 m_pitch;
	dFloat32 m_posit_x;
	dFloat32 m_posit_y;
};


void Hexapod(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	CreateLevelMesh (scene, "flatPlane.ngd", true);
	dHexapodManager* const robotManager = new dHexapodManager(scene);

	dMatrix location (dGetIdentityMatrix());
	location.m_posit.m_y = 1.0f;
	robotManager->MakeHexapod (scene, location);

	dVector origin(0.0f);
	origin.m_x = -4.0f;
	origin.m_y  = 1.5f;
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}

