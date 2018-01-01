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


class dEffectorTreeInputModifier: public dEffectorTreeInterface
{
	public:
	dEffectorTreeInputModifier(dEffectorTreePose* const poseGenerator)
		:dEffectorTreeInterface(poseGenerator->m_rootBody)
		,m_poseGenerator(poseGenerator)
		,m_euler(0.0f)
		,m_position(0.0f)
	{
		dAssert (0);
		m_position.m_w = 1.0f;
	}

	~dEffectorTreeInputModifier()
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

		dMatrix modifierMatrix (dPitchMatrix(m_euler.m_x) * dYawMatrix(m_euler.m_y) * dRollMatrix(m_euler.m_z));
		modifierMatrix.m_posit = m_position;

		dEffectorTransform& rootTransform = output.GetFirst()->GetInfo();

//		rootTransform.m_rotation = dQuaternion (modifierMatrix) * rootTransform.m_rotation;
//		rootTransform.m_posit += rootTransform.m_rotation.RotateVector(m_position);

		dMatrix rootMatrix (rootTransform.m_rotation, rootTransform.m_posit);
		dMatrix matrix (rootMatrix.Inverse() * modifierMatrix * rootMatrix);
		dQuaternion rotation (matrix);
		for (dEffectorPose::dListNode* node = output.GetFirst()->GetNext(); node; node = node->GetNext()) {
			dEffectorTransform& transform = output.GetFirst()->GetInfo();
			transform.m_rotation = transform.m_rotation * rotation;
			transform.m_posit = matrix.m_posit + rotation.RotateVector(transform.m_posit);
		}
	}

	dEffectorTreePose* m_poseGenerator;
	dVector m_euler;
	dVector m_position;
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
			SetLinearSpeed(0.4f);
		}
	};

	class dHexapodNode
	{
		public:
		dHexapodNode(dHexapodNode* const parent)
			:m_parent(parent)
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


		void Debug(dCustomJoint::dDebugDisplay* const debugContext) const
		{
			if (m_effector) {
				m_effector->Debug(debugContext);
				for (dList<dHexapodNode*>::dListNode* nodePtr = m_children.GetFirst(); nodePtr; nodePtr = nodePtr->GetNext()) {
					dHexapodNode* const node = nodePtr->GetInfo();
					node->Debug(debugContext);
				}
			}
		}

		dHexapodNode* m_parent;
		dHexapodEffector* m_effector;
		dList<dHexapodNode*> m_children;
	};

	class dHexapodLimb: public dHexapodNode
	{
		public:
		dHexapodLimb(dHexapodNode* const parent, dHexapodEffector* const effector)
			:dHexapodNode(parent)
		{
			m_effector = effector;
			if (parent) {
				parent->m_children.Append (this);
			}
		}
	};

	class dHexapodRoot: public dHexapodNode
	{
		public:
		dHexapodRoot(DemoEntityManager* const scene, const dMatrix& location)
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
			NewtonBody* const hexaBody = CreateBox(scene, baseMatrix * location, size, mass, 1.0f);
dAssert (0);
/*
			void* const hexaBodyNode = NewtonInverseDynamicsAddRoot(m_kinematicSolver, hexaBody);
			m_effector = new dHexapodEffector(m_kinematicSolver, hexaBodyNode, baseMatrix * location);
			m_effector->SetAsSixdof();
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
				AddLimb (scene, hexaBodyNode, rightLocation * location, mass * 0.1f, 0.3f);

				dMatrix similarTransform (dGetIdentityMatrix());
				similarTransform.m_posit.m_x = rightLocation.m_posit.m_x;
				similarTransform.m_posit.m_y = rightLocation.m_posit.m_y;
				dMatrix leftLocation (rightLocation * similarTransform.Inverse() * dYawMatrix(3.141592f) * similarTransform);
				AddLimb (scene, hexaBodyNode, leftLocation * location, mass * 0.1f, 0.3f);
}
			}
*/
			// finalize inverse dynamics solver
			NewtonInverseDynamicsEndBuild(m_kinematicSolver);
		}

		void AddLimb(DemoEntityManager* const scene, void* const rootNode, const dMatrix& matrix, dFloat partMass, dFloat limbLenght)
		{
			dAssert (0);
/*
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
			dHexapodEffector* const effector = new dHexapodEffector(m_kinematicSolver, armHingeNode, effectorMatrix * matrix);
			effector->SetAsThreedof();
			effector->SetMaxLinearFriction(partMass * DEMO_GRAVITY * 10.0f);
			effector->SetMaxAngularFriction(partMass * DEMO_GRAVITY * 10.0f);
			new dHexapodLimb (this, effector);
*/
		}

		~dHexapodRoot()
		{
			if (m_kinematicSolver) {
				NewtonInverseDynamicsDestroy (m_kinematicSolver);
			}
		}

		virtual void UpdateEffectors(dFloat timestep)
		{
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
		,m_animTreeNode(NULL)
		,m_inputModifier(NULL)
	{
	}

	~dHaxapodController()
	{
		if (m_robot) {
			delete m_robot;
			delete m_animTreeNode;
		}
	}

	void SetTarget (dFloat x, dFloat y, dFloat pitch, dFloat yaw, dFloat roll)
	{
		if (m_inputModifier) {
			m_inputModifier->SetTarget(x, y, pitch, yaw, roll);
		}
	}

	void MakeHexapod(DemoEntityManager* const scene, const dMatrix& location)
	{
		m_robot = new dHexapodRoot(scene, location);

		int stack = 1;
		int nodesCount = 0;
		dHexapodNode* effectors[128];

		// determine the number of effectors;
		dHexapodNode* pool[32];
		pool[0] = m_robot;
		while (stack) {
			stack --;
			dHexapodNode* const node = pool[stack];
			if (node->m_effector) {
				effectors[nodesCount] = node;
				nodesCount ++;
			}
			for (dList<dHexapodNode*>::dListNode* ptr = node->m_children.GetFirst(); ptr; ptr = ptr->GetNext()) {
				pool[stack] = ptr->GetInfo();
				stack ++;
			}
		}
		dAssert (0);
/*
		// create a fix pose frame generator
		dEffectorTreeFixPose* const fixPose = new dEffectorTreeFixPose(m_robot->m_effector);
		m_inputModifier = new dEffectorTreeInputModifier(fixPose);
		m_animTreeNode = new dEffectorTreeRoot(m_inputModifier);

		dMatrix rootMatrix(m_robot->m_effector->GetBodyMatrix().Inverse());
		for (int i = 0; i < nodesCount; i++) {
			dEffectorTreeInterface::dEffectorTransform frame;
			dHexapodNode* const node = effectors[i];

			dMatrix effectorMatrix(node->m_effector->GetBodyMatrix());
			dMatrix poseMatrix(effectorMatrix * rootMatrix);
			
			frame.m_effector = node->m_effector;
			frame.m_posit = poseMatrix.m_posit;
			frame.m_rotation = dQuaternion(poseMatrix);

			fixPose->GetPose().Append(frame);
			m_animTreeNode->GetPose().Append(frame);
		}
*/
	}

	void PostUpdate(dFloat timestep, int threadIndex) 
	{
	}

	void PreUpdate(dFloat timestep, int threadIndex)
	{
		if (m_robot) {
			m_animTreeNode->Update();
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
	dEffectorTreeRoot* m_animTreeNode;
	dEffectorTreeInputModifier* m_inputModifier;
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
		ImGui::SliderFloat("posit_x", &me->m_posit_x, -0.5f, 0.5f);
		ImGui::SliderFloat("posit_y", &me->m_posit_y, -0.5f, 0.5f);

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

