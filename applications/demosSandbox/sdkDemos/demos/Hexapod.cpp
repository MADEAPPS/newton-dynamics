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
#include "HeightFieldPrimitive.h"


class dEffectorWalkPoseGenerator: public dEffectorTreeFixPose
{
	public:
	dEffectorWalkPoseGenerator(NewtonBody* const rootBody)
		:dEffectorTreeFixPose(rootBody)
		,m_acc(0.0f)
		,m_amplitud_x(0.35f)
		,m_amplitud_y(0.1f)
		,m_period (1.0f)
		,cycle()
	{
		m_sequence[0] = 0;
		m_sequence[3] = 0;
		m_sequence[4] = 0;
		m_sequence[1] = 1;
		m_sequence[2] = 1;
		m_sequence[5] = 1;

		// make left walk cycle
		const int size = 11;
		const int splite = (size - 1) / 2 - 1;
		dFloat64 knots[size];
		dBigVector leftControlPoints[size + 2];
		for (int i = 0; i < size; i ++) {
			knots[i] = dFloat (i) / (size - 1);
		}
		memset (leftControlPoints, 0, sizeof (leftControlPoints));

		dFloat x = -m_amplitud_x / 2.0f;
		dFloat step_x = m_amplitud_x / splite;
		for (int i = 0; i <= splite; i ++) {
			leftControlPoints[i + 1].m_y = m_amplitud_y * dSin (dPi * dFloat (i) / splite);
			leftControlPoints[i + 1].m_x = x;
			x += step_x;
		}

		x = m_amplitud_x / 2.0f;
		step_x = -m_amplitud_x / (size - splite - 1);
		for (int i = splite; i < size; i++) {
			leftControlPoints[i + 1].m_x = x;
			x += step_x;
		}
		leftControlPoints[0].m_x = leftControlPoints[1].m_x;
		leftControlPoints[size + 1].m_x = leftControlPoints[size].m_x;

//		cycle.CreateFromKnotVectorAndControlPoints(3, size, knots, leftControlPoints);
		cycle.CreateFromKnotVectorAndControlPoints(1, size, knots, &leftControlPoints[1]);
	}

	virtual void Evaluate(dEffectorPose& output, dFloat timestep)
	{
		dEffectorTreeFixPose::Evaluate(output, timestep);

		dFloat param = m_acc / m_period;
		dBigVector left (cycle.CurvePoint(param));
		dBigVector right (cycle.CurvePoint(dMod (param + 0.5f, 1.0f)));

		dFloat high[2];
		dFloat stride[2];
		high[0] = dFloat (left.m_y);
		high[1] = dFloat (right.m_y);
		stride[0] = dFloat(left.m_x);
		stride[1] = dFloat(right.m_x);

		int index = 0;
		for (dEffectorPose::dListNode* node = output.GetFirst(); node; node = node->GetNext()) {
			dEffectorTransform& transform = node->GetInfo();
			transform.m_posit.m_y += high[m_sequence[index]];
			transform.m_posit.m_x += stride[m_sequence[index]];
			index ++;
		}
		m_acc = dMod(m_acc + timestep, m_period);
	}

	dFloat m_acc;
	dFloat m_period;
	dFloat m_amplitud_x;
	dFloat m_amplitud_y;
	dBezierSpline cycle;
	int m_sequence[6]; 
};

class dEffectorTreePostureGenerator: public dEffectorTreeInterface
{
	public:
	dEffectorTreePostureGenerator(dEffectorTreeInterface* const poseGenerator)
		:dEffectorTreeInterface(poseGenerator->GetRootBody())
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

	void SetTarget(dFloat z, dFloat y, dFloat roll, dFloat yaw, dFloat pitch)
	{
		m_position.m_y = y;
		m_position.m_z = z;
		m_euler.m_x = pitch;
		m_euler.m_y = yaw;
		m_euler.m_z = roll;
	}

	virtual void Evaluate(dEffectorPose& output, dFloat timestep)
	{
		m_poseGenerator->Evaluate(output, timestep);

		dQuaternion rotation (dPitchMatrix(m_euler.m_x) * dYawMatrix(m_euler.m_y) * dRollMatrix(m_euler.m_z));
		for (dEffectorPose::dListNode* node = output.GetFirst(); node; node = node->GetNext()) {
			dEffectorTransform& transform = node->GetInfo();
			transform.m_rotation = transform.m_rotation * rotation;
			transform.m_posit = m_position + rotation.RotateVector(transform.m_posit);
		}
	}

	dVector m_euler;
	dVector m_position;
	dEffectorTreeInterface* m_poseGenerator;
};


class dEffectorBlendIdleWalk: public dEffectorTreeTwoWayBlender
{
	public:
	dEffectorBlendIdleWalk(NewtonBody* const rootBody, dEffectorTreeInterface* const node0, dEffectorTreeInterface* const node1)
		:dEffectorTreeTwoWayBlender(rootBody, node0, node1)
	{
	}

	void SetBlendFactor (dFloat blend)
	{
		m_param = blend;
	}
};

class dHexapodController: public dCustomControllerBase
{
	public:
	class dHexapodEffector: public dCustomInverseDynamicsEffector
	{
		public:
		dHexapodEffector (NewtonInverseDynamics* const invDynSolver, void* const invDynNode, NewtonBody* const referenceBody, const dMatrix& attachmentMatrixInGlobalSpace)
			:dCustomInverseDynamicsEffector (invDynSolver, invDynNode, referenceBody, attachmentMatrixInGlobalSpace)
		{
			SetLinearSpeed(1.0f);
		}
	};

	dHexapodController()
		:m_animTreeNode(NULL)
		,m_kinematicSolver(NULL)
		,m_walkIdleBlender(NULL)
		,m_postureModifier(NULL)
	{
	}

	~dHexapodController()
	{
		delete m_animTreeNode;
		NewtonInverseDynamicsDestroy(m_kinematicSolver);
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
		dMatrix align(dYawMatrix(dPi * 0.5f));
		NewtonCollision* const collision = CreateConvexCollision(world, align, size, _CAPSULE_PRIMITIVE, 0);
		DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");

		NewtonBody* const body = CreateSimpleSolid(scene, geometry, mass, location, collision, materialID);
		ScaleIntertia(body, inertiaScale);

		geometry->Release();
		NewtonDestroyCollision(collision);
		return body;
	}

	dCustomInverseDynamicsEffector* AddLeg(DemoEntityManager* const scene, void* const rootNode, const dMatrix& matrix, dFloat partMass, dFloat limbLenght)
	{
		NewtonBody* const parent = NewtonInverseDynamicsGetBody(m_kinematicSolver, rootNode);

		dFloat inertiaScale = 4.0f;
		// make limb base
		dMatrix baseMatrix(dRollMatrix(dPi * 0.5f));
		dMatrix cylinderMatrix(baseMatrix * matrix);
		NewtonBody* const base = CreateCylinder(scene, cylinderMatrix, partMass, inertiaScale, 0.2f, 0.1f);
		dCustomInverseDynamics* const baseHinge = new dCustomInverseDynamics(cylinderMatrix, base, parent);
		baseHinge->SetJointTorque(1000.0f);
		baseHinge->SetTwistAngle(-0.5f * dPi, 0.5f * dPi);
		void* const baseHingeNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, rootNode, baseHinge->GetJoint());

		//make limb forward arm
		dMatrix forwardArmMatrix(dPitchMatrix(-30.0f * dDegreeToRad));
		dVector forwardArmSize(limbLenght * 0.25f, limbLenght * 0.25f, limbLenght, 0.0f);
		forwardArmMatrix.m_posit += forwardArmMatrix.m_right.Scale(forwardArmSize.m_z * 0.5f);
		NewtonBody* const forwardArm = CreateBox(scene, forwardArmMatrix * matrix, forwardArmSize, partMass, inertiaScale);
		dMatrix forwardArmPivot(forwardArmMatrix);
		forwardArmPivot.m_posit -= forwardArmMatrix.m_right.Scale(forwardArmSize.m_z * 0.5f);
		dCustomInverseDynamics* const forwardArmHinge = new dCustomInverseDynamics(forwardArmPivot * matrix, forwardArm, base);
		forwardArmHinge->SetJointTorque(1000.0f);
		forwardArmHinge->SetTwistAngle(-0.5f * dPi, 0.5f * dPi);
		void* const forwardArmHingeNode = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, baseHingeNode, forwardArmHinge->GetJoint());

		//make limb forward arm
		dMatrix armMatrix(dPitchMatrix(-90.0f * dDegreeToRad));
		dFloat armSize = limbLenght * 1.25f;
		armMatrix.m_posit += forwardArmMatrix.m_right.Scale(limbLenght);
		armMatrix.m_posit.m_y -= armSize * 0.5f;
		NewtonBody* const arm = CreateCapsule(scene, armMatrix * matrix, partMass, inertiaScale, armSize * 0.2f, armSize);
		dMatrix armPivot(armMatrix);
		armPivot.m_posit.m_y += armSize * 0.5f;
		dCustomInverseDynamics* const armHinge = new dCustomInverseDynamics(armPivot * matrix, arm, forwardArm);
		armHinge->SetJointTorque(1000.0f);
		armHinge->SetTwistAngle(-0.5f * dPi, 0.5f * dPi);
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
		dCustomInverseDynamicsEffector* legEffectors[32];

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
			dMatrix leftLocation (rightLocation * similarTransform.Inverse() * dYawMatrix(dPi) * similarTransform);
			legEffectors[legEffectorCount] = AddLeg (scene, hexaBodyNode, leftLocation * location, mass * 0.1f, 0.3f);
			legEffectorCount ++;
		}

		// finalize inverse dynamics solver
		NewtonInverseDynamicsEndBuild(m_kinematicSolver);
		
		// create a fix pose frame generator
		dEffectorTreeFixPose* const idlePose = new dEffectorTreeFixPose(hexaBody);
		dEffectorTreeFixPose* const walkPoseGenerator = new dEffectorWalkPoseGenerator(hexaBody);
		m_walkIdleBlender = new dEffectorBlendIdleWalk (hexaBody, idlePose, walkPoseGenerator);

		m_postureModifier = new dEffectorTreePostureGenerator(m_walkIdleBlender);
		m_animTreeNode = new dEffectorTreeRoot(hexaBody, m_postureModifier);

		dMatrix rootMatrix;
		NewtonBodyGetMatrix (hexaBody, &rootMatrix[0][0]);
		rootMatrix = rootMatrix.Inverse();
		for (int i = 0; i < legEffectorCount; i++) {
			dEffectorTreeInterface::dEffectorTransform frame;
			dCustomInverseDynamicsEffector* const effector = legEffectors[i];
			dMatrix effectorMatrix(effector->GetBodyMatrix());

			dMatrix poseMatrix(effectorMatrix * rootMatrix);
			
			frame.m_effector = effector;
			frame.m_posit = poseMatrix.m_posit;
			frame.m_rotation = dQuaternion(poseMatrix);

			idlePose->GetPose().Append(frame);
			walkPoseGenerator->GetPose().Append(frame);
			m_animTreeNode->GetPose().Append(frame);
		}
	}

	void PostUpdate(dFloat timestep, int threadIndex) 
	{
	}

	void PreUpdate(dFloat timestep, int threadIndex)
	{
		m_animTreeNode->Update(timestep);
		NewtonInverseDynamicsUpdate(m_kinematicSolver, timestep, threadIndex);
	}

	void Debug(dCustomJoint::dDebugDisplay* const debugContext) const
	{
		const dEffectorTreeInterface::dEffectorPose& pose = m_animTreeNode->GetPose();
		for (dEffectorTreeInterface::dEffectorPose::dListNode* node = pose.GetFirst(); node; node = node->GetNext()) {
			dCustomInverseDynamicsEffector* const effector = node->GetInfo().m_effector;
			effector->Debug(debugContext);
		}
	}

	dEffectorTreeRoot* m_animTreeNode;
	NewtonInverseDynamics* m_kinematicSolver;
	dEffectorBlendIdleWalk* m_walkIdleBlender; // do not delete 
	dEffectorTreePostureGenerator* m_postureModifier; // do not delete 
};

class dHexapodManager: public dCustomControllerManager<dHexapodController>
{
	public:
	dHexapodManager(DemoEntityManager* const scene)
		:dCustomControllerManager<dHexapodController>(scene->GetNewton(), "sixAxisManipulator")
		,m_currentController(NULL)
		,m_yaw(0.0f)
		,m_roll(0.0f)
		,m_pitch(0.0f)
		,m_posit_x(0.0f)
		,m_posit_y(0.0f)
		,m_speed(0.0f)
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

		ImGui::SliderFloat("speed", &me->m_speed, 0.0f, 1.0f);
		ImGui::SliderFloat("pitch", &me->m_pitch, -10.0f, 10.0f);
		ImGui::SliderFloat("yaw", &me->m_yaw, -10.0f, 10.0f);
		ImGui::SliderFloat("roll", &me->m_roll, -10.0f, 10.0f);
		ImGui::SliderFloat("posit_x", &me->m_posit_x, -0.1f, 0.1f);
		ImGui::SliderFloat("posit_y", &me->m_posit_y, -0.4f, 0.4f);

		for (dListNode* node = me->GetFirst(); node; node = node->GetNext()) {
			dHexapodController* const controller = &node->GetInfo();

			controller->m_walkIdleBlender->SetBlendFactor (me->m_speed);
			controller->SetTarget (me->m_posit_x, -me->m_posit_y, -me->m_pitch * dDegreeToRad, -me->m_yaw * dDegreeToRad, -me->m_roll * dDegreeToRad);
		}
	}

	virtual dHexapodController* CreateController()
	{
		return (dHexapodController*)dCustomControllerManager<dHexapodController>::CreateController();
	}

	dHexapodController* MakeHexapod(DemoEntityManager* const scene, const dMatrix& location)
	{
		dHexapodController* const controller = (dHexapodController*)CreateController();
		controller->MakeHexapod(scene, location);
		m_currentController = controller;
		return controller;
	}

	void OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
	{
		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			dHexapodController* const controller = &node->GetInfo();
			controller->Debug(debugContext);
		}
	}

	dHexapodController* m_currentController;
	dFloat32 m_yaw;
	dFloat32 m_roll;
	dFloat32 m_pitch;
	dFloat32 m_posit_x;
	dFloat32 m_posit_y;
	dFloat32 m_speed;
};


void Hexapod(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	CreateLevelMesh (scene, "flatPlane.ngd", true);
	//CreateHeightFieldTerrain(scene, HEIGHTFIELD_DEFAULT_SIZE, HEIGHTFIELD_DEFAULT_CELLSIZE, 1.5f, 0.3f, 200.0f, -50.0f);
	dHexapodManager* const robotManager = new dHexapodManager(scene);

	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID(world);
	NewtonMaterialSetDefaultFriction(world, defaultMaterialID, defaultMaterialID, 1.0f, 1.0f);
	NewtonMaterialSetDefaultElasticity(world, defaultMaterialID, defaultMaterialID, 0.1f);

	dMatrix location (dGetIdentityMatrix());
	location.m_posit = dVector(FindFloor(world, dVector(-0.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));
	location.m_posit.m_y += 1.0f;

	const int count = 5;
	dMatrix location1(location);
	location1.m_posit.m_z += 2.0f;
	for (int i = 0; i < count; i++) {
		location.m_posit.m_x += 2.0f;
		location1.m_posit.m_x += 2.0f;
		robotManager->MakeHexapod (scene, location);
		//robotManager->MakeHexapod (scene, location1);
	}

	location.m_posit = dVector(FindFloor(scene->GetNewton(), dVector(-0.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));
	dVector origin(FindFloor(world, dVector(-4.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));
	origin.m_y  += 2.5f;

//	dVector size(3.0f, 0.125f, 3.0f, 0.0f);
//	dMatrix shapeOffsetMatrix (dGetIdentityMatrix());
//	AddPrimitiveArray(scene, 50.0f, location.m_posit, size, count, count, 6.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);

	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}


