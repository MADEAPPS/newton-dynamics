/* Copyright (c) <2003-2019> <Newton Game Dynamics>
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


#define HEXAPOD_MASS 500.0f

class dModelAnimTreePoseWalkSequence: public dModelAnimTreePose
{
	public:
	dModelAnimTreePoseWalkSequence(dModelRootNode* const model, const dModelKeyFramePose& pose)
		:dModelAnimTreePose(model, pose)
		,m_acc(0.0f)
		,m_amplitud_x(0.35f)
		,m_amplitud_y(0.1f)
		,m_period(1.0f)
		,m_cycle()
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
		for (int i = 0; i < size; i++) {
			knots[i] = dFloat(i) / (size - 1);
		}
		memset(leftControlPoints, 0, sizeof (leftControlPoints));

		dFloat x = -m_amplitud_x / 2.0f;
		dFloat step_x = m_amplitud_x / splite;
		for (int i = 0; i <= splite; i++) {
			leftControlPoints[i + 1].m_y = m_amplitud_y * dSin(dPi * dFloat(i) / splite);
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

		//cycle.CreateFromKnotVectorAndControlPoints(3, size, knots, leftControlPoints);
		m_cycle.CreateFromKnotVectorAndControlPoints(1, size, knots, &leftControlPoints[1]);
	}

	void Evaluate(dFloat timestep)
	{
		m_acc = dMod(m_acc + timestep, m_period);
	}

	virtual void GeneratePose(dModelKeyFramePose& output)
	{
		dModelAnimTreePose::GeneratePose(output);

		dFloat param = m_acc / m_period;
		dBigVector left(m_cycle.CurvePoint(param));
		dBigVector right(m_cycle.CurvePoint(dMod(param + 0.5f, 1.0f)));

		dFloat high[2];
		dFloat stride[2];
		high[0] = dFloat(left.m_y);
		high[1] = dFloat(right.m_y);
		stride[0] = dFloat(left.m_x);
		stride[1] = dFloat(right.m_x);

		int index = 0;
		for (dModelKeyFramePose::dListNode* node = output.GetFirst(); node; node = node->GetNext()) {
			dModelKeyFrame& transform = node->GetInfo();
			transform.m_posit.m_y += high[m_sequence[index]];
			transform.m_posit.m_x += stride[m_sequence[index]];
			index++;
		}
	}

	dFloat m_acc;
	dFloat m_period;
	dFloat m_amplitud_x;
	dFloat m_amplitud_y;
	dBezierSpline m_cycle;
	int m_sequence[6];
};

class dModelAnimTreeHipController: public dModelAnimTree
{
	public:
	dModelAnimTreeHipController(dModelRootNode* const model, dModelAnimTree* const child)
		:dModelAnimTree(model)
		,m_euler(0.0f)
		,m_position(0.0f)
		,m_child(child)
	{
		m_position.m_w = 1.0f;
	}

	~dModelAnimTreeHipController()
	{
		delete m_child;
	}

	virtual void Evaluate(dFloat timestep)
	{
		m_child->Evaluate(timestep);
	}

	void SetTarget(dFloat z, dFloat y, dFloat roll, dFloat yaw, dFloat pitch)
	{
		m_position.m_y = y;
		m_position.m_z = z;
		m_euler.m_x = pitch * dDegreeToRad;
		m_euler.m_y = yaw * dDegreeToRad;
		m_euler.m_z = roll * dDegreeToRad;
	}

	virtual void GeneratePose(dModelKeyFramePose& output)
	{
		m_child->GeneratePose(output);
		dQuaternion rotation(dPitchMatrix(m_euler.m_x) * dYawMatrix(m_euler.m_y) * dRollMatrix(m_euler.m_z));
		for (dModelKeyFramePose::dListNode* node = output.GetFirst(); node; node = node->GetNext()) {
			dModelKeyFrame& transform = node->GetInfo();
			transform.m_rotation = transform.m_rotation * rotation;
			transform.m_posit = m_position + rotation.RotateVector(transform.m_posit);
		}
	}

	dVector m_euler;
	dVector m_position;
	dModelAnimTree* m_child;
};

class dHexapod: public dModelRootNode
{
	public:
	dHexapod(NewtonBody* const rootBody, const dMatrix& bindMatrix)
		:dModelRootNode(rootBody, bindMatrix)
		,m_pose()
		,m_animtree(NULL)
		,m_walkIdleBlender(NULL)
		,m_postureModifier(NULL)
	{
	}

	~dHexapod()
	{
		if (m_animtree) {
			delete m_animtree;
		}
	}

	void ApplyControls (dFloat timestep, dFloat speed, dFloat z, dFloat y, dFloat roll, dFloat yaw, dFloat pitch)
	{
		m_walkIdleBlender->SetParam(speed);
		m_postureModifier->SetTarget(z, y, roll, yaw, pitch);

		m_animtree->Evaluate(timestep);
		m_animtree->GeneratePose(m_pose);

		for (dModelKeyFramePose::dListNode* node = m_pose.GetFirst(); node; node = node->GetNext()) {
			dModelKeyFrame& transform = node->GetInfo();
			transform.m_effector->SetTargetMatrix(dMatrix (transform.m_rotation, transform.m_posit));
		}
	}

	dModelKeyFramePose m_pose;
	dModelAnimTree* m_animtree;

	// do not delete !!
	dModelAnimTreePoseBlender* m_walkIdleBlender;
	dModelAnimTreeHipController* m_postureModifier;
};

class dHexapodManager: public dModelManager
{
	public:
	dHexapodManager(DemoEntityManager* const scene)
		:dModelManager(scene->GetNewton())
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
		scene->Print(color, "inverse Dynamics Hexapod");

		ImGui::Separator();
		ImGui::SliderFloat("speed", &me->m_speed, 0.0f, 1.0f);
		ImGui::SliderFloat("pitch", &me->m_pitch, -10.0f, 10.0f);
		ImGui::SliderFloat("yaw", &me->m_yaw, -10.0f, 10.0f);
		ImGui::SliderFloat("roll", &me->m_roll, -10.0f, 10.0f);
		ImGui::SliderFloat("posit_x", &me->m_posit_x, -0.1f, 0.1f);
		ImGui::SliderFloat("posit_y", &me->m_posit_y, -0.4f, 0.4f);
		
		//for (dListNode* node = me->GetFirst(); node; node = node->GetNext()) {
		//	dHexapodController* const controller = &node->GetInfo();
		//	controller->m_walkIdleBlender->SetBlendFactor(me->m_speed);
		//	controller->SetTarget(me->m_posit_x, -me->m_posit_y, -me->m_pitch, -me->m_yaw, -me->m_roll);
		//}
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

	void NormalizeMassAndInertia(dModelRootNode* const model, dFloat modelMass) const
	{
		int stack = 1;
		int bodyCount = 0;
		NewtonBody* bodyArray[1024];
		dModelNode* stackBuffer[32];

		stackBuffer[0] = model;
		while (stack) {
			stack--;
			dModelNode* const root = stackBuffer[stack];
			bodyArray[bodyCount] = root->GetBody();
			bodyCount++;
			const dModelChildrenList& children = root->GetChildren();
			for (dModelChildrenList::dListNode* node = children.GetFirst(); node; node = node->GetNext()) {
				stackBuffer[stack] = node->GetInfo().GetData();
				stack++;
			}
		}

		dFloat totalMass = 0.0f;
		for (int i = 0; i < bodyCount; i++) {
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;
			dFloat mass;

			NewtonBody* const body = bodyArray[i];
			NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
			totalMass += mass;
		}

		dFloat massNormalize = modelMass / totalMass;
		for (int i = 0; i < bodyCount; i++) {
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;
			dFloat mass;

			NewtonBody* const body = bodyArray[i];
			NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);

			mass *= massNormalize;
			Ixx *= massNormalize;
			Iyy *= massNormalize;
			Izz *= massNormalize;

			dFloat minInertia = dMin(Ixx, dMin(Iyy, Izz));
			if (minInertia < 4.0f) {
				dFloat maxInertia = dMax(dFloat(10.0f), dMax(Ixx, dMax(Iyy, Izz)));
				Ixx = maxInertia;
				Iyy = maxInertia;
				Izz = maxInertia;
			}

			NewtonBodySetMassMatrix(body, mass, Ixx, Iyy, Izz);
		}
	}

	NewtonBody* CreateBox(DemoEntityManager* const scene, const dMatrix& location, const dVector& size, dFloat mass, dFloat inertiaScale) const
	{
		NewtonWorld* const world = scene->GetNewton();
		int materialID = NewtonMaterialGetDefaultGroupID(world);
		NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);
		DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");
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
		DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");

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
		DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");

		NewtonBody* const body = CreateSimpleSolid(scene, geometry, mass, location, collision, materialID);
		ScaleIntertia(body, inertiaScale);

		geometry->Release();
		NewtonDestroyCollision(collision);
		return body;
	}

	dCustomKinematicController* AddLeg(DemoEntityManager* const scene, dHexapod* const rootNode, const dMatrix& matrix, dFloat partMass, dFloat limbLenght)
	{
		NewtonBody* const parent = rootNode->GetBody();

		dFloat inertiaScale = 4.0f;
		// make limb base
		dMatrix baseMatrix(dRollMatrix(dPi * 0.5f));
		dMatrix cylinderMatrix(baseMatrix * matrix);
		NewtonBody* const base = CreateCylinder(scene, cylinderMatrix, partMass, inertiaScale, 0.2f, 0.1f);
		dCustomHinge* const baseHinge = new dCustomHinge (cylinderMatrix, base, parent);
		baseHinge->EnableLimits(true);
		baseHinge->SetLimits(-80.0f * dDegreeToRad, -80.0f * dDegreeToRad);
		dModelNode* const baseHingeNode = new dModelNode(base, dGetIdentityMatrix(), rootNode);

		//make limb forward arm
		dMatrix forwardArmMatrix(dPitchMatrix(-30.0f * dDegreeToRad));
		dVector forwardArmSize(limbLenght * 0.25f, limbLenght * 0.25f, limbLenght, 0.0f);
		forwardArmMatrix.m_posit += forwardArmMatrix.m_right.Scale(forwardArmSize.m_z * 0.5f);
		NewtonBody* const forwardArm = CreateBox(scene, forwardArmMatrix * matrix, forwardArmSize, partMass, inertiaScale);
		dMatrix forwardArmPivot(forwardArmMatrix);
		forwardArmPivot.m_posit -= forwardArmMatrix.m_right.Scale(forwardArmSize.m_z * 0.5f);
		dCustomHinge* const forwardArmHinge = new dCustomHinge (forwardArmPivot * matrix, forwardArm, base);
		forwardArmHinge->EnableLimits(true);
		forwardArmHinge->SetLimits(-80.0f * dDegreeToRad, -80.0f * dDegreeToRad);
		dModelNode* const forwardArmHingeNode = new dModelNode(forwardArm, dGetIdentityMatrix(), baseHingeNode);

		//make limb forward arm
		dMatrix armMatrix(dPitchMatrix(-90.0f * dDegreeToRad));
		dFloat armSize = limbLenght * 1.25f;
		armMatrix.m_posit += forwardArmMatrix.m_right.Scale(limbLenght);
		armMatrix.m_posit.m_y -= armSize * 0.5f;
		NewtonBody* const arm = CreateCapsule(scene, armMatrix * matrix, partMass, inertiaScale, armSize * 0.2f, armSize);
		dMatrix armPivot(armMatrix);
		armPivot.m_posit.m_y += armSize * 0.5f;
		dCustomHinge* const armHinge = new dCustomHinge (armPivot * matrix, arm, forwardArm);
		armHinge->EnableLimits(true);
		armHinge->SetLimits(-80.0f * dDegreeToRad, -80.0f * dDegreeToRad);
		dModelNode* const armHingeNode = new dModelNode(arm, dGetIdentityMatrix(), forwardArmHingeNode);
		

		dMatrix effectorMatrix(dGetIdentityMatrix());
		effectorMatrix.m_posit = armPivot.m_posit;
		effectorMatrix.m_posit.m_y -= armSize;
		dCustomKinematicController* const effector = new dCustomKinematicController(armHingeNode->GetBody(), effectorMatrix * matrix, parent);
		effector->SetAsLinear();
		effector->SetSolverModel(1);
		//effector->SetAsThreedof();
		effector->SetMaxLinearFriction(HEXAPOD_MASS * 9.8f * 10.0f);

		return effector;
	}

	void MakeHexapod(DemoEntityManager* const scene, const dMatrix& location)
	{
		// make the root body
		dMatrix baseMatrix(dGetIdentityMatrix());
		baseMatrix.m_posit.m_y += 0.35f;
		dVector size(1.3f, 0.31f, 0.5f, 0.0f);
		NewtonBody* const hexaBody = CreateBox(scene, baseMatrix * location, size, 1.0f, 1.0f);

		// make a kinematic controlled model.
		dHexapod* const hexapod = new dHexapod(hexaBody, dGetIdentityMatrix());

		// add the model to the manager
		AddRoot(hexapod);

		// add all the hexapod limbs procedurally
		for (int i = 0; i < 3; i++) {
			dModelKeyFrame keyFrame;

			// make right legs
			dMatrix rightLocation(baseMatrix);
			rightLocation.m_posit += rightLocation.m_right.Scale(size.m_z * 0.65f);
			rightLocation.m_posit += rightLocation.m_front.Scale(size.m_x * 0.3f - size.m_x * i / 3.0f);
			keyFrame.m_effector = AddLeg(scene, hexapod, rightLocation * location, 0.1f, 0.3f);
			keyFrame.SetMatrix (keyFrame.m_effector->GetTargetMatrix());
			hexapod->m_pose.Append(keyFrame);

			// make left legs
			dMatrix similarTransform(dGetIdentityMatrix());
			similarTransform.m_posit.m_x = rightLocation.m_posit.m_x;
			similarTransform.m_posit.m_y = rightLocation.m_posit.m_y;
			dMatrix leftLocation(rightLocation * similarTransform.Inverse() * dYawMatrix(dPi) * similarTransform);
			keyFrame.m_effector = AddLeg(scene, hexapod, leftLocation * location, 0.1f, 0.3f);
			keyFrame.SetMatrix (keyFrame.m_effector->GetTargetMatrix());
			hexapod->m_pose.Append(keyFrame);
		}

		// normalize the mass of body parts
 		NormalizeMassAndInertia(hexapod, HEXAPOD_MASS);

		// create a fix pose frame generator
		dModelAnimTreePose* const idlePose = new dModelAnimTreePose(hexapod, hexapod->m_pose);
		dModelAnimTreePose* const walkPoseGenerator = new dModelAnimTreePoseWalkSequence(hexapod, hexapod->m_pose);
		hexapod->m_walkIdleBlender = new dModelAnimTreePoseBlender(hexapod, idlePose, walkPoseGenerator);
		hexapod->m_postureModifier = new dModelAnimTreeHipController(hexapod, hexapod->m_walkIdleBlender);

		m_currentController = hexapod;
		hexapod->m_animtree = hexapod->m_postureModifier;
	}

	virtual void OnDebug(dModelRootNode* const model, dCustomJoint::dDebugDisplay* const debugContext)
	{
		dHexapod* const hexapod = (dHexapod*)model;
		dFloat scale = debugContext->GetScale();
		debugContext->SetScale(0.5f);
		for (dModelKeyFramePose::dListNode* node = hexapod->m_pose.GetFirst(); node; node = node->GetNext()) {
			const dModelKeyFrame& keyFrame = node->GetInfo();
			keyFrame.m_effector->Debug(debugContext);
		}
		debugContext->SetScale(scale);
	}

	virtual void OnPreUpdate(dModelRootNode* const model, dFloat timestep) const
	{
		if (model == m_currentController) {
			m_currentController->ApplyControls(timestep, m_speed, m_posit_x, m_posit_y, m_roll, m_yaw, m_pitch);
		}
	}

	dHexapod* m_currentController;
	dFloat m_yaw;
	dFloat m_roll;
	dFloat m_pitch;
	dFloat m_posit_x;
	dFloat m_posit_y;
	dFloat m_speed;
};

void Hexapod(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	CreateLevelMesh (scene, "flatPlane.ngd", true);
	//CreateHeightFieldTerrain(scene, HEIGHTFIELD_DEFAULT_SIZE, HEIGHTFIELD_DEFAULT_CELLSIZE, 1.5f, 0.3f, 200.0f, -50.0f);

	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID(world);
	NewtonMaterialSetDefaultFriction(world, defaultMaterialID, defaultMaterialID, 1.0f, 1.0f);
	NewtonMaterialSetDefaultElasticity(world, defaultMaterialID, defaultMaterialID, 0.1f);

	dMatrix location (dGetIdentityMatrix());
	location.m_posit = dVector(FindFloor(world, dVector(-0.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));
	location.m_posit.m_y += 1.0f;

	int count = 5;
count = 1;
	dMatrix location1(location);
	dFloat x0 = location.m_posit.m_x;

	dHexapodManager* const robotManager = new dHexapodManager(scene);
	for (int j = 0; j < 1; j++) {
		location.m_posit.m_z += 2.0f;
		location.m_posit.m_x = x0;
		for (int i = 0; i < count; i++) {
			location.m_posit.m_x += 2.0f;
			robotManager->MakeHexapod(scene, location);
		}
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


