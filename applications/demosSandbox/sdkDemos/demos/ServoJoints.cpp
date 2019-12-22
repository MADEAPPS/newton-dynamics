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
#include "PhysicsUtils.h"
#include "TargaToOpenGl.h"
#include "DemoEntityManager.h"
#include "DebugDisplay.h"
#include "HeightFieldPrimitive.h"

#define D_TRACTOR_CAMERA_DIST 10.0f


class dTractorControls
{
	public:
	dTractorControls()
	{
		memset(this, 0, sizeof (dTractorControls));
	}
	dFloat m_throttle;
	dFloat m_steeringValue;
	dFloat m_frontArmAngle;
	dFloat m_frontBucketAngle;
};

class dDoubleDifferentialGear: public dCustomDifferentialGear
{
	public:
	dDoubleDifferentialGear(dFloat gearRatio, dCustomDoubleHinge* const childDiff, dFloat diffSign, dCustomDoubleHinge* const diff)
		:dCustomDifferentialGear(gearRatio, dVector(1.0f, 0.0f, 0.0f, 0.0f), childDiff->GetBody0(), diffSign, diff)
		,m_axleGear(childDiff)
	{
		dMatrix matrix0;
		dMatrix matrix1;
		dMatrix axleBodyMatrix;

		dAssert(m_axleGear->IsType(dCustomDoubleHinge::GetType()));
		childDiff->CalculateGlobalMatrix(matrix0, matrix1);
		NewtonBodyGetMatrix(m_axleGear->GetBody1(), &axleBodyMatrix[0][0]);
		m_axlePin = axleBodyMatrix.UnrotateVector(matrix1.m_up);
	}

	dVector CalculateAxlePin(const dVector& localPin) const
	{
		dMatrix axleBodyMatrix;
		NewtonBodyGetMatrix(m_axleGear->GetBody1(), &axleBodyMatrix[0][0]);
		return axleBodyMatrix.RotateVector(localPin);
	}

	dCustomDoubleHinge* m_axleGear;
};

class dTractorEngine: public dCustomDoubleHinge
{
	public:

	dTractorEngine(const dMatrix& pinAndPivotFrame, NewtonBody* const engine, NewtonBody* const chassis)
		:dCustomDoubleHinge(pinAndPivotFrame, engine, chassis)
		,m_alpha(0.0f)
	{
		EnableLimits(false);
		EnableLimits1(false);
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		//dMatrix matrix1;
		//NewtonBodyGetMatrix(m_body1, &matrix1[0][0]);
		//dMatrix matrix0(GetMatrix0().Inverse() * GetMatrix1() * matrix1);
		//NewtonBodySetMatrixNoSleep(m_body0, &matrix0[0][0]);
		dCustomDoubleHinge::SubmitConstraints(timestep, threadIndex);
	}

	void SubmitAngularRow(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
	{
		dCustomDoubleHinge::SubmitAngularRow(matrix0, matrix1, timestep);

		const dVector& tractionDir = matrix1.m_up;
		NewtonUserJointAddAngularRow(m_joint, 0.0f, &tractionDir[0]);

		dVector omega0(0.0f);
		dVector omega1(0.0f);
		NewtonBodyGetOmega(m_body0, &omega0[0]);
		NewtonBodyGetOmega(m_body1, &omega1[0]);

		dFloat targetOmega = m_alpha * 40.0f;
		dFloat alpha = (targetOmega - tractionDir.DotProduct3(omega0 - omega1)) / timestep;
		alpha = dClamp (alpha, dFloat (-500.0f), dFloat (500.0f));

		NewtonUserJointSetRowAcceleration(m_joint, alpha);
		NewtonUserJointSetRowMinimumFriction(m_joint, -4000.0f);
		NewtonUserJointSetRowMaximumFriction(m_joint, 4000.0f);
	}

	dFloat m_alpha;
};

class dTractorModel: public dModelRootNode
{
	public:
	dTractorModel(NewtonWorld* const world, const char* const modelName, const dMatrix& location)
		:dModelRootNode(NULL, dGetIdentityMatrix())
		,m_engine(NULL)
		,m_leftWheel(NULL)
		,m_rightWheel(NULL)
		,m_frontArm(NULL)
		,m_frontBucket(NULL)
	{
		MakeChassis(world, modelName, location);
		MakeDriveTrain();
		MakeFrontBucket();
	}

	void ApplyControl (const dTractorControls& controls, dFloat timestep)
	{
		NewtonBodySetSleepState(GetBody(), 0);

		m_controls = controls;

		m_engine->m_alpha = controls.m_throttle;
		m_leftWheel->SetTargetAngle1(m_controls.m_steeringValue * 25.0f * dDegreeToRad);
		m_rightWheel->SetTargetAngle1(-m_controls.m_steeringValue * 25.0f * dDegreeToRad);
		m_frontArm->SetTargetAngle(m_controls.m_frontArmAngle * dDegreeToRad);
		m_frontBucket->SetTargetAngle(m_controls.m_frontBucketAngle * dDegreeToRad);
	}

	private:
	NewtonBody* CreateBodyPart(NewtonWorld* const world, DemoEntity* const bodyPart, dFloat mass)
	{
		NewtonCollision* const shape = bodyPart->CreateCollisionFromchildren(world);
		dAssert(shape);

		// calculate the bone matrix
		dMatrix matrix(bodyPart->CalculateGlobalMatrix());

		// create the rigid body that will make this bone
		NewtonBody* const body = NewtonCreateDynamicBody(world, shape, &matrix[0][0]);

		// assign the material ID
		NewtonBodySetMaterialGroupID(body, NewtonMaterialGetDefaultGroupID(world));

		// destroy the collision helper shape 
		NewtonDestroyCollision(shape);

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(body);

		// save the root node as the use data
		NewtonCollisionSetUserData(collision, this);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonBodySetMassProperties(body, mass, collision);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(body, bodyPart);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);
		return body;
	}

	void MakeChassis(NewtonWorld* const world, const char* const modelName, const dMatrix& location)
	{
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		// make a clone of the mesh 
		DemoEntity* const vehicleModel = DemoEntity::LoadNGD_mesh(modelName, world, scene->GetShaderCache());
		scene->Append(vehicleModel);

		dMatrix matrix(vehicleModel->GetCurrentMatrix());
		matrix.m_posit = location.m_posit;
		vehicleModel->ResetMatrix(*scene, matrix);

		DemoEntity* const rootEntity = (DemoEntity*)vehicleModel->Find("MainBody");
		m_body = CreateBodyPart(world, rootEntity, 4000.0f);

		dVector com;
		NewtonBodyGetCentreOfMass(m_body, &com[0]);
		com.m_y -= 0.75f;
		NewtonBodySetCentreOfMass(m_body, &com[0]);
	}

	void GetTireDimensions(DemoEntity* const bodyPart, dFloat& radius, dFloat& width)
	{
		radius = 0.0f;
		dFloat maxWidth = 0.0f;
		dFloat minWidth = 0.0f;

		DemoMesh* const mesh = (DemoMesh*)bodyPart->GetMesh();
		dAssert(mesh->IsType(DemoMesh::GetRttiType()));
		const dMatrix& matrix = bodyPart->GetMeshMatrix();
		dFloat* const array = mesh->m_vertex;
		for (int i = 0; i < mesh->m_vertexCount; i++) {
			dVector p(matrix.TransformVector(dVector(array[i * 3 + 0], array[i * 3 + 1], array[i * 3 + 2], 1.0f)));
			maxWidth = dMax(p.m_x, maxWidth);
			minWidth = dMin(p.m_x, minWidth);
			radius = dMax(p.m_y, radius);
		}
		width = maxWidth - minWidth;
		radius -= width * 0.5f;
	}

	NewtonCollision* const MakeTireShape(DemoEntity* const tireModel)
	{
		dFloat width;
		dFloat radius;
		GetTireDimensions(tireModel, radius, width);
		dMatrix align(dGetIdentityMatrix());
		NewtonWorld* const world = NewtonBodyGetWorld(GetBody());
		NewtonCollision* const tireShape = NewtonCreateChamferCylinder(world, radius, width, 0, &align[0][0]);
		return tireShape;
	}

	dModelNode* MakeTireBody(const char* const entName, NewtonCollision* const tireCollision, dFloat mass, dModelNode* const parentNode)
	{
		NewtonBody* const parentBody = parentNode->GetBody();
		NewtonWorld* const world = NewtonBodyGetWorld(GetBody());
		DemoEntity* const parentModel = (DemoEntity*)NewtonBodyGetUserData(parentBody);
		DemoEntity* const tireModel = parentModel->Find(entName);

		// calculate the bone matrix
		dMatrix matrix(tireModel->CalculateGlobalMatrix());

		// create the rigid body that will make this bone
		NewtonBody* const tireBody = NewtonCreateDynamicBody(world, tireCollision, &matrix[0][0]);

		// assign the material ID
		NewtonBodySetMaterialGroupID(tireBody, NewtonMaterialGetDefaultGroupID(world));

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(tireBody);

		// save the root node as the use data
		NewtonCollisionSetUserData(collision, this);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonBodySetMassProperties(tireBody, mass, collision);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(tireBody, tireModel);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(tireBody, PhysicsApplyGravityForce);

		dMatrix bindMatrix(tireModel->GetParent()->CalculateGlobalMatrix(parentModel).Inverse());
		dModelNode* const bone = new dModelNode(tireBody, bindMatrix, parentNode);
		return bone;
	}

	dModelNode* MakeTire(const char* const entName, dFloat mass, dModelNode* const parentNode)
	{
		NewtonBody* const parentBody = parentNode->GetBody();
		DemoEntity* const parentModel = (DemoEntity*)NewtonBodyGetUserData(parentBody);
		DemoEntity* const tireModel = parentModel->Find(entName);

		NewtonCollision* const tireCollision = MakeTireShape(tireModel);
		dModelNode* const bone = MakeTireBody(entName, tireCollision, mass, parentNode);
		NewtonDestroyCollision(tireCollision);

		return bone;
	}

	dModelNode* MakeFronAxel()
	{
		NewtonWorld* const world = NewtonBodyGetWorld(GetBody());

		// add front_axel 
		NewtonBody* const chassisBody = GetBody();
		DemoEntity* const chassisEntity = (DemoEntity*)NewtonBodyGetUserData(chassisBody);
		DemoEntity* const axelEntity = chassisEntity->Find("front_axel");
		NewtonBody* const fronAxelBody = CreateBodyPart(world, axelEntity, 100.0f);

		// connect the part to the main body with a hinge
		dMatrix hingeFrame;
		NewtonBodyGetMatrix(fronAxelBody, &hingeFrame[0][0]);
		hingeFrame = dRollMatrix(90.0f * dDegreeToRad) * hingeFrame;
		dCustomHinge* const hinge = new dCustomHinge(hingeFrame, fronAxelBody, chassisBody);
		hinge->EnableLimits(true);
		hinge->SetLimits(-15.0f * dDegreeToRad, 15.0f * dDegreeToRad);

		//dMatrix bindMatrix(bodyPart->GetParent()->CalculateGlobalMatrix(parentModel).Inverse());
		dMatrix bindMatrix(dGetIdentityMatrix());
		return new dModelNode(fronAxelBody, bindMatrix, this);
	}

	dModelNode* MakeDifferential(dModelNode* const leftWheel, dModelNode* const rightWheel)
	{
		NewtonBody* const chassis = GetBody();
		NewtonWorld* const world = NewtonBodyGetWorld(GetBody());
		NewtonCollision* const shape = NewtonCreateCylinder(world, 0.125f, 0.125f, 0.75f, 0, NULL);

		// create the rigid body that will make this bone
		dMatrix diffMatrix;
		NewtonBodyGetMatrix(chassis, &diffMatrix[0][0]);
		diffMatrix = dRollMatrix(0.5f * dPi) * diffMatrix;

		// make a non collideble engine body
		NewtonBody* const diffBody = NewtonCreateDynamicBody(world, shape, &diffMatrix[0][0]);

		// destroy the collision helper shape
		NewtonDestroyCollision(shape);

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(diffBody);
		NewtonCollisionSetMode(collision, 0);

		// calculate the moment of inertia and the relative center of mass of the solid
		dFloat mass = 50.0f;
		dFloat radius = 1.0f;
		dFloat Inertia = 2.0f * mass * radius * radius / 5.0f;
		NewtonBodySetMassMatrix(diffBody, mass, Inertia, Inertia, Inertia);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(diffBody, PhysicsApplyGravityForce);

		// connect engine to chassis with a hinge
		dMatrix diffAxis;
		diffAxis.m_front = diffMatrix.m_front;
		diffAxis.m_up = diffMatrix.m_right;
		diffAxis.m_right = diffAxis.m_front.CrossProduct(diffAxis.m_up);
		diffAxis.m_posit = diffMatrix.m_posit;

		// add the engine joint 
		dCustomDoubleHinge* const diff = new dCustomDoubleHinge(diffAxis, diffBody, chassis);

		dMatrix matrix;
		dMatrix leftWheelMatrix;
		leftWheel->GetJoint()->CalculateGlobalMatrix(leftWheelMatrix, matrix);
		new dCustomDifferentialGear(1.0f, leftWheelMatrix.m_front, leftWheel->GetBody(), 1.0f, diff);

		dMatrix rightWheelMatrix;
		rightWheel->GetJoint()->CalculateGlobalMatrix(rightWheelMatrix, matrix);
		new dCustomDifferentialGear(1.0f, rightWheelMatrix.m_front, rightWheel->GetBody(), -1.0f, diff);

		// connect engine to chassis.
		dMatrix bindMatrix(dGetIdentityMatrix());
		return new dModelNode(diffBody, bindMatrix, this);
	}

	dModelNode* MakeRearAxle ()
	{
		dMatrix matrix;
		dModelNode* const leftWheel = MakeTire("rl_tire", 100.0f, this);
		NewtonBodyGetMatrix(leftWheel->GetBody(), &matrix[0][0]);
		new dCustomHinge(matrix, leftWheel->GetBody(), leftWheel->GetParent()->GetBody());

		dModelNode* const rightWheel = MakeTire("rr_tire", 100.0f, this);
		NewtonBodyGetMatrix(rightWheel->GetBody(), &matrix[0][0]);
		new dCustomHinge(matrix, rightWheel->GetBody(), rightWheel->GetParent()->GetBody());

		return MakeDifferential(leftWheel, rightWheel);
	}

	dModelNode* MakeFrontAxle()
	{
		dMatrix matrix;
		dModelNode* const axelNode = MakeFronAxel();

		dModelNode* const leftWheel = MakeTire("fl_tire", 50.0f, axelNode);
		NewtonBodyGetMatrix(leftWheel->GetBody(), &matrix[0][0]);
		m_leftWheel = new dCustomDoubleHingeActuator(matrix, leftWheel->GetBody(), leftWheel->GetParent()->GetBody());
		m_leftWheel->EnabledAxis0(false);
		m_leftWheel->SetAngularRate1(2.0f);

		dModelNode* const rightWheel = MakeTire("fr_tire", 50.0f, axelNode);
		NewtonBodyGetMatrix(rightWheel->GetBody(), &matrix[0][0]);
		m_rightWheel = new dCustomDoubleHingeActuator(matrix, rightWheel->GetBody(), rightWheel->GetParent()->GetBody());
		m_rightWheel->EnabledAxis0(false);
		m_rightWheel->SetAngularRate1(2.0f);

		return MakeDifferential(leftWheel, rightWheel);
	}

	void MakeDriveTrain()
	{
		//rr_tire, rl_tire, front_axel, fr_tire, fl_tire
		dModelNode* const rearDiff = MakeRearAxle();
		dModelNode* const fronfDiff = MakeFrontAxle();

		NewtonBody* const chassis = GetBody();
		NewtonWorld* const world = NewtonBodyGetWorld(GetBody());
		NewtonCollision* const shape = NewtonCreateCylinder(world, 0.125f, 0.125f, 0.75f, 0, NULL);

		// create the rigid body that will make this bone
		dMatrix engineMatrix;
		NewtonBodyGetMatrix(chassis, &engineMatrix[0][0]);
		engineMatrix = dRollMatrix(0.5f * dPi) * engineMatrix;

		NewtonBody* const engineBody = NewtonCreateDynamicBody(world, shape, &engineMatrix[0][0]);

		// destroy the collision helper shape
		NewtonDestroyCollision(shape);

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(engineBody);
		NewtonCollisionSetMode(collision, 0);

		// calculate the moment of inertia and the relative center of mass of the solid
		dFloat mass = 75.0f;
		dFloat radius = 1.0f;
		dFloat Inertia = 2.0f * mass * radius * radius / 5.0f;
		NewtonBodySetMassMatrix(engineBody, mass, Inertia, Inertia, Inertia);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(engineBody, PhysicsApplyGravityForce);

		// connect engine to chassis with a hinge
		dMatrix engineAxis;
		engineAxis.m_front = engineMatrix.m_front;
		engineAxis.m_up = engineMatrix.m_right;
		engineAxis.m_right = engineAxis.m_front.CrossProduct(engineAxis.m_up);
		engineAxis.m_posit = engineMatrix.m_posit;

		// add the engine joint 
		m_engine = new dTractorEngine(engineAxis, engineBody, chassis);

		// connect engine to chassis.
		dMatrix bindMatrix(dGetIdentityMatrix());
		new dModelNode(engineBody, bindMatrix, this);

		new dDoubleDifferentialGear(5.0f, (dCustomDoubleHinge*)rearDiff->GetJoint(), 1.0f, m_engine);
		new dDoubleDifferentialGear(5.0f, (dCustomDoubleHinge*)fronfDiff->GetJoint(), -1.0f, m_engine);
	}

	void MakeHydraulic (dModelNode* const node, const char* const name0, const char* const name1, const char* const effectorName1)
	{
		NewtonWorld* const world = NewtonBodyGetWorld(GetBody());

		dModelNode* const parentNode = node->GetParent();
		NewtonBody* const parentBody = parentNode->GetBody();
		DemoEntity* const parentEntity = (DemoEntity*)NewtonBodyGetUserData(parentBody);
		DemoEntity* const hydrolicEntity0 = parentEntity->Find(name0);
		NewtonBody* const hydrolicBody0 = CreateBodyPart(world, hydrolicEntity0, 10.0f);

		dMatrix hingeFrame;
		NewtonBodyGetMatrix(hydrolicBody0, &hingeFrame[0][0]);
		new dCustomHinge(hingeFrame, hydrolicBody0, parentBody);

		dMatrix bindMatrix(dGetIdentityMatrix());
		dModelNode* const node0 = new dModelNode(hydrolicBody0, bindMatrix, parentNode);

		DemoEntity* const hydrolicEntity1 = hydrolicEntity0->Find(name1);
		NewtonBody* const hydrolicBody1 = CreateBodyPart(world, hydrolicEntity1, 10.0f);

		NewtonBodyGetMatrix(hydrolicBody1, &hingeFrame[0][0]);
		new dCustomSlider(hingeFrame, hydrolicBody1, hydrolicBody0);
		dModelNode* const node1 = new dModelNode(hydrolicBody1, bindMatrix, node0);

		DemoEntity* const effectorEntity = hydrolicEntity0->Find(effectorName1);
		dMatrix matrix(effectorEntity->CalculateGlobalMatrix());
		dCustomSixdof* const attachment = new dCustomSixdof(matrix, node1->GetBody(), node->GetBody());
		attachment->DisableAxisX();
		attachment->DisableRotationX();
		attachment->DisableRotationY();
		attachment->DisableRotationZ();
	}
	
	dModelNode* MakeActuator(dModelNode* const parent, const char* const name, dFloat mass)
	{
		NewtonWorld* const world = NewtonBodyGetWorld(GetBody());

		NewtonBody* const armBody = parent->GetBody();
		DemoEntity* const armEntity = (DemoEntity*)NewtonBodyGetUserData(armBody);
		DemoEntity* const bucketEntity = armEntity->Find(name);
		NewtonBody* const bucketBody = CreateBodyPart(world, bucketEntity, 100.0f);

		// connect the part to the main body with a hinge
		dMatrix hingeFrame;
		NewtonBodyGetMatrix(bucketBody, &hingeFrame[0][0]);
		dCustomHingeActuator* const actuator = new dCustomHingeActuator(hingeFrame, bucketBody, armBody);
		actuator->SetAngularRate(0.5f);

		dMatrix bindMatrix(dGetIdentityMatrix());
		return new dModelNode(bucketBody, bindMatrix, parent);
	}

	void MakeFrontBucket()
	{
		dModelNode* const armNode = MakeActuator(this, "arms", 100.0f);
		m_frontArm = (dCustomHingeActuator*)armNode->GetJoint();

		MakeHydraulic (armNode, "armHydraulicPiston_left", "armHydraulic_left", "attachement_left");
		MakeHydraulic (armNode, "armHydraulicPiston_right", "armHydraulic_right", "attachement_right");

		dModelNode* const buckectNode = MakeActuator(armNode, "frontBucket", 100.0f);
		m_frontBucket = (dCustomHingeActuator*)buckectNode->GetJoint();
		m_frontBucket->SetAngularRate(4.0f);

		MakeHydraulic (buckectNode, "frontBucketHydraulic001", "frontBucketHydraulicPiston001", "attachment_frontBucket001");
		MakeHydraulic (buckectNode, "frontBucketHydraulic002", "frontBucketHydraulicPiston002", "attachment_frontBucket002");
	}

	dTractorEngine* m_engine;
	dCustomDoubleHingeActuator* m_leftWheel;
	dCustomDoubleHingeActuator* m_rightWheel;
	dCustomHingeActuator* m_frontArm;
	dCustomHingeActuator* m_frontBucket;
	dTractorControls m_controls;
};


class ServoVehicleManagerManager: public dModelManager
{
	public:
	ServoVehicleManagerManager(DemoEntityManager* const scene, int threadMaterialID)
		:dModelManager(scene->GetNewton())
		,m_player(NULL)
		,m_frontArmAngle(0.0f)
		,m_frontBucketAngle(0.0f)
	{
		scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
		scene->Set2DDisplayRenderFunction(RenderPlayerHelp, NULL, this);

		// create a material for early collision culling
		NewtonWorld* const world = scene->GetNewton();
		int material = NewtonMaterialGetDefaultGroupID(world);

		NewtonMaterialSetCallbackUserData(world, material, material, this);
		NewtonMaterialSetDefaultElasticity(world, material, material, 0.1f);
		NewtonMaterialSetDefaultFriction(world, material, material, 0.9f, 0.9f);
	}

	static void UpdateCameraCallback(DemoEntityManager* const manager, void* const context, dFloat timestep)
	{
		ServoVehicleManagerManager* const me = (ServoVehicleManagerManager*)context;
		me->UpdateCamera(timestep);
	}

	static void RenderPlayerHelp(DemoEntityManager* const scene, void* const context)
	{
		ServoVehicleManagerManager* const me = (ServoVehicleManagerManager*)context;
		me->RenderPlayerHelp(scene);
	}

	void RenderPlayerHelp(DemoEntityManager* const scene)
	{
		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "Navigation Keys");
		scene->Print(color, "drive forward:      w");
		scene->Print(color, "drive backward:     s");
		scene->Print(color, "turn right:         d");
		scene->Print(color, "turn left:          a");

		scene->Print(color, "front bucket controls");
		ImGui::SliderFloat("front arm angle", &m_frontArmAngle, 0.0f, 60.0f);
		ImGui::SliderFloat("front bucket angle", &m_frontBucketAngle, 0.0f, 130.0f);
	}

	void UpdateCamera(dFloat timestep)
	{
		if (!m_player) {
			return;
		}

		DemoEntity* const player = (DemoEntity*)NewtonBodyGetUserData(m_player->GetBody());
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());
		DemoCamera* const camera = scene->GetCamera();
		dMatrix camMatrix(camera->GetNextMatrix());
		dMatrix playerMatrix(player->GetNextMatrix());

		dVector frontDir(camMatrix[0]);
		dVector camOrigin(playerMatrix.m_posit + dVector(0.0f, 2.0f, 0.0f, 0.0f));
		camOrigin -= frontDir.Scale(D_TRACTOR_CAMERA_DIST);
		camera->SetNextMatrix(*scene, camMatrix, camOrigin);
	}

	dModelRootNode* CreateTractor(const char* const modelName, const dMatrix& location)
	{
		dTractorModel* const tractor = new dTractorModel(GetWorld(), modelName, location);

		// the the model to calculate the local transformation
		tractor->SetTransformMode(true);

		// add the model to the manager
		AddRoot(tractor);

		m_player = tractor;
		return tractor;
	}

/*
	virtual void OnDebug(dModelRootNode* const model, dCustomJoint::dDebugDisplay* const debugContext)
	{
		dExcavatorModel* const excavator = (dExcavatorModel*)model;
		excavator->OnDebug(debugContext);
	}
*/

	virtual void OnPreUpdate(dModelRootNode* const model, dFloat timestep) const
	{
		dTractorModel* const tractor = (dTractorModel*)model;

		NewtonWorld* const world = NewtonBodyGetWorld(tractor->GetBody());
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		dTractorControls controls;
		controls.m_throttle = (dFloat(scene->GetKeyState('W')) - dFloat(scene->GetKeyState('S')));
		controls.m_steeringValue = (dFloat(scene->GetKeyState('A')) - dFloat(scene->GetKeyState('D')));
		controls.m_frontArmAngle = m_frontArmAngle;
		controls.m_frontBucketAngle = m_frontBucketAngle;

		tractor->ApplyControl(controls, timestep);
	}

	virtual void OnUpdateTransform(const dModelNode* const bone, const dMatrix& localMatrix) const
	{
		NewtonBody* const body = bone->GetBody();
		DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(body);
		if (ent) {
			DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));

			dQuaternion rot(localMatrix);
			ent->SetMatrix(*scene, rot, localMatrix.m_posit);
		}
	}

	dTractorModel* m_player;
	dFloat32 m_frontArmAngle;
	dFloat32 m_frontBucketAngle;
};

void ServoJoints (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	//NewtonBody* const floor = CreateLevelMesh (scene, "flatPlane.ngd", true);
	CreateLevelMesh (scene, "flatPlane.ngd", true);
	//CreateHeightFieldTerrain (scene, 9, 8.0f, 1.5f, 0.2f, 200.0f, -50.0f);
	//NewtonCollision* const floorCollision = NewtonBodyGetCollision(floor);

	ServoVehicleManagerManager* const vehicleManager = new ServoVehicleManagerManager (scene, 0);

	NewtonWorld* const world = scene->GetNewton();
	dVector origin(FindFloor(world, dVector(-10.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = FindFloor(world, origin, 100.0f);
	matrix.m_posit.m_y += 1.5f;
	vehicleManager->CreateTractor("tractor.ngd", matrix);
/*
	NewtonWorld* const world = scene->GetNewton();
	dVector origin (FindFloor (world, dVector (-10.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));

	// add an input Manage to manage the inputs and user interaction 
	ServoInputManager* const inputManager = new ServoInputManager (scene);
	inputManager;

	//  create a skeletal transform controller for controlling rag doll
	ServoVehicleManagerManager* const vehicleManager = new ServoVehicleManagerManager (scene);

	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = FindFloor (world, origin, 100.0f);
	matrix.m_posit.m_y += 0.5f;
	
	// load a the mesh of the articulate vehicle
	dModelRootNode* const forklift = vehicleManager->CreateForklift(matrix, "forklift.ngd", sizeof(inverseKinematicsRidParts) / sizeof (inverseKinematicsRidParts[0]), inverseKinematicsRidParts);
	inputManager->AddPlayer(forklift);
*/

#if 1
	//place heavy load to show reproduce black bird dream problems
	matrix.m_posit.m_x += 2.0f;	
	matrix.m_posit.m_z -= 2.0f;	
//	MakeHeavyLoad (scene, matrix);

	// add some object to play with
	LoadLumberYardMesh(scene, dVector(5.0f, 0.0f, 0.0f, 0.0f), 0);
	LoadLumberYardMesh(scene, dVector(5.0f, 0.0f, 6.0f, 0.0f), 0);
	LoadLumberYardMesh(scene, dVector(10.0f, 0.0f, -4.0f, 0.0f), 0);
	LoadLumberYardMesh(scene, dVector(10.0f, 0.0f,  2.0f, 0.0f), 0);
	LoadLumberYardMesh(scene, dVector(15.0f, 0.0f, 0.0f, 0.0f), 0);
	LoadLumberYardMesh(scene, dVector(15.0f, 0.0f, 6.0f, 0.0f), 0);
#endif	
	origin.m_x -= 10.0f;
	origin.m_y += 4.0f;
	//origin.m_z = 6.0f;
	dQuaternion rot (dVector (0.0f, 1.0f, 0.0f, 0.0f), 90.0f * dDegreeToRad);  
	scene->SetCameraMatrix(rot, origin);
}



