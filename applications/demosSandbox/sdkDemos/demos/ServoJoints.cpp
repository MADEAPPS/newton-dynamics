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


class dTractorControls
{
	public:
	dTractorControls()
	{
		memset(this, 0, sizeof (dTractorControls));
	}
	dFloat m_throttle;
	dFloat m_steeringValue;
//	dFloat m_bucket_x;
//	dFloat m_bucket_y;
//	dFloat m_bucket_angle;
//	int m_cabinSpeed;
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
	{
		MakeChassis(world, modelName, location);

		MakeDriveTrain();
	}

	void ApplyControl (const dTractorControls& controls, dFloat timestep)
	{
		NewtonBodySetSleepState(GetBody(), 0);

		m_engine->m_alpha = controls.m_throttle;
		m_leftWheel->SetTargetAngle1(m_controls.m_steeringValue * 25.0f * dDegreeToRad);
		m_rightWheel->SetTargetAngle1(-m_controls.m_steeringValue * 25.0f * dDegreeToRad);

		m_controls = controls;
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
		com.m_y -= 0.5f;
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
//diffMatrix.m_posit.m_y += 1.0f;

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
//engineMatrix.m_posit.m_y += 2.0f;

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

		//dMatrix matrix;
		//dMatrix rearDiffMatrix;
		//rearDiff->GetJoint()->CalculateGlobalMatrix(rearDiffMatrix,  matrix);
		new dDoubleDifferentialGear(5.0f, (dCustomDoubleHinge*)rearDiff->GetJoint(), 1.0f, m_engine);

		//dMatrix frontDiffMatrix;
		//fronfDiff->GetJoint()->CalculateGlobalMatrix(frontDiffMatrix, matrix);
		new dDoubleDifferentialGear(5.0f, (dCustomDoubleHinge*)fronfDiff->GetJoint(), -1.0f, m_engine);
	}

	dTractorEngine* m_engine;
	dCustomDoubleHingeActuator* m_leftWheel;
	dCustomDoubleHingeActuator* m_rightWheel;
	dTractorControls m_controls;
};


class ServoVehicleManagerManager: public dModelManager
{
	public:
	ServoVehicleManagerManager(DemoEntityManager* const scene, int threadMaterialID)
		:dModelManager(scene->GetNewton())
		,m_player(NULL)
		//,m_threadMaterialID(threadMaterialID)
		//,m_cabinSpeed(0)
		//,m_bucket_x(0.0f)
		//,m_bucket_y(0.0f)
		//,m_bucket_angle(0.0f)
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
/*
		scene->Print(color, "bucket controls");
		ImGui::SliderInt("cabin rotation", &m_cabinSpeed, -3, 3);
		ImGui::SliderFloat("bucket x", &m_bucket_x, -2.0f, 4.0f);
		ImGui::SliderFloat("bucket y", &m_bucket_y, -6.0f, 6.0f);
		ImGui::SliderFloat("bucket angle", &m_bucket_angle, -60.0f, 130.0f);
*/
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
		camOrigin -= frontDir.Scale(7.0f);
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

	static int StandardAABBOverlapTest(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
	{
		const NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
		const NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);
		const NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
		const NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);

		if (NewtonCollisionGetUserData(collision0) != NewtonCollisionGetUserData(collision1)) {
			return 1;
		}

		NewtonCollisionMaterial material0;
		NewtonCollisionMaterial material1;
		NewtonCollisionGetMaterial(collision0, &material0);
		NewtonCollisionGetMaterial(collision1, &material1);

		//m_terrain	 = 1 << 0,
		//m_bodyPart = 1 << 1,
		//m_tirePart = 1 << 2,
		//m_linkPart = 1 << 3,
		//m_propBody = 1 << 4,

		const dLong mask0 = material0.m_userId & material1.m_userParam[0].m_int;
		const dLong mask1 = material1.m_userId & material0.m_userParam[0].m_int;
		return (mask0 && mask1) ? 1 : 0;
	}

	static int ThreadStaticContactsGeneration(const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonCollision* const collision0, const NewtonBody* const body1, const NewtonCollision* const collision1, NewtonUserContactPoint* const contactBuffer, int maxCount, int threadIndex)
	{
		dAssert(NewtonBodyGetMaterialGroupID(body0) == NewtonBodyGetMaterialGroupID(body1));
		dAssert(NewtonBodyGetMaterialGroupID(body0) != NewtonMaterialGetDefaultGroupID(NewtonBodyGetWorld(body0)));
		dAssert(NewtonBodyGetMaterialGroupID(body1) != NewtonMaterialGetDefaultGroupID(NewtonBodyGetWorld(body1)));

		dAssert(NewtonCollisionGetUserID(collision0) == ARTICULATED_VEHICLE_DEFINITION::m_linkPart);
		dAssert(NewtonCollisionGetUserID(collision1) == ARTICULATED_VEHICLE_DEFINITION::m_terrain);

		dExcavatorModel* const excavator = (dExcavatorModel*)NewtonCollisionGetUserData(collision0);
		dAssert(excavator);
		return excavator->CollideLink(material, body0, body1, contactBuffer);
	}

	
	int m_threadMaterialID;
	int m_cabinSpeed;
	dFloat32 m_bucket_x;
	dFloat32 m_bucket_y;
	dFloat32 m_bucket_angle;
*/

	virtual void OnPreUpdate(dModelRootNode* const model, dFloat timestep) const
	{
		dTractorModel* const tractor = (dTractorModel*)model;

		NewtonWorld* const world = NewtonBodyGetWorld(tractor->GetBody());
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		dTractorControls controls;
		controls.m_throttle = (dFloat(scene->GetKeyState('W')) - dFloat(scene->GetKeyState('S')));
		controls.m_steeringValue = (dFloat(scene->GetKeyState('A')) - dFloat(scene->GetKeyState('D')));
		//controls.m_cabinSpeed = m_cabinSpeed;
		//controls.m_bucket_x = m_bucket_x;
		//controls.m_bucket_y = m_bucket_y;
		//controls.m_bucket_angle = m_bucket_angle;

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
};

void ServoJoints (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	NewtonBody* const floor = CreateLevelMesh (scene, "flatPlane.ngd", true);
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
//	LoadLumberYardMesh(scene, dVector(5.0f, 0.0f, 0.0f, 0.0f), 0);
//	LoadLumberYardMesh(scene, dVector(5.0f, 0.0f, 6.0f, 0.0f), 0);
//	LoadLumberYardMesh(scene, dVector(10.0f, 0.0f, -4.0f, 0.0f), 0);
//	LoadLumberYardMesh(scene, dVector(10.0f, 0.0f,  2.0f, 0.0f), 0);
//	LoadLumberYardMesh(scene, dVector(15.0f, 0.0f, 0.0f, 0.0f), 0);
//	LoadLumberYardMesh(scene, dVector(15.0f, 0.0f, 6.0f, 0.0f), 0);
#endif	
	origin.m_x -= 10.0f;
	origin.m_y += 4.0f;
	//origin.m_z = 6.0f;
	dQuaternion rot (dVector (0.0f, 1.0f, 0.0f, 0.0f), 0.0f * dDegreeToRad);  
	scene->SetCameraMatrix(rot, origin);
}



