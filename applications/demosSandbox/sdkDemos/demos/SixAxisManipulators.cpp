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

struct dArmRobotConfig
{
	char m_partName[32];
	dFloat m_mass;
	dFloat m_minLimit;
	dFloat m_maxLimit;
};

static dArmRobotConfig armRobotConfig[] =
{
	{ "bone_base", 210.0f, -1000.0f, 1000.0f},
	{ "bone_base1", 200.0f, -1000.0f, 1000.0f},
	{ "bone_arm0", 180.0f, -45.0f, 135.0f},
	{ "bone_arm1", 160.0f, -90.0f, 35.0f},
	{ "effector_arm", 1000.0f, 0.0f, 0.0f}
};

#if 0
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
			m_euler.m_x = pitch;
			m_euler.m_y = azimuth;
			m_euler.m_z = roll;
		}

		virtual void Evaluate(dEffectorPose& output, dFloat timestep)
		{
			m_poseGenerator->Evaluate(output, timestep);

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

	class dKukaEffector: public dCustomInverseDynamicsEffector
	{
		public:
		dKukaEffector(NewtonInverseDynamics* const invDynSolver, void* const invDynNode, NewtonBody* const referenceBody, const dMatrix& attachmentMatrixInGlobalSpace)
			:dCustomInverseDynamicsEffector(invDynSolver, invDynNode, referenceBody, attachmentMatrixInGlobalSpace)
		{
			SetLinearSpeed(2.0f);
			SetMaxLinearFriction (armRobotConfig[4].m_mass * DEMO_GRAVITY * 50.0f);
		}
	};

	dSixAxisController()
		:m_animTreeNode(NULL)
		,m_kinematicSolver(NULL)
		,m_inputModifier(NULL)
	{
	}

	~dSixAxisController()
	{
		delete m_animTreeNode;
		NewtonInverseDynamicsDestroy (m_kinematicSolver);
	}

	void SetTarget (dFloat x, dFloat y, dFloat azimuth, dFloat pitch, dFloat roll)
	{
		if (m_inputModifier) {
			m_inputModifier->SetTarget(x, y, azimuth, pitch, roll);
		}
	}

	DemoEntity* FindMesh(const DemoEntity* const bodyPart) const
	{
		for (DemoEntity* child = bodyPart->GetChild(); child; child = child->GetSibling()) {
			if (child->GetMesh()) {
				return child;
			}
		}
		dAssert(0);
		return NULL;
	}

	NewtonCollision* MakeConvexHull(const DemoEntity* const bodyPart) const
	{
		dVector points[1024 * 16];

		const DemoEntity* const meshEntity = FindMesh(bodyPart);

		DemoMesh* const mesh = (DemoMesh*)meshEntity->GetMesh();
		dAssert(mesh->IsType(DemoMesh::GetRttiType()));
		dAssert(mesh->m_vertexCount && (mesh->m_vertexCount < int(sizeof(points) / sizeof(points[0]))));

		// go over the vertex array and find and collect all vertices's weighted by this bone.
		const dFloat* const array = mesh->m_vertex;
		for (int i = 0; i < mesh->m_vertexCount; i++) {
			points[i][0] = array[i * 3 + 0];
			points[i][1] = array[i * 3 + 1];
			points[i][2] = array[i * 3 + 2];
			points[i][3] = 0.0f;
		}
		dMatrix matrix(meshEntity->GetMeshMatrix());
		matrix = matrix * meshEntity->GetCurrentMatrix();
		//matrix = matrix * bodyPart->GetParent()->GetCurrentMatrix();
		matrix.TransformTriplex(&points[0][0], sizeof(dVector), &points[0][0], sizeof(dVector), mesh->m_vertexCount);

		NewtonWorld* const world = GetManager()->GetWorld();
		return NewtonCreateConvexHull(world, mesh->m_vertexCount, &points[0][0], sizeof(dVector), 1.0e-3f, 0, NULL);
	}


	NewtonBody* CreateBodyPart(DemoEntity* const bodyPart, const dArmRobotConfig& definition)
	{
		NewtonCollision* const shape = MakeConvexHull(bodyPart);

		// calculate the bone matrix
		dMatrix matrix(bodyPart->CalculateGlobalMatrix());

		NewtonWorld* const world = GetManager()->GetWorld();

		// create the rigid body that will make this bone
		NewtonBody* const body = NewtonCreateDynamicBody(world, shape, &matrix[0][0]);

		// destroy the collision helper shape 
		NewtonDestroyCollision(shape);

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(body);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonBodySetMassProperties(body, definition.m_mass, collision);

		// save the user lifterData with the bone body (usually the visual geometry)
		NewtonBodySetUserData(body, bodyPart);

		// assign a body part id
		//NewtonCollisionSetUserID(collision, definition.m_bodyPartID);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);
		return body;
	}

	void MakeKukaRobot(DemoEntityManager* const scene, DemoEntity* const model)
	{
		m_kinematicSolver = NewtonCreateInverseDynamics(scene->GetNewton());

		NewtonBody* const baseFrameBody = CreateBodyPart(model, armRobotConfig[0]);
		void* const baseFrameNode = NewtonInverseDynamicsAddRoot(m_kinematicSolver, baseFrameBody);
		NewtonBodySetMassMatrix(baseFrameBody, 0.0f, 0.0f, 0.0f, 0.0f);	

		dMatrix boneAligmentMatrix(
			dVector(0.0f, 1.0f, 0.0f, 0.0f),
			dVector(0.0f, 0.0f, 1.0f, 0.0f),
			dVector(1.0f, 0.0f, 0.0f, 0.0f),
			dVector(0.0f, 0.0f, 0.0f, 1.0f));

		int stackIndex = 0;
		DemoEntity* childEntities[32];
		void* parentBones[32];
		for (DemoEntity* child = model->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = baseFrameNode;
			childEntities[stackIndex] = child;
			stackIndex++;
		}

		dKukaEffector* effector = NULL;
		const int partCount = sizeof(armRobotConfig) / sizeof(armRobotConfig[0]);
		while (stackIndex) {
			stackIndex--;
			DemoEntity* const entity = childEntities[stackIndex];
			void* const parentJoint = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			for (int i = 0; i < partCount; i++) {
				if (!strcmp(armRobotConfig[i].m_partName, name)) {

					if (strstr(name, "bone")) {
						// add a bone and all it children
						dMatrix matrix;
						NewtonBody* const limbBody = CreateBodyPart(entity, armRobotConfig[i]);
						NewtonBodyGetMatrix(limbBody, &matrix[0][0]);

						NewtonBody* const parentBody = NewtonInverseDynamicsGetBody(m_kinematicSolver, parentJoint);
						dCustomInverseDynamics* const rotatingColumnHinge = new dCustomInverseDynamics(boneAligmentMatrix * matrix, limbBody, parentBody);
						rotatingColumnHinge->SetJointTorque(armRobotConfig[i].m_mass * DEMO_GRAVITY * 50.0f);
						rotatingColumnHinge->SetTwistAngle(armRobotConfig[i].m_minLimit * dDegreeToRad, armRobotConfig[i].m_maxLimit * dDegreeToRad);
						void* const limbJoint = NewtonInverseDynamicsAddChildNode(m_kinematicSolver, parentJoint, rotatingColumnHinge->GetJoint());

						for (DemoEntity* child = entity->GetChild(); child; child = child->GetSibling()) {
							parentBones[stackIndex] = limbJoint;
							childEntities[stackIndex] = child;
							stackIndex++;
						}
					} else if (strstr(name, "effector")) {
						// add effector
						dMatrix gripperMatrix(entity->CalculateGlobalMatrix());
						effector = new dKukaEffector(m_kinematicSolver, parentJoint, baseFrameBody, gripperMatrix);
						effector->SetAsThreedof();
						effector->SetLinearSpeed(2.0f);
						effector->SetMaxLinearFriction(armRobotConfig[i].m_mass * DEMO_GRAVITY * 50.0f);
					}
					break;
				}
			}
		}

		// create the Animation tree for manipulation 
		DemoEntity* const effectoBone = model->Find("effector_arm");
		dMatrix baseFrameMatrix(model->GetCurrentMatrix());
		dMatrix effectorLocalMatrix(effectoBone->CalculateGlobalMatrix(model));

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

	void PreUpdate(dFloat timestep, int threadIndex)
	{
		if (m_inputModifier) {
			m_animTreeNode->Update(timestep);
		}
		NewtonInverseDynamicsUpdate(m_kinematicSolver, timestep, threadIndex);
	}

	void PostUpdate(dFloat timestep, int threadIndex)
	{
		int count = 1;
		void* nodes[32];
		dMatrix parentMatrix[32];
		nodes[0] = NewtonInverseDynamicsGetRoot(m_kinematicSolver);

		parentMatrix[0] = dGetIdentityMatrix();

		NewtonWorld* const world = GetManager()->GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		while (count) {
			dMatrix matrix;

			count --;
			void* const rootNode = nodes[count];
			NewtonBody* const body = NewtonInverseDynamicsGetBody(m_kinematicSolver, rootNode);
			NewtonBodyGetMatrix (body, &matrix[0][0]);
			dMatrix localMatrix (matrix * parentMatrix[count]);

			DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(body);

			dQuaternion rot(localMatrix);
			ent->SetMatrix(*scene, rot, localMatrix.m_posit);

			matrix = matrix.Inverse();
			for (void* node = NewtonInverseDynamicsGetFirstChildNode(m_kinematicSolver, rootNode); node; node = NewtonInverseDynamicsGetNextChildNode(m_kinematicSolver, node)) {
				nodes[count] = node;
				parentMatrix[count] = matrix;
				count ++;
			}
		}
	}

	void Debug(dCustomJoint::dDebugDisplay* const debugContext) const
	{
		if (m_animTreeNode) {
			const dEffectorTreeInterface::dEffectorPose& pose = m_animTreeNode->GetPose();
			for (dEffectorTreeInterface::dEffectorPose::dListNode* node = pose.GetFirst(); node; node = node->GetNext()) {
				dCustomInverseDynamicsEffector* const effector = node->GetInfo().m_effector;
				effector->Debug(debugContext);
			}
		}
	}

	dEffectorTreeRoot* m_animTreeNode;
	NewtonInverseDynamics* m_kinematicSolver;
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
			controller->SetTarget (me->m_posit_x, me->m_posit_y, me->m_azimuth * dDegreeToRad, me->m_gripper_pitch * dDegreeToRad, me->m_gripper_roll * dDegreeToRad);
		}
	}

	virtual dSixAxisController* CreateController()
	{
		return (dSixAxisController*)dCustomControllerManager<dSixAxisController>::CreateController();
	}

	dSixAxisController* MakeKukaRobot(DemoEntityManager* const scene, const dMatrix& origin)
	{
		DemoEntity* const model = DemoEntity::LoadNGD_mesh("robotArm.ngd", scene->GetNewton());
		scene->Append(model);
		model->ResetMatrix(*scene, origin);

		dSixAxisController* const controller = (dSixAxisController*)CreateController();
		controller->MakeKukaRobot(scene, model);
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

	dMatrix origin(dYawMatrix(0.0f * dDegreeToRad));
	dMatrix origin1(dYawMatrix(180.0f * dDegreeToRad));
	origin.m_posit.m_z = -1.0f;
	origin1.m_posit.m_z =  1.0f;

	int count = 10;
count = 1;
origin = dGetIdentityMatrix();
origin.m_posit.m_x = 2.0f;
	for (int i = 0; i < count; i ++) {
		robotManager->MakeKukaRobot (scene, origin);
//		robotManager->MakeKukaRobot (scene, origin1);
		origin.m_posit.m_x += 1.0f;
		origin1.m_posit.m_x += 1.0f;
	}
	
	origin.m_posit = dVector (-3.0f, 0.5f, 0.0f, 1.0f);
origin.m_posit = dVector(-2.0f, 0.5f, 0.0f, 1.0f);
	scene->SetCameraMatrix(dGetIdentityMatrix(), origin.m_posit);
}

#else


#if 0
class dSixAxisManager: public dAnimIDManager
{
	public:
	dSixAxisManager(DemoEntityManager* const scene)
		:dAnimIDManager(scene->GetNewton())
//		,m_currentController(NULL)
//		,m_azimuth(0.0f)
//		,m_posit_x(0.0f)
//		,m_posit_y(0.0f)
//		,m_gripper_roll(0.0f)
//		,m_gripper_pitch(0.0f)
	{
//		scene->Set2DDisplayRenderFunction(RenderHelpMenu, NULL, this);
	}

	~dSixAxisManager()
	{
	}
/*
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
			controller->SetTarget(me->m_posit_x, me->m_posit_y, me->m_azimuth * dDegreeToRad, me->m_gripper_pitch * dDegreeToRad, me->m_gripper_roll * dDegreeToRad);
		}
	}

	virtual dSixAxisController* CreateController()
	{
		return (dSixAxisController*)dCustomControllerManager<dSixAxisController>::CreateController();
	}

	void OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
	{
		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
			dSixAxisController* const controller = &node->GetInfo();
			controller->Debug(debugContext);
		}
	}
*/

	DemoEntity* FindMesh(const DemoEntity* const bodyPart) const
	{
		for (DemoEntity* child = bodyPart->GetChild(); child; child = child->GetSibling()) {
			if (child->GetMesh()) {
				return child;
			}
		}
		dAssert(0);
		return NULL;
	}

	NewtonCollision* MakeConvexHull(const DemoEntity* const bodyPart) const
	{
		dVector points[1024 * 16];

		const DemoEntity* const meshEntity = FindMesh(bodyPart);

		DemoMesh* const mesh = (DemoMesh*)meshEntity->GetMesh();
		dAssert(mesh->IsType(DemoMesh::GetRttiType()));
		dAssert(mesh->m_vertexCount && (mesh->m_vertexCount < int(sizeof(points) / sizeof(points[0]))));

		// go over the vertex array and find and collect all vertices's weighted by this bone.
		const dFloat* const array = mesh->m_vertex;
		for (int i = 0; i < mesh->m_vertexCount; i++) {
			points[i][0] = array[i * 3 + 0];
			points[i][1] = array[i * 3 + 1];
			points[i][2] = array[i * 3 + 2];
			points[i][3] = 0.0f;
		}
		dMatrix matrix(meshEntity->GetMeshMatrix());
		matrix = matrix * meshEntity->GetCurrentMatrix();
		//matrix = matrix * bodyPart->GetParent()->GetCurrentMatrix();
		matrix.TransformTriplex(&points[0][0], sizeof(dVector), &points[0][0], sizeof(dVector), mesh->m_vertexCount);
//		return NewtonCreateConvexHull(GetWorld(), mesh->m_vertexCount, &points[0][0], sizeof(dVector), 1.0e-3f, SERVO_VEHICLE_DEFINITION::m_bodyPart, NULL);
		return NewtonCreateConvexHull(GetWorld(), mesh->m_vertexCount, &points[0][0], sizeof(dVector), 1.0e-3f, 0, NULL);
	}

	NewtonBody* CreateBodyPart(DemoEntity* const bodyPart, const dArmRobotConfig& definition)
	{
		NewtonCollision* const shape = MakeConvexHull(bodyPart);

		// calculate the bone matrix
		dMatrix matrix(bodyPart->CalculateGlobalMatrix());

		NewtonWorld* const world = GetWorld();

		// create the rigid body that will make this bone
		NewtonBody* const body = NewtonCreateDynamicBody(world, shape, &matrix[0][0]);

		// destroy the collision helper shape 
		NewtonDestroyCollision(shape);

		// get the collision from body
		NewtonCollision* const collision = NewtonBodyGetCollision(body);

		// calculate the moment of inertia and the relative center of mass of the solid
		NewtonBodySetMassProperties(body, definition.m_mass, collision);
//NewtonBodySetMassProperties(body, 0.0f, collision);

		// save the user lifterData with the bone body (usually the visual geometry)
		NewtonBodySetUserData(body, bodyPart);

		// assign a body part id
		//NewtonCollisionSetUserID(collision, definition.m_bodyPartID);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);
		return body;
	}

	dAnimIDController* MakeKukaRobot(DemoEntityManager* const scene, const dMatrix& origin)
	{
		DemoEntity* const model = DemoEntity::LoadNGD_mesh("robotArm.ngd", scene->GetNewton(), scene->GetShaderCache());
		scene->Append(model);
		model->ResetMatrix(*scene, origin);

		NewtonBody* const rootBody = CreateBodyPart(model, armRobotConfig[0]);
		NewtonBodySetMassMatrix(rootBody, 0.0f, 0.0f, 0.0f, 0.0f);
dAssert(0);
return NULL;
/*
		dAnimIDController* const rig = CreateCharacterRig (rootBody);

		int stackIndex = 0;
		DemoEntity* childEntities[32];
		dAnimIDRigJoint* parentBones[32];
		for (DemoEntity* child = model->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = rig;
			childEntities[stackIndex] = child;
			stackIndex++;
		}

		const int partCount = sizeof(armRobotConfig) / sizeof(armRobotConfig[0]);
		while (stackIndex) {
			stackIndex--;
			DemoEntity* const entity = childEntities[stackIndex];
			dAnimIDRigJoint* const parentJoint = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			for (int i = 0; i < partCount; i++) {
				if (!strcmp(armRobotConfig[i].m_partName, name)) {

					if (strstr (name, "bone")) {
						// add a bone and all it children
						NewtonBody* const limbBody = CreateBodyPart(entity, armRobotConfig[i]);

						dMatrix matrix;
						NewtonBodyGetMatrix(limbBody, &matrix[0][0]);
						dAnimationRigHinge* const limbJoint = new dAnimationRigHinge(matrix, parentJoint, limbBody);

						limbJoint->SetFriction(armRobotConfig[i].m_mass * DEMO_GRAVITY * 50.0f);
						limbJoint->SetLimits(armRobotConfig[i].m_minLimit * dDegreeToRad, armRobotConfig[i].m_maxLimit * dDegreeToRad);

						for (DemoEntity* child = entity->GetChild(); child; child = child->GetSibling()) {
							parentBones[stackIndex] = limbJoint;
							childEntities[stackIndex] = child;
							stackIndex++;
						}
					} else if (strstr(name, "effector")) {
						// add an end effector (end effector can't have children)
						dMatrix pivot (entity->CalculateGlobalMatrix());
						dAnimationRigEffector* const effector = new dAnimationRigEffector(parentJoint->GetAsRigLimb(), pivot);
						effector->SetLinearSpeed(2.0f);
						effector->SetMaxLinearFriction(armRobotConfig[i].m_mass * DEMO_GRAVITY * 50.0f);
					}
					break;
				}
			}
		}

		rig->Finalize();
		return rig;
*/
	}

	void OnUpdateTransform (const dAnimIDRigJoint* const bone, const dMatrix& localMatrix) const
	{
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(GetWorld());
		NewtonBody* const newtonBody = bone->GetNewtonBody();
		DemoEntity* const meshEntity = (DemoEntity*)NewtonBodyGetUserData(newtonBody);
		
		dQuaternion rot(localMatrix);
		meshEntity->SetMatrix(*scene, rot, localMatrix.m_posit);
	}

/*
	dSixAxisController* m_currentController;
	dFloat32 m_azimuth;
	dFloat32 m_posit_x;
	dFloat32 m_posit_y;
	dFloat32 m_gripper_roll;
	dFloat32 m_gripper_pitch;
*/
};

#endif
void SixAxisManipulators(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	dTrace(("sorry demo %s temporarilly disabled\n", __FUNCTION__));
	return;
/*

	CreateLevelMesh(scene, "flatPlane.ngd", true);
	dSixAxisManager* const robotManager = new dSixAxisManager(scene);

	dMatrix origin(dYawMatrix(0.0f * dDegreeToRad));
	dMatrix origin1(dYawMatrix(180.0f * dDegreeToRad));
	origin.m_posit.m_z = -1.0f;
	origin1.m_posit.m_z = 1.0f;

	int count = 10;
count = 1;
origin = dGetIdentityMatrix();
origin.m_posit.m_x = 2.0f;
	for (int i = 0; i < count; i++) {
		robotManager->MakeKukaRobot(scene, origin);
		//robotManager->MakeKukaRobot (scene, origin1);
		origin.m_posit.m_x += 1.0f;
		origin1.m_posit.m_x += 1.0f;
	}

	//	origin.m_posit = dVector (-3.0f, 0.5f, 0.0f, 1.0f);
	origin.m_posit = dVector(-2.0f, 0.5f, 0.0f, 1.0f);
	scene->SetCameraMatrix(dGetIdentityMatrix(), origin.m_posit);
*/
}

#endif
