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


#if 0

struct dArmRobotConfig
{
	char m_partName[32];
	dFloat m_mass;
	dFloat m_minLimit;
	dFloat m_maxLimit;
};

static dArmRobotConfig armRobotConfig[] =
{
	{ "bone_base", 210.0f, -1000.0f, 1000.0f },
	{ "bone_base1", 200.0f, -1000.0f, 1000.0f },
	{ "bone_arm0", 180.0f, -45.0f, 135.0f },
	{ "bone_arm1", 160.0f, -90.0f, 35.0f },
	{ "effector_arm", 1000.0f, 0.0f, 0.0f }
};

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

	void MakeSixAxisRobot(DemoEntityManager* const scene, DemoEntity* const model)
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

	dSixAxisController* MakeSixAxisRobot(DemoEntityManager* const scene, const dMatrix& origin)
	{
		DemoEntity* const model = DemoEntity::LoadNGD_mesh("robotArm.ngd", scene->GetNewton(), scene->GetShaderCache());

		scene->Append(model);
		model->ResetMatrix(*scene, origin);

DemoEntity* const model1 = DemoEntity::LoadNGD_mesh("robot2.ngd", scene->GetNewton(), scene->GetShaderCache());
scene->Append(model1);
model1->ResetMatrix(*scene, origin);

		dSixAxisController* const controller = (dSixAxisController*)CreateController();
		controller->MakeSixAxisRobot(scene, model);
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

#else


class dRobotJointDefinition
{
	public:
	struct dJointLimit
	{
		dFloat m_minTwistAngle;
		dFloat m_maxTwistAngle;
		dFloat m_coneAngle;
	};

	struct dFrameMatrix
	{
		dFloat m_pitch;
		dFloat m_yaw;
		dFloat m_roll;
	};

	char m_boneName[32];
	dFloat m_friction;
	dJointLimit m_jointLimits;
	dFrameMatrix m_frameBasics;
	dAnimationRagdollJoint::dRagdollMotorType m_type;
};


static dRobotJointDefinition robot2Definition[] =
{
	{ "bone_base000" },

	{ "bone_base001", 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 90.0f, 0.0f }, dAnimationRagdollJoint::m_threeDof },
//	{ "bone_righCalf", 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 90.0f }, dAnimationRagdollJoint::m_oneDof },
//	{ "bone_rightAnkle", 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 90.0f }, dAnimationRagdollJoint::m_zeroDof },
//	{ "bone_rightFoot", 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, -90.0f, 0.0f }, dAnimationRagdollJoint::m_twoDof },

//	{ "bone_leftLeg", 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 90.0f, 0.0f }, dAnimationRagdollJoint::m_threeDof },
//	{ "bone_leftCalf", 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 90.0f }, dAnimationRagdollJoint::m_oneDof },
//	{ "bone_leftAnkle", 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, 0.0f, 90.0f }, dAnimationRagdollJoint::m_zeroDof },
//	{ "bone_leftFoot", 100.0f,{ -15.0f, 15.0f, 30.0f },{ 0.0f, -90.0f, 0.0f }, dAnimationRagdollJoint::m_twoDof },
};

#if 0
class dSixAxisRobot: public dAnimationRagdollRoot
{
	public:
	dSixAxisRobot(NewtonBody* const body, const dMatrix& bindMarix)
		:dAnimationRagdollRoot(body, bindMarix)
//		,m_hipEffector(NULL)
//		,m_leftFootEffector__(NULL)
//		,m_rightFootEffector__(NULL)
	{
	}

	~dSixAxisRobot()
	{
	}

/*
	void SetHipEffector(dAnimationJoint* const hip)
	{
		m_hipEffector = new dKinematicEndEffector(hip);
		m_loopJoints.Append(m_hipEffector);
	}

	void SetLeftFootEffector(dAnimationJoint* const joint)
	{
		m_leftFootEffector__ = new dKinematicEndEffector(joint);
		m_loopJoints.Append(m_leftFootEffector__);
	}

	void SetRightFootEffector(dAnimationJoint* const joint)
	{
		m_rightFootEffector__ = new dKinematicEndEffector(joint);
		m_loopJoints.Append(m_rightFootEffector__);
	}

	dVector CalculateEffectorOrigin(dAnimationRagDollEffector* const hipEffector) const
	{
		return dVector(0.0f);
	}

	void SelftBalance()
	{
	}
*/

	void PreUpdate(dFloat timestep)
	{
//		dAssert(0);
/*
		// do most of the control here, so that the is no need do subclass  
		//dDynamicsRagdoll* const ragDoll = (dDynamicsRagdoll*)model;
		NewtonBody* const rootBody = GetBody();
		NewtonBodySetSleepState(rootBody, 0);

		// get pivot 
		dMatrix rootMatrix;
		//		dMatrix pivotMatrix;
		NewtonBodyGetMatrix(GetBody(), &rootMatrix[0][0]);

		dVector com(CalculateCenterOfMass());

		if (m_leftFootEffector__) {
			dMatrix footMatrix(m_leftFootEffector__->GetMatrix());
			m_leftFootEffector__->SetTarget(footMatrix);
		}

		if (m_rightFootEffector__) {
			dMatrix footMatrix(m_rightFootEffector__->GetMatrix());
			m_rightFootEffector__->SetTarget(footMatrix);
		}
*/

		/*
		NewtonBodyGetMatrix(ragDoll->m_leftFootEffector->GetBody(), &pivotMatrix[0][0]);

		dVector comRadius (com - pivotMatrix.m_posit);
		dVector bodyRadius (rootMatrix.m_posit - pivotMatrix.m_posit);

		dFloat r0 = dSqrt(comRadius.DotProduct3(comRadius));
		dFloat r1 = dSqrt(bodyRadius.DotProduct3(bodyRadius));

		dVector updir (0.0f, 1.0f, 0.0f, 0.0f);
		dVector p1 (pivotMatrix.m_posit + updir.Scale (r0));
		dVector targtePosit (rootMatrix.m_posit + (p1 - com).Scale (r1/r0));
		targtePosit.m_w = 1.0f;
		*/
		dMatrix matrix(dPitchMatrix(0.0f * dDegreeToRad) * dYawMatrix(0.0f * dDegreeToRad) * dRollMatrix(0.0f * dDegreeToRad));
		//com.m_y = rootMatrix.m_posit.m_y;
		//com.m_w = 1.0f;
		//matrix.m_posit = rootMatrix.m_posit;
		//matrix.m_posit = com;

		//matrix.m_posit = rootMatrix.m_posit;
		//m_hipEffector->SetTarget(matrix);

		//dVector error(matrix.m_posit - rootMatrix.m_posit);
		//dTrace(("%f %f %f\n", error[0], error[1], error[2]));

	}

//	dAnimationRagDollEffector* m_hipEffector;
//	dAnimationRagDollEffector* m_leftFootEffector__;
//	dAnimationRagDollEffector* m_rightFootEffector__;
};


class dSixAxisManager: public dAnimationModelManager
{
	public:
	dSixAxisManager(DemoEntityManager* const scene)
		:dAnimationModelManager(scene->GetNewton())
		//, m_currentRig(NULL)
	{
		//scene->Set2DDisplayRenderFunction(RenderHelpMenu, NULL, this);
	}

	~dSixAxisManager()
	{
	}

	virtual void OnUpdateTransform(const dAnimationJoint* const bone, const dMatrix& localMatrix) const
	{
		// calculate the local transform for this player body
		NewtonBody* const body = bone->GetBody();
		DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(body);
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));

		dQuaternion rot(localMatrix);
		ent->SetMatrix(*scene, rot, localMatrix.m_posit);
	}

	void MakeSixAxisRobot(DemoEntityManager* const scene, const dMatrix& origin)
	{
/*
		DemoEntity* const model = DemoEntity::LoadNGD_mesh("robotArm.ngd", scene->GetNewton(), scene->GetShaderCache());

		scene->Append(model);
		model->ResetMatrix(*scene, origin);

		DemoEntity* const model1 = DemoEntity::LoadNGD_mesh("robot2.ngd", scene->GetNewton(), scene->GetShaderCache());
		scene->Append(model1);
		model1->ResetMatrix(*scene, origin);

		dSixAxisController* const controller = (dSixAxisController*)CreateController();
		controller->MakeSixAxisRobot(scene, model);
		m_currentController = controller;
		return controller;
*/

		NewtonWorld* const world = GetWorld();
		dAssert (scene == (DemoEntityManager*)NewtonWorldGetUserData(world));

		DemoEntity* const modelEntity = DemoEntity::LoadNGD_mesh("robot2.ngd", world, scene->GetShaderCache());

		//dMatrix matrix0(modelEntity->GetCurrentMatrix());
		//matrix0.m_posit = location;
		modelEntity->ResetMatrix(*scene, origin);
		scene->Append(modelEntity);

		// add the root childBody
		NewtonBody* const rootBody = CreateBodyPart(modelEntity);
		dAssert(rootBody);

		// build the rag doll with rigid bodies connected by joints
		dSixAxisRobot* const robot = new dSixAxisRobot(rootBody, dGetIdentityMatrix());
		AddModel(robot);
		robot->SetCalculateLocalTransforms(true);

		// save the controller as the collision user data, for collision culling
		NewtonCollisionSetUserData(NewtonBodyGetCollision(rootBody), robot);

		int stackIndex = 0;
		DemoEntity* childEntities[32];
		dAnimationJoint* parentBones[32];

		for (DemoEntity* child = modelEntity->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = robot;
			childEntities[stackIndex] = child;
			stackIndex++;
		}
/*
		// walk model hierarchic adding all children designed as rigid body bones. 
		while (stackIndex) {
			stackIndex--;
			DemoEntity* const entity = childEntities[stackIndex];
			const char* const name = entity->GetName().GetStr();
			dTrace(("%s\n", name));
			dAnimationJoint* parentNode = parentBones[stackIndex];

			for (int i = 0; i < size; i++) {
				if (!strcmp(defintion[i].m_boneName, name)) {
					NewtonBody* const childBody = CreateBodyPart(entity);

					// connect this body part to its parent with a rag doll joint
					parentNode = CreateChildNode(childBody, parentNode, defintion[i]);

					NewtonCollisionSetUserData(NewtonBodyGetCollision(childBody), parentNode);
					break;
				}
			}

			for (DemoEntity* child = entity->GetChild(); child; child = child->GetSibling()) {
				parentBones[stackIndex] = parentNode;
				childEntities[stackIndex] = child;
				stackIndex++;
			}
		}

		SetModelMass(100.0f, robot);

		// transform the entire contraction to its location
		dMatrix worldMatrix(modelEntity->GetCurrentMatrix() * location);
		worldMatrix.m_posit = location.m_posit;
		NewtonBodySetMatrixRecursive(rootBody, &worldMatrix[0][0]);


		// attach effectors here
		for (dAnimationJoint* joint = GetFirstJoint(robot); joint; joint = GetNextJoint(joint)) {
			if (joint->GetAsRoot()) {
				dAssert(robot == joint);
				robot->SetHipEffector(joint);
			}
			else if (joint->GetAsLeaf()) {
				NewtonBody* const body = joint->GetBody();
				DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(body);
				if (entity->GetName().Find("left") != -1) {
					robot->SetLeftFootEffector(joint);
				}
				else if (entity->GetName().Find("right") != -1) {
					robot->SetRightFootEffector(joint);
				}
			}
		}


		robot->Finalize();
*/
	}

	private:

	static void ClampAngularVelocity(const NewtonBody* body, dFloat timestep, int threadIndex)
	{
		dVector omega;
		NewtonBodyGetOmega(body, &omega[0]);
		omega.m_w = 0.0f;
		dFloat mag2 = omega.DotProduct3(omega);
		if (mag2 > (100.0f * 100.0f)) {
			omega = omega.Normalize().Scale(100.0f);
			NewtonBodySetOmega(body, &omega[0]);
		}

		//PhysicsApplyGravityForce(body, timestep, threadIndex);
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		dFloat mass;

		dFloat gravity = -0.0f;
		NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
		dVector dir(0.0f, gravity, 0.0f);
		dVector force(dir.Scale(mass));
		NewtonBodySetForce(body, &force.m_x);
	}

	NewtonBody* CreateBodyPart(DemoEntity* const bodyPart)
	{
		NewtonWorld* const world = GetWorld();
		NewtonCollision* const shape = bodyPart->CreateCollisionFromchildren(world);
		dAssert(shape);

		// calculate the bone matrix
		dMatrix matrix(bodyPart->CalculateGlobalMatrix());

		// create the rigid body that will make this bone
		NewtonBody* const bone = NewtonCreateDynamicBody(world, shape, &matrix[0][0]);

		// calculate the moment of inertia and the relative center of mass of the solid
		//NewtonBodySetMassProperties (bone, definition.m_mass, shape);
		NewtonBodySetMassProperties(bone, 1.0f, shape);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(bone, bodyPart);

		// assign the material for early collision culling
		//NewtonBodySetMaterialGroupID(bone, m_material);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback (bone, PhysicsApplyGravityForce);
		//NewtonBodySetForceAndTorqueCallback(bone, ClampAngularVelocity);

		// destroy the collision helper shape 
		NewtonDestroyCollision(shape);
		return bone;
	}
};
#endif


class dSixAxisJointDefinition
{
	public:
	struct dJointLimit
	{
		dFloat m_minTwistAngle;
		dFloat m_maxTwistAngle;
	};

	char* m_boneName;
	char* m_parentNoneName;
	dJointLimit m_jointLimits;
};

static dSixAxisJointDefinition robot1[] =
{
	{ "bone_base001", NULL, { -1000.0f, 1000.0f } },
	{ "bone_base002", NULL, { -120.0f, 45.0f } },
	{ "bone_base003", NULL, { -120.0f, 15.0f } },
	{ "bone_base004", NULL, { -1000.0f, 1000.0f } },
	{ "bone_base005", NULL, { -225.0f, 45.0f } },
	{ "bone_base006", NULL, { -1000.0f, 1000.0f } },
	{ "effector", "bone_base001", { -1000.0f, 1000.0f } },
	{ NULL, NULL},
};

class dSixAxisManager: public dModelManager
{
	public:
	dSixAxisManager(DemoEntityManager* const scene)
		:dModelManager(scene->GetNewton())
		,m_effector(NULL)
//		,m_currentController(NULL)
//		,m_azimuth(0.0f)
//		,m_posit_x(0.0f)
//		,m_posit_y(0.0f)
//		,m_gripper_roll(0.0f)
//		,m_gripper_pitch(0.0f)
	{
//			scene->Set2DDisplayRenderFunction(RenderHelpMenu, NULL, this);
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

	dSixAxisController* MakeSixAxisRobot(DemoEntityManager* const scene, const dMatrix& origin)
	{
		DemoEntity* const model = DemoEntity::LoadNGD_mesh("robotArm.ngd", scene->GetNewton(), scene->GetShaderCache());

		scene->Append(model);
		model->ResetMatrix(*scene, origin);

		DemoEntity* const model1 = DemoEntity::LoadNGD_mesh("robot2.ngd", scene->GetNewton(), scene->GetShaderCache());
		scene->Append(model1);
		model1->ResetMatrix(*scene, origin);

		dSixAxisController* const controller = (dSixAxisController*)CreateController();
		controller->MakeSixAxisRobot(scene, model);
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
*/

	static void ClampAngularVelocity(const NewtonBody* body, dFloat timestep, int threadIndex)
	{
		dVector omega;
		NewtonBodyGetOmega(body, &omega[0]);
		omega.m_w = 0.0f;
		dFloat mag2 = omega.DotProduct3(omega);
		if (mag2 > (100.0f * 100.0f)) {
			omega = omega.Normalize().Scale(100.0f);
			NewtonBodySetOmega(body, &omega[0]);
		}

//		PhysicsApplyGravityForce(body, timestep, threadIndex);
	}

	NewtonBody* CreateBodyPart(DemoEntity* const bodyPart)
	{
		NewtonWorld* const world = GetWorld();
		NewtonCollision* const shape = bodyPart->CreateCollisionFromchildren(world);
		dAssert(shape);

		// calculate the bone matrix
		dMatrix matrix(bodyPart->CalculateGlobalMatrix());

		// create the rigid body that will make this bone
		NewtonBody* const bone = NewtonCreateDynamicBody(world, shape, &matrix[0][0]);

		// calculate the moment of inertia and the relative center of mass of the solid
		//NewtonBodySetMassProperties (bone, definition.m_mass, shape);
		NewtonBodySetMassProperties(bone, 1.0f, shape);

		// save the user data with the bone body (usually the visual geometry)
		NewtonBodySetUserData(bone, bodyPart);

		// assign the material for early collision culling
		//NewtonBodySetMaterialGroupID(bone, m_material);
		NewtonBodySetMaterialGroupID(bone, 0);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		//NewtonBodySetForceAndTorqueCallback (bone, PhysicsApplyGravityForce);
		NewtonBodySetForceAndTorqueCallback(bone, ClampAngularVelocity);

		// destroy the collision helper shape 
		NewtonDestroyCollision(shape);
		return bone;
	}

	void ConnectWithEffectoJoint(DemoEntity* const effectorNode, NewtonBody* const parent, const dSixAxisJointDefinition& definition)
	{
		dMatrix matrix(effectorNode->CalculateGlobalMatrix());
		m_effector = new dCustomKinematicController (parent, matrix);
		m_effector->SetMaxLinearFriction(1000.0f);
		m_effector->SetMaxAngularFriction(100.0f);
	}

	void ConnectWithHingeJoint(NewtonBody* const bone, NewtonBody* const parent, const dSixAxisJointDefinition& definition)
	{
		dMatrix matrix;
		NewtonBodyGetMatrix(bone, &matrix[0][0]);

		dMatrix pinAndPivotInGlobalSpace(dRollMatrix(90.0f * dDegreeToRad) * matrix);

		dCustomHinge* const joint = new dCustomHinge(pinAndPivotInGlobalSpace, bone, parent);
		if (definition.m_jointLimits.m_maxTwistAngle < 360.0f) {
			joint->EnableLimits(true);
			joint->SetLimits(definition.m_jointLimits.m_minTwistAngle * dDegreeToRad, definition.m_jointLimits.m_maxTwistAngle * dDegreeToRad);
		}
	}

	void SetModelMass(dFloat mass, int bodyCount, NewtonBody** const bodyArray) const
	{
		dFloat volume = 0.0f;
		for (int i = 0; i < bodyCount; i++) {
			volume += NewtonConvexCollisionCalculateVolume(NewtonBodyGetCollision(bodyArray[i]));
		}
		dFloat density = mass / volume;

		for (int i = 0; i < bodyCount; i++) {
			dFloat Ixx;
			dFloat Iyy;
			dFloat Izz;

			NewtonBody* const body = bodyArray[i];
			NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);
			dFloat scale = density * NewtonConvexCollisionCalculateVolume(NewtonBodyGetCollision(body));
			mass *= scale;
			Ixx *= scale;
			Iyy *= scale;
			Izz *= scale;
			NewtonBodySetMassMatrix(body, mass, Ixx, Iyy, Izz);
		}
	}

	void MakeSixAxisRobot(DemoEntityManager* const scene, const dMatrix& origin, dSixAxisJointDefinition* const definition)
	{
		DemoEntity* const model = DemoEntity::LoadNGD_mesh("robot1.ngd", scene->GetNewton(), scene->GetShaderCache());
		scene->Append(model);
		model->ResetMatrix(*scene, origin);

		// create the root body, do not set the transform call back 
		NewtonBody* const rootBody = CreateBodyPart(model);

		// make a kinematic controlled model.
		dModelRootNode* const root = new dModelRootNode(rootBody, dGetIdentityMatrix());

		// the the model to calculate the local transformation
		root->SetTranformMode(true);

		// add the model to the manager
		AddRoot(root);

		// save the controller as the collision user data, for collision culling
		//NewtonCollisionSetUserData(NewtonBodyGetCollision(rootBone), controller);

		int stackIndex = 0;
		DemoEntity* childEntities[32];
		dModelNode* parentBones[32];
		for (DemoEntity* child = model->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = root;
			childEntities[stackIndex] = child;
			stackIndex++;
		}

		int bodyCount = 1;
		NewtonBody* bodyArray[1024];
		bodyArray[0] = rootBody;

		// walk model hierarchic adding all children designed as rigid body bones. 
//		const int definitionCount = sizeof(robot1) / sizeof(robot1[0]);
		while (stackIndex) {
			stackIndex--;
			DemoEntity* const entity = childEntities[stackIndex];
			dModelNode* parentBone = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			dTrace(("name: %s\n", name));
			for (int i = 0; definition[i].m_boneName; i++) {
				if (!strcmp(definition[i].m_boneName, name)) {
					if (strstr (name, "bone")) {
						NewtonBody* const childBody = CreateBodyPart(entity);
						bodyArray[bodyCount] = childBody;
						bodyCount++;

						// connect this body part to its parent with a rag doll joint
						NewtonBody* const parentBody = parentBone->GetBody();
						ConnectWithHingeJoint(childBody, parentBody, definition[i]);
					
						dMatrix bindMatrix(entity->GetParent()->CalculateGlobalMatrix((DemoEntity*)NewtonBodyGetUserData(parentBody)).Inverse());
						parentBone = new dModelNode(childBody, bindMatrix, parentBone);
						//// save the controller as the collision user data, for collision culling
						//NewtonCollisionSetUserData(NewtonBodyGetCollision(childBody), parentBone);

						for (DemoEntity* child = entity->GetChild(); child; child = child->GetSibling()) {
							parentBones[stackIndex] = parentBone;
							childEntities[stackIndex] = child;
							stackIndex++;
						}
					} else if (strstr (name, "effector")) {	
						NewtonBody* const parentBody = parentBone->GetBody();
						ConnectWithEffectoJoint(entity, parentBody, definition[i]);
					}
					break;
				}
			}

		}

		// set mass distribution by density and volume
		SetModelMass(500.0f, bodyCount, bodyArray);

		// make root body static
		NewtonBodySetMassMatrix(rootBody, 0.0f, 0.0f, 0.0f, 0.0f);

/*
		// set the collision mask
		// note this container work best with a material call back for setting bit field 
		//dAssert(0);
		//controller->SetDefaultSelfCollisionMask();

		// transform the entire contraction to its location
		dMatrix worldMatrix(rootEntity->GetCurrentMatrix() * location);
		NewtonBodySetMatrixRecursive(rootBone, &worldMatrix[0][0]);
*/
	}

	virtual void OnDebug(dModelRootNode* const model, dCustomJoint::dDebugDisplay* const debugContext) 
	{
		if (m_effector) {
			m_effector->Debug(debugContext);
		}
	}

	virtual void OnUpdateTransform(const dModelNode* const bone, const dMatrix& localMatrix) const
	{
		NewtonBody* const body = bone->GetBody();
		DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(body);
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));

		dQuaternion rot(localMatrix);
		ent->SetMatrix(*scene, rot, localMatrix.m_posit);
	}


	dCustomKinematicController* m_effector;
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
	CreateLevelMesh (scene, "flatPlane.ngd", true);
	dSixAxisManager* const robotManager = new dSixAxisManager(scene);

	dMatrix origin(dYawMatrix(0.0f * dDegreeToRad));
	dMatrix origin1(dYawMatrix(180.0f * dDegreeToRad));
	origin.m_posit.m_z = -1.0f;
	origin1.m_posit.m_z =  1.0f;

	int count = 10;
count = 1;
origin = dGetIdentityMatrix();
origin.m_posit.m_x = 0.0f;
	for (int i = 0; i < count; i ++) {
		robotManager->MakeSixAxisRobot (scene, origin, robot1);
		origin.m_posit.m_x += 1.0f;
		origin1.m_posit.m_x += 1.0f;
	}

	origin.m_posit = dVector(0.0f, 1.0f, 3.0f, 1.0f);
	dMatrix rotation (dYawMatrix(90.0f * dDegreeToRad));
	scene->SetCameraMatrix(rotation, origin.m_posit);
}

