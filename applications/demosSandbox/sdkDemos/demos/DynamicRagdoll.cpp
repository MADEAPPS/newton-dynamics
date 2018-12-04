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
#include "PhysicsUtils.h"
#include "TargaToOpenGl.h"
#include "DemoEntityManager.h"
#include "DebugDisplay.h"
#include "HeightFieldPrimitive.h"

#define DEMO_MUSCLE_STRENGTH	10.0f

struct dRagDollConfig
{
	char m_partName[32];
	dFloat m_mass;
	dFloat m_minLimit;
	dFloat m_maxLimit;
	dFloat m_frictionScale;
};

static dRagDollConfig ragDollConfig[] =
{
	{ "bone_spine0", 1.0f, -1000.0f, 1000.0f, 50.0f },

	{ "bone_rightLeg", 1.0f, -70.0f, 50.0f, 200.0f },
	{ "bone_rightKnee", 1.0f, -70.0f, 20.0f, 50.0f },
	{ "effector_rightLeg", 1.0f, 0.0f, 0.0f, 50.0f },
//	{ "boneFD_rightAnkle", 50.0f, -90.0f, 45.0f, 100.0f },
//	{ "effector_rightAnkle", 100.0f, 0.0f, 0.0f, 100.0f },
//	{ "bone_rightToe", 50.0f, -90.0f, 45.0f, 100.0f },
//	{ "effector_rightToe", 100.0f, 0.0f, 0.0f, 100.0f },

	{ "bone_leftLeg", 1.0f, -70.0f, 50.0f, 200.0f },
	{ "bone_leftknee", 1.0f, -70.0f, 20.0f, 50.0f },
	{ "effector_leftLeg", 1.0f, 0.0f, 0.0f, 50.0f },
//	{ "boneFD_leftAnkle", 50.0f, -90.0f, 45.0f, 100.0f },
//	{ "effector_leftAnkle", 100.0f, 0.0f, 0.0f, 100.0f },
//	{ "bone_leftToe", 50.0f, -90.0f, 45.0f, 100.0f },
//	{ "effector_leftToe", 100.0f, 0.0f, 0.0f, 100.0f },
};

class dWalkGenerator: public dAnimationEffectorBlendPose
{
	public:
	dWalkGenerator(dAnimationCharacterRig* const character, dAnimationRigEffector* const leftFeet, dAnimationRigEffector* const rightFeet)
		:dAnimationEffectorBlendPose(character)
		,m_acc(0.0f)
		,m_amplitud_x(2.0f)
		,m_amplitud_y(1.3f)
		,m_period(1.0f)
		,m_cycle()
		,m_leftFeet(leftFeet)
		,m_rightFeet(rightFeet)
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
		memset(leftControlPoints, 0, sizeof(leftControlPoints));

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

	void Evaluate(dAnimationPose& output, dFloat timestep)
	{
		dAnimationEffectorBlendPose::Evaluate(output, timestep);

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
		for (dAnimationPose::dListNode* node = output.GetFirst(); node; node = node->GetNext()) {
			dAnimationTransform& transform = node->GetInfo();
			if ((transform.m_effector == m_leftFeet) || (transform.m_effector == m_rightFeet)) {
				transform.m_posit.m_y += high[m_sequence[index]];
				transform.m_posit.m_x += stride[m_sequence[index]];
			}
			index++;
		}
		m_acc = dMod(m_acc + timestep, m_period);
	}

	dFloat m_acc;
	dFloat m_period;
	dFloat m_amplitud_x;
	dFloat m_amplitud_y;
	dBezierSpline m_cycle;
	int m_sequence[6];
	dAnimationRigEffector* m_leftFeet;
	dAnimationRigEffector* m_rightFeet;
};

class dEffectorTreePostureGenerator: public dAnimationEffectorBlendNode
{
	public:
	dEffectorTreePostureGenerator(dAnimationCharacterRig* const character, dAnimationEffectorBlendNode* const child)
		:dAnimationEffectorBlendNode(character)
		,m_euler(0.0f)
		,m_position(0.0f)
		,m_child(child)
	{
	}

	~dEffectorTreePostureGenerator()
	{
		delete m_child;
	}

	void Evaluate(dAnimationPose& output, dFloat timestep)
	{
		m_child->Evaluate(output, timestep);

		dQuaternion rotation(dPitchMatrix(m_euler.m_x) * dYawMatrix(m_euler.m_y) * dRollMatrix(m_euler.m_z));
		for (dAnimationPose::dListNode* node = output.GetFirst(); node; node = node->GetNext()) {
			dAnimationTransform& transform = node->GetInfo();
			transform.m_rotation = transform.m_rotation * rotation;
			transform.m_posit = m_position + rotation.RotateVector(transform.m_posit);
		}
	}

	dVector m_euler;
	dVector m_position;
	dAnimationEffectorBlendNode* m_child;
};

class dAnimationKeeController: public dAnimationRigForwardDynamicLimb
{
	public:
	dAnimationKeeController(const dMatrix& basicMatrix, dAnimationRigJoint* const parent, NewtonBody* const body, const dRagDollConfig& config)
		:dAnimationRigForwardDynamicLimb(basicMatrix, parent, body)
	{
		SetFriction(config.m_frictionScale * config.m_mass * DEMO_MUSCLE_STRENGTH);
		SetLimits(config.m_minLimit * dDegreeToRad, config.m_maxLimit * dDegreeToRad);


		dMatrix matrix0;
		dMatrix matrix1;
		dMatrix rootMatrix(GetRoot()->GetBasePoseMatrix());
		CalculateGlobalMatrix(matrix0, matrix1);
		m_offsetAngle = CalculateAngle(rootMatrix.m_up, matrix0.m_up, rootMatrix.m_right);
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		dAnimationRigForwardDynamicLimb::SubmitConstraints(timestep, threadIndex);
	}

	dFloat m_offsetAngle;
};

class BalancingDummyManager : public dAnimationCharacterRigManager
{
	public:

	class dAnimationCharacterUserData: public DemoEntity::UserData
	{
		public:
		dAnimationCharacterUserData(dAnimationCharacterRig* const rig, dAnimationEffectorBlendTwoWay* const walk, dEffectorTreePostureGenerator* const posture)
			:DemoEntity::UserData()
			,m_rig(rig)
			,m_walk(walk)
			,m_posture(posture)
			,m_hipHigh(0.0f)
			,m_walkSpeed(0.0f)
		{
		}

		void OnRender(dFloat timestep) const
		{
		}

		void OnInterpolateMatrix(DemoEntityManager& world, dFloat param) const
		{
		}

		void OnTransformCallback(DemoEntityManager& world) const
		{
		}

		dAnimationCharacterRig* m_rig;
		dAnimationEffectorBlendTwoWay* m_walk;
		dEffectorTreePostureGenerator* m_posture;

		dFloat m_hipHigh;
		dFloat m_walkSpeed;
	};


	BalancingDummyManager(DemoEntityManager* const scene)
		:dAnimationCharacterRigManager(scene->GetNewton())
		,m_currentRig(NULL)
	{
		scene->Set2DDisplayRenderFunction(RenderHelpMenu, NULL, this);
	}

	~BalancingDummyManager()
	{
	}

	static void RenderHelpMenu(DemoEntityManager* const scene, void* const context)
	{
		BalancingDummyManager* const me = (BalancingDummyManager*)context;
		if (me->m_currentRig) {
			DemoEntity* const entiry = (DemoEntity*) NewtonBodyGetUserData(me->m_currentRig->GetNewtonBody());
			dAnimationCharacterUserData* const controlData = (dAnimationCharacterUserData*) entiry->GetUserData();

			dVector color(1.0f, 1.0f, 0.0f, 0.0f);
			scene->Print(color, "Sliders control");
			
			dFloat32 val0 = dFloat32(controlData->m_walkSpeed);
			ImGui::SliderFloat_DoubleSpace("walkSpeed", &val0, 0.0f, 1.0f);
			controlData->m_walkSpeed = val0;

			dFloat32 val1 = dFloat32(controlData->m_hipHigh);
			ImGui::SliderFloat_DoubleSpace("hip high", &val1, -0.5f, 1.5f);
			controlData->m_hipHigh = val1;
		}
	}

	void OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
	{
		dAnimationCharacterRigManager::OnDebug(debugContext);
//		for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
//			dSixAxisController* const controller = &node->GetInfo();
//			controller->Debug(debugContext);
//		}
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
		//return NewtonCreateConvexHull(GetWorld(), mesh->m_vertexCount, &points[0][0], sizeof(dVector), 1.0e-3f, SERVO_VEHICLE_DEFINITION::m_bodyPart, NULL);
		return NewtonCreateConvexHull(GetWorld(), mesh->m_vertexCount, &points[0][0], sizeof(dVector), 1.0e-3f, 0, NULL);
	}

	NewtonBody* CreateBodyPart(DemoEntity* const bodyPart, const dRagDollConfig& definition)
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

		// save the user lifterData with the bone body (usually the visual geometry)
		NewtonBodySetUserData(body, bodyPart);

		// assign a body part id
		//NewtonCollisionSetUserID(collision, definition.m_bodyPartID);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);
		return body;
	}

	dAnimationCharacterRig* CreateRagDoll(DemoEntityManager* const scene, const dMatrix& origin)
	{
/*
DemoEntity* const xxxx0 = DemoEntity::LoadNGD_mesh("tred_1.ngd", scene->GetNewton());
DemoEntity* const xxxx1 = DemoEntity::LoadNGD_mesh("tred_2.ngd", scene->GetNewton());
scene->Append(xxxx0);
scene->Append(xxxx1);

dMatrix matrix0(xxxx0->GetCurrentMatrix());
matrix0.m_posit.m_x += 5.0f;
matrix0.m_posit.m_z += 2.0f;
xxxx0->ResetMatrix(*scene, matrix0);

dMatrix matrix1(xxxx1->GetCurrentMatrix());
matrix1.m_posit.m_x += 5.0f;
matrix1.m_posit.m_z -= 2.0f;
xxxx1->ResetMatrix(*scene, matrix1);
*/

		DemoEntity* const model = DemoEntity::LoadNGD_mesh("tred_2.ngd", scene->GetNewton());
		scene->Append(model);

		dMatrix modelMatrix(model->GetCurrentMatrix());
		modelMatrix.m_posit = dVector(0.0f);
		modelMatrix.m_posit.m_w = 1.0f;

		dMatrix rootMatrix(modelMatrix * origin);
		model->ResetMatrix(*scene, rootMatrix);

		NewtonBody* const rootBody = CreateBodyPart(model, ragDollConfig[0]);

//new dCustom6dof(rootMatrix, rootBody);
//NewtonBodySetMassMatrix(rootBody, 0.0f, 0.0f, 0.0f, 0.0f);

		DemoEntity* const localFrame = model->Find("rootLocalFrame");
		dAssert(localFrame);
		dMatrix localFrameMatrix(localFrame->CalculateGlobalMatrix());
		dAnimationCharacterRig* const rig = CreateCharacterRig(rootBody, localFrameMatrix);

		int stackIndex = 0;
		DemoEntity* childEntities[32];
		dAnimationRigJoint* parentBones[32];
		for (DemoEntity* child = model->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = rig;
			childEntities[stackIndex] = child;
			stackIndex++;
		}

		dAnimationRigEffector* leftFeet = NULL;
		dAnimationRigEffector* rightFeet = NULL;

		const int partCount = sizeof(ragDollConfig) / sizeof(ragDollConfig[0]);
		while (stackIndex) {
			stackIndex--;
			DemoEntity* const entity = childEntities[stackIndex];
			dAnimationRigJoint* const parentJoint = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			for (int i = 0; i < partCount; i++) {
				if (!strcmp(ragDollConfig[i].m_partName, name)) {

					if (strstr(name, "bone")) {
						// add a bone and all it children
						NewtonBody* const limbBody = CreateBodyPart(entity, ragDollConfig[i]);

						dMatrix matrix;
						NewtonBodyGetMatrix(limbBody, &matrix[0][0]);

						dAnimationRigLimb* limbJoint = NULL;
						if (strstr(name, "boneFD")) {
							dAnimationRigForwardDynamicLimb* const hinge = new dAnimationKeeController(matrix, parentJoint, limbBody, ragDollConfig[i]);
							limbJoint = hinge;
						} else {
							dAnimationRigHinge* const hinge = new dAnimationRigHinge(matrix, parentJoint, limbBody);
							hinge->SetFriction(ragDollConfig[i].m_frictionScale * ragDollConfig[i].m_mass * DEMO_MUSCLE_STRENGTH);
							hinge->SetLimits(ragDollConfig[i].m_minLimit * dDegreeToRad, ragDollConfig[i].m_maxLimit * dDegreeToRad);
							limbJoint = hinge;
						}

						for (DemoEntity* child = entity->GetChild(); child; child = child->GetSibling()) {
							parentBones[stackIndex] = limbJoint;
							childEntities[stackIndex] = child;
							stackIndex++;
						}
					} else if (strstr(name, "effector")) {
						// add an end effector (end effector can't have children)
						dMatrix pivot(entity->CalculateGlobalMatrix());
						dAnimationRigEffector* const effector = new dAnimationRigEffector(pivot, parentJoint->GetAsRigLimb(), rig);
						effector->SetLinearSpeed(2.0f);
						effector->SetMaxLinearFriction(ragDollConfig[i].m_frictionScale * ragDollConfig[i].m_mass * DEMO_MUSCLE_STRENGTH * 50.0f);

						if (!strcmp (name, "effector_leftLeg")) {
							leftFeet = effector;
						}
						if (!strcmp(name, "effector_rightLeg")) {
							rightFeet = effector;
						}
					}
					break;
				}
			}
		}

		rig->Finalize();

		dAnimationEffectorBlendPose* const fixPose = new dAnimationEffectorBlendPose(rig);
		dAnimationEffectorBlendPose* const walkPose = new dWalkGenerator(rig, leftFeet, rightFeet);
		dAnimationEffectorBlendTwoWay* const walkBlend = new dAnimationEffectorBlendTwoWay(rig, fixPose, walkPose);
		dEffectorTreePostureGenerator* const posture = new dEffectorTreePostureGenerator (rig, walkBlend);
		dAnimationEffectorBlendRoot* const animTree = new dAnimationEffectorBlendRoot(rig, posture);

		dAnimationCharacterUserData* const renderCallback = new dAnimationCharacterUserData(rig, walkBlend, posture);
		model->SetUserData(renderCallback);
		
		rig->SetAnimationTree (animTree);

		m_currentRig = rig;
		return rig;
	}

	void OnUpdateTransform(const dAnimationRigJoint* const bone, const dMatrix& localMatrix) const
	{
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(GetWorld());
		NewtonBody* const newtonBody = bone->GetNewtonBody();
		DemoEntity* const meshEntity = (DemoEntity*)NewtonBodyGetUserData(newtonBody);

		dQuaternion rot(localMatrix);
		meshEntity->SetMatrix(*scene, rot, localMatrix.m_posit);
	}

	void PreUpdate(dFloat timestep)
	{
		if (m_currentRig) {
			DemoEntity* const entiry = (DemoEntity*)NewtonBodyGetUserData(m_currentRig->GetNewtonBody());
			dAnimationCharacterUserData* const controlData = (dAnimationCharacterUserData*)entiry->GetUserData();

			dAnimationEffectorBlendTwoWay* const walkBlend = controlData->m_walk;
			walkBlend->SetParam (controlData->m_walkSpeed);

			dEffectorTreePostureGenerator* const posture = controlData->m_posture;
			posture->m_position.m_y = 0.25f * controlData->m_hipHigh;
		}

		dAnimationCharacterRigManager::PreUpdate(timestep);
	}

	dAnimationCharacterRig* m_currentRig;
};

void DynamicRagDoll(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	CreateLevelMesh(scene, "flatPlane.ngd", true);
	BalancingDummyManager* const robotManager = new BalancingDummyManager(scene);

	int count = 10;
	count = 1;

	dMatrix origin (dYawMatrix(-90.0f * dDegreeToRad));
//origin = dGetIdentityMatrix();
	origin.m_posit.m_x = 2.0f;
//	origin.m_posit.m_y = 2.1f;
	origin.m_posit.m_y = 3.0f;
	for (int i = 0; i < count; i++) {
		robotManager->CreateRagDoll(scene, origin);
		//robotManager->CreateRagDoll (scene, origin1);
		origin.m_posit.m_x += 1.0f;
	}

	origin.m_posit = dVector(-4.0f, 3.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(dGetIdentityMatrix(), origin.m_posit);
}




