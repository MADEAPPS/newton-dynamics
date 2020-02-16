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

#define SIZE_ROBOT_MASS 500.0f

class dSixAxisJointDefinition
{
	public:
	enum jointType
	{
		m_node,
		m_hinge,
		m_sliderActuator,
		m_effector,
	};

	struct dJointLimit
	{
		dFloat m_minLimit;
		dFloat m_maxLimit;
	};

	const char* m_boneName;
	jointType m_type;
	dJointLimit m_jointLimits;
	dFloat m_massFraction;
};

static dSixAxisJointDefinition robot1[] =
{
	{ "bone_base000", dSixAxisJointDefinition::m_node, { -1000.0f, 1000.0f }, 1.0f },
	{ "bone_base001", dSixAxisJointDefinition::m_hinge, { -1000.0f, 1000.0f }, 0.4f },
	{ "bone_base002", dSixAxisJointDefinition::m_hinge, { -120.0f, 45.0f }, 0.3f },
	{ "bone_base003", dSixAxisJointDefinition::m_hinge, { -120.0f, 15.0f }, 0.2f },
	{ "bone_base004", dSixAxisJointDefinition::m_hinge, { -1000.0f, 1000.0f }, 0.1f },
	{ "bone_base005", dSixAxisJointDefinition::m_hinge, { -225.0f, 45.0f }, 0.06f },
	{ "bone_base006", dSixAxisJointDefinition::m_hinge, { -1000.0f, 1000.0f }, 0.06f },
	{ "actuator_slider0", dSixAxisJointDefinition::m_sliderActuator, { -0.3f, 0.1f }, 0.02f},
	{ "actuator_slider1", dSixAxisJointDefinition::m_sliderActuator, { -0.3f, 0.1f }, 0.02f},
	{ "effector", dSixAxisJointDefinition::m_effector, { -1000.0f, 1000.0f }},
	{ NULL},
};

class dSixAxisRobot: public dModelRootNode
{
	public:
	dSixAxisRobot(NewtonBody* const rootBody, DemoEntity* const splineCurve)
		:dModelRootNode(rootBody, dGetIdentityMatrix())
		,m_pivotMatrix(dGetIdentityMatrix())
		,m_gripperMatrix(dGetIdentityMatrix())
		,m_effector(NULL)
		,m_gripper0(NULL)
		,m_gripper1(NULL)
		,m_gripperPosit(0.0f)
		,m_azimuth(0.0f)
		,m_posit_x(0.0f)
		,m_posit_y(0.0f)
		,m_pitch(0.0f)
		,m_yaw(0.0f)
		,m_roll(0.0f)
		,m_splineCurve(splineCurve)
	{
	}

	~dSixAxisRobot()
	{
	}

	void SetPivotMatrix()
	{
		if (m_effector) {
			dMatrix baseMatrix;
			dMatrix pivotMatrix;
			dModelNode* const referenceNode = GetChildren().GetFirst()->GetInfo();
			NewtonBodyGetMatrix(GetBody(), &baseMatrix[0][0]);
			NewtonBodyGetMatrix(referenceNode->GetBody(), &pivotMatrix[0][0]);
			m_pivotMatrix = pivotMatrix * baseMatrix.Inverse();
			m_gripperMatrix = m_effector->GetTargetMatrix() * m_pivotMatrix.Inverse();
		}
	}

	void UpdateEffectors (dFloat gripper, dFloat azimuth, dFloat posit_x, dFloat posit_y, dFloat pitch, dFloat yaw, dFloat roll)
	{
		if (m_effector) {
			// clamp rotation to not more than one degree per step.
			dFloat angleStep = 0.5f;
			dFloat azimuthError = dClamp (azimuth - m_azimuth, -angleStep, angleStep);
			m_azimuth += dFloat32 (azimuthError);

			m_posit_x = dFloat32 (posit_x);
			m_posit_y = dFloat32 (posit_y);
			m_pitch = dFloat32 (pitch);
			m_yaw = dFloat32 (yaw);
			m_roll = dFloat32 (roll);

			dMatrix yawMatrix(dYawMatrix(m_azimuth * dDegreeToRad));
			dMatrix gripperMatrix(dPitchMatrix(m_pitch * dDegreeToRad) * dYawMatrix(m_yaw * dDegreeToRad) * dRollMatrix(m_roll * dDegreeToRad) * m_gripperMatrix);

			gripperMatrix.m_posit.m_x += m_posit_x;
			gripperMatrix.m_posit.m_y += m_posit_y;
			m_effector->SetTargetMatrix(gripperMatrix * yawMatrix * m_pivotMatrix);
		}

		m_gripperPosit = dFloat32 (gripper);
		if (m_gripper0) {
			m_gripper0->SetTargetPosit(m_gripperPosit);
		}
		if (m_gripper1) {
			m_gripper1->SetTargetPosit(m_gripperPosit);
		}
	}
	
	dMatrix m_pivotMatrix;
	dMatrix m_gripperMatrix;
	dCustomKinematicController* m_effector;
	dCustomSliderActuator* m_gripper0;
	dCustomSliderActuator* m_gripper1;

	dFloat32 m_gripperPosit;
	dFloat32 m_azimuth;
	dFloat32 m_posit_x;
	dFloat32 m_posit_y;
	dFloat32 m_pitch;
	dFloat32 m_yaw;
	dFloat32 m_roll;
	DemoEntity* m_splineCurve;
};

class dSixAxisManager: public dModelManager
{
	public:
	dSixAxisManager(DemoEntityManager* const scene)
		:dModelManager(scene->GetNewton())
		,m_currentController(NULL)
		,m_gripperPosit(0.0f)
		,m_azimuth(0.0f)
		,m_posit_x(-0.5f)
		,m_posit_y(-0.5f)
		,m_gripper_pitch(0.0f)
		,m_gripper_yaw(0.0f)
		,m_gripper_roll(0.0f)
	{
		scene->Set2DDisplayRenderFunction(RenderHelpMenu, RenderUI, this);
	}

	~dSixAxisManager()
	{
	}

	static void RenderHelpMenu(DemoEntityManager* const scene, void* const context)
	{
		dSixAxisManager* const me = (dSixAxisManager*)context;

		dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		scene->Print(color, "gripper position");
		ImGui::SliderFloat("Gripper", &me->m_gripperPosit, -0.2f, 0.03f);

		scene->Print(color, "linear degrees of freedom");
		ImGui::SliderFloat("Azimuth", &me->m_azimuth, -180.0f, 180.0f);
		ImGui::SliderFloat("posit_x", &me->m_posit_x, -1.4f, 0.2f);
		ImGui::SliderFloat("posit_y", &me->m_posit_y, -1.2f, 0.4f);

		ImGui::Separator();
		scene->Print(color, "angular degrees of freedom");
		ImGui::SliderFloat("pitch", &me->m_gripper_pitch, -180.0f, 180.0f);
		ImGui::SliderFloat("yaw", &me->m_gripper_yaw, -80.0f, 130.0f);
		ImGui::SliderFloat("roll", &me->m_gripper_roll, -180.0f, 180.0f);
	}

	static void RenderUI(DemoEntityManager* const scene, void* const context)
	{
		dSixAxisManager* const me = (dSixAxisManager*)context;
		me->RenderUI(scene);
	}

	void RenderUI(DemoEntityManager* const scene)
	{
		//TestSpline();
	}

	void TestSpline() const
	{
		static dBezierSpline spline;
		if (spline.GetControlPointCount() == 0) {
			dFloat64 knots[] = { 0.0f, 1.0f / 3.0f, 2.0f / 3.0f, 1.0f };
			dBigVector control[] =
			{
				dBigVector(200.0f, 200.0f, 0.0f, 1.0f),
				dBigVector(150.0f, 250.0f, 0.0f, 1.0f),
				dBigVector(250.0f, 300.0f, 0.0f, 1.0f),
				dBigVector(350.0f, 250.0f, 0.0f, 1.0f),
				dBigVector(250.0f, 150.0f, 0.0f, 1.0f),
				dBigVector(200.0f, 200.0f, 0.0f, 1.0f),
			};

			spline.CreateFromKnotVectorAndControlPoints(3, sizeof (knots) / sizeof (knots[0]), knots, control);
			dFloat64 u = (knots[1] + knots[2]) * 0.5f;
			spline.InsertKnot(u);
			spline.InsertKnot(u);
			spline.InsertKnot(u);
			spline.InsertKnot(u);
			spline.InsertKnot(u);
			spline.RemoveKnot(u, 1.0e-3f);
			//spline.RemoveKnot (u, 1.0e-3f);
		}

		glDisable(GL_LIGHTING);
//		glDisable(GL_TEXTURE_2D);

		const int segments = 20;
		glColor3f(1.0f, 1.0f, 1.0f);
		glBegin(GL_LINES);
		dBigVector p0(spline.CurvePoint(0.0f));
		for (int i = 1; i <= segments; i++) {
			dBigVector p1(spline.CurvePoint(dFloat(i) / segments));
			glVertex3f(dFloat32 (p0.m_x), dFloat32 (p0.m_y), dFloat32 (p0.m_z));
			glVertex3f(dFloat32 (p1.m_x), dFloat32 (p1.m_y), dFloat32 (p1.m_z));
			p0 = p1;
		}
		glEnd();

		glColor3f(0.0f, 1.0f, 1.0f);
		glPointSize(4.0f);
		glBegin(GL_POINTS);
		//dArray<dBigVector>& controlPoints = spline.GetControlPointArray();
		for (int i = 0; i < spline.GetControlPointCount(); i++) {
			dBigVector p(spline.GetControlPoint(i));
			glVertex3f(dFloat32 (p.m_x), dFloat32 (p.m_y), dFloat32 (p.m_z));
		}
		glEnd();

		// recreate the spline from sample points at equally spaced distance 
		int pointCount = 4;
		dBigVector points[5];
		points[0] = spline.CurvePoint(0.0f / 3.0f) + dBigVector(300.0f, 0.0f, 0.0f, 0.0f);
		points[1] = spline.CurvePoint(1.0f / 3.0f) + dBigVector(300.0f, 0.0f, 0.0f, 0.0f);
		points[2] = spline.CurvePoint(2.0f / 3.0f) + dBigVector(300.0f, 0.0f, 0.0f, 0.0f);
		points[3] = spline.CurvePoint(3.0f / 3.0f) + dBigVector(300.0f, 0.0f, 0.0f, 0.0f);

		dBigVector derivP0(spline.CurveDerivative(0.0f));
		dBigVector derivP1(spline.CurveDerivative(1.0f));

		static dBezierSpline spline1;
		spline1.GlobalCubicInterpolation(pointCount, points, derivP0, derivP1);

		glColor3f(1.0f, 1.0f, 1.0f);
		glBegin(GL_LINES);
		p0 = spline1.CurvePoint(0.0f);
		for (int i = 1; i <= segments; i++) {
			dFloat64 u = dFloat(i) / segments;
			dBigVector p1(spline1.CurvePoint(u));
			glVertex3f(dFloat32 (p0.m_x), dFloat32 (p0.m_y), dFloat32 (p0.m_z));
			glVertex3f(dFloat32 (p1.m_x), dFloat32 (p1.m_y), dFloat32 (p1.m_z));
			p0 = p1;
		}
		glEnd();

		glPointSize(4.0f);
		glBegin(GL_POINTS);
		glColor3f(0.0f, 1.0f, 1.0f);
		for (int i = 0; i < spline.GetControlPointCount(); i++) {
			dBigVector p(spline1.GetControlPoint(i));
			glVertex3f(dFloat32 (p.m_x), dFloat32 (p.m_y), dFloat32 (p.m_z));
		}

		glColor3f(1.0f, 0.0f, 0.0f);
		for (int i = 0; i < pointCount; i++) {
			glVertex3f(dFloat32 (points[i].m_x), dFloat32 (points[i].m_y), dFloat32 (points[i].m_z));
		}
		glEnd();
	}

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

		PhysicsApplyGravityForce(body, timestep, threadIndex);
	}

	NewtonBody* CreateBodyPart(DemoEntity* const bodyPart, const dSixAxisJointDefinition& definition)
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
		NewtonBodySetMassProperties(bone, definition.m_massFraction, shape);

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

	dCustomKinematicController* ConnectEffector(dModelRootNode* const model, dFloat mass, DemoEntity* const effectorNode, NewtonBody* const body, const dSixAxisJointDefinition& definition)
	{
		NewtonBody* const effectorReferenceBody = model->GetBody(); 
		dMatrix matrix(effectorNode->CalculateGlobalMatrix());
		dCustomKinematicController* const effector = new dCustomKinematicController(body, matrix, effectorReferenceBody);
		effector->SetSolverModel(1);
		effector->SetMaxLinearFriction(mass * 9.8f * 50.0f);
		effector->SetMaxAngularFriction(mass * 50.0f);
		return effector;
	}

	dCustomJoint* ConnectJoint(NewtonBody* const bone, NewtonBody* const parent, const dSixAxisJointDefinition& definition)
	{
		dMatrix matrix;
		NewtonBodyGetMatrix(bone, &matrix[0][0]);

		dMatrix pinAndPivotInGlobalSpace(dRollMatrix(90.0f * dDegreeToRad) * matrix);

		dCustomJoint* joint = NULL;
		if (definition.m_type == dSixAxisJointDefinition::m_hinge) {
			dCustomHinge* const hinge = new dCustomHinge(pinAndPivotInGlobalSpace, bone, parent);
			if (definition.m_jointLimits.m_maxLimit < 360.0f) {
				hinge->EnableLimits(true);
				hinge->SetLimits(definition.m_jointLimits.m_minLimit * dDegreeToRad, definition.m_jointLimits.m_maxLimit * dDegreeToRad);
			}
			joint = hinge;
		} else if (definition.m_type == dSixAxisJointDefinition::m_sliderActuator) {
			dCustomSliderActuator* const slider = new dCustomSliderActuator (pinAndPivotInGlobalSpace, 2.0f, definition.m_jointLimits.m_minLimit, definition.m_jointLimits.m_maxLimit, bone, parent);
			slider->SetMinForce (-9.8f * 100.0f);
			slider->SetMaxForce (1.0e20f);
			joint = slider;
		}
		return joint;
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
				stackBuffer[stack] = node->GetInfo();
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
			if (minInertia < 10.0f) {
				dFloat maxInertia = dMax(dFloat(10.0f), dMax(Ixx, dMax(Iyy, Izz)));
				Ixx = maxInertia;
				Iyy = maxInertia;
				Izz = maxInertia;
			}

			NewtonBodySetMassMatrix(body, mass, Ixx, Iyy, Izz);
		}
	}

	DemoEntity* MakeCuve(DemoEntityManager* const scene, const dMatrix& origin)
	{
		// add a spline path for the robot to track
		dMatrix pathMatrix(origin);
		pathMatrix = dYawMatrix(90.0f * dDegreeToRad) * pathMatrix;
		pathMatrix.m_posit.m_y += 0.8f;
		pathMatrix.m_posit.m_x += 1.75f;

		DemoEntity* const splineCurve = new DemoEntity(pathMatrix, NULL);
		scene->Append(splineCurve);

		dFloat64 knots[] = { 0.0f, 1.0f / 3.0f, 2.0f / 3.0f, 1.0f };
		dBigVector control[] =
		{
			dBigVector(0.0f, 0.0f, 0.0f, 1.0f),
			dBigVector(-0.5f, 0.5f, 0.0f, 1.0f),
			dBigVector(0.5f, 1.0f, 0.0f, 1.0f),
			dBigVector(1.5f, 0.5f, 0.0f, 1.0f),
			dBigVector(0.5f, -0.5f, 0.0f, 1.0f),
			dBigVector(0.0f, 0.0f, 0.0f, 1.0f),
		};
		dBezierSpline spline;
		spline.CreateFromKnotVectorAndControlPoints(3, sizeof(knots) / sizeof(knots[0]), knots, control);
		DemoBezierCurve* const curveMesh = new DemoBezierCurve(spline);
		curveMesh->SetVisible(true);

		splineCurve->SetMesh(curveMesh, dGetIdentityMatrix());
		curveMesh->Release();

		return splineCurve;
	}

	void MakeSixAxisRobot(DemoEntityManager* const scene, const dMatrix& origin, dSixAxisJointDefinition* const definition)
	{
		DemoEntity* const entityModel = DemoEntity::LoadNGD_mesh("robot1.ngd", scene->GetNewton(), scene->GetShaderCache());
		scene->Append(entityModel);
		entityModel->ResetMatrix(*scene, origin);

		// add a spline path for the robot to track
		DemoEntity* const splinePath = MakeCuve(scene, origin);

		// create the root body, do not set the transform call back 
		NewtonBody* const rootBody = CreateBodyPart(entityModel, definition[0]);

		// make a kinematic controlled model.
		dSixAxisRobot* const robot = new dSixAxisRobot(rootBody, splinePath);

		// add the model to the manager
		AddRoot(robot);

		// the the model to calculate the local transformation
		robot->SetTransformMode(true);

		int stackIndex = 0;
		DemoEntity* childEntities[32];
		dModelNode* parentBones[32];
		for (DemoEntity* child = entityModel->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = robot;
			childEntities[stackIndex] = child;
			stackIndex++;
		}

		// walk model hierarchic adding all children designed as rigid body bones. 
		while (stackIndex) {
			stackIndex--;
			DemoEntity* const entity = childEntities[stackIndex];
			dModelNode* parentBone = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			//dTrace(("name: %s\n", name));
			for (int i = 1; definition[i].m_boneName; i++) {
				if (!strcmp(definition[i].m_boneName, name)) {
					NewtonBody* const parentBody = parentBone->GetBody();
					if (definition[i].m_type == dSixAxisJointDefinition::m_effector) {
						robot->m_effector = ConnectEffector(robot, SIZE_ROBOT_MASS, entity, parentBody, definition[i]);
					} else {
						NewtonBody* const childBody = CreateBodyPart(entity, definition[i]);
						dCustomJoint* const joint = ConnectJoint(childBody, parentBody, definition[i]);
						if (joint->IsType(dCustomSliderActuator::GetType())) {
							if (!robot->m_gripper0) {
								robot->m_gripper0 = (dCustomSliderActuator*) joint;
							} else {
								robot->m_gripper1 = (dCustomSliderActuator*) joint;
							}
						}
						
						dMatrix bindMatrix(entity->GetParent()->CalculateGlobalMatrix((DemoEntity*)NewtonBodyGetUserData(parentBody)).Inverse());
						dModelNode* const bone = new dModelNode(childBody, bindMatrix, parentBone);

						for (DemoEntity* child = entity->GetChild(); child; child = child->GetSibling()) {
							parentBones[stackIndex] = bone;
							childEntities[stackIndex] = child;
							stackIndex++;
						}
					}
					break;
				}
			}
		}

		// set mass distribution by density and volume
		NormalizeMassAndInertia(robot, SIZE_ROBOT_MASS);

		// make the root body static 
		NewtonBodySetMassMatrix(rootBody, 0.0f, 0.0f, 0.0f, 0.0f);

		m_currentController = robot;
		robot->SetPivotMatrix();
	}

	virtual void OnDebug(dModelRootNode* const model, dCustomJoint::dDebugDisplay* const debugContext) 
	{
		dSixAxisRobot* const robot = (dSixAxisRobot*)model;
		if (robot->m_effector) {
			robot->m_effector->Debug(debugContext);
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

	virtual void OnPreUpdate(dModelRootNode* const model, dFloat timestep, int threadID) const 
	{
		if (model == m_currentController) {
			m_currentController->UpdateEffectors (m_gripperPosit, m_azimuth, m_posit_x, m_posit_y, 
												  m_gripper_pitch, m_gripper_yaw, m_gripper_roll);
		}
	}
	
	dSixAxisRobot* m_currentController;

	dFloat32 m_gripperPosit;
	dFloat32 m_azimuth;
	dFloat32 m_posit_x;
	dFloat32 m_posit_y;
	dFloat32 m_gripper_pitch;
	dFloat32 m_gripper_yaw;
	dFloat32 m_gripper_roll;
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

