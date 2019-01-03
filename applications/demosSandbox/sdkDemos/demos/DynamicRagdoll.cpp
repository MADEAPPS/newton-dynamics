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
	{ "bone_spine0", 200.0f, -1000.0f, 1000.0f, 50.0f },

	{ "bone_rightLeg", 25.0f, -70.0f, 50.0f, 100.0f },
	{ "bone_rightKnee", 20.0f, -70.0f, 20.0f, 100.0f },
	{ "boneFD_rightAnkle", 5.0f, -75.0f, 60.0f, 10.0f },
	{ "boneFD_rightToe",  5.0f, -30.0f, 30.0f, 10.0f },
	{ "effector_rightLeg", 100.0f, 0.0f, 0.0f, 50.0f },
	
	{ "bone_leftLeg", 25.0f, -70.0f, 50.0f, 100.0f },
	{ "bone_leftknee", 20.0f, -70.0f, 20.0f, 100.0f },
	{ "boneFD_leftAnkle", 5.0f, -75.0f, 60.0f, 10.0f },
	{ "boneFD_leftToe", 5.0f, -30.0f, 30.0f, 10.0f },
	{ "effector_leftLeg", 100.0f, 0.0f, 0.0f, 50.0f },
};


class dWalkGenerator: public dAnimIDBlendNodePose
{
	public:
	dWalkGenerator(dAnimIDController* const character, dAnimIDRigEffector* const leftFeet, dAnimIDRigEffector* const rightFeet)
		:dAnimIDBlendNodePose(character)
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
		dAnimIDBlendNodePose::Evaluate(output, timestep);

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
timestep *= 0.01f;
		m_acc = dMod(m_acc + timestep, m_period);
	}

	dFloat m_acc;
	dFloat m_period;
	dFloat m_amplitud_x;
	dFloat m_amplitud_y;
	dBezierSpline m_cycle;
	int m_sequence[6];
	dAnimIDRigEffector* m_leftFeet;
	dAnimIDRigEffector* m_rightFeet;
};

class dAnimationBipeHipController: public dAnimIDBlendNode
{
	public:
	dAnimationBipeHipController(dAnimIDController* const character, dAnimIDBlendNode* const child)
		:dAnimIDBlendNode(character, child)
		, m_euler(0.0f)
		, m_position(0.0f)
	{
	}

	~dAnimationBipeHipController()
	{
	}

	virtual void Debug(dCustomJoint::dDebugDisplay* const debugContext) const
	{
	}

	virtual void Evaluate(dAnimationPose& output, dFloat timestep)
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
};

class dAnimationBalanceController: public dAnimIDBlendNode
{
	public:
	class dConvexHullPoint 
	{
		public:
		dConvexHullPoint ()
		{
		}

		dVector m_point;
		dConvexHullPoint *m_next;
	};

	dAnimationBalanceController(dAnimIDController* const character, dAnimIDBlendNode* const child)
		:dAnimIDBlendNode(character, child)
	{
	}

	int SupportPoint (int count, const dVector* const array, const dVector& dir) const
	{
		int index = 0;
		dFloat dist = array[0].DotProduct3(dir);
		for (int i = 1; i < count; i ++) {
			dFloat dist1 = array[i].DotProduct3(dir);
			if (dist1 > dist) {
				index = i;
				dist = dist1;
			}
		}
		return index;
	}

	int BuildSupportPolygon (dVector* const polygon, int maxCount) const
	{
		dAnimIDRigJoint* stackPool[32];
		int stack = 1;

		stackPool[0] = m_character;

		int pointCount = 0;
		dVector contactPoints[128];

		dVector point(0.0f);
		dVector normal(0.0f);

		while (stack) {
			stack--;

			const dAnimIDRigJoint* const node = stackPool[stack];
			NewtonBody* const newtonBody = node->GetNewtonBody();

			if (newtonBody) {
				for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(newtonBody); joint; joint = NewtonBodyGetNextContactJoint(newtonBody, joint)) {
					for (void* contact = NewtonContactJointGetFirstContact(joint); contact; contact = NewtonContactJointGetNextContact(joint, contact)) {
						NewtonMaterial* const material = NewtonContactGetMaterial(contact);

						NewtonMaterialGetContactPositionAndNormal(material, newtonBody, &point.m_x, &normal.m_x);
						contactPoints[pointCount] = point;
						pointCount++;
						if (pointCount >= sizeof (contactPoints) / sizeof (contactPoints[0])) {
							pointCount--;
						}
					}
				}

				const dList<dAnimAcyclicJoint*>& children = node->GetChildren();
				for (dList<dAnimAcyclicJoint*>::dListNode* child = children.GetFirst(); child; child = child->GetNext()) {
					stackPool[stack] = (dAnimIDRigJoint*)child->GetInfo();
					stack++;
				}
			}
		}

		int hullPoints = 0;
		if (pointCount > 3) {
			dVector median(0.0f);
			dVector variance(0.0f);
			dVector coVariance(0.0f);
			dVector origin(m_character->GetProxyBody()->GetMatrix().m_posit);

			for (int i = 0; i < pointCount; i ++) {
				dVector x (contactPoints[i] - origin);
				median += x;
				variance += x * x;
				coVariance += x * dVector (x.m_y, x.m_z, x.m_x, 0.0f);
				contactPoints[i] = x;
			}
			dFloat den = 1.0f / pointCount;

			median = median.Scale (den);
			variance = variance.Scale (den) - median * median;
			dFloat maxVariance = dMax(dMax(variance.m_x, variance.m_y), variance.m_z);
			if (maxVariance < 1.0e-3f) {
				return 0;
			}
			coVariance = coVariance.Scale (den) - median * dVector (median.m_y, median.m_z, median.m_x, 0.0f);

			dMatrix basisMatrix (dGetIdentityMatrix());
			basisMatrix[0][0] = variance.m_x;
			basisMatrix[1][1] = variance.m_y;
			basisMatrix[2][2] = variance.m_z;

			basisMatrix[0][1] = coVariance.m_x;
			basisMatrix[1][0] = coVariance.m_x;
			
			basisMatrix[0][2] = coVariance.m_z;
			basisMatrix[2][0] = coVariance.m_z;

			basisMatrix[1][2] = coVariance.m_y;
			basisMatrix[2][1] = coVariance.m_y;

			dVector eigenValues; 
			dMatrix axis (basisMatrix.JacobiDiagonalization (eigenValues));

			for (int i = 0; i < pointCount; i ++) {
				contactPoints[i] = contactPoints[i] - axis[1].Scale(axis[1].DotProduct3(contactPoints[i] - median));
			}
	

			dConvexHullPoint convexHull[32];
			dConvexHullPoint* hullStackBuffer[64];

			int index0 = SupportPoint (pointCount, contactPoints, axis[0]);
			convexHull[0].m_point = contactPoints[index0];
			pointCount --;
			dSwap (contactPoints[index0], contactPoints[pointCount]);

			index0 = SupportPoint(pointCount, contactPoints, axis[0].Scale (-1.0f));
			convexHull[1].m_point = contactPoints[index0];
			pointCount--;
			dSwap(contactPoints[index0], contactPoints[pointCount]);

			index0 = SupportPoint(pointCount, contactPoints, axis[2]);
			convexHull[2].m_point = contactPoints[index0];
			pointCount--;
			dSwap(contactPoints[index0], contactPoints[pointCount]);

			convexHull[0].m_next = &convexHull[1];
			convexHull[1].m_next = &convexHull[2];
			convexHull[2].m_next = &convexHull[0];
			dVector hullNormal ((convexHull[2].m_point - convexHull[0].m_point).CrossProduct (convexHull[1].m_point - convexHull[0].m_point));
			if (hullNormal.DotProduct3(hullNormal) < 1.0e-9f) {
				return 0;
			}

			int edgeAlloc = 3;
			int hullStack = 3;

			hullStackBuffer[0] = &convexHull[0];
			hullStackBuffer[1] = &convexHull[1];
			hullStackBuffer[2] = &convexHull[2];

			while (hullStack && pointCount) {
				hullStack--;
				dConvexHullPoint* const edge = hullStackBuffer[hullStack];
				
				dVector dir (hullNormal.CrossProduct(edge->m_next->m_point - edge->m_point));
				index0 = SupportPoint(pointCount, contactPoints, axis[0].Scale (-1.0f));

				dVector newPoint (contactPoints[index0]);
				dFloat dist (dir.DotProduct3 (newPoint - edge->m_point));
				if (dist > 1.0e-3f) {
					dConvexHullPoint* newEdge = &convexHull[edgeAlloc];
					edgeAlloc++;
					dAssert(edgeAlloc < sizeof(convexHull) / sizeof(convexHull[0]));
					newEdge->m_point = newPoint;
					newEdge->m_next = edge->m_next;
					edge->m_next = newEdge;

					hullStackBuffer[hullStack] = newEdge;
					hullStack++;
					hullStackBuffer[hullStack] = edge;
					hullStack++;

					pointCount--;
					dSwap(contactPoints[index0], contactPoints[pointCount]);
				}
			}

			dConvexHullPoint* edge = convexHull;
			do {
				polygon[hullPoints] = edge->m_point + origin;
				hullPoints++;
				edge = edge->m_next;
			} while (edge != convexHull);
		}
		return hullPoints;
	}

	virtual void Debug(dCustomJoint::dDebugDisplay* const debugContext) const
	{
		dVector polygon[32];
		int count = BuildSupportPolygon (polygon, sizeof (polygon) / sizeof (polygon[0]));
		if (count) {
			int i0 = count - 1;
			for (int i = 0; i < count; i++) {
				polygon[i].m_y += 0.2f;
			}

			debugContext->SetColor(dVector(1.0f, 1.0f, 0.0f, 1.0f));
			for (int i = 0; i < count; i++) {
				debugContext->DrawLine(polygon[i0], polygon[i]);
				i0 = i;
			}
		}
	}

	void Evaluate(dAnimationPose& output, dFloat timestep)
	{
		m_child->Evaluate(output, timestep);

/*
		dQuaternion rotation(dPitchMatrix(m_euler.m_x) * dYawMatrix(m_euler.m_y) * dRollMatrix(m_euler.m_z));
		for (dAnimationPose::dListNode* node = output.GetFirst(); node; node = node->GetNext()) {
			dAnimationTransform& transform = node->GetInfo();
			transform.m_rotation = transform.m_rotation * rotation;
			transform.m_posit = m_position + rotation.RotateVector(transform.m_posit);
		}
*/
	}
};



class dAnimationAnkleJoint: public dAnimIDRigForwardDynamicLimb
{
	public:
	dAnimationAnkleJoint(const dMatrix& basicMatrix, dAnimIDRigJoint* const parent, NewtonBody* const body, const dRagDollConfig& config)
		:dAnimIDRigForwardDynamicLimb(basicMatrix, parent, body)
	{
		SetFriction(config.m_frictionScale * config.m_mass * DEMO_MUSCLE_STRENGTH);
		SetLimits(config.m_minLimit * dDegreeToRad, config.m_maxLimit * dDegreeToRad);

		dMatrix matrix0;
		dMatrix matrix1;
		dMatrix rootMatrix(GetRoot()->GetBasePoseMatrix());
		CalculateGlobalMatrix(matrix0, matrix1);
		m_offsetAngle = dPi + CalculateAngle(rootMatrix.m_up, matrix0.m_up, rootMatrix.m_right);
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		dAnimIDRigForwardDynamicLimb::SubmitConstraints(timestep, threadIndex);

		dMatrix matrix0;
		dMatrix matrix1;
		dVector normal;
		
		CalculateGlobalMatrix(matrix0, matrix1);
	
//		NewtonWorld* const world = NewtonBodyGetWorld(GetBody0());
//		dVector floor (FindFloor(world, matrix1.m_posit, 10.0f, &normal));
//		dMatrix floorMatrix(dGetIdentityMatrix());
//		floorMatrix.m_front = matrix0.m_front;
//		floorMatrix.m_right = normal.CrossProduct(floorMatrix.m_front);
//		floorMatrix.m_right = floorMatrix.m_right.Normalize();
//		floorMatrix.m_up = floorMatrix.m_right.CrossProduct(floorMatrix.m_front);
		dAnimIDController* const root = GetRoot();
		dMatrix floorMatrix (dRollMatrix(dPi) * root->GetBasePoseMatrix());

		dFloat deltaAngle = CalculateAngle(floorMatrix.m_up, matrix0.m_up, matrix0.m_front) - m_offsetAngle;

		float speed = 3.0f;
		dFloat currentSpeed = 0.0f;
		dFloat step = speed * timestep;
		if (deltaAngle > step) {
			currentSpeed = -speed;
		} else if (deltaAngle < -step) {
			currentSpeed = speed;
		} else {
			currentSpeed = -0.3f * deltaAngle / timestep;
		}

		NewtonJoint* const joint = dCustomHinge::GetJoint();
		dFloat accel = NewtonUserJointCalculateRowZeroAccelaration(joint) + currentSpeed / timestep;
		NewtonUserJointSetRowAcceleration(joint, accel);
	}

	dFloat m_offsetAngle;
};


class dAnimationToeJoint : public dAnimIDRigForwardDynamicLimb
{
	public:
	dAnimationToeJoint(const dMatrix& basicMatrix, dAnimIDRigJoint* const parent, NewtonBody* const body, const dRagDollConfig& config)
		:dAnimIDRigForwardDynamicLimb(basicMatrix, parent, body)
	{
		SetFriction(config.m_frictionScale * config.m_mass * DEMO_MUSCLE_STRENGTH);
		SetLimits(config.m_minLimit * dDegreeToRad, config.m_maxLimit * dDegreeToRad);

		dMatrix matrix0;
		dMatrix matrix1;
		CalculateGlobalMatrix(matrix0, matrix1);
		dMatrix rootMatrix(dYawMatrix(dPi) * dPitchMatrix(dPi * 0.5f) * GetRoot()->GetBasePoseMatrix());
		m_offsetAngle = CalculateAngle(matrix0.m_right, rootMatrix.m_right, rootMatrix.m_front);
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		dAnimIDRigForwardDynamicLimb::SubmitConstraints(timestep, threadIndex);

		dMatrix matrix0;
		dMatrix matrix1;
		dVector normal;

		CalculateGlobalMatrix(matrix0, matrix1);

		//NewtonWorld* const world = NewtonBodyGetWorld(GetBody0());
		//dVector floor (FindFloor(world, matrix1.m_posit, 10.0f, &normal));
		//dMatrix floorMatrix(dGetIdentityMatrix());
		//floorMatrix.m_front = matrix0.m_front;
		//floorMatrix.m_right = normal.CrossProduct(floorMatrix.m_front);
		//floorMatrix.m_right = floorMatrix.m_right.Normalize();
		//floorMatrix.m_up = floorMatrix.m_right.CrossProduct(floorMatrix.m_front);

		dMatrix rootMatrix(dYawMatrix(dPi) * dPitchMatrix(dPi * 0.5f) * GetRoot()->GetBasePoseMatrix());
		dFloat deltaAngle = -CalculateAngle(matrix0.m_right, rootMatrix.m_right, rootMatrix.m_front) + m_offsetAngle;;

		float speed = 3.0f;
		dFloat currentSpeed = 0.0f;
		dFloat step = speed * timestep;
		if (deltaAngle > step) {
			currentSpeed = -speed;
		} else if (deltaAngle < -step) {
			currentSpeed = speed;
		} else {
			currentSpeed = -0.3f * deltaAngle / timestep;
		}

		NewtonJoint* const joint = dCustomHinge::GetJoint();
		dFloat accel = NewtonUserJointCalculateRowZeroAccelaration(joint) + currentSpeed / timestep;
		NewtonUserJointSetRowAcceleration(joint, accel);
	}

	dFloat m_offsetAngle;
};



class BalancingDummyManager : public dAnimIDManager
{
	public:

	class dAnimationCharacterUserData: public DemoEntity::UserData
	{
		public:
		dAnimationCharacterUserData(dAnimIDController* const rig, dAnimIDBlendNodeTwoWay* const walk, dAnimationBipeHipController* const posture)
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

		dAnimIDController* m_rig;
		dAnimIDBlendNodeTwoWay* m_walk;
		dAnimationBipeHipController* m_posture;

		dFloat m_hipHigh;
		dFloat m_walkSpeed;
	};


	BalancingDummyManager(DemoEntityManager* const scene)
		:dAnimIDManager(scene->GetNewton())
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
		dAnimIDManager::OnDebug(debugContext);
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

	dAnimIDController* CreateRagDoll(DemoEntityManager* const scene, const dMatrix& origin)
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

//NewtonBodySetMassMatrix(rootBody, 0.0f, 0.0f, 0.0f, 0.0f);

		DemoEntity* const localFrame = model->Find("rootLocalFrame");
		dAssert(localFrame);
		dMatrix localFrameMatrix(localFrame->CalculateGlobalMatrix());
		dAnimIDController* const rig = CreateCharacterRig(rootBody, localFrameMatrix);

		int stackIndex = 0;
		DemoEntity* childEntities[32];
		dAnimIDRigJoint* parentBones[32];
		for (DemoEntity* child = model->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = rig;
			childEntities[stackIndex] = child;
			stackIndex++;
		}

		dAnimIDRigEffector* leftFeet = NULL;
		dAnimIDRigEffector* rightFeet = NULL;

		const int partCount = sizeof(ragDollConfig) / sizeof(ragDollConfig[0]);
		while (stackIndex) {
			stackIndex--;
			DemoEntity* const entity = childEntities[stackIndex];
			dAnimIDRigJoint* const parentJoint = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			for (int i = 0; i < partCount; i++) {
				if (!strcmp(ragDollConfig[i].m_partName, name)) {

					if (strstr(name, "bone")) {
						// add a bone and all it children
						NewtonBody* const limbBody = CreateBodyPart(entity, ragDollConfig[i]);

						dMatrix matrix;
						NewtonBodyGetMatrix(limbBody, &matrix[0][0]);

						dAnimIDRigLimb* limbJoint = NULL;
						if (strstr(name, "boneFD")) {
							dAnimIDRigForwardDynamicLimb* footJoint = NULL;
							if (strstr(name, "Ankle")) {
								footJoint = new dAnimationAnkleJoint(matrix, parentJoint, limbBody, ragDollConfig[i]);
							} else {
								footJoint = new dAnimationToeJoint(matrix, parentJoint, limbBody, ragDollConfig[i]);
							}
							limbJoint = footJoint;
						} else {
							dAnimIDRigHinge* const hinge = new dAnimIDRigHinge(matrix, parentJoint, limbBody);
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
						dAnimIDRigEffector* const effector = new dAnimIDRigEffector(pivot, parentJoint->GetAsRigLimb(), rig);
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

		dAnimIDBlendNodePose* const fixPose = new dAnimIDBlendNodePose(rig);
		dAnimIDBlendNodePose* const walkPose = new dWalkGenerator(rig, leftFeet, rightFeet);
		dAnimIDBlendNodeTwoWay* const walkBlend = new dAnimIDBlendNodeTwoWay(rig, fixPose, walkPose);
		dAnimationBipeHipController* const posture = new dAnimationBipeHipController (rig, walkBlend);
		dAnimationBalanceController* const balance = new dAnimationBalanceController (rig, posture);
		dAnimIDBlendNodeRoot* const animTree = new dAnimIDBlendNodeRoot(rig, balance);

		dAnimationCharacterUserData* const renderCallback = new dAnimationCharacterUserData(rig, walkBlend, posture);
		model->SetUserData(renderCallback);
		
		rig->SetAnimationTree (animTree);

		m_currentRig = rig;
		return rig;
	}

	void OnUpdateTransform(const dAnimIDRigJoint* const bone, const dMatrix& localMatrix) const
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

			dAnimIDBlendNodeTwoWay* const walkBlend = controlData->m_walk;
			walkBlend->SetParam (controlData->m_walkSpeed);

			dAnimationBipeHipController* const posture = controlData->m_posture;
			posture->m_position.m_y = 0.25f * controlData->m_hipHigh;
		}

		dAnimIDManager::PreUpdate(timestep);
	}

	dAnimIDController* m_currentRig;
};

void DynamicRagDoll(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	CreateLevelMesh(scene, "flatPlane.ngd", true);
	BalancingDummyManager* const robotManager = new BalancingDummyManager(scene);

	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID(world);
	NewtonMaterialSetDefaultFriction(world, defaultMaterialID, defaultMaterialID, 1.0f, 1.0f);
	NewtonMaterialSetDefaultElasticity(world, defaultMaterialID, defaultMaterialID, 0.0f);


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




