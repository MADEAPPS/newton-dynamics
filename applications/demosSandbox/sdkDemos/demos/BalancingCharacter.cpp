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


class dBalancingRagdollBoneDefinition
{
	public:
	enum jointType
	{
		m_none,
		m_ball,
		m_0dof,
		m_1dof,
		m_2dof,
		m_3dof,
		m_effector,
	};

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

	char* m_boneName;
	jointType m_type;
	dFloat m_massFraction;
	dJointLimit m_jointLimits;
	dFrameMatrix m_frameBasics;
};

static dBalancingRagdollBoneDefinition tredDefinition[] =
{
	{ "bone_pelvis", dBalancingRagdollBoneDefinition::m_none, 1.0f },

//{ "bone_rightLeg", dBalancingRagdollBoneDefinition::m_2dof, 0.3f, {60.0f, 60.0f, 70.0f}, { 0.0f, 0.0f, 45.0f }},
//{ "bone_righCalf", dBalancingRagdollBoneDefinition::m_1dof, 0.2f, {-60.0f, 60.0f, 0.0f}, { 0.0f, 0.0f, 90.0f }},

	{ "bone_rightLeg", dBalancingRagdollBoneDefinition::m_3dof, 0.3f, {60.0f, 60.0f, 70.0f}, { 0.0f, 90.0f, 0.0f }},
	{ "bone_righCalf", dBalancingRagdollBoneDefinition::m_1dof, 0.2f, {-60.0f, 60.0f, 0.0f}, { 0.0f, 0.0f, 90.0f }},
	{ "bone_rightAnkle", dBalancingRagdollBoneDefinition::m_0dof, 0.2f, {0.0f, 0.0f, 0.0f}, { 0.0f, 0.0f, 0.0f }},
	{ "bone_rightFoot", dBalancingRagdollBoneDefinition::m_3dof, 0.2f, {0.0f, 0.0f, 45.0f}, { 0.0f, 0.0f, 180.0f }},
	{ "effector_rightAnkle", dBalancingRagdollBoneDefinition::m_effector},

	{ "bone_leftLeg", dBalancingRagdollBoneDefinition::m_3dof, 0.3f, { 60.0f, 60.0f, 70.0f }, { 0.0f, 90.0f, 0.0f } },
	{ "bone_leftCalf", dBalancingRagdollBoneDefinition::m_1dof, 0.2f, {-60.0f, 60.0f, 0.0f }, { 0.0f, 0.0f, 90.0f } },
	{ "bone_leftAnkle", dBalancingRagdollBoneDefinition::m_0dof, 0.2f, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f } },
	{ "bone_leftFoot", dBalancingRagdollBoneDefinition::m_3dof, 0.2f, { 0.0f, 0.0f, 45.0f }, { 0.0f, 0.0f, 180.0f } },
	{ "effector_leftAnkle", dBalancingRagdollBoneDefinition::m_effector },

	{ NULL},
};

class dModelDescritor
{
	public:
	char* m_filename;
	dFloat m_mass;
	dBalancingRagdollBoneDefinition* m_skeletonDefinition;
};

static dModelDescritor tred = {"tred.ngd", 500.0f, tredDefinition};


class dBalacingRagollEffector: public dCustomKinematicController
{
	public:
	dBalacingRagollEffector(NewtonBody* const body, NewtonBody* const referenceBody, const dMatrix& attachmentMatrixInGlobalSpace, dFloat modelMass, const dVector& localPivot)
		:dCustomKinematicController(body, attachmentMatrixInGlobalSpace, referenceBody)
	{
		dMatrix matrix1(GetMatrix1());
		m_localPivot = GetMatrix1();
		m_localPivot.m_posit = localPivot;
		m_localPivot = matrix1 * m_localPivot.Inverse();

		m_x = matrix1.m_posit.m_x - m_localPivot.m_posit.m_x;
		m_y = matrix1.m_posit.m_y - m_localPivot.m_posit.m_y;
		m_z = matrix1.m_posit.m_z - m_localPivot.m_posit.m_z;
		m_angle = 0.0f;

		SetSolverModel(1);
		SetControlMode(dCustomKinematicController::m_linearAndTwist);
		SetMaxAngularFriction(modelMass * 100.0f);
		SetMaxLinearFriction(modelMass * 9.8f * 10.0f);
	}


	dMatrix m_localPivot;
	dFloat m_x;
	dFloat m_y;
	dFloat m_z;
	dFloat m_angle;
};

class dModelAnimTreeFootBase: public dModelAnimTree
{
	public:
	dModelAnimTreeFootBase(dModelRootNode* const model, dModelAnimTree* const child, dCustomKinematicController* const footEffector0, dCustomKinematicController* const footEffector1)
		:dModelAnimTree(model)
		,m_child(child)
		,m_rootEffector0(footEffector0)
		,m_rootEffector1(footEffector1)
	{
	}

	~dModelAnimTreeFootBase()
	{
		delete m_child;
	}

	virtual void Debug(dCustomJoint::dDebugDisplay* const debugContext) const
	{
		m_child->Debug(debugContext);
	}

	int GetModelKeyFrames(const dModelKeyFramePose& input, dModelKeyFrame** const output) const
	{
		int count = 0;
		for (dModelKeyFramePose::dListNode* node = input.GetFirst(); node; node = node->GetNext()) {
			dModelKeyFrame& transform = node->GetInfo();
			if ((transform.m_effector == m_rootEffector0) || (transform.m_effector == m_rootEffector1)) {
				output[count] = &transform;
				count++;
			}
		}
		return count;
	}

	dModelAnimTree* m_child;
	dCustomKinematicController* m_rootEffector0;
	dCustomKinematicController* m_rootEffector1;
};

class dModelAnimTreePoseBalance: public dModelAnimTreeFootBase
{
	public:
	dModelAnimTreePoseBalance(dModelRootNode* const model, dModelAnimTree* const child, dCustomKinematicController* const footEffector0, dCustomKinematicController* const footEffector1)
		:dModelAnimTreeFootBase(model, child, footEffector0, footEffector1)
	{
	}

	virtual void Debug(dCustomJoint::dDebugDisplay* const debugContext) const
	{
		dMatrix matrix;
		NewtonBodyGetMatrix(GetRoot()->GetBody(), &matrix[0][0]);
		matrix.m_posit = CalculateCenterOfMass();
		debugContext->DrawFrame(matrix);
		m_child->Debug(debugContext);
	}

/*
	bool CalculateUpVector(dVector& upvector, dCustomKinematicController* const effector) const
	{
		bool ret = false;
		NewtonBody* const body0 = effector->GetBody0();
		for (NewtonJoint* contactjoint = NewtonBodyGetFirstContactJoint(body0); contactjoint; contactjoint = NewtonBodyGetNextContactJoint(body0, contactjoint)) {
			ret = true;
			dVector averageNormal(0.0f);
			for (void* contact = NewtonContactJointGetFirstContact(contactjoint); contact; contact = NewtonContactJointGetNextContact(contactjoint, contact)) {
				NewtonMaterial* const material = NewtonContactGetMaterial(contact);
				dVector point(0.0f);
				dVector normal(0.0f);
				NewtonMaterialGetContactPositionAndNormal(material, body0, &point.m_x, &normal.m_x);
				averageNormal += normal;
			}
			upvector = averageNormal.Normalize();
			break;
		}

		ret = true;
		upvector = dVector (0.0f, 1.0f, 0.0f, 0.0f);
		upvector = dPitchMatrix(30.0f * dDegreeToRad).RotateVector(upvector);

		return ret;

	}
*/

	dVector CalculateCenterOfMass() const
	{
		dMatrix matrix;
		dVector com(0.0f);
		dVector localCom;
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		dFloat mass;
		dFloat totalMass = 0.0f;

		int stack = 1;
		int bodyCount = 0;
		const dModelNode* stackBuffer[32];

		stackBuffer[0] = GetRoot();
		while (stack) {
			stack--;
			const dModelNode* const root = stackBuffer[stack];

			NewtonBody* const body = root->GetBody();
			NewtonBodyGetMatrix(body, &matrix[0][0]);
			NewtonBodyGetCentreOfMass(body, &localCom[0]);
			NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);

			totalMass += mass;
			com += matrix.TransformVector(localCom).Scale(mass);

			bodyCount++;
			const dModelChildrenList& children = root->GetChildren();
			for (const dModelChildrenList::dListNode* node = children.GetFirst(); node; node = node->GetNext()) {
				stackBuffer[stack] = node->GetInfo().GetData();
				stack++;
			}
		}

		com = com.Scale(1.0f / totalMass);
		com.m_w = 1.0f;
		return com;
	}

	void GeneratePose(dFloat timestep, dModelKeyFramePose& output)
	{
		m_child->GeneratePose(timestep, output);
/*		
		dModelKeyFrame* feetPose[2];
		const int count = GetModelKeyFrames(output, &feetPose[0]);
		for (int i = 0; i < count; i++) {
			dMatrix targetMatrix (m_rootEffector0->GetBodyMatrix());
			dVector com (CalculateCenterOfMass());

			dVector error (targetMatrix.m_posit - com);
			//error.m_x = 0.0f;
			error.m_y = 0.0f;
			//error.m_z = 0.0f;

			targetMatrix.m_posit -= error.Scale (0.35f);
			//dTrace (("(%f %f %f) (%f %f %f)\n", com.m_x, com.m_y, com.m_z, targetMatrix.m_posit.m_x, targetMatrix.m_posit.m_y, targetMatrix.m_posit.m_z));

			dMatrix rootMatrix;
			NewtonBodyGetMatrix (m_rootEffector0->GetBody1(), &rootMatrix[0][0]);

			dMatrix effectorMatrix (targetMatrix * rootMatrix.Inverse());
			for (dModelKeyFramePose::dListNode* node = output.GetFirst(); node; node = node->GetNext()) {
				//dModelKeyFrame& transform = node->GetInfo();
				//transform.m_posit = effectorMatrix.m_posit;
				//transform.SetMatrix(effectorMatrix);
			}
		}
*/
	}
};

/*
class dModelAnimTreeFootAlignment: public dModelAnimTreeFootBase
{
	#define RAY_CAST_LENGHT0	0.125f
	#define RAY_CAST_LENGHT1	2.0f
	class dFeetRayHitSensor
	{
		public:
		dFeetRayHitSensor(dCustomKinematicController* const effector)
			:m_rotation()
			,m_footBody(NULL)
			,m_ankleBody(NULL)
			,m_effector(effector)
		{
			if (m_effector) {
				m_footBody = effector->GetBody0();
				NewtonCollision* const box = NewtonBodyGetCollision(m_footBody);

				dMatrix matrix;
				NewtonBodyGetMatrix(m_footBody, &matrix[0][0]);

				dVector worldDir[] = { 
					{ 1.0f, -1.0f, 1.0, 0.0f },
					{ -1.0f, -1.0f, 1.0, 0.0f },
					{ -1.0f, -1.0f, -1.0, 0.0f },
					{ 1.0f, -1.0f, -1.0, 0.0f }
				};

				for (int i = 0; i < 4; i++) {
					dVector dir = matrix.UnrotateVector(worldDir[i]);
					NewtonCollisionSupportVertex(box, &dir[0], &m_sensorHitPoint[i][0]);
					m_sensorHitPoint[i].m_w = 0.0f;
				}

				for (NewtonJoint* joint = NewtonBodyGetFirstJoint(m_footBody); joint; joint = NewtonBodyGetNextJoint(m_footBody, joint)) {
					dCustomJoint* const customJoint = (dCustomJoint*)NewtonJointGetUserData(joint);
					NewtonBody* const body0 = customJoint->GetBody0();
					NewtonBody* const body1 = customJoint->GetBody1();
					NewtonBody* const otherBody = (body0 == m_footBody) ? body1 : body0;
					if (otherBody != m_footBody) {
						m_ankleBody = otherBody;
						break;
					}
				}
			}
		}

		dCustomKinematicController* GetEffector() const
		{
			return m_effector;
		}

		void DebugDraw(dCustomJoint::dDebugDisplay* const debugContext) const
		{
			if (m_effector) {
				dMatrix matrix;
				NewtonBodyGetMatrix(m_footBody, &matrix[0][0]);

				for (int i = 0; i < 4; i++) {
					dVector point0(matrix.TransformVector(m_sensorHitPoint[i]));
					dVector point1(point0);
					point0.m_y += RAY_CAST_LENGHT0;
					point1.m_y -= RAY_CAST_LENGHT1;
					debugContext->DrawLine(point0, point1);
				}
			}
		}

		void AlignMatrix(dModelKeyFrame* const transform, dFloat timestep)
		{
			dVector upvector;
			if (CalculateSupportNormal(upvector)) {
				// calculate foot desired matrix
				dMatrix rootMatrix;
				NewtonBodyGetMatrix(m_effector->GetBody1(), &rootMatrix[0][0]);
				dMatrix targetMatrix(dMatrix(transform->m_rotation, transform->m_posit) * rootMatrix);
				dFloat cosAngle = upvector.DotProduct3(targetMatrix.m_up);
				if (cosAngle < 0.9997f) {
					if (cosAngle > 0.87f) {
						// align the matrix to the floor contacts.
						dVector lateralDir(targetMatrix.m_up.CrossProduct(upvector));
						lateralDir = lateralDir.Normalize();
						dFloat coneAngle = dAcos(dClamp(cosAngle, dFloat(-1.0f), dFloat(1.0f)));

						dMatrix pivotMatrix(m_effector->GetMatrix0().Inverse() * targetMatrix);

						dQuaternion rotation(lateralDir, coneAngle);
						dMatrix coneRotation(rotation, pivotMatrix.m_posit);
						coneRotation.m_posit -= coneRotation.RotateVector(pivotMatrix.m_posit);
						targetMatrix = targetMatrix * coneRotation;

						// calculate and set new modified effector matrix.
						transform->SetMatrix(targetMatrix * rootMatrix.Inverse());
					} else {
						cosAngle = 0;
					}
				}
			}
		}

		private:
		static unsigned FindFloorPrefilter(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData)
		{
			dFeetRayHitSensor* const data = (dFeetRayHitSensor*)userData;
			return ((data->m_footBody == body) || (data->m_ankleBody == body)) ? 0 : 1;
		}

		static dFloat FindFloor(const NewtonBody* const body, const NewtonCollision* const shapeHit, const dFloat* const hitContact, const dFloat* const hitNormal, dLong collisionID, void* const userData, dFloat intersectParam)
		{
			dFeetRayHitSensor* const data = (dFeetRayHitSensor*)userData;
			dAssert(body != data->m_footBody);
			dAssert(body != data->m_ankleBody);
			if (intersectParam < data->m_hitParam) {
				data->m_hitParam = intersectParam;
			}
			return data->m_hitParam;
		}

		bool CalculateSupportNormal(dVector& upvector)
		{
			bool ret = false;

			dMatrix matrix;
			dVector supportPlane[4];
			NewtonWorld* const world = NewtonBodyGetWorld(m_footBody);

			NewtonBodyGetMatrix(m_footBody, &matrix[0][0]);

			int count = 0;
			for (int i = 0; i < 4; i++) {
				dVector point0(matrix.TransformVector(m_sensorHitPoint[i]));
				dVector point1(point0);
				m_hitParam = 1.2f;
				point0.m_y += RAY_CAST_LENGHT0;
				point1.m_y -= RAY_CAST_LENGHT1;
				NewtonWorldRayCast(world, &point0[0], &point1[0], FindFloor, this, FindFloorPrefilter, 0);
				if (m_hitParam < 1.0f) {
					supportPlane[count] = point0 + (point1 - point0).Scale(m_hitParam);
					count++;
				}
			}

			if (count >= 3) {
				dVector area(0.0f);
				dVector e0(supportPlane[1] - supportPlane[0]);
				for (int i = 2; i < count; i++) {
					dVector e1(supportPlane[i] - supportPlane[0]);
					area += e1.CrossProduct(e0);
					e0 = e1;
				}
				ret = true;
				upvector = area.Normalize();
				//upvector = dPitchMatrix(45.0f * dDegreeToRad).RotateVector(dVector (0, 1.0f, 0.0f, 0.0f));
			}

			ret = true;
			static dFloat angle = 0.0f;
			dMatrix xxxx(dPitchMatrix(angle));
			angle -= 0.002f;
			upvector = xxxx[1];
			//upvector = area.Normalize();
			//upvector = dPitchMatrix(45.0f * dDegreeToRad).RotateVector(dVector (0, 1.0f, 0.0f, 0.0f));

			return ret;
		}

		dVector m_sensorHitPoint[4];
		dQuaternion m_rotation;
		NewtonBody* m_footBody;
		NewtonBody* m_ankleBody;
		dCustomKinematicController* m_effector;
		dFloat m_hitParam;
	};

	public:
	dModelAnimTreeFootAlignment(dModelRootNode* const model, dModelAnimTree* const child, dCustomKinematicController* const footEffector0, dCustomKinematicController* const footEffector1)
		:dModelAnimTreeFootBase(model, child, footEffector0, footEffector1)
		,m_foot0(footEffector0)
		,m_foot1(footEffector1)
	{
	}

	virtual void Debug(dCustomJoint::dDebugDisplay* const debugContext) const
	{
		m_foot0.DebugDraw(debugContext);
		m_foot1.DebugDraw(debugContext);
		m_child->Debug(debugContext);
	}

	void GeneratePose(dFloat timestep, dModelKeyFramePose& output)
	{
		m_child->GeneratePose(timestep, output);

		dModelKeyFrame* feetPose[2];
		const int count = GetModelKeyFrames(output, &feetPose[0]);
		for (int i = 0; i < count; i++) {

			dMatrix rootMatrix;
			dModelKeyFrame* const transform = feetPose[i];
			dCustomKinematicController* const effector = transform->m_effector;
			if (effector == m_foot0.GetEffector()) {
				m_foot0.AlignMatrix(transform, timestep);
			}
			if (effector == m_foot1.GetEffector()) {
				m_foot1.AlignMatrix(transform, timestep);
			}
		}
	}

	dFeetRayHitSensor m_foot0;
	dFeetRayHitSensor m_foot1;
};
*/

class dBalancingRagdoll: public dModelRootNode
{
	public:
	dBalancingRagdoll(NewtonBody* const rootBody)
		:dModelRootNode(rootBody, dGetIdentityMatrix())
		,m_pose()
		,m_animtree(NULL)
	{
	}

	~dBalancingRagdoll()
	{
		if (m_animtree) {
			delete m_animtree;
		}
	}

	void SetAllPartsNonCollidable()
	{
		int stack = 1;
		dModelNode* stackBuffer[32];

		stackBuffer[0] = this;
		void* const aggregate = NewtonCollisionAggregateCreate(NewtonBodyGetWorld(GetBody()));
		while (stack) {
			stack--;
			dModelNode* const root = stackBuffer[stack];
			NewtonBody* const body = root->GetBody();
			NewtonCollisionAggregateAddBody(aggregate, body);

			const dModelChildrenList& children = root->GetChildren();
			for (dModelChildrenList::dListNode* node = children.GetFirst(); node; node = node->GetNext()) {
				stackBuffer[stack] = node->GetInfo().GetData();
				stack++;
			}
		}

		NewtonCollisionAggregateSetSelfCollision(aggregate, false);
	}


	void SetAnimTree(dCustomKinematicController* const rootEffector0, dCustomKinematicController* const rootEffector1)
	{
		dModelAnimTree* const poseGenerator = new dModelAnimTreePose(this, m_pose);
		dModelAnimTreePoseBalance* const poseBalance = new dModelAnimTreePoseBalance(this, poseGenerator, rootEffector0, rootEffector1);
		//dModelAnimTree* const footRoll = new dModelAnimTreeFootAlignment(this, poseBalance, rootEffector0, rootEffector1);
		//m_animtree = footRoll;
		m_animtree = poseBalance;
	}

	void OnDebug(dCustomJoint::dDebugDisplay* const debugContext)
	{
		dFloat scale = debugContext->GetScale();
		debugContext->SetScale(0.5f);
		for (dModelKeyFramePose::dListNode* node = m_pose.GetFirst(); node; node = node->GetNext()) {
			const dModelKeyFrame& keyFrame = node->GetInfo();
			keyFrame.m_effector->Debug(debugContext);
		}

		if (m_animtree) {
			m_animtree->Debug(debugContext);
		}

		debugContext->SetScale(scale);
	}

	void ApplyControls (dFloat timestep)
	{
		m_animtree->GeneratePose(timestep, m_pose);

		for (dModelKeyFramePose::dListNode* node = m_pose.GetFirst(); node; node = node->GetNext()) {
			dModelKeyFrame& transform = node->GetInfo();
			transform.m_effector->SetTargetMatrix(dMatrix(transform.m_rotation, transform.m_posit));
		}
	}

	dModelKeyFramePose m_pose;
	dModelAnimTree* m_animtree;
};


class dBalancingRagdollManager: public dModelManager
{
	public:
	dBalancingRagdollManager(DemoEntityManager* const scene)
		:dModelManager(scene->GetNewton())
		//,m_currentController(NULL)
	{
		//scene->Set2DDisplayRenderFunction(RenderHelpMenu, NULL, this);
	}

	~dBalancingRagdollManager()
	{
	}

	static void RenderHelpMenu(DemoEntityManager* const scene, void* const context)
	{
		dAssert (0);
		//dVector color(1.0f, 1.0f, 0.0f, 0.0f);
		//dBalancingRagdollManager* const me = (dBalancingRagdollManager*)context;
		//scene->Print(color, "linear degrees of freedom");
		//ImGui::SliderFloat("Azimuth", &me->m_azimuth, -180.0f, 180.0f);
		//ImGui::SliderFloat("posit_x", &me->m_posit_x, -1.4f, 0.2f);
		//ImGui::SliderFloat("posit_y", &me->m_posit_y, -1.2f, 0.4f);
		//
		//ImGui::Separator();
		//scene->Print(color, "angular degrees of freedom");
		//ImGui::SliderFloat("pitch", &me->m_gripper_pitch, -180.0f, 180.0f);
		//ImGui::SliderFloat("yaw", &me->m_gripper_yaw, -80.0f, 80.0f);
		//ImGui::SliderFloat("roll", &me->m_gripper_roll, -180.0f, 180.0f);
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

	NewtonBody* CreateBodyPart(DemoEntity* const bodyPart, const dBalancingRagdollBoneDefinition& definition)
	{
		NewtonWorld* const world = GetWorld();
		NewtonCollision* const shape = bodyPart->CreateCollisionFromchildren(world);
		dAssert(shape);

		// calculate the boneBody matrix
		dMatrix matrix(bodyPart->CalculateGlobalMatrix());

		// create the rigid body that will make this boneBody
		NewtonBody* const boneBody = NewtonCreateDynamicBody(world, shape, &matrix[0][0]);

		// calculate the moment of inertia and the relative center of mass of the solid
		//NewtonBodySetMassProperties (boneBody, definition.m_mass, shape);
		NewtonBodySetMassProperties(boneBody, definition.m_massFraction, shape);

		// save the user data with the boneBody body (usually the visual geometry)
		NewtonBodySetUserData(boneBody, bodyPart);

		// assign the material for early collision culling
		//NewtonBodySetMaterialGroupID(boneBody, m_material);
		NewtonBodySetMaterialGroupID(boneBody, 0);

		//dVector damping (0.0f);
		//NewtonBodySetLinearDamping(boneBody, damping.m_x);
		//NewtonBodySetAngularDamping(boneBody, &damping[0]);

		// set the bod part force and torque call back to the gravity force, skip the transform callback
		//NewtonBodySetForceAndTorqueCallback (boneBody, PhysicsApplyGravityForce);
		NewtonBodySetForceAndTorqueCallback(boneBody, ClampAngularVelocity);

		// destroy the collision helper shape 
		NewtonDestroyCollision(shape);
		return boneBody;
	}

	dBalacingRagollEffector* ConnectEffector(dModelNode* const effectorNode, const dMatrix& effectorMatrix, const dFloat modelMass)
	{
		const dModelNode* const referenceNode = effectorNode->GetParent()->GetParent();
		dAssert(referenceNode);
		dCustomJoint* const joint = FindJoint(referenceNode->GetBody(), referenceNode->GetParent()->GetBody());
		dAssert(joint);
		dAssert(joint->GetBody1() == effectorNode->GetRoot()->GetBody());
		dMatrix pivotMatrix(joint->GetMatrix1());
		dBalacingRagollEffector* const effector = new dBalacingRagollEffector(effectorNode->GetBody(), effectorNode->GetRoot()->GetBody(), effectorMatrix, modelMass, pivotMatrix.m_posit);
		return effector;
	}

	void ConnectLimb(NewtonBody* const bone, NewtonBody* const parent, const dBalancingRagdollBoneDefinition& definition)
	{
		dMatrix matrix;
		NewtonBodyGetMatrix(bone, &matrix[0][0]);

		dBalancingRagdollBoneDefinition::dFrameMatrix frameAngle(definition.m_frameBasics);
		dMatrix pinAndPivotInGlobalSpace(dPitchMatrix(frameAngle.m_pitch * dDegreeToRad) * dYawMatrix(frameAngle.m_yaw * dDegreeToRad) * dRollMatrix(frameAngle.m_roll * dDegreeToRad));
		pinAndPivotInGlobalSpace = pinAndPivotInGlobalSpace * matrix;

		switch (definition.m_type)
		{
			case dBalancingRagdollBoneDefinition::m_ball:
			{
				dCustomBallAndSocket* const joint = new dCustomBallAndSocket(pinAndPivotInGlobalSpace, bone, parent);
				joint->EnableCone(false);
				joint->EnableTwist(false);
				break;
			}

			case dBalancingRagdollBoneDefinition::m_0dof:
			{
				dCustomHinge* const joint = new dCustomHinge(pinAndPivotInGlobalSpace, bone, parent);
				joint->EnableLimits(true);
				joint->SetLimits(0.0f, 0.0f);
				break;
			}

			case dBalancingRagdollBoneDefinition::m_1dof:
			{
#if 0
				dCustomHinge* const joint = new dCustomHinge(pinAndPivotInGlobalSpace, bone, parent);
				joint->EnableLimits(true);
				joint->SetLimits(definition.m_jointLimits.m_minTwistAngle * dDegreeToRad, definition.m_jointLimits.m_maxTwistAngle * dDegreeToRad);
#else
				dCustomBallAndSocket* const joint = new dCustomBallAndSocket(pinAndPivotInGlobalSpace, bone, parent);
				joint->EnableTwist(true);
				joint->SetTwistLimits(definition.m_jointLimits.m_minTwistAngle * dDegreeToRad, definition.m_jointLimits.m_maxTwistAngle * dDegreeToRad);

				joint->EnableCone(true);
				joint->SetConeLimits(0.0f);
#endif
				break;
			}

			case dBalancingRagdollBoneDefinition::m_2dof:
			{
				dCustomBallAndSocket* const joint = new dCustomBallAndSocket(pinAndPivotInGlobalSpace, bone, parent);
				joint->EnableTwist(true);
				joint->SetTwistLimits(0.0f, 0.0f);

				joint->EnableCone(true);
				joint->SetConeLimits(definition.m_jointLimits.m_coneAngle * dDegreeToRad);
				break;
			}

			case dBalancingRagdollBoneDefinition::m_3dof:
			{
				dCustomBallAndSocket* const joint = new dCustomBallAndSocket(pinAndPivotInGlobalSpace, bone, parent);
				joint->EnableTwist(true);
				joint->SetTwistLimits(definition.m_jointLimits.m_minTwistAngle * dDegreeToRad, definition.m_jointLimits.m_maxTwistAngle * dDegreeToRad);

				joint->EnableCone(true);
				joint->SetConeLimits(definition.m_jointLimits.m_coneAngle * dDegreeToRad);
				break;
			}

			default:
				dAssert (0);
		}
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
			if (minInertia < 10.0f) {
				dFloat maxInertia = dMax(dFloat(10.0f), dMax(Ixx, dMax(Iyy, Izz)));
				Ixx = maxInertia;
				Iyy = maxInertia;
				Izz = maxInertia;
			}

			NewtonBodySetMassMatrix(body, mass, Ixx, Iyy, Izz);
		}
	}

	void CreateKinematicModel(dModelDescritor& descriptor, const dMatrix& location) 
	{
		NewtonWorld* const world = GetWorld();
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

		DemoEntity* const entityModel = DemoEntity::LoadNGD_mesh(descriptor.m_filename, world, scene->GetShaderCache());
		scene->Append(entityModel);
		dMatrix matrix0(entityModel->GetCurrentMatrix());
		entityModel->ResetMatrix(*scene, matrix0 * location);

		// create the root body, do not set the transform call back 
		NewtonBody* const rootBody = CreateBodyPart(entityModel, descriptor.m_skeletonDefinition[0]);

		// make a kinematic controlled model.
		dBalancingRagdoll* const model = new dBalancingRagdoll(rootBody);

		// add the model to the manager
		AddRoot(model);

		// the the model to calculate the local transformation
		model->SetTranformMode(true);

		// save the controller as the collision user data, for collision culling
		//NewtonCollisionSetUserData(NewtonBodyGetCollision(rootBone), controller);

		int stackIndex = 0;
		dModelNode* parentBones[32];
		DemoEntity* childEntities[32];
		for (DemoEntity* child = entityModel->GetChild(); child; child = child->GetSibling()) {
			parentBones[stackIndex] = model;
			childEntities[stackIndex] = child;
			stackIndex++;
			//dTrace(("name: %s\n", child->GetName().GetStr()));
		}

		// walk model hierarchic adding all children designed as rigid body bones. 
		int footEffectorsCoint = 0;
		dCustomKinematicController* footEffectors[2];
		footEffectors[0] = NULL;
		footEffectors[1] = NULL;
		while (stackIndex) {
			stackIndex--;

			DemoEntity* const entity = childEntities[stackIndex];
			dModelNode* parentBone = parentBones[stackIndex];

			const char* const name = entity->GetName().GetStr();
			dTrace(("name: %s\n", name));
			for (int i = 1; descriptor.m_skeletonDefinition[i].m_boneName; i++) {
				if (!strcmp(descriptor.m_skeletonDefinition[i].m_boneName, name)) {
					NewtonBody* const parentBody = parentBone->GetBody();
					if (descriptor.m_skeletonDefinition[i].m_type == dBalancingRagdollBoneDefinition::m_effector) {
						dModelKeyFrame effectorPose;
						dMatrix effectorMatrix(entity->CalculateGlobalMatrix());
						effectorPose.m_effector = ConnectEffector(parentBone, effectorMatrix, descriptor.m_mass);

						effectorPose.SetMatrix (effectorPose.m_effector->GetTargetMatrix());
						model->m_pose.Append(effectorPose);

						footEffectors[footEffectorsCoint] = effectorPose.m_effector;
						footEffectorsCoint ++;

					} else {
						NewtonBody* const childBody = CreateBodyPart(entity, descriptor.m_skeletonDefinition[i]);
						ConnectLimb(childBody, parentBody, descriptor.m_skeletonDefinition[i]);

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
		NormalizeMassAndInertia(model, descriptor.m_mass);

		// make internal part non collidable
		model->SetAllPartsNonCollidable();
#if 0
		dCustomHinge* fixToWorld = new dCustomHinge (matrix0 * location, model->GetBody(), NULL);
		fixToWorld->EnableLimits(true);
		fixToWorld->SetLimits(0.0f, 0.0f);
#endif

		// setup the pose generator 
		model->SetAnimTree(footEffectors[0], footEffectors[1]);


		//m_currentController = robot;
	}

	virtual void OnDebug(dModelRootNode* const root, dCustomJoint::dDebugDisplay* const debugContext)
	{
		dBalancingRagdoll* const model = (dBalancingRagdoll*)root;
		model->OnDebug(debugContext);
	}

	virtual void OnUpdateTransform(const dModelNode* const bone, const dMatrix& localMatrix) const
	{
		NewtonBody* const body = bone->GetBody();
		DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(body);
		DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));
		
		dQuaternion rot(localMatrix);
		ent->SetMatrix(*scene, rot, localMatrix.m_posit);
	}

	virtual void OnPreUpdate(dModelRootNode* const root, dFloat timestep) const 
	{
		dBalancingRagdoll* const model = (dBalancingRagdoll*)root;
		model->ApplyControls (timestep);
	}
	
	//dBalancingRagdoll* m_currentController;
};



void BalancingCharacter(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	CreateLevelMesh(scene, "flatPlane.ngd", true);

	dBalancingRagdollManager* const manager = new dBalancingRagdollManager(scene);
	NewtonWorld* const world = scene->GetNewton();
	int defaultMaterialID = NewtonMaterialGetDefaultGroupID(world);
	NewtonMaterialSetDefaultFriction(world, defaultMaterialID, defaultMaterialID, 1.0f, 1.0f);
	NewtonMaterialSetDefaultElasticity(world, defaultMaterialID, defaultMaterialID, 0.0f);

	dMatrix origin (dYawMatrix(-90.0f * dDegreeToRad));
	origin.m_posit.m_y += 1.0f;
	manager->CreateKinematicModel(tred, origin);

	origin.m_posit.m_x = -8.0f;
	origin.m_posit.m_y = 1.0f;
	origin.m_posit.m_z = 0.0f;
	scene->SetCameraMatrix(dGetIdentityMatrix(), origin.m_posit);
}
