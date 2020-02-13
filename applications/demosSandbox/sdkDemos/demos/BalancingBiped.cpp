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

//#define D_BIPED_TEXTURE_NAME "marble.tga"
#define D_BIPED_MASS			80.0f
#define D_BIPED_TEXTURE_NAME	"metal_30.tga"

// ********************************
// classes declaration
// ********************************

class dBalancingBiped;
class dBalacingCharacterEffector: public dCustomKinematicController
{
	public:
	dBalacingCharacterEffector(dModelNode* const footAnkleBone, NewtonBody* const referenceBody, const dMatrix& attachmentMatrixInGlobalSpace, dFloat modelMass);
	void SetMatrix(dFloat x, dFloat y, dFloat z, dFloat pitch);
	bool HasGroundContact() const;
	dVector GetFootComInGlobalSpace () const;

	NewtonBody* GetFootBody() const { return m_footBone->GetBody();}

	dMatrix m_origin;
	dFloat m_pitch;
	dFloat m_yaw;
	dFloat m_roll;
	dModelNode* const m_footBone;
};

class dEffectorController
{
	public:
	dEffectorController(dBalancingBiped* const biped): m_biped(biped){}
	virtual ~dEffectorController(){}
	virtual bool Update(dFloat timestep) = 0;
	virtual void Debug(dCustomJoint::dDebugDisplay* const debugContext) {}

	protected:
	dBalancingBiped* m_biped;
};

class dBehaviorState
{
	public:
	dBehaviorState(dBalancingBiped* const biped): m_biped(biped){}
	virtual ~dBehaviorState(){}

	virtual void Enter(dFloat timestep) {};
	virtual void Exit(dFloat timestep) {};
	virtual bool Update(dFloat timestep) = 0;
	virtual void Debug(dCustomJoint::dDebugDisplay* const debugContext) { dAssert(0);}

	protected:
	dBalancingBiped* m_biped;
};

class dBalanceController: public dEffectorController
{
	public:
	dBalanceController(dBalancingBiped* const biped):dEffectorController(biped){}
	virtual ~dBalanceController(){}
	virtual void Debug(dCustomJoint::dDebugDisplay* const debugContext);
	virtual bool Update(dFloat timestep);

	private:
	void SetEffectorPosition(dBalacingCharacterEffector* const effector, const dVector& step) const;
};

class dIdleState: public dBehaviorState
{
	public:
	dIdleState(dBalancingBiped* const biped)
		:dBehaviorState(biped)
		,m_balanceController(biped)
	{}
	virtual ~dIdleState() {}

	virtual bool Update(dFloat timestep)
	{
		return m_balanceController.Update(timestep);
	}

	virtual void Debug(dCustomJoint::dDebugDisplay* const debugContext)
	{
		m_balanceController.Debug(debugContext);
	}

	dBalanceController m_balanceController;
};

class dBalanceStateMachine
{
	public:
	dBalanceStateMachine()
		:m_currentState(NULL)
		,m_states()
		,m_statesCount(0)
	{
	}

	~dBalanceStateMachine()
	{
		for (int i = 0; i < m_statesCount; i++)
		{
			delete m_states[i];
		}
	}

	void Debug(dCustomJoint::dDebugDisplay* const debugContext)
	{
		m_currentState->Debug(debugContext);
	}

	void SetCurrent(dBehaviorState* const state)
	{
		m_currentState = state;
	}

	void AddState(dBehaviorState* const state)
	{
		m_states[m_statesCount] = state;
		m_statesCount++;
	}

	void Update(dFloat timestep) 
	{
		if (m_currentState) {
			m_currentState->Update(timestep);
		}
	}

	dBehaviorState* m_currentState;
	dArray<dBehaviorState*> m_states;
	int m_statesCount;
};

class dBalancingBiped: public dModelRootNode
{
	public:
	dBalancingBiped(NewtonWorld* const world, const dMatrix& coordinateSystem, const dMatrix& location);
	void Debug(dCustomJoint::dDebugDisplay* const debugContext);
	void Update(dFloat timestep);
	private:
	int GetContactPoints(NewtonBody* const body, dVector* const points) const;
	int CalculateSupportPolygon(dVector& center, dVector* const supporPolygonOut, int maxPoints) const;
	void CaculateComAndVelocity();
	void NormalizeWeight(dFloat mass);
	void CaculateComAndVelocity(const dModelNode* const node, void* const context);
	void NormalizeMassCallback(const dModelNode* const node, void* const context) const;
	void ApplyNormalizeMassCallback(const dModelNode* const node, void* const context) const;
	void MakeHip(NewtonWorld* const world, const dMatrix& location);
	void AddUpperBody(NewtonWorld* const world);
	void CreateDescreteContactFootCollision(NewtonBody* const footBody) const;
	dBalacingCharacterEffector* AddLeg(NewtonWorld* const world, dFloat dist);

	dMatrix m_comFrame;
	dMatrix m_localComFrame;
	dVector m_localComVeloc;
	dVector m_localGravityDir;
	dBalacingCharacterEffector* m_leftFoot;
	dBalacingCharacterEffector* m_rightFoot;

	dBalanceStateMachine m_stateMachine;
	friend class dBalanceController;
};



// ********************************
// implementation
// ********************************

dBalacingCharacterEffector::dBalacingCharacterEffector(dModelNode* const footAnkleBone, NewtonBody* const referenceBody, const dMatrix& attachmentMatrixInGlobalSpace, dFloat modelMass)
	:dCustomKinematicController(footAnkleBone->GetParent()->GetBody(), attachmentMatrixInGlobalSpace, referenceBody)
	,m_origin(GetTargetMatrix())
	,m_footBone(footAnkleBone)
{
	// set the joint as exact solver
	SetSolverModel(1);
	SetControlMode(dCustomKinematicController::m_linearAndTwist);
	//SetControlMode(dCustomKinematicController::m_linear);
	//SetControlMode(dCustomKinematicController::m_full6dof);

	dFloat gravity = 10.0f;
	SetMaxAngularFriction(modelMass * 100.0f * gravity);
	SetMaxLinearFriction(modelMass * gravity * 1.2f);

	dVector euler0;
	dVector euler1;
	m_origin.GetEulerAngles(euler0, euler1);
	m_pitch = euler0.m_x;
	m_yaw = euler0.m_y;
	m_roll = euler0.m_z;
}

void dBalacingCharacterEffector::SetMatrix(dFloat x, dFloat y, dFloat z, dFloat pitch)
{
	dMatrix matrix(dPitchMatrix(m_pitch + pitch) * dYawMatrix(m_yaw) * dRollMatrix(m_roll));
	matrix.m_posit = m_origin.TransformVector(dVector(x, y, z, dFloat(1.0f)));
	SetTargetMatrix(matrix);
}

bool dBalacingCharacterEffector::HasGroundContact() const
{
	NewtonBody* const body = GetBody0();
	for (NewtonJoint* contactJoint = NewtonBodyGetFirstContactJoint(body); contactJoint; contactJoint = NewtonBodyGetNextContactJoint(body, contactJoint)) {
		if (NewtonJointIsActive(contactJoint)) {
			return NewtonContactJointGetFirstContact(contactJoint) ? true : false;
		}
	}
	return false;
}

dVector dBalacingCharacterEffector::GetFootComInGlobalSpace() const
{
	dMatrix matrix;
	dVector com;

	NewtonBody* const body = GetBody0();
	NewtonBodyGetMatrix(body, &matrix[0][0]);
	NewtonBodyGetCentreOfMass(body, &com[0]);
	return matrix.TransformVector(com);
}


dBalancingBiped::dBalancingBiped(NewtonWorld* const world, const dMatrix& coordinateSystem, const dMatrix& location)
	:dModelRootNode(NULL, dGetIdentityMatrix())
	,m_comFrame(coordinateSystem)
	,m_localComFrame(coordinateSystem)
	,m_localComVeloc(0.0f)
	,m_localGravityDir(0.0f)
	,m_leftFoot(NULL)
	,m_rightFoot(NULL)
	,m_stateMachine()
{
	MakeHip(world, location);
	AddUpperBody(world);
	m_leftFoot = AddLeg(world, 0.15f);
	m_rightFoot = AddLeg(world, -0.15f);

	// normalize weight to 90 kilogram (about a normal human)
	NormalizeWeight(D_BIPED_MASS);

	dIdleState* const idle = new dIdleState(this);
	m_stateMachine.AddState(idle);

	m_stateMachine.SetCurrent(idle);

	//NewtonBodySetMassMatrix(GetBody(), 0.0f, 0.0f, 0.0f, 0.0f);
}

void dBalancingBiped::Debug(dCustomJoint::dDebugDisplay* const debugContext)
{
	// draw biped center of mass frame
	dMatrix matrix;
	NewtonBodyGetMatrix(GetBody(), &matrix[0][0]);
	debugContext->DrawFrame(m_comFrame);

	m_stateMachine.Debug(debugContext);
	//if (m_leftFoot) {
	//	m_leftFoot->Debug(debugContext);
	//}
	//if (m_rightFoot) {
	//	m_rightFoot->Debug(debugContext);
	//}
}

void dBalancingBiped::Update(dFloat timestep)
{
	static int xxx;
	//dTrace (("frame number: %d\n", xxx));
	xxx++;
	// initialize data
	CaculateComAndVelocity();
	m_stateMachine.Update(timestep);
//	m_balanceController.Update(timestep);
}

int dBalancingBiped::GetContactPoints(NewtonBody* const body, dVector* const points) const
{
	dVector point(0.0f);
	dVector normal(0.0f);
	int count = 0;
	for (NewtonJoint* contactJoint = NewtonBodyGetFirstContactJoint(body); contactJoint; contactJoint = NewtonBodyGetNextContactJoint(body, contactJoint)) {
		if (NewtonJointIsActive(contactJoint)) {
			for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact)) {
				NewtonMaterial* const material = NewtonContactGetMaterial(contact);
				NewtonMaterialGetContactPositionAndNormal(material, body, &point.m_x, &normal.m_x);
				points[count] = point;
				count++;
			}
		}
	}
	return count;
}

int dBalancingBiped::CalculateSupportPolygon(dVector& center, dVector* const supporPolygonOut, int maxPoints) const
{
	int count = 0;
	if (m_leftFoot) {
		count += GetContactPoints(m_leftFoot->GetFootBody(), &supporPolygonOut[count]);
	}
	if (m_rightFoot) {
		count += GetContactPoints(m_rightFoot->GetFootBody(), &supporPolygonOut[count]);
	}
	dAssert(count <= maxPoints);

	center = dVector(0.0f);
	if (count) {
		count = Calculate2dConvexHullProjection(count, supporPolygonOut);
		for (int i = 0; i < count; i++) {
			supporPolygonOut[i] = m_comFrame.UntransformVector(supporPolygonOut[i]);
			center += supporPolygonOut[i];
		}
		center = center.Scale(1.0f / count);
	}
	return count;
}

void dBalancingBiped::CaculateComAndVelocity()
{
	dMatrix matrix;
	NewtonBodyGetMatrix(GetBody(), &matrix[0][0]);

	// calculate the local frame of center of mass
	m_comFrame = m_localComFrame * matrix;

	m_localComVeloc = dVector(0.0f);
	m_comFrame.m_posit = dVector(0.0f);
	ForEachNode((dModelNode::Callback)&dBalancingBiped::CaculateComAndVelocity, NULL);

	m_comFrame.m_posit = m_comFrame.m_posit.Scale(1.0f / D_BIPED_MASS);
	m_comFrame.m_posit.m_w = 1.0f;
	m_localComFrame.m_posit = matrix.UntransformVector(m_comFrame.m_posit);
	m_localComVeloc = m_comFrame.UnrotateVector(m_localComVeloc.Scale(1.0f / D_BIPED_MASS));
	m_localGravityDir = m_comFrame.UnrotateVector(dVector(0.0f, 1.0f, 0.0f, 0.0f));
}

void dBalancingBiped::NormalizeWeight(dFloat mass)
{
	dFloat totalMass = 0.0f;
	ForEachNode((dModelNode::Callback)&dBalancingBiped::NormalizeMassCallback, &totalMass);

	dFloat normalizeMassScale = mass / totalMass;
	ForEachNode((dModelNode::Callback)&dBalancingBiped::ApplyNormalizeMassCallback, &normalizeMassScale);
}

void dBalancingBiped::CaculateComAndVelocity(const dModelNode* const node, void* const context)
{
	dMatrix matrix;
	dVector com(0.0f);
	dVector veloc(0.0f);
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	NewtonBody* const body = node->GetBody();

	NewtonBodyGetVelocity(body, &veloc[0]);
	NewtonBodyGetCentreOfMass(body, &com[0]);
	NewtonBodyGetMatrix(body, &matrix[0][0]);
	NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);

	com.m_w = 1.0f;
	m_localComVeloc += veloc.Scale(mass);
	m_comFrame.m_posit += matrix.TransformVector(com).Scale(mass);
}

void dBalancingBiped::NormalizeMassCallback(const dModelNode* const node, void* const context) const
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	dFloat* const totalMass = (dFloat*)context;
	NewtonBodyGetMass(node->GetBody(), &mass, &Ixx, &Iyy, &Izz);
	*totalMass += mass;
}

void dBalancingBiped::ApplyNormalizeMassCallback(const dModelNode* const node, void* const context) const
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;
	dFloat scale = *((dFloat*)context);
	NewtonBodyGetMass(node->GetBody(), &mass, &Ixx, &Iyy, &Izz);

	mass *= scale;
	Ixx *= scale;
	Iyy *= scale;
	Izz *= scale;
	NewtonBodySetMassMatrix(node->GetBody(), mass, Ixx, Iyy, Izz);
}

void dBalancingBiped::MakeHip(NewtonWorld* const world, const dMatrix& location)
{
	DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);

	dVector size(0.25f, 0.2f, 0.25f, 0.0f);
	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _CAPSULE_PRIMITIVE, 0);
	DemoMesh* const geometry = new DemoMesh("hip", scene->GetShaderCache(), collision, D_BIPED_TEXTURE_NAME, D_BIPED_TEXTURE_NAME, D_BIPED_TEXTURE_NAME);

	m_body = CreateSimpleSolid(scene, geometry, 100.0f, location, collision, 0);

	dMatrix matrix;
	NewtonBodyGetMatrix(GetBody(), &matrix[0][0]);

	// calculate the local frame of center of mass
	m_localComFrame = m_comFrame * matrix.Inverse();
	m_localComFrame.m_posit = dVector(0.0f);
	m_localComFrame.m_posit.m_w = 1.0f;
	m_comFrame = m_localComFrame * matrix;

	geometry->Release();
	NewtonDestroyCollision(collision);
}

void dBalancingBiped::AddUpperBody(NewtonWorld* const world)
{
	dMatrix matrix;
	NewtonBodyGetMatrix(m_body, &matrix[0][0]);

	DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);
	dVector size(0.3f, 0.4f, 0.2f, 0.0f);
	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);
	DemoMesh* const geometry = new DemoMesh("torso", scene->GetShaderCache(), collision, D_BIPED_TEXTURE_NAME, D_BIPED_TEXTURE_NAME, D_BIPED_TEXTURE_NAME);

	dFloat hipRadius = 0.25f * 0.5f;
	dMatrix location(matrix);
	location.m_posit += matrix.m_up.Scale(hipRadius + size.m_y * 0.5f);
	NewtonBody* const torsoBody = CreateSimpleSolid(scene, geometry, 100.0f, location, collision, 0);

	geometry->Release();
	NewtonDestroyCollision(collision);

	dMatrix jointMatrix(matrix);
	jointMatrix.m_posit += matrix.m_up.Scale(hipRadius);
	dCustomHinge* const fixJoint = new dCustomHinge(jointMatrix, torsoBody, GetBody());
	fixJoint->EnableLimits(true);
	fixJoint->SetLimits(0.0f, 0.0f);
	new dModelNode(torsoBody, dGetIdentityMatrix(), this);
}

void dBalancingBiped::CreateDescreteContactFootCollision(NewtonBody* const footBody) const
{
	NewtonWorld* const world = NewtonBodyGetWorld(footBody);
	NewtonCollision* const sphereCollision = NewtonCreateSphere(world, 0.05f, 0, NULL);

	dMatrix subShapeLocation(dGetIdentityMatrix());
	NewtonCollision* const threePointCollision = NewtonCreateCompoundCollision(world, 0);
	NewtonCompoundCollisionBeginAddRemove(threePointCollision);

	dFloat height = 0.012f;
	dFloat widthSeparation = 0.05f;
	dFloat frontSeparation = 0.11f;

#if 1
	// tree points contacts
	subShapeLocation.m_posit.m_z = frontSeparation;
	subShapeLocation.m_posit.m_x = height;
	subShapeLocation.m_posit.m_y = 0.0f;
	NewtonCollisionSetMatrix(sphereCollision, &subShapeLocation[0][0]);
	NewtonCompoundCollisionAddSubCollision(threePointCollision, sphereCollision);

#else
	// four points contacts
	subShapeLocation.m_posit.m_z = frontSeparation;
	subShapeLocation.m_posit.m_x = height;
	subShapeLocation.m_posit.m_y = widthSeparation;
	NewtonCollisionSetMatrix(sphereCollision, &subShapeLocation[0][0]);
	NewtonCompoundCollisionAddSubCollision(threePointCollision, sphereCollision);

	subShapeLocation.m_posit.m_z = frontSeparation;
	subShapeLocation.m_posit.m_x = height;
	subShapeLocation.m_posit.m_y = -widthSeparation;
	NewtonCollisionSetMatrix(sphereCollision, &subShapeLocation[0][0]);
	NewtonCompoundCollisionAddSubCollision(threePointCollision, sphereCollision);
#endif

	subShapeLocation.m_posit.m_z = -frontSeparation;
	subShapeLocation.m_posit.m_x = height;
	subShapeLocation.m_posit.m_y = widthSeparation;
	NewtonCollisionSetMatrix(sphereCollision, &subShapeLocation[0][0]);
	NewtonCompoundCollisionAddSubCollision(threePointCollision, sphereCollision);

	subShapeLocation.m_posit.m_z = -frontSeparation;
	subShapeLocation.m_posit.m_x = height;
	subShapeLocation.m_posit.m_y = -widthSeparation;
	NewtonCollisionSetMatrix(sphereCollision, &subShapeLocation[0][0]);
	NewtonCompoundCollisionAddSubCollision(threePointCollision, sphereCollision);

	NewtonCompoundCollisionEndAddRemove(threePointCollision);

	NewtonBodySetCollision(footBody, threePointCollision);

	NewtonDestroyCollision(sphereCollision);
	NewtonDestroyCollision(threePointCollision);
}

dBalacingCharacterEffector* dBalancingBiped::AddLeg(NewtonWorld* const world, dFloat dist)
{
	dMatrix matrix;
	NewtonBodyGetMatrix(m_body, &matrix[0][0]);

	// create capsule collision and mesh
	DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(world);
	dVector size(0.15f, 0.35f, 0.15f, 0.0f);
	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _CAPSULE_PRIMITIVE, 0);
	DemoMesh* const geometry = new DemoMesh("leg", scene->GetShaderCache(), collision, D_BIPED_TEXTURE_NAME, D_BIPED_TEXTURE_NAME, D_BIPED_TEXTURE_NAME);

	// create upper leg body and visual entity
	dMatrix location(matrix);
	location.m_posit += matrix.m_front.Scale(dist);
	location.m_posit -= matrix.m_up.Scale(size.m_y - size.m_x * 0.5f);
	location = dRollMatrix(90.0f * dDegreeToRad) * location;

	// flex the leg 10 degree to the front.
	dMatrix tiltLegMatrix(dPitchMatrix(-10.0f * dDegreeToRad));
	// get hip pivot point
	dVector jointPivot(location.m_posit + location.m_front.Scale(size.m_y * 0.5f + 0.2f * 0.5f));
	tiltLegMatrix.m_posit = jointPivot - tiltLegMatrix.RotateVector(jointPivot);
	location = location * tiltLegMatrix;
	NewtonBody* const legBody = CreateSimpleSolid(scene, geometry, 20.0f, location, collision, 0);

	// create leg-hip joint
	dMatrix jointMatrix(location);
	jointMatrix.m_posit = jointPivot;
	new dCustomBallAndSocket(jointMatrix, legBody, GetBody());
	dModelNode* const legBone = new dModelNode(legBody, dGetIdentityMatrix(), this);

	// create shin body and visual entity
	location.m_posit -= location.m_front.Scale(size.m_y + size.m_x * 0.5f);

	// flex shin 20 degrees backward
	dMatrix tiltShinMatrix(dPitchMatrix(20.0f * dDegreeToRad));
	// get the knee pivot
	dVector kneePivot(location.m_posit + location.m_front.Scale(size.m_y * 0.5f + size.m_x * 0.25f));
	tiltShinMatrix.m_posit = kneePivot - tiltShinMatrix.RotateVector(kneePivot);
	location = location * tiltShinMatrix;
	NewtonBody* const shinBody = CreateSimpleSolid(scene, geometry, 15.0f, location, collision, 0);

	// get the knee pivot matrix
	jointMatrix = location;
	jointMatrix = dRollMatrix(90.0f * dDegreeToRad) * jointMatrix;
	jointMatrix.m_posit = kneePivot;
	// create knee joint
	dCustomHinge* const kneeHinge = new dCustomHinge(jointMatrix, shinBody, legBody);
	kneeHinge->EnableLimits(true);
	kneeHinge->SetLimits(-120.0f * dDegreeToRad, 10.0f * dDegreeToRad);
	dModelNode* const shinBone = new dModelNode(shinBody, dGetIdentityMatrix(), legBone);

	// release collision and visual mesh
	geometry->Release();
	NewtonDestroyCollision(collision);

	// create a box to represent the foot
	dVector footSize(0.07f, 0.15f, 0.25f, 0.0f);
	NewtonCollision* const footCollision = CreateConvexCollision(world, dGetIdentityMatrix(), footSize, _BOX_PRIMITIVE, 0);
	DemoMesh* const footGeometry = new DemoMesh("foot", scene->GetShaderCache(), collision, D_BIPED_TEXTURE_NAME, D_BIPED_TEXTURE_NAME, D_BIPED_TEXTURE_NAME);

	// create foot body and visual entity
	location.m_posit -= location.m_front.Scale(size.m_y * 0.5f + size.m_x * 0.5f);
	location.m_posit += location.m_right.Scale(footSize.m_z * 0.25f);

	// get he ankle pivot
	dVector anklePivot(location.m_posit - location.m_right.Scale(footSize.m_z * 0.25f) + location.m_front.Scale(footSize.m_x * 0.5f));
	// flex the foot 10 degrees forward
	dMatrix tiltAnkleMatrix(dPitchMatrix(-10.0f * dDegreeToRad));
	tiltAnkleMatrix.m_posit = anklePivot - tiltAnkleMatrix.RotateVector(anklePivot);
	location = location * tiltAnkleMatrix;
	NewtonBody* const footBody = CreateSimpleSolid(scene, footGeometry, 10.0f, location, footCollision, 0);
	CreateDescreteContactFootCollision(footBody);

	jointMatrix = location;
	jointMatrix.m_posit = anklePivot;
	dMatrix effectorMatrix(jointMatrix);

	jointMatrix = dRollMatrix(90.0f * dDegreeToRad) * jointMatrix;
	jointMatrix = dPitchMatrix(90.0f * dDegreeToRad) * jointMatrix;
	dCustomDoubleHinge* const heelJoint = new dCustomDoubleHinge(jointMatrix, footBody, shinBody);
	heelJoint->EnableLimits(true);
	heelJoint->EnableLimits1(true);

dFloat xxx = 100.0f;
	heelJoint->SetFriction(xxx);
	heelJoint->SetFriction1(xxx);

	dModelNode* const footAnkleBone = new dModelNode(footBody, dGetIdentityMatrix(), shinBone);

	// release collision and visual mesh
	footGeometry->Release();
	NewtonDestroyCollision(footCollision);

	// save ankle matrix as the effector pivot 
	return new dBalacingCharacterEffector(footAnkleBone, m_body, effectorMatrix, D_BIPED_MASS);
}


void dBalanceController::Debug(dCustomJoint::dDebugDisplay* const debugContext)
{
	dVector supportPolygon[32];
	dVector supportPolygonCenter;

	// calculate support polygon in the local space of the biped.
	int count = m_biped->CalculateSupportPolygon(supportPolygonCenter, supportPolygon, sizeof (supportPolygon) / sizeof (supportPolygon[0]));

	// draw support polygon
	if (count) {
		int i0 = count - 1;
		debugContext->SetColor(dVector(1.0f, 1.0f, 0.0f, 0.0f));
		for (int i = 0; i < count; i++) {
			dVector p0(m_biped->m_comFrame.TransformVector(supportPolygon[i0]));
			dVector p1(m_biped->m_comFrame.TransformVector(supportPolygon[i]));
			p0.m_y += 0.02f;
			p1.m_y += 0.02f;
			debugContext->DrawLine(p0, p1);
			i0 = i;
		}

		dFloat heigh = m_biped->m_localGravityDir.DotProduct3(supportPolygonCenter);
		supportPolygonCenter = m_biped->m_comFrame.TransformVector(supportPolygonCenter);
		supportPolygonCenter.m_y += 0.02f;
		debugContext->DrawPoint(supportPolygonCenter, 2.0f);

		dVector centerOfGravity(m_biped->m_comFrame.TransformVector(dVector(0.0f, heigh, 0.0f, 1.0f)));
		centerOfGravity.m_y += 0.02f;
		debugContext->SetColor(dVector(0.0f, 1.0f, 0.0f, 0.0f));
		debugContext->DrawPoint(centerOfGravity, 2.0f);
	}

	if (m_biped->m_leftFoot) {
		//dMatrix matrix;
		//NewtonBodyGetMatrix(m_biped->m_leftFoot->GetBody0(), &matrix[0][0]);
		//debugContext->DrawFrame(matrix);
		m_biped->m_leftFoot->Debug(debugContext);
	}
	if (m_biped->m_rightFoot) {
		//dMatrix matrix;
		//NewtonBodyGetMatrix(m_biped->m_rightFoot->GetBody0(), &matrix[0][0]);
		//debugContext->DrawFrame(matrix);
		m_biped->m_rightFoot->Debug(debugContext);
	}
}

bool dBalanceController::Update(dFloat timestep)
{
	// calculate support polygon in the local space of the biped.
	dVector supportPolygon[32];
	dVector supportPolygonCenter;
	int count = m_biped->CalculateSupportPolygon(supportPolygonCenter, supportPolygon, sizeof (supportPolygon) / sizeof (supportPolygon[0]));
	if (!count) {
		return false;
	}

	dFloat heigh = m_biped->m_localGravityDir.DotProduct3(supportPolygonCenter);
	dVector step(supportPolygonCenter - m_biped->m_localGravityDir.Scale(heigh));
	dFloat dist2 = step.DotProduct3(step);
	const dFloat maxStep = 1.0e-3f;
	const dFloat maxStep2 = maxStep * maxStep;
	if (dist2 > maxStep2) {
		step = step.Normalize().Scale(maxStep);
	}
	step = step.Scale(0.3f);

	dFloat err2 = step.DotProduct3(step);
	if (err2 > maxStep2* 0.01f) {
		if (m_biped->m_leftFoot) {
			SetEffectorPosition(m_biped->m_leftFoot, step);
		}
		if (m_biped->m_rightFoot) {
			SetEffectorPosition(m_biped->m_rightFoot, step);
		}
	}
	return true;
}

void dBalanceController::SetEffectorPosition(dBalacingCharacterEffector* const effector, const dVector& step) const
{
	dMatrix bodyMatrix;
	dMatrix effectorMatrix(effector->GetTargetMatrix());
	NewtonBodyGetMatrix(effector->GetBody1(), &bodyMatrix[0][0]);
	dVector localPosition(m_biped->m_comFrame.UntransformVector(bodyMatrix.TransformVector(effectorMatrix.m_posit)));
	localPosition -= step;
	dVector position(bodyMatrix.UntransformVector(m_biped->m_comFrame.TransformVector(localPosition)));
	effectorMatrix.m_posit = position;
	effector->SetTargetMatrix(effectorMatrix);
}

class dBalancingBipedManager: public dModelManager
{
	public:
	dBalancingBipedManager(DemoEntityManager* const scene)
		:dModelManager(scene->GetNewton())
	{
		//scene->SetUpdateCameraFunction(UpdateCameraCallback, this);
		//scene->Set2DDisplayRenderFunction(RenderPlayerHelp, NULL, this);

		// create a material for early collision culling
		NewtonWorld* const world = scene->GetNewton();
		int material = NewtonMaterialGetDefaultGroupID(world);

		NewtonMaterialSetCallbackUserData(world, material, material, this);
		NewtonMaterialSetDefaultElasticity(world, material, material, 0.0f);
		NewtonMaterialSetDefaultFriction(world, material, material, 0.9f, 0.9f);
		NewtonMaterialSetCollisionCallback(world, material, material, NULL, HandleSoftContacts);
	}

	dBalancingBiped* CreateBiped (const dMatrix& location)
	{
		dMatrix coordinateSystem (dYawMatrix(-90.0f * dDegreeToRad) * location);
		dBalancingBiped* const biped = new dBalancingBiped(GetWorld(), coordinateSystem, location);
		AddRoot (biped);
		return biped;
	}

	virtual void OnUpdateTransform(const dModelNode* const bone, const dMatrix& globalMatrix) const
	{
		// this function is no called because the the model is hierarchical 
		// but all body parts are in global space, therefore the normal rigid body transform callback
		// set the transform after each update.
	}

	virtual void OnDebug(dModelRootNode* const model, dCustomJoint::dDebugDisplay* const debugContext)
	{
		dBalancingBiped* const biped = (dBalancingBiped*)model;
		biped->Debug (debugContext);
	}

	virtual void OnPreUpdate(dModelRootNode* const model, dFloat timestep, int threadID) const
	{
		dBalancingBiped* const biped = (dBalancingBiped*)model;
		biped->Update(timestep);
	}

	static void HandleSoftContacts(const NewtonJoint* const contactJoint, dFloat timestep, int threadIndex)
	{
/*
		// iterate over all contact point checking is a sphere shape belong to the Biped, 
		// if so declare this a soft contact 
		dAssert (NewtonJointIsActive(contactJoint));
		NewtonBody* const body0 = NewtonJointGetBody0(contactJoint);
		NewtonBody* const body1 = NewtonJointGetBody1(contactJoint);
		for (void* contact = NewtonContactJointGetFirstContact(contactJoint); contact; contact = NewtonContactJointGetNextContact(contactJoint, contact)) {
			NewtonMaterial* const material = NewtonContactGetMaterial(contact);

			NewtonCollision* const shape0 = NewtonMaterialGetBodyCollidingShape (material, body0);
			NewtonCollision* const shape1 = NewtonMaterialGetBodyCollidingShape (material, body1);
			// this check is too simplistic but will do for this demo. 
			// a better set up should use the Collision Material to stores application info
			if ((NewtonCollisionGetType(shape0) == SERIALIZE_ID_SPHERE) || (NewtonCollisionGetType(shape1) == SERIALIZE_ID_SPHERE)) {
				NewtonMaterialSetAsSoftContact(material, 0.03f);
			}
		}
*/
	}
};


void BalancingBiped(DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();

	CreateLevelMesh(scene, "flatPlane.ngd", true);

	dBalancingBipedManager* const manager = new dBalancingBipedManager(scene);

	dMatrix origin (dYawMatrix(0.0f * dDegreeToRad));
	origin.m_posit.m_y += 1.05f;
	manager->CreateBiped(origin);

	origin.m_posit.m_x = -2.5f;
	origin.m_posit.m_y = 1.5f;
	origin.m_posit.m_z = 2.0f;
	origin = dYawMatrix(45.0f * dDegreeToRad) * origin;
	dQuaternion rot (origin);
	scene->SetCameraMatrix(rot, origin.m_posit);
}
