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
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DemoMesh.h"
#include "OpenGlUtil.h"


static NewtonBody* CreateBox (DemoEntityManager* const scene, const dVector& location, const dVector& size)
{
    NewtonWorld* const world = scene->GetNewton();
    int materialID =  NewtonMaterialGetDefaultGroupID (world);
    NewtonCollision* const collision = CreateConvexCollision (world, dGetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);
   	DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");

    dFloat mass = 1.0f;
    dMatrix matrix (dGetIdentityMatrix());
    matrix.m_posit = location;
    matrix.m_posit.m_w = 1.0f;
    NewtonBody* const body = CreateSimpleSolid (scene, geometry, mass, matrix, collision, materialID);

    geometry->Release();
    NewtonDestroyCollision(collision);
    return body;
}

static NewtonBody* CreateSphere(DemoEntityManager* const scene, const dVector& location, const dVector& size)
{
	NewtonWorld* const world = scene->GetNewton();
	int materialID = NewtonMaterialGetDefaultGroupID(world);
	dMatrix matrix(dGetIdentityMatrix());
	NewtonCollision* const collision = CreateConvexCollision(world, &matrix[0][0], size, _SPHERE_PRIMITIVE, 0);
	DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");

	dFloat mass = 1.0f;
	matrix.m_posit = location;
	matrix.m_posit.m_w = 1.0f;
	NewtonBody* const body = CreateSimpleSolid(scene, geometry, mass, matrix, collision, materialID);

	geometry->Release();
	NewtonDestroyCollision(collision);
	return body;
}


static NewtonBody* CreateCapule (DemoEntityManager* const scene, const dVector& location, const dVector& size)
{
	NewtonWorld* const world = scene->GetNewton();
	int materialID =  NewtonMaterialGetDefaultGroupID (world);
	dMatrix uprightAligment (dRollMatrix(90.0f * dDegreeToRad));
	NewtonCollision* const collision = CreateConvexCollision (world, &uprightAligment[0][0], size, _CAPSULE_PRIMITIVE, 0);
	DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");

	dFloat mass = 1.0f;
	dMatrix matrix (dGetIdentityMatrix());
	matrix.m_posit = location;
	matrix.m_posit.m_w = 1.0f;
	NewtonBody* const body = CreateSimpleSolid (scene, geometry, mass, matrix, collision, materialID);

	geometry->Release();
	NewtonDestroyCollision(collision);
	return body;
}

static NewtonBody* CreateWheel (DemoEntityManager* const scene, const dVector& location, dFloat radius, dFloat height)
{
    NewtonWorld* const world = scene->GetNewton();
    int materialID =  NewtonMaterialGetDefaultGroupID (world);
    dVector size (radius, height, radius, 0.0f);
    NewtonCollision* const collision = CreateConvexCollision (world, dGetIdentityMatrix(), size, _CHAMFER_CYLINDER_PRIMITIVE, 0);
    DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");

    dFloat mass = 1.0f;
    dMatrix matrix (dGetIdentityMatrix());
    matrix.m_posit = location;
    matrix.m_posit.m_w = 1.0f;
    NewtonBody* const body = CreateSimpleSolid (scene, geometry, mass, matrix, collision, materialID);

    geometry->Release();
    NewtonDestroyCollision(collision);
    return body;
}

static NewtonBody* CreateCylinder (DemoEntityManager* const scene, const dVector& location, dFloat radius, dFloat height)
{
    NewtonWorld* const world = scene->GetNewton();
    int materialID =  NewtonMaterialGetDefaultGroupID (world);
    dVector size (radius, height, radius, 0.0f);
    NewtonCollision* const collision = CreateConvexCollision (world, dGetIdentityMatrix(), size, _CYLINDER_PRIMITIVE, 0);
    DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");

    dFloat mass = 1.0f;
    dMatrix matrix (dGetIdentityMatrix());
    matrix.m_posit = location;
    matrix.m_posit.m_w = 1.0f;
    NewtonBody* const body = CreateSimpleSolid (scene, geometry, mass, matrix, collision, materialID);

    geometry->Release();
    NewtonDestroyCollision(collision);
    return body;
}

static void AddDistance (DemoEntityManager* const scene, const dVector& origin)
{
	dVector size (1.0f, 1.0f, 1.0f);
	NewtonBody* const box0 = CreateBox(scene, origin + dVector (0.0f, 6.0f + size.m_y + 0.25f, 0.0f, 0.0f), size.Scale (0.2f));
	NewtonBody* const box1 = CreateCapule (scene, origin + dVector (0.0f, 6.0f, 0.0f, 0.0f), size);
	NewtonBody* const box2 = CreateCapule (scene, origin + dVector (0.0f, 6.0f - size.m_y * 4.0f, 0.0f, 0.0f), size);

	dMatrix pinMatrix (dGrammSchmidt (dVector (0.0f, -1.0f, 0.0f, 0.0f)));
	NewtonBodySetMassMatrix(box0, 0.0f, 0.0f, 0.0f, 0.0f);

	// connect first box to the world
	dMatrix matrix0;
	NewtonBodyGetMatrix (box1, &matrix0[0][0]);
	pinMatrix.m_posit = matrix0.m_posit + dVector (0.0f, size.m_y, 0.0f, 0.0f);
	new dCustomBallAndSocket (pinMatrix, box1, box0);

	// link the two boxes with a distance joint
	dMatrix matrix1;
	NewtonBodyGetMatrix (box2, &matrix1[0][0]);

	// get the origins
	dVector pivot0 (matrix0.m_posit - dVector (0.0f, size.m_y, 0.0f, 0.0f));
	dVector pivot1 (matrix1.m_posit + dVector (0.0f, size.m_y, 0.0f, 0.0f));

	// connect bodies at a corner
	new dCustomFixDistance (pivot1, pivot0, box2, box1);
}


static void FunnyDistanceJointNullForce(const NewtonBody* body, dFloat timestep, int threadIndex)
{
	dVector force(dVector(0.0f, 0.0f, 0.0f));
	NewtonBodySetForce(body, &force.m_x);
}

static void AddFixDistance(DemoEntityManager* const scene, const dVector& origin)
{
	dVector size(1.0f, 1.0f, 1.0f);
	NewtonBody* const box0 = CreateSphere(scene, origin + dVector(0.0f, 6.0f - size.m_y * 0.0f, 0.0f, 0.0f), size);
	NewtonBody* const box1 = CreateSphere(scene, origin + dVector(0.0f, 6.0f - size.m_y * 1.0f, 0.0f, 0.0f), size);
	NewtonBody* const box2 = CreateSphere(scene, origin + dVector(0.0f, 6.0f - size.m_y * 2.0f, 0.0f, 0.0f), size);
	NewtonBody* const box3 = CreateSphere(scene, origin + dVector(0.0f, 6.0f - size.m_y * 3.0f, 0.0f, 0.0f), size);

	NewtonBodySetForceAndTorqueCallback(box0, FunnyDistanceJointNullForce);
	NewtonBodySetForceAndTorqueCallback(box1, FunnyDistanceJointNullForce);
	NewtonBodySetForceAndTorqueCallback(box2, FunnyDistanceJointNullForce);
	NewtonBodySetForceAndTorqueCallback(box3, FunnyDistanceJointNullForce);

	dMatrix matrix0;
	dMatrix matrix1;
	NewtonBodyGetMatrix(box0, &matrix0[0][0]);
	NewtonBodyGetMatrix(box1, &matrix1[0][0]);
	new dCustomFixDistance(matrix1.m_posit, matrix0.m_posit, box1, box0);

	dMatrix matrix2;
	NewtonBodyGetMatrix(box1, &matrix1[0][0]);
	NewtonBodyGetMatrix(box2, &matrix2[0][0]);
	new dCustomFixDistance(matrix2.m_posit, matrix1.m_posit, box2, box1);

	dMatrix matrix3;
	NewtonBodyGetMatrix(box2, &matrix2[0][0]);
	NewtonBodyGetMatrix(box3, &matrix3[0][0]);
	new dCustomFixDistance(matrix3.m_posit, matrix2.m_posit, box3, box2);
}

static void AddLimitedBallAndSocket (DemoEntityManager* const scene, const dVector& origin)
{
	dVector size(1.0f, 1.0f, 1.0f);
	NewtonBody* const base = CreateBox(scene, origin + dVector (0.0f,  5.0f + size.m_y + 0.25f, 0.0f, 0.0f), size.Scale (0.2f));
	NewtonBody* const box0 = CreateCapule(scene, origin + dVector(0.0f, 5.0f, 0.0f, 0.0f), size);
	NewtonBody* const box1 = CreateCapule(scene, origin + dVector(0.0f, 5.0f- size.m_y * 2.0f, 0.0f, 0.0f), size);

	NewtonBodySetMassMatrix(base, 0.0f, 0.0f, 0.0f, 0.0f);
	dMatrix pinMatrix(dGrammSchmidt(dVector(0.0f, -1.0f, 0.0f, 0.0f)));

	// connect first box to the world
	dMatrix matrix;
	NewtonBodyGetMatrix(box0, &matrix[0][0]);
	pinMatrix.m_posit = matrix.m_posit + dVector(0.0f, size.m_y, 0.0f, 0.0f);

	// tilt the cone limit
	dMatrix tiltConeMatrix (dYawMatrix(-30.0f * dDegreeToRad) * pinMatrix);
	dCustomBallAndSocket* const joint0 = new dCustomBallAndSocket(pinMatrix, tiltConeMatrix, box0, base);
	joint0->EnableCone(true);
	joint0->EnableTwist(true);
	joint0->SetConeLimits (60.0f * dDegreeToRad);
	joint0->SetTwistLimits (-1000.0f * dDegreeToRad, 1000.0f * dDegreeToRad);

	// connect first box1 to box0 the world
	NewtonBodyGetMatrix(box1, &matrix[0][0]);
	pinMatrix.m_posit = matrix.m_posit + dVector(0.0f, size.m_y, 0.0f, 0.0f);

	dCustomBallAndSocket* const joint1 = new dCustomBallAndSocket(pinMatrix, box1, box0);
	joint1->EnableCone(true);
	joint1->SetConeLimits(30.0f * dDegreeToRad);
	joint1->EnableTwist(true);
	joint1->SetTwistLimits(-30.0f * dDegreeToRad, 30.0f * dDegreeToRad);
}

static void AddBallAndSockectWithFriction (DemoEntityManager* const scene, const dVector& origin)
{
	dVector size (1.0f, 1.0f, 1.0f);
	NewtonBody* const base = CreateBox(scene, origin + dVector (0.0f,  5.0f + size.m_y + 0.25f, 0.0f, 0.0f), size.Scale (0.2f));
	NewtonBody* const box0 = CreateCapule (scene, origin + dVector (0.0f,  5.0f, 0.0f, 0.0f), size);
	NewtonBody* const box1 = CreateCapule (scene, origin + dVector (0.0f,  5.0f- size.m_y * 2.0f, 0.0f, 0.0f), size);

	NewtonBodySetMassMatrix(base, 0.0f, 0.0f, 0.0f, 0.0f);
	dMatrix pinMatrix (dGrammSchmidt (dVector (0.0f, -1.0f, 0.0f, 0.0f)));

	// connect first box to the world
	dMatrix matrix0;
	NewtonBodyGetMatrix (box0, &matrix0[0][0]);
	pinMatrix.m_posit = matrix0.m_posit + dVector (0.0f, size.m_y, 0.0f, 0.0f);
	dMatrix rotateLimit (dYawMatrix (-30.0f * dDegreeToRad) * pinMatrix);
	dCustomBallAndSocket* const joint0 = new dCustomBallAndSocket (pinMatrix, rotateLimit, box0, base);
	joint0->EnableCone(true);
	joint0->SetConeFriction(200.0f);
	joint0->SetConeLimits (120.0f * dDegreeToRad);
	joint0->EnableTwist(true);
	joint0->SetTwistFriction(50.0f);
	joint0->SetTwistLimits(-150.0f * dDegreeToRad, 150.0f * dDegreeToRad);

	// link the two boxes
	dMatrix matrix1;
	NewtonBodyGetMatrix (box1, & matrix1[0][0]);
	pinMatrix.m_posit = (matrix0.m_posit + matrix1.m_posit).Scale (0.5f);
	dCustomBallAndSocket* const joint1 = new dCustomBallAndSocket (pinMatrix, box1, box0);
	joint1->EnableCone(true);
	joint1->SetConeFriction(200.0f);
	joint1->SetConeLimits(120.0f * dDegreeToRad);
	joint1->EnableTwist(true);
	joint1->SetTwistLimits(-90.0f * dDegreeToRad, 90.0f * dDegreeToRad);
}

static void Add6DOF (DemoEntityManager* const scene, const dVector& origin)
{
	dVector size (1.0f, 1.0f, 1.0f);
	NewtonBody* const base = CreateBox(scene, origin + dVector (0.0f,  5.0f + size.m_y + 0.25f, 0.0f, 0.0f), size.Scale (0.2f));
	NewtonBody* const box0 = CreateCapule (scene, origin + dVector (0.0f,  5.0f, 0.0f, 0.0f), size);
	NewtonBody* const box1 = CreateCapule (scene, origin + dVector (0.0f,  5.0f- size.m_y * 2.0f, 0.0f, 0.0f), size);

	const dFloat yawLimit = 120.0f * dDegreeToRad;
	const dFloat rollLimit = 80.0f * dDegreeToRad;
	const dFloat pitchLimit = 80.0f * dDegreeToRad;

	NewtonBodySetMassMatrix(base, 0.0f, 0.0f, 0.0f, 0.0f);
	dMatrix pinMatrix (dGrammSchmidt (dVector (0.0f, -1.0f, 0.0f, 0.0f)));

	// connect first box to the world
	dMatrix matrix0;
	NewtonBodyGetMatrix (box0, & matrix0[0][0]);
	pinMatrix.m_posit = matrix0.m_posit + dVector (0.0f, size.m_y, 0.0f, 0.0f);
	dCustom6dof* const joint0 = new dCustom6dof (pinMatrix, box0, base);
	joint0->SetYawLimits (-yawLimit, yawLimit);
	joint0->SetPitchLimits(-pitchLimit, pitchLimit);
	joint0->SetRollLimits(-rollLimit, rollLimit);
	//joint0->SetPitchLimits(-0, 0);
	//joint0->DisableRotationX ();

	// link the two boxes
	dMatrix matrix1;
	NewtonBodyGetMatrix (box1, &matrix1[0][0]);
	pinMatrix.m_posit = (matrix0.m_posit + matrix1.m_posit).Scale (0.5f);
	dCustom6dof* const joint1 = new dCustom6dof (pinMatrix, box1, box0);
	joint1->SetYawLimits(-yawLimit, yawLimit);
	joint1->SetRollLimits(-rollLimit, rollLimit);
	joint1->SetPitchLimits(-pitchLimit, pitchLimit);
	//joint1->DisableRotationX();

}

static void AddDoubleHinge(DemoEntityManager* const scene, const dVector& origin)
{
	dVector size(1.0f, 1.0f, 1.0f);

//	NewtonBody* const box0 = CreateBox(scene, origin + dVector(0.0f, 4.0f, 0.0f, 0.0f), dVector(0.25f, 0.25f, 4.0f, 0.0f));
//	NewtonBodySetMassMatrix(box0, 0.0f, 0.0f, 0.0f, 0.0f);

	dMatrix matrix;
	dVector damp(0.0f);
	dVector omega(0.0f, 10.0f, 20.0f, 0.0f);
//	dVector omega (0.0f, 10.0f, 100.0f, 0.0f);

	NewtonBody* const box1 = CreateWheel(scene, origin + dVector(0.0f, 4.0f, 2.0f, 0.0f), 1.0f, 0.5f);
	NewtonBodyGetMatrix(box1, &matrix[0][0]);
	matrix = dYawMatrix (dPi * 0.5f) * matrix; 
	NewtonBodySetMatrix(box1, &matrix[0][0]);
	((DemoEntity*) NewtonBodyGetUserData(box1))->ResetMatrix (*scene, matrix);
	NewtonBodySetOmega(box1, &omega[0]);
	NewtonBodySetLinearDamping(box1, 0.0f);
	NewtonBodySetAngularDamping(box1, &damp[0]);

	NewtonBody* const box2 = CreateWheel(scene, origin + dVector(0.0f, 4.0f, -2.0f, 0.0f), 1.0f, 0.5f);
	NewtonBodyGetMatrix(box2, &matrix[0][0]);
	matrix = dYawMatrix(dPi * 0.5f) * matrix;
	NewtonBodySetMatrix(box2, &matrix[0][0]);
	((DemoEntity*) NewtonBodyGetUserData(box2))->ResetMatrix (*scene, matrix);
	NewtonBodySetOmega(box2, &omega[0]);
	NewtonBodySetLinearDamping(box2, 0.0f);
	NewtonBodySetAngularDamping(box2, &damp[0]);

	// link the two boxes
	NewtonBodyGetMatrix(box1, &matrix[0][0]);
	//dCustomDoubleHinge* const joint1 = new dCustomDoubleHinge(matrix, box1, box0);
dAssert (0);
//	dCustomDoubleHinge* const joint1 = new dCustomDoubleHinge(matrix, box1);
//	joint1->SetHardMiddleAxis(0);
//	joint1->EnableLimits(false);
//	joint1->SetLimits(-5.0f * dPi, 2.0f * dPi);

	// link the two boxes
//	NewtonBodyGetMatrix(box2, &matrix[0][0]);
//	dCustomDoubleHinge* const joint2 = new dCustomDoubleHinge(matrix, box2, box0);
//	joint2->EnableLimits1(true);
//	joint2->EnableLimits1(false);
//	joint2->SetLimits1 (-3.0f * dPi, 5.0f * dPi);
}

class JoesRagdollJoint: public dCustomBallAndSocket
{
	public:
	dQuaternion m_target; // relative target rotation to reach at next timestep

	dFloat m_reduceError;
	dFloat m_pin_length;
	dFloat m_angularFriction;
	dFloat m_stiffness;

	dFloat m_anim_speed;
	dFloat m_anim_offset;
	dFloat m_anim_time;
	
	JoesRagdollJoint(NewtonBody* child, NewtonBody* parent, const dMatrix &localMatrix0, const dMatrix &localMatrix1, NewtonWorld *world)
		:dCustomBallAndSocket(localMatrix0, localMatrix1, child, parent)
	{
		m_localMatrix0 = localMatrix0;
		m_localMatrix1 = localMatrix1;

		m_target = dQuaternion(dVector(1.0f, 0, 0), 0.0f);
		m_reduceError = 0.95f; // amount of error to reduce per timestep (more -> oszillation)
		m_stiffness = 0.98f;
		m_angularFriction = 300.0f;

		m_anim_speed = 0.0f;
		m_anim_offset = 0.0f;
		m_anim_time = 0.0f;
	}

	dVector BodyGetPointVelocity(const NewtonBody* const body, const dVector &point)
	{
		dMatrix matrix;
		dVector v(0.0f);
		dVector w(0.0f);
		dVector c(0.0f);
		NewtonBodyGetVelocity(body, &v[0]);
		NewtonBodyGetOmega(body, &w[0]);
		NewtonBodyGetMatrix(body, &matrix[0][0]);
		c = matrix.m_posit; // TODO: Does not handle COM offset !!!
		return v + w.CrossProduct(point - c);
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		dFloat invTimestep = 1.0f / timestep;

		dMatrix matrix0;
		dMatrix matrix1;

		CalculateGlobalMatrix(matrix0, matrix1);

		if (m_anim_speed != 0.0f) // some animation to illustrate purpose
		{
			m_anim_time += timestep * m_anim_speed;
			dFloat a0 = dSin(m_anim_time);
			dFloat a1 = m_anim_offset * 3.14f;
			dVector axis(dSin(a1), 0.0f, dCos(a1));
			//dVector axis (1,0,0);
			m_target = dQuaternion(axis, a0 * 0.5f);
		}

		// measure error
		dQuaternion q0(matrix0);
		dQuaternion q1(matrix1);
		dQuaternion qt0 = m_target * q1;
		dQuaternion qErr = ((q0.DotProduct(qt0) < 0.0f)	? dQuaternion(-q0.m_q0, q0.m_q1, q0.m_q2, q0.m_q3) : dQuaternion(q0.m_q0, -q0.m_q1, -q0.m_q2, -q0.m_q3)) * qt0;
		qErr.Normalize();

		dFloat errorAngle = 2.0f * dAcos(dMax(dFloat(-1.0f), dMin(dFloat(1.0f), qErr.m_q0)));
		dVector errorAngVel(0, 0, 0);

		dMatrix basis;
		if (errorAngle > 1.0e-10f) {
			dVector errorAxis(qErr.m_q1, qErr.m_q2, qErr.m_q3, 0.0f);
			errorAxis = errorAxis.Scale(1.0f / dSqrt(errorAxis.DotProduct3(errorAxis)));
			errorAngVel = errorAxis.Scale(errorAngle * invTimestep);

			basis = dGrammSchmidt(errorAxis);
		} else {
			basis = dMatrix(qt0, dVector(0.0f, 0.0f, 0.0f, 1.0f));
		}

		dVector angVel0(0.0f);
		dVector angVel1(0.0f);
		NewtonBodyGetOmega(m_body0, (dFloat*)&angVel0);
		NewtonBodyGetOmega(m_body1, (dFloat*)&angVel1);

		dVector angAcc = (errorAngVel.Scale(m_reduceError) - (angVel0 - angVel1)).Scale(invTimestep);

		dCustomBallAndSocket::SubmitConstraints(timestep, threadIndex);
		// motors
		for (int n = 0; n < 3; n++) {
			// calculate the desired acceleration
			dVector &axis = basis[n];
			dFloat relAccel = angAcc.DotProduct3(axis);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &axis[0]);
			NewtonUserJointSetRowAcceleration(m_joint, relAccel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		}
	}
};

/*
void AddJoesPoweredRagDoll (DemoEntityManager* const scene, const dVector& origin, const dFloat animSpeed, const int numSegments)
{
    dFloat height = 1.0f;
    dFloat width = 4.0f;

    dVector size (width, height, width);
    NewtonBody* parent = CreateBox (scene, origin + dVector (0.0f,  0.5f, 0.0f, 0.0f), size);
	
    for (int i=0; i < numSegments; i++)
	{
        dFloat height = 1.0f;
        dFloat width = 0.5f;

        dVector size (width, height, width);
        NewtonBody* child = CreateBox (scene, origin + dVector (0.0f,  0.5f + height * dFloat(i+1), 0.0f, 0.0f), size);

        dMatrix matrix0 = dGetIdentityMatrix(); matrix0.m_posit = dVector (0.0f, height*-0.5f, 0.0f, 1.0f);
        dMatrix matrix1 = dGetIdentityMatrix(); matrix1.m_posit = dVector (0.0f, height*0.5f, 0.0f, 1.0f);
        JoesRagdollJoint* joint = new JoesRagdollJoint (child, parent, matrix0, matrix1, scene->GetNewton());

		if (animSpeed != 0.0f) {
			joint->m_anim_speed = animSpeed, joint->m_anim_offset = dFloat(i) / dFloat(numSegments); // animated      
	}

        parent = child;
	}
}*/


static void AddJoesPoweredRagDoll (DemoEntityManager* const scene, const dVector& origin, const dFloat animSpeed, const int numSegments,
	const int numArms = 1,
	const dFloat torsoHeight = 1.0f, 
	const dFloat torsoWidth = 4.0f, 
	const dFloat randomness = 0.0f, 
	const dFloat armHeight = 1.0f, 
	const dFloat armWidth = 0.5f,
	const int pickMe = -1)
{
    dFloat height = torsoHeight;
    dFloat width = torsoWidth;

    dVector size (width, height, width);
    NewtonBody* torso = CreateBox (scene, origin + dVector (0.0f,  0.5f, 0.0f, 0.0f), size);
	dMatrix torsoMatrix; 
	NewtonBodyGetMatrix (torso, (dFloat*) &torsoMatrix);

	int bodyIndex = 0;
	NewtonBody* pickBody = 0;
	for (int j=0; j < numArms; j++)
	{
		dFloat angle = dFloat(j) / dFloat(numArms) * dPi*2.0f;
		dMatrix armRotation = dPitchMatrix(angle);
		dMatrix armTransform = armRotation * torsoMatrix;
		
		NewtonBody* parent = torso;
		int numBodies = numSegments;
		if (randomness > 0.0f) numBodies += int (dGaussianRandom (dFloat(numSegments)) + 0.5f);
		for (int i=0; i < numBodies; i++)
		{
			dFloat height = armHeight;
			dFloat width = armWidth;

			dVector size (width, height, width);
			dVector pos (0.0f,  height * dFloat(i + (numArms>1 ? 2 : 1)), 0.0f, 0.0f);
			NewtonBody* child = CreateBox (scene, pos, size);
			
			dMatrix bodyMatrix; 
			NewtonBodyGetMatrix (child, (dFloat*) &bodyMatrix);
			bodyMatrix = bodyMatrix * armTransform;
			NewtonBodySetMatrix (child, (dFloat*) &bodyMatrix);

			dMatrix matrix0 = dGetIdentityMatrix(); matrix0.m_posit = dVector (0.0f, height*-0.5f, 0.0f, 1.0f);
			dMatrix matrix1 = dGetIdentityMatrix(); matrix1.m_posit = dVector (0.0f, height*0.5f, 0.0f, 1.0f);
			if (parent == torso) 
			{
				matrix1.m_posit.m_y += height;
				matrix1 = matrix1 * armRotation;
			}
			if (randomness > 0.0f)
			{
				dMatrix rotation =  dPitchMatrix(dGaussianRandom  (dPi) * randomness);
				rotation = rotation * dYawMatrix(dGaussianRandom  (dPi) * randomness);
				rotation = rotation * dYawMatrix(dGaussianRandom  (dPi) * randomness);
				matrix0 = matrix0 * rotation;
			}
			JoesRagdollJoint* joint = new JoesRagdollJoint (child, parent, matrix0, matrix1, scene->GetNewton());

			if (animSpeed != 0.0f) {
				joint->m_anim_speed = animSpeed, joint->m_anim_offset = dFloat(i) / dFloat(numBodies); // animated      
			}

			parent = child;
			if (bodyIndex == pickMe) {
				pickBody = child;
			}
			bodyIndex++;
		}
	}

	if (pickBody)
	{
		dMatrix matrix;
		NewtonBodyGetMatrix(pickBody, &matrix[0][0]);
		new dCustomBallAndSocket(matrix, pickBody);
	}
}

static void AddHinge (DemoEntityManager* const scene, const dVector& origin)
{
    dVector size (1.5f, 1.5f, 0.125f);
	NewtonBody* parent = CreateBox(scene, origin + dVector (-0.8f, 4.0f, 0.0f, 0.0f), dVector (0.2f, 0.125f, 0.125f));
	NewtonBodySetMassMatrix(parent, 0.0f, 0.0f, 0.0f, 0.0f);
    //the joint pin is the first row of the matrix, to make a upright pin we
    //take the x axis and rotate by 90 degree around the y axis
    dMatrix localPin (dRollMatrix(90.0f * dDegreeToRad));

	dMatrix matrix;
	dVector position (origin);
	position.m_y += 4.0f;
	NewtonBody* child = NULL;

	int count = 6;
	for (int i = 0; i < count; i ++) {
		child = CreateBox (scene, position, size);
		NewtonBodyGetMatrix(child, &matrix[0][0]);
		matrix.m_posit += dVector(-size.m_x * 0.5f, 0.0f, 0.0f);
		matrix = localPin * matrix;
		dCustomHinge* const hinge = new dCustomHinge (matrix, child, parent);

		hinge->EnableLimits (true);
		hinge->SetLimits (-45.0f * dDegreeToRad, 45.0f * dDegreeToRad);
		hinge->SetFriction(20.0f);

		parent = child;
		position.m_x += size.m_x;
	}
}


static void AddSlider (DemoEntityManager* const scene, const dVector& origin)
{
    // make a reel static
    NewtonBody* const box0 = CreateBox (scene, origin + dVector (0.0f, 4.0f, 0.0f, 0.0f), dVector (8.0f, 0.25f, 0.25f, 0.0f));
    NewtonBody* const box1 = CreateWheel (scene, origin + dVector (0.0f, 4.0f, 0.0f, 0.0f), 1.0f, 0.5f);

	dMatrix matrix;
	NewtonBodySetMassMatrix(box0, 0.0f, 0.0f, 0.0f, 0.0f);

	// connect the bodies by a Slider joint
    NewtonBodyGetMatrix (box1, &matrix[0][0]);
    dCustomSlider* const slider = new dCustomSlider (matrix, box1, box0);

	// add some friction for more realism
	slider->SetFriction(10.0f);

    // enable limit of first axis
    slider->EnableLimits(true);

    // set limit on second axis
    slider->SetLimits (-4.0f, 4.0f);
}

static void AddSliderSpringDamper (DemoEntityManager* const scene, const dVector& origin)
{
	// make a reel static
	NewtonBody* const box0 = CreateBox(scene, origin + dVector(0.0f, 4.0f, 0.0f, 0.0f), dVector(8.0f, 0.25f, 0.25f, 0.0f));
	NewtonBody* const box1 = CreateWheel(scene, origin + dVector(0.0f, 4.0f, 0.0f, 0.0f), 1.0f, 0.5f);

	dMatrix matrix;
	NewtonBodySetMassMatrix(box0, 0.0f, 0.0f, 0.0f, 0.0f);

	// connect the bodies by a Slider joint
	NewtonBodyGetMatrix(box1, &matrix[0][0]);
	dCustomSlider* const slider = new dCustomSlider(matrix, box1, box0);

	// enable limit of first axis
	slider->EnableLimits(true);

	// set limit on second axis
	slider->SetLimits(-4.0f, 4.0f);
	slider->SetAsSpringDamper(true, 0.7f, 80.0f, 0.0f);
}


static void AddHingeSpringDamper (DemoEntityManager* const scene, const dVector& origin)
{
	dMatrix matrix;

	// add a hinge spring
	NewtonBody* const box0 = CreateCylinder (scene, origin + dVector (0.0f, 4.0f, 0.0f, 0.0f), 0.25f, 2.0f);
	NewtonBody* const box1 = CreateWheel(scene, origin + dVector(0.0f, 4.0f, 0.0f, 0.0f), 1.0f, 0.5f);

	NewtonBodySetMassMatrix(box0, 0.0f, 0.0f, 0.0f, 0.0f);

	// connect the bodies by a Slider joint
	NewtonBodyGetMatrix(box1, &matrix[0][0]);
	dCustomHinge* const hinge = new dCustomHinge(matrix, box1, box0);

	// enable limit of first axis
	hinge->EnableLimits(true);

	// set limit on second axis
	hinge->SetLimits (-120.0f * dDegreeToRad, 120.0f * dDegreeToRad);
	hinge->SetAsSpringDamper(true, 0.9f, 300.0f, 1.0f);
}


static void AddSlidingContact(DemoEntityManager* const scene, const dVector& origin)
{
	// make a reel static
	NewtonBody* const box0 = CreateBox(scene, origin + dVector(0.0f, 4.0f, 0.0f, 0.0f), dVector(8.0f, 0.25f, 0.25f, 0.0f));
	NewtonBody* const box1 = CreateWheel(scene, origin + dVector(0.0f, 4.0f, 0.0f, 0.0f), 1.0f, 0.5f);

	NewtonBodySetMassMatrix(box0, 0.0f, 0.0f, 0.0f, 0.0f);

	dMatrix matrix;
	//rotate body so that is lok like a wheel
	NewtonBodyGetMatrix(box1, &matrix[0][0]);
	matrix = dRollMatrix(90.0f * dDegreeToRad) * matrix;
	NewtonBodySetMatrix(box1, &matrix[0][0]);
	DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(box1);
	ent->ResetMatrix(*scene, matrix);


	//make joint matrix
	dMatrix jointMatrix(dGetIdentityMatrix());
	jointMatrix.m_posit = matrix.m_posit;

	dCustomSlidingContact* const slider = new dCustomSlidingContact(jointMatrix, box1, box0);
	slider->EnableLimits (true);
	slider->SetLimits (-4.0f, 4.0f);
	slider->SetAsSpringDamper(true, 0.7f, 80.0f, 0.0f);

	// enable limit of first axis
	slider->EnableAngularLimits(true);
	slider->SetAngularLimits(-7.0f * dPi, 5.0f * dPi);
}

static void AddCylindrical (DemoEntityManager* const scene, const dVector& origin)
{
    // make a reel static
    NewtonBody* const box0 = CreateCylinder (scene, origin + dVector (0.0f, 4.0f, 0.0f, 0.0f), 0.25f, 8.0f);
    NewtonBody* const box1 = CreateWheel (scene, origin + dVector (0.0f, 4.0f, 0.0f, 0.0f), 1.0f, 0.5f);

    dMatrix matrix;
	NewtonBodySetMassMatrix(box0, 0.0f, 0.0f, 0.0f, 0.0f);

	// connect the bodies by a CorkScrew joint
    NewtonBodyGetMatrix (box1, &matrix[0][0]);
    dCustomCorkScrew* const cylinder = new dCustomCorkScrew (matrix, box1, box0);

    // enable limit of first axis
    cylinder->EnableLimits(true);
    cylinder->SetLimits (-4.0f, 4.0f);

	// set angular limit on second axis
	cylinder->EnableAngularLimits(true);
	cylinder->SetAngularLimits(-4.0f * dPi, 6.0f * dPi);
}

static dCustomHinge* AddHingeWheel (DemoEntityManager* const scene, const dVector& origin, dFloat radius, dFloat height, NewtonBody* const parent)
{
    NewtonBody* const wheel = CreateWheel (scene, origin, height, radius);

    // the joint pin is the first row of the matrix
    //dMatrix localPin (dRollMatrix(90.0f * dDegreeToRad));
    dMatrix localPin (dGetIdentityMatrix());
    dMatrix matrix;
    NewtonBodyGetMatrix (wheel, & matrix[0][0]);
    matrix = localPin * matrix;

    // connect first box to the world
    return new dCustomHinge (matrix, wheel, parent);
}


static void AddGear (DemoEntityManager* const scene, const dVector& origin)
{
    NewtonBody* const box0 = CreateCylinder(scene, origin + dVector (0.0f, 4.0f, 0.0f), 0.25f, 4.0f);

	// this is a fix joint
	NewtonBodySetMassMatrix(box0, 0.0f, 0.0f, 0.0f, 0.0f);

	// connect two bodies with a hinge 
    dCustomHinge* const hinge0 = AddHingeWheel (scene, origin + dVector (-1.0f, 4.0f, 0.0f), 0.5f, 1.0f, box0);
    dCustomHinge* const hinge1 = AddHingeWheel (scene, origin + dVector ( 1.0f, 4.0f, 0.0f), 0.5f, 1.0f, box0);

    NewtonBody* const body0 = hinge0->GetBody0();
    NewtonBody* const body1 = hinge1->GetBody0();

    dMatrix matrix0;
    dMatrix matrix1;
    NewtonBodyGetMatrix (body0, &matrix0[0][0]);
    NewtonBodyGetMatrix (body1, &matrix1[0][0]);

	// relate the two body motion with a gear joint
    dVector pin0 (matrix0.RotateVector(dVector (1.0f, 0.0f, 0.0f)));
    dVector pin1 (matrix1.RotateVector(dVector (1.0f, 0.0f, 0.0f)));
    new dCustomGear (4.0f, pin0, pin1, body0, body1);
}


static dCustomSlider* AddSliderWheel (DemoEntityManager* const scene, const dVector& origin, dFloat radius, dFloat height, NewtonBody* const parent)
{
    NewtonBody* const wheel = CreateWheel (scene, origin, height, radius);

    // the joint pin is the first row of the matrix
    //dMatrix localPin (dRollMatrix(90.0f * dDegreeToRad));
    dMatrix localPin (dGetIdentityMatrix());
    dMatrix matrix;
    NewtonBodyGetMatrix (wheel, & matrix[0][0]);
    matrix = localPin * matrix;

    // connect first box to the world
    return new dCustomSlider (matrix, wheel, parent);
}

void AddPulley (DemoEntityManager* const scene, const dVector& origin)
{
    NewtonBody* const reel0 = CreateBox(scene, origin + dVector (0.0f, 4.0f, 2.0f), dVector(4.0f, 0.25f, 0.25f));
	// this is just for show
    NewtonBody* const reel1 = CreateBox(scene, origin + dVector (0.0f, 4.0f, 0.0f), dVector(4.0f, 0.25f, 0.25f));
	NewtonBodySetMassMatrix (reel0, 0.0f, 0.0f, 0.0f, 0.0f);
	NewtonBodySetMassMatrix (reel1, 0.0f, 0.0f, 0.0f, 0.0f);
    
	dMatrix matrix;
    dCustomSlider* const slider0 = AddSliderWheel (scene, origin + dVector (0.0f, 4.0f, 2.0f), 0.5f, 1.0f, reel0);
    dCustomSlider* const slider1 = AddSliderWheel (scene, origin + dVector (0.0f, 4.0f, 0.0f), 0.5f, 0.5f, reel0);

    slider0->EnableLimits(true);
    slider0->SetLimits (-2.0f, 2.0f);

    NewtonBody* const body0 = slider0->GetBody0();
    NewtonBody* const body1 = slider1->GetBody0();

    dMatrix matrix0;
    dMatrix matrix1;
    NewtonBodyGetMatrix (body0, &matrix0[0][0]);
    NewtonBodyGetMatrix (body1, &matrix1[0][0]);

    dVector pin0 (matrix0.RotateVector(dVector (1.0f, 0.0f, 0.0f)));
    dVector pin1 (matrix1.RotateVector(dVector (1.0f, 0.0f, 0.0f)));
    new dCustomPulley (4.0f, pin0, pin1, body0, body1);

	// make an aggregate for disabling collisions
	void* const aggregate = NewtonCollisionAggregateCreate (scene->GetNewton());
	NewtonCollisionAggregateSetSelfCollision (aggregate, 0);
	NewtonCollisionAggregateAddBody (aggregate, reel0);
	NewtonCollisionAggregateAddBody (aggregate, reel1);
	NewtonCollisionAggregateAddBody (aggregate, body0);
	NewtonCollisionAggregateAddBody (aggregate, body1);
}

static dCustomCorkScrew* AddCylindricalWheel (DemoEntityManager* const scene, const dVector& origin, dFloat radius, dFloat height, NewtonBody* const parent)
{
    NewtonBody* const wheel = CreateWheel (scene, origin, height, radius);

    // the joint pin is the first row of the matrix
    dMatrix matrix;
    NewtonBodyGetMatrix (wheel, &matrix[0][0]);

    return new dCustomCorkScrew (matrix, wheel, parent);
}

static void AddGearAndRack (DemoEntityManager* const scene, const dVector& origin)
{
    NewtonBody* const reel0 = CreateCylinder(scene, origin + dVector (0.0f, 4.0f, 0.0f), 0.25f, 4.0f);
    NewtonBody* const reel1 = CreateBox(scene, origin + dVector (0.0f, 4.0f, 2.0f), dVector(4.0f, 0.25f, 0.25f));
    
	dMatrix matrix;
	NewtonBodySetMassMatrix(reel0, 0.0f, 0.0f, 0.0f, 0.0f);
	NewtonBodySetMassMatrix(reel1, 0.0f, 0.0f, 0.0f, 0.0f);

    dCustomHinge* const hinge0 = AddHingeWheel (scene, origin + dVector (-1.0f, 4.0f, 0.0f), 0.5f, 0.5f, reel0);
    dCustomHinge* const hinge1 = AddHingeWheel (scene, origin + dVector ( 1.0f, 4.0f, 0.0f), 0.5f, 0.5f, reel0);
    dCustomCorkScrew* const cylinder = AddCylindricalWheel(scene, origin + dVector (0.0f, 4.0f, 2.0f), 0.5f, 1.0f, reel0);

    cylinder->EnableLimits(true);
    cylinder->SetLimits(-2.0f, 2.0f);

    NewtonBody* const body0 = hinge0->GetBody0();
    NewtonBody* const body1 = hinge1->GetBody0();
    NewtonBody* const body2 = cylinder->GetBody0();

    dMatrix matrix0;
    dMatrix matrix1;
    dMatrix matrix2;

    NewtonBodyGetMatrix (body0, &matrix0[0][0]);
    NewtonBodyGetMatrix (body1, &matrix1[0][0]);
    NewtonBodyGetMatrix (body2, &matrix2[0][0]);

    dVector pin0 (matrix0.RotateVector(dVector( 1.0f, 0.0f, 0.0f)));
    dVector pin1 (matrix1.RotateVector(dVector( 1.0f, 0.0f, 0.0f)));
    dVector pin2 (matrix2.RotateVector(dVector( 1.0f, 0.0f, 0.0f)));

    new dCustomGear (5.0f, pin0, pin2, body0, body2);
    new dCustomRackAndPinion (0.125f, pin1, pin2, body1, body2);

	// make an aggregate for disabling collisions
	void* const aggregate = NewtonCollisionAggregateCreate(scene->GetNewton());
	NewtonCollisionAggregateSetSelfCollision(aggregate, 0);
	NewtonCollisionAggregateAddBody(aggregate, reel0);
	NewtonCollisionAggregateAddBody(aggregate, reel1);
	NewtonCollisionAggregateAddBody(aggregate, body0);
	NewtonCollisionAggregateAddBody(aggregate, body1);
	NewtonCollisionAggregateAddBody(aggregate, body2);
}


static void AddDifferential(DemoEntityManager* const scene, const dVector& origin)
{
	dVector size(1.0f, 1.0f, 1.0f);
	NewtonBody* const box0 = CreateBox(scene, origin + dVector(0.0f, 4.0f, 0.0f, 0.0f), dVector(0.25f, 0.25f, 4.0f, 0.0f));
	NewtonBody* const box1 = CreateWheel(scene, origin + dVector(0.0f, 4.0f, 2.0f, 0.0f), 1.0f, 0.5f);
	NewtonBody* const box2 = CreateWheel(scene, origin + dVector(0.0f, 4.0f, -2.0f, 0.0f), 1.0f, 0.5f);
	NewtonBody* const box3 = CreateWheel(scene, origin + dVector(0.0f, 4.0f,  0.0f, 0.0f), 1.0f, 0.5f);

	NewtonBodySetMassMatrix(box0, 0.0f, 0.0f, 0.0f, 0.0f);

	// align the object so that is looks nice
	dMatrix matrix;
	NewtonBodyGetMatrix(box1, &matrix[0][0]);
	matrix = dYawMatrix(dPi * 0.5f) * matrix;
	NewtonBodySetMatrix(box1, &matrix[0][0]);
	((DemoEntity*)NewtonBodyGetUserData(box1))->ResetMatrix(*scene, matrix);

	NewtonBodyGetMatrix(box2, &matrix[0][0]);
	matrix = dYawMatrix(dPi * 0.5f) * matrix;
	NewtonBodySetMatrix(box2, &matrix[0][0]);
	((DemoEntity*)NewtonBodyGetUserData(box2))->ResetMatrix(*scene, matrix);

	// connect right tire
	dMatrix matrix1;
	NewtonBodyGetMatrix(box1, &matrix1[0][0]);
	dCustomHinge* const joint1 = new dCustomHinge(matrix1, box1, box0);
	joint1->EnableLimits(false);

	// connect left tire
	dMatrix matrix2;
	NewtonBodyGetMatrix(box2, &matrix2[0][0]);
	dCustomHinge* const joint2 = new dCustomHinge(matrix2, box2, box0);
	joint2->EnableLimits(false);

	// make the gear system
	dMatrix matrix3;
	NewtonBodyGetMatrix(box3, &matrix3[0][0]);
	new dCustomDoubleHinge(matrix3, box3, box0);

	dMatrix referenceMatrix;
	NewtonBodyGetMatrix(box0, &referenceMatrix[0][0]);

	// connect right differential
	new dCustomDifferentialGear(2.0f, matrix1.m_front, matrix3.m_front, referenceMatrix.m_up, box1, box3, box0);

	// connect left differential
	new dCustomDifferentialGear(2.0f, matrix2.m_front, matrix3.m_front, referenceMatrix.m_up.Scale (-1.0f), box2, box3, box0);

	dVector damp(0.0f);
	dVector omega(10.0f, 10.0f, 0.0f, 0.0f);
	NewtonBodySetOmega(box3, &omega[0]);
	NewtonBodySetAngularDamping(box1, &damp[0]);
	NewtonBodySetAngularDamping(box2, &damp[0]);
	NewtonBodySetAngularDamping(box3, &damp[0]);
}


class MyPathFollow: public dCustomPathFollow
{
	public:
	MyPathFollow(const dMatrix& pinAndPivotFrame, NewtonBody* const body, NewtonBody* const pathBody)
		:dCustomPathFollow (pinAndPivotFrame, body, pathBody)
	{
	}

	void GetPointAndTangentAtLocation (const dVector& location,  dVector& positOut, dVector& tangentOut) const
	{
		DemoEntity* const pathEntity = (DemoEntity*) NewtonBodyGetUserData (GetBody1());
		DemoBezierCurve* const mesh = (DemoBezierCurve*)pathEntity->GetMesh();
		const dBezierSpline& spline = mesh->m_curve;

		dMatrix matrix;
		NewtonBodyGetMatrix(GetBody1(), &matrix[0][0]);

		dVector p(matrix.UntransformVector(location));
		dBigVector point;
		dFloat64 knot = spline.FindClosestKnot (point, p, 4);
		dBigVector tangent (spline.CurveDerivative (knot));
		tangent = tangent.Scale (1.0 / sqrt (tangent.DotProduct3(tangent)));

//		positOut = matrix.TransformVector (dVector (point.m_x, point.m_y, point.m_z));
		positOut = matrix.TransformVector (point);
		tangentOut = tangent;
	}
};

static void AddPathFollow (DemoEntityManager* const scene, const dVector& origin)
{
	// create a Bezier Spline path for AI car to drive
	NewtonBody* const pathBody = CreateBox(scene, origin, dVector(4.0f, 0.25f, 0.25f));
	NewtonBodySetMassMatrix(pathBody, 0.0f, 0.0f, 0.0f, 0.0f);
	DemoEntity* const rollerCosterPath = (DemoEntity*) NewtonBodyGetUserData(pathBody);
	
	dBezierSpline spline;
	dFloat64 knots[] = {0.0f, 1.0f / 5.0f, 2.0f / 5.0f, 3.0f / 5.0f, 4.0f / 5.0f, 1.0f};

	dBigVector control[] =
	{
		dBigVector(100.0f - 100.0f, 20.0f, 200.0f - 250.0f, 1.0f),
		dBigVector(150.0f - 100.0f, 10.0f, 150.0f - 250.0f, 1.0f),
		dBigVector(175.0f - 100.0f, 30.0f, 250.0f - 250.0f, 1.0f),
		dBigVector(200.0f - 100.0f, 70.0f, 250.0f - 250.0f, 1.0f),
		dBigVector(215.0f - 100.0f, 20.0f, 250.0f - 250.0f, 1.0f),
		dBigVector(150.0f - 100.0f, 50.0f, 350.0f - 250.0f, 1.0f),
		dBigVector( 50.0f - 100.0f, 30.0f, 250.0f - 250.0f, 1.0f),
		dBigVector(100.0f - 100.0f, 20.0f, 200.0f - 250.0f, 1.0f),
	};

	spline.CreateFromKnotVectorAndControlPoints(3, sizeof (knots) / sizeof (knots[0]), knots, control);

	DemoBezierCurve* const mesh = new DemoBezierCurve (spline);
	rollerCosterPath->SetMesh(mesh, dGetIdentityMatrix());
	
	mesh->SetVisible(true);
	mesh->SetRenderResolution(500);
	mesh->Release();
	
	const int count = 32;
	NewtonBody* bodies[count];

	dBigVector point0;
	dVector positions[count + 1];
	dFloat64 knot = spline.FindClosestKnot(point0, dBigVector (dVector(100.0f - 100.0f, 20.0f, 200.0f - 250.0f, 0.0f)), 4);
	positions[0] = point0;
	for (int i = 0; i < count; i ++) {
		dBigVector point1;
		dBigVector tangent(spline.CurveDerivative(knot));
		tangent = tangent.Scale (1.0 / sqrt (tangent.DotProduct3(tangent)));
		knot = spline.FindClosestKnot(point1, dBigVector (point0 + tangent.Scale (2.0f)), 4);
		point0 = point1;
		positions[i + 1] = point1;
	}

	dMatrix pathBodyMatrix;
	NewtonBodyGetMatrix(pathBody, &pathBodyMatrix[0][0]);

	dFloat attachmentOffset = 0.8f;
	for (int i = 0; i < count; i ++) {
		dMatrix matrix;
		bodies[i] = CreateWheel(scene, dVector (0.0f, 0.0f, 0.0f, 0.0f), 1.0f, 0.5f);
		NewtonBodySetLinearDamping(bodies[i], 0.0f);
		NewtonBody* const box = bodies[i];
		NewtonBodyGetMatrix(box, &matrix[0][0]);

		dVector location0(positions[i + 0].m_x, positions[i + 0].m_y, positions[i + 0].m_z, 0.0);
		dVector location1(positions[i + 1].m_x, positions[i + 1].m_y, positions[i + 1].m_z, 0.0);

		location0 = pathBodyMatrix.TransformVector(location0);
		location1 = pathBodyMatrix.TransformVector(location1);

		dVector dir (location1 - location0);
		dir.m_w = 0.0f;
		matrix.m_front = dir.Scale (1.0f / dSqrt (dir.DotProduct3(dir)));
		matrix.m_right = matrix.m_front.CrossProduct(matrix.m_up);
		matrix.m_right = matrix.m_right.Scale(1.0f / dSqrt(matrix.m_right.DotProduct3(matrix.m_right)));
		matrix.m_up = matrix.m_right.CrossProduct(matrix.m_front);
		matrix.m_posit = pathBodyMatrix.TransformVector(dVector (positions[i].m_x, positions[i].m_y - attachmentOffset, positions[i].m_z, 1.0));
		dMatrix matrix1 (dYawMatrix(0.5f * dPi) * matrix);

		NewtonBodySetMatrix(box, &matrix1[0][0]);
		DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(box);
		ent->ResetMatrix(*scene, matrix1);

		matrix.m_posit = pathBodyMatrix.TransformVector(dVector(positions[i].m_x, positions[i].m_y, positions[i].m_z, 1.0));
		new MyPathFollow(matrix, box, pathBody);

		dVector veloc (dir.Scale (20.0f));
		NewtonBodySetVelocity(box, &veloc[0]);
	}
	
	for (int i = 1; i < count; i ++) {
		NewtonBody* const box0 = bodies[i - 1];
		NewtonBody* const box1 = bodies[i];

		dMatrix matrix0;
		dMatrix matrix1;
		NewtonBodyGetMatrix(box0, &matrix0[0][0]);
		NewtonBodyGetMatrix(box1, &matrix1[0][0]);

		matrix0.m_posit.m_y += attachmentOffset;
		matrix1.m_posit.m_y += attachmentOffset;

		//new CustomDistanceRope (matrix1.m_posit, matrix0.m_posit, box1, box0);
		new dCustomFixDistance (matrix1.m_posit, matrix0.m_posit, box1, box0);
	}

	void* const aggregate = NewtonCollisionAggregateCreate (scene->GetNewton());
	for (int i = 0; i < count; i ++) {
		NewtonCollisionAggregateAddBody(aggregate, bodies[i]);
	}
	NewtonCollisionAggregateSetSelfCollision (aggregate, false);
}



struct JoesNewRagdollJoint: public dCustomJoint
{
	dQuaternion m_target; // relative target rotation to reach at next timestep

	// motor:
	dFloat m_reduceError;
	dFloat m_pin_length;
	dFloat m_angularFriction;
	dFloat m_stiffness;

	dFloat m_anim_speed;
	dFloat m_anim_offset;
	dFloat m_anim_time;

	// limits:
	bool m_isLimitJoint;

	dFloat m_minTwistAngle;
	dFloat m_maxTwistAngle;

	dFloat m_coneAngle;
	dFloat m_arcAngleCos;
	dFloat m_arcAngleSin;


	JoesNewRagdollJoint(NewtonBody* child, NewtonBody* parent, const dMatrix &localMatrix0, const dMatrix &localMatrix1, NewtonWorld *world, bool isLimitJoint = false)
		:dCustomJoint(6, child, parent)
	{
		m_localMatrix0 = localMatrix0;
		m_localMatrix1 = localMatrix1;

		m_target = dQuaternion(dVector(1.0f, 0, 0), 0.0f);
		m_reduceError = 0.95f; // amount of error to reduce per timestep (more -> oszillation)
		m_stiffness = 0.98f;
		m_angularFriction = 300.0f;

		m_anim_speed = 0.0f;
		m_anim_offset = 0.0f;
		m_anim_time = 0.0f;

		m_isLimitJoint = isLimitJoint;

		SetTwistSwingLimits(0.1f, 0.2f, -0.1f, 0.1f);
		//SetTwistSwingLimits (0.1f, 0.0f, -0.1f, 0.1f);
	}

	void SetTwistSwingLimits(const dFloat coneAngle, const dFloat arcAngle, const dFloat minTwistAngle, const dFloat maxTwistAngle)
	{
		dFloat const maxAng = 2.8f; // to prevent flipping on the pole on the backside

		m_coneAngle = min(maxAng, coneAngle);
		dFloat angle = max(0.0f, min(maxAng, arcAngle + m_coneAngle) - m_coneAngle);
		m_arcAngleCos = dFloat(cos(angle));
		m_arcAngleSin = dFloat(sin(angle));

		m_minTwistAngle = minTwistAngle;
		m_maxTwistAngle = maxTwistAngle;
	}

	dVector BodyGetPointVelocity(const NewtonBody* const body, const dVector &point)
	{
		dMatrix matrix;
		dVector v(0.0f);
		dVector w(0.0f);
		dVector c(0.0f);
		NewtonBodyGetVelocity(body, &v[0]);
		NewtonBodyGetOmega(body, &w[0]);
		NewtonBodyGetMatrix(body, &matrix[0][0]);
		c = matrix.m_posit; // TODO: Does not handle COM offset !!!
		return v + w.CrossProduct(point - c);
	}

	static void SubmitConstraintsCallback(const NewtonJoint* const userJoint, dFloat timestep, int threadIndex)
	{
		JoesNewRagdollJoint *joint = (JoesNewRagdollJoint*)NewtonJointGetUserData(userJoint);
		joint->SubmitConstraints(timestep, threadIndex);
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		dFloat invTimestep = 1.0f / timestep;

		dMatrix matrix0;
		dMatrix matrix1;

		//CalculateGlobalMatrix(matrix0, matrix1);
		dMatrix body0Matrix;
		// Get the global matrices of each rigid body.
		NewtonBodyGetMatrix(m_body0, &body0Matrix[0][0]);

		dMatrix body1Matrix(dGetIdentityMatrix());
		if (m_body1) {
			NewtonBodyGetMatrix(m_body1, &body1Matrix[0][0]);
		}
		matrix0 = m_localMatrix0 * body0Matrix;
		matrix1 = m_localMatrix1 * body1Matrix;



		const dVector& p0 = matrix0.m_posit;
		const dVector& p1 = matrix1.m_posit;
		NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_front[0]);
		NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_up[0]);
		NewtonUserJointAddLinearRow(m_joint, &p0[0], &p1[0], &matrix0.m_right[0]);



		if (m_isLimitJoint) {

			//dCustomBallAndSocket::SubmitConstraints(timestep, threadIndex);


			const dVector& coneDir0 = matrix0.m_front;
			const dVector& coneDir1 = matrix1.m_front;
			dFloat dot = coneDir0.DotProduct3(coneDir1);
			if (dot < -0.999) return; // should never happen

			// do the twist

			if (m_maxTwistAngle >= m_minTwistAngle) // twist restricted?
			{
				dQuaternion quat0(matrix0), quat1(matrix1);
				dFloat *q0 = (dFloat*)&quat0;
				dFloat *q1 = (dFloat*)&quat1;

				// factor rotation about x axis between quat0 and quat1. Code is an optimization of this: qt = q0.Inversed() * q1; halfTwistAngle = atan (qt.x / qt.w);
				dFloat twistAngle = 2.0f * dFloat(atan(
					((((q0[0] * q1[1]) + (-q0[1] * q1[0])) + (-q0[2] * q1[3])) - (-q0[3] * q1[2])) /
					((((q0[0] * q1[0]) - (-q0[1] * q1[1])) - (-q0[2] * q1[2])) - (-q0[3] * q1[3]))));

				// select an axis for the twist - any on the unit arc from coneDir0 to coneDir1 would do - average seemed best after some tests
				dVector twistAxis = coneDir0 + coneDir1;
				twistAxis = twistAxis.Scale(1.0f / dFloat(sqrt(twistAxis.DotProduct3(twistAxis))));

				if (m_maxTwistAngle == m_minTwistAngle) // no freedom for any twist
				{
					NewtonUserJointAddAngularRow(m_joint, twistAngle - m_maxTwistAngle, (dFloat*)&twistAxis);
				}
				else if (twistAngle > m_maxTwistAngle) {
					NewtonUserJointAddAngularRow(m_joint, twistAngle - m_maxTwistAngle, (dFloat*)&twistAxis);
					NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
				}
				else if (twistAngle < m_minTwistAngle) {
					NewtonUserJointAddAngularRow(m_joint, twistAngle - m_minTwistAngle, (dFloat*)&twistAxis);
					NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);
				}
			}

			// do the swing
#if 0
			// simple cone limit:

			dFloat angle = acos(dot) - m_coneAngle;
			if (angle > 0) {
				dVector swingAxis = (coneDir0.CrossProduct(coneDir1));
				swingAxis = swingAxis.Scale(1.0 / sqrt(swingAxis.DotProduct3(swingAxis)));
				NewtonUserJointAddAngularRow(m_joint, angle, (dFloat*)&swingAxis);
				NewtonUserJointSetRowMinimumFriction(m_joint, 0.0);
			}
#else
			// cone / arc limit - think of an piece of pizza (arc) and an allowed max distance from it (cone):

			if (m_coneAngle > 0.0f && dot < 0.999f) {
				// project current axis to the arc plane (y)
				dVector d = matrix1.UnrotateVector(matrix0.m_front);
				dVector cone = d; cone.m_y = 0; cone = cone.Scale(1.0f / dFloat(sqrt(cone.DotProduct3(cone))));

				// clamp the result to be within the arc angle
				if (cone.m_x < m_arcAngleCos)
					cone = dVector(m_arcAngleCos, 0.0f, ((cone.m_z < 0.0f) ? -m_arcAngleSin : m_arcAngleSin));

				// do a regular cone constraint from that
				dFloat angle = dFloat(acos(max(-1.0f, min(1.0f, d.DotProduct3(cone))))) - m_coneAngle;
				if (angle > 0.0f) {
					dVector swingAxis = matrix1.RotateVector(d.CrossProduct(cone));
					swingAxis = swingAxis.Scale(1.0f / dFloat(sqrt(swingAxis.DotProduct3(swingAxis))));
					NewtonUserJointAddAngularRow(m_joint, angle, (dFloat*)&swingAxis);
					NewtonUserJointSetRowMinimumFriction(m_joint, 0.0f);
				}
			}
#endif
		}
		else {

			if (m_anim_speed != 0.0f) // some animation to illustrate purpose
			{
				m_anim_time += timestep * m_anim_speed;
				dFloat a0 = dFloat(sin(m_anim_time));
				dFloat a1 = m_anim_offset * dPi;
				dVector axis(dFloat(sin(a1)), 0.0f, dFloat(cos(a1)));
				//dVector axis (1,0,0);
				m_target = dQuaternion(axis, a0 * 0.5f);
			}

			// measure error
			dQuaternion q0(matrix0);
			dQuaternion q1(matrix1);
			dQuaternion qt0 = m_target * q1;
			dQuaternion qErr = ((q0.DotProduct(qt0) < 0.0f) ? dQuaternion(-q0.m_q0, q0.m_q1, q0.m_q2, q0.m_q3) : dQuaternion(q0.m_q0, -q0.m_q1, -q0.m_q2, -q0.m_q3)) * qt0;
			qErr.Normalize();

			dFloat errorAngle = 2.0f * dFloat(acos(dMax(dFloat(-1.0f), dMin(dFloat(1.0f), qErr.m_q0))));
			dVector errorAngVel(0, 0, 0);

			dMatrix basis;
			if (errorAngle > 1.0e-10f) {
				dVector errorAxis(qErr.m_q1, qErr.m_q2, qErr.m_q3, 0.0f);
				errorAxis = errorAxis.Scale(1.0f / dSqrt(errorAxis.DotProduct3(errorAxis)));
				errorAngVel = errorAxis.Scale(errorAngle * invTimestep);

				basis = dGrammSchmidt(errorAxis);
			}
			else {
				basis = dMatrix(qt0, dVector(0.0f, 0.0f, 0.0f, 1.0f));
			}

			dVector angVel0(0.0f);
			dVector angVel1(0.0f);
			NewtonBodyGetOmega(m_body0, (dFloat*)&angVel0);
			NewtonBodyGetOmega(m_body1, (dFloat*)&angVel1);

			dVector angAcc = (errorAngVel.Scale(m_reduceError) - (angVel0 - angVel1)).Scale(invTimestep);

			//dCustomBallAndSocket::SubmitConstraints(timestep, threadIndex);
			// motors
			for (int n = 0; n < 3; n++) {
				// calculate the desired acceleration
				dVector &axis = basis[n];
				dFloat relAccel = angAcc.DotProduct3(axis);

				NewtonUserJointAddAngularRow(m_joint, 0.0f, &axis[0]);
				NewtonUserJointSetRowAcceleration(m_joint, relAccel);
				NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
				NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
				NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
			}

		}
	}
};


void AddJoesLimitJoint (DemoEntityManager* const scene, const dVector& origin)
{
   NewtonBody* shoulder =   CreateBox (scene, origin + dVector (-0.5f,  2.0f, 0.0f, 0.0f), dVector (0.5f,  0.5f, 0.5f, 0.0f));
   NewtonBodySetMassMatrix (shoulder, 0,0,0,0);
   NewtonBody* bicep =      CreateBox (scene, origin + dVector ( 0.0f,  2.0f, 0.0f, 0.0f), dVector (1.0f,  0.25f, 0.25f, 0.0f));
   NewtonBody* arm =      CreateBox (scene, origin + dVector ( 1.0f,  2.0f, 0.0f, 0.0f), dVector (1.0f,  0.2f, 0.2f, 0.0f));

   dMatrix localMatShoulder0 = dGetIdentityMatrix(); localMatShoulder0.m_posit = dVector (0.25f, 0.0f, 0.0f, 1.0f);
   dMatrix localMatShoulder1 = dGetIdentityMatrix(); localMatShoulder1.m_posit = dVector (-0.5f, 0.0f, 0.0f, 1.0f);

   dMatrix localMatBicep0 = dGetIdentityMatrix(); localMatBicep0.m_posit = dVector (0.5f, 0.0f, 0.0f, 1.0f);
   dMatrix localMatBicep1 = dGetIdentityMatrix(); localMatBicep1.m_posit = dVector (-0.5f, 0.0f, 0.0f, 1.0f);
   /*{
      dMatrix rotation =  dPitchMatrix(randF(bodyIndex*3+0) * M_PI * 0.25f * randomness);
      rotation = rotation * dYawMatrix(randF(bodyIndex*3+1) * M_PI * 0.25f * randomness);
      rotation = rotation * dYawMatrix(randF(bodyIndex*3+2) * M_PI * 0.25f * randomness);
      matrix0 = matrix0 * rotation;
   }*/
   JoesNewRagdollJoint* ellbowJoint = new JoesNewRagdollJoint (arm, bicep, localMatBicep1, localMatBicep0, scene->GetNewton(), true);
   NewtonUserJointSetSolverModel (ellbowJoint->GetJoint(), 2);
   JoesNewRagdollJoint* shoulderJoint = new JoesNewRagdollJoint (bicep, shoulder, localMatShoulder1, localMatShoulder0, scene->GetNewton(), true);
   NewtonUserJointSetSolverModel (shoulderJoint->GetJoint(), 2);
}


void StandardJoints (DemoEntityManager* const scene)
{
    scene->CreateSkyBox();

    // customize the scene after loading
    // set a user friction variable in the body for variable friction demos
    // later this will be done using LUA script
    dMatrix offsetMatrix (dGetIdentityMatrix());

    CreateLevelMesh (scene, "flatPlane.ngd", 1);

    dVector location (0.0f);
    dVector size (1.5f, 2.0f, 2.0f, 0.0f);

//	Add6DOF(scene, dVector(-20.0f, 0.0f, 32.0f));
//	AddJoesPoweredRagDoll(scene, dVector(40.0f, 10.0f, -30.0f), 0.0f, 20);
//	AddJoesPoweredRagDoll(scene, dVector(40.0f, 10.0f, -20.0f), 1.5f, 4);
//	AddJoesPoweredRagDoll(scene, dVector(40.0f, 10.0f, -10.0f), 0.0f, 4);
//	AddJoesPoweredRagDoll(scene, dVector(40.0f, 10.0f,   0.0f), 0.0f, 4, 4, 1.0f, 1.0f);
//	AddJoesPoweredRagDoll(scene, dVector(40.0f, 10.0f,  10.0f), 0.0f, 7, 2, 0.4f, 0.4f, 1.3f);
//	AddJoesPoweredRagDoll(scene, dVector(40.0f, 10.0f,  20.0f), 0.0f, 5, 3, 0.4f, 0.4f, 1.0f, 0.5f, 0.5f);
//	AddJoesPoweredRagDoll(scene, dVector(40.0f, 10.0f,  30.0f), 0.0f, 3, 5, 1.0f, 1.0f, 1.3f, 0.5f, 0.5f, 4); // no picking problem here
//	AddJoesLimitJoint (scene, dVector(-24.0f, 0.0f, -15.0f));

//	AddDoubleHinge(scene, dVector(-20.0f, 0.0f, 30.0f));

#if 1
	Add6DOF (scene, dVector (-20.0f, 0.0f, -25.0f));
	AddDistance (scene, dVector (-20.0f, 0.0f, -20.0f));
	AddLimitedBallAndSocket (scene, dVector (-20.0f, 0.0f, -15.0f));
	AddBallAndSockectWithFriction (scene, dVector (-20.0f, 0.0f, -10.0f));
	AddFixDistance(scene, dVector(-20.0f, 0.0f, -5.0f));
	AddHinge (scene, dVector (-20.0f, 0.0f, 0.0f));
	AddHingeSpringDamper (scene, dVector (dVector (-20.0f, 0.0f, 5.0f)));
	AddSlider (scene, dVector (-20.0f, 0.0f, 7.0f));
	AddSliderSpringDamper (scene, dVector (dVector (-20.0f, 0.0f, 9.0f)));
	AddCylindrical (scene, dVector (-20.0f, 0.0f, 11.0f));
	AddSlidingContact (scene, dVector (-20.0f, 0.0f, 13.0f));
//	AddDoubleHinge(scene, dVector (-20.0f, 0.0f, 17.0f));
	AddGear (scene, dVector (-20.0f, 0.0f, 22.0f));
	AddPulley (scene, dVector (-20.0f, 0.0f, 25.0f));
	AddGearAndRack (scene, dVector (-20.0f, 0.0f, 29.0f));

//	AddPathFollow (scene, dVector (20.0f, 0.0f, 0.0f));

#endif
    // place camera into position
    dMatrix camMatrix (dGetIdentityMatrix());
    dQuaternion rot (camMatrix);
	//dVector origin (-50.0f, 5.0f, 0.0f, 0.0f);
	//dVector origin (-30.0f, 5.0f, 10.0f, 0.0f);
	dVector origin(-30.0f, 5.0f, 32.0f, 0.0f);
    scene->SetCameraMatrix(rot, origin);
}


