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
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DemoMesh.h"
#include "OpenGlUtil.h"

class dFlexyPipeHandle: public dCustomJoint
{
	public:
	dFlexyPipeHandle(NewtonBody* const body, const dVector& pin)
		:dCustomJoint(6, body, NULL)
	{
		m_localMatrix0 = dGrammSchmidt(pin);

		SetSolverModel(3);
		m_angularFriction = 200.0f;
		m_linearFriction = 3000.0f;
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		dMatrix matrix;
		NewtonBodyGetMatrix(m_body0, &matrix[0][0]);
		matrix = GetMatrix0() * matrix;

		for (int i = 0; i < 3; i++) {
			NewtonUserJointAddLinearRow(m_joint, &matrix.m_posit[0], &matrix.m_posit[0], &matrix[i][0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

			const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_linearFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_linearFriction);
		}

		// because this is a velocity base dry friction, we can use a small angule appriximation 
		for (int i = 0; i < 3; i++) {
			NewtonUserJointAddAngularRow(m_joint, 0.0f, &matrix[i][0]);
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);

			const dFloat stopAccel = NewtonUserJointCalculateRowZeroAcceleration(m_joint);
			NewtonUserJointSetRowAcceleration(m_joint, stopAccel);
			NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
			NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);
		}

		TestControls(timestep);
	}

	void TestControls(dFloat timestep)
	{
		NewtonWorld* const world = NewtonBodyGetWorld(m_body0);
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(world);

		dFloat linearSpeed = 2.0f * (scene->GetKeyState('O') - scene->GetKeyState('P'));
		if (linearSpeed) {
			dMatrix matrix;
			NewtonBodyGetMatrix(m_body0, &matrix[0][0]);
			matrix = GetMatrix0() * matrix;

			dVector veloc(matrix[0].Scale(linearSpeed));
			SetVelocity(veloc, timestep);
		}

		dFloat angularSpeed = 6.0f * (scene->GetKeyState('K') - scene->GetKeyState('L'));
		if (angularSpeed) {
			dMatrix matrix;
			NewtonBodyGetMatrix(m_body0, &matrix[0][0]);
			matrix = GetMatrix0() * matrix;

			dVector omega(matrix[0].Scale(angularSpeed));
			SetOmega(omega, timestep);
		}
	}

	void SetVelocity(const dVector& veloc, dFloat timestep) 
	{
		dVector bodyVeloc(0.0f);
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;
		dFloat mass;

		// Get body mass
		NewtonBodyGetMass(m_body0, &mass, &Ixx, &Iyy, &Izz);

		// get body internal data
		NewtonBodyGetVelocity(m_body0, &bodyVeloc[0]);

		// calculate angular velocity error 
		dVector velocError(veloc - bodyVeloc);
		dFloat vMag2 = velocError.DotProduct3(velocError);
		if (vMag2 > dFloat (1.0e-4f)) {
			dVector dir (velocError.Normalize());

			// calculate impulse
			dVector linearImpulse(velocError.Scale (mass) + dir.Scale (m_linearFriction * timestep));
			//dVector linearImpulse(velocError.Scale (mass));

			// apply impulse to achieve desired velocity
			dVector angularImpulse(0.0f);
			NewtonBodyApplyImpulsePair(m_body0, &linearImpulse[0], &angularImpulse[0], timestep);
		}
	}

	void SetOmega(const dVector& omega, dFloat timestep)
	{
		dMatrix bodyInertia;
		dVector bodyOmega(0.0f);

		// get body internal data
		NewtonBodyGetOmega(m_body0, &bodyOmega[0]);
		NewtonBodyGetInertiaMatrix(m_body0, &bodyInertia[0][0]);

		// calculate angular velocity error 
		dVector omegaError(omega - bodyOmega);

		dFloat wMag2 = omegaError.DotProduct3(omegaError);
		if (wMag2 > dFloat (1.0e-4f)) {
			dVector dir (omegaError.Normalize());

			// calculate impulse
			dVector angularImpulse(bodyInertia.RotateVector(omegaError) + dir.Scale (m_angularFriction * timestep));

			// apply impulse to achieve desired omega
			dVector linearImpulse(0.0f);
			NewtonBodyApplyImpulsePair(m_body0, &linearImpulse[0], &angularImpulse[0], timestep);
		}
	}

	dFloat m_linearFriction;
	dFloat m_angularFriction;
};

class dFlexyPipeSpinner: public dCustomBallAndSocket
{
	public: 
	dFlexyPipeSpinner (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent)
		:dCustomBallAndSocket(pinAndPivotFrame, child, parent)
	{
		EnableTwist(false);
		EnableCone(false);
	}

	void ApplyTwistAction (const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
	{
		dFloat jacobian0[6];
		dFloat jacobian1[6];

		dVector pin0(matrix0.m_front);
		dVector pin1(matrix1.m_front.Scale(-1.0f));

		jacobian0[0] = 0.0f;
		jacobian0[1] = 0.0f;
		jacobian0[2] = 0.0f;
		jacobian0[3] = pin0.m_x;
		jacobian0[4] = pin0.m_y;
		jacobian0[5] = pin0.m_z;

		jacobian1[0] = 0.0f;
		jacobian1[1] = 0.0f;
		jacobian1[2] = 0.0f;
		jacobian1[3] = pin1.m_x;
		jacobian1[4] = pin1.m_y;
		jacobian1[5] = pin1.m_z;

		dVector omega0(0.0f);
		dVector omega1(0.0f);
		NewtonBodyGetOmega(m_body0, &omega0[0]);
		NewtonBodyGetOmega(m_body1, &omega1[0]);

		dFloat relOmega = omega0.DotProduct3(pin0) + omega1.DotProduct3(pin1);
		dFloat relAccel = -relOmega / timestep;
		NewtonUserJointAddGeneralRow(m_joint, jacobian0, jacobian1);
		NewtonUserJointSetRowAcceleration(m_joint, relAccel);
	}

	void ApplyElasticConeAction(const dMatrix& matrix0, const dMatrix& matrix1, dFloat timestep)
	{
		dFloat relaxation = 0.01f;
		dFloat spring = 1000.0f;
		dFloat damper = 50.0f;
		dFloat maxConeAngle = 45.0f * dDegreeToRad;

		dFloat cosAngleCos = matrix1.m_front.DotProduct3(matrix0.m_front);
		if (cosAngleCos >= dFloat(0.998f)) {
			// when angle is very small we use Cartesian approximation 
			dFloat angle0 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_up);
			NewtonUserJointAddAngularRow(m_joint, angle0, &matrix1.m_up[0]);
			NewtonUserJointSetRowMassIndependentSpringDamperAcceleration(m_joint, relaxation, spring, damper);
			
			dFloat angle1 = CalculateAngle(matrix0.m_front, matrix1.m_front, matrix1.m_right);
			NewtonUserJointAddAngularRow(m_joint, angle1, &matrix1.m_right[0]);
			NewtonUserJointSetRowMassIndependentSpringDamperAcceleration(m_joint, relaxation, spring, damper);
		} else {
			// angle is large enough that we calculable the actual cone angle
			dVector lateralDir(matrix1[0].CrossProduct(matrix0[0]));
			dAssert(lateralDir.DotProduct3(lateralDir) > 1.0e-6f);
			lateralDir = lateralDir.Normalize();
			dFloat coneAngle = dAcos(dClamp(matrix1.m_front.DotProduct3(matrix0.m_front), dFloat(-1.0f), dFloat(1.0f)));
			dMatrix coneRotation(dQuaternion(lateralDir, coneAngle), matrix1.m_posit);
			
			// flexible spring action happens alone the cone angle only
			if (coneAngle > maxConeAngle) {
				NewtonUserJointAddAngularRow(m_joint, maxConeAngle - coneAngle, &lateralDir[0]);
				NewtonUserJointSetRowMaximumFriction(m_joint, 0.0f);

			} else {
				// apply spring damper action
				NewtonUserJointAddAngularRow(m_joint, -coneAngle, &lateralDir[0]);
				NewtonUserJointSetRowMassIndependentSpringDamperAcceleration(m_joint, relaxation, spring, damper);	
			}
		}
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		dMatrix matrix0;
		dMatrix matrix1;
		dCustomBallAndSocket::SubmitConstraints(timestep, threadIndex);

		CalculateGlobalMatrix(matrix0, matrix1);
		ApplyTwistAction (matrix0, matrix1, timestep);
		ApplyElasticConeAction(matrix0, matrix1, timestep);
	}
};

class MyPathFollow : public dCustomPathFollow
{
	public:
	MyPathFollow(const dMatrix& pinAndPivotFrame, NewtonBody* const body, NewtonBody* const pathBody)
		:dCustomPathFollow(pinAndPivotFrame, body, pathBody)
	{
	}

	void GetPointAndTangentAtLocation(const dVector& location, dVector& positOut, dVector& tangentOut) const
	{
		DemoEntity* const pathEntity = (DemoEntity*)NewtonBodyGetUserData(GetBody1());
		DemoBezierCurve* const mesh = (DemoBezierCurve*)pathEntity->GetMesh();
		const dBezierSpline& spline = mesh->m_curve;

		dMatrix matrix;
		NewtonBodyGetMatrix(GetBody1(), &matrix[0][0]);

		dVector p(matrix.UntransformVector(location));
		dBigVector point;
		dFloat64 knot = spline.FindClosestKnot(point, p, 4);
		dBigVector tangent(spline.CurveDerivative(knot));
		tangent = tangent.Scale(1.0 / dSqrt(tangent.DotProduct3(tangent)));
		positOut = matrix.TransformVector(point);
		tangentOut = tangent;
	}
};

static NewtonBody* CreateBox (DemoEntityManager* const scene, const dVector& location, const dVector& size, const char* const textName = "smilli.tga")
{
    NewtonWorld* const world = scene->GetNewton();
    int materialID =  NewtonMaterialGetDefaultGroupID (world);
    NewtonCollision* const collision = CreateConvexCollision (world, dGetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);
   	DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), collision, textName, textName, textName);

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
	DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");

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
	DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");

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
    DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");

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
    DemoMesh* const geometry = new DemoMesh("primitive", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");

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
	NewtonBodySetGyroscopicTorque(box0, 1);
	NewtonBodySetMassMatrix(base, 0.0f, 0.0f, 0.0f, 0.0f);
	dMatrix pinMatrix(dGrammSchmidt(dVector(0.0f, -1.0f, 0.0f, 0.0f)));

	// connect first box to the world
	dMatrix matrix;
	NewtonBodyGetMatrix(box0, &matrix[0][0]);
	pinMatrix.m_posit = matrix.m_posit + dVector(0.0f, size.m_y, 0.0f, 0.0f);

	// tilt the cone limit
//	dMatrix tiltConeMatrix (dYawMatrix(-30.0f * dDegreeToRad) * pinMatrix);
	dMatrix tiltConeMatrix (dYawMatrix(0.0f * dDegreeToRad) * pinMatrix);
	dCustomBallAndSocket* const joint0 = new dCustomBallAndSocket(pinMatrix, tiltConeMatrix, box0, base);
	joint0->EnableCone(true);
	joint0->EnableTwist(true);
	joint0->SetConeLimits (60.0f * dDegreeToRad);
//	joint0->SetTwistLimits (-1000.0f * dDegreeToRad, 1000.0f * dDegreeToRad);
	joint0->SetTwistLimits (0.0f * dDegreeToRad, 0.0f * dDegreeToRad);

	// connect first box1 to box0 the world
//	NewtonBody* const box1 = CreateCapule(scene, origin + dVector(0.0f, 5.0f - size.m_y * 2.0f, 0.0f, 0.0f), size);
//	NewtonBodySetGyroscopicTorque(box1, 1);
//	NewtonBodyGetMatrix(box1, &matrix[0][0]);
//	pinMatrix.m_posit = matrix.m_posit + dVector(0.0f, size.m_y, 0.0f, 0.0f);
//	dCustomBallAndSocket* const joint1 = new dCustomBallAndSocket(pinMatrix, box1, box0);
//	joint1->EnableCone(true);
//	joint1->SetConeLimits(30.0f * dDegreeToRad);
//	joint1->EnableTwist(true);
//	joint1->SetTwistLimits(-30.0f * dDegreeToRad, 30.0f * dDegreeToRad);
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

//	const dFloat yawLimit = 120.0f * dDegreeToRad;
	const dFloat yawLimit = 60.0f * dDegreeToRad;
//	const dFloat rollLimit = 80.0f * dDegreeToRad;
	const dFloat rollLimit = 60.0f * dDegreeToRad;
//	const dFloat pitchLimit = 80.0f * dDegreeToRad;
	const dFloat pitchLimit = 60.0f * dDegreeToRad;

//const dFloat yawLimit = 0.0f * dDegreeToRad;
//const dFloat rollLimit = 0.0f * dDegreeToRad;
//const dFloat pitchLimit = 0.0f * dDegreeToRad;

	NewtonBodySetMassMatrix(base, 0.0f, 0.0f, 0.0f, 0.0f);
	dMatrix pinMatrix (dGrammSchmidt (dVector (0.0f, -1.0f, 0.0f, 0.0f)));

	// connect first box to the world
	dMatrix matrix0;
	NewtonBodyGetMatrix (box0, & matrix0[0][0]);
	pinMatrix.m_posit = matrix0.m_posit + dVector (0.0f, size.m_y, 0.0f, 0.0f);
	dCustomSixdof* const joint0 = new dCustomSixdof (pinMatrix, box0, base);
	joint0->SetYawLimits (-yawLimit, yawLimit);
	joint0->SetRollLimits(-rollLimit, rollLimit);
	joint0->SetPitchLimits(-pitchLimit, pitchLimit);
	//joint0->SetPitchLimits(-0, 0);
	//joint0->DisableRotationX ();

	// link the two boxes
//	dMatrix matrix1;
//	NewtonBody* const box1 = CreateCapule(scene, origin + dVector(0.0f, 5.0f - size.m_y * 2.0f, 0.0f, 0.0f), size);
//	NewtonBodyGetMatrix (box1, &matrix1[0][0]);
//	pinMatrix.m_posit = (matrix0.m_posit + matrix1.m_posit).Scale (0.5f);
//	dCustomSixdof* const joint1 = new dCustomSixdof (pinMatrix, box1, box0);
//	joint1->SetYawLimits(-yawLimit, yawLimit);
//	joint1->SetRollLimits(-rollLimit, rollLimit);
//	joint1->SetPitchLimits(-pitchLimit, pitchLimit);
	//joint1->DisableRotationX();
}


static void AddDoubleHinge(DemoEntityManager* const scene, const dVector& origin)
{
	dVector size(1.0f, 1.0f, 1.0f);

	NewtonBody* const box0 = CreateBox(scene, origin + dVector(0.0f, 4.0f, 0.0f, 0.0f), dVector(0.25f, 0.25f, 5.0f, 0.0f));
	NewtonBodySetMassMatrix(box0, 0.0f, 0.0f, 0.0f, 0.0f);

	{
		dMatrix matrix;
		dVector damp(0.0f);
		dVector omega1(0.0f, 2.0f, 50.0f, 0.0f);
		NewtonBody* const box1 = CreateWheel(scene, origin + dVector(0.0f, 4.0f, 2.5f, 0.0f), 1.0f, 0.5f);
		DemoEntity* const boxEntity1 = ((DemoEntity*) NewtonBodyGetUserData(box1));

		NewtonBodySetGyroscopicTorque(box1, 1);
		NewtonBodyGetMatrix(box1, &matrix[0][0]);
		matrix = dYawMatrix (dPi * 0.5f) * matrix; 
		NewtonBodySetMatrix(box1, &matrix[0][0]);
	
		boxEntity1->ResetMatrix (*scene, matrix);
		NewtonBodySetOmega(box1, &omega1[0]);
		NewtonBodySetLinearDamping(box1, 0.0f);
		NewtonBodySetAngularDamping(box1, &damp[0]);
		
		//	// link the two boxes
		NewtonBodyGetMatrix(box1, &matrix[0][0]);
		dCustomDoubleHinge* const joint = new dCustomDoubleHinge(matrix, box1, box0);
		joint->EnableLimits(false);
		joint->SetLimits(-30.0f * dPi, 10.0f * dPi);
	}

	{
		dMatrix matrix;
		dVector damp(0.0f);
		dVector omega2 (0.0f, 10.0f, 100.0f, 0.0f);
		NewtonBody* const box2 = CreateWheel(scene, origin + dVector(0.0f, 4.0f, -2.5f, 0.0f), 1.0f, 0.5f);
		DemoEntity* const boxEntity2 = ((DemoEntity*) NewtonBodyGetUserData(box2));
		NewtonBodySetGyroscopicTorque(box2, 1);
		NewtonBodyGetMatrix(box2, &matrix[0][0]);
		matrix = dYawMatrix(dPi * 0.5f) * matrix;
		NewtonBodySetMatrix(box2, &matrix[0][0]);
		boxEntity2->ResetMatrix (*scene, matrix);
		NewtonBodySetOmega(box2, &omega2[0]);
		NewtonBodySetLinearDamping(box2, 0.0f);
		NewtonBodySetAngularDamping(box2, &damp[0]);

		NewtonBodyGetMatrix(box2, &matrix[0][0]);
		dCustomDoubleHinge* const joint = new dCustomDoubleHinge(matrix, box2, box0);
		joint->EnableLimits1(true);
		joint->SetLimits1 (-10.0f * dPi, 15.0f * dPi);
	}

	{
		dMatrix matrix;
		dVector damp(0.0f);
		NewtonBody* const box2 = CreateWheel(scene, origin + dVector(0.0f, 4.0f, -0.85f, 0.0f), 1.0f, 0.5f);
		DemoEntity* const boxEntity2 = ((DemoEntity*)NewtonBodyGetUserData(box2));
		NewtonBodySetGyroscopicTorque(box2, 1);
		NewtonBodyGetMatrix(box2, &matrix[0][0]);
		matrix = dYawMatrix(dPi * 0.5f) * matrix;
		NewtonBodySetMatrix(box2, &matrix[0][0]);
		boxEntity2->ResetMatrix(*scene, matrix);
		NewtonBodySetLinearDamping(box2, 0.0f);
		NewtonBodySetAngularDamping(box2, &damp[0]);

		NewtonBodyGetMatrix(box2, &matrix[0][0]);
		dCustomDoubleHinge* const joint = new dCustomDoubleHinge(matrix, box2, box0);
		joint->SetMassIndependentSpringDamper(true, 0.7f, 80.0f, 0.0f);
	}

	{
		dMatrix matrix;
		dVector damp(0.0f);
		NewtonBody* const box2 = CreateWheel(scene, origin + dVector(0.0f, 4.0f, 0.84f, 0.0f), 1.0f, 0.5f);
		DemoEntity* const boxEntity2 = ((DemoEntity*)NewtonBodyGetUserData(box2));
		NewtonBodySetGyroscopicTorque(box2, 1);
		NewtonBodyGetMatrix(box2, &matrix[0][0]);
		matrix = dYawMatrix(dPi * 0.5f) * matrix;
		NewtonBodySetMatrix(box2, &matrix[0][0]);
		boxEntity2->ResetMatrix(*scene, matrix);
		NewtonBodySetLinearDamping(box2, 0.0f);
		NewtonBodySetAngularDamping(box2, &damp[0]);

		NewtonBodyGetMatrix(box2, &matrix[0][0]);
		dCustomDoubleHinge* const joint = new dCustomDoubleHinge(matrix, box2, box0);
		joint->SetFriction(10.0f);
		joint->SetFriction1(10.0f);
	}
}

static void AddSliderBug(DemoEntityManager* const scene, const dVector& origin)
{
	NewtonBody* const outerBody = CreateBox(scene, origin + dVector(0.0f, 4.0f, 0.0f, 0.0f), dVector(2.0f, 1.5f, 1.5f, 0.0f).Scale (0.25f));
	NewtonBody* const cylinder = CreateCylinder(scene, origin + dVector(0.0f, 4.0f, 0.0f, 0.0f), 1.0f * 0.25f, 4.0f * 0.25f);

	// connect the bodies by a Slider joint
	dMatrix matrix;
	NewtonBodyGetMatrix(cylinder, &matrix[0][0]);
	dCustomCorkScrew* const slider = new dCustomCorkScrew(matrix, outerBody, cylinder);
//	dCustomSlider* const slider = new dCustomSlider(matrix, outerBody, cylinder);
//	dCustomSlidingContact* const slider = new dCustomSlidingContact(matrix, outerBody, cylinder);

	// set limit on second axis
	slider->EnableLimits(true);
	slider->SetLimits(-0.5f, 0.5f);
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

static void AddHingeMotor(DemoEntityManager* const scene, const dVector& origin)
{
	NewtonBody* parent = CreateBox(scene, origin + dVector(-0.8f, 6.0f, 0.0f, 0.0f), dVector(0.2f, 0.2f, 0.2f));
	NewtonBodySetMassMatrix(parent, 0.0f, 0.0f, 0.0f, 0.0f);

	dMatrix matrix;
	NewtonBodyGetMatrix(parent, &matrix[0][0]);
	NewtonBody* child = NULL;
	dVector size(0.25f, 0.25f, 1.5f, .0f);

	matrix.m_posit.m_x += 0.22f;
	matrix.m_posit.m_z += size.m_z / 2.0f - 0.1f;

	dFloat rpm = 3.0f;
	int count = 10;

	for (int i = 0; i < count; i++) {
		child = CreateBox(scene, matrix.m_posit, size, "wood_1.tga");

		NewtonBodyGetMatrix(child, &matrix[0][0]);
		dMatrix hingeMatrix(matrix);
		hingeMatrix.m_posit.m_z -= (size.m_z / 2.0f - 0.1f);
		dCustomHinge* const hinge = new dCustomHinge(hingeMatrix, child, parent);

		hinge->EnableMotor(true, rpm + (i&1 ? -2.0f : 2.0f));
		hinge->SetFriction(1.0e4f);

		parent = child;
		matrix.m_posit.m_x += size.m_x;
		matrix.m_posit.m_z += size.m_z - 0.2f;
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
	hinge->SetMassIndependentSpringDamper(true, 0.8f, 300.0f, 1.0f);
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

static void AddPathFollow(DemoEntityManager* const scene, const dVector& origin)
{
	// create a Bezier Spline path for AI car to drive
	NewtonBody* const pathBody = CreateBox(scene, origin, dVector(4.0f, 0.25f, 0.25f));
	NewtonBodySetMassMatrix(pathBody, 0.0f, 0.0f, 0.0f, 0.0f);
	DemoEntity* const rollerCosterPath = (DemoEntity*)NewtonBodyGetUserData(pathBody);

	dBezierSpline spline;
	dFloat64 knots[] = { 0.0f, 1.0f / 5.0f, 2.0f / 5.0f, 3.0f / 5.0f, 4.0f / 5.0f, 1.0f };

	dBigVector control[] =
	{
		dBigVector(100.0f - 100.0f, 20.0f, 200.0f - 250.0f, 1.0f),
		dBigVector(150.0f - 100.0f, 10.0f, 150.0f - 250.0f, 1.0f),
		dBigVector(175.0f - 100.0f, 30.0f, 250.0f - 250.0f, 1.0f),
		dBigVector(200.0f - 100.0f, 70.0f, 250.0f - 250.0f, 1.0f),
		dBigVector(215.0f - 100.0f, 20.0f, 250.0f - 250.0f, 1.0f),
		dBigVector(150.0f - 100.0f, 50.0f, 350.0f - 250.0f, 1.0f),
		dBigVector(50.0f - 100.0f, 30.0f, 250.0f - 250.0f, 1.0f),
		dBigVector(100.0f - 100.0f, 20.0f, 200.0f - 250.0f, 1.0f),
	};

	spline.CreateFromKnotVectorAndControlPoints(3, sizeof(knots) / sizeof(knots[0]), knots, control);

	DemoBezierCurve* const mesh = new DemoBezierCurve(spline);
	rollerCosterPath->SetMesh(mesh, dGetIdentityMatrix());

	mesh->SetVisible(true);
	mesh->SetRenderResolution(500);
	mesh->Release();

	const int count = 32;
	//	const int count = 1;
	NewtonBody* bodies[count];

	dBigVector point0;
	dVector positions[count + 1];
	dFloat64 knot = spline.FindClosestKnot(point0, dBigVector(dVector(100.0f - 100.0f, 20.0f, 200.0f - 250.0f, 0.0f)), 4);
	positions[0] = point0;
	for (int i = 0; i < count; i++) {
		dBigVector point1;
		dBigVector tangent(spline.CurveDerivative(knot));
		tangent = tangent.Scale(1.0 / dSqrt(tangent.DotProduct3(tangent)));
		knot = spline.FindClosestKnot(point1, dBigVector(point0 + tangent.Scale(2.0f)), 4);
		point0 = point1;
		positions[i + 1] = point1;
	}

	dMatrix pathBodyMatrix;
	NewtonBodyGetMatrix(pathBody, &pathBodyMatrix[0][0]);

	dFloat attachmentOffset = 0.8f;
	for (int i = 0; i < count; i++) {
		dMatrix matrix;
		bodies[i] = CreateWheel(scene, dVector(0.0f, 0.0f, 0.0f, 0.0f), 1.0f, 0.5f);
		NewtonBodySetLinearDamping(bodies[i], 0.0f);
		NewtonBody* const box = bodies[i];
		NewtonBodyGetMatrix(box, &matrix[0][0]);

		dVector location0(positions[i + 0].m_x, positions[i + 0].m_y, positions[i + 0].m_z, 0.0);
		dVector location1(positions[i + 1].m_x, positions[i + 1].m_y, positions[i + 1].m_z, 0.0);

		location0 = pathBodyMatrix.TransformVector(location0);
		location1 = pathBodyMatrix.TransformVector(location1);

		dVector dir(location1 - location0);
		dir.m_w = 0.0f;
		matrix.m_front = dir.Scale(1.0f / dSqrt(dir.DotProduct3(dir)));
		matrix.m_right = matrix.m_front.CrossProduct(matrix.m_up);
		matrix.m_right = matrix.m_right.Scale(1.0f / dSqrt(matrix.m_right.DotProduct3(matrix.m_right)));
		matrix.m_up = matrix.m_right.CrossProduct(matrix.m_front);
		matrix.m_posit = pathBodyMatrix.TransformVector(dVector(positions[i].m_x, positions[i].m_y - attachmentOffset, positions[i].m_z, 1.0));
		dMatrix matrix1(dYawMatrix(0.5f * dPi) * matrix);

		NewtonBodySetMatrix(box, &matrix1[0][0]);
		DemoEntity* const ent = (DemoEntity*)NewtonBodyGetUserData(box);
		ent->ResetMatrix(*scene, matrix1);

		matrix.m_posit = pathBodyMatrix.TransformVector(dVector(positions[i].m_x, positions[i].m_y, positions[i].m_z, 1.0));
		new MyPathFollow(matrix, box, pathBody);

		dVector veloc(dir.Scale(20.0f));
		NewtonBodySetVelocity(box, &veloc[0]);
	}

	for (int i = 1; i < count; i++) {
		NewtonBody* const box0 = bodies[i - 1];
		NewtonBody* const box1 = bodies[i];

		dMatrix matrix0;
		dMatrix matrix1;
		NewtonBodyGetMatrix(box0, &matrix0[0][0]);
		NewtonBodyGetMatrix(box1, &matrix1[0][0]);

		matrix0.m_posit.m_y += attachmentOffset;
		matrix1.m_posit.m_y += attachmentOffset;

		//new CustomDistanceRope (matrix1.m_posit, matrix0.m_posit, box1, box0);
		new dCustomFixDistance(matrix1.m_posit, matrix0.m_posit, box1, box0);
	}

	void* const aggregate = NewtonCollisionAggregateCreate(scene->GetNewton());
	for (int i = 0; i < count; i++) {
		NewtonCollisionAggregateAddBody(aggregate, bodies[i]);
	}
	NewtonCollisionAggregateSetSelfCollision(aggregate, false);
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

	NewtonBodyGetMatrix(box3, &matrix[0][0]);
	matrix = dYawMatrix(dPi * 0.5f) * matrix;
	NewtonBodySetMatrix(box3, &matrix[0][0]);
	((DemoEntity*)NewtonBodyGetUserData(box3))->ResetMatrix(*scene, matrix);

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
	dCustomDoubleHinge* const diff = new dCustomDoubleHinge(matrix3, box3, box0);

	// connect right differential
	new dCustomDifferentialGear(2.0f, matrix1.m_front, box1, 1.0f, diff);

	// connect left differential
	new dCustomDifferentialGear(2.0f, matrix2.m_front, box2, -1.0f, diff);

	dVector damp(0.0f);
	dVector omega(0.0f, 2.0f, 0.0f, 0.0f);
	NewtonBodySetOmega(box3, &omega[0]);
	NewtonBodySetAngularDamping(box1, &damp[0]);
	NewtonBodySetAngularDamping(box2, &damp[0]);
	NewtonBodySetAngularDamping(box3, &damp[0]);
}

void AddFlexyPipe(DemoEntityManager* const scene, const dVector& origin)
{
	dArray<NewtonBody*> bodies;

	int count = 50;
//int count = 10;
	NewtonWorld* const world = scene->GetNewton();

	// set the friction low to emulate motion on some fluid 
	int materialID = NewtonMaterialGetDefaultGroupID(world);
	NewtonMaterialSetDefaultFriction(world, materialID, materialID, 0.2f, 0.2f);

	dVector capsuleSize(0.25f, 0.5f, 0.25f);
	dMatrix aligment(dYawMatrix(90.0f * dDegreeToRad));
	NewtonCollision* const capsuleShape = CreateConvexCollision(world, &aligment[0][0], capsuleSize, _CAPSULE_PRIMITIVE, 0);
	DemoMesh* const capsuleGeometry = new DemoMesh("primitive", scene->GetShaderCache(), capsuleShape, "smilli.tga", "smilli.tga", "smilli.tga");

	dFloat mass = 1.0f;
	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_y += 1.0f;
	matrix.m_posit.m_w = 1.0f;
	for (int i = 0; i < count; i++) {
		bodies[i] = CreateSimpleSolid(scene, capsuleGeometry, mass, matrix, capsuleShape, 0);
		matrix.m_posit.m_z += capsuleSize.m_y;
	}
	capsuleGeometry->Release();
	NewtonDestroyCollision(capsuleShape);

	// add the axial spinner joints
	for (int i = 1; i < count; i++) {
		dMatrix matrix0;
		dMatrix matrix1;
		NewtonBodyGetMatrix(bodies[i - 1], &matrix0[0][0]);
		NewtonBodyGetMatrix(bodies[i], &matrix1[0][0]);
		dMatrix jointMatrix(dGrammSchmidt(matrix1.m_posit - matrix0.m_posit));
		jointMatrix.m_posit = (matrix0.m_posit + matrix1.m_posit).Scale(0.5f);
		new dFlexyPipeSpinner (jointMatrix, bodies[i], bodies[i - 1]);
	}

	// add a cylinder to add as a controller handle 
	dMatrix matrix0;
	dMatrix matrix1;
	NewtonBodyGetMatrix(bodies[0], &matrix0[0][0]);
	NewtonBodyGetMatrix(bodies[1], &matrix1[0][0]);

	dMatrix handleMatrix (matrix0);
	handleMatrix.m_posit -= (matrix1.m_posit - matrix0.m_posit);

	dVector cylinderSize(0.5f, 0.75f, 0.5f);
	NewtonCollision* const cylinderShape = CreateConvexCollision(world, &aligment[0][0], cylinderSize, _CYLINDER_PRIMITIVE, 0);
	DemoMesh* const cylinderGeometry = new DemoMesh("primitive", scene->GetShaderCache(), cylinderShape, "smilli.tga", "smilli.tga", "smilli.tga");

	NewtonBody* const cylinderHandle = CreateSimpleSolid(scene, cylinderGeometry, mass * 30.0f, handleMatrix, cylinderShape, 0);

	// connect the pipe to the handle
	dVector dir(matrix1.m_posit - matrix0.m_posit);
	dMatrix jointMatrix(dGrammSchmidt(dir));
	jointMatrix.m_posit = (matrix0.m_posit + matrix1.m_posit).Scale (0.5f) - dir;

	dCustomBallAndSocket* const joint = new dCustomBallAndSocket(jointMatrix, bodies[0], cylinderHandle);
	joint->EnableTwist(true);
	joint->SetTwistLimits(0.0f, 0.0f);
	joint->SetTwistFriction(1.0e20f);
	joint->EnableCone(true);
	joint->SetConeLimits(0.0f);
	joint->SetConeFriction(1.0e20f);
	//joint->SetConeStiffness(0.95f);

	cylinderGeometry->Release();
	NewtonDestroyCollision(cylinderShape);

	// attach the handle to the world
	new dFlexyPipeHandle(cylinderHandle, dir);
}

void StandardJoints (DemoEntityManager* const scene)
{
    scene->CreateSkyBox();

    // customize the scene after loading
    // set a user friction variable in the body for variable friction demos
    // later this will be done using LUA script
    dMatrix offsetMatrix (dGetIdentityMatrix());

    CreateLevelMesh (scene, "flatPlane.ngd", true);

    dVector location (0.0f);
    dVector size (1.5f, 2.0f, 2.0f, 0.0f);

//	joints still with problems
//	Add6DOF (scene, dVector (-20.0f, 0.0f, -25.0f));
//	AddSliderBug(scene, dVector(-20.0f, 0.0f, 17.0f));
//	AddDoubleHinge(scene, dVector(-20.0f, 0.0f, 17.0f));
//	AddPathFollow(scene, dVector(20.0f, 0.0f, 0.0f));
//	AddDifferential(scene, dVector(-20.0f, 0.0f, 33.0f));
//	AddHingeSpringDamper (scene, dVector (dVector (-20.0f, 0.0f, 5.0f)));
//	AddHinge(scene, dVector(-20.0f, 0.0f, 0.0f));
//	AddPathFollow (scene, dVector (20.0f, 0.0f, 0.0f));
//	AddFlexyPipe(scene, dVector(-20.0f, 0.0f, 0.0f));

#if 1
	Add6DOF (scene, dVector (-20.0f, 0.0f, -25.0f));
	AddDistance (scene, dVector (-20.0f, 0.0f, -20.0f));
	AddLimitedBallAndSocket (scene, dVector (-20.0f, 0.0f, -15.0f));
	AddBallAndSockectWithFriction (scene, dVector (-20.0f, 0.0f, -10.0f));
	AddFixDistance(scene, dVector(-20.0f, 0.0f, -5.0f));
	AddHinge (scene, dVector (-20.0f, 0.0f, 0.0f));
	AddHingeMotor(scene, dVector(0.0f, 0.0f, -20.0f));
	AddHingeSpringDamper (scene, dVector (dVector (-20.0f, 0.0f, 5.0f)));
	AddSlider (scene, dVector (-20.0f, 0.0f, 7.0f));
	AddSliderSpringDamper (scene, dVector (dVector (-20.0f, 0.0f, 9.0f)));
	AddCylindrical (scene, dVector (-20.0f, 0.0f, 11.0f));
	AddSlidingContact (scene, dVector (-20.0f, 0.0f, 13.0f));
	AddDoubleHinge(scene, dVector (-20.0f, 0.0f, 17.0f));
	AddGear (scene, dVector (-20.0f, 0.0f, 22.0f));
	AddPulley (scene, dVector (-20.0f, 0.0f, 25.0f));
	AddGearAndRack (scene, dVector (-20.0f, 0.0f, 29.0f));
	AddDifferential(scene, dVector(-20.0f, 0.0f, 35.0f));
	AddPathFollow (scene, dVector (20.0f, 0.0f, 0.0f));
	//AddFlexyPipe(scene, dVector(-20.0f, 0.0f, 0.0f));
#endif

    // place camera into position
    dMatrix camMatrix (dGetIdentityMatrix());
    dQuaternion rot (camMatrix);
	dVector origin (-50.0f, 7.0f, 0.0f, 0.0f);
//	dVector origin (-30.0f, 7.0f, 30.0f, 0.0f);
    scene->SetCameraMatrix(rot, origin);

//	NewtonSerializeToFile(scene->GetNewton(), "xxx.bin", NULL, NULL);
}


