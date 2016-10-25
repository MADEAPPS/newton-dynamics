/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DemoMesh.h"
#include "OpenGlUtil.h"
#include <CustomGear.h>
#include <Custom6DOF.h>
#include <CustomHinge.h>
#include <CustomSlider.h>
#include <CustomPulley.h>
#include <dBezierSpline.h>
#include <CustomCorkScrew.h>
#include <CustomPathFollow.h>
#include <CustomBallAndSocket.h>
#include <CustomRackAndPinion.h>
#include <CustomSlidingContact.h>

// optionally uncomment this for hard joint simulations 
#define _USE_HARD_JOINTS


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

static NewtonBody* CreateCapule (DemoEntityManager* const scene, const dVector& location, const dVector& size)
{
	NewtonWorld* const world = scene->GetNewton();
	int materialID =  NewtonMaterialGetDefaultGroupID (world);
	dMatrix uprightAligment (dRollMatrix(3.141592f * 90.0f / 180.0f));
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
    dVector size (radius, height, 0.0f, 0.0f);
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
    dVector size (radius, height, 0.0f, 0.0f);
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
	NewtonBody* const box0 = CreateBox(scene, origin + dVector (0.0f,  5.0f + size.m_y + 0.25f, 0.0f, 0.0f), size.Scale (0.2f));
	NewtonBody* const box1 = CreateCapule (scene, origin + dVector (0.0f,  5.0f, 0.0f, 0.0f), size);
	NewtonBody* const box2 = CreateCapule (scene, origin + dVector (0.0f,  5.0 - size.m_y * 4.0f, 0.0f, 0.0f), size);

	dMatrix pinMatrix (dGrammSchmidt (dVector (0.0f, -1.0f, 0.0f, 0.0f)));
	NewtonBodySetMassMatrix(box0, 0.0f, 0.0f, 0.0f, 0.0f);

	// connect first box to the world
	dMatrix matrix0;
	NewtonBodyGetMatrix (box1, &matrix0[0][0]);
	pinMatrix.m_posit = matrix0.m_posit + dVector (0.0f, size.m_y, 0.0f, 0.0f);
	new CustomBallAndSocket (pinMatrix, box1, box0);

	// link the two boxes with a distance joint
	dMatrix matrix1;
	NewtonBodyGetMatrix (box2, &matrix1[0][0]);

	// get the origins
	dVector pivot0 (matrix0.m_posit - dVector (0.0f, size.m_y, 0.0f, 0.0f));
	dVector pivot1 (matrix1.m_posit + dVector (0.0f, size.m_y, 0.0f, 0.0f));

	// connect bodies at a corner
	new CustomPointToPoint (pivot1, pivot0, box2, box1);
	
#ifdef _USE_HARD_JOINTS
	NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate(scene->GetNewton(), box0, NULL);
	NewtonSkeletonContainerAttachBone(skeleton, box1, box0);
	NewtonSkeletonContainerAttachBone(skeleton, box2, box1);
	NewtonSkeletonContainerFinalize(skeleton);
#endif
}

static void AddLimitedBallAndSocket (DemoEntityManager* const scene, const dVector& origin)
{
	dVector size(1.0f, 1.0f, 1.0f);
	NewtonBody* const base = CreateBox(scene, origin + dVector (0.0f,  5.0f + size.m_y + 0.25f, 0.0f, 0.0f), size.Scale (0.2f));
	NewtonBody* const box0 = CreateCapule(scene, origin + dVector(0.0f, 5.0f, 0.0f, 0.0f), size);
	NewtonBody* const box1 = CreateCapule(scene, origin + dVector(0.0f, 5.0 - size.m_y * 2.0f, 0.0f, 0.0f), size);
	NewtonBody* const box2 = CreateCapule(scene, origin + dVector(0.0f, 5.0 - size.m_y * 4.0f, 0.0f, 0.0f), size);

	NewtonBodySetMassMatrix(base, 0.0f, 0.0f, 0.0f, 0.0f);
	dMatrix pinMatrix(dGrammSchmidt(dVector(0.0f, -1.0f, 0.0f, 0.0f)));

	// connect first box to the world
	dMatrix matrix;
	NewtonBodyGetMatrix(box0, &matrix[0][0]);
	pinMatrix.m_posit = matrix.m_posit + dVector(0.0f, size.m_y, 0.0f, 0.0f);

	CustomLimitBallAndSocket* const joint0 = new CustomLimitBallAndSocket(pinMatrix, box0, base);
	joint0->SetConeAngle (30.0f * 3.141592f / 180.0f);
	joint0->SetTwistAngle (-30.0f * 3.141592f / 180.0f, 30.0f * 3.141592f / 180.0f);

	// connect first box1 to box0 the world
	NewtonBodyGetMatrix(box1, &matrix[0][0]);
	pinMatrix.m_posit = matrix.m_posit + dVector(0.0f, size.m_y, 0.0f, 0.0f);

	CustomLimitBallAndSocket* const joint1 = new CustomLimitBallAndSocket(pinMatrix, box1, box0);
	joint1->SetConeAngle(30.0f * 3.141592f / 180.0f);
	joint1->SetTwistAngle(-30.0f * 3.141592f / 180.0f, 30.0f * 3.141592f / 180.0f);

	// connect first box2 to box1 the world
	NewtonBodyGetMatrix(box2, &matrix[0][0]);
	pinMatrix.m_posit = matrix.m_posit + dVector(0.0f, size.m_y, 0.0f, 0.0f);

	CustomLimitBallAndSocket* const joint2 = new CustomLimitBallAndSocket(pinMatrix, box2, box1);
	joint2->SetConeAngle(30.0f * 3.141592f / 180.0f);
	joint2->SetTwistAngle(-30.0f * 3.141592f / 180.0f, 30.0f * 3.141592f / 180.0f);

#ifdef _USE_HARD_JOINTS
	NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate(scene->GetNewton(), base, NULL);
	NewtonSkeletonContainerAttachBone(skeleton, box0, base);
	NewtonSkeletonContainerAttachBone(skeleton, box1, box0);
	NewtonSkeletonContainerAttachBone(skeleton, box2, box1);
	NewtonSkeletonContainerFinalize(skeleton);
#endif
}


static void AddBallAndSockectWithFriction (DemoEntityManager* const scene, const dVector& origin)
{
	dVector size (1.0f, 1.0f, 1.0f);
	NewtonBody* const base = CreateBox(scene, origin + dVector (0.0f,  5.0f + size.m_y + 0.25f, 0.0f, 0.0f), size.Scale (0.2f));
	NewtonBody* const box0 = CreateCapule (scene, origin + dVector (0.0f,  5.0f, 0.0f, 0.0f), size);
	NewtonBody* const box1 = CreateCapule (scene, origin + dVector (0.0f,  5.0 - size.m_y * 2.0f, 0.0f, 0.0f), size);

	NewtonBodySetMassMatrix(base, 0.0f, 0.0f, 0.0f, 0.0f);
	dMatrix pinMatrix (dGrammSchmidt (dVector (0.0f, -1.0f, 0.0f, 0.0f)));

	// connect first box to the world
	dMatrix matrix0;
	NewtonBodyGetMatrix (box0, &matrix0[0][0]);
	pinMatrix.m_posit = matrix0.m_posit + dVector (0.0f, size.m_y, 0.0f, 0.0f);
	new CustomBallAndSocketWithFriction (pinMatrix, box0, base, 20.0f);

	// link the two boxes
	dMatrix matrix1;
	NewtonBodyGetMatrix (box1, & matrix1[0][0]);
	pinMatrix.m_posit = (matrix0.m_posit + matrix1.m_posit).Scale (0.5f);
	new CustomBallAndSocketWithFriction (pinMatrix, box1, box0, 10.0f);

#ifdef _USE_HARD_JOINTS
	NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate(scene->GetNewton(), base, NULL);
	NewtonSkeletonContainerAttachBone(skeleton, box0, base);
	NewtonSkeletonContainerAttachBone(skeleton, box1, box0);
	NewtonSkeletonContainerFinalize(skeleton);
#endif
}

static void Add6DOF (DemoEntityManager* const scene, const dVector& origin)
{
	dVector size (1.0f, 1.0f, 1.0f);
	NewtonBody* const base = CreateBox(scene, origin + dVector (0.0f,  5.0f + size.m_y + 0.25f, 0.0f, 0.0f), size.Scale (0.2f));
	NewtonBody* const box0 = CreateCapule (scene, origin + dVector (0.0f,  5.0f, 0.0f, 0.0f), size);
	NewtonBody* const box1 = CreateCapule (scene, origin + dVector (0.0f,  5.0 - size.m_y * 2.0f, 0.0f, 0.0f), size);

	const dFloat angle = 60.0f * 3.1415592f / 180.0f;
	NewtonBodySetMassMatrix(base, 0.0f, 0.0f, 0.0f, 0.0f);
	dMatrix pinMatrix (dGrammSchmidt (dVector (0.0f, -1.0f, 0.0f, 0.0f)));

	// connect first box to the world
	dMatrix matrix0;
	NewtonBodyGetMatrix (box0, & matrix0[0][0]);
	pinMatrix.m_posit = matrix0.m_posit + dVector (0.0f, size.m_y, 0.0f, 0.0f);
	Custom6DOF* const joint0 = new Custom6DOF (pinMatrix, pinMatrix, box0, base);
	joint0->SetAngularLimits (dVector (-angle, -angle, -angle, 0.0f), dVector (angle, angle, angle, 0.0f));

	// link the two boxes
	dMatrix matrix1;
	NewtonBodyGetMatrix (box1, &matrix1[0][0]);
	pinMatrix.m_posit = (matrix0.m_posit + matrix1.m_posit).Scale (0.5f);
	Custom6DOF* const joint1 = new Custom6DOF (pinMatrix, pinMatrix, box1, box0);
	joint1->SetAngularLimits (dVector (-angle, -angle, -angle, 0.0f), dVector (angle, angle, angle, 0.0f));

#ifdef _USE_HARD_JOINTS
	NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate(scene->GetNewton(), base, NULL);
	NewtonSkeletonContainerAttachBone(skeleton, box0, base);
	NewtonSkeletonContainerAttachBone(skeleton, box1, box0);
	NewtonSkeletonContainerFinalize(skeleton);
#endif
}

static void AddUniversal(DemoEntityManager* const scene, const dVector& origin)
{
	dVector size(1.0f, 1.0f, 1.0f);
	NewtonBody* const box0 = CreateBox(scene, origin + dVector(0.0f, 4.0f, 0.0f, 0.0f), dVector(0.25f, 0.25f, 4.0f, 0.0f));
	NewtonBody* const box1 = CreateWheel(scene, origin + dVector(0.0f, 4.0f, 2.0f, 0.0f), 1.0f, 0.5f);
	NewtonBody* const box2 = CreateWheel(scene, origin + dVector(0.0f, 4.0f, -2.0f, 0.0f), 1.0f, 0.5f);

	NewtonBodySetMassMatrix(box0, 0.0f, 0.0f, 0.0f, 0.0f);

	// align the object so that is looks nice
	dMatrix matrix;
	NewtonBodyGetMatrix(box1, &matrix[0][0]);
	matrix = dYawMatrix (3.1416f * 0.5f) * matrix; 
	NewtonBodySetMatrix(box1, &matrix[0][0]);
	((DemoEntity*) NewtonBodyGetUserData(box1))->ResetMatrix (*scene, matrix);

	NewtonBodyGetMatrix(box2, &matrix[0][0]);
	matrix = dYawMatrix(3.1416f * 0.5f) * matrix;
	NewtonBodySetMatrix(box2, &matrix[0][0]);
	((DemoEntity*) NewtonBodyGetUserData(box2))->ResetMatrix (*scene, matrix);


	// link the two boxes
	NewtonBodyGetMatrix(box1, &matrix[0][0]);
	CustomUniversal* const joint1 = new CustomUniversal(matrix, box1, box0);
	joint1->EnableLimit_0(true);
	joint1->SetLimis_0 (-5.0f * 3.141592f, 2.0f * 3.141592f);
	joint1->EnableLimit_1(true);
	joint1->SetLimis_1 (-3.0f * 3.141592f, 4.0f * 3.141592f);

	// link the two boxes
	NewtonBodyGetMatrix(box2, &matrix[0][0]);
	CustomUniversal* const joint2 = new CustomUniversal(matrix, box2, box0);
	joint2->EnableLimit_0(true);
	joint2->SetLimis_0 (-3.0f * 3.141592f, 5.0f * 3.141592f);
	joint2->EnableLimit_1(true);
	joint2->SetLimis_1(-4.0f * 3.141592f, 2.0f * 3.141592f);


#ifdef _USE_HARD_JOINTS
	NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate(scene->GetNewton(), box0, NULL);
	NewtonSkeletonContainerAttachBone(skeleton, box1, box0);
	NewtonSkeletonContainerAttachBone(skeleton, box2, box0);
	NewtonSkeletonContainerFinalize(skeleton);
#endif
}


class JoesRagdollJoint: public CustomBallAndSocket
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

	int xxx;
	
	JoesRagdollJoint(NewtonBody* child, NewtonBody* parent, const dMatrix &localMatrix0, const dMatrix &localMatrix1, NewtonWorld *world)
		:CustomBallAndSocket(localMatrix0, localMatrix1, child, parent)
	{
		m_localMatrix0 = localMatrix0;
		m_localMatrix1 = localMatrix1;

		m_target = dQuaternion(dVector(1.0f, 0, 0), 0.0f);
		m_reduceError = 0.2f; // amount of error to reduce per timestep (more -> oszillation)
		m_angularFriction = 300.0f;
		m_stiffness = 0.9f;

		m_anim_speed = 0.0f;
		m_anim_offset = 0.0f;
		m_anim_time = 0.0f;

		static int xxxxx;
		xxx = xxxxx;
		xxxxx++;
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
if (xxx == 0) {
NewtonBody* const body0 = GetBody0();
dVector xxx(0.0f, -500.0f, 0.0f, 0.0f);
NewtonBodyAddForce (body0, &xxx.m_x);
}

		dMatrix matrix0;
		dMatrix matrix1;

		CalculateGlobalMatrix(matrix0, matrix1);

		if (m_anim_speed != 0.0f) // some animation to illustrate purpose
		{
			m_anim_time += timestep * m_anim_speed;
			dFloat a0 = sin(m_anim_time);
			dFloat a1 = m_anim_offset * 3.14f;
			dVector axis(sin(a1), 0.0f, cos(a1));
			//dVector axis (1,0,0);
			m_target = dQuaternion(axis, a0 * 0.5f);
		}

		// measure error
		dQuaternion q0(matrix0);
		dQuaternion q1(matrix1);
		dQuaternion qt0 = m_target * q1;
		dQuaternion qErr = ((q0.DotProduct(qt0) < 0.0f)	? dQuaternion(-q0.m_q0, q0.m_q1, q0.m_q2, q0.m_q3) : dQuaternion(q0.m_q0, -q0.m_q1, -q0.m_q2, -q0.m_q3)) * qt0;

		dFloat errorAngle = 2.0f * acos(dMax(dFloat(-1.0f), dMin(dFloat(1.0f), qErr.m_q0)));
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

		CustomBallAndSocket::SubmitConstraints(timestep, threadIndex);
		// motors
		for (int n = 0; n < 3; n++) {
			// calculate the desired acceleration
			dVector &axis = basis[n];
			dFloat relAccel = angAcc.DotProduct3(axis);

			NewtonUserJointAddAngularRow(m_joint, 0.0f, &axis[0]);
			NewtonUserJointSetRowAcceleration(m_joint, relAccel);

//			NewtonUserJointSetRowMinimumFriction(m_joint, -m_angularFriction);
//			NewtonUserJointSetRowMaximumFriction(m_joint, m_angularFriction);

if (n <= 1){
NewtonUserJointSetRowMinimumFriction(m_joint, -100.0f);
NewtonUserJointSetRowMaximumFriction(m_joint, 100.0f);
}
			NewtonUserJointSetRowStiffness(m_joint, m_stiffness);
		}
	}
};


void AddJoesPoweredRagDoll (DemoEntityManager* const scene, const dVector& origin, const dFloat animSpeed, const int numSegments)
{
    dFloat height = 1.0f;
    dFloat width = 4.0f;

    dVector size (width, height, width);
    NewtonBody* parent = CreateBox (scene, origin + dVector (0.0f,  0.5f, 0.0f, 0.0f), size);
//NewtonBodySetMassMatrix (parent, 0.0f, 0.0f, 0.0f, 0.0f);

#ifdef _USE_HARD_JOINTS
    NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate (scene->GetNewton(), parent, NULL);
#endif

    for (int i=0; i < numSegments; i++)
    {
        dFloat height = 1.0f;
        dFloat width = 0.5f;

        dVector size (width, height, width);
        NewtonBody* child = CreateBox (scene, origin + dVector (0.0f,  0.5f + height * dFloat(i+1), 0.0f, 0.0f), size);

//dVector xxxxx (0.5f, 0.0f, 0.0f, 0.0f);
//NewtonBodySetOmega(child, &xxxxx[0]);
        
        dMatrix matrix0 = dGetIdentityMatrix(); matrix0.m_posit = dVector (0.0f, height*-0.5f, 0.0f, 1.0f);
        dMatrix matrix1 = dGetIdentityMatrix(); matrix1.m_posit = dVector (0.0f, height*0.5f, 0.0f, 1.0f);
        JoesRagdollJoint* joint = new JoesRagdollJoint (child, parent, matrix0, matrix1, scene->GetNewton());

		if (animSpeed != 0.0f) {
			joint->m_anim_speed = animSpeed, joint->m_anim_offset = dFloat(i) / dFloat(numSegments); // animated      
		}

#ifdef _USE_HARD_JOINTS
        NewtonSkeletonContainerAttachBone (skeleton, child, parent);
#endif
        parent = child;
    }

#ifdef _USE_HARD_JOINTS
    NewtonSkeletonContainerFinalize(skeleton);
#endif
}



static void AddPoweredRagDoll (DemoEntityManager* const scene, const dVector& origin)
{
	dVector size (1.0f, 1.0f, 1.0f);
	NewtonBody* const box0 = CreateCapule(scene, origin + dVector(0.0f, 9.0f, 0.0f, 0.0f), size);
//	NewtonBody* const box1 = CreateCapule(scene, origin + dVector(0.0f, 9.0 - size.m_y * 2.0f, 0.0f, 0.0f), size);
//	NewtonBody* const box2 = CreateCapule(scene, origin + dVector(0.0f, 9.0 - size.m_y * 4.0f, 0.0f, 0.0f), size);
//	NewtonBody* const box3 = CreateCapule(scene, origin + dVector(0.0f, 9.0 - size.m_y * 6.0f, 0.0f, 0.0f), size);

	dMatrix pinMatrix (dGrammSchmidt (dVector (0.0f, -1.0f, 0.0f, 0.0f)));
//dMatrix pinMatrix (dGrammSchmidt (dVector (1.0f, 0.0f, 0.0f, 0.0f)));

	// connect first box to the world
	dMatrix matrix0;
	NewtonBodyGetMatrix (box0, & matrix0[0][0]);
	pinMatrix.m_posit = matrix0.m_posit + dVector (0.0f, size.m_y, 0.0f, 0.0f);
	CustomControlledBallAndSocket* const joint0 = new CustomControlledBallAndSocket (pinMatrix, box0, NULL);
	joint0->SetAngularVelocity (2000.0f * 3.141592f / 180.0f);
//	joint0->SetPitchAngle (-45.0f * 3.141592f / 180.0f);
//	joint0->SetYawAngle (-85.0f * 3.141592f / 180.0f);
//	joint0->SetRollAngle (120.0f * 3.141592f / 180.0f);
	
joint0->SetPitchAngle (90.0f * 3.141592f / 180.0f);	
/*
	// link the two boxes
	dMatrix matrix1;
	NewtonBodyGetMatrix (box1, &matrix1[0][0]);
	pinMatrix.m_posit = (matrix0.m_posit + matrix1.m_posit).Scale (0.5f);
	CustomControlledBallAndSocket* const joint1 = new CustomControlledBallAndSocket (pinMatrix, box0, box1);
	joint1->SetAngularVelocity (1000.0f * 3.141592f / 180.0f);
	joint1->SetPitchAngle (45.0f * 3.141592f / 180.0f);
	joint1->SetYawAngle ( 30.0f * 3.141592f / 180.0f);
	joint1->SetRollAngle (25.0f * 3.141592f / 180.0f);

	// link next box
	dMatrix matrix2;
	NewtonBodyGetMatrix(box2, &matrix2[0][0]);
	pinMatrix.m_posit = (matrix1.m_posit + matrix2.m_posit).Scale(0.5f);
	CustomControlledBallAndSocket* const joint2 = new CustomControlledBallAndSocket(pinMatrix, box1, box2);
	joint2->SetAngularVelocity(1000.0f * 3.141592f / 180.0f);
	joint2->SetPitchAngle(45.0f * 3.141592f / 180.0f);
	joint2->SetYawAngle(30.0f * 3.141592f / 180.0f);
	joint2->SetRollAngle(25.0f * 3.141592f / 180.0f);

	// link next box
	dMatrix matrix3;
	NewtonBodyGetMatrix(box3, &matrix3[0][0]);
	pinMatrix.m_posit = (matrix2.m_posit + matrix3.m_posit).Scale(0.5f);
	CustomControlledBallAndSocket* const joint3 = new CustomControlledBallAndSocket(pinMatrix, box2, box3);
	joint3->SetAngularVelocity(1000.0f * 3.141592f / 180.0f);
	joint3->SetPitchAngle(45.0f * 3.141592f / 180.0f);
	joint3->SetYawAngle(30.0f * 3.141592f / 180.0f);
	joint3->SetRollAngle(25.0f * 3.141592f / 180.0f);

#ifdef _USE_HARD_JOINTS
	NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate(scene->GetNewton(), box0, NULL);
	NewtonSkeletonContainerAttachBone(skeleton, box1, box0);
	NewtonSkeletonContainerAttachBone(skeleton, box2, box1);
	NewtonSkeletonContainerAttachBone(skeleton, box3, box2);
	NewtonSkeletonContainerFinalize(skeleton);
#endif
*/
}

void AddHinge (DemoEntityManager* const scene, const dVector& origin)
{
    dVector size (1.5f, 1.5f, 0.125f);
	NewtonBody* parent = CreateBox(scene, origin + dVector (-0.8f, 4.0f, 0.0f, 0.0f), dVector (0.2f, 0.125f, 0.125f));
	NewtonBodySetMassMatrix(parent, 0.0f, 0.0f, 0.0f, 0.0f);
    //the joint pin is the first row of the matrix, to make a upright pin we
    //take the x axis and rotate by 90 degree around the y axis
    dMatrix localPin (dRollMatrix(90.0f * 3.141592f / 180.0f));

	dMatrix matrix;
	dVector position (origin);
	position.m_y += 4.0f;
	NewtonBody* child = NULL;

#ifdef _USE_HARD_JOINTS
	NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate(scene->GetNewton(), parent, NULL);
#endif

	int count = 4;
	for (int i = 0; i < count; i ++) {
		child = CreateBox (scene, position, size);
		NewtonBodyGetMatrix(child, &matrix[0][0]);
		matrix.m_posit += dVector(-size.m_x * 0.5f, 0.0f, 0.0f);
		matrix = localPin * matrix;
		CustomHinge* const hinge = new CustomHinge (matrix, child, parent);

#ifdef _USE_HARD_JOINTS
		NewtonSkeletonContainerAttachBone (skeleton, child, parent);	
#endif

		hinge->EnableLimits (true);
		hinge->SetLimits (-45.0f * 3.141592f / 180.0f, 45.0f * 3.141592f / 180.0f);
		hinge->SetFriction(20.0f);
		parent = child;
		position.m_x += size.m_x;
	}

/*
	// link the two boxes
	NewtonBody* const heavyBox = CreateBox (scene, position, dVector (1.5f, 1.5f, 1.5f));
	NewtonBodyGetMatrix(heavyBox, &matrix[0][0]);
	NewtonBodySetMassProperties(heavyBox, 10.0f, NewtonBodyGetCollision(heavyBox));
	matrix.m_posit += dVector(-size.m_x * 0.5f, 0.0f, 0.0f);
	matrix = localPin * matrix;
	CustomHinge* const hinge = new CustomHinge(matrix, heavyBox, parent);
	hinge->EnableLimits(true);
	hinge->SetLimits(-45.0f * 3.141592f / 180.0f, 45.0f * 3.141592f / 180.0f);
	hinge->SetFriction(20.0f);
*/
#ifdef _USE_HARD_JOINTS
//	NewtonSkeletonContainerAttachBone (skeleton, heavyBox, parent);
	NewtonSkeletonContainerFinalize (skeleton);
#endif

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
    CustomSlider* const slider = new CustomSlider (matrix, box1, box0);

    // enable limit of first axis
    slider->EnableLimits(true);

    // set limit on second axis
    slider->SetLimits (-4.0f, 4.0f);

#ifdef _USE_HARD_JOINTS
	NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate(scene->GetNewton(), box0, NULL);
	NewtonSkeletonContainerAttachBone(skeleton, box1, box0);
	NewtonSkeletonContainerFinalize(skeleton);
#endif
}

static void AddSlidingContact(DemoEntityManager* const scene, const dVector& origin)
{
	// make a reel static
	NewtonBody* const box0 = CreateBox(scene, origin + dVector(0.0f, 4.0f, 0.0f, 0.0f), dVector(8.0f, 0.25f, 0.25f, 0.0f));
	NewtonBody* const box1 = CreateWheel(scene, origin + dVector(0.0f, 4.0f, 0.0f, 0.0f), 1.0f, 0.5f);

	dMatrix matrix;

	NewtonBodySetMassMatrix(box0, 0.0f, 0.0f, 0.0f, 0.0f);

	// connect the bodies by a Slider joint
	NewtonBodyGetMatrix(box1, &matrix[0][0]);
	CustomSlidingContact* const slider = new CustomSlidingContact(matrix, box1, box0);
	slider->EnableLinearLimits (true);
	slider->SetLinearLimis (-4.0f, 4.0f);

	// enable limit of first axis
	slider->EnableAngularLimits(true);
	slider->SetAngularLimis (-7.0f * 3.1416f, 5.0f * 3.1416f);

#ifdef _USE_HARD_JOINTS
	NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate(scene->GetNewton(), box0, NULL);
	NewtonSkeletonContainerAttachBone(skeleton, box1, box0);
	NewtonSkeletonContainerFinalize(skeleton);
#endif
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
    CustomCorkScrew* const cylinder = new CustomCorkScrew (matrix, box1, box0);

    // enable limit of first axis
    cylinder->EnableLinearLimits(true);
    cylinder->SetLinearLimis (-4.0f, 4.0f);

	// set angular limit on second axis
	cylinder->EnableAngularLimits(true);
	cylinder->SetAngularLimis(-4.0f * 3.1416f, 6.0f * 3.1416f);

#ifdef _USE_HARD_JOINTS
	NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate(scene->GetNewton(), box0, NULL);
	NewtonSkeletonContainerAttachBone(skeleton, box1, box0);
	NewtonSkeletonContainerFinalize(skeleton);
#endif
}


static CustomHinge* AddHingeWheel (DemoEntityManager* const scene, const dVector& origin, dFloat radius, dFloat height, NewtonBody* const parent)
{
    NewtonBody* const wheel = CreateWheel (scene, origin, height, radius);

    // the joint pin is the first row of the matrix
    //dMatrix localPin (dRollMatrix(90.0f * 3.141592f / 180.0f));
    dMatrix localPin (dGetIdentityMatrix());
    dMatrix matrix;
    NewtonBodyGetMatrix (wheel, & matrix[0][0]);
    matrix = localPin * matrix;

    // connect first box to the world
    return new CustomHinge (matrix, wheel, parent);
}


static void AddGear (DemoEntityManager* const scene, const dVector& origin)
{
    NewtonBody* const box0 = CreateCylinder(scene, origin + dVector (0.0f, 4.0f, 0.0f), 0.25f, 4.0f);

	// this is a fix joint
	NewtonBodySetMassMatrix(box0, 0.0f, 0.0f, 0.0f, 0.0f);

	// connect two bodies with a hinge 
    CustomHinge* const hinge0 = AddHingeWheel (scene, origin + dVector (-1.0f, 4.0f, 0.0f), 0.5f, 1.0f, box0);
    CustomHinge* const hinge1 = AddHingeWheel (scene, origin + dVector ( 1.0f, 4.0f, 0.0f), 0.5f, 1.0f, box0);

    NewtonBody* const body0 = hinge0->GetBody0();
    NewtonBody* const body1 = hinge1->GetBody0();

    dMatrix matrix0;
    dMatrix matrix1;
    NewtonBodyGetMatrix (body0, &matrix0[0][0]);
    NewtonBodyGetMatrix (body1, &matrix1[0][0]);

	// relate the two body motion with a gear joint
    dVector pin0 (matrix0.RotateVector(dVector (1.0f, 0.0f, 0.0f)));
    dVector pin1 (matrix1.RotateVector(dVector (1.0f, 0.0f, 0.0f)));
    new CustomGear (4.0f, pin0, pin1, body0, body1);

#ifdef _USE_HARD_JOINTS
	NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate(scene->GetNewton(), box0, NULL);
//	NewtonSkeletonContainerAttachBone(skeleton, box0, NULL);
	NewtonSkeletonContainerAttachBone(skeleton, hinge0->GetBody0(), box0);
	NewtonSkeletonContainerAttachBone(skeleton, hinge1->GetBody0(), box0);
	NewtonSkeletonContainerFinalize(skeleton);
#endif

}


static CustomSlider* AddSliderWheel (DemoEntityManager* const scene, const dVector& origin, dFloat radius, dFloat height, NewtonBody* const parent)
{
    NewtonBody* const wheel = CreateWheel (scene, origin, height, radius);

    // the joint pin is the first row of the matrix
    //dMatrix localPin (dRollMatrix(90.0f * 3.141592f / 180.0f));
    dMatrix localPin (dGetIdentityMatrix());
    dMatrix matrix;
    NewtonBodyGetMatrix (wheel, & matrix[0][0]);
    matrix = localPin * matrix;

    // connect first box to the world
    return new CustomSlider (matrix, wheel, parent);
}

void AddPulley (DemoEntityManager* const scene, const dVector& origin)
{
    NewtonBody* const reel0 = CreateBox(scene, origin + dVector (0.0f, 4.0f, 2.0f), dVector(4.0f, 0.25f, 0.25f));
	// this is just for show
    NewtonBody* const reel1 = CreateBox(scene, origin + dVector (0.0f, 4.0f, 0.0f), dVector(4.0f, 0.25f, 0.25f));
	NewtonBodySetMassMatrix (reel0, 0.0f, 0.0f, 0.0f, 0.0f);
	NewtonBodySetMassMatrix (reel1, 0.0f, 0.0f, 0.0f, 0.0f);
    
	dMatrix matrix;
    CustomSlider* const slider0 = AddSliderWheel (scene, origin + dVector (0.0f, 4.0f, 2.0f), 0.5f, 1.0f, reel0);
    CustomSlider* const slider1 = AddSliderWheel (scene, origin + dVector (0.0f, 4.0f, 0.0f), 0.5f, 0.5f, reel0);

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
    new CustomPulley (4.0f, pin0, pin1, body0, body1);

#ifdef _USE_HARD_JOINTS
	NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate(scene->GetNewton(), reel0, NULL);
	NewtonSkeletonContainerAttachBone(skeleton, slider0->GetBody0(), reel0);
	NewtonSkeletonContainerAttachBone(skeleton, slider1->GetBody0(), reel0);
	NewtonSkeletonContainerFinalize(skeleton);
#endif

	// make an aggregate for disabling collisions
	void* const aggregate = NewtonCollisionAggregateCreate (scene->GetNewton());
	NewtonCollisionAggregateSetSelfCollision (aggregate, 0);
	NewtonCollisionAggregateAddBody (aggregate, reel0);
	NewtonCollisionAggregateAddBody (aggregate, reel1);
	NewtonCollisionAggregateAddBody (aggregate, body0);
	NewtonCollisionAggregateAddBody (aggregate, body1);
}


static CustomCorkScrew* AddCylindricalWheel (DemoEntityManager* const scene, const dVector& origin, dFloat radius, dFloat height, NewtonBody* const parent)
{
    NewtonBody* const wheel = CreateWheel (scene, origin, height, radius);

    // the joint pin is the first row of the matrix
    dMatrix matrix;
    NewtonBodyGetMatrix (wheel, &matrix[0][0]);

    return new CustomCorkScrew (matrix, wheel, parent);
}


static void AddGearAndRack (DemoEntityManager* const scene, const dVector& origin)
{
    NewtonBody* const reel0 = CreateCylinder(scene, origin + dVector (0.0f, 4.0f, 0.0f), 0.25f, 4.0f);
    NewtonBody* const reel1 = CreateBox(scene, origin + dVector (0.0f, 4.0f, 2.0f), dVector(4.0f, 0.25f, 0.25f));
    
	dMatrix matrix;
	NewtonBodySetMassMatrix(reel0, 0.0f, 0.0f, 0.0f, 0.0f);
	NewtonBodySetMassMatrix(reel1, 0.0f, 0.0f, 0.0f, 0.0f);

    CustomHinge* const hinge0 = AddHingeWheel (scene, origin + dVector (-1.0f, 4.0f, 0.0f), 0.5f, 0.5f, reel0);
    CustomHinge* const hinge1 = AddHingeWheel (scene, origin + dVector ( 1.0f, 4.0f, 0.0f), 0.5f, 0.5f, reel0);
    CustomCorkScrew* const cylinder = AddCylindricalWheel(scene, origin + dVector (0.0f, 4.0f, 2.0f), 0.5f, 1.0f, reel0);

    cylinder->EnableLinearLimits(true);
    cylinder->SetLinearLimis(-2.0f, 2.0f);

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

    new CustomGear (5.0f, pin0, pin2, body0, body2);
    new CustomRackAndPinion (0.125f, pin1, pin2, body1, body2);

#ifdef _USE_HARD_JOINTS
	NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate(scene->GetNewton(), reel0, NULL);
	NewtonSkeletonContainerAttachBone(skeleton, hinge0->GetBody0(), reel0);
	NewtonSkeletonContainerAttachBone(skeleton, hinge1->GetBody0(), reel0);
	NewtonSkeletonContainerAttachBone(skeleton, cylinder->GetBody0(), reel0);
	NewtonSkeletonContainerFinalize(skeleton);
#endif

	// make an aggregate for disabling collisions
	void* const aggregate = NewtonCollisionAggregateCreate(scene->GetNewton());
	NewtonCollisionAggregateSetSelfCollision(aggregate, 0);
	NewtonCollisionAggregateAddBody(aggregate, reel0);
	NewtonCollisionAggregateAddBody(aggregate, reel1);
	NewtonCollisionAggregateAddBody(aggregate, body0);
	NewtonCollisionAggregateAddBody(aggregate, body1);
	NewtonCollisionAggregateAddBody(aggregate, body2);
}


class MyPathFollow: public CustomPathFollow
{
	public:
	MyPathFollow (const dMatrix& pinAndPivotFrame, NewtonBody* const body, DemoEntity* const path)
		:CustomPathFollow (pinAndPivotFrame, body)
		,m_rollerCosterPath (path)
	{
	}

	void GetPointAndTangentAtLocation (const dVector& location,  dVector& positOut, dVector& tangentOut) const
	{
		DemoBezierCurve* const mesh = (DemoBezierCurve*)m_rollerCosterPath->GetMesh(); 

		const dBezierSpline& spline = mesh->m_curve;

		dBigVector point;
		dFloat64 knot = spline.FindClosestKnot (point, location, 4);
		dBigVector tangent (spline.CurveDerivative (knot));
		tangent = tangent.Scale (1.0 / sqrt (tangent.DotProduct3(tangent)));

		positOut = dVector (point.m_x, point.m_y, point.m_z);
		tangentOut = dVector (tangent.m_x, tangent.m_y, tangent.m_z);
	}

	DemoEntity* m_rollerCosterPath;
};


class CustomDistanceRope: public CustomPointToPoint
{
	public:
	CustomDistanceRope (const dVector& pivotFrame0, const dVector& pivotFrame1, NewtonBody* const child, NewtonBody* const parent)
		:CustomPointToPoint(pivotFrame0, pivotFrame1, child, parent)
	{
	}

	void SubmitConstraints(dFloat timestep, int threadIndex)
	{
		dMatrix matrix0;
		dMatrix matrix1;

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix(matrix0, matrix1);

		dVector p0(matrix0.m_posit);
		dVector p1(matrix1.m_posit);

		dVector dir(p1 - p0);
		dFloat mag2 = dir.DotProduct3(dir);
		dir = dir.Scale(1.0f / dSqrt(mag2));
		dMatrix matrix(dGrammSchmidt(dir));
		dFloat x = dSqrt(mag2) - m_distance;

		dMatrix body0Matrix;
		dMatrix body1Matrix;
		dVector com0(0.0f);
		dVector com1(0.0f);
		dVector veloc0(0.0f);
		dVector veloc1(0.0f);

		NewtonBody* const body0 = GetBody0();
		NewtonBody* const body1 = GetBody1();

		NewtonBodyGetCentreOfMass(body0, &com0[0]);
		NewtonBodyGetMatrix(body0, &body0Matrix[0][0]);
		NewtonBodyGetPointVelocity(body0, &p0[0], &veloc0[0]);

		NewtonBodyGetCentreOfMass(body1, &com1[0]);
		NewtonBodyGetMatrix(body1, &body1Matrix[0][0]);
		NewtonBodyGetPointVelocity(body1, &p1[0], &veloc1[0]);

		dFloat v((veloc0 - veloc1).DotProduct3(dir));
		dFloat a = (x - v * timestep) / (timestep * timestep);

		dVector r0((p0 - body0Matrix.TransformVector(com0)).CrossProduct(matrix.m_front));
		dVector r1((p1 - body1Matrix.TransformVector(com1)).CrossProduct(matrix.m_front));
		dFloat jacobian0[6];
		dFloat jacobian1[6];

		jacobian0[0] = matrix[0][0];
		jacobian0[1] = matrix[0][1];
		jacobian0[2] = matrix[0][2];
		jacobian0[3] = r0[0];
		jacobian0[4] = r0[1];
		jacobian0[5] = r0[2];

		jacobian1[0] = -matrix[0][0];
		jacobian1[1] = -matrix[0][1];
		jacobian1[2] = -matrix[0][2];
		jacobian1[3] = -r1[0];
		jacobian1[4] = -r1[1];
		jacobian1[5] = -r1[2];
		NewtonUserJointAddGeneralRow(m_joint, jacobian0, jacobian1);
		NewtonUserJointSetRowAcceleration(m_joint, a);
	}
};


static void AddPathFollow (DemoEntityManager* const scene, const dVector& origin)
{
	// create a Bezier Spline path for AI car to drive
	DemoEntity* const rollerCosterPath = new DemoEntity(dGetIdentityMatrix(), NULL);
	scene->Append(rollerCosterPath);
	
	dBezierSpline spline;
	dFloat64 knots[] = {0.0f, 1.0f / 5.0f, 2.0f / 5.0f, 3.0f / 5.0f, 4.0f / 5.0f, 1.0f};

	dBigVector o (origin[0], origin[1],  origin[2],  0.0f);
	dBigVector control[] =
	{
		dBigVector(100.0f - 100.0f, 20.0f, 200.0f - 250.0f, 1.0f) + o,
		dBigVector(150.0f - 100.0f, 10.0f, 150.0f - 250.0f, 1.0f) + o,
		dBigVector(175.0f - 100.0f, 30.0f, 250.0f - 250.0f, 1.0f) + o,
		dBigVector(200.0f - 100.0f, 70.0f, 250.0f - 250.0f, 1.0f) + o,
		dBigVector(215.0f - 100.0f, 20.0f, 250.0f - 250.0f, 1.0f) + o,
		dBigVector(150.0f - 100.0f, 50.0f, 350.0f - 250.0f, 1.0f) + o,
		dBigVector( 50.0f - 100.0f, 30.0f, 250.0f - 250.0f, 1.0f) + o,
		dBigVector(100.0f - 100.0f, 20.0f, 200.0f - 250.0f, 1.0f) + o,
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
	dFloat64 knot = spline.FindClosestKnot(point0, dBigVector (origin + dVector(100.0f - 100.0f, 20.0f, 200.0f - 250.0f, 0.0f)), 4);
	positions[0] = dVector (point0.m_x, point0.m_y, point0.m_z, 0.0);
	dFloat average = 0.0f;
	for (int i = 0; i < count; i ++) {
		dBigVector point1;
		average += positions[i].m_y;
		dBigVector tangent(spline.CurveDerivative(knot));
		tangent = tangent.Scale (1.0 / sqrt (tangent.DotProduct3(tangent)));
		knot = spline.FindClosestKnot(point1, dBigVector (point0 + tangent.Scale (2.0f)), 4);
		point0 = point1;
		positions[i + 1] = dVector (point0.m_x, point0.m_y, point0.m_z, 0.0);
	}

	average /= count;
	for (int i = 0; i < count + 1; i ++) {
		positions[i].m_y = average;
	}

	dFloat attachmentOffset = 0.8f;
	for (int i = 0; i < count; i ++) {
		dMatrix matrix;
		dVector location0 (positions[i].m_x, positions[i].m_y, positions[i].m_z, 0.0);
		bodies[i] = CreateWheel(scene, location0, 1.0f, 0.5f);
		NewtonBodySetLinearDamping(bodies[i], 0.0f);
		NewtonBody* const box = bodies[i];
		NewtonBodyGetMatrix(box, &matrix[0][0]);

		dVector location1 (positions[i + 1].m_x, positions[i + 1].m_y, positions[i + 1].m_z, 0.0);
		dVector dir (location1 - location0);
		matrix.m_front = dir.Scale (1.0f / dSqrt (dir.DotProduct3(dir)));
		matrix.m_right = matrix.m_front.CrossProduct(matrix.m_up);
		dMatrix matrix1 (dYawMatrix(0.5f * 3.141692f) * matrix);
		NewtonBodySetMatrix(box, &matrix1[0][0]);
		matrix.m_posit.m_y += attachmentOffset;		
		new MyPathFollow(matrix, box, rollerCosterPath);

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
		new CustomDistanceRope (matrix1.m_posit, matrix0.m_posit, box1, box0);
	}
	
	void* const aggregate = NewtonCollisionAggregateCreate (scene->GetNewton());
	for (int i = 0; i < count; i ++) {
		NewtonCollisionAggregateAddBody(aggregate, bodies[i]);
	}
	NewtonCollisionAggregateSetSelfCollision (aggregate, false);

#ifdef _USE_HARD_JOINTS
	NewtonSkeletonContainer* const skeleton = NewtonSkeletonContainerCreate(scene->GetNewton(), bodies[0], NULL);
	for (int i = 1; i < count; i++) {
		NewtonSkeletonContainerAttachBone(skeleton, bodies[i], bodies[i - 1]);
	}
	NewtonSkeletonContainerFinalize(skeleton);
#endif
}

void StandardJoints (DemoEntityManager* const scene)
{
    scene->CreateSkyBox();

    // customize the scene after loading
    // set a user friction variable in the body for variable friction demos
    // later this will be done using LUA script
    dMatrix offsetMatrix (dGetIdentityMatrix());

    CreateLevelMesh (scene, "flatPlane.ngd", 1);
	//CreateLevelMesh (scene, "flatPlane1.ngd", 1);

    dVector location (0.0f);
    dVector size (1.5f, 2.0f, 2.0f, 0.0f);

//	AddJoesPoweredRagDoll (scene, dVector (0.0f, 0.0f, -25.0f), 0.0f, 20);
	AddJoesPoweredRagDoll (scene, dVector (0.0f, 0.0f, -25.0f), 0.0f, 1);
//	AddJoesPoweredRagDoll (scene, dVector (0.0f, 0.0f, -15.0f), 0.0f, 20, false);

//AddHinge (scene, dVector (-20.0f, 0.0f, 0.0f));

/*
	AddDistance (scene, dVector (-20.0f, 0.0f, -25.0f));
	AddLimitedBallAndSocket (scene, dVector (-20.0f, 0.0f, -20.0f));
//	AddPoweredRagDoll (scene, dVector (-20.0f, 0.0f, -15.0f));
	AddBallAndSockectWithFriction (scene, dVector (-20.0f, 0.0f, -10.0f));
	Add6DOF (scene, dVector (-20.0f, 0.0f, -5.0f));
	AddHinge (scene, dVector (-20.0f, 0.0f, 0.0f));
	AddSlider (scene, dVector (-20.0f, 0.0f, 5.0f));
	AddCylindrical (scene, dVector (-20.0f, 0.0f, 10.0f));
	AddUniversal (scene, dVector (-20.0f, 0.0f, 15.0f));
	AddGear (scene, dVector (-20.0f, 0.0f, 20.0f));
	AddPulley (scene, dVector (-20.0f, 0.0f, 25.0f));
	AddGearAndRack (scene, dVector (-20.0f, 0.0f, 30.0f));
	AddSlidingContact (scene, dVector (-20.0f, 0.0f, 35.0f));
//	AddPathFollow (scene, dVector (20.0f, 0.0f, 0.0f));
	AddJoesPoweredRagDoll (scene, dVector (0.0f, 0.0f, -25.0f), 0.0f, 20, true);
	AddJoesPoweredRagDoll (scene, dVector (0.0f, 0.0f, -15.0f), 0.0f, 20, false);
	AddJoesPoweredRagDoll (scene, dVector (0.0f, 0.0f,   5.0f), 1.5f, 4, false); // animated
	AddJoesPoweredRagDoll (scene, dVector (0.0f, 0.0f,  15.0f), 0.0f, 4, false);

*/
    // place camera into position
    dMatrix camMatrix (dGetIdentityMatrix());
    dQuaternion rot (camMatrix);
    dVector origin (-40.0f, 5.0f, 0.0f, 0.0f);
    scene->SetCameraMatrix(rot, origin);
}



