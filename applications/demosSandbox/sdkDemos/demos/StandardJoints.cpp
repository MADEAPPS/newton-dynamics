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
#include <CustomCorkScrew.h>
#include <CustomBallAndSocket.h>
#include <CustomRackAndPinion.h>

class CustomDistance: public CustomJoint  
{
	public: 
	CustomDistance (const dVector& pivotInChildInGlobalSpace, const dVector& pivotInParentInGlobalSpace, NewtonBody* const child, NewtonBody* const parent)
		:CustomJoint(3, child, parent)
	{
		dVector dist (pivotInChildInGlobalSpace - pivotInParentInGlobalSpace) ;
		m_distance = dSqrt (dist % dist);

		dMatrix childMatrix (dGetIdentityMatrix());
		dMatrix parentMatrix (dGetIdentityMatrix());

		childMatrix.m_posit = pivotInChildInGlobalSpace;
		parentMatrix.m_posit = pivotInParentInGlobalSpace;
		childMatrix.m_posit.m_w = 1.0f;
		parentMatrix.m_posit.m_w = 1.0f;

		dMatrix dummy;
		//CalculateLocalMatrix (childMatrix, m_localPivot, dummy);
		//CalculateLocalMatrix (parentMatrix, dummy, m_parentPivot);
		CalculateLocalMatrix (childMatrix, m_localMatrix0, dummy);
		CalculateLocalMatrix (parentMatrix, dummy, m_localMatrix1);
	}

    void SubmitConstraints (dFloat timestep, int threadIndex)
	{
		dMatrix matrix0;
		dMatrix matrix1;

		// calculate the position of the pivot point and the Jacobian direction vectors, in global space. 
		CalculateGlobalMatrix (matrix0, matrix1);

		dVector p0 (matrix0.m_posit);
		dVector p1 (matrix1.m_posit);

		dVector dist (p1 - p0);
		dFloat mag2 = dist % dist;
		if (mag2 > 0.0f) {
			dist = dist.Scale (1.0f / dSqrt (mag2));
			p1 -= dist.Scale(m_distance);
		}

		// Restrict the movement on the pivot point along all tree orthonormal direction
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1.m_front[0]);
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1.m_up[0]);
		NewtonUserJointAddLinearRow (m_joint, &p0[0], &p1[0], &matrix1.m_right[0]);
	}

	//dMatrix m_localPivot;
	//dMatrix m_parentPivot;
	dFloat m_distance;
};

class CustomBallAndSocketWithFriction: public CustomBallAndSocket
{
	public:
    CustomBallAndSocketWithFriction (const dMatrix& pinAndPivotFrame, NewtonBody* const child, NewtonBody* const parent, dFloat dryFriction)
        :CustomBallAndSocket (pinAndPivotFrame, child, parent)
        ,m_dryFriction (dryFriction)
    {
    }

    void SubmitConstraints (dFloat timestep, int threadIndex)
    {
        CustomBallAndSocket::SubmitConstraints (timestep, threadIndex);

        dVector omega0(0.0f, 0.0f, 0.0f, 0.0f);
        dVector omega1(0.0f, 0.0f, 0.0f, 0.0f);

        // get the omega vector
        NewtonBodyGetOmega(m_body0, &omega0[0]);
        if (m_body1) {
            NewtonBodyGetOmega(m_body1, &omega1[0]);
        }

        dVector relOmega (omega0 - omega1);
        dFloat omegaMag = dSqrt (relOmega % relOmega);
        if (omegaMag > 0.1f) {
            // tell newton to used this the friction of the omega vector to apply the rolling friction
            dMatrix basis (dGrammSchmidt (relOmega));
            NewtonUserJointAddAngularRow (m_joint, 0.0f, &basis[2][0]);
            NewtonUserJointAddAngularRow (m_joint, 0.0f, &basis[1][0]);
            NewtonUserJointAddAngularRow (m_joint, 0.0f, &basis[0][0]);

            // calculate the acceleration to stop the ball in one time step
            dFloat invTimestep = (timestep > 0.0f) ? 1.0f / timestep: 1.0f;

            // override the desired acceleration, with the desired acceleration for full stop. 
            NewtonUserJointSetRowAcceleration (m_joint, -omegaMag * invTimestep);

            // set the friction limit proportional the sphere Inertia
            NewtonUserJointSetRowMinimumFriction (m_joint, -m_dryFriction);
            NewtonUserJointSetRowMaximumFriction (m_joint,  m_dryFriction);
        } else {
            // when omega is too low this is correct but the small angle approximation theorem.
            dMatrix basis (dGetIdentityMatrix());
            for (int i = 0; i < 3; i ++) {
                NewtonUserJointAddAngularRow (m_joint, 0.0f, &basis[i][0]);
                NewtonUserJointSetRowMinimumFriction (m_joint, -m_dryFriction);
                NewtonUserJointSetRowMaximumFriction (m_joint,  m_dryFriction);
            }
        }
    }

    dFloat m_dryFriction;
};

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
	NewtonBody* const box0 = CreateCapule (scene, origin + dVector (0.0f,  5.0f, 0.0f, 0.0f), size);
	NewtonBody* const box1 = CreateCapule (scene, origin + dVector (0.0f,  5.0 - size.m_y * 4.0f, 0.0f, 0.0f), size);

	dMatrix pinMatrix (dGrammSchmidt (dVector (0.0f, -1.0f, 0.0f, 0.0f)));

	// connect first box to the world
	dMatrix matrix0;
	NewtonBodyGetMatrix (box0, &matrix0[0][0]);
	pinMatrix.m_posit = matrix0.m_posit + dVector (0.0f, size.m_y, 0.0f, 0.0f);
	new CustomBallAndSocket (pinMatrix, box0, NULL);

	// link the two boxes with a distance joint
	dMatrix matrix1;
	NewtonBodyGetMatrix (box1, &matrix1[0][0]);

	// get the origins
	dVector pivot0 (matrix0.m_posit);
	dVector pivot1 (matrix1.m_posit);

	// connect bodies at a corner
	new CustomDistance (pivot0, pivot1, box0, box1);
}


static void AddBallAndSockectWithFriction (DemoEntityManager* const scene, const dVector& origin)
{
	dVector size (1.0f, 1.0f, 1.0f);
	NewtonBody* const box0 = CreateCapule (scene, origin + dVector (0.0f,  5.0f, 0.0f, 0.0f), size);
	NewtonBody* const box1 = CreateCapule (scene, origin + dVector (0.0f,  5.0 - size.m_y * 2.0f, 0.0f, 0.0f), size);

	dMatrix pinMatrix (dGrammSchmidt (dVector (0.0f, -1.0f, 0.0f, 0.0f)));

	// connect first box to the world
	dMatrix matrix0;
	NewtonBodyGetMatrix (box0, &matrix0[0][0]);
	pinMatrix.m_posit = matrix0.m_posit + dVector (0.0f, size.m_y, 0.0f, 0.0f);
	new CustomBallAndSocketWithFriction (pinMatrix, box0, NULL, 2.0f);

	// link the two boxes
	dMatrix matrix1;
	NewtonBodyGetMatrix (box1, & matrix1[0][0]);
	pinMatrix.m_posit = (matrix0.m_posit + matrix1.m_posit).Scale (0.5f);
	new CustomBallAndSocket (pinMatrix, box0, box1);
}


static void Add6DOF (DemoEntityManager* const scene, const dVector& origin)
{
	dVector size (1.0f, 1.0f, 1.0f);
	NewtonBody* const box0 = CreateCapule (scene, origin + dVector (0.0f,  5.0f, 0.0f, 0.0f), size);
	NewtonBody* const box1 = CreateCapule (scene, origin + dVector (0.0f,  5.0 - size.m_y * 2.0f, 0.0f, 0.0f), size);

	const dFloat angle = 60.0f * 3.1415592f / 180.0f;
	dMatrix pinMatrix (dGrammSchmidt (dVector (0.0f, -1.0f, 0.0f, 0.0f)));

	// connect first box to the world
	dMatrix matrix0;
	NewtonBodyGetMatrix (box0, & matrix0[0][0]);
	pinMatrix.m_posit = matrix0.m_posit + dVector (0.0f, size.m_y, 0.0f, 0.0f);
	Custom6DOF* const joint0 = new Custom6DOF (pinMatrix, pinMatrix, box0, NULL);
	joint0->SetAngularLimits (dVector (-angle, -angle, -angle, 0.0f), dVector (angle, angle, angle, 0.0f));

	// link the two boxes
	dMatrix matrix1;
	NewtonBodyGetMatrix (box1, &matrix1[0][0]);
	pinMatrix.m_posit = (matrix0.m_posit + matrix1.m_posit).Scale (0.5f);
	Custom6DOF* const joint1 = new Custom6DOF (pinMatrix, pinMatrix, box0, box1);
	joint1->SetAngularLimits (dVector (-angle, -angle, -angle, 0.0f), dVector (angle, angle, angle, 0.0f));
}

static void AddPoweredRagDoll (DemoEntityManager* const scene, const dVector& origin)
{
	dVector size (1.0f, 1.0f, 1.0f);
	NewtonBody* const box0 = CreateCapule (scene, origin + dVector (0.0f,  5.0f, 0.0f, 0.0f), size);
	NewtonBody* const box1 = CreateCapule (scene, origin + dVector (0.0f,  5.0 - size.m_y * 2.0f, 0.0f, 0.0f), size);

	dMatrix pinMatrix (dGrammSchmidt (dVector (0.0f, -1.0f, 0.0f, 0.0f)));

	// connect first box to the world
	dMatrix matrix0;
	NewtonBodyGetMatrix (box0, & matrix0[0][0]);
	pinMatrix.m_posit = matrix0.m_posit + dVector (0.0f, size.m_y, 0.0f, 0.0f);
	CustomControlledBallAndSocket* const joint0 = new CustomControlledBallAndSocket (pinMatrix, box0, NULL);
	joint0->SetAngularVelocity (1000.0f * 3.141592f / 180.0f);
	joint0->SetPitchAngle (-45.0f * 3.141592f / 180.0f);
	joint0->SetYawAngle (-85.0f * 3.141592f / 180.0f);
	joint0->SetRollAngle (120.0f * 3.141592f / 180.0f);

	// link the two boxes
	dMatrix matrix1;
	NewtonBodyGetMatrix (box1, &matrix1[0][0]);
	pinMatrix.m_posit = (matrix0.m_posit + matrix1.m_posit).Scale (0.5f);
	CustomControlledBallAndSocket* const joint1 = new CustomControlledBallAndSocket (pinMatrix, box0, box1);
	joint1->SetAngularVelocity (1000.0f * 3.141592f / 180.0f);
	joint1->SetPitchAngle (45.0f * 3.141592f / 180.0f);
	joint1->SetYawAngle ( 30.0f * 3.141592f / 180.0f);
	joint1->SetRollAngle (25.0f * 3.141592f / 180.0f);
}




void AddHinge (DemoEntityManager* const scene, const dVector& origin)
{
    dVector size (1.5f, 4.0f, 0.125f);
    NewtonBody* const box0 = CreateBox (scene, origin + dVector (0.0f, 4.0f, 0.0f, 0.0f), size);
    NewtonBody* const box1 = CreateBox (scene, origin + dVector (1.5f, 4.0f, 0.0f, 0.0f), size);
    NewtonBody* const box2 = CreateBox (scene, origin + dVector (3.0f, 4.0f, 0.0f, 0.0f), size);

    // the joint pin is the first row of the matrix, to make a upright pin we
    // take the x axis and rotate by 90 degree around the y axis
    dMatrix localPin (dRollMatrix(90.0f * 3.141592f / 180.0f));

    // connect first box to the world
    dMatrix matrix;
    NewtonBodyGetMatrix (box0, & matrix[0][0]);
    matrix.m_posit += dVector (-size.m_x * 0.5f, 0.0f, 0.0f);
    matrix = localPin * matrix;

    // add hinge with limit and friction
    CustomHinge* const hinge0 = new CustomHinge (matrix, box0, NULL);
    hinge0->EnableLimits (true);
    hinge0->SetLimis(-45.0f * 3.141592f / 180.0f, 45.0f * 3.141592f / 180.0f);
    hinge0->SetFriction (20.0f);

    // link the two boxes
    NewtonBodyGetMatrix (box1, &matrix[0][0]);
    matrix.m_posit += dVector (-size.m_x * 0.5f, 0.0f, 0.0f);
    matrix = localPin * matrix;
    CustomHinge* const hinge1 = new CustomHinge (matrix, box0, box1);
    hinge1->EnableLimits (true);
    hinge1->SetLimis (-45.0f * 3.141592f / 180.0f, 45.0f * 3.141592f / 180.0f);
    hinge1->SetFriction (20.0f);

    // link the two boxes
    NewtonBodyGetMatrix (box2, &matrix[0][0]);
    matrix.m_posit += dVector (-size.m_x * 0.5f, 0.0f, 0.0f);
    matrix = localPin * matrix;
    CustomHinge* const hinge2 = new CustomHinge (matrix, box1, box2);
    hinge2->EnableLimits (true);
    hinge2->SetLimis (-45.0f * 3.141592f / 180.0f, 45.0f * 3.141592f / 180.0f);
    //hinge2->SetFriction (20.0f);
}

static void AddSlider (DemoEntityManager* const scene, const dVector& origin)
{
    // make a reel static
    NewtonBody* const reel = CreateBox (scene, origin + dVector (0.0f, 4.0f, 0.0f, 0.0f), dVector (8.0f, 0.25f, 0.25f, 0.0f));
    NewtonBodySetMassMatrix (reel, 0.0f, 0.0f, 0.0f, 0.0f);

    NewtonBody* const wheel = CreateWheel (scene, origin + dVector (0.0f, 4.0f, 0.0f, 0.0f), 1.0f, 0.5f);

    dMatrix matrix;
    NewtonBodyGetMatrix (wheel, &matrix[0][0]);

    CustomSlider* const slider = new CustomSlider (matrix, wheel, reel);

    // enable limit of first axis
    slider->EnableLimits(true);

    // set limit on second axis
    slider->SetLimis (-4.0f, 4.0f);
}

static void AddCylindrical (DemoEntityManager* const scene, const dVector& origin)
{
    // make a reel static
    NewtonBody* const reel = CreateCylinder (scene, origin + dVector (0.0f, 4.0f, 0.0f, 0.0f), 0.25f, 8.0f);
    NewtonBodySetMassMatrix (reel, 0.0f, 0.0f, 0.0f, 0.0f);

    NewtonBody* const wheel = CreateWheel (scene, origin + dVector (0.0f, 4.0f, 0.0f, 0.0f), 1.0f, 0.5f);

    dMatrix matrix;
    NewtonBodyGetMatrix (wheel, &matrix[0][0]);

    CustomCorkScrew* const cylinder = new CustomCorkScrew (matrix, wheel, reel);

    // enable limit of first axis
    cylinder->EnableLinearLimits(true);

    // set limit on second axis
    cylinder->SetLinearLimis (-4.0f, 4.0f);
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
    NewtonBody* const reel = CreateCylinder(scene, origin + dVector (0.0f, 4.0f, 0.0f), 0.25f, 4.0f);
    NewtonBodySetMassMatrix (reel, 0.0f, 0.0f, 0.0f, 0.0f);

    CustomHinge* const hinge0 = AddHingeWheel (scene, origin + dVector (-1.0f, 4.0f, 0.0f), 0.5f, 1.0f, reel);
    CustomHinge* const hinge1 = AddHingeWheel (scene, origin + dVector ( 1.0f, 4.0f, 0.0f), 0.5f, 1.0f, reel);

    NewtonBody* const body0 = hinge0->GetBody0();
    NewtonBody* const body1 = hinge1->GetBody0();

    dMatrix matrix0;
    dMatrix matrix1;
    NewtonBodyGetMatrix (body0, &matrix0[0][0]);
    NewtonBodyGetMatrix (body1, &matrix1[0][0]);

    dVector pin0 (matrix0.RotateVector(dVector (1.0f, 0.0f, 0.0f)));
    dVector pin1 (matrix1.RotateVector(dVector (1.0f, 0.0f, 0.0f)));
    new CustomGear (4.0f, pin0, pin1, body0, body1);
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
    NewtonBody* const reel1 = CreateBox(scene, origin + dVector (0.0f, 4.0f, 0.0f), dVector(4.0f, 0.25f, 0.25f));
    NewtonBodySetMassMatrix (reel0, 0.0f, 0.0f, 0.0f, 0.0f);
    NewtonBodySetMassMatrix (reel1, 0.0f, 0.0f, 0.0f, 0.0f);

    CustomSlider* const slider0 = AddSliderWheel (scene, origin + dVector (0.0f, 4.0f, 2.0f), 0.5f, 1.0f, reel0);
    CustomSlider* const slider1 = AddSliderWheel (scene, origin + dVector (0.0f, 4.0f, 0.0f), 0.5f, 0.5f, reel1);

    slider0->EnableLimits(true);
    slider0->SetLimis (-2.0f, 2.0f);

    NewtonBody* const body0 = slider0->GetBody0();
    NewtonBody* const body1 = slider1->GetBody0();

    dMatrix matrix0;
    dMatrix matrix1;
    NewtonBodyGetMatrix (body0, &matrix0[0][0]);
    NewtonBodyGetMatrix (body1, &matrix1[0][0]);

    dVector pin0 (matrix0.RotateVector(dVector (1.0f, 0.0f, 0.0f)));
    dVector pin1 (matrix1.RotateVector(dVector (1.0f, 0.0f, 0.0f)));
    new CustomPulley (4.0f, pin0, pin1, body0, body1);
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
    NewtonBodySetMassMatrix (reel0, 0.0f, 0.0f, 0.0f, 0.0f);
    NewtonBodySetMassMatrix (reel1, 0.0f, 0.0f, 0.0f, 0.0f);

    CustomHinge* const hinge0 = AddHingeWheel (scene, origin + dVector (-1.0f, 4.0f, 0.0f), 0.5f, 0.5f, reel0);
    CustomHinge* const hinge1 = AddHingeWheel (scene, origin + dVector ( 1.0f, 4.0f, 0.0f), 0.5f, 0.5f, reel0);
    CustomCorkScrew* const cylinder = AddCylindricalWheel(scene, origin + dVector (0.0f, 4.0f, 2.0f), 0.5f, 1.0f, reel1);

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

    new CustomGear (1.0f, pin0, pin2, body0, body2);
    new CustomRackAndPinion (1.0f, pin1, pin2, body1, body2);
}




void StandardJoints (DemoEntityManager* const scene)
{
    scene->CreateSkyBox();

    // customize the scene after loading
    // set a user friction variable in the body for variable friction demos
    // later this will be done using LUA script
    dMatrix offsetMatrix (dGetIdentityMatrix());

    CreateLevelMesh (scene, "flatPlane.ngd", 1);

    dVector location (0.0f, 0.0f, 0.0f, 0.0f);
    dVector size (1.5f, 2.0f, 2.0f, 0.0f);

	AddDistance (scene, dVector (-20.0f, 0.0f, -20.0f));
	AddBallAndSockectWithFriction (scene, dVector (-20.0f, 0.0f, -15.0f));
	Add6DOF (scene, dVector (-20.0f, 0.0f, -10.0f));
	AddPoweredRagDoll (scene, dVector (-20.0f, 0.0f, -5.0f));

	AddHinge (scene, dVector (-20.0f, 0.0f, -0.0f));
	AddSlider (scene, dVector (-20.0f, 0.0f, 5.0f));
	AddCylindrical (scene, dVector (-20.0f, 0.0f, 10.0f));

    //add relational joints example 
	AddGear (scene, dVector (-20.0f, 0.0f, 15.0f));
	AddPulley (scene, dVector (-20.0f, 0.0f, 20.0f));
	AddGearAndRack (scene, dVector (-20.0f, 0.0f, 25.0f));

	// this joint is not very stable when using non rotational inertia, like these examples
	// AddUniversal (mSceneMgr, m_physicsWorld, Vector3 (2.0f, 0.0f, 30.0f));

    // place camera into position
    dMatrix camMatrix (dGetIdentityMatrix());
    dQuaternion rot (camMatrix);
    dVector origin (-40.0f, 5.0f, 0.0f, 0.0f);
    scene->SetCameraMatrix(rot, origin);
}



