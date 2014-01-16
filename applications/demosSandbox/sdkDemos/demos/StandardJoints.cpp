/*
// add standard joints example
AddBallAndSockect (mSceneMgr, m_physicsWorld, Vector3 (-15.0f, 0.0f, -25.0f));
AddCylindrical (mSceneMgr, m_physicsWorld, Vector3 (-10.0f, 0.0f, -25.0f));
AddHinge (mSceneMgr, m_physicsWorld, Vector3 (-5.0f, 0.0f, -25.0f));
AddUniversal (mSceneMgr, m_physicsWorld, Vector3 (2.0f, 0.0f, -25.0f));
AddSlider (mSceneMgr, m_physicsWorld, Vector3 (8.0f, 0.0f, -25.0f));

//add relational joints example 
AddGear (mSceneMgr, m_physicsWorld, Vector3 (-10.0f, 0.0f, -15.0f));
AddPulley (mSceneMgr, m_physicsWorld, Vector3 (0.0f, 0.0f, -15.0f));
AddGearAndRack (mSceneMgr, m_physicsWorld, Vector3 (10.0f, 0.0f, -15.0f));
*/

#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DemoMesh.h"
#include "../toolBox/OpenGlUtil.h"
#include <CustomGear.h>
#include <CustomHinge.h>
#include <CustomSlider.h>
#include <CustomCorkScrew.h>
#include <CustomBallAndSocket.h>


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
            dMatrix basis (dgGrammSchmidt (relOmega));
            NewtonUserJointAddAngularRow (m_joint, 0.0f, &basis[2][0]);
            NewtonUserJointAddAngularRow (m_joint, 0.0f, &basis[1][0]);
            NewtonUserJointAddAngularRow (m_joint, 0.0f, &basis[0][0]);

            // calculate the acceleration to stop the ball in one time step
            dFloat invTimestep = (timestep > 0.0f) ? 1.0f / timestep: 1.0f;

            // override the desired accelearion, with the deseried acceleration for full stop. 
            NewtonUserJointSetRowAcceleration (m_joint, -omegaMag * invTimestep);

            // set the friction limit proportional the sphere Inertia
            NewtonUserJointSetRowMinimumFriction (m_joint, -m_dryFriction);
            NewtonUserJointSetRowMaximumFriction (m_joint,  m_dryFriction);
        } else {
            // when omega is too low this is correct but the small angle approximation theorem.
            dMatrix basis (GetIdentityMatrix());
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
    NewtonCollision* const collision = CreateConvexCollision (world, GetIdentityMatrix(), size, _BOX_PRIMITIVE, 0);
   	DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");

    dFloat mass = 1.0f;
    dMatrix matrix (GetIdentityMatrix());
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
    NewtonCollision* const collision = CreateConvexCollision (world, GetIdentityMatrix(), size, _CHAMFER_CYLINDER_PRIMITIVE, 0);
    DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");

    dFloat mass = 1.0f;
    dMatrix matrix (GetIdentityMatrix());
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
    NewtonCollision* const collision = CreateConvexCollision (world, GetIdentityMatrix(), size, _CYLINDER_PRIMITIVE, 0);
    DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");

    dFloat mass = 1.0f;
    dMatrix matrix (GetIdentityMatrix());
    matrix.m_posit = location;
    matrix.m_posit.m_w = 1.0f;
    NewtonBody* const body = CreateSimpleSolid (scene, geometry, mass, matrix, collision, materialID);

    geometry->Release();
    NewtonDestroyCollision(collision);
    return body;
}


static void AddBallAndSockect (DemoEntityManager* const scene, const dVector& origin)
{
//    NewtonWork* const world = scene->GetNewton();
    dVector size (1.0f, 1.0f, 1.0f);
    NewtonBody* const box0 = CreateBox (scene, origin + dVector (0.0f, 5.0f, 0.0f, 0.0f), size);
    NewtonBody* const box1 = CreateBox (scene, origin + dVector (size.m_x, 5.0 - size.m_y, size.m_z, 0.0f), size);

    // make the first body static
    //NewtonBodySetMassMatrix(box0, 0.0f, 0.0f, 0.0f, 0.0f);
    
    // connect first box to the world
    dMatrix matrix;
    NewtonBodyGetMatrix (box0, & matrix[0][0]);
    matrix.m_posit += dVector (-size.m_x * 0.5f, size.m_y * 0.5f, -size.m_z * 0.5f, 0.0f);
    new CustomBallAndSocketWithFriction (matrix, box0, NULL, 2.0f);

    // link the two boxes
    NewtonBodyGetMatrix (box1, & matrix[0][0]);
    matrix.m_posit += dVector (-size.m_x * 0.5f, size.m_y * 0.5f, -size.m_z * 0.5f, 0.0f);
    new CustomBallAndSocket (matrix, box0, box1);
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
    dMatrix localPin (GetIdentityMatrix());
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


void StandardJoints (DemoEntityManager* const scene)
{
    scene->CreateSkyBox();

    // customize the scene after loading
    // set a user friction variable in the body for variable friction demos
    // later this will be done using LUA script
    dMatrix offsetMatrix (GetIdentityMatrix());

    CreateLevelMesh (scene, "flatPlane.ngd", 1);

    dVector location (0.0f, 0.0f, 0.0f, 0.0f);
    dVector size (1.5f, 2.0f, 2.0f, 0.0f);

    AddBallAndSockect (scene, dVector (-20.0f, 0.0f, -15.0f));
    AddHinge (scene, dVector (-20.0f, 0.0f, -10.0f));
    AddSlider (scene, dVector (-20.0f, 0.0f, -5.0f));
    AddCylindrical (scene, dVector (-20.0f, 0.0f, 0.0f));

    // this joint is not very stable when using non rotational inertia, like these examples
//  AddUniversal (mSceneMgr, m_physicsWorld, Vector3 (2.0f, 0.0f, -25.0f));

    //add relational joints example 
    AddGear (scene, dVector (-20.0f, 0.0f, 5.0f));
//  AddPulley (mSceneMgr, m_physicsWorld, Vector3 (0.0f, 0.0f, -15.0f));
//  AddGearAndRack (mSceneMgr, m_physicsWorld, Vector3 (10.0f, 0.0f, -15.0f));

    // place camera into position
    dMatrix camMatrix (GetIdentityMatrix());
    dQuaternion rot (camMatrix);
    dVector origin (-40.0f, 5.0f, 0.0f, 0.0f);
    scene->SetCameraMatrix(rot, origin);
}



