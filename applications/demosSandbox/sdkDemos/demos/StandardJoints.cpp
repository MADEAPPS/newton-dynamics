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


//    AddBallAndSockect (mSceneMgr, m_physicsWorld, Vector3 (-15.0f, 0.0f, -25.0f));
//    AddCylindrical (mSceneMgr, m_physicsWorld, Vector3 (-10.0f, 0.0f, -25.0f));
//    AddHinge (mSceneMgr, m_physicsWorld, Vector3 (-5.0f, 0.0f, -25.0f));
//    AddUniversal (mSceneMgr, m_physicsWorld, Vector3 (2.0f, 0.0f, -25.0f));
//    AddSlider (mSceneMgr, m_physicsWorld, Vector3 (8.0f, 0.0f, -25.0f));

    //add relational joints example 
//    AddGear (mSceneMgr, m_physicsWorld, Vector3 (-10.0f, 0.0f, -15.0f));
//    AddPulley (mSceneMgr, m_physicsWorld, Vector3 (0.0f, 0.0f, -15.0f));
//    AddGearAndRack (mSceneMgr, m_physicsWorld, Vector3 (10.0f, 0.0f, -15.0f));


    // place camera into position
    dMatrix camMatrix (GetIdentityMatrix());
    dQuaternion rot (camMatrix);
    dVector origin (-40.0f, 5.0f, 0.0f, 0.0f);
    scene->SetCameraMatrix(rot, origin);
}



