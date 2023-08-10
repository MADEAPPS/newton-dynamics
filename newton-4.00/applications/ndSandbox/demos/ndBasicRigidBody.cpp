/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndSkyBox.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

#if 0
void ndBasicRigidBody (ndDemoEntityManager* const scene)
{
	// build a floor
	BuildFloorBox(scene, ndGetIdentityMatrix());
	
	ndMatrix origin1(ndGetIdentityMatrix());

	//AddSphere(scene, origin1, 1.0f, 0.5f);
	//AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 1, 2, 7);
	AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 10, 10, 7);
	//AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 4, 4, 4);
	//AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 2, 2, 7);

	ndQuaternion rot;
	ndVector origin(-60.0f, 5.0f, 0.0f, 1.0f);
	scene->SetCameraMatrix(rot, origin);
}

#else

class ndCustomJointHinge : public ndIkJointHinge
{
    public:
    ndCustomJointHinge(const ndMatrix& frame, ndBodyKinematic* const child, ndBodyKinematic* const parent)
        :ndIkJointHinge(frame, child, parent)
    {
        SetAsSpringDamper(1.0e-3, 1000.0f, 10.0f);
    }

    void JacobianDerivative(ndConstraintDescritor& desc)
    {
        ndIkJointHinge::JacobianDerivative(desc);

        if (m_ikMode)
        {
            ndMatrix matrix0;
            ndMatrix matrix1;
            CalculateGlobalMatrix(matrix0, matrix1);
            SubmitSpringDamper(desc, matrix0, matrix1);
        }
    }
};

class TestIKSolver : public ndModelArticulation
{
    public:
    TestIKSolver(ndDemoEntityManager* const scene, ndMatrix& location)
        :ndModelArticulation()
    {
        ndFloat32 xSize = 0.25f;
        ndFloat32 ySize = 0.125f;
        ndFloat32 zSize = 0.15f;
        ndFloat32 cartMass = 5.0f;
        ndFloat32 poleMass = 10.0f;
        ndFloat32 poleLength = 0.4f;
        ndFloat32 poleRadio = 0.05f;
        ndPhysicsWorld* const world = scene->GetWorld();

        // make cart
        ndSharedPtr<ndBody> cartBody(world->GetBody(AddBox(scene, location, cartMass, xSize, ySize, zSize, "smilli.tga")));
        ndModelArticulation::ndNode* const modelRoot = AddRootBody(cartBody);
        ndMatrix matrix(cartBody->GetMatrix());
        matrix.m_posit.m_y += ySize / 2.0f;
        cartBody->SetMatrix(matrix);
        cartBody->GetAsBodyDynamic()->SetSleepAccel(cartBody->GetAsBodyDynamic()->GetSleepAccel() * ndFloat32(0.1f));

        // make pole leg
        matrix.m_posit.m_y += ySize / 2.0f;
        ndSharedPtr<ndBody> poleBody(world->GetBody(AddCapsule(scene, ndGetIdentityMatrix(), poleMass, poleRadio, poleRadio, poleLength, "smilli.tga")));
        ndMatrix poleLocation(ndRollMatrix(90.0f * ndDegreeToRad) * matrix);
        poleLocation.m_posit.m_y += poleLength * 0.5f;

        ndMatrix T(ndGetIdentityMatrix());
        T.m_posit.m_y = -poleLength * 0.5f;
        poleLocation = poleLocation * T * ndRollMatrix(30.0f * ndDegreeToRad) * T.OrthoInverse();

        poleBody->SetMatrix(poleLocation);
        poleBody->GetAsBodyDynamic()->SetSleepAccel(poleBody->GetAsBodyDynamic()->GetSleepAccel() * ndFloat32(0.1f));

        ndMatrix polePivot(ndYawMatrix(90.0f * ndDegreeToRad) * poleLocation);
        polePivot.m_posit -= poleLocation.m_front.Scale (poleLength * 0.5f);
        //ndSharedPtr<ndJointBilateralConstraint> poleJoint(new ndJointHinge(polePivot, poleBody->GetAsBodyKinematic(), modelRoot->m_body->GetAsBodyKinematic()));
        ndSharedPtr<ndJointBilateralConstraint> poleJoint(new ndCustomJointHinge(polePivot, poleBody->GetAsBodyKinematic(), modelRoot->m_body->GetAsBodyKinematic()));

        // add path to the model
        world->AddJoint(poleJoint);
        AddLimb(modelRoot, poleBody, poleJoint);

        // fix model to the world for first test.
        ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointFix6dof(cartBody->GetMatrix(), cartBody->GetAsBodyDynamic(), world->GetSentinelBody()));
        world->AddJoint(fixJoint);
    }

    void Update(ndWorld* const world, ndFloat32 timestep)
    {
        ndBodyKinematic* const rootBody = GetRoot()->m_body->GetAsBodyKinematic();
        ndSkeletonContainer* const skeleton = rootBody->GetSkeleton();
        if (!skeleton)
        {
            return;
        }

        ndIkSolver* const invDynamicsSolver = (ndIkSolver*)&m_invDynamicsSolver;
        invDynamicsSolver->SolverBegin(skeleton, nullptr, 0, world, timestep);
        invDynamicsSolver->Solve();
        invDynamicsSolver->SolverEnd();
    }
};

void ndBasicRigidBody(ndDemoEntityManager* const scene)
{
    BuildFlatPlane(scene, true);

    ndSetRandSeed(42);
    ndWorld* const world = scene->GetWorld();
    ndMatrix matrix(ndYawMatrix(-0.0f * ndDegreeToRad));

    ndSharedPtr<ndModel> model(new TestIKSolver(scene, matrix));
    world->AddModel(model);

    matrix.m_posit.m_x -= 0.0f;
    matrix.m_posit.m_y += 0.5f;
    matrix.m_posit.m_z += 2.0f;
    ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
    scene->SetCameraMatrix(rotation, matrix.m_posit);
}

#endif
