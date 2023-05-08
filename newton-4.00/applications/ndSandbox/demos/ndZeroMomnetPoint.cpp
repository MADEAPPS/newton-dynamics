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
#include "ndUIEntity.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndLoadFbxMesh.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndAnimationPose.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"
#include "ndAnimationSequenceBase.h"
#include "ndAnimationSequencePlayer.h"


namespace ndZmp
{
	class ndModelUnicycle : public ndModelArticulation
	{
		public:
		ndModelUnicycle()
			:ndModelArticulation()
			,m_controlJoint(nullptr)
		{
		}

		void Update(ndWorld* const world, ndFloat32 timestep)
		{
			//static ndFloat32 tick = 0.0f;
			//tick += timestep;
			//ndFloat32 angle = 15.0f * ndDegreeToRad * ndSin(ndPi * tick / 0.5f);
			//m_controlJoint->SetOffsetAngle(angle);

			ndSkeletonContainer* const skeleton = m_rootNode->m_body->GetAsBodyDynamic()->GetSkeleton();
			ndAssert(skeleton);

			m_invDynamicsSolver.SolverBegin(skeleton, nullptr, 0, world, timestep);
			m_invDynamicsSolver.Solve();

			ndVector force(ndVector::m_zero);
			ndVector torque(ndVector::m_zero);
			for (ndNode* node = m_rootNode->IteratorFirst(); node; node = node->IteratorNext())
			{
				ndBodyDynamic* const body = node->m_body->GetAsBodyDynamic();
				force += m_invDynamicsSolver.GetBodyForce(body);
				torque += m_invDynamicsSolver.GetBodyTorque(body);
			}
			if (ndAbs(torque.m_z) > 0.1f)
			{
				ndTrace(("apply com correction %f\n", torque.m_z));
				//ndAssert(0);
			}
			m_invDynamicsSolver.SolverEnd();
		}

		ndJointHinge* m_controlJoint;
	};

	ndModelArticulation* BuildModel(ndDemoEntityManager* const scene, const ndMatrix& location)
	{
		ndModelUnicycle* const model = new ndModelUnicycle();

		ndFloat32 mass = 10.0f;
		ndFloat32 xSize = 0.25f;
		ndFloat32 ySize = 0.40f;
		ndFloat32 zSize = 0.30f;

		ndPhysicsWorld* const world = scene->GetWorld();
		
		// add hip body
		ndSharedPtr<ndBody> hipBody(world->GetBody(AddBox(scene, location, mass, xSize, ySize, zSize, "smilli.tga")));
		ndModelArticulation::ndNode* const modelRoot = model->AddRootBody(hipBody);

		ndMatrix matrix(hipBody->GetMatrix());
		matrix.m_posit.m_y += 0.5f;
		hipBody->SetMatrix(matrix);

		ndMatrix limbLocation(matrix);
		limbLocation.m_posit.m_z += zSize * 0.0f;
		limbLocation.m_posit.m_y -= ySize * 0.5f;
		limbLocation.m_posit.m_x += xSize * 0.5f * 0.0f;

		// make single leg
		ndFloat32 limbMass = 0.5f;
		ndFloat32 limbLength = 0.3f;
		ndFloat32 limbRadio = 0.025f;

		ndSharedPtr<ndBody> legBody(world->GetBody(AddCapsule(scene, ndGetIdentityMatrix(), limbMass, limbRadio, limbRadio, limbLength, "smilli.tga")));
		ndMatrix legLocation(ndRollMatrix(-90.0f * ndDegreeToRad) * limbLocation);
		legLocation.m_posit.m_y -= limbLength * 0.5f;
		legBody->SetMatrix(legLocation);
		ndMatrix legPivot(ndYawMatrix(90.0f * ndDegreeToRad) * legLocation);
		legPivot.m_posit.m_y += limbLength * 0.5f;
		ndSharedPtr<ndJointBilateralConstraint> legJoint(new ndJointHinge(legPivot, legBody->GetAsBodyKinematic(), modelRoot->m_body->GetAsBodyKinematic()));
		ndJointHinge* const hinge = (ndJointHinge*)*legJoint;
		hinge->SetAsSpringDamper(0.001f, 1500, 40.0f);
		model->m_controlJoint = hinge;

		// make wheel
		ndFloat32 wheelMass = 2.0f * limbMass;
		ndFloat32 wheelRadio = 4.0f * limbRadio;
		ndSharedPtr<ndBody> wheelBody(world->GetBody(AddSphere(scene, ndGetIdentityMatrix(), wheelMass, wheelRadio, "smilli.tga")));
		ndMatrix wheelMatrix(legPivot);
		wheelMatrix.m_posit.m_y -= limbLength;
		wheelBody->SetMatrix(wheelMatrix);
		ndSharedPtr<ndJointBilateralConstraint> wheelJoint(new ndJointSpherical(wheelMatrix, wheelBody->GetAsBodyKinematic(), legBody->GetAsBodyKinematic()));

		// teleport the model so that is on the floor
		ndMatrix probeMatrix(wheelMatrix);
		probeMatrix.m_posit.m_x += 1.0f;
		ndMatrix floor(FindFloor(*world, probeMatrix, wheelBody->GetAsBodyKinematic()->GetCollisionShape(), 20.0f));
		ndFloat32 dist = wheelMatrix.m_posit.m_y - floor.m_posit.m_y;

		//dist = 0;
		ndMatrix rootMatrix(modelRoot->m_body->GetMatrix());

		rootMatrix.m_posit.m_y -= dist;
		wheelMatrix.m_posit.m_y -= dist;
		legLocation.m_posit.m_y -= dist;

		legBody->SetMatrix(legLocation);
		wheelBody->SetMatrix(wheelMatrix);
		modelRoot->m_body->SetMatrix(rootMatrix);

		legBody->GetNotifyCallback()->OnTransform(0, legLocation);
		wheelBody->GetNotifyCallback()->OnTransform(0, wheelMatrix);
		modelRoot->m_body->GetNotifyCallback()->OnTransform(0, rootMatrix);

#if 0
		// add the joints manually, because on this model the wheel is not activate actuated.
		world->AddJoint(legJoint);
		world->AddJoint(wheelJoint);

		// add model limbs
		//ndModelArticulation::ndNode* const legLimb = model->AddLimb(modelRoot, legBody, legJoint);
		//model->AddLimb(legLimb, wheelBody, wheelJoint);
		model->AddLimb(modelRoot, legBody, legJoint);
#else
		// add model limbs
		ndModelArticulation::ndNode* const legLimb = model->AddLimb(modelRoot, legBody, legJoint);
		model->AddLimb(legLimb, wheelBody, wheelJoint);
#endif
		return model;
	}
}

using namespace ndZmp;
void ndZeroMomentPoint(ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildFloorBox(scene, ndGetIdentityMatrix());
	BuildFlatPlane(scene, true);
	
	ndWorld* const world = scene->GetWorld();
	ndMatrix matrix(ndYawMatrix(-0.0f * ndDegreeToRad));
	//matrix.m_posit.m_x = 1.0f;
	
	//ndZeroMomentModel* const robot = new ndZeroMomentModel(scene, matrix);
	//scene->SetSelectedModel(robot);
	//ndSharedPtr<ndModel> modelPtr(robot);
	//world->AddModel(modelPtr);
	//
	//ndModelUI* const quadrupedUI = new ndModelUI(scene, robot);
	//ndSharedPtr<ndUIEntity> quadrupedUIPtr(quadrupedUI);
	//scene->Set2DDisplayRenderFunction(quadrupedUIPtr);

	ndSharedPtr<ndModel> model(BuildModel(scene, matrix));
	scene->GetWorld()->AddModel(model);

	ndModelArticulation* const articulation = (ndModelArticulation*)model->GetAsModelArticulation();
	ndBodyKinematic* const rootBody = articulation->GetRoot()->m_body->GetAsBodyKinematic();
	ndSharedPtr<ndJointBilateralConstraint> fixJoint(new ndJointPlane(rootBody->GetMatrix().m_posit, ndVector(0.0f, 0.0f, 1.0f, 0.0f), rootBody, world->GetSentinelBody()));
	world->AddJoint(fixJoint);
	
	matrix.m_posit.m_x -= 0.0f;
	matrix.m_posit.m_y += 0.5f;
	matrix.m_posit.m_z += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 90.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);

	//ndFileFormatSave xxxxSave;
	//xxxxSave.SaveModels(scene->GetWorld(), "xxxx.nd");
	//ndFileFormatLoad xxxxLoad;
	//xxxxLoad.Load("xxxx.nd");
	//// offset bodies positions for calibration
	//const ndList<ndSharedPtr<ndBody>>& bodyList = xxxxLoad.GetBodyList();
	//for (ndList<ndSharedPtr<ndBody>>::ndNode* node = bodyList.GetFirst(); node; node = node->GetNext())
	//{
	//	ndSharedPtr<ndBody>& body = node->GetInfo();
	//	ndMatrix bodyMatrix (body->GetMatrix());
	//	bodyMatrix.m_posit.m_x += 1.0f;
	//	body->SetMatrix(bodyMatrix);
	//}
	//xxxxLoad.AddToWorld(scene->GetWorld());
}
