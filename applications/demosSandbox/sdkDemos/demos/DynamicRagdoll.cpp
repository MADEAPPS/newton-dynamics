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
#include "DemoMesh.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "TargaToOpenGl.h"
#include "DemoEntityManager.h"
#include "dCustomBallAndSocket.h"
#include "DebugDisplay.h"
#include "HeightFieldPrimitive.h"


class DynamicRagdollManager: public dCustomActiveCharacterManager
{
	public: 
	DynamicRagdollManager (DemoEntityManager* const scene)
		:dCustomActiveCharacterManager (scene->GetNewton())
	{
		// create a material for early collision culling
		m_material = NewtonMaterialCreateGroupID(scene->GetNewton());
		//NewtonMaterialSetCallbackUserData (scene->GetNewton(), m_material, m_material, this);
		//NewtonMaterialSetCollisionCallback (scene->GetNewton(), m_material, m_material, OnBoneAABBOverlap, NULL);
		scene->Set2DDisplayRenderFunction (DemoHelp, NULL, this);
	}

	static void DemoHelp (DemoEntityManager* const scene, void* const context)
	{
//		SuperCarVehicleControllerManager* const me = (SuperCarVehicleControllerManager*) context;
//		me->RenderVehicleHud (scene, lineNumber);
	}

#if 0
	static int OnBoneAABBOverlap (const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex)
	{
		dAssert (0);

		NewtonCollision* const collision0 = NewtonBodyGetCollision(body0);
		NewtonCollision* const collision1 = NewtonBodyGetCollision(body1);
		dCustomActiveCharacterController::dSkeletonBone* const bone0 = (dCustomActiveCharacterController::dSkeletonBone*)NewtonCollisionGetUserData (collision0);
		dCustomActiveCharacterController::dSkeletonBone* const bone1 = (dCustomActiveCharacterController::dSkeletonBone*)NewtonCollisionGetUserData (collision1);

		dAssert (bone0);
		dAssert (bone1);
		if (bone0->m_myController && bone1->m_myController) {
			return bone0->m_myController->SelfCollisionTest (bone0, bone1) ? 1 : 0;
		}
*/
		return 1;
	}

	virtual void OnUpdateTransform (const dCustomActiveCharacterController::dSkeletonBone* const bone, const dMatrix& localMatrix) const
	{
		DemoEntity* const ent = (DemoEntity*) NewtonBodyGetUserData(bone->m_body);
		DemoEntityManager* const scene = (DemoEntityManager*) NewtonWorldGetUserData(NewtonBodyGetWorld(bone->m_body));
		
		dQuaternion rot (localMatrix);
		ent->SetMatrix (*scene, rot, localMatrix.m_posit);

		dCustomActiveCharacterControllerManager::OnUpdateTransform (bone, localMatrix);
	}

#endif



	class MySaveLoad: public dCustomJointSaveLoad
	{
		public:
		MySaveLoad(NewtonWorld* const world, FILE* const file, int material)
			:dCustomJointSaveLoad(world, file)
			,m_material(material)
		{
		}

		const char* GetUserDataName(const NewtonBody* const body) const
		{
			DemoEntity* const entity = (DemoEntity*)NewtonBodyGetUserData(body);
			return entity ? entity->GetName().GetStr() : NULL;
		}

		virtual const void InitRigiBody(const NewtonBody* const body, const char* const bodyName) const
		{
			dMatrix matrix;
			DemoEntityManager* const scene = (DemoEntityManager*)NewtonWorldGetUserData(NewtonBodyGetWorld(body));

			NewtonCollision* const collision = NewtonBodyGetCollision(body);
			DemoMesh* const mesh = new DemoMesh("ragdoll", collision, "smilli.tga", "smilli.tga", "smilli.tga");

			NewtonBodyGetMatrix(body, &matrix[0][0]);
			DemoEntity* const entity = new DemoEntity(matrix, NULL);
			entity->SetNameID (bodyName);
			entity->SetMesh(mesh, dGetIdentityMatrix());
			scene->Append(entity);
			mesh->Release();

			// save the pointer to the graphic object with the body.
			NewtonBodySetUserData(body, entity);

			// assign the wood id
			NewtonBodySetMaterialGroupID(body, m_material);

			//set continuous collision mode
			//NewtonBodySetContinuousCollisionMode (rigidBody, continueCollisionMode);

			// set a destructor for this rigid body
			NewtonBodySetDestructorCallback(body, PhysicsBodyDestructor);

			// set the transform call back function
			NewtonBodySetTransformCallback(body, DemoEntity::TransformCallback);

			// set the force and torque call back function
			NewtonBodySetForceAndTorqueCallback(body, PhysicsApplyGravityForce);
		}

		int m_material;
	};
	

	NewtonBody* ParseRagdollFile(const char* const name, dCustomActiveCharacterController* const controller)
	{
		char fileName[2048];
		dGetWorkingFileName(name, fileName);

		char* const oldloc = setlocale(LC_ALL, 0);
		setlocale(LC_ALL, "C");
		FILE* const imputFile = fopen(fileName, "rt");
		dAssert(imputFile);

		MySaveLoad saveLoad(GetWorld(), imputFile, m_material);
		NewtonBody* const rootBone = saveLoad.Load ();

		fclose(imputFile);
		setlocale(LC_ALL, oldloc);

		return rootBone;
	}


	void CreateBasicGait (const dVector& posit, dFloat angle)
	{
		dCustomActiveCharacterController* const controller = CreateTransformController();
		NewtonBody* const root = ParseRagdollFile("balancingGait.txt", controller);

		dMatrix bodyMatrix;
		NewtonBodyGetMatrix (root, &bodyMatrix[0][0]);

		bodyMatrix = dYawMatrix(angle) * bodyMatrix;
		bodyMatrix.m_posit = posit;
		bodyMatrix.m_posit.m_w = 1.0f;
		NewtonBodySetMatrixRecursive (root, &bodyMatrix[0][0]);
/*
return;
dCustomHinge* xxx = new dCustomHinge(bodyMatrix, root);
xxx->SetFriction (20000.0f);
xxx->SetLimits(0.0f, 0.0f);

		void* const rootNode = controller->AddRoot(root);

		int stack = 1;
		void* stackPool[128];
		stackPool[0] = rootNode;

		dString pevisEffector ("Bip01_Pelvis"); 
		dString leftFootEffector ("Bip01_L_Foot");
		dString rightFootEffector ("Bip01_R_Foot");

		dTree<int, NewtonJoint*> filter; 
		while (stack) {
			stack --;
			void* const node = stackPool[stack];
			NewtonBody* const body = controller->GetBody(node);
			DemoEntity* const entity = (DemoEntity*) NewtonBodyGetUserData(body);


			// add the end effectors.
			if (entity->GetName() == pevisEffector) {
				// root effect has it pivot at the center of mass 
				dVector com;
				dMatrix matrix;
				NewtonBodyGetMatrix(body, &matrix[0][0]);

				NewtonBodyGetCentreOfMass(body, &com[0]);
				matrix.m_posit = matrix.TransformVector(com);
				controller->AddEndEffector(node, matrix);

			} else if (entity->GetName() == leftFootEffector) {

				// all other effector are centered and oriented a the joint pivot 
				dMatrix matrix;
				NewtonBodyGetMatrix(body, &matrix[0][0]);

				dCustomJoint* const joint = controller->GetJoint(node);
				dAssert (joint->GetBody0() == body);
				matrix = joint->GetMatrix0() * matrix;
				dCustomRagdollMotor_EndEffector* const effector = controller->AddEndEffector(node, matrix);
dCustomKinematicController* xxx = new dCustomKinematicController (body, matrix);

				matrix.m_posit.m_z -= 0.4f;
				matrix.m_posit.m_x -= 0.2f;
				matrix.m_posit.m_y += 0.0f;
				effector->SetTargetMatrix(matrix);

xxx->SetPickMode(0);
xxx->SetTargetMatrix(matrix);
xxx->SetMaxLinearFriction (5000.0f);
xxx->SetMaxAngularFriction (5000.0f);
				

			} else if (entity->GetName() == rightFootEffector) {
				// all other effector are centered and oriented a the joint pivot 
				dMatrix matrix;
				NewtonBodyGetMatrix(body, &matrix[0][0]);

				dCustomJoint* const joint = controller->GetJoint(node);
				dAssert(joint->GetBody0() == body);
				matrix = joint->GetMatrix0() * matrix;
				dCustomRagdollMotor_EndEffector* const effector = controller->AddEndEffector(node, matrix);
dCustomKinematicController* xxx = new dCustomKinematicController (body, matrix);

				matrix.m_posit.m_z += 0.4f;
				matrix.m_posit.m_x += 0.2f;
				matrix.m_posit.m_y += 0.0f;
				effector->SetTargetMatrix(matrix);

xxx->SetPickMode(0);
xxx->SetTargetMatrix(matrix);
xxx->SetMaxLinearFriction(5000.0f);
xxx->SetMaxAngularFriction(5000.0f);
			}


			for (NewtonJoint* newtonJoint = NewtonBodyGetFirstJoint(body); newtonJoint; newtonJoint = NewtonBodyGetNextJoint(body, newtonJoint)) {
				if (!filter.Find(newtonJoint)) {
					filter.Insert(newtonJoint);
					dCustomJoint* const customJoint = (dCustomJoint*)NewtonJointGetUserData(newtonJoint);
					if (customJoint->IsType(dCustomRagdollMotor::GetType())) {
						dCustomRagdollMotor* const ragDollMotor = (dCustomRagdollMotor*)customJoint;
						void* const bone = controller->AddBone(ragDollMotor, node);
						//ragDollMotor->SetMode(true);
						stackPool[stack] = bone;
						stack ++;
					}
				}
			}
		}

		controller->Finalize();
*/
	}

	int m_material;
};


void DynamicRagDoll (DemoEntityManager* const scene)
{
	// load the sky box
	scene->CreateSkyBox();
	CreateLevelMesh (scene, "flatPlane.ngd", true);
	//CreateHeightFieldTerrain(scene, HEIGHTFIELD_DEFAULT_SIZE, HEIGHTFIELD_DEFAULT_CELLSIZE, 1.5f, 0.2f, 200.0f, -50.0f);

	// load a skeleton mesh for using as a ragdoll manager
//	DemoEntity ragDollModel(dGetIdentityMatrix(), NULL);
//	ragDollModel.LoadNGD_mesh ("skeleton.ngd", scene->GetNewton());
//	ragDollModel.LoadNGD_mesh ("gymnast.ngd", scene->GetNewton());

	//  create a skeletal transform controller for controlling rag doll
	DynamicRagdollManager* const manager = new DynamicRagdollManager (scene);

	NewtonWorld* const world = scene->GetNewton();
	//dMatrix matrix (dGetIdentityMatrix());

//	dVector origin (-10.0f, 1.0f, 0.0f, 1.0f);
	dVector origin (FindFloor (world, dVector (-4.0f, 50.0f, 0.0f, 1.0f), 2.0f * 50.0f));

//	int count = 10;
	int count = 1;
	for (int x = 0; x < count; x ++) {
		for (int z = 0; z < count; z ++) {
			dVector p (origin + dVector ((x - count / 2) * 3.0f - count / 2, 0.0f, (z - count / 2) * 3.0f, 0.0f));
			p = FindFloor (world, p, 100.0f);
			p.m_y += 0.9f;
			manager->CreateBasicGait (p, 0.0f);
		}
	}
/*
	const int defaultMaterialID = NewtonMaterialGetDefaultGroupID(scene->GetNewton());
	const dVector location(origin);
	const dVector size(0.25f, 0.25f, 0.375f, 0.0f);
	const int count1 = 5;
	const dMatrix shapeOffsetMatrix(dGetIdentityMatrix());
	AddPrimitiveArray(scene, 10.0f, location, size, count1, count1, 5.0f, _BOX_PRIMITIVE, defaultMaterialID, shapeOffsetMatrix);
*/

	origin.m_x = -6.0f;
//	origin.m_x -= 2.0f;
	origin.m_y  = 0.5f;
	dQuaternion rot;
	scene->SetCameraMatrix(rot, origin);
}



