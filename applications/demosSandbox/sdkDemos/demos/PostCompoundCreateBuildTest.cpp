#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "../DemoEntityManager.h"
#include "../DemoCamera.h"
#include "PhysicsUtils.h"
#include "../DemoMesh.h"
#include "../toolBox/OpenGlUtil.h"


NewtonBody* createTestCompound (DemoEntityManager* const scene, dFloat mass, const dMatrix& matrix, const dVector& size, int materialId)
{
	NewtonWorld* const world = scene->GetNewton();
	
	NewtonCollision* collision = NewtonCreateCompoundCollision(world, 0);

	//create the rigid body
	NewtonBody* const rigidBody = NewtonCreateDynamicBody (world, collision, &matrix[0][0]);
	NewtonDestroyCollision(collision);

	collision = NewtonBodyGetCollision(rigidBody);

	NewtonCompoundCollisionBeginAddRemove(collision);//BEGIN

	NewtonCollision* sub_col = NewtonCreateBox(world, size.m_x,size.m_y,size.m_z,0,0);
	NewtonCompoundCollisionAddSubCollision(collision, sub_col);
	NewtonDestroyCollision(sub_col);

	NewtonCompoundCollisionEndAddRemove(collision);	//END

	// add this line because after you change the collision of a body, 
	// if the bod is static, the broad phase does not knows that a child shape shape, so it has no reason to update the 
	// node location in the database. this will not be need it when the collision instance is the representative of the 
	// body in the broaphase, but for now this si a work around
	NewtonBodySetMatrix(rigidBody, &matrix[0][0]);

	DemoMesh* const geometry = new DemoMesh("cylinder_1", collision, "smilli.tga", "smilli.tga", "smilli.tga");

	// add an new entity to the world
	DemoEntity* const entity = new DemoEntity(matrix, NULL);
	scene->Append (entity);
	entity->SetMesh(geometry);

	geometry->Release();

	// calculate the moment of inertia and the relative center of mass of the solid
//	dVector origin;
//	dVector inertia;
//	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	

//	dFloat Ixx = mass * inertia[0];
//	dFloat Iyy = mass * inertia[1];
//	dFloat Izz = mass * inertia[2];
//	// set the correct center of gravity for this body
//	NewtonBodySetCentreOfMass (rigidBody, &origin[0]);

	// set the mass matrix
//	NewtonBodySetMassMatrix (rigidBody, mass, Ixx, Iyy, Izz);
	NewtonBodySetMassProperties (rigidBody, mass, collision);


	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (rigidBody, entity);

	// assign the wood id
	NewtonBodySetMaterialGroupID (rigidBody, materialId);

	//  set continue collision mode
	//	NewtonBodySetContinuousCollisionMode (rigidBody, continueCollisionMode);

	// set a destructor for this rigid body
	NewtonBodySetDestructorCallback (rigidBody, PhysicsBodyDestructor);

	// set the transform call back function
	NewtonBodySetTransformCallback (rigidBody, DemoEntity::TransformCallback);

	// set the force and torque call back function
	NewtonBodySetForceAndTorqueCallback (rigidBody, PhysicsApplyGravityForce);

	// set the matrix for both the rigid body and the graphic body
	//NewtonBodySetMatrix (rigidBody, &matrix[0][0]);
	//PhysicsSetTransform (rigidBody, &matrix[0][0], 0);

	//dVector xxx (0, -9.8f * mass, 0.0f, 0.0f);
	//NewtonBodySetForce (rigidBody, &xxx[0]);

	// force the body to be active of inactive
	//	NewtonBodySetAutoSleep (rigidBody, sleepMode);
	return rigidBody;

}

void PostCompoundCreateBuildTest(DemoEntityManager* const scene)
{
	// load the skybox
	scene->CreateSkyBox();


	// customize the scene after loading
	// set a user friction variable in the body for variable friction demos
	// later this will be done using LUA script
//	NewtonWorld* const world = scene->GetNewton();
	dMatrix offsetMatrix (GetIdentityMatrix());

//	int materialID = NewtonMaterialGetDefaultGroupID (world);

	createTestCompound(scene, 0, dMatrix(dQuaternion(1,0,0,0),dVector(0,0,0)), dVector(10,1,10), 0);
	for(int i=-4;i<4;i++)
		createTestCompound(scene, 10, dMatrix(dQuaternion(1,0,0,0),dVector(i * 1.1f,10,0)), dVector(1,1,1), 0);

	// place camera into position
	//dMatrix camMatrix (dYawMatrix(90.0f * 3.1416f /180.0f));
	dMatrix camMatrix (GetIdentityMatrix());
	dQuaternion rot (camMatrix);
	dVector origin (-15.0f, 0.0f, 0.0f, 0.0f);
	scene->SetCameraMatrix(rot, origin);

}