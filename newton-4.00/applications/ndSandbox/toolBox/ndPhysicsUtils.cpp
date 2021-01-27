/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software./Users/juliojerez/Desktop/newton-dynamics/applications/demosSandbox/sdkDemos/toolBox/PhysicsUtils.cpp
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndDemoMesh.h"
#include "ndDemoEntity.h"
#include "ndPhysicsUtils.h"
#include "ndDemoEntityManager.h"
#include "ndOpenGlUtil.h"
#include "ndDebugDisplay.h"
#include "ndHighResolutionTimer.h"

#ifdef DEMO_CHECK_ASYN_UPDATE
dInt32 g_checkAsyncUpdate = 1;
#endif

#if 0
//const D_MESH_HEADER	"Newton Mesh"
static const char* D_MESH_HEADER = "Newton Mesh";


dVector ForceBetweenBody (ndBodyKinematic* const body0, ndBodyKinematic* const body1)
{
	dVector reactionforce (0.0f);
	for (NewtonJoint* joint = ndBodyKinematicGetFirstContactJoint(body0); joint; joint = ndBodyKinematicGetNextContactJoint(body0, joint)) {
		if (NewtonJointIsActive(joint) &&  (NewtonJointGetBody0(joint) == body0) || (NewtonJointGetBody0(joint) == body1)) {
			for (void* contact = NewtonContactJointGetFirstContact (joint); contact; contact = NewtonContactJointGetNextContact (joint, contact)) {
				dVector point(0.0f);
				dVector normal(0.0f);	
				dVector contactForce(0.0f);
				NewtonMaterial* const material = NewtonContactGetMaterial (contact);
				NewtonMaterialGetContactPositionAndNormal (material, body0, &point.m_x, &normal.m_x);
				NewtonMaterialGetContactForce(material, body0, &contactForce[0]);
				reactionforce += contactForce;
			}
			break;
		}
	}
	return reactionforce;
}

void GetConnectedBodiesByJoints (ndBodyKinematic* const body) 
{
	for (NewtonJoint* joint = ndBodyKinematicGetFirstJoint(body); joint; joint = ndBodyKinematicGetNextJoint(body, joint)) {
		dCustomJoint* const customJoint = (dCustomJoint*) NewtonJointGetUserData(joint);
		ndBodyKinematic* const body0 = customJoint->GetBody0();
		ndBodyKinematic* const body1 = customJoint->GetBody1();
		ndBodyKinematic* const otherBody = (body0 == body) ? body1 : body0;
		// do whatever you need to do here
		ndBodyKinematicSetFreezeState (otherBody, 0);
	}
}
	
class RayCastPlacementData
{
	public:
	RayCastPlacementData()
		:m_param(1.2f)
	{
	}
	dVector m_normal;
	dFloat32 m_param;
};

static dFloat32 RayCastPlacement (const ndBodyKinematic* const body, const NewtonCollision* const collisionHit, const dFloat32* const contact, const dFloat32* const normal, dLong collisionID, void* const userData, dFloat32 intersetParam)
{
	// if the collision has a parent, the this can be it si a sub shape of a compound collision 
	const NewtonCollision* const parent = NewtonCollisionGetParentInstance(collisionHit);
	if (parent) {
		// you can use this to filter sub collision shapes.  
		dAssert (NewtonCollisionGetSubCollisionHandle (collisionHit));
	}

	RayCastPlacementData* const paramPtr = (RayCastPlacementData*)userData;
	if (intersetParam < paramPtr->m_param) {
		paramPtr->m_param = intersetParam;
		paramPtr->m_normal = dVector(normal[0], normal[1], normal[2], 0.0f); 
	}
	return paramPtr->m_param;
}

static unsigned RayPrefilter (const ndBodyKinematic* const body, const NewtonCollision* const collision, void* const userData)
{
	// if the collision has a parent, the this can be it si a sub shape of a compound collision 
	const NewtonCollision* const parent = NewtonCollisionGetParentInstance(collision);
	if (parent) {
		// you can use this to filter sub collision shapes.  
		dAssert (NewtonCollisionGetSubCollisionHandle (collision));
	}

	return 1;
}




#if 0

static unsigned ConvexCastCallback (const ndBodyKinematic* body, const NewtonCollision* collision, void* userData)
{
	ndBodyKinematic* me = (ndBodyKinematic*) userData;
	return (me == body) ? 0 : 1;
}

void ConvexCastPlacement (ndBodyKinematic* body)
{
	dFloat param;
	dMatrix matrix;
	ndWorld* world;
	NewtonCollision* collision;
	ndWorldConvexCastReturnInfo info[16];


	ndBodyKinematicGetMatrix (body, &matrix[0][0]);

	matrix.m_posit.m_y += 40.0f;
	dVector p (matrix.m_posit);
	p.m_y -= 80.0f;

	world = ndBodyKinematicGetWorld(body);
	collision = ndBodyKinematicGetCollision(body);
	ndWorldConvexCast (world, &matrix[0][0], &p[0], collision, &param, body, ConvexCastCallback, info, 16, 0);
	dAssert (param < 1.0f);

	matrix.m_posit.m_y += (p.m_y - matrix.m_posit.m_y) * param;

	ndBodyKinematicSetMatrix(body, &matrix[0][0]);
}


void ShowJointInfo(const NewtonCustomJoint* joint)
{
	NewtonJointRecord info;

	if (showContacts) {
		joint->GetInfo (&info);

		dMatrix bodyMatrix0;
		ndBodyKinematicGetMatrix (info.m_attachBody_0, &bodyMatrix0[0][0]);
		dMatrix matrix0 (*((dMatrix*) &info.m_attachmenMatrix_0[0]));
		matrix0 = matrix0 * bodyMatrix0;
		DebugDrawLine (matrix0.m_posit, matrix0.m_posit + matrix0.m_front, dVector (1, 0, 0));
		DebugDrawLine (matrix0.m_posit, matrix0.m_posit + matrix0.m_up, dVector (0, 1, 0));
		DebugDrawLine (matrix0.m_posit, matrix0.m_posit + matrix0.m_right, dVector (0, 0, 1));


		dMatrix bodyMatrix1;
		ndBodyKinematicGetMatrix (info.m_attachBody_1, &bodyMatrix1[0][0]);
		dMatrix matrix1 (*((dMatrix*) &info.m_attachmenMatrix_1[0]));
		matrix1 = matrix1 * bodyMatrix1;
		DebugDrawLine (matrix1.m_posit, matrix1.m_posit + matrix1.m_front, dVector (1, 0, 0));
		DebugDrawLine (matrix1.m_posit, matrix1.m_posit + matrix1.m_up,	   dVector (0, 1, 0));
		DebugDrawLine (matrix1.m_posit, matrix1.m_posit + matrix1.m_right, dVector (0, 0, 1));

	}

}


static void ShowMeshCollidingFaces (
	const ndBodyKinematic* bodyWithTreeCollision, 
	const ndBodyKinematic* body, 
	dInt32 faceID, 
	dInt32 vertexCount, 
	const dFloat* vertex, 
	dInt32 vertexstrideInBytes)
{

	// we are coping data to and array of memory, another call back may be doing the same thing
	// here fore we need to avoid race conditions
	ndWorldCriticalSectionLock (ndBodyKinematicGetWorld (bodyWithTreeCollision));

	dVector face[64];
	dInt32 stride = vertexstrideInBytes / sizeof (dFloat);
	for (dInt32 j = 0; j < vertexCount; j ++) {
		face [j] = dVector (vertex[j * stride + 0], vertex[j * stride + 1] , vertex[j * stride + 2]);
	}
	DebugDrawPolygon (vertexCount, face);

	// unlock the critical section
	ndWorldCriticalSectionUnlock (ndBodyKinematicGetWorld (bodyWithTreeCollision));
}


void SetShowMeshCollision (SceneManager& me, dInt32 mode)
{
	NewtonTreeCollisionCallback showFaceCallback;

	showFaceCallback = nullptr;
	if (mode) {
		showFaceCallback = ShowMeshCollidingFaces;
	}

	// iterate the world
	for (const ndBodyKinematic* body = ndWorldGetFirstBody (me.m_world); body; body = ndWorldGetNextBody (me.m_world, body)) {
		NewtonCollision* collision;
		NewtonCollisionInfoRecord info;

		collision = ndBodyKinematicGetCollision (body);
		NewtonCollisionGetInfo (collision, &info);

		switch (info.m_collisionType) 
		{
			case SERIALIZE_ID_TREE:
			case SERIALIZE_ID_SCENE:
			case SERIALIZE_ID_USERMESH:
			case SERIALIZE_ID_HEIGHTFIELD:
			{
				NewtonStaticCollisionSetDebugCallback (collision, showFaceCallback);
				break;
			}

			default: 
				break;
		}
	}
}

void SetShowIslands (SceneManager& me, dInt32 mode)
{
	showIslans = mode;
}

dInt32 PhysicsIslandUpdate (const ndWorld* world, const void* islandHandle, dInt32 bodyCount)
{
	if (showIslans) {
		dVector minAABB ( 1.0e10f,  1.0e10f,  1.0e10f, 0.0f);
		dVector maxAABB (-1.0e10f, -1.0e10f, -1.0e10f, 0.0f);
		for (dInt32 i = 0; i < bodyCount; i ++) {
			dVector p0;
			dVector p1;

#if 0
			// show the engine loose aabb
			NewtonIslandGetBodyAABB (islandHandle, i, &p0[0], &p1[0]);
#else
			// calculate the shape aabb

			dMatrix matrix;
			ndBodyKinematic* body;
			NewtonCollision *collision;

			body = NewtonIslandGetBody (islandHandle, i);
			collision = ndBodyKinematicGetCollision (body);
			ndBodyKinematicGetMatrix (body, &matrix[0][0]);
			CalculateAABB (collision, matrix, p0, p1);
#endif
			for (dInt32 j = 0; j < 3; j ++ ) {
				minAABB[j] = p0[j] < minAABB[j] ? p0[j] : minAABB[j];
				maxAABB[j] = p1[j] > maxAABB[j] ? p1[j] : maxAABB[j];
			}
		}
		DebugDrawAABB (minAABB, maxAABB);
	}

	//	g_activeBodies += bodyCount;
	return 1;
}

void GenericContactProcess (const NewtonJoint* contactJoint, dFloat32 timestep, dInt32 threadIndex)
{
	dInt32 isHightField;
	ndBodyKinematic* body;
	NewtonCollision* collision;
	NewtonCollisionInfoRecord info;

	isHightField = 1;
	body = NewtonJointGetBody0 (contactJoint);
	collision = ndBodyKinematicGetCollision(body);
	NewtonCollisionGetInfo(collision, &info);
	if (info.m_collisionType != SERIALIZE_ID_HEIGHTFIELD) {
		body = NewtonJointGetBody1 (contactJoint);
		collision = ndBodyKinematicGetCollision(body);
		NewtonCollisionGetInfo(collision, &info);
		isHightField  = (info.m_collisionType == SERIALIZE_ID_HEIGHTFIELD); 
	}

	#define HOLE_IN_TERRAIN 10
	if (isHightField) {
		void* nextContact;
		for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = nextContact) {
			dInt32 faceID;
			NewtonMaterial* material;

			nextContact = NewtonContactJointGetNextContact (contactJoint, contact);

			material = NewtonContactGetMaterial (contact);
			faceID = NewtonMaterialGetContactFaceAttribute (material);
			if (faceID == HOLE_INTERRAIN) {
				NewtonContactJointRemoveContact (contactJoint, contact); 
			}
		}
	}
}


void GetForceOnStaticBody (ndBodyKinematic* body, ndBodyKinematic* staticBody)
{
	for (NewtonJoint* joint = ndBodyKinematicGetFirstContactJoint (body); joint; joint = ndBodyKinematicGetNextContactJoint (body, joint)) {
		ndBodyKinematic* body0;
		ndBodyKinematic* body1;

		body0 = NewtonJointGetBody0(joint);
		body1 = NewtonJointGetBody1(joint);
		if ((body0 == staticBody) || (body1 == staticBody)) {

			for (void* contact = NewtonContactJointGetFirstContact (joint); contact; contact = NewtonContactJointGetNextContact (joint, contact)) {
	
				dVector point(0.0f);
				dVector normal(0.0f);	
				dFloat32 forceMag;
				NewtonMaterial* material;

				material = NewtonContactGetMaterial (contact);
				
				NewtonMaterialGetContactForce (material, &forceMag);
				NewtonMaterialGetContactPositionAndNormal (material, &point.m_x, &normal.m_x);

				dVector force (normal.Scale (-forceMag));

				// do wherever you want withteh force
			}
		}
	}
}




static void ExtrudeFaces (void* userData, dInt32 vertexCount, const dFloat32* faceVertec, dInt32 id)
{
	dFloat32 OFFSET = 0.1f;
	dFloat32 face[32][10];

	NewtonMesh* mesh = (NewtonMesh*) userData;

	// calculate the face normal
	dVector normal (0.0f);
	dVector p0 (faceVertec[0 * 3 + 0], faceVertec[0 * 3 + 1], faceVertec[0 * 3 + 2]);
	dVector p1 (faceVertec[1 * 3 + 0], faceVertec[1 * 3 + 1], faceVertec[1 * 3 + 2]);

	dVector e0 (p1 - p0);
	for (dInt32 i = 2; i < vertexCount; i ++) {
		dVector p2 (faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
		dVector e1 (p2 - p0);

		normal += e0 * e1;
		e0 = e1;
	}
	normal = normal.Scale (1.0f / dSqrt (normal % normal));

	dVector displacemnet (normal.Scale (OFFSET));

	// add the face displace by some offset
	for (dInt32 i = 0; i < vertexCount; i ++) {
		dVector p1 (faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
		p1 += displacemnet;

		face[i][0] = p1.m_x; 
		face[i][1] = p1.m_y;  
		face[i][2] = p1.m_z;   

		face[i][3] = normal.m_x; 
		face[i][4] = normal.m_y;  
		face[i][5] = normal.m_z;  

		face[i][6] = 0.0f; 
		face[i][7] = 0.0f;  
		face[i][8] = 0.0f;  
		face[i][9] = 0.0f;  
	}
	
	// add the face
	NewtonMeshAddFace (mesh, vertexCount, &face[0][0], 10 * sizeof (dFloat32), id);


	// now add on face walk the perimeter and add a rivet face
	dVector q0 (faceVertec[(vertexCount - 1) * 3 + 0], faceVertec[(vertexCount - 1) * 3 + 1], faceVertec[(vertexCount - 1) * 3 + 2]);
	q0 += displacemnet;
	for (dInt32 i = 0; i < vertexCount; i ++) {
		dVector q1 (faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
		q1 += displacemnet;

		// calculate the river normal
		dVector edge (q1 - q0);
		dVector n (edge * normal);
		n = n.Scale (1.0f / sqrtf (n % n));

		// build a quad to serve a the face between the two parellel faces
		face[0][0] = q0.m_x; 
		face[0][1] = q0.m_y;  
		face[0][2] = q0.m_z;   
		face[0][3] = n.m_x; 
		face[0][4] = n.m_y;  
		face[0][5] = n.m_z;  
		face[0][6] = 0.0f; 
		face[0][7] = 0.0f;  
		face[0][8] = 0.0f;  
		face[0][9] = 0.0f;  

		face[1][0] = q1.m_x; 
		face[1][1] = q1.m_y;  
		face[1][2] = q1.m_z;   
		face[1][3] = n.m_x; 
		face[1][4] = n.m_y;  
		face[1][5] = n.m_z;  
		face[1][6] = 0.0f; 
		face[1][7] = 0.0f;  
		face[1][8] = 0.0f;  
		face[1][9] = 0.0f;  

		face[2][0] = q1.m_x - displacemnet.m_x; 
		face[2][1] = q1.m_y - displacemnet.m_y;  
		face[2][2] = q1.m_z - displacemnet.m_z;   
		face[2][3] = n.m_x; 
		face[2][4] = n.m_y;  
		face[2][5] = n.m_z;  
		face[2][6] = 0.0f; 
		face[2][7] = 0.0f;  
		face[2][8] = 0.0f;  
		face[2][9] = 0.0f;  

		face[3][0] = q0.m_x - displacemnet.m_x; 
		face[3][1] = q0.m_y - displacemnet.m_y;  
		face[3][2] = q0.m_z - displacemnet.m_z;   
		face[3][3] = n.m_x; 
		face[3][4] = n.m_y;  
		face[3][5] = n.m_z;  
		face[3][6] = 0.0f; 
		face[3][7] = 0.0f;  
		face[3][8] = 0.0f;  
		face[3][9] = 0.0f;  

		// save the first point for the next rivet
		q0 = q1;

		// add this face to the mesh
		NewtonMeshAddFace (mesh, 4, &face[0][0], 10 * sizeof (dFloat32), id);
	}
}


NewtonMesh* CreateCollisionTreeDoubleFaces (ndWorld* world, NewtonCollision* optimizedDoubelFacesTree)
{
	NewtonMesh* mesh = NewtonMeshCreate(world);
	dMatrix matrix (dGetIdentityMatrix());

	NewtonMeshBeginBuild(mesh);
	NewtonCollisionForEachPolygonDo (optimizedDoubelFacesTree, &matrix[0][0], ExtrudeFaces, mesh);	
	NewtonMeshEndBuild(mesh);

	return mesh;
}

#endif

// return the collision joint, if the body collide
NewtonJoint* CheckIfBodiesCollide (ndBodyKinematic* const body0, ndBodyKinematic* const body1)
{
	for (NewtonJoint* joint = ndBodyKinematicGetFirstContactJoint (body0); joint; joint = ndBodyKinematicGetNextContactJoint (body0, joint)) {
		if ((NewtonJointGetBody0(joint) == body1) || (NewtonJointGetBody1(joint) == body1)) {
			return joint;
		}
	}
	return nullptr;
}


//to get the collision points
void HandlecollisionPoints (NewtonJoint* const contactjoint)
{
	ndBodyKinematic* const body0 = NewtonJointGetBody0(contactjoint);
	ndBodyKinematic* const body1 = NewtonJointGetBody1(contactjoint);
	for (void* contact = NewtonContactJointGetFirstContact (contactjoint); contact; contact = NewtonContactJointGetNextContact (contactjoint, contact)) {

		NewtonMaterial* material = NewtonContactGetMaterial (contact);

		// do whatever you want here
		//dFloat32 forceMag;
		dVector point(0.0f);
		dVector normal(0.0f);	
		//NewtonMaterialGetContactForce (material, &forceMag);
		NewtonMaterialGetContactPositionAndNormal (material, body0, &point.m_x, &normal.m_x);
		NewtonMaterialGetContactPositionAndNormal (material, body1, &point.m_x, &normal.m_x);

		// do whatever you want with the force
	}
}


void GetContactOnBody (ndBodyKinematic* const body)
{
	for (NewtonJoint* joint = ndBodyKinematicGetFirstContactJoint (body); joint; joint = ndBodyKinematicGetNextContactJoint (body, joint)) {
		ndBodyKinematic* const body0 = NewtonJointGetBody0(joint);
		ndBodyKinematic* const body1 = NewtonJointGetBody1(joint);
		for (void* contact = NewtonContactJointGetFirstContact (joint); contact; contact = NewtonContactJointGetNextContact (joint, contact)) {
			NewtonMaterial* const material = NewtonContactGetMaterial (contact);

			//dFloat32 forceMag;
			dVector point(0.0f);
			dVector normal(0.0f);	
			//NewtonMaterialGetContactForce (material, &forceMag);
			NewtonMaterialGetContactPositionAndNormal (material, body1, &point.m_x, &normal.m_x);
			NewtonMaterialGetContactPositionAndNormal (material, body0, &point.m_x, &normal.m_x);
			// do whatever you want with the force
		}
	}
}

// rigid body destructor
void  PhysicsBodyDestructor (const ndBodyKinematic* body)
{
//	RenderPrimitive* primitive;

	// get the graphic object form the rigid body
//	primitive = (RenderPrimitive*) ndBodyKinematicGetUserData (body);

	// destroy the graphic object
	//	delete primitive;
}

// add force and torque to rigid body
void PhysicsApplyGravityForce (const ndBodyKinematic* body, dFloat32 timestep, dInt32 threadIndex)
{
	dFloat32 Ixx;
	dFloat32 Iyy;
	dFloat32 Izz;
	dFloat32 mass;

	ndBodyKinematicGetMass (body, &mass, &Ixx, &Iyy, &Izz);
	dVector dir(0.0f, 1.0f, 0.0f);
//	dVector dir(1.0f, 0.0f, 0.0f);
//mass = 0.0f;
	dVector force (dir.Scale (mass * DEMO_GRAVITY));
	ndBodyKinematicSetForce (body, &force.m_x);

	// for regular gravity objects, clamp high angular velocities 
	dVector omega(0.0f);
	ndBodyKinematicGetOmega(body, &omega[0]);
	dFloat32 mag2 = omega.DotProduct3(omega);
	dFloat32 maxMag = 100.0f;
	if (mag2 > (maxMag * maxMag)) {
		omega = omega.Normalize().Scale(maxMag);
		ndBodyKinematicSetOmega(body, &omega[0]);
	}


#ifdef DEMO_CHECK_ASYN_UPDATE
	dAssert(g_checkAsyncUpdate);
#endif

	// test going to sleep bug
//	ndBodyKinematicSetSleepState(body, 0);
}

void GenericContactProcess (const NewtonJoint* contactJoint, dFloat32 timestep, dInt32 threadIndex)
{
#if 0 
	dFloat speed0;
	dFloat speed1;
	SpecialEffectStruct* currectEffect;

	// get the pointer to the special effect structure
	currectEffect = (SpecialEffectStruct *)NewtonMaterialGetMaterialPairUserData (material);

	// save the contact information
	NewtonMaterialGetContactPositionAndNormal (material, &currectEffect->m_position.m_x, &currectEffect->m_normal.m_x);
	NewtonMaterialGetContactTangentDirections (material, &currectEffect->m_tangentDir0.m_x, &currectEffect->m_tangentDir1.m_x);


	// Get the maximum normal speed of this impact. this can be used for positioning collision sound
	speed0 = NewtonMaterialGetContactNormalSpeed (material);
	if (speed0 > currectEffect->m_contactMaxNormalSpeed) {
		// save the position of the contact (for 3d sound of particles effects)
		currectEffect->m_contactMaxNormalSpeed = speed0;
	}

	// get the maximum of the two sliding contact speed
	speed0 = NewtonMaterialGetContactTangentSpeed (material, 0);
	speed1 = NewtonMaterialGetContactTangentSpeed (material, 1);
	if (speed1 > speed0) {
		speed0 = speed1;
	}

	// Get the maximum tangent speed of this contact. this can be used for particles(sparks) of playing scratch sounds 
	if (speed0 > currectEffect->m_contactMaxTangentSpeed) {
		// save the position of the contact (for 3d sound of particles effects)
		currectEffect->m_contactMaxTangentSpeed = speed0;
	}


#endif
	
	// read the table direction
//	dVector dir (tableDir);
//	dVector updir (TableDir);
//	ndBodyKinematic* const body = NewtonJointGetBody0(contactJoint);
//	for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = NewtonContactJointGetNextContact (contactJoint, contact)) {
//		dFloat speed;
//		dVector point;
//		dVector normal;	
//		dVector dir0;	
//		dVector dir1;	
//		dVector force;
//		NewtonMaterial* material;
//
//		material = NewtonContactGetMaterial (contact);
//		NewtonMaterialGetContactPositionAndNormal (material, body, &point.m_x, &normal.m_x);
//
//		// if the normal is vertical is large the say 40 degrees
//		if (fabsf (normal % upDir) > 0.7f) {
//			// rotate the normal to be aligned with the table direction
//			NewtonMaterialContactRotateTangentDirections (material, dir);
//		}
//	}


	ndBodyKinematic* const body = NewtonJointGetBody0(contactJoint);
	for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = NewtonContactJointGetNextContact (contactJoint, contact)) {
		dVector point(0.0f);
		dVector normal(0.0f);	
		dVector dir0(0.0f);	
		dVector dir1(0.0f);	
		dVector force(0.0f);

		NewtonMaterial* const material = NewtonContactGetMaterial (contact);

		NewtonMaterialGetContactForce (material, body, &force.m_x);
		NewtonMaterialGetContactPositionAndNormal (material, body, &point.m_x, &normal.m_x);
		NewtonMaterialGetContactTangentDirections (material, body, &dir0.m_x, &dir1.m_x);
		//dFloat32 speed = NewtonMaterialGetContactNormalSpeed(material);

		//speed = NewtonMaterialGetContactNormalSpeed(material);
		// play sound base of the contact speed.
		//
	}
}



NewtonCollision* CreateConvexCollision (ndWorld* const world, const dMatrix& srcMatrix, const dVector& originalSize, ndPrimitiveType type, dInt32 materialID__)
{
	dVector size (originalSize);

	NewtonCollision* collision = nullptr;
	switch (type) 
	{
		case _NULL_PRIMITIVE:
		{
			collision = NewtonCreateNull (world); 
			break;
		}

		case _SPHERE_PRIMITIVE:
		{
			// create the collision 
			collision = NewtonCreateSphere (world, size.m_x * 0.5f, 0, nullptr); 
			break;
		}

		case _BOX_PRIMITIVE:
		{
			// create the collision 
			collision = NewtonCreateBox (world, size.m_x, size.m_y, size.m_z, 0, nullptr); 
			break;
		}


		case _CONE_PRIMITIVE:
		{
			dFloat32 r = size.m_x * 0.5f;
			dFloat32 h = size.m_y;

			// create the collision 
			collision = NewtonCreateCone (world, r, h, 0, nullptr); 
			break;
		}

		case _CYLINDER_PRIMITIVE:
		{
			// create the collision 
			collision = NewtonCreateCylinder (world, size.m_x * 0.5f, size.m_z * 0.5f, size.m_y, 0, nullptr); 
			break;
		}


		case _CAPSULE_PRIMITIVE:
		{
			// create the collision 
			collision = NewtonCreateCapsule (world, size.m_x * 0.5f, size.m_z * 0.5f, size.m_y, 0, nullptr); 
			break;
		}

		case _CHAMFER_CYLINDER_PRIMITIVE:
		{
			// create the collision 
			collision = NewtonCreateChamferCylinder (world, size.m_x * 0.5f, size.m_y, 0, nullptr); 
			break;
		}

		case _RANDOM_CONVEX_HULL_PRIMITIVE:
		{
			// Create a clouds of random point around the origin
			#define SAMPLE_COUNT 200
			dVector cloud [SAMPLE_COUNT];

			// make sure that at least the top and bottom are present
			cloud [0] = dVector ( size.m_x * 0.5f, 0.0f, 0.0f, 0.0f);
			cloud [1] = dVector (-size.m_x * 0.5f, 0.0f, 0.0f, 0.0f);
			cloud [2] = dVector ( 0.0f,  size.m_y * 0.5f, 0.0f, 0.0f); 
			cloud [3] = dVector ( 0.0f, -size.m_y * 0.5f, 0.0f, 0.0f);
			cloud [4] = dVector (0.0f, 0.0f,  size.m_z * 0.5f, 0.0f); 
			cloud [5] = dVector (0.0f, 0.0f, -size.m_z * 0.5f, 0.0f); 

			dInt32 count = 6;
			// populate the cloud with pseudo Gaussian random points
			for (dInt32 i = 6; i < SAMPLE_COUNT; i ++) {
				cloud [i].m_x = dGaussianRandom (size.m_x);
				cloud [i].m_y = dGaussianRandom (size.m_y);
				cloud [i].m_z = dGaussianRandom (size.m_z);
				count ++;
			}
			collision = NewtonCreateConvexHull (world, count, &cloud[0].m_x, sizeof (dVector), 0.01f, 0, nullptr); 
			break;
		}

		case _REGULAR_CONVEX_HULL_PRIMITIVE:
		{
			// Create a clouds of random point around the origin
			#define STEPS_HULL 6
			//#define STEPS_HULL 3

			//dVector cloud [STEPS_HULL * 4 + 256];
			dFloat32 cloud [STEPS_HULL * 4 + 256][3];
			dInt32 count = 0;
			dFloat32 radius = size.m_y;
			dFloat32 height = size.m_x * 0.999f;
			dFloat32 x = - height * 0.5f;
			dMatrix rotation (dPitchMatrix(2.0f * dPi / STEPS_HULL));
			for (dInt32 i = 0; i < 4; i ++) {
				dFloat32 pad = ((i == 1) || (i == 2)) * 0.25f * radius;
				dVector p (x, 0.0f, radius + pad);
				x += 0.3333f * height;
				dMatrix acc (dGetIdentityMatrix());
				for (dInt32 j = 0; j < STEPS_HULL; j ++) {
					dVector tmp (acc.RotateVector(p)); 
					cloud[count][0] = tmp.m_x;
					cloud[count][1] = tmp.m_y;
					cloud[count][2] = tmp.m_z;
					acc = acc * rotation;
					count ++;
				}
			}

			collision = NewtonCreateConvexHull (world, count, &cloud[0][0], 3 * sizeof (dFloat32), 0.02f, 0, nullptr); 
			break;
		}

		case _COMPOUND_CONVEX_CRUZ_PRIMITIVE:
		{
			//dMatrix matrix (GetIdentityMatrix());
			dMatrix matrix (dPitchMatrix(15.0f * dDegreeToRad) * dYawMatrix(15.0f * dDegreeToRad) * dRollMatrix(15.0f * dDegreeToRad));

			matrix.m_posit = dVector (size.m_x * 0.5f, 0.0f, 0.0f, 1.0f);
			NewtonCollision* const collisionA = NewtonCreateBox (world, size.m_x, size.m_x * 0.25f, size.m_x * 0.25f, 0, &matrix[0][0]); 
			matrix.m_posit = dVector (0.0f, size.m_x * 0.5f, 0.0f, 1.0f);
			NewtonCollision* const collisionB = NewtonCreateBox (world, size.m_x * 0.25f, size.m_x, size.m_x * 0.25f, 0, &matrix[0][0]); 
			matrix.m_posit = dVector (0.0f, 0.0f, size.m_x * 0.5f, 1.0f);
			NewtonCollision* const collisionC = NewtonCreateBox (world, size.m_x * 0.25f, size.m_x * 0.25f, size.m_x, 0, &matrix[0][0]); 

			collision = NewtonCreateCompoundCollision (world, 0);
			NewtonCompoundCollisionBeginAddRemove(collision);

			NewtonCompoundCollisionAddSubCollision (collision, collisionA);
			NewtonCompoundCollisionAddSubCollision (collision, collisionB);
			NewtonCompoundCollisionAddSubCollision (collision, collisionC);

			NewtonCompoundCollisionEndAddRemove(collision);	

			NewtonDestroyCollision(collisionA);
			NewtonDestroyCollision(collisionB);
			NewtonDestroyCollision(collisionC);
			break;
		}

		default: dAssert (0);
	}


	dMatrix matrix (srcMatrix);
	matrix.m_front = matrix.m_front.Scale (1.0f / dSqrt (matrix.m_front.DotProduct3(matrix.m_front)));
	matrix.m_right = matrix.m_front.CrossProduct(matrix.m_up);
	matrix.m_right = matrix.m_right.Scale (1.0f / dSqrt (matrix.m_right.DotProduct3(matrix.m_right)));
	matrix.m_up = matrix.m_right.CrossProduct(matrix.m_front);
	NewtonCollisionSetMatrix(collision, &matrix[0][0]);

	return collision;
}

ndBodyKinematic* CreateSimpleBody (ndWorld* const world, void* const userData, dFloat32 mass, const dMatrix& matrix, NewtonCollision* const collision, dInt32 materialId, bool generalInertia)
{
	dAssert(0);
	return nullptr;
/*
	// calculate the moment of inertia and the relative center of mass of the solid
	//	dVector origin;
	//	dVector inertia;
	//	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	
	//	dFloat32 Ixx = mass * inertia[0];
	//	dFloat32 Iyy = mass * inertia[1];
	//	dFloat32 Izz = mass * inertia[2];

	//create the rigid body
	ndBodyKinematic* const rigidBody = generalInertia ? NewtonCreateAsymetricDynamicBody (world, collision, &matrix[0][0]) : NewtonCreateDynamicBody(world, collision, &matrix[0][0]);

	// use a more convenient function for setting mass and inertia matrix
	ndBodyKinematicSetMassProperties (rigidBody, mass, collision);

	// save the pointer to the graphic object with the body.
	ndBodyKinematicSetUserData (rigidBody, userData);

	// assign the wood id
	ndBodyKinematicSetMaterialGroupID (rigidBody, materialId);

	//  set continuous collision mode
	//	ndBodyKinematicSetContinuousCollisionMode (rigidBody, continueCollisionMode);

	// set a destructor for this rigid body
	ndBodyKinematicSetDestructorCallback (rigidBody, PhysicsBodyDestructor);

	// set the transform call back function
	ndBodyKinematicSetTransformCallback (rigidBody, ndDemoEntity::TransformCallback);

	// set the force and torque call back function
	ndBodyKinematicSetForceAndTorqueCallback (rigidBody, PhysicsApplyGravityForce);

	// set the matrix for both the rigid body and the graphic body
	//ndBodyKinematicSetMatrix (rigidBody, &matrix[0][0]);
	//PhysicsSetTransform (rigidBody, &matrix[0][0], 0);

	//dVector xxx (0, -9.8f * mass, 0.0f, 0.0f);
	//ndBodyKinematicSetForce (rigidBody, &xxx[0]);

	// force the body to be active of inactive
	//	ndBodyKinematicSetAutoSleep (rigidBody, sleepMode);
	return rigidBody;
*/
}

ndBodyKinematic* CreateSimpleSolid (ndDemoEntityManager* const scene, ndDemoMesh* const mesh, dFloat32 mass, const dMatrix& matrix, NewtonCollision* const collision, dInt32 materialId, bool generalInertia)
{
	dAssert(0);
	return nullptr;
/*
	dAssert (collision);

	// add an new entity to the world
	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	scene->Append (entity);
	if (mesh) {
		entity->SetMesh(mesh, dGetIdentityMatrix());
	}
	return CreateSimpleBody (scene->GetWorld(), entity, mass, matrix, collision, materialId, generalInertia);
*/
}


ndBodyKinematic* CreateInstancedSolid(ndDemoEntityManager* const scene, ndDemoEntity* const parent, dFloat32 mass, const dMatrix& matrix, NewtonCollision* const collision, dInt32 materialId, bool generalInertia)
{
	dAssert(0);
	return nullptr;
/*
	dAssert(collision);
	// add an new entity to the world
	ndDemoEntity* const entity = new ndDemoEntity(matrix, parent);
	return CreateSimpleBody(scene->GetWorld(), entity, mass, matrix, collision, materialId, generalInertia);
*/
}


void AddPrimitiveArray (ndDemoEntityManager* const scene, dFloat32 mass, const dVector& origin, const dVector& size, dInt32 xCount, dInt32 zCount, dFloat32 spacing, ndPrimitiveType type, dInt32 materialID, const dMatrix& shapeOffsetMatrix, dFloat32 startElevation, dFloat32 offsetHigh)
{
	dAssert(0);
/*
	// create the shape and visual mesh as a common data to be re used
	ndWorld* const world = scene->GetWorld();
	NewtonCollision* const collision = CreateConvexCollision (world, shapeOffsetMatrix, size, type, materialID);

	// test collision mode
	//NewtonCollisionSetCollisonMode(collision, 0);

	ndDemoMesh* const geometry = new ndDemoMesh("primitive", scene->GetShaderCache(), collision, "smilli.tga", "smilli.tga", "smilli.tga");

	dMatrix matrix (dGetIdentityMatrix());
	for (dInt32 i = 0; i < xCount; i ++) {
		dFloat32 x = origin.m_x + (i - xCount / 2) * spacing;
		for (dInt32 j = 0; j < zCount; j ++) {
			dFloat32 z = origin.m_z + (j - zCount / 2) * spacing;

			matrix.m_posit.m_x = x;
			matrix.m_posit.m_z = z;
			dVector floor (FindFloor (world, dVector (matrix.m_posit.m_x, startElevation, matrix.m_posit.m_z, 0.0f), 2.0f * startElevation));
			matrix.m_posit.m_y = floor.m_y + size.m_y * 0.5f + offsetHigh;
			if (matrix.m_posit.m_y < 900.0f) {
				CreateSimpleSolid(scene, geometry, mass, matrix, collision, materialID);
			}
		}
	}
	// do not forget to release the assets	
	geometry->Release(); 
	NewtonDestroyCollision (collision);
*/
}

void CalculateAABB (const NewtonCollision* const collision, const dMatrix& matrix, dVector& minP, dVector& maxP)
{
	for (dInt32 i = 0; i < 3; i ++) {
		dVector support(0.0f);
		dVector dir (0.0f);
		dir[i] = 1.0f;

		dVector localDir (matrix.UnrotateVector (dir));
		NewtonCollisionSupportVertex (collision, &localDir[0], &support[0]);
		support = matrix.TransformVector (support);
		maxP[i] = support[i];  

		localDir = localDir.Scale (-1.0f);
		NewtonCollisionSupportVertex (collision, &localDir[0], &support[0]);
		support = matrix.TransformVector (support);
		minP[i] = support[i];  
	}
}

void SetAutoSleepMode (ndWorld* const world, dInt32 mode)
{
	mode = mode ? 0 : 1;
	for (const ndBodyKinematic* body = ndWorldGetFirstBody (world); body; body = ndWorldGetNextBody (world, body)) {
		ndBodyKinematicSetAutoSleep (body, mode);
	}
}

class CollsionTreeFaceMap
{
	public:
	struct FaceInfo
	{
		//void* m_face;
		dInt32 m_materialIndex;
	};

	CollsionTreeFaceMap (NewtonCollision* const collisionTree)
		:m_faceCount(0)
		,m_collisionTree(collisionTree)
	{
		NewtonTreeCollisionForEachFace (m_collisionTree, CountFaces, this); 
		m_faceMapInfo = new FaceInfo[m_faceCount];

		m_faceCount = 0;
		NewtonTreeCollisionForEachFace (m_collisionTree, MarkFaces, this); 
	}

	~CollsionTreeFaceMap()
	{
		delete[] m_faceMapInfo;
	}

	static dInt32 CountFaces (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount)
	{
		CollsionTreeFaceMap* const me = (CollsionTreeFaceMap*) context;
		me->m_faceCount ++;
		return 1;
	}
	static dInt32 MarkFaces (void* const context, const dFloat32* const polygon, dInt32 strideInBytes, const dInt32* const indexArray, dInt32 indexCount)
	{
		CollsionTreeFaceMap* const me = (CollsionTreeFaceMap*) context;

		// repmap material index, by enumerating the face and storing the user material info at each face index
		dInt32 faceIndex = NewtonTreeCollisionGetFaceAttribute (me->m_collisionTree, indexArray, indexCount); 
		me->m_faceMapInfo[me->m_faceCount].m_materialIndex = faceIndex;
		NewtonTreeCollisionSetFaceAttribute (me->m_collisionTree, indexArray, indexCount, me->m_faceCount); 

		me->m_faceCount ++;
		return 1;
	}
	dInt32 m_faceCount;
	FaceInfo* m_faceMapInfo;
	NewtonCollision* m_collisionTree;
};


NewtonCollision* CreateCollisionTree (ndWorld* const world, ndDemoEntity* const entity, dInt32 materialID, bool optimize)
{
	// measure the time to build a collision tree
	dUnsigned64 timer0 = dGetTimeInMicrosenconds();

	// create the collision tree geometry
	NewtonCollision* collision = NewtonCreateTreeCollision(world, materialID);

	// set the application level callback
#ifdef USE_STATIC_MESHES_DEBUG_COLLISION
	//NewtonStaticCollisionSetDebugCallback (collision, ShowMeshCollidingFaces);
#endif

	// prepare to create collision geometry
	NewtonTreeCollisionBeginBuild(collision);

	// iterate the entire geometry an build the collision
	for (ndDemoEntity* model = entity->GetFirst(); model; model = model->GetNext()) {

		dMatrix matrix (model->GetMeshMatrix() * model->CalculateGlobalMatrix(entity));
		ndDemoMesh* const mesh = (ndDemoMesh*)model->GetMesh();
		dAssert(0);
		//dAssert (mesh->IsType(ndDemoMesh::GetRttiType()));

		dFloat32* const vertex = mesh->m_vertex;
		for (ndDemoMesh::dListNode* nodes = mesh->GetFirst(); nodes; nodes = nodes->GetNext()) {
			ndDemoSubMesh& segment = nodes->GetInfo();
			//dInt32 matID = segment.m_textureHandle;
			dInt32 matID = 1;
			if (segment.m_textureName.Find("wood") != -1) {
				matID  = 2;
			} else if (segment.m_textureName.Find("floor") != -1) {
				matID = 3;
			} else if (segment.m_textureName.Find("marble") != -1) {
				matID = 3;
			}


			for (dInt32 i = 0; i < segment.m_indexCount; i += 3) {
				dFloat32 face[3][3];
				for (dInt32 j = 0; j < 3; j ++) {
					dInt32 index = segment.m_indexes[i + j] * 3;
					face[j][0] = vertex[index + 0];
					face[j][1] = vertex[index + 1];
					face[j][2] = vertex[index + 2];
				}
				matrix.TransformTriplex (&face[0][0], 3 * sizeof (dFloat32), &face[0][0], 3 * sizeof (dFloat32), 3);

				// use material ids as physics materials 
				NewtonTreeCollisionAddFace(collision, 3, &face[0][0], 3 * sizeof (dFloat32), matID);
			}
		}
	}
	//NewtonTreeCollisionEndBuild(collision, optimize ? 2 : 0);
	NewtonTreeCollisionEndBuild(collision, optimize ? 1 : 0);

	// measure the time to build a collision tree
	timer0 = (dGetTimeInMicrosenconds() - timer0) / 1000;

	return collision;
}

ndBodyKinematic* CreateLevelMeshBody (ndWorld* const world, ndDemoEntity* const ent, bool optimize)
{
	NewtonCollision* const collision = CreateCollisionTree (world, ent, 0, optimize);

	// Get the root Matrix
	dMatrix matrix (ent->CalculateGlobalMatrix(nullptr));

	// create the level rigid body
	ndBodyKinematic* const level = NewtonCreateDynamicBody(world, collision, &matrix[0][0]);

//NewtonCollision* const collision1 = NewtonCreateNull(world);
//ndBodyKinematic* const level = NewtonCreateDynamicBody(world, collision1, &matrix[0][0]);
//ndBodyKinematicSetCollision(level, collision);
//NewtonDestroyCollision (collision1);

	// save the pointer to the graphic object with the body.
	ndBodyKinematicSetUserData (level, ent);

#if 0
	NewtonCollisionInfoRecord collisionInfo;
	NewtonCollisionGetInfo (collision, &collisionInfo);
	if (collisionInfo.m_collisionType == SERIALIZE_ID_TREE) {
		dInt32 count;
		dVector p0(-100, -100, -100);
		dVector p1(100, 100, 100);
		const dFloat* vertexArray;
		dInt32 vertexStrideInBytes;
		dInt32 vertexCount;
		dInt32 indexList[256];
		dInt32 attributeList[256/3];
		count = NewtonTreeCollisionGetVertexListTriangleListInAABB (collision, &p0[0], &p1[0], 
			&vertexArray, &vertexCount, &vertexStrideInBytes, 
			indexList, sizeof (indexList)/sizeof (indexList[0]), 
			attributeList); 
	}
#endif

	// set a destructor for this rigid body
	//ndBodyKinematicSetDestructorCallback (m_level, Destructor);

	// release the collision tree (this way the application does not have to do book keeping of Newton objects
	NewtonDestroyCollision (collision);

	// now we will make a lookup table for quick material index lookup for face to index
	//CollsionTreeFaceMap faceMap (ndBodyKinematicGetCollision(level));
	return level;
}

ndBodyKinematic* AddFloorBox(ndDemoEntityManager* const scene, const dVector& origin, const dVector& size)
{
	dAssert(0);
	return nullptr;
/*
	// create the shape and visual mesh as a common data to be re used
	ndWorld* const world = scene->GetWorld();
	const dInt32 materialID = NewtonMaterialGetDefaultGroupID(scene->GetWorld());
	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _BOX_PRIMITIVE, materialID);

	// test collision mode
	//NewtonCollisionSetCollisonMode(collision, 0);

	ndDemoMesh* const geometry = new ndDemoMesh("primitive", scene->GetShaderCache(), collision, "wood_3.tga", "wood_3.tga", "wood_3.tga");

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;
	ndBodyKinematic* const body = CreateSimpleSolid(scene, geometry, 0.0f, matrix, collision, materialID);
	// do not forget to release the assets	
	geometry->Release();
	NewtonDestroyCollision(collision);
	return body;
*/
}

ndBodyKinematic* CreatePLYMesh (ndDemoEntityManager* const scene, const char* const fileName, bool optimized)
{
	dAssert(0);
	return nullptr;
/*
	FILE* const file = fopen(fileName, "rb");
	if (!file) {
		return nullptr;
	}

	char buffer[265];

	dInt32 faceCount;
	dInt32 vertexCount;
	fgets(buffer, sizeof (buffer), file);
	fgets(buffer, sizeof (buffer), file);
	fscanf(file, "%s %s %d\n", buffer, buffer, &vertexCount);
	fgets(buffer, sizeof (buffer), file);
	fgets(buffer, sizeof (buffer), file);
	fgets(buffer, sizeof (buffer), file);
	fscanf(file, "%s %s %d\n", buffer, buffer, &faceCount);
	fgets(buffer, sizeof (buffer), file);
	fgets(buffer, sizeof (buffer), file);

	dArray<dVector> points(vertexCount);
	for (dInt32 i = 0; i < vertexCount; i++) {
		dFloat32 x;
		dFloat32 y;
		dFloat32 z;
		fscanf(file, "%f %f %f\n", &x, &y, &z);
		points[i] = dVector(x, y, z, dFloat32(0.0f));
	}

	ndWorld* const world = scene->GetWorld();

	// create the collision tree geometry
	NewtonCollision* collision = NewtonCreateTreeCollision(world, 0);

	// prepare to create collision geometry
	NewtonTreeCollisionBeginBuild(collision);
	for (dInt32 i = 0; i < faceCount; i++) {
		dInt32 count;
		dFloat32 face[32][3];
		fscanf(file, "%d", &count);
		for (dInt32 j = 0; j < count; j++) {
			dInt32 index;
			fscanf(file, "%d", &index);
			face[j][0] = points[index][0];
			face[j][1] = points[index][1];
			face[j][2] = points[index][2];
		}
		fscanf(file, "\n");
		NewtonTreeCollisionAddFace(collision, 3, &face[0][0], 3 * sizeof (dFloat32), 0);
	}
	fclose(file);

	NewtonTreeCollisionEndBuild(collision, 1);

	// create the level rigid body
	dMatrix matrix (dGetIdentityMatrix());
	ndBodyKinematic* const level = NewtonCreateDynamicBody(world, collision, &matrix[0][0]);
	NewtonDestroyCollision(collision);

	// save the pointer to the graphic object with the body.
//	ndBodyKinematicSetUserData(level, ent);

	NewtonInvalidateCache(world);

	return level;
*/
}

void LoadLumberYardMesh(ndDemoEntityManager* const scene, const dVector& location, dInt32 shapeid)
{
	dAssert(0);
/*
	ndDemoEntity* const entity = ndDemoEntity::LoadNGD_mesh ("lumber.ngd", scene->GetWorld(), scene->GetShaderCache());

	dTree<NewtonCollision*, ndDemoMesh*> filter;
	ndWorld* const world = scene->GetWorld();

	dFloat32 density = 1000.0f;

	dInt32 defaultMaterialID = NewtonMaterialGetDefaultGroupID(scene->GetWorld());
	for (ndDemoEntity* child = entity->GetFirst(); child; child = child->GetNext()) {
		ndDemoMesh* const mesh = (ndDemoMesh*)child->GetMesh();
		if (mesh) {
			dAssert(0);
			//dAssert(mesh->IsType(ndDemoMesh::GetRttiType()));
			dTree<NewtonCollision*, ndDemoMesh*>::dTreeNode* node = filter.Find(mesh);
			if (!node) {
				// make a collision shape only for and instance
				dFloat32* const array = mesh->m_vertex;
				dVector minBox(1.0e10f, 1.0e10f, 1.0e10f, 1.0f);
				dVector maxBox(-1.0e10f, -1.0e10f, -1.0e10f, 1.0f);

				for (dInt32 i = 0; i < mesh->m_vertexCount; i++) {
					dVector p(array[i * 3 + 0], array[i * 3 + 1], array[i * 3 + 2], 1.0f);
					minBox.m_x = dMin(p.m_x, minBox.m_x);
					minBox.m_y = dMin(p.m_y, minBox.m_y);
					minBox.m_z = dMin(p.m_z, minBox.m_z);

					maxBox.m_x = dMax(p.m_x, maxBox.m_x);
					maxBox.m_y = dMax(p.m_y, maxBox.m_y);
					maxBox.m_z = dMax(p.m_z, maxBox.m_z);
				}

				dVector size(maxBox - minBox);
				dMatrix offset(dGetIdentityMatrix());
				offset.m_posit = (maxBox + minBox).Scale(0.5f);
				NewtonCollision* const shape = NewtonCreateBox(world, size.m_x, size.m_y, size.m_z, shapeid, &offset[0][0]);
				node = filter.Insert(shape, mesh);
			}

			// create a body and add to the world
			NewtonCollision* const shape = node->GetInfo();
			dMatrix matrix(child->GetMeshMatrix() * child->CalculateGlobalMatrix());
			matrix.m_posit += location;
			dFloat32 mass = density * NewtonConvexCollisionCalculateVolume(shape);
			CreateSimpleSolid(scene, mesh, mass, matrix, shape, defaultMaterialID);
		}
	}

	// destroy all shapes
	while (filter.GetRoot()) {
		NewtonCollision* const shape = filter.GetRoot()->GetInfo();
		NewtonDestroyCollision(shape);
		filter.Remove(filter.GetRoot());
	}
	delete entity;
*/
}

dCustomJoint* FindJoint(const ndBodyKinematic* const body0, const ndBodyKinematic* const body1)
{
	NewtonJoint* const joint = ndWorldFindJoint(body0, body1);
	return joint ? (dCustomJoint*)NewtonJointGetUserData(joint) : nullptr;
}

#endif


dVector FindFloor(const ndWorld& world, const dVector& origin, dFloat32 dist)
{
	dVector p0(origin);
	dVector p1(origin - dVector(0.0f, dAbs(dist), 0.0f, 0.0f));

	ndRayCastClosestHitCallback rayCaster(world.GetScene());
	dFloat32 param = rayCaster.TraceRay(p0, p1);
	return (param < 1.0f) ? rayCaster.m_contact.m_point : p0;
}

ndBodyKinematic* MousePickBody(ndWorld* const world, const dVector& origin, const dVector& end, dFloat32& paramterOut, dVector& positionOut, dVector& normalOut)
{
	class ndRayPickingCallback: public ndRayCastClosestHitCallback
	{
		public: 
		ndRayPickingCallback(const ndScene* const scene)
			:ndRayCastClosestHitCallback(scene)
		{
		}

		dFloat32 OnRayCastAction(const ndContactPoint& contact, dFloat32 intersetParam)
		{
			if (contact.m_body0->GetInvMass() == dFloat32(0.0f)) 
			{
				return 1.2f;
			}
			return ndRayCastClosestHitCallback::OnRayCastAction(contact, intersetParam);
		}
	};

	ndRayPickingCallback rayCaster(world->GetScene());
	dFloat32 param = rayCaster.TraceRay(origin, end);
	if (param < 1.0f)
	{
		paramterOut = param;
		positionOut = rayCaster.m_contact.m_point;
		normalOut = rayCaster.m_contact.m_normal;
		return (ndBodyKinematic* )rayCaster.m_contact.m_body0;
	}

	return nullptr;
}
