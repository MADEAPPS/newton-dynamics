/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software./Users/juliojerez/Desktop/newton-dynamics/applications/demosSandbox/sdkDemos/toolBox/PhysicsUtils.cpp
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "toolbox_stdafx.h"
#include "DemoMesh.h"
#include "DemoEntity.h"
#include "PhysicsUtils.h"
#include "DemoEntityManager.h"
#include "OpenGlUtil.h"
#include "DebugDisplay.h"
#include "dHighResolutionTimer.h"

//const D_MESH_HEADER	"Newton Mesh"
static const char* D_MESH_HEADER = "Newton Mesh";


class dMousePickClass
{
	public:
	dMousePickClass ()
		:m_normal(0.0f)
		,m_param (1.0f)
		,m_body(NULL)
	{
	}

	// implement a ray cast pre-filter
	static unsigned RayCastPrefilter (const NewtonBody* body,  const NewtonCollision* const collision, void* const userData)
	{
		// ray cannot pick trigger volumes
		//return NewtonCollisionIsTriggerVolume(collision) ? 0 : 1;

		const NewtonCollision* const parent = NewtonCollisionGetParentInstance(collision);
		if (parent) {
			// you can use this to filter sub collision shapes.  
			dAssert (NewtonCollisionGetSubCollisionHandle (collision));
		}

		return (NewtonBodyGetType(body) == NEWTON_DYNAMIC_BODY) ? 1 : 0;
	}

	static dFloat RayCastFilter (const NewtonBody* const body, const NewtonCollision* const collisionHit, const dFloat* const contact, const dFloat* const normal, dLong collisionID, void* const userData, dFloat intersetParam)
	{
		dFloat mass;
		dFloat Ixx;
		dFloat Iyy;
		dFloat Izz;

		// check if we are hitting a sub shape
		const NewtonCollision* const parent = NewtonCollisionGetParentInstance(collisionHit);
		if (parent) {
			// you can use this to filter sub collision shapes.  
			dAssert (NewtonCollisionGetSubCollisionHandle (collisionHit));
		}

		dMousePickClass* const data = (dMousePickClass*) userData;
		NewtonBodyGetMass (body, &mass, &Ixx, &Iyy, &Izz);
		if ((mass > 0.0f) || (NewtonBodyGetType(body) == NEWTON_KINEMATIC_BODY)) {
			data->m_body = body;
		}


		if (intersetParam < data->m_param) {
			data->m_param = intersetParam;
			data->m_normal = dVector (normal[0], normal[1], normal[2]);
		}
		return intersetParam;
	}

	dVector m_normal;
	dFloat m_param;
	const NewtonBody* m_body;
};


dVector ForceBetweenBody (NewtonBody* const body0, NewtonBody* const body1)
{
	dVector reactionforce (0.0f);
	for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint(body0); joint; joint = NewtonBodyGetNextContactJoint(body0, joint)) {
		if ((NewtonJointGetBody0(joint) == body0) || (NewtonJointGetBody0(joint) == body1)) {
			for (void* contact = NewtonContactJointGetFirstContact (joint); contact; contact = NewtonContactJointGetNextContact (joint, contact)) {
				dVector point(0.0f);
				dVector normal(0.0f);	
				dVector contactForce(0.0f);
				NewtonMaterial* const material = NewtonContactGetMaterial (contact);
				NewtonMaterialGetContactPositionAndNormal (material, body0, &point.m_x, &normal.m_x);
				NewtonMaterialGetContactForce(material, body0, &contactForce[0]);
				//forceAcc += normal.Scale (forceMag);
				reactionforce += contactForce;
			}
			break;
		}
	}
	return reactionforce;
}

void GetConnectedBodiesByJoints (NewtonBody* const body) 
{
	for (NewtonJoint* joint = NewtonBodyGetFirstJoint(body); joint; joint = NewtonBodyGetNextJoint(body, joint)) {
		dCustomJoint* const customJoint = (dCustomJoint*) NewtonJointGetUserData(joint);
		NewtonBody* const body0 = customJoint->GetBody0();
		NewtonBody* const body1 = customJoint->GetBody1();
		NewtonBody* const otherBody = (body0 == body) ? body1 : body0;
		// do whatever you need to do here
		NewtonBodySetFreezeState (otherBody, 0);
	}
}
	



static dFloat RayCastPlacement (const NewtonBody* const body, const NewtonCollision* const collisionHit, const dFloat* const contact, const dFloat* const normal, dLong collisionID, void* const userData, dFloat intersetParam)
{
	// if the collision has a parent, the this can be it si a sub shape of a compound collision 
	const NewtonCollision* const parent = NewtonCollisionGetParentInstance(collisionHit);
	if (parent) {
		// you can use this to filter sub collision shapes.  
		dAssert (NewtonCollisionGetSubCollisionHandle (collisionHit));
	}


	dFloat* const paramPtr = (dFloat*)userData;
	if (intersetParam < paramPtr[0]) {
		paramPtr[0] = intersetParam;
	}
	return paramPtr[0];
}


static unsigned RayPrefilter (const NewtonBody* const body, const NewtonCollision* const collision, void* const userData)
{
	// if the collision has a parent, the this can be it si a sub shape of a compound collision 
	const NewtonCollision* const parent = NewtonCollisionGetParentInstance(collision);
	if (parent) {
		// you can use this to filter sub collision shapes.  
		dAssert (NewtonCollisionGetSubCollisionHandle (collision));
	}

	return 1;
}

dVector FindFloor (const NewtonWorld* world, const dVector& origin, dFloat dist)
{
	// shot a vertical ray from a high altitude and collect the intersection parameter.
	dVector p0 (origin); 
	dVector p1 (origin - dVector (0.0f, dAbs (dist), 0.0f, 0.0f)); 

	dFloat parameter = 1.2f;
	NewtonWorldRayCast (world, &p0[0], &p1[0], RayCastPlacement, &parameter, RayPrefilter, 0);
	if (parameter < 1.0f) {
		p0 -= dVector (0.0f, dAbs (dist) * parameter, 0.0f, 0.0f);
	}
	return p0;
}



#if 0

static unsigned ConvexCastCallback (const NewtonBody* body, const NewtonCollision* collision, void* userData)
{
	NewtonBody* me = (NewtonBody*) userData;
	return (me == body) ? 0 : 1;
}

void ConvexCastPlacement (NewtonBody* body)
{
	dFloat param;
	dMatrix matrix;
	NewtonWorld* world;
	NewtonCollision* collision;
	NewtonWorldConvexCastReturnInfo info[16];


	NewtonBodyGetMatrix (body, &matrix[0][0]);

	matrix.m_posit.m_y += 40.0f;
	dVector p (matrix.m_posit);
	p.m_y -= 80.0f;

	world = NewtonBodyGetWorld(body);
	collision = NewtonBodyGetCollision(body);
	NewtonWorldConvexCast (world, &matrix[0][0], &p[0], collision, &param, body, ConvexCastCallback, info, 16, 0);
	dAssert (param < 1.0f);

	matrix.m_posit.m_y += (p.m_y - matrix.m_posit.m_y) * param;

	NewtonBodySetMatrix(body, &matrix[0][0]);
}


void ShowJointInfo(const NewtonCustomJoint* joint)
{
	NewtonJointRecord info;

	if (showContacts) {
		joint->GetInfo (&info);

		dMatrix bodyMatrix0;
		NewtonBodyGetMatrix (info.m_attachBody_0, &bodyMatrix0[0][0]);
		dMatrix matrix0 (*((dMatrix*) &info.m_attachmenMatrix_0[0]));
		matrix0 = matrix0 * bodyMatrix0;
		DebugDrawLine (matrix0.m_posit, matrix0.m_posit + matrix0.m_front, dVector (1, 0, 0));
		DebugDrawLine (matrix0.m_posit, matrix0.m_posit + matrix0.m_up, dVector (0, 1, 0));
		DebugDrawLine (matrix0.m_posit, matrix0.m_posit + matrix0.m_right, dVector (0, 0, 1));


		dMatrix bodyMatrix1;
		NewtonBodyGetMatrix (info.m_attachBody_1, &bodyMatrix1[0][0]);
		dMatrix matrix1 (*((dMatrix*) &info.m_attachmenMatrix_1[0]));
		matrix1 = matrix1 * bodyMatrix1;
		DebugDrawLine (matrix1.m_posit, matrix1.m_posit + matrix1.m_front, dVector (1, 0, 0));
		DebugDrawLine (matrix1.m_posit, matrix1.m_posit + matrix1.m_up,	   dVector (0, 1, 0));
		DebugDrawLine (matrix1.m_posit, matrix1.m_posit + matrix1.m_right, dVector (0, 0, 1));

	}

}


static void ShowMeshCollidingFaces (
	const NewtonBody* bodyWithTreeCollision, 
	const NewtonBody* body, 
	int faceID, 
	int vertexCount, 
	const dFloat* vertex, 
	int vertexstrideInBytes)
{

	// we are coping data to and array of memory, another call back may be doing the same thing
	// here fore we need to avoid race conditions
	NewtonWorldCriticalSectionLock (NewtonBodyGetWorld (bodyWithTreeCollision));

	dVector face[64];
	int stride = vertexstrideInBytes / sizeof (dFloat);
	for (int j = 0; j < vertexCount; j ++) {
		face [j] = dVector (vertex[j * stride + 0], vertex[j * stride + 1] , vertex[j * stride + 2]);
	}
	DebugDrawPolygon (vertexCount, face);

	// unlock the critical section
	NewtonWorldCriticalSectionUnlock (NewtonBodyGetWorld (bodyWithTreeCollision));
}


void SetShowMeshCollision (SceneManager& me, int mode)
{
	NewtonTreeCollisionCallback showFaceCallback;

	showFaceCallback = NULL;
	if (mode) {
		showFaceCallback = ShowMeshCollidingFaces;
	}

	// iterate the world
	for (const NewtonBody* body = NewtonWorldGetFirstBody (me.m_world); body; body = NewtonWorldGetNextBody (me.m_world, body)) {
		NewtonCollision* collision;
		NewtonCollisionInfoRecord info;

		collision = NewtonBodyGetCollision (body);
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


void SetShowIslands (SceneManager& me, int mode)
{
	showIslans = mode;
}


//static void CalculateAABB (const NewtonBody* body, dVector& minP, dVector& maxP)

int PhysicsIslandUpdate (const NewtonWorld* world, const void* islandHandle, int bodyCount)
{
	if (showIslans) {
		dVector minAABB ( 1.0e10f,  1.0e10f,  1.0e10f, 0.0f);
		dVector maxAABB (-1.0e10f, -1.0e10f, -1.0e10f, 0.0f);
		for (int i = 0; i < bodyCount; i ++) {
			dVector p0;
			dVector p1;

#if 0
			// show the engine loose aabb
			NewtonIslandGetBodyAABB (islandHandle, i, &p0[0], &p1[0]);
#else
			// calculate the shape aabb

			dMatrix matrix;
			NewtonBody* body;
			NewtonCollision *collision;

			body = NewtonIslandGetBody (islandHandle, i);
			collision = NewtonBodyGetCollision (body);
			NewtonBodyGetMatrix (body, &matrix[0][0]);
			CalculateAABB (collision, matrix, p0, p1);
#endif
			for (int j = 0; j < 3; j ++ ) {
				minAABB[j] = p0[j] < minAABB[j] ? p0[j] : minAABB[j];
				maxAABB[j] = p1[j] > maxAABB[j] ? p1[j] : maxAABB[j];
			}
		}
		DebugDrawAABB (minAABB, maxAABB);
	}

	//	g_activeBodies += bodyCount;
	return 1;
}


void GenericContactProcess (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
{
	int isHightField;
	NewtonBody* body;
	NewtonCollision* collision;
	NewtonCollisionInfoRecord info;

	isHightField = 1;
	body = NewtonJointGetBody0 (contactJoint);
	collision = NewtonBodyGetCollision(body);
	NewtonCollisionGetInfo(collision, &info);
	if (info.m_collisionType != SERIALIZE_ID_HEIGHTFIELD) {
		body = NewtonJointGetBody1 (contactJoint);
		collision = NewtonBodyGetCollision(body);
		NewtonCollisionGetInfo(collision, &info);
		isHightField  = (info.m_collisionType == SERIALIZE_ID_HEIGHTFIELD); 
	}

	#define HOLE_IN_TERRAIN 10
	if (isHightField) {
		void* nextContact;
		for (void* contact = NewtonContactJointGetFirstContact (contactJoint); contact; contact = nextContact) {
			int faceID;
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


void GetForceOnStaticBody (NewtonBody* body, NewtonBody* staticBody)
{
	for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint (body); joint; joint = NewtonBodyGetNextContactJoint (body, joint)) {
		NewtonBody* body0;
		NewtonBody* body1;

		body0 = NewtonJointGetBody0(joint);
		body1 = NewtonJointGetBody1(joint);
		if ((body0 == staticBody) || (body1 == staticBody)) {

			for (void* contact = NewtonContactJointGetFirstContact (joint); contact; contact = NewtonContactJointGetNextContact (joint, contact)) {
	
				dVector point(0.0f);
				dVector normal(0.0f);	
				dFloat forceMag;
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




static void ExtrudeFaces (void* userData, int vertexCount, const dFloat* faceVertec, int id)
{
	dFloat OFFSET = 0.1f;
	dFloat face[32][10];

	NewtonMesh* mesh = (NewtonMesh*) userData;

	// calculate the face normal
	dVector normal (0.0f);
	dVector p0 (faceVertec[0 * 3 + 0], faceVertec[0 * 3 + 1], faceVertec[0 * 3 + 2]);
	dVector p1 (faceVertec[1 * 3 + 0], faceVertec[1 * 3 + 1], faceVertec[1 * 3 + 2]);

	dVector e0 (p1 - p0);
	for (int i = 2; i < vertexCount; i ++) {
		dVector p2 (faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
		dVector e1 (p2 - p0);

		normal += e0 * e1;
		e0 = e1;
	}
	normal = normal.Scale (1.0f / dSqrt (normal % normal));

	dVector displacemnet (normal.Scale (OFFSET));

	// add the face displace by some offset
	for (int i = 0; i < vertexCount; i ++) {
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
	NewtonMeshAddFace (mesh, vertexCount, &face[0][0], 10 * sizeof (dFloat), id);


	// now add on face walk the perimeter and add a rivet face
	dVector q0 (faceVertec[(vertexCount - 1) * 3 + 0], faceVertec[(vertexCount - 1) * 3 + 1], faceVertec[(vertexCount - 1) * 3 + 2]);
	q0 += displacemnet;
	for (int i = 0; i < vertexCount; i ++) {
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
		NewtonMeshAddFace (mesh, 4, &face[0][0], 10 * sizeof (dFloat), id);
	}
}


NewtonMesh* CreateCollisionTreeDoubleFaces (NewtonWorld* world, NewtonCollision* optimizedDoubelFacesTree)
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
NewtonJoint* CheckIfBodiesCollide (NewtonBody* const body0, NewtonBody* const body1)
{
	for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint (body0); joint; joint = NewtonBodyGetNextContactJoint (body0, joint)) {
		if ((NewtonJointGetBody0(joint) == body1) || (NewtonJointGetBody1(joint) == body1)) {
			return joint;
		}
	}
	return NULL;
}


//to get the collision points
void HandlecollisionPoints (NewtonJoint* const contactjoint)
{
	NewtonBody* const body0 = NewtonJointGetBody0(contactjoint);
	NewtonBody* const body1 = NewtonJointGetBody1(contactjoint);
	for (void* contact = NewtonContactJointGetFirstContact (contactjoint); contact; contact = NewtonContactJointGetNextContact (contactjoint, contact)) {

		NewtonMaterial* material = NewtonContactGetMaterial (contact);

		// do whatever you want here
		//dFloat forceMag;
		dVector point(0.0f);
		dVector normal(0.0f);	
		//NewtonMaterialGetContactForce (material, &forceMag);
		NewtonMaterialGetContactPositionAndNormal (material, body0, &point.m_x, &normal.m_x);
		NewtonMaterialGetContactPositionAndNormal (material, body1, &point.m_x, &normal.m_x);

		// do whatever you want with the force
	}
}


void GetContactOnBody (NewtonBody* const body)
{
	for (NewtonJoint* joint = NewtonBodyGetFirstContactJoint (body); joint; joint = NewtonBodyGetNextContactJoint (body, joint)) {
		NewtonBody* const body0 = NewtonJointGetBody0(joint);
		NewtonBody* const body1 = NewtonJointGetBody1(joint);
		for (void* contact = NewtonContactJointGetFirstContact (joint); contact; contact = NewtonContactJointGetNextContact (joint, contact)) {
			NewtonMaterial* material = NewtonContactGetMaterial (contact);

			//dFloat forceMag;
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
void  PhysicsBodyDestructor (const NewtonBody* body)
{
//	RenderPrimitive* primitive;

	// get the graphic object form the rigid body
//	primitive = (RenderPrimitive*) NewtonBodyGetUserData (body);

	// destroy the graphic object
	//	delete primitive;
}


// add force and torque to rigid body
void  PhysicsApplyGravityForce (const NewtonBody* body, dFloat timestep, int threadIndex)
{
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	dFloat mass;

	NewtonBodyGetMass (body, &mass, &Ixx, &Iyy, &Izz);
	dVector dir(0.0f, 1.0f, 0.0f);
//	dVector dir(1.0f, 0.0f, 0.0f);
//mass = 0.0f;
	dVector force (dir.Scale (mass * DEMO_GRAVITY));
	NewtonBodySetForce (body, &force.m_x);
}


void GenericContactProcess (const NewtonJoint* contactJoint, dFloat timestep, int threadIndex)
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
//	NewtonBody* const body = NewtonJointGetBody0(contactJoint);
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


	NewtonBody* const body = NewtonJointGetBody0(contactJoint);
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
		//dFloat speed = NewtonMaterialGetContactNormalSpeed(material);

		//speed = NewtonMaterialGetContactNormalSpeed(material);
		// play sound base of the contact speed.
		//
	}
}



NewtonCollision* CreateConvexCollision (NewtonWorld* const world, const dMatrix& srcMatrix, const dVector& originalSize, PrimitiveType type, int materialID__)
{
	dVector size (originalSize);

	NewtonCollision* collision = NULL;
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
			collision = NewtonCreateSphere (world, size.m_x * 0.5f, 0, NULL); 
			break;
		}

		case _BOX_PRIMITIVE:
		{
			// create the collision 
			collision = NewtonCreateBox (world, size.m_x, size.m_y, size.m_z, 0, NULL); 
			break;
		}


		case _CONE_PRIMITIVE:
		{
			dFloat r = size.m_x * 0.5f;
			dFloat h = size.m_y;

			// create the collision 
			collision = NewtonCreateCone (world, r, h, 0, NULL); 
			break;
		}

		case _CYLINDER_PRIMITIVE:
		{
			// create the collision 
			collision = NewtonCreateCylinder (world, size.m_x * 0.5f, size.m_z * 0.5f, size.m_y, 0, NULL); 
			break;
		}


		case _CAPSULE_PRIMITIVE:
		{
			// create the collision 
			collision = NewtonCreateCapsule (world, size.m_x * 0.5f, size.m_z * 0.5f, size.m_y, 0, NULL); 
			break;
		}

		case _CHAMFER_CYLINDER_PRIMITIVE:
		{
			// create the collision 
			collision = NewtonCreateChamferCylinder (world, size.m_x * 0.5f, size.m_y, 0, NULL); 
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

			int count = 6;
			// populate the cloud with pseudo Gaussian random points
			for (int i = 6; i < SAMPLE_COUNT; i ++) {
				cloud [i].m_x = dGaussianRandom (size.m_x);
				cloud [i].m_y = dGaussianRandom (size.m_y);
				cloud [i].m_z = dGaussianRandom (size.m_z);
				count ++;
			}
			collision = NewtonCreateConvexHull (world, count, &cloud[0].m_x, sizeof (dVector), 0.01f, 0, NULL); 
			break;
		}

		case _REGULAR_CONVEX_HULL_PRIMITIVE:
		{
			// Create a clouds of random point around the origin
			#define STEPS_HULL 6
			//#define STEPS_HULL 3

			//dVector cloud [STEPS_HULL * 4 + 256];
			dFloat cloud [STEPS_HULL * 4 + 256][3];
			int count = 0;
			dFloat radius = size.m_y;
			dFloat height = size.m_x * 0.999f;
			dFloat x = - height * 0.5f;
			dMatrix rotation (dPitchMatrix(2.0f * dPi / STEPS_HULL));
			for (int i = 0; i < 4; i ++) {
				dFloat pad = ((i == 1) || (i == 2)) * 0.25f * radius;
				dVector p (x, 0.0f, radius + pad);
				x += 0.3333f * height;
				dMatrix acc (dGetIdentityMatrix());
				for (int j = 0; j < STEPS_HULL; j ++) {
					dVector tmp (acc.RotateVector(p)); 
					cloud[count][0] = tmp.m_x;
					cloud[count][1] = tmp.m_y;
					cloud[count][2] = tmp.m_z;
					acc = acc * rotation;
					count ++;
				}
			}

			collision = NewtonCreateConvexHull (world, count, &cloud[0][0], 3 * sizeof (dFloat), 0.02f, 0, NULL); 
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

NewtonBody* CreateSimpleBody (NewtonWorld* const world, void* const userData, dFloat mass, const dMatrix& matrix, NewtonCollision* const collision, int materialId, bool generalInertia)
{

	// calculate the moment of inertia and the relative center of mass of the solid
	//	dVector origin;
	//	dVector inertia;
	//	NewtonConvexCollisionCalculateInertialMatrix (collision, &inertia[0], &origin[0]);	
	//	dFloat Ixx = mass * inertia[0];
	//	dFloat Iyy = mass * inertia[1];
	//	dFloat Izz = mass * inertia[2];

	//create the rigid body
	NewtonBody* const rigidBody = generalInertia ? NewtonCreateAsymetricDynamicBody (world, collision, &matrix[0][0]) : NewtonCreateDynamicBody(world, collision, &matrix[0][0]);

	// set the correct center of gravity for this body (these function are for legacy)
	//	NewtonBodySetCentreOfMass (rigidBody, &origin[0]);
	//	NewtonBodySetMassMatrix (rigidBody, mass, Ixx, Iyy, Izz);

	// use a more convenient function for setting mass and inertia matrix
	NewtonBodySetMassProperties (rigidBody, mass, collision);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (rigidBody, userData);

	// assign the wood id
	NewtonBodySetMaterialGroupID (rigidBody, materialId);

	//  set continuous collision mode
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

NewtonBody* CreateSimpleSolid (DemoEntityManager* const scene, DemoMesh* const mesh, dFloat mass, const dMatrix& matrix, NewtonCollision* const collision, int materialId, bool generalInertia)
{
	dAssert (collision);

	// add an new entity to the world
	DemoEntity* const entity = new DemoEntity(matrix, NULL);
	scene->Append (entity);
	if (mesh) {
		entity->SetMesh(mesh, dGetIdentityMatrix());
	}
	return CreateSimpleBody (scene->GetNewton(), entity, mass, matrix, collision, materialId, generalInertia);
}



void AddPrimitiveArray (DemoEntityManager* const scene, dFloat mass, const dVector& origin, const dVector& size, int xCount, int zCount, dFloat spacing, PrimitiveType type, int materialID, const dMatrix& shapeOffsetMatrix, dFloat startElevation, dFloat offsetHigh)
{
	// create the shape and visual mesh as a common data to be re used
	NewtonWorld* const world = scene->GetNewton();
	NewtonCollision* const collision = CreateConvexCollision (world, shapeOffsetMatrix, size, type, materialID);

	// test collision mode
	//NewtonCollisionSetCollisonMode(collision, 0);

	DemoMesh* const geometry = new DemoMesh("primitive", collision, "smilli.tga", "smilli.tga", "smilli.tga");

	dMatrix matrix (dGetIdentityMatrix());
	for (int i = 0; i < xCount; i ++) {
		dFloat x = origin.m_x + (i - xCount / 2) * spacing;
		for (int j = 0; j < zCount; j ++) {
			dFloat z = origin.m_z + (j - zCount / 2) * spacing;

			matrix.m_posit.m_x = x;
			matrix.m_posit.m_z = z;
			dVector floor (FindFloor (world, dVector (matrix.m_posit.m_x, startElevation, matrix.m_posit.m_z, 0.0f), 2.0f * startElevation));
			matrix.m_posit.m_y = floor.m_y + size.m_y * 0.5f + offsetHigh;
			CreateSimpleSolid (scene, geometry, mass, matrix, collision, materialID);
		}
	}
	// do not forget to release the assets	
	geometry->Release(); 
	NewtonDestroyCollision (collision);
}


void CalculateAABB (const NewtonCollision* const collision, const dMatrix& matrix, dVector& minP, dVector& maxP)
{
	for (int i = 0; i < 3; i ++) {
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


void SetAutoSleepMode (NewtonWorld* const world, int mode)
{
	mode = mode ? 0 : 1;
	for (const NewtonBody* body = NewtonWorldGetFirstBody (world); body; body = NewtonWorldGetNextBody (world, body)) {
		NewtonBodySetAutoSleep (body, mode);
	}
}


class CollsionTreeFaceMap
{
	public:
	struct FaceInfo
	{
		//void* m_face;
		int m_materialIndex;
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

	static int CountFaces (void* const context, const dFloat* const polygon, int strideInBytes, const int* const indexArray, int indexCount)
	{
		CollsionTreeFaceMap* const me = (CollsionTreeFaceMap*) context;
		me->m_faceCount ++;
		return 1;
	}
	static int MarkFaces (void* const context, const dFloat* const polygon, int strideInBytes, const int* const indexArray, int indexCount)
	{
		CollsionTreeFaceMap* const me = (CollsionTreeFaceMap*) context;

		// repmap material index, by enumerating the face and storing the user material info at each face index
		int faceIndex = NewtonTreeCollisionGetFaceAttribute (me->m_collisionTree, indexArray, indexCount); 
		me->m_faceMapInfo[me->m_faceCount].m_materialIndex = faceIndex;
		NewtonTreeCollisionSetFaceAttribute (me->m_collisionTree, indexArray, indexCount, me->m_faceCount); 

		me->m_faceCount ++;
		return 1;
	}
	int m_faceCount;
	FaceInfo* m_faceMapInfo;
	NewtonCollision* m_collisionTree;
};


NewtonCollision* CreateCollisionTree (NewtonWorld* const world, DemoEntity* const entity, int materialID, bool optimize)
{
	// measure the time to build a collision tree
	unsigned64 timer0 = dGetTimeInMicrosenconds();

	// create the collision tree geometry
	NewtonCollision* collision = NewtonCreateTreeCollision(world, materialID);

	// set the application level callback
#ifdef USE_STATIC_MESHES_DEBUG_COLLISION
	NewtonStaticCollisionSetDebugCallback (collision, ShowMeshCollidingFaces);
#endif

	// prepare to create collision geometry
	NewtonTreeCollisionBeginBuild(collision);

	// iterate the entire geometry an build the collision
	for (DemoEntity* model = entity->GetFirst(); model; model = model->GetNext()) {

		dMatrix matrix (model->GetMeshMatrix() * model->CalculateGlobalMatrix(entity));
		DemoMesh* const mesh = (DemoMesh*)model->GetMesh();
		dAssert (mesh->IsType(DemoMesh::GetRttiType()));

		dFloat* const vertex = mesh->m_vertex;
		for (DemoMesh::dListNode* nodes = mesh->GetFirst(); nodes; nodes = nodes->GetNext()) {
			DemoSubMesh& segment = nodes->GetInfo();
			int matID = segment.m_textureHandle;
			for (int i = 0; i < segment.m_indexCount; i += 3) {
				dFloat face[3][3];
				for (int j = 0; j < 3; j ++) {
					int index = segment.m_indexes[i + j] * 3;
					face[j][0] = vertex[index + 0];
					face[j][1] = vertex[index + 1];
					face[j][2] = vertex[index + 2];
				}
				matrix.TransformTriplex (&face[0][0], 3 * sizeof (dFloat), &face[0][0], 3 * sizeof (dFloat), 3);

				// use material ids as physics materials 
				NewtonTreeCollisionAddFace(collision, 3, &face[0][0], 3 * sizeof (dFloat), matID);
			}
		}
	}
	NewtonTreeCollisionEndBuild(collision, optimize ? 1 : 0);


	// test Serialization
#if 0
	FILE* file = fopen ("serialize.bin", "wb");
	NewtonCollisionSerialize (world, collision, DemoEntityManager::SerializeFile, file);
	fclose (file);
	NewtonDestroyCollision (collision);

	file = fopen ("serialize.bin", "rb");
	collision = NewtonCreateCollisionFromSerialization (world, DemoEntityManager::DeserializeFile, file);
	fclose (file);
#endif	



	// measure the time to build a collision tree
	timer0 = (dGetTimeInMicrosenconds() - timer0) / 1000;

	return collision;
}



NewtonBody* CreateLevelMeshBody (NewtonWorld* const world, DemoEntity* const ent, bool optimize)
{
	NewtonCollision* const collision = CreateCollisionTree (world, ent, 0, optimize);

	// Get the root Matrix
	dMatrix matrix (ent->CalculateGlobalMatrix(NULL));

	// create the level rigid body
	NewtonBody* const level = NewtonCreateDynamicBody(world, collision, &matrix[0][0]);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (level, ent);

#if 0
	NewtonCollisionInfoRecord collisionInfo;
	NewtonCollisionGetInfo (collision, &collisionInfo);
	if (collisionInfo.m_collisionType == SERIALIZE_ID_TREE) {
		int count;
		dVector p0(-100, -100, -100);
		dVector p1(100, 100, 100);
		const dFloat* vertexArray;
		int vertexStrideInBytes;
		int vertexCount;
		int indexList[256];
		int attributeList[256/3];
		count = NewtonTreeCollisionGetVertexListTriangleListInAABB (collision, &p0[0], &p1[0], 
			&vertexArray, &vertexCount, &vertexStrideInBytes, 
			indexList, sizeof (indexList)/sizeof (indexList[0]), 
			attributeList); 
	}
#endif

	// set a destructor for this rigid body
	//NewtonBodySetDestructorCallback (m_level, Destructor);

	// release the collision tree (this way the application does not have to do book keeping of Newton objects
	NewtonDestroyCollision (collision);

	// now we will make a lookup table for quick material index lookup for face to index
	//CollsionTreeFaceMap faceMap (NewtonBodyGetCollision(level));
	return level;
}


NewtonBody* AddFloorBox(DemoEntityManager* const scene, const dVector& origin, const dVector& size)
{
	// create the shape and visual mesh as a common data to be re used
	NewtonWorld* const world = scene->GetNewton();
	const int materialID = NewtonMaterialGetDefaultGroupID(scene->GetNewton());
	NewtonCollision* const collision = CreateConvexCollision(world, dGetIdentityMatrix(), size, _BOX_PRIMITIVE, materialID);

	// test collision mode
	//NewtonCollisionSetCollisonMode(collision, 0);

	DemoMesh* const geometry = new DemoMesh("primitive", collision, "wood_3.tga", "wood_3.tga", "wood_3.tga");

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit = origin;
	matrix.m_posit.m_w = 1.0f;
	NewtonBody* const body = CreateSimpleSolid(scene, geometry, 0.0f, matrix, collision, materialID);
	// do not forget to release the assets	
	geometry->Release();
	NewtonDestroyCollision(collision);
	return body;
}

NewtonBody* CreateLevelMesh (DemoEntityManager* const scene, const char* const name, bool optimized)
{
	// load the scene from a ngd file format
	char fileName[2048];
	dGetWorkingFileName (name, fileName);
	scene->LoadScene (fileName);

	NewtonBody* levelBody = NULL;
	NewtonWorld* const world = scene->GetNewton();
	for (DemoEntityManager::dListNode* node = scene->GetLast(); node; node = node->GetPrev()) {
		DemoEntity* const ent = node->GetInfo();
		DemoMesh* const mesh = (DemoMesh*) ent->GetMesh();
		dAssert (mesh->IsType(DemoMesh::GetRttiType()));

		if (mesh) {
			const dString& namePtr = mesh->GetName();
			if (namePtr == "levelGeometry_mesh") {
				levelBody = CreateLevelMeshBody (world, ent, optimized);
				break;
			}
		}
	}
	return levelBody;
}


class MakeViualMesh: public dScene::dSceneExportCallback
{
	public: 
	MakeViualMesh (NewtonWorld* const world)
		:m_world (world)
	{
	}

	NewtonMesh* CreateVisualMesh (NewtonBody* const body, char* const name, int maxNameSize) const
	{
		// here the use should take the user data from the body create newtonMesh form it and return that back
		NewtonCollision* const collision = NewtonBodyGetCollision(body);
		NewtonMesh* const mesh = NewtonMeshCreateFromCollision(collision);

		sprintf (name, "visual Mesh");
		return mesh;
	}

	NewtonWorld* m_world;
};

void ExportScene (NewtonWorld* const world, const char* const fileName)
{
	MakeViualMesh context (world);
	dScene testScene (world);
	testScene.NewtonWorldToScene (world, &context);
	testScene.Serialize (fileName);
}


void CalculatePickForceAndTorque (const NewtonBody* const body, const dVector& pointOnBodyInGlobalSpace, const dVector& targetPositionInGlobalSpace, dFloat timestep)
{
	dFloat mass;
	dFloat Ixx;
	dFloat Iyy;
	dFloat Izz;
	const dFloat stiffness = 0.33f;
	const dFloat damping = -0.05f;

	NewtonBodyGetMass(body, &mass, &Ixx, &Iyy, &Izz);

	// calculate the desired impulse
	dVector posit(targetPositionInGlobalSpace - pointOnBodyInGlobalSpace);
	dVector impulse(posit.Scale(stiffness * mass));

	// apply linear impulse
	NewtonBodyApplyImpulseArray(body, 1, sizeof (dVector), &impulse[0], &pointOnBodyInGlobalSpace[0], timestep);

	// apply linear and angular damping
	dMatrix inertia;
	dVector linearMomentum(0.0f);
	dVector angularMomentum(0.0f);

	NewtonBodyGetOmega(body, &angularMomentum[0]);
	NewtonBodyGetVelocity(body, &linearMomentum[0]);


	NewtonBodyGetInertiaMatrix(body, &inertia[0][0]);

	angularMomentum = inertia.RotateVector(angularMomentum);
	angularMomentum = angularMomentum.Scale(damping);
	linearMomentum = linearMomentum.Scale(mass * damping);

	NewtonBodyApplyImpulsePair(body, &linearMomentum[0], &angularMomentum[0], timestep);
}


NewtonBody* MousePickBody (NewtonWorld* const nWorld, const dVector& origin, const dVector& end, dFloat& paramterOut, dVector& positionOut, dVector& normalOut)
{
	dMousePickClass rayCast;
	NewtonWorldRayCast(nWorld, &origin[0], &end[0], dMousePickClass::RayCastFilter, &rayCast, dMousePickClass::RayCastPrefilter, 0);

	if (rayCast.m_body) {
		positionOut = origin + (end - origin).Scale (rayCast.m_param);
		normalOut = rayCast.m_normal;
		paramterOut = rayCast.m_param;
	}
	return (NewtonBody*) rayCast.m_body;
}


void LoadLumberYardMesh(DemoEntityManager* const scene, const dVector& location, int shapeid)
{
	DemoEntity entity (dGetIdentityMatrix(), NULL);
	entity.LoadNGD_mesh ("lumber.ngd", scene->GetNewton());

	dTree<NewtonCollision*, DemoMesh*> filter;
	NewtonWorld* const world = scene->GetNewton();

	dFloat density = 15.0f;

	int defaultMaterialID = NewtonMaterialGetDefaultGroupID(scene->GetNewton());
	for (DemoEntity* child = entity.GetFirst(); child; child = child->GetNext()) {
		DemoMesh* const mesh = (DemoMesh*)child->GetMesh();
		if (mesh) {
			dAssert(mesh->IsType(DemoMesh::GetRttiType()));
			dTree<NewtonCollision*, DemoMesh*>::dTreeNode* node = filter.Find(mesh);
			if (!node) {
				// make a collision shape only for and instance
				dFloat* const array = mesh->m_vertex;
				dVector minBox(1.0e10f, 1.0e10f, 1.0e10f, 1.0f);
				dVector maxBox(-1.0e10f, -1.0e10f, -1.0e10f, 1.0f);

				for (int i = 0; i < mesh->m_vertexCount; i++) {
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
			dFloat mass = density * NewtonConvexCollisionCalculateVolume(shape);
			CreateSimpleSolid(scene, mesh, mass, matrix, shape, defaultMaterialID);
		}
	}

	// destroy all shapes
	while (filter.GetRoot()) {
		NewtonCollision* const shape = filter.GetRoot()->GetInfo();
		NewtonDestroyCollision(shape);
		filter.Remove(filter.GetRoot());
	}
}
