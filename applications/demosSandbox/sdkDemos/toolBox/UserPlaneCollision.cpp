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
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DebugDisplay.h"
#include "UserPlaneCollision.h"

#define PASS_A_QUAD
#define MAX_THREAD_FACES	32

class dInfinitePlane
{
	public:
	dInfinitePlane (NewtonWorld* const world, const dVector& plane)
		:m_minBox (-0.1f, -2000.0f, -2000.0f, 0.0f)
		,m_maxBox ( 0.1f,  2000.0f,  2000.0f, 0.0f)

	{
		// get the transformation matrix that takes the plane to the world local space
		m_rotation = dGrammSchmidt(plane);

		// build a unit grid in local space (this will be the shadow at projection of the collision aabb)
		m_unitSphape[0] = dVector (0.0f,  1.0f,  1.0f);
		m_unitSphape[1] = dVector (0.0f, -1.0f,  1.0f);
		m_unitSphape[2] = dVector (0.0f, -1.0f, -1.0f);
		m_unitSphape[3] = dVector (0.0f,  1.0f, -1.0f);

		// save the plane in local space
		m_plane = m_rotation.UntransformPlane (plane);

#ifdef PASS_A_QUAD
		// passing a single quad	
		for (int i = 0; i < MAX_THREAD_FACES; i ++) {
			m_faceIndices[i][0] = 4;
			// face attribute
			m_indexArray[i][4] = 0;
			// face normal
			m_indexArray[i][4 + 1] = 4;
			// face area (the plane is clipped around the box, the face size is always optimal)
			m_indexArray[i][4 + 2 + 4] = 0;

			for (int j = 0; j < 4; j ++) {
				// face vertex index
				m_indexArray[i][j] = j;
				// face adjacent index (infinite plane does not have shared edge with other faces)
				m_indexArray[i][j + 4 + 2] = 4;
			}
		}
#else
		// passing two triangle	
		for (int i = 0; i < MAX_THREAD_FACES; i ++) {
			// first triangle
			{
				// index count
				m_faceIndices[i][0] = 3;
				
				// face indices
				m_indexArray[i][0] = 0;
				m_indexArray[i][1] = 1;
				m_indexArray[i][2] = 2;

				// face attribute
				m_indexArray[i][3] = 0;

				// face normal
				m_indexArray[i][4] = 4;

				// face adjacent index (infinite plane does not have shared edge with other faces)
				m_indexArray[i][5] = 4;
				m_indexArray[i][6] = 4;
				m_indexArray[i][7] = 4;

				// face area (the plane is clipped around the box, the face size is always optimal)
				m_indexArray[i][8] = 0;
			}

			// second triangle
			{
				// index count
				m_faceIndices[i][1] = 3;
				
				// face indices
				m_indexArray[i][0 + 9] = 0;
				m_indexArray[i][1 + 9] = 2;
				m_indexArray[i][2 + 9] = 3;

				// face attribute
				m_indexArray[i][3 + 9] = 0;

				// face normal
				m_indexArray[i][4 + 9] = 4;

				// face adjacent index (infinite plane does not have shared edge with other faces)
				m_indexArray[i][5 + 9] = 4;
				m_indexArray[i][6 + 9] = 4;
				m_indexArray[i][7 + 9] = 4;

				// face area (the plane is clipped around the box, the face size is always optimal)
				m_indexArray[i][8 + 9] = 0;
			}
		}
#endif

		// create a Newton user collision 
		m_collision = NewtonCreateUserMeshCollision (world, &m_minBox[0], &m_maxBox[0], this, 
													 PlaneCollisionCollideCallback, PlaneMeshCollisionRayHitCallback, 
													 PlaneCollisionDestroyCallback, PlaneCollisionGetCollisionInfo, 
													 PlaneCollisionAABBOverlapTest, PlaneCollisionGetFacesInAABB, 
													 UserCollisionSerializationCallback, 0);

		// set a debug display call back
		NewtonStaticCollisionSetDebugCallback (m_collision, ShowMeshCollidingFaces);

		// set the collisoin offset Matrix;
		NewtonCollisionSetMatrix(m_collision, &m_rotation[0][0]);
		
	}

	~dInfinitePlane(void)
	{
	}

	private:
	static void PlaneCollisionDestroyCallback(void* userData)
	{
		dInfinitePlane* const me = (dInfinitePlane*) userData;
		delete me;
	}

	static int PlaneCollisionGetFacesInAABB (void* me, const dFloat* p0, const dFloat* p1, const dFloat** vertexArray, int* vertexCount, int* vertexStrideInBytes, const int* indexList, int maxIndexCount, const int* userDataList)
	{
		dAssert (0);
		return 0;
	}


	static int PlaneCollisionAABBOverlapTest (void* userData, const dFloat* const box0, const dFloat* const box1)
	{
		const dInfinitePlane* const me = (dInfinitePlane*) userData;

		dVector minBox (box0[0], box0[1], box0[2], 0.0f);
		dFloat test = me->m_plane.DotProduct3(minBox) +  me->m_plane.m_w;
		if (test > 0.0f) {
			return 0;
		}

		dVector maxBox (box1[0], box1[1], box1[2], 0.0f);
		test = me->m_plane.DotProduct3(maxBox) +  me->m_plane.m_w;
		if (test < 0.0f) {
			return 0;
		}
		
		return 1;
	}

	static void UserCollisionSerializationCallback (void* const userData, NewtonSerializeCallback function, void* const serializeHandle)
	{
		//dAssert (0);
		// do nothing for now, until some one use thismax missing hedaare
	}

	static void PlaneCollisionGetCollisionInfo (void* const userData, NewtonCollisionInfoRecord* const infoRecord)
	{
		dInfinitePlane* const me = (dInfinitePlane*) userData;
		infoRecord->m_collisionType = SERIALIZE_ID_USERMESH;
		infoRecord->m_paramArray[0] = me->m_plane.m_x; 
		infoRecord->m_paramArray[1] = me->m_plane.m_y; 
		infoRecord->m_paramArray[2] = me->m_plane.m_z; 
		infoRecord->m_paramArray[3] = me->m_plane.m_w; 
	}


	static dFloat PlaneMeshCollisionRayHitCallback (NewtonUserMeshCollisionRayHitDesc* const rayDesc)
	{
		dVector q0 (rayDesc->m_p0[0], rayDesc->m_p0[1], rayDesc->m_p0[2]);
		dVector q1 (rayDesc->m_p1[0], rayDesc->m_p1[1], rayDesc->m_p1[2]);
		dVector dq (q1 - q0);

		// calculate intersection between point lien a plane and return intersection parameter
		dInfinitePlane* const me = (dInfinitePlane*) rayDesc->m_userData;
		dFloat t = -(me->m_plane.DotProduct3(q0) + me->m_plane.m_w) / (me->m_plane.DotProduct3(dq));
		if ((t > 0.0f) && (t < 1.0f)) {
			rayDesc->m_normalOut[0] = me->m_plane[0];
			rayDesc->m_normalOut[1] = me->m_plane[1];
			rayDesc->m_normalOut[2] = me->m_plane[2];
		} else {
			t = 1.2f;
		}
		return t;
	}


	static void PlaneCollisionCollideCallbackConstinue (NewtonUserMeshCollisionCollideDesc* const collideDesc, const void* const continueCollisionHandle)
	{
		dInfinitePlane* const me = (dInfinitePlane*) collideDesc->m_userData;

		// build that aabb of each face and submit only the one that pass the test.
		if (NewtonUserMeshCollisionContinuousOverlapTest (collideDesc, continueCollisionHandle, &me->m_minBox[0], &me->m_maxBox[0])) {
			const dVector& p0 = me->m_minBox;
			const dVector& p1 = me->m_maxBox;
			dVector centre ((p1 + p0).Scale (0.5f));

			//find the projection of center point over the plane
			dFloat t = - (me->m_plane.DotProduct3(centre) + me->m_plane.m_w);
			centre += me->m_plane.Scale (t);

			//know calculate the scale factor
			dVector size (p1 - p0);
			dFloat s = dMax(size.m_x, dMax (size.m_y, size.m_z)) * 0.5f;

			dInt32 threadNumber = collideDesc->m_threadNumber;

			// initialize the callback data structure
#ifdef PASS_A_QUAD
			collideDesc->m_faceCount = 1;
#else
			collideDesc->m_faceCount = 2;
#endif
			collideDesc->m_vertexStrideInBytes = sizeof (dVector);
			collideDesc->m_faceIndexCount = &me->m_faceIndices[threadNumber][0];
			collideDesc->m_faceVertexIndex = &me->m_indexArray[threadNumber][0];
			collideDesc->m_vertex = &me->m_collisionVertex[threadNumber][0][0];
			dVector* const polygon = &me->m_collisionVertex[threadNumber][0];
			for (int i = 0; i < 4; i ++) {
				polygon[i] = centre + me->m_unitSphape[i].Scale (s);
			}
			// save face normal
			polygon[4] =  me->m_plane;

			// show debug display info
			if (DebugDisplayOn()) {
				dMatrix matrix;
				dVector face[64];

				NewtonBodyGetMatrix (collideDesc->m_polySoupBody, &matrix[0][0]);
				matrix.TransformTriplex (&face[0].m_x, sizeof (dVector), &polygon[0].m_x, sizeof (dVector), 4);

				NewtonWorld* const world = NewtonBodyGetWorld (collideDesc->m_polySoupBody);
				// critical section lock
				NewtonWorldCriticalSectionLock (world, threadNumber);
				//DebugDrawPolygon (4, &face[0]);
				// unlock the critical section
				NewtonWorldCriticalSectionUnlock (world);
			}
		}
	}


	static void PlaneCollisionCollideCallbackDescrete (NewtonUserMeshCollisionCollideDesc* const collideDesc)
	{
		dInfinitePlane* const me = (dInfinitePlane*) collideDesc->m_userData;

		dVector p0 (collideDesc->m_boxP0[0], collideDesc->m_boxP0[1], collideDesc->m_boxP0[2], 0.0f);
		dVector p1 (collideDesc->m_boxP1[0], collideDesc->m_boxP1[1], collideDesc->m_boxP1[2], 0.0f);
		dVector suportVertex ((me->m_plane.m_x > 0.0f) ? p0.m_x : p1.m_x, (me->m_plane.m_y > 0.0f) ? p0.m_y : p1.m_y, (me->m_plane.m_z > 0.0f) ? p0.m_z : p1.m_z);

		dFloat dist = me->m_plane.DotProduct3(suportVertex) + me->m_plane.m_w;
		if (dist < 0.25f) {
			// calculate the aabb center
			dVector centre ((p1 + p0).Scale (0.5f));

			//find the projection of center point over the plane
			dFloat t = - (me->m_plane.DotProduct3(centre) + me->m_plane.m_w);
			centre += me->m_plane.Scale (t);

			//know calculate the scale factor
			dVector size (p1 - p0);
			dFloat s = dMax(size.m_x, dMax (size.m_y, size.m_z));

			dInt32 threadNumber = collideDesc->m_threadNumber;

			// initialize the callback data structure
#ifdef PASS_A_QUAD
			collideDesc->m_faceCount = 1;
#else
			collideDesc->m_faceCount = 2;
#endif
			collideDesc->m_vertexStrideInBytes = sizeof (dVector);
			collideDesc->m_faceIndexCount = &me->m_faceIndices[threadNumber][0];
			collideDesc->m_faceVertexIndex = &me->m_indexArray[threadNumber][0];
			collideDesc->m_vertex = &me->m_collisionVertex[threadNumber][0][0];
			dVector* const polygon = &me->m_collisionVertex[threadNumber][0];
			for (int i = 0; i < 4; i ++) {
				polygon[i] = centre + me->m_unitSphape[i].Scale (s);
			}
			// save face normal
			polygon[4] =  me->m_plane;

			// show debug display info
			if (DebugDisplayOn()) {
				dMatrix matrix;
				dVector face[64];

				NewtonBodyGetMatrix (collideDesc->m_polySoupBody, &matrix[0][0]);
				matrix.TransformTriplex (&face[0].m_x, sizeof (dVector), &polygon[0].m_x, sizeof (dVector), 4);

				NewtonWorld* const world = NewtonBodyGetWorld (collideDesc->m_polySoupBody);
				// critical section lock
				NewtonWorldCriticalSectionLock (world, threadNumber);
				//DebugDrawPolygon (4, &face[0]);
				// unlock the critical section
				NewtonWorldCriticalSectionUnlock (world);
			}
		}
	}

	static void PlaneCollisionCollideCallback (NewtonUserMeshCollisionCollideDesc* const collideDesc, const void* const continueCollisionHandle)
	{
		if (continueCollisionHandle) {
			PlaneCollisionCollideCallbackConstinue (collideDesc, continueCollisionHandle);
		} else {
			PlaneCollisionCollideCallbackDescrete (collideDesc);
		}
	}

	public:
	static DemoMesh* CreateVisualMesh (const dVector& plane, int shader)
	{
		dAssert (0);
		return NULL;
/*
		// make a visual entity
		DemoMesh* const mesh = new DemoMesh ("userInfinitePlane", shader);

		// build a unit grid in local space (this will be the shadow at projection of the collision aabb)
		dFloat size = 1000.0f;
		dVector shape[4];
		shape[0] = dVector (0.0f,  1.0f,  1.0f);
		shape[1] = dVector (0.0f, -1.0f,  1.0f);
		shape[2] = dVector (0.0f, -1.0f, -1.0f);
		shape[3] = dVector (0.0f,  1.0f, -1.0f);

		dFloat UVtileSize = 16;
		mesh->AllocVertexData (4);
		for (int i = 0; i < 4; i ++) {
			mesh->m_vertex[i * 3  + 0] = shape[i].m_x * size;
			mesh->m_vertex[i * 3  + 1] = shape[i].m_y * size;
			mesh->m_vertex[i * 3  + 2] = shape[i].m_z * size;

			mesh->m_normal[i * 3  + 0] = 1.0f;
			mesh->m_normal[i * 3  + 1] = 0.0f;
			mesh->m_normal[i * 3  + 2] = 0.0f;

			mesh->m_uv[i * 2  + 0] = shape[0].m_y * UVtileSize;
			mesh->m_uv[i * 2  + 1] = shape[0].m_z * UVtileSize;
		}

		DemoSubMesh* const subMesh = mesh->AddSubMesh();
		subMesh->AllocIndexData (6);
		for (int i = 0; i < 2; i ++) {
			subMesh->m_indexes[i * 3 + 0] = 0;
			subMesh->m_indexes[i * 3 + 1] = i + 1;
			subMesh->m_indexes[i * 3 + 2] = i + 2;
		}

		mesh->OptimizeForRender();
		return mesh;
*/
	}

/*
	static DemoEntity* CreateVisualMesh (DemoEntityManager* const scene, const dVector& plane)
	{
		// make a visual entity
		DemoMesh____* const mesh = CreateVisualMesh (plane);

		// get the transformation matrix that takes the plane to the world local space
		dMatrix matrix (dgGrammSchmidt(plane));
		matrix.m_posit = plane.Scale (-plane.m_w / (plane % plane));
		matrix.m_posit.m_w = 1.0f;

		DemoEntity* const entity = new DemoEntity(matrix, NULL);
		scene->Append (entity);
		entity->SetMesh(mesh, GetIdentityMatrix());
		mesh->Release();

		return entity;
	}

    static void TestAddingUserMeshToSceneCollsion (NewtonWorld* const world)
    {
        NewtonCollision* const sceneCollision = NewtonCreateSceneCollision (world, 0);
        dMatrix matrix (GetIdentityMatrix());
        NewtonBody* const body = NewtonCreateDynamicBody(world, sceneCollision, &matrix[0][0]);
        NewtonDestroyCollision(sceneCollision);

        // make a use mesh collision for testing
        dVector minBox(-100.0f, -100.0f, -100.0f, 0.0f);
        dVector maxBox(100.0f, 100.0f, 100.0f, 0.0f);
        NewtonCollision* const collision0 = NewtonCreateUserMeshCollision (world, &minBox[0], &maxBox[0], NULL, 
            NULL, NULL, NULL, NULL, NULL, NULL, NULL, 0);
        NewtonCollision* const collision1 = NewtonCreateUserMeshCollision (world, &minBox[0], &maxBox[0], NULL, 
            NULL, NULL, NULL, NULL, NULL, NULL, NULL, 0);
        NewtonCollision* const collision2 = NewtonCreateUserMeshCollision (world, &minBox[0], &maxBox[0], NULL, 
            NULL, NULL, NULL, NULL, NULL, NULL, NULL, 0);

        // add some use mesh collision
        NewtonCollision* const rootScene = NewtonBodyGetCollision(body);
        NewtonSceneCollisionBeginAddRemove(rootScene);
        void* const node0 = NewtonSceneCollisionAddSubCollision(rootScene, collision0);
        void* const node1 = NewtonSceneCollisionAddSubCollision(rootScene, collision1);
        void* const node2 = NewtonSceneCollisionAddSubCollision(rootScene, collision2);
        NewtonSceneCollisionEndAddRemove(rootScene);
        NewtonDestroyCollision(collision0);
        NewtonDestroyCollision(collision1);
        NewtonDestroyCollision(collision2);

        // test empty the sub collision shapes
        NewtonCollision* const rootScene1 = NewtonBodyGetCollision(body);
        NewtonSceneCollisionBeginAddRemove(rootScene1);
        NewtonSceneCollisionRemoveSubCollision(rootScene1, node0);
        NewtonSceneCollisionRemoveSubCollision(rootScene1, node2);
        NewtonSceneCollisionRemoveSubCollision(rootScene1, node1);
        NewtonSceneCollisionEndAddRemove(rootScene1);

        // delete the body
        NewtonDestroyBody(body);
    }


	static NewtonBody* CreateInfinitePlane (DemoEntityManager* const scene, const dVector& plane)
	{
        //TestAddingUserMeshToSceneCollsion (scene->GetNewton());

		// create the Plane collision
		dInfinitePlane* const planeCollision = new dInfinitePlane (scene->GetNewton(), plane);

		// create the the rigid body for
		dMatrix matrix (GetIdentityMatrix());
		NewtonBody* const body = NewtonCreateDynamicBody(scene->GetNewton(), planeCollision->m_collision, &matrix[0][0]);

		// release the collision tree (this way the application does not have to do book keeping of Newton objects
		NewtonDestroyCollision (planeCollision->m_collision);
		planeCollision->m_collision = NewtonBodyGetCollision(body);

		// create a visual mesh
		DemoEntity* const entity = CreateVisualMesh (scene, plane);

		// save the pointer to the graphic object with the body.
		NewtonBodySetUserData (body, entity);

		return body;
	}
*/

	// the box is define in local space 
	dVector m_minBox;
	dVector m_maxBox;
	dVector m_plane;
	dMatrix m_rotation;
	NewtonCollision* m_collision;
	dVector m_unitSphape[4];

	// this is local per thread data 
	dInt32 m_faceIndices[MAX_THREAD_FACES][2];
	dInt32 m_indexArray[MAX_THREAD_FACES][2 * 9]; // 2 trinagles of (3 indices + 1 attribute + 1 index normal + 3 adjacent edge normals + 1 face diagonal size) = 9 * 2indices
	dVector m_collisionVertex[MAX_THREAD_FACES][5];   // 4 vertex + 1 face normal
};


NewtonCollision* CreateInfinitePlane (NewtonWorld* const world, const dVector& planeEquation)
{
	dInfinitePlane* const planeCollision = new dInfinitePlane (world, planeEquation);
	return planeCollision->m_collision;
}

DemoMesh* CreateVisualPlaneMesh (const dVector& plane, int shader)
{
	return dInfinitePlane::CreateVisualMesh (plane, shader);
}
