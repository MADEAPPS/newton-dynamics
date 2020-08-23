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

#include <toolbox_stdafx.h>
#include "OpenGlUtil.h"
#include "DebugDisplay.h"
#include "PlaneCollision.h"

#include <dList.h>
//#include <dGlueToCollada.h>

#define MAX_THREAD_FACES	8



class PlaneCollision
{
	public:
	PlaneCollision(NewtonWorld* nWorld, const dVector& plane)
	{
		int i;
		int j;
		dVector m_minBox (-1000.0f, -1000.0f, -1000.0f);
		dVector m_maxBox ( 1000.0f,  1000.0f,  1000.0f);

		// get the transformation matrix that takes the plane to the world local space
		m_rotation = dgGrammSchmidt(plane);

		// build a unit grid in local space (this will be the shadow at projection of the collision aabb)
		m_sphape[0] = dVector (0.0f,  1.0f,  1.0f);
		m_sphape[1] = dVector (0.0f, -1.0f,  1.0f);
		m_sphape[2] = dVector (0.0f, -1.0f, -1.0f);
		m_sphape[3] = dVector (0.0f,  1.0f, -1.0f);

		// save the plane
		m_plane = plane;

		for (i = 0; i < MAX_THREAD_FACES; i ++) {
			m_attribute[i] = 0;
			m_faceIndices[i] = 4;
			for (j = 0; j < 4; j ++) {
				m_indexArray[i][j] = j;
			}
		}

		// create a Newton user collision 
		m_collision = NewtonCreateUserMeshCollision (nWorld, &m_minBox[0], &m_maxBox[0], this, 
					                                 PlaneCollisionCollideCallback, PlaneMeshCollisionRayHitCallback, 
													 PlaneCollisionDestroyCallback, PlaneCollisionGetCollisionInfo, 
													 PlaneCollisionGetFacesInAABB, 0);
	}
	~PlaneCollision(void)
	{
	}

	static void PlaneCollisionDestroyCallback(void* userData)
	{
		PlaneCollision* me;
		me = (PlaneCollision*) userData;
		delete me;
	}

	static void PlaneCollisionCollideCallback (NewtonUserMeshCollisionCollideDesc* collideDesc)
	{
		float t;
		float s;
		dInt32 i;
		dFloat dist;
		dInt32 threadNumber;
		PlaneCollision* me;
		me = (PlaneCollision*) collideDesc->m_userData;

		dVector p0 (collideDesc->m_boxP0[0], collideDesc->m_boxP0[1], collideDesc->m_boxP0[2]);
		dVector p1 (collideDesc->m_boxP1[0], collideDesc->m_boxP1[1], collideDesc->m_boxP1[2]);

		dVector suportVertex ((me->m_plane.m_x > 0.0f) ? p0.m_x : p1.m_x, 
							  (me->m_plane.m_y > 0.0f) ? p0.m_y : p1.m_y,
							  (me->m_plane.m_z > 0.0f) ? p0.m_z : p1.m_z);


		dist = me->m_plane % suportVertex + me->m_plane.m_w;
		if (dist < 0.25f) {
			// calculate the aabb center
			dVector centre ((p1 + p0).Scale (0.5f));

			//find the projection of center point over the plane
			t = - (me->m_plane % centre + me->m_plane.m_w);
			centre += me->m_plane.Scale (t);

			//know calculate the scale factor
			dVector size (p1 - p0);
			s = max (size.m_x, max (size.m_y, size.m_z));

			threadNumber = collideDesc->m_threadNumber;

			// initialize the callback data structure
			collideDesc->m_faceCount = 1;
			collideDesc->m_vertexStrideInBytes = sizeof (dVector);
			collideDesc->m_userAttribute = &me->m_attribute[threadNumber];
			collideDesc->m_faceIndexCount = &me->m_faceIndices[threadNumber];
			collideDesc->m_faceVertexIndex = &me->m_indexArray[threadNumber][0];
			collideDesc->m_vertex = &me->m_collisionVertex[threadNumber][0][0];
			for (i = 0; i < 4; i ++) {
				me->m_collisionVertex[threadNumber][i] = centre + me->m_rotation.RotateVector(me->m_sphape[i].Scale (s));
			}

			// show debug display info
			if (DebugDisplayOn()) {
				dMatrix matrix;
				NewtonWorld* world;
				dVector face[64];

				NewtonBodyGetMatrix (collideDesc->m_polySoupBody, &matrix[0][0]);
				matrix.TransformTriplex (face, sizeof (dVector), 
										 &me->m_collisionVertex[threadNumber][0], sizeof (dVector), 4);
				
				world = NewtonBodyGetWorld (collideDesc->m_polySoupBody);
				// critical section lock
				NewtonWorldCriticalSectionLock (world);
				DebugDrawPolygon (4, &face[0]);
				// unlock the critical section
				NewtonWorldCriticalSectionUnlock (world);
			}

		}
	}

	static dFloat PlaneMeshCollisionRayHitCallback (NewtonUserMeshCollisionRayHitDesc* rayDesc)
	{
		dFloat t;
		PlaneCollision* me;

		dVector q0 (rayDesc->m_p0[0], rayDesc->m_p0[1], rayDesc->m_p0[2]);
		dVector q1 (rayDesc->m_p1[0], rayDesc->m_p1[1], rayDesc->m_p1[2]);
		dVector dq (q1 - q0);

		// calculate intersection between point lien a plane and return intesetion parameter
		me = (PlaneCollision*) rayDesc->m_userData;
		t = -(me->m_plane % q0 + me->m_plane.m_w) / (me->m_plane % dq);
		if ((t > 0.0f) && (t < 1.0f)) {
			rayDesc->m_normalOut[0] = me->m_plane[0];
			rayDesc->m_normalOut[1] = me->m_plane[1];
			rayDesc->m_normalOut[2] = me->m_plane[2];
		} else {
			t = -1.0f;
		}

		return t;
	}

	static void PlaneCollisionGetCollisionInfo (void* userData, NewtonCollisionInfoRecord* infoRecord)
	{
		_ASSERTE (0);
/*
		PlaneCollision* me;

		me = (PlaneCollision*) userData;
		infoRecord->m_collisionType = D_PLANE_COLLISON_ID;

		infoRecord->m_paramArray[0] = me->m_plane.m_x; 
		infoRecord->m_paramArray[1] = me->m_plane.m_y; 
		infoRecord->m_paramArray[2] = me->m_plane.m_z; 
		infoRecord->m_paramArray[3] = me->m_plane.m_w; 
*/
	}

	static int PlaneCollisionGetFacesInAABB (
		void* me, 
		const dFloat* p0, 
		const dFloat* p1,
		const dFloat** vertexArray, 
		int* vertexCount, 
		int* vertexStrideInBytes, 
		const int* indexList, 
		int maxIndexCount,
		const int* userDataList)
	{
		_ASSERTE (0);
		return 0;
	}


	dVector m_sphape[4];
	dVector m_plane;

	dMatrix m_rotation;
	NewtonCollision* m_collision;

	dInt32 m_attribute[MAX_THREAD_FACES];
	dInt32 m_faceIndices[MAX_THREAD_FACES];
	dInt32 m_indexArray[MAX_THREAD_FACES][4];
	dVector m_collisionVertex[MAX_THREAD_FACES][4];


};

NewtonCollision* CreatePlaneCollidion (NewtonWorld* nWorld, const dVector& Plane)
{
	PlaneCollision* plane;

	plane = new PlaneCollision (nWorld, Plane);
	return plane->m_collision;
}

