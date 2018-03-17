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



// UserHeightFieldCollision.cpp: implementation of the UserHeightFieldCollision class.
//
//////////////////////////////////////////////////////////////////////
#include <toolbox_stdafx.h>
#include "SkyBox.h"
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"
#include "DebugDisplay.h"
#include "UserHeightFieldCollision.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#if 0
#define TEST_INFO_AND_AABB

#ifdef TEST_INFO_AND_AABB
static void GetCollisionInfo (void* userData, NewtonCollisionInfoRecord* infoRecord)
{
	// copy here what ever information you wan t pass to the APP;
	UserHeightFieldCollision* me = (UserHeightFieldCollision*) userData;

	//we will pass the info int the HightField Structure bu the APP can use the extra space
	infoRecord->m_heightField.m_width = HEIGHT_SIZE;
	infoRecord->m_heightField.m_height = HEIGHT_SIZE;
	infoRecord->m_heightField.m_verticalScale = HIGHTSCALE_SIZE;
	infoRecord->m_heightField.m_horizonalScale = CELL_SIZE;
	infoRecord->m_heightField.m_elevation = (unsigned short*) me->GetElevationMap ();
	infoRecord->m_heightField.m_gridsDiagonals = 0;
	infoRecord->m_heightField.m_atributes = NULL;
}


static int UserMeshCollisionGetFacesInAABB (
	void* userData, const dFloat* p0, const dFloat* p1,
    const dFloat** vertexArray, 
	int* vertexCount, int* vertexStrideInBytes, 
    const int* indexList, int maxIndexCount, const int* userDataList)
{
	// this is implementation dependent, 			
	// The application must find all that Faces intersecting BBox p0-p1 and copy then in to the passed parameters.

	// copy the pointer vertexArray 
	// copy the vertex count into vertexCount
	// copy the vertex stride into vertexStrideInBytes

	// copy each index of the triangle list into indexList, do no copy more indices than maxIndexCount
	// for each face copy the face attribute into pointer userDataList

	// the implementation of this function is very similar to function
	// void  UserHeightFieldCollision::MeshCollisionCollideCallback (NewtonUserMeshCollisionCollideDesc* collideDesc)
	//at the end of this file

	// it must returbn the Face Count

	return 0;
}

#endif




UserHeightFieldCollision::UserHeightFieldCollision(NewtonWorld* nWorld)
	:RenderPrimitive()
{
dAssert (0);
/*
	int width;
	int height;
	int index;
	int vertexIndex;
	dFloat minY;
	dFloat maxY;
	OGLMesh* geometry;
	dSubMesh* segment;
	NewtonCollision* collision;

	width = HEIGHT_SIZE;
	height = HEIGHT_SIZE;

	// build a simple height field array. Note this could be load it from a texture map file
	for (int z = 0; z < height; z ++) {
		for (int x = 0; x < width; x ++) {
			dFloat y;
			// add sine wave for rolling terrain
			y = (5.0f * dSin(x/5.0f) + 7.0f * dCos((z)/7.0f)) * 1.5f;
			m_heightField[z][x] = y;
		}
	}

	minY =  1.0e10f;
	maxY = -1.0e10f;


	geometry = new OGLMesh("hightMap");
	AddMesh (geometry);
	geometry->Release();

	geometry->AllocVertexData(width * height);
	for (int z = 0; z < height; z ++) {
		dInt32 z0;
		dInt32 z1;
		z0 = ((z - 1) < 0) ? 0 : z - 1;
		z1 = ((z + 1) > (height - 1)) ? height - 1 : z + 1 ;
		for (int x = 0; x < width; x ++) {
			dInt32 x0;
			dInt32 x1;

			x0 = ((x - 1) < 0) ? 0 : x - 1;
			x1 = ((x + 1) > (width - 1)) ? width - 1 : x + 1 ;

			dVector p0 (CELL_SIZE * x0, m_heightField[z][x1], CELL_SIZE * z);
			dVector p1 (CELL_SIZE * x1, m_heightField[z][x0], CELL_SIZE * z);
			dVector x10 (p1 - p0);

			dVector q0 (CELL_SIZE * x, m_heightField[z0][x], CELL_SIZE * z0);
			dVector q1 (CELL_SIZE * x, m_heightField[z1][x], CELL_SIZE * z1);
			dVector z10 (q1 - q0);

			dVector normal (z10 * x10);
			normal = normal.Scale (dSqrt (1.0f / (normal % normal)));
			dVector point (CELL_SIZE * x, m_heightField[z][x], CELL_SIZE * z);

			minY = point.m_y < minY ? point.m_y : minY;
			maxY = point.m_y > maxY ? point.m_y : maxY;

			geometry->m_vertex[(z * width + x) * 3 + 0] = point.m_x;
			geometry->m_vertex[(z * width + x) * 3 + 1] = point.m_y;
			geometry->m_vertex[(z * width + x) * 3 + 2] = point.m_z;

			geometry->m_normal[(z * width + x) * 3 + 0] = normal.m_x;
			geometry->m_normal[(z * width + x) * 3 + 1] = normal.m_y;
			geometry->m_normal[(z * width + x) * 3 + 2] = normal.m_z;

			geometry->m_uv[(z * width + x) * 2 + 0] = x * TEXTURE_SCALE;
			geometry->m_uv[(z * width + x) * 2 + 1] = z * TEXTURE_SCALE;
		}
	}

	segment = geometry->AddSubMesh();
	segment->m_textureHandle = LoadTexture("GrassAndDirt.tga");

	segment->AllocIndexData(((width - 1) * (height - 1)) * 6);

	index = 0;
	vertexIndex = 0;
	for (int z = 0; z < height - 1; z ++) {
		vertexIndex = z * width;
		for (int x = 0; x < width - 1; x ++) {
			segment->m_indexes[index + 0] = GLushort (vertexIndex);
			segment->m_indexes[index + 1] = GLushort (vertexIndex + width + 1);
			segment->m_indexes[index + 2] = GLushort (vertexIndex + 1);
			index += 3;

			segment->m_indexes[index + 0] = GLushort (vertexIndex);
			segment->m_indexes[index + 1] = GLushort (vertexIndex + width);
			segment->m_indexes[index + 2] = GLushort (vertexIndex + width + 1);
			index += 3;
			vertexIndex ++;
		}
	}

	// Optimize the mesh for hardware rendering if possible
	geometry->OptimizeForRender(); 

	
	// create a rigid body and collision geometry
	m_minBox.m_x = 0.0f;
	m_minBox.m_z = 0.0f;
	m_minBox.m_y = minY;
	m_maxBox.m_x = CELL_SIZE * width;
	m_maxBox.m_z = CELL_SIZE * height;
	m_maxBox.m_y = maxY;

#ifdef TEST_INFO_AND_AABB
	collision = NewtonCreateUserMeshCollision (nWorld, &m_minBox[0], &m_maxBox[0], this, MeshCollisionCollideCallback, UserMeshCollisionRayHitCallback, NULL, GetCollisionInfo, UserMeshCollisionGetFacesInAABB, 0);

	NewtonCollisionInfoRecord collisionInfo;
	NewtonCollisionGetInfo (collision, &collisionInfo);
	if (collisionInfo.m_collisionType == SERIALIZE_ID_USERMESH) {
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

#else
	collision = NewtonCreateUserMeshCollision (nWorld, &m_minBox[0], &m_maxBox[0], this, MeshCollisionCollideCallback, UserMeshCollisionRayHitCallback, NULL, NULL, NULL);
#endif


	dVector boxP0(0.0f); 
	dVector boxP1(0.0f); 
	// get the position of the aabb of this geometry

	dMatrix matrix (GetMatrix());
	NewtonCollisionCalculateAABB (collision, &matrix[0][0], &boxP0.m_x, &boxP1.m_x); 
	matrix.m_posit = (boxP0 + boxP1).Scale (-0.5f);
	matrix.m_posit.m_w = 1.0f;
	SetMatrix (matrix);


	// create the level rigid body
	m_level = NewtonCreateBody(nWorld, collision);

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (m_level, this);

	// release the collision tree (this way the application does not have to do book keeping of Newton objects
	NewtonReleaseCollision (nWorld, collision);

	// set the global position of this body
	NewtonBodySetMatrix (m_level, &matrix[0][0]); 

	// set the destructor for this object
//	NewtonBodySetDestructorCallback (m_level, destructor);


	// get the position of the aabb of this geometry
	NewtonCollisionCalculateAABB (collision, &matrix[0][0], &boxP0.m_x, &boxP1.m_x); 

	// add some extra padding the world size
	boxP0.m_x -=  10.0f;
	boxP0.m_y -=  10.0f;
	boxP0.m_z -=  10.0f;
	boxP1.m_x +=  10.0f;
	boxP1.m_y += 400.0f;
	boxP1.m_z +=  10.0f;

	// set the world size
	NewtonSetWorldSize (nWorld, &boxP0.m_x, &boxP1.m_x); 
*/
}

UserHeightFieldCollision::~UserHeightFieldCollision()
{
}


NewtonBody* UserHeightFieldCollision::GetRigidBody() const
{
	return m_level;
}


// calculate the bounding box surrounding a line segment
void UserHeightFieldCollision::CalculateMinExtend2d (const dVector& p0, const dVector& p1, dVector& boxP0, dVector& boxP1)
{
	dFloat x0;
	dFloat x1;
	dFloat z0;
	dFloat z1;

	x0 = dMin (p0.m_x, p1.m_x) - 1.0e-3f;
	z0 = dMin (p0.m_z, p1.m_z) - 1.0e-3f;

	x1 = dMax (p0.m_x, p1.m_x) + 1.0e-3f;
	z1 = dMax (p0.m_z, p1.m_z) + 1.0e-3f;

	x0 = CELL_SIZE * dFloor (x0 * (1.0f / CELL_SIZE));
	z0 = CELL_SIZE * dFloor (z0 * (1.0f / CELL_SIZE));
	x1 = CELL_SIZE * dFloor (x1 * (1.0f / CELL_SIZE)) + CELL_SIZE;
	z1 = CELL_SIZE * dFloor (z1 * (1.0f / CELL_SIZE)) + CELL_SIZE;

	boxP0.m_x = dMax (x0, m_minBox.m_x);
	boxP0.m_z = dMax (z0, m_minBox.m_z);

	boxP1.m_x = dMin (x1, m_maxBox.m_x);
	boxP1.m_z = dMin (z1, m_maxBox.m_z);
}


void UserHeightFieldCollision::CalculateMinExtend3d (const dVector& p0, const dVector& p1, dVector& boxP0, dVector& boxP1)
{
	dFloat x0;
	dFloat x1;
	dFloat y0;
	dFloat y1;
	dFloat z0;
	dFloat z1;

	x0 = dMin (p0.m_x, p1.m_x) - 1.0e-3f;
	y0 = dMin (p0.m_y, p1.m_y) - 1.0e-3f;
	z0 = dMin (p0.m_z, p1.m_z) - 1.0e-3f;

	x1 = dMax (p0.m_x, p1.m_x) + 1.0e-3f;
	y1 = dMax (p0.m_y, p1.m_y) + 1.0e-3f;
	z1 = dMax (p0.m_z, p1.m_z) + 1.0e-3f;

	x0 = CELL_SIZE * dFloor (x0 * (1.0f / CELL_SIZE));
	y0 = CELL_SIZE * dFloor (y0 * (1.0f / CELL_SIZE));
	z0 = CELL_SIZE * dFloor (z0 * (1.0f / CELL_SIZE));

	x1 = CELL_SIZE * dFloor (x1 * (1.0f / CELL_SIZE)) + CELL_SIZE;
	y1 = CELL_SIZE * dFloor (y1 * (1.0f / CELL_SIZE)) + CELL_SIZE;
	z1 = CELL_SIZE * dFloor (z1 * (1.0f / CELL_SIZE)) + CELL_SIZE;

	boxP0.m_x = dMax (x0, m_minBox.m_x);
	boxP0.m_y = dMax (y0, m_minBox.m_y);
	boxP0.m_z = dMax (z0, m_minBox.m_z);

	boxP1.m_x = dMin (x1, m_maxBox.m_x);
	boxP1.m_y = dMin (y1, m_maxBox.m_y);
	boxP1.m_z = dMin (z1, m_maxBox.m_z);
}


// clip a line segment against a box  
bool UserHeightFieldCollision::ClipRay2d (dVector& p0, dVector& p1, const dVector& boxP0, const dVector& boxP1) 
{
	dFloat t;
	dFloat tmp0;
	dFloat tmp1;

	// clip against positive x axis
	tmp0 = boxP1.m_x - p0.m_x;
	if (tmp0 > 0.0f) {
		tmp1 = boxP1.m_x - p1.m_x;
		if (tmp1 < 0.0f) {
			t = tmp0 / (p1.m_x - p0.m_x);
			p1.m_x = boxP1.m_x;
			p1.m_y = p0.m_y + (p1.m_y - p0.m_y) * t;
			p1.m_z = p0.m_z + (p1.m_z - p0.m_z) * t;
		}
	} else {
		tmp1 = boxP1.m_x - p1.m_x;
		if (tmp1 > 0.0f) {
			t = tmp0 / (p1.m_x - p0.m_x);
			p0.m_x = boxP1.m_x;
			p0.m_y = p0.m_y + (p1.m_y - p0.m_y) * t;
			p0.m_z = p0.m_z + (p1.m_z - p0.m_z) * t;
		} else {
			return false;
		}
	}

	// clip against negative x axis
	tmp0 = boxP0.m_x - p0.m_x;
	if (tmp0 < 0.0f) {
		tmp1 = boxP0.m_x - p1.m_x;
		if (tmp1 > 0.0f) {
			t = tmp0 / (p1.m_x - p0.m_x);
			p1.m_x = boxP0.m_x;
			p1.m_y = p0.m_y + (p1.m_y - p0.m_y) * t;
			p1.m_z = p0.m_z + (p1.m_z - p0.m_z) * t;
		}
	} else {
		tmp1 = boxP0.m_x - p1.m_x;
		if (tmp1 < 0.0f) {
			t = tmp0 / (p1.m_x - p0.m_x);
			p0.m_x = boxP0.m_x;
			p0.m_y = p0.m_y + (p1.m_y - p0.m_y) * t;
			p0.m_z = p0.m_z + (p1.m_z - p0.m_z) * t;
		} else {
			return false;
		}
	}
	
	// clip against positive z axis
	tmp0 = boxP1.m_z - p0.m_z;
	if (tmp0 > 0.0f) {
		tmp1 = boxP1.m_z - p1.m_z;
		if (tmp1 < 0.0f) {
			t = tmp0 / (p1.m_z - p0.m_z);
			p1.m_z = boxP1.m_z;
			p1.m_x = p0.m_x + (p1.m_x - p0.m_x) * t;
			p1.m_y = p0.m_y + (p1.m_y - p0.m_y) * t;
		}
	} else {
		tmp1 = boxP1.m_z - p1.m_z;
		if (tmp1 > 0.0f) {
			t = tmp0 / (p1.m_z - p0.m_z);
			p0.m_z = boxP1.m_z;
			p0.m_x = p0.m_x + (p1.m_x - p0.m_x) * t;
			p0.m_y = p0.m_y + (p1.m_y - p0.m_y) * t;
		} else {
			return false;
		}
	}

	// clip against negative z axis
	tmp0 = boxP0.m_z - p0.m_z;
	if (tmp0 < 0.0f) {
		tmp1 = boxP0.m_z - p1.m_z;
		if (tmp1 > 0.0f) {
			t = tmp0 / (p1.m_z - p0.m_z);
			p1.m_z = boxP0.m_z;
			p1.m_x = p0.m_x + (p1.m_x - p0.m_x) * t;
			p1.m_y = p0.m_y + (p1.m_y - p0.m_y) * t;
		}
	} else {
		tmp1 = boxP0.m_z - p1.m_z;
		if (tmp1 < 0.0f) {
			t = tmp0 / (p1.m_z - p0.m_z);
			p0.m_z = boxP0.m_z;
			p0.m_x = p0.m_x + (p1.m_x - p0.m_x) * t;
			p0.m_y = p0.m_y + (p1.m_y - p0.m_y) * t;
		} else {
			return false;
		}
	}

	// the line or part of the line segment is contained by the cell
	return true;
}


// calculate the intersection of a ray and a triangle
dFloat UserHeightFieldCollision::RayCastTriangle (const dVector& p0, const dVector& dp, const dVector& origin, const dVector& e1, const dVector& e2)
{
	dFloat t;
	dFloat b0;
	dFloat b1;
	dFloat b00;
	dFloat b11;
	dFloat a00;
	dFloat a10;
	dFloat a11;
	dFloat det;
	dFloat dot;
	dFloat tol;

	// clip line again first triangle
	dVector normal (e2 * e1);

	dot = normal % dp;
	if (dot <= 1.0e-6f) {
		t = ((origin - p0) % normal) / dot;
		if (t > 0.0f) {
			if (t < 1.0f) {
				dVector q (p0 + dp.Scale (t));
				a00 = e1 % e1;
				a11 = e2 % e2;
				a10 = e1 % e2;
				det = a00 * a11 - a10 * a10;
				// det must be positive and different than zero
				//dAssert (det > 0.0f);
				
				dVector q0p0 (q - origin);
				b0 = q0p0 % e1;
				b1 = q0p0 % e2;

				tol = -det * 1.0e-3f;
				b00 = b0 * a11 - b1 * a10;
				if (b00 >= tol) {
					b11 = b1 * a00 - b0 * a10;
					if (b11 >= tol) {
						if ((b00 + b11) <= (det * 1.001f)) {
							// found a hit return this value
							return t;
						}
					}
				}
			}
		}
	}

	// if it come here the there no intersection
	return 1.2f;
}

// calculate the intersection point of a line segment and the two triangles making a the cell of a heih pam terrain
dFloat UserHeightFieldCollision::RayCastCell (dInt32 xIndex0, dInt32 zIndex0, const dVector& p0, const dVector& dp, dVector& normalOut)
{
	dFloat t;


	// if debug mode on save the line cell in wire frame
	dVector p00 ((xIndex0 + 0) * CELL_SIZE, m_heightField[zIndex0 + 0][xIndex0 + 0], (zIndex0 + 0) * CELL_SIZE);
	dVector p10 ((xIndex0 + 1) * CELL_SIZE, m_heightField[zIndex0 + 0][xIndex0 + 1], (zIndex0 + 0) * CELL_SIZE);
	dVector p11 ((xIndex0 + 1) * CELL_SIZE, m_heightField[zIndex0 + 1][xIndex0 + 1], (zIndex0 + 1) * CELL_SIZE);
	dVector p01 ((xIndex0 + 0) * CELL_SIZE, m_heightField[zIndex0 + 1][xIndex0 + 0], (zIndex0 + 1) * CELL_SIZE);

	if (DebugDisplayOn()) {
		DebugDrawLine (p00, p10);
		DebugDrawLine (p10, p11);
		DebugDrawLine (p11, p01);
		DebugDrawLine (p01, p00);
	}
	

	// get the 3d point at the corner of the cell

	// clip line again first triangle
	dVector e0 (p10 - p00);
	dVector e1 (p11 - p00);

	t = RayCastTriangle (p0, dp, p00, e0, e1);
	if (t < 1.0f) {
		normalOut = e1 * e0;
		return t;
	}

	// clip line against second triangle
	dVector e2 (p01 - p00);
	t = RayCastTriangle (p0, dp, p00, e1, e2);
	if (t < 1.0f) {
		normalOut = e2 * e1;
	}
	return t;
}



// determine if a ray segment intersection the height map cell
dFloat  UserHeightFieldCollision::UserMeshCollisionRayHitCallback (NewtonUserMeshCollisionRayHitDesc* rayDesc)
{
	dInt32 xInc;
	dInt32 zInc;
	dInt32 xIndex0;
	dInt32 zIndex0;
	dFloat t;
	dFloat tx;
	dFloat tz;
	dFloat txAcc;
	dFloat tzAcc;
	dFloat val;
	dFloat ix0;
	dFloat iz0;
	dFloat scale;
	dFloat invScale;
	
	dFloat stepX;
	dFloat stepZ;
	dVector normalOut(0.0f);
	UserHeightFieldCollision *map;



	// set the debug line counter to zero
//	debugRayCast = 0;


	// the user data is the pointer to the collision geometry
	map = (UserHeightFieldCollision*) rayDesc->m_userData;

	dVector q0 (rayDesc->m_p0[0], rayDesc->m_p0[1], rayDesc->m_p0[2]);
	dVector q1 (rayDesc->m_p1[0], rayDesc->m_p1[1], rayDesc->m_p1[2]);

//if (q1.m_y < 0.0f) return q0.m_y / (q0.m_y - q1.m_y);

	dVector boxP0(0.0f);
	dVector boxP1(0.0f);

	// calculate the ray bounding box
	map->CalculateMinExtend2d (q0, q1, boxP0, boxP1);

	dVector dq (q1 - q0);
//	dVector padding (dq.Scale (CELL_SIZE * 10.0f / (dSqrt (dq % dq) + 1.0e-6f)));
	
	// make sure the line segment crosses the original segment box
//	dVector p0 (q0 - padding);
//	dVector p1 (q1 + padding);
	dVector p0 (q0);
	dVector p1 (q1);

	// clip the line against the bounding box
	if (map->ClipRay2d (p0, p1, boxP0, boxP1)) {
		dVector dp (p1 - p0);

		scale = CELL_SIZE;
		invScale = (1.0f / CELL_SIZE);
		ix0 = dFloor (p0.m_x * invScale);
		iz0 = dFloor (p0.m_z * invScale);

		// implement a 3ddda line algorithm 
		if (dp.m_x > 0.0f) {
			xInc = 1;
			val = 1.0f / dp.m_x;
			stepX = scale * val;
			tx = (scale * (ix0 + 1.0f) - p0.m_x) * val;
		} else if (dp.m_x < 0.0f) {
			xInc = -1;
			val = -1.0f / dp.m_x;
			stepX = scale * val;
			tx = -(scale * ix0 - p0.m_x) * val;
		} else {
			xInc = 0;
			stepX = 0.0f;
			tx = 1.0e10f;
		}

		if (dp.m_z > 0.0f) {
			zInc = 1;
			val = 1.0f / dp.m_z;
			stepZ = scale * val;
			tz = (scale * (iz0 + 1.0f) - p0.m_z) * val;
		} else if (dp.m_z < 0.0f) {
			zInc = -1;
			val = -1.0f / dp.m_z;
			stepZ = scale * val;
			tz = -(scale * iz0 - p0.m_z) * val;
		} else {
			zInc = 0;
			stepZ = 0.0f;
			tz = 1.0e10f;
		}

		txAcc = tx;
		tzAcc = tz;
		xIndex0 = dInt32 (ix0);
		zIndex0 = dInt32 (iz0);

		// for each cell touched by the line
		do {
			t = map->RayCastCell (xIndex0, zIndex0, q0, dq, normalOut);
			if (t < 1.0f) {
				// bail out at the first intersection and copy the data into the descriptor
				normalOut = normalOut.Scale (1.0f / dSqrt (normalOut % normalOut));
				rayDesc->m_normalOut[0] = normalOut.m_x;
				rayDesc->m_normalOut[1] = normalOut.m_y;
				rayDesc->m_normalOut[2] = normalOut.m_z;
				rayDesc->m_userIdOut = (xIndex0 << 16) + zIndex0;
				return t;
			}

			if (txAcc < tzAcc) {
				xIndex0 += xInc;
				tx = txAcc;
				txAcc += stepX;
			} else {
				zIndex0 += zInc;
				tz = tzAcc;
				tzAcc += stepZ;
			}
		} while ((tx <= 1.0f) || (tz <= 1.0f));
	}

	// if no cell was hit, return a large value
	return 1.2f;
}



void  UserHeightFieldCollision::MeshCollisionCollideCallback (NewtonUserMeshCollisionCollideDesc* collideDesc)
{
	dInt32 x;
	dInt32 z;
	dInt32 x0;
	dInt32 x1;
	dInt32 z0;
	dInt32 z1;
	dInt32 step;
	dInt32 index;
	dInt32 faceCount;
	dInt32 vertexIndex;
	dInt32 threadNumber;
	UserHeightFieldCollision *map;
	dVector boxP0(0.0f);
	dVector boxP1(0.0f);

	// the user data is the pointer to the collision geometry
	map = (UserHeightFieldCollision*) collideDesc->m_userData;

	dVector p0 (collideDesc->m_boxP0[0], collideDesc->m_boxP0[1], collideDesc->m_boxP0[2]);
	dVector p1 (collideDesc->m_boxP1[0], collideDesc->m_boxP1[1], collideDesc->m_boxP1[2]);

	map->CalculateMinExtend3d (p0, p1, boxP0, boxP1);

	x0 = (dInt32) (boxP0.m_x * (1.0f / CELL_SIZE));
	x1 = (dInt32) (boxP1.m_x * (1.0f / CELL_SIZE));

	z0 = (dInt32) (boxP0.m_z * (1.0f / CELL_SIZE));
	z1 = (dInt32) (boxP1.m_z * (1.0f / CELL_SIZE));

	threadNumber = collideDesc->m_threadNumber;

	// initialize the callback data structure
	collideDesc->m_vertexStrideInBytes = sizeof (dVector);
	collideDesc->m_userAttribute = &map->m_attribute[threadNumber][0];
	collideDesc->m_faceIndexCount = &map->m_faceIndices[threadNumber][0];
	collideDesc->m_faceVertexIndex = &map->m_indexArray[threadNumber][0];
	collideDesc->m_vertex = &map->m_collisionVertex[threadNumber][0][0];
  
	// scan the vertices's intersected by the box extend
	vertexIndex = 0;
	for (z = z0; z <= z1; z ++) {
		for (x = x0; x <= x1; x ++) {
			map->m_collisionVertex[threadNumber][vertexIndex] = dVector(CELL_SIZE * x, map->m_heightField[z][x], CELL_SIZE * z);
			vertexIndex ++;
			//dAssert (vertexIndex < MAX_COLLIDING_FACES * 2);
		}
	}


	// build a vertex list index list mesh from the vertices's intersected by the extend
	index = 0;
	faceCount = 0;
	vertexIndex = 0;
	step = x1 - x0 + 1;
	for (z = z0; z < z1; z ++) {
		for (x = x0; x < x1; x ++) {
			
			map->m_attribute[threadNumber][faceCount] = (x << 16) + z;
			map->m_faceIndices[threadNumber][faceCount] = 3;

			map->m_indexArray[threadNumber][index + 0] = vertexIndex;
			map->m_indexArray[threadNumber][index + 1] = (vertexIndex + step + 1);
			map->m_indexArray[threadNumber][index + 2] = (vertexIndex + 1);


			index += 3;
			faceCount ++;
	
			map->m_attribute[threadNumber][faceCount] = (x << 16) + z;
			map->m_faceIndices[threadNumber][faceCount] = 3;

			map->m_indexArray[threadNumber][index + 0] = vertexIndex;
			map->m_indexArray[threadNumber][index + 1] = (vertexIndex + step);
			map->m_indexArray[threadNumber][index + 2] = (vertexIndex + step + 1);

			index += 3;
			faceCount ++;
			vertexIndex ++;

			//dAssert (faceCount < MAX_COLLIDING_FACES);
			
		}
		vertexIndex ++;
	}

	collideDesc->m_faceCount = faceCount;

	if (DebugDisplayOn()) {
		dMatrix matrix;
		NewtonWorld* world;

		world = NewtonBodyGetWorld (collideDesc->m_polySoupBody);
		NewtonBodyGetMatrix (collideDesc->m_polySoupBody, &matrix[0][0]);

		// critical section lock
		NewtonWorldCriticalSectionLock (world);
		for (dInt32 i = 0; i < faceCount; i ++) {
			dInt32 j0;
			dInt32 j1;
			dInt32 j2;
			dVector points[3];
			j0 = map->m_indexArray[threadNumber][i * 3 + 0];
			j1 = map->m_indexArray[threadNumber][i * 3 + 1];
			j2 = map->m_indexArray[threadNumber][i * 3 + 2];

			points[0] = matrix.TransformVector(dVector (map->m_collisionVertex[threadNumber][j0]));
			points[1] = matrix.TransformVector(dVector (map->m_collisionVertex[threadNumber][j1]));
			points[2] = matrix.TransformVector(dVector (map->m_collisionVertex[threadNumber][j2]));
			DebugDrawPolygon (3,  points);
		}

		// unlock the critical section
		NewtonWorldCriticalSectionUnlock (world);


	}
}
#endif
