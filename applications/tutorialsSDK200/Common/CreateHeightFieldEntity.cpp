/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "StdAfx.h"
#include "Entity.h"
#include "OpenGlUtil.h"
#include "SceneManager.h"
#include "CreateHeightFieldEntity.h"

#define TEXTURE_SCALE			(1.0f / 16.0f)

void CreateHeightFieldMesh (NewtonCollision* collision, Entity* ent)
{
	NewtonCollisionInfoRecord collisionInfo;

	// keep the compiler happy
	memset (&collisionInfo, 0, sizeof (NewtonCollisionInfoRecord));
	NewtonCollisionGetInfo (collision, &collisionInfo);

	// get the info from the collision mesh and create a visual mesh
	int width = collisionInfo.m_heightField.m_width;
	int height = collisionInfo.m_heightField.m_height;
	dFloat* const elevations = collisionInfo.m_heightField.m_elevation;
	dFloat vScale = collisionInfo.m_heightField.m_verticalScale;
	dFloat hScale = collisionInfo.m_heightField.m_horizonalScale;

	// allocate space to store vertex data
	ent->m_vertexCount = width * height;
	ent->m_vertex = (dFloat*) malloc (3 * width * height * sizeof (dFloat));
	ent->m_normal = (dFloat*) malloc (3 * width * height * sizeof (dFloat));
	ent->m_uv = (dFloat*) malloc (2 * width * height * sizeof (dFloat));



	// scan the height field and convert every cell into two triangles
	for (int z = 0; z < height; z ++) {
		int z0 = ((z - 1) < 0) ? 0 : z - 1;
		int z1 = ((z + 1) > (height - 1)) ? height - 1 : z + 1 ;
		for (int x = 0; x < width; x ++) {
			int x0 = ((x - 1) < 0) ? 0 : x - 1;
			int x1 = ((x + 1) > (width - 1)) ? width - 1 : x + 1 ;

			dVector p0 (hScale * x0, elevations[z * width + x1] * vScale, hScale * z);
			dVector p1 (hScale * x1, elevations[z * width + x0] * vScale, hScale * z);
			dVector x10 (p1 - p0);

			dVector q0 (hScale * x, elevations[z0 * width + x] * vScale, hScale * z0);
			dVector q1 (hScale * x, elevations[z1 * width + x] * vScale, hScale * z1);
			dVector z10 (q1 - q0);

			dVector normal (z10 * x10);
			normal = normal.Scale (dSqrt (1.0f / (normal % normal)));
			dVector point (hScale * x, elevations[z * width + x] * vScale, hScale * z);

			ent->m_vertex[(z * width + x) * 3 + 0] = point.m_x;
			ent->m_vertex[(z * width + x) * 3 + 1] = point.m_y;
			ent->m_vertex[(z * width + x) * 3 + 2] = point.m_z;

			ent->m_normal[(z * width + x) * 3 + 0] = normal.m_x;
			ent->m_normal[(z * width + x) * 3 + 1] = normal.m_y;
			ent->m_normal[(z * width + x) * 3 + 2] = normal.m_z;

			ent->m_uv[(z * width + x) * 2 + 0] = x * TEXTURE_SCALE;
			ent->m_uv[(z * width + x) * 2 + 1] = z * TEXTURE_SCALE;
		}
	}

	
	// since the bitmap sample is 256 x 256, i fix into a single 16 bit index vertex array with
	ent->m_subMeshCount = 1;
	ent->m_subMeshes = (Entity::SubMesh*) malloc (sizeof (Entity::SubMesh));

	// allocate space to the index list
	ent->m_subMeshes[0].m_textureHandle = LoadTexture ("grassAndDirt.tga");
	ent->m_subMeshes[0].m_indexCount = (width - 1) * (height - 1) * 6;
	ent->m_subMeshes[0].m_indexArray = (unsigned short*) malloc (ent->m_subMeshes[0].m_indexCount * sizeof (unsigned short));

	// now following the grid pattern and create and index list
	int index = 0;
	int vertexIndex = 0;
	for (int z = 0; z < height - 1; z ++) {
		vertexIndex = z * width;
		for (int x = 0; x < width - 1; x ++) {

			ent->m_subMeshes[0].m_indexArray[index + 0] = GLushort (vertexIndex);
			ent->m_subMeshes[0].m_indexArray[index + 1] = GLushort (vertexIndex + width);
			ent->m_subMeshes[0].m_indexArray[index + 2] = GLushort (vertexIndex + 1);
			index += 3;

			ent->m_subMeshes[0].m_indexArray[index + 0] = GLushort (vertexIndex + 1);
			ent->m_subMeshes[0].m_indexArray[index + 1] = GLushort (vertexIndex + width);
			ent->m_subMeshes[0].m_indexArray[index + 2] = GLushort (vertexIndex + width + 1);
			index += 3;
			vertexIndex ++;
		}
	}

	// Optimize the mesh for hardware rendering if possible
	ent->OptimizeMesh();

/*
	dVector boxP0; 
	dVector boxP1; 
	// get the position of the aabb of this geometry
	dMatrix matrix (ent->m_curRotation, ent->m_curPosition);
	NewtonCollisionCalculateAABB (collision, &matrix[0][0], &boxP0.m_x, &boxP1.m_x); 

	// place the origin of the visual mesh at the center of the height field
	matrix.m_posit = (boxP0 + boxP1).Scale (-0.5f);
	matrix.m_posit.m_w = 1.0f;
	ent->m_curPosition = matrix.m_posit;
	ent->m_prevPosition = matrix.m_posit;


	// create the level rigid body
	body = NewtonCreateBody(world, collision);

	// release the collision tree (this way the application does not have to do book keeping of Newton objects
	NewtonReleaseCollision (world, collision);


	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (body, ent);

	// set the global position of this body
	NewtonBodySetMatrix (body, &matrix[0][0]); 


	// set the destructor for this object
//	NewtonBodySetDestructorCallback (body, Destructor);

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
	NewtonSetWorldSize (world, &boxP0.m_x, &boxP1.m_x); 
	return body;
*/
}

