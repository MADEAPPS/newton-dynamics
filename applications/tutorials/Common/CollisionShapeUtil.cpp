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
#include "CollisionShapeUtil.h"




// Utility function to calculate the Bounding Box of a collision shape 
void CalculateBoxdingBox (const NewtonCollision* shape, dVector& boxMin, dVector& boxMax)
{
	// for exact axis find minimum and maximum limits
	for (int i = 0; i < 3; i ++) {
		dVector point;
		dVector dir (0.0f, 0.0f, 0.0f, 0.0f);

		// find the most extreme point along this axis and this is maximum Box size on that direction
		dir[i] = 1.0f;
		NewtonCollisionSupportVertex (shape, &dir[0], &point[0]);
		boxMax[i] = point % dir;

		// find the most extreme point along the opposite axis and this is maximum Box size on that direction
		dir[i] = -1.0f;
		NewtonCollisionSupportVertex (shape, &dir[0], &point[0]);
		boxMin[i] = -(point % dir);
	}
}


NewtonCollision* CreateNewtonBox (NewtonWorld* world, Entity *ent, int shapeId)
{
	dVector minBox;
	dVector maxBox;

	// Get the Bounding Box for this entity
	ent->GetBBox (minBox, maxBox);

	//calculate the box size and dimensions of the physics collision shape 
	dVector size (maxBox - minBox);
	dVector origin ((maxBox + minBox).Scale (0.5f));
	size.m_w = 1.0f;
	origin.m_w = 1.0f;

	// make and offset Matrix for this collision shape.
	dMatrix offset (GetIdentityMatrix());
	offset.m_posit = origin;

	// now create a collision Box for this entity
	return NewtonCreateBox (world, size.m_x, size.m_y, size.m_z, shapeId, &offset[0][0]);
}

NewtonCollision* CreateNewtonCapsule (NewtonWorld* world, Entity *ent, dFloat height, dFloat radius, int shapeId, const dMatrix& orientation)
{
	dVector minBox;
	dVector maxBox;

	// Get the Bounding Box for this entity
	ent->GetBBox (minBox, maxBox);

	dMatrix offset (orientation);
	// Place the shape origin at the geometrical center of the entity
	offset.m_posit = (maxBox + minBox).Scale (0.5f);
	offset.m_posit.m_w = 1.0f;

	// now create a collision Box for this entity
	return NewtonCreateCapsule(world, radius, height, shapeId, &offset[0][0]);
}

NewtonCollision* CreateNewtonCylinder (NewtonWorld* world, Entity *ent, dFloat height, dFloat radius, int shapeId, const dMatrix& orientation)
{
	dVector minBox;
	dVector maxBox;

	// Get the Bounding Box for this entity
	ent->GetBBox (minBox, maxBox);

	dMatrix offset (orientation);
	// Place the shape origin at the geometrical center of the entity
	offset.m_posit = (maxBox + minBox).Scale (0.5f);
	offset.m_posit.m_w = 1.0f;

	// now create a collision Box for this entity
	return NewtonCreateCylinder(world, radius, height, shapeId, &offset[0][0]);
}


#if 1 
// create a convex collision Shape from the vertices of an entity
NewtonCollision* CreateNewtonConvex (NewtonWorld* world, Entity *ent, int shapeId)
{
	// now create a convex hull shape from the vertex geometry 
	return NewtonCreateConvexHull(world, ent->m_vertexCount, ent->m_vertex, 3 * sizeof (dFloat), 0.1f, shapeId, NULL);
}

#else
// create a convex collision Shape from the vertices of an entity
NewtonCollision* CreateNewtonConvex (NewtonWorld* world, Entity *ent, int shapeId)
{
	dVector minBox;
	dVector maxBox;
	NewtonCollision* collision;
	dVector* tmpArray = new dVector [ent->m_vertexCount];

	// Get the Bounding Box for this entity
	ent->GetBBox (minBox, maxBox);

	dVector size (maxBox - minBox);
	dVector origin ((maxBox + minBox).Scale (0.5f));
	size.m_w = 1.0f;
	origin.m_w = 1.0f;

	// Translate all points to the origin point
	for (int i = 0; i < ent->m_vertexCount; i ++) {
		dVector tmp (ent->m_vertex[i * 3 + 0], ent->m_vertex[i * 3 + 1], ent->m_vertex[i * 3 + 2], 0.0f);
		tmpArray[i] = tmp - origin;
	}

	// make and offset Matrix for this collision shape.
	dMatrix offset (GetIdentityMatrix());
	offset.m_posit = origin;

	// now create a convex hull shape from the vertex geometry 
	collision = NewtonCreateConvexHull(world, ent->m_vertexCount, &tmpArray[0][0], sizeof (dVector), 0.1f, shapeId, &offset[0][0]);

	delete tmpArray;

	return collision;
}
#endif


// Create a compound collision from each of the sub meshes of and entity
NewtonCollision* CreateNewtonCompoundFromEntitySubmesh (NewtonWorld* world, Entity* ent, int* shapeIdArray)
{
	_ASSERTE (0);
	return NULL;
/*
	NewtonCollision* collision;
	NewtonCollision* shapedArray[256];

	for (int i = 0; i <  ent->m_subMeshCount; i ++) {
		int indexCount = 0;
		dVector vertexArray[1024];
		for (int j = 0; j < ent->m_subMeshes[i].m_indexCount; j += 3 ) {
			int index;

			index = ent->m_subMeshes[i].m_indexArray[j + 0] * 3;
			vertexArray[indexCount + 0] = dVector (ent->m_vertex[index + 0], ent->m_vertex[index + 1], ent->m_vertex[index + 2]);

			index = ent->m_subMeshes[i].m_indexArray[j + 1] * 3;
			vertexArray[indexCount + 1] = dVector (ent->m_vertex[index + 0], ent->m_vertex[index + 1], ent->m_vertex[index + 2]);

			index = ent->m_subMeshes[i].m_indexArray[j + 2] * 3;
			vertexArray[indexCount + 2] = dVector (ent->m_vertex[index + 0], ent->m_vertex[index + 1], ent->m_vertex[index + 2]);
			indexCount += 3;
		}
		shapedArray[i] = NewtonCreateConvexHull(world, indexCount, &vertexArray[0][0], sizeof (dVector), 0.1f, shapeIdArray[i], NULL);
	}

	collision = NewtonCreateCompoundCollision (world,  ent->m_subMeshCount, shapedArray, 0);

	for (int i = 0; i <  ent->m_subMeshCount; i ++) {
		NewtonReleaseCollision (world, shapedArray[i]);
	}

	return collision;
*/
}


static dFloat UserMeshCollisionCallback (const NewtonBody* const body, const NewtonCollision* const collisionTree, dFloat interception, dFloat* normal, int faceId, void* usedData)
{
	return 1.0f;
}


NewtonCollision* CreateMeshCollision (NewtonWorld* world, Entity* ent, int* shapeIdArray)
{
	// now create and empty collision tree
	NewtonCollision* const collision = NewtonCreateTreeCollision (world, 0);

	// start adding faces to the collision tree 
	NewtonTreeCollisionBeginBuild (collision);
	// step over the collision geometry and add all faces to the collision tree 
	for (int i = 0; i <  ent->m_subMeshCount; i ++) {

		// add each sub mesh as a face id, will will sue this later for a multi material sound effect in and advanced tutorial
		for (int j = 0; j < ent->m_subMeshes[i].m_indexCount; j += 3 ) {
			int index;
			dVector face[3];

			index = ent->m_subMeshes[i].m_indexArray[j + 0] * 3;
			face[0] = dVector (ent->m_vertex[index + 0], ent->m_vertex[index + 1], ent->m_vertex[index + 2]);

			index = ent->m_subMeshes[i].m_indexArray[j + 1] * 3;
			face[1] = dVector (ent->m_vertex[index + 0], ent->m_vertex[index + 1], ent->m_vertex[index + 2]);

			index = ent->m_subMeshes[i].m_indexArray[j + 2] * 3;
			face[2] = dVector (ent->m_vertex[index + 0], ent->m_vertex[index + 1], ent->m_vertex[index + 2]);

			if (shapeIdArray) {
				NewtonTreeCollisionAddFace(collision, 3, &face[0].m_x, sizeof (dVector), shapeIdArray[i]);
			} else {
				NewtonTreeCollisionAddFace(collision, 3, &face[0].m_x, sizeof (dVector), i + 1);
			}
		}
	}

	// end adding faces to the collision tree, also optimize the mesh for best performance
	NewtonTreeCollisionEndBuild (collision, 1);


	// look here, this is how you add the user callback to the collision tree mesh,
	// not to be confused with the filter callback which is called on each collision shape that the ray hit.
	// this function is called on each face of this collision tree that ray hit. 
	NewtonTreeCollisionSetUserRayCastCallback(collision, UserMeshCollisionCallback);

	return collision;
}



static dFloat UserHeightFieldCollisionCallback (const NewtonBody* const body, const NewtonCollision* const heightField, dFloat interception, int row, int col, dFloat* normal, int faceId, void* usedData)
{
	return 1.0f;
}


// Create a height field collision form a elevation file
NewtonCollision* CreateHeightFieldCollision (NewtonWorld* world, char* fileName, int* shapeIdArray)
{
	char fullPathName[2048];

	#define CELL_SIZE				12.0f
	#define ELEVATION_SCALE			256.0f 
	#define TEXTURE_SCALE			(1.0f / 16.0f)
	#define ELEVATION_SCALE_INV		(1.0f / ELEVATION_SCALE) 

	//load from raw data
	GetWorkingFileName (fileName, fullPathName);
	FILE* const file = fopen (fullPathName, "rb");
	_ASSERTE (file);

	int width = 256;
	int height = 256;

	// load the data;
	short* const elevationsShort = (short*) malloc (width * height * sizeof (short));
	dFloat* const elevations = (dFloat*) malloc (width * height * sizeof (dFloat));
	char* const attibutes = (char*) malloc (width * width * sizeof (char));
	fread (elevationsShort, sizeof (unsigned short), width * height, file);
	for (int i = 0; i < width * height; i ++) {
		elevations[i] = dFloat (elevationsShort[i]) * ELEVATION_SCALE_INV;
	}
	
	memset (attibutes, 1, width * height * sizeof (char));
	if (shapeIdArray) {
		for (int i = 0; i < width * height; i ++) {
			attibutes[i] = char(shapeIdArray[0]);
		}
	}

	NewtonCollision* const collision = NewtonCreateHeightFieldCollision (world, width, height, 0, elevations, attibutes, CELL_SIZE, 0);
	free (elevationsShort);
	free (elevations);
	free (attibutes);
	fclose (file);


	// look here, this is how you add the user callback to the collision tree mesh,
	// not to be confused with the filter callback which is called on each collision shape that the ray hit.
	// this function is called on each face of this collision tree that ray hit. 
	NewtonHeightFieldSetUserRayCastCallback(collision, UserHeightFieldCollisionCallback);

	return collision;
}


static void DebugShowGeometryCollision (void* userData, int vertexCount, const dFloat* faceVertec, int id)
{

	int i = vertexCount - 1;
	dVector p0 (faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
	for (i = 0; i < vertexCount; i ++) {
		dVector p1 (faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2]);
		glVertex3f (p0.m_x, p0.m_y, p0.m_z);
		glVertex3f (p1.m_x, p1.m_y, p1.m_z);
		p0 = p1;
	}
}


void ShowCollisionShape (const NewtonCollision* shape, const dMatrix& matrix)
{
	NewtonCollisionForEachPolygonDo (shape, &matrix[0][0], DebugShowGeometryCollision, NULL);
}



