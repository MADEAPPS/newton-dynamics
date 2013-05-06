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


// LevelPrimitive.cpp: implementation of the LevelPrimitive class.
//
//////////////////////////////////////////////////////////////////////
#include <toolbox_stdafx.h>
#include "OpenGlUtil.h"
#include "../OGLMesh.h"
#include "../PhysicsUtils.h"
#include "DebugDisplay.h"
#include "LevelPrimitive.h"

//#include "PlaneCollision.h"

#ifdef _MSC_VER
#include <mmsystem.h>
#endif


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
#ifdef USE_TEST_SERIALIZATION

static const char* MAGIC_NUMBER = "serialize data";

// for litle indians, 
static void SerializeFile (void* serializeHandle, const void* buffer, int size)
{
	_ASSERTE ((size % 4) == 0);
	fwrite (buffer, size, 1, (FILE*) serializeHandle);
}

static void DeSerializeFile (void* serializeHandle, void* buffer, int size)
{
	_ASSERTE ((size % 4) == 0);
	fread (buffer, size, 1, (FILE*) serializeHandle);
}

/*
// for big indian serialization
static void SerializeFile (void* serializeHandle, const void* buffer, int size)
{
	_ASSERTE ((size % 4) == 0);
	const int* ptr = (int*)buffer;
	for (int i = 0; i < int (size/sizeof (int)); i ++) {
		int data = SWAP_INT32(ptr[i]);
		fwrite (&data, sizeof (int), 1, (FILE*) serializeHandle);
	}
}

static void DeSerializeFile (void* serializeHandle, void* buffer, int size)
{
	_ASSERTE ((size % 4) == 0);
	fread (buffer, size, 1, (FILE*) serializeHandle);
	SWAP_FLOAT32_ARRAY (buffer, size);
}
*/
#endif


#ifdef USE_TEST_ALL_FACE_USER_RAYCAST_CALLBACK
static dFloat AllRayHitCallback (dFloat interseption, dFloat* normal, int faceId, void* userData)
{
	return interseption;
}
#endif




LevelPrimitive::LevelPrimitive(const char* name, NewtonWorld* nWorld, int optimized)
	:OGLModel()
{
/*
FILE* file;
NewtonCollision* collision1;
//file = fopen ("C:/temp/Archive/test-ppc.bin", "rb");
file = fopen ("C:/temp/Archive/test-x86.bin", "rb");
collision1 = NewtonCreateCollisionFromSerialization (nWorld, DeSerializeFile, file);
fclose (file);

NewtonCollisionInfoRecord collisionInfo;
NewtonCollisionGetInfo (collision1, &collisionInfo);
NewtonCollisionGetInfo (collision1, &collisionInfo);
*/


_ASSERTE (0);
/*
	NewtonCollision* collision;
	
	// open the level data
	char fullPathName[2048];
	GetWorkingFileName (name, fullPathName);

	OGLLoaderContext context;
	dMatrix rotMatrix (dYawMatrix (-3.14159265f * 0.5f));
_ASSERTE (0);
//	LoadCollada(fullPathName, context, rotMatrix, 1.0f);


	// measure the time to build a collision tree
#ifdef _MSC_VER
	int timer0 = timeGetTime();
#endif

	// create the collision tree geometry
	collision = NewtonCreateTreeCollision(nWorld, 0);

	// set the application level callback
#ifdef USE_STATIC_MESHES_DEBUG_COLLISION
 	NewtonStaticCollisionSetDebugCallback (collision, DebugCallback);
#endif


	// prepare to create collision geometry
	NewtonTreeCollisionBeginBuild(collision);
	dMatrix pallete[512];
	dMatrix invRootMatrix (GetMatrix().Inverse());
	UpdateMatrixPalette (invRootMatrix, pallete, sizeof (m_matrixPalete) / sizeof (dMatrix));

	int faceCount = 0;
	// iterate the entire geometry an build the collision
	for (dList<dMeshInstance>::dListNode* node = m_meshList.GetFirst(); node; node = node->GetNext()) { 
		OGLMesh* const mesh = (OGLMesh*)node->GetInfo().m_mesh;
		dFloat* const vertex = mesh->m_vertex;

		const dMatrix& globalMatrix = pallete[node->GetInfo().m_boneID];
		for (dMesh::dListNode* nodes = mesh->GetFirst(); nodes; nodes = nodes->GetNext()) {
			dSubMesh& segment = nodes->GetInfo();
			for (int i = 0; i < segment.m_indexCount; i += 3) {
				int index;	
				int matID;
				dVector face[3];

				index = segment.m_indexes[i + 0] * 3;
				face[0] = dVector (vertex[index + 0], vertex[index + 1], vertex[index + 2]);

				index = segment.m_indexes[i + 1] * 3;
				face[1] = dVector (vertex[index + 0], vertex[index + 1], vertex[index + 2]);

				index = segment.m_indexes[i + 2] * 3;
				face[2] = dVector (vertex[index + 0], vertex[index + 1], vertex[index + 2]);

				globalMatrix.TransformTriplex (face, sizeof (dVector), face, sizeof (dVector), 3);

				// stress test the collision builder
//				matID = matID == 2 ? 1 : 2 ;
				matID = 0;
				faceCount ++;
				NewtonTreeCollisionAddFace(collision, 3, &face[0].m_x, sizeof (dVector), matID);
			}
		}
	}

	NewtonTreeCollisionEndBuild(collision, optimized);


//NewtonMesh* xxx = CreateCollisionTreeDoubleFaces (nWorld, collision);
	
	// measure the time to build a collision tree
#ifdef _MSC_VER
	timer0 = timeGetTime() - timer0;
	timer0 *= 1;
#endif

#if 0
	{
		NewtonCollisionInfoRecord collisionInfo;
		NewtonCollisionGetInfo (collision, &collisionInfo);
		if (collisionInfo.m_collisionType == SERIALIZE_ID_TREE) {
			int count;
			dVector p0(-1000, -1000, -1000);
			dVector p1(1000, 1000, 1000);
			const dFloat* vertexArray;
			int vertexStrideInBytes;
			int vertexCount;
			int indexList[256];
			int attributeList[256/3];
			count = NewtonTreeCollisionGetVertexListIndexListInAABB (collision, &p0[0], &p1[0], 
																     &vertexArray, &vertexCount, &vertexStrideInBytes, 
				                                                     indexList, sizeof (indexList)/sizeof (indexList[0]), 
				                                                     attributeList); 

			// check AABB accuracy

			vertexStrideInBytes /= sizeof (dFloat);
			dVector minP1 ( 1.0e10f,  1.0e10f,  1.0e10f, 0.0f);
			dVector maxP1 (-1.0e10f, -1.0e10f, -1.0e10f, 0.0f);
			for (int i = 0; i < vertexCount; i ++) {
				for (int j = 0; j < 3; j ++ ) {
					minP1[j] = vertexArray[i * vertexStrideInBytes + j] < minP1[j] ? vertexArray[i * vertexStrideInBytes + j] : minP1[j];
					maxP1[j] = vertexArray[i * vertexStrideInBytes + j] > maxP1[j] ? vertexArray[i * vertexStrideInBytes + j] : maxP1[j];
				}
			}

			dVector minP;
			dVector maxP;
			dMatrix matrix (GetIdentityMatrix());
			CalculateAABB (collision, matrix, minP, maxP);

			minP1 -= minP;
			maxP1 -= maxP;
		}
	}
#endif

#if 0
dVector dir (1, 0, 0, 0);
dVector vertex;
NewtonCollisionSupportVertex (collision, &dir[0], &vertex[0]);

while(GetChild())
{
dModel* xxx;
xxx = GetChild();
xxx->Detach();
delete xxx;
}

dModel* xxx;
dGeometry* xxxx;
xxx = new dModel (this);
xxxx = dGeometry::MakeMeshFromCollision(nWorld, collision);
strcpy (xxxx->m_name, "xxxx");
SetGeometry(xxxx);
xxxx->Release();


int xxx;
dVector p0(-100, -100, -100);
dVector p1(100, 100, 100);
const dFloat* vertexArray;
int vertexStrideInBytes;
int vertexCount;
int indexList[256];
int attributeList[256/3];
xxx = NewtonTreeCollisionGetVertexListIndexListInAABB (collision, &p0[0], &p1[0], 
													   &vertexArray, &vertexCount, &vertexStrideInBytes, 
													   indexList, sizeof (indexList)/sizeof (indexList[0]), 
													   attributeList); 
#endif

#ifdef USE_TEST_SERIALIZATION
	{
		FILE* file;
		// save the collision file
		char fullPathName[256];
		GetWorkingFileName ("collisiontest.bin", fullPathName);

		file = fopen (fullPathName, "wb");
		SerializeFile (file, MAGIC_NUMBER, int (strlen (MAGIC_NUMBER) + 1));
		NewtonCollisionSerialize (nWorld, collision, SerializeFile, file);
		fclose (file);

		// load the collision file
		NewtonReleaseCollision (nWorld, collision);

		file = fopen (fullPathName, "rb");
		char magicNumber[256]; 
		DeSerializeFile (file, magicNumber, int (strlen (MAGIC_NUMBER) + 1));
//		if (!strcmp (magicNumber, MAGIC_NUMBER)) {
			collision = NewtonCreateCollisionFromSerialization (nWorld, DeSerializeFile, file);
//		}
		fclose (file);

		NewtonCollisionInfoRecord collisionInfo;
		NewtonCollisionGetInfo (collision, &collisionInfo);
		NewtonCollisionGetInfo (collision, &collisionInfo);
	}
#endif


	// create the level rigid body
	m_level = NewtonCreateBody(nWorld, collision);

	// release the collision tree (this way the application does not have to do book keeping of Newton objects
	NewtonReleaseCollision (nWorld, collision);

	// set the global position of this body
	dMatrix matrix (GetMatrix());
	NewtonBodySetMatrix (m_level, &matrix[0][0]); 

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (m_level, this);


	// set a destructor for this rigid body
//	NewtonBodySetDestructorCallback (m_level, Destructor);

	
	dVector boxP0; 
	dVector boxP1; 
	// get the position of the aabb of this geometry
	NewtonCollisionCalculateAABB (collision, &matrix[0][0], &boxP0.m_x, &boxP1.m_x); 

	// add some extra padding the world size
	boxP0.m_x -=  50.0f;
	boxP0.m_y -=  50.0f;
	boxP0.m_z -=  50.0f;
	boxP1.m_x +=  50.0f;
	boxP1.m_y += 500.0f;
	boxP1.m_z +=  50.0f;

	// set the world size
	NewtonSetWorldSize (nWorld, &boxP0.m_x, &boxP1.m_x); 

#if 0
	int count;
	NewtonCollision* collisionB;
	collisionB = NewtonCreateBox (nWorld, 3.5f, 3.5f, 3.5f, NULL);

	dMatrix matrixA (GetIdentityMatrix());
	dMatrix matrixB (GetIdentityMatrix());
	matrixA.m_posit.m_y += 10.0f;
	matrixB.m_posit.m_z += 0.0f;

	dVector contacts[20];
	dVector normals[20];
	dFloat depths[20];
	count = NewtonCollisionCollide (nWorld, 10, 
									collision, &matrixA[0][0], 
									collisionB, &matrixB[0][0],
									&contacts[0][0], &normals[0][0], &depths[0], 0);
#endif


#ifdef USE_TEST_ALL_FACE_USER_RAYCAST_CALLBACK
	// set a ray cast callback for all face ray cast 
	NewtonTreeCollisionSetUserRayCastCallback (collision, AllRayHitCallback);
	dVector p0 (0,  100, 0, 0);
	dVector p1 (0, -100, 0, 0);
	dVector normal;
	int id;
	dFloat parameter;
	parameter = NewtonCollisionRayCast (collision, &p0[0], &p1[0], &normal[0], &id);
#endif
*/
}

LevelPrimitive::~LevelPrimitive()
{
}

/*
// rigid body destructor
void LevelPrimitive::Destructor (const NewtonBody* body)
{
	LevelPrimitive* level;

	// get the graphic object form the rigid body
	level = (LevelPrimitive*) NewtonBodyGetUserData (body);

	// destroy the graphic object
	delete level;
}
*/


void LevelPrimitive::DebugCallback (const NewtonBody* bodyWithTreeCollision, const NewtonBody* body, int faceID, int vertexCount, const dFloat* vertex, int vertexstrideInBytes)
{
	// the application can use this function for debugging purpose by writing the 
	// face to a global variable then display which face of the mesh are been used for collision.
	// ,,,,,,,,,,,
	// ..........

	// the application can use this function also to modify the collision geometry by changing the face ID
	// this could be uses full to make surface change for example from dry to wet, or breaking glass
	// for this the application should used the functions:
	// int id = NewtonTreeCollisionGetFaceAtribute (treeCollision, indexArray); 
	// NewtonTreeCollisionSetFaceAtribute (treeCollision, indexArray, new id);

	if (DebugDisplayOn()) {
		dVector face[64];
		int stride = vertexstrideInBytes / sizeof (dFloat);
		for (int j = 0; j < vertexCount; j ++) {
			face [j] = dVector (vertex[j * stride + 0], vertex[j * stride + 1] , vertex[j * stride + 2]);
		}
		DebugDrawPolygon (vertexCount, face);
	}
}
