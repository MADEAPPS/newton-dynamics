/* Copyright (c) <2003-2019> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoMesh.h"
#include "ndDemoEntityManager.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndDebugDisplay.h"
#include "ndHeightFieldPrimitive.h"

#if 0
class HeightfieldMap
{
	public:
	#ifdef USE_TEST_ALL_FACE_USER_RAYCAST_CALLBACK
		static dFloat32 AllRayHitCallback (const NewtonBody* const body, const NewtonCollision* const treeCollision, dFloat32 intersection, dFloat32* const normal, int faceId, void* const usedData)
		{
			return intersection;
		}
	#endif


	static NewtonBody* CreateHeightFieldTerrain (ndDemoEntityManager* const scene, int sizeInPowerOfTwos, dFloat32 cellSize, dFloat32 elevationScale, dFloat32 roughness, dFloat32 maxElevation, dFloat32 minElevation)
	{
		dAssert(0);
		return nullptr;
/*
		int size = (1 << sizeInPowerOfTwos) + 1 ;
		dFloat32* const elevation = new dFloat32 [size * size];
		//MakeFractalTerrain (elevation, sizeInPowerOfTwos, elevationScale, roughness, maxElevation, minElevation);
		MakeFractalTerrain (elevation, sizeInPowerOfTwos, elevationScale, roughness, maxElevation, minElevation);

		for (dInt32 i = 0; i < 4; i ++) {
			ApplySmoothFilter (elevation, size);
		}

		//SetBaseHeight (elevation, size);
		// apply simple calderas
		//		MakeCalderas (elevation, size, maxElevation * 0.7f, maxElevation * 0.1f);

		//	// create the visual mesh
		ndDemoMesh* const mesh = new ndDemoMesh ("terrain", scene->GetShaderCache(), elevation, size, cellSize, 1.0f/4.0f, TILE_SIZE);

		ndDemoEntity* const entity = new ndDemoEntity(dGetIdentityMatrix(), nullptr);
		scene->Append (entity);
		entity->SetMesh(mesh, dGetIdentityMatrix());
		mesh->Release();

		// create the height field collision and rigid body

		// create the attribute map
		int width = size;
		int height = size;
		char* const attibutes = new char [size * size];
		memset (attibutes, 0, width * height * sizeof (char));
		NewtonCollision* collision = NewtonCreateHeightFieldCollision (scene->GetWorld(), width, height, 1, 0, elevation, attibutes, 1.0f, cellSize, cellSize, 0);

#ifdef USE_STATIC_MESHES_DEBUG_COLLISION
		NewtonStaticCollisionSetDebugCallback (collision, ShowMeshCollidingFaces);
#endif

		NewtonCollisionInfoRecord collisionInfo;
		// keep the compiler happy
		memset (&collisionInfo, 0, sizeof (NewtonCollisionInfoRecord));
		NewtonCollisionGetInfo (collision, &collisionInfo);

		width = collisionInfo.m_heightField.m_width;
		height = collisionInfo.m_heightField.m_height;
		//elevations = collisionInfo.m_heightField.m_elevation;

		dVector boxP0; 
		dVector boxP1; 
		// get the position of the aabb of this geometry
		dMatrix matrix (entity->GetCurrentMatrix());
		NewtonCollisionCalculateAABB (collision, &matrix[0][0], &boxP0.m_x, &boxP1.m_x); 
		matrix.m_posit = (boxP0 + boxP1).Scale (-0.5f);
		matrix.m_posit.m_w = 1.0f;
		//SetMatrix (matrix);
		entity->ResetMatrix (*scene, matrix);

		// create the terrainBody rigid body
		NewtonBody* const terrainBody = NewtonCreateDynamicBody(scene->GetWorld(), collision, &matrix[0][0]);

		// release the collision tree (this way the application does not have to do book keeping of Newton objects
		NewtonDestroyCollision (collision);

		// in newton 300 collision are instance, you need to ready it after you create a body, if you want to male call on the instance
		collision = NewtonBodyGetCollision(terrainBody);
#if 0
		// uncomment this to test horizontal displacement
		unsigned short* const horizontalDisplacemnet = new unsigned short[size * size];
		for (dInt32 i = 0; i < size * size; i++) {
			horizontalDisplacemnet[i] = dRand();
		}
		NewtonHeightFieldSetHorizontalDisplacement(collision, horizontalDisplacemnet, 0.02f);
		delete horizontalDisplacemnet;
#endif

		// save the pointer to the graphic object with the body.
		NewtonBodySetUserData (terrainBody, entity);

		// set the global position of this body
		//	NewtonBodySetMatrix (m_terrainBody, &matrix[0][0]); 

		// set the destructor for this object
		//NewtonBodySetDestructorCallback (terrainBody, Destructor);

		// get the position of the aabb of this geometry
		//NewtonCollisionCalculateAABB (collision, &matrix[0][0], &boxP0.m_x, &boxP1.m_x); 

#ifdef USE_TEST_ALL_FACE_USER_RAYCAST_CALLBACK
		// set a ray cast callback for all face ray cast 
		NewtonTreeCollisionSetUserRayCastCallback (collision, AllRayHitCallback);
		
		dVector p0 (0,  100, 0, 0);
		dVector p1 (0, -100, 0, 0);
		dVector normal;
		dLong id;
		dFloat32 parameter;
		parameter = NewtonCollisionRayCast (collision, &p0[0], &p1[0], &normal[0], &id);
#endif

		delete[] attibutes;
		delete[] elevation;
		return terrainBody;
*/
	}
};

#endif

#define D_TERRAIN_WIDTH				1024
#define D_TERRAIN_HEIGHT			1024

#define D_TERRAIN_NOISE_OCTAVES		8
#define D_TERRAIN_NOISE_AMPLITUD	1.0f
#define D_TERRAIN_NOISE_PERSISTANCE	0.5f

#define D_TERRAIN_NOISE_FREQUENCY	256.0f
#define D_TERRAIN_NOISE_LACUNARITY	0.6f

#define D_TERRAIN_GRID_SIZE			4.0f
#define D_TERRAIN_ELEVATION_SCALE	40.0f

#define D_TERRAIN_TILE_SIZE			128

class ndHeightfieldMesh : public ndDemoMesh
{
	public: 
	ndHeightfieldMesh(const dArray<dVector>& gridMap, const ndShaderPrograms& shaderCache)
		:ndDemoMesh ("heightfield")
	{
		dArray<ndMeshPointUV> points;
		dArray<dInt32> indexList;

		m_shader = shaderCache.m_diffuseEffect;

		BuildTilesArray(indexList);
		BuildVertexAndNormals(indexList, gridMap, points);
		OptimizeForRender(points, indexList);
	}

	private:
	void BuildTilesArray(dArray<dInt32>& indexList)
	{
		for (dInt32 z = 0; z < D_TERRAIN_HEIGHT - 1; z += D_TERRAIN_TILE_SIZE)
		{
			for (dInt32 x = 0; x < D_TERRAIN_WIDTH - 1; x += D_TERRAIN_TILE_SIZE)
			{
				BuildTile(indexList, x, z);
			}
		}
	}

	void BuildTile(dArray<dInt32>& indexList, dInt32 x0, dInt32 z0)
	{
		dInt32 start = indexList.GetCount();
		const dInt32 zMax = ((z0 + D_TERRAIN_TILE_SIZE) >= D_TERRAIN_HEIGHT) ? D_TERRAIN_HEIGHT - 1 : z0 + D_TERRAIN_TILE_SIZE;
		const dInt32 xMax = ((x0 + D_TERRAIN_TILE_SIZE) >= D_TERRAIN_WIDTH) ? D_TERRAIN_WIDTH - 1 : x0 + D_TERRAIN_TILE_SIZE;

		for (dInt32 z = z0; z < zMax; z ++)
		{
			for (dInt32 x = x0; x < xMax; x ++)
			{
				indexList.PushBack((z + 0) * D_TERRAIN_WIDTH + x + 0);
				indexList.PushBack((z + 1) * D_TERRAIN_WIDTH + x + 1);
				indexList.PushBack((z + 0) * D_TERRAIN_WIDTH + x + 1);

				indexList.PushBack((z + 1) * D_TERRAIN_WIDTH + x + 1);
				indexList.PushBack((z + 0) * D_TERRAIN_WIDTH + x + 0);
				indexList.PushBack((z + 1) * D_TERRAIN_WIDTH + x + 0);
			}
		}

		ndDemoSubMesh* const segment = AddSubMesh();
		//segment->m_material.m_textureHandle = (GLuint)material;
		segment->m_material.m_textureHandle = (GLuint)1;

		segment->SetOpacity(1.0f);
		segment->m_segmentStart = start;
		segment->m_indexCount = indexList.GetCount() - start;
	}

	void BuildVertexAndNormals(const dArray<dInt32>& indexList, const dArray<dVector>& gridMap, dArray<ndMeshPointUV>& points)
	{
		points.SetCount(gridMap.GetCount());
		memset(&points[0], 0, gridMap.GetCount() * sizeof(ndMeshPointUV));

		for (dInt32 i = 0; i < indexList.GetCount(); i += 3)
		{
			const dInt32 i0 = indexList[i + 0];
			const dInt32 i1 = indexList[i + 1];
			const dInt32 i2 = indexList[i + 2];
			const dVector& p0 = gridMap[i0];
			const dVector& p1 = gridMap[i1];
			const dVector& p2 = gridMap[i2];

			dVector e10(p1 - p0);
			dVector e20(p2 - p0);
			dVector normal(e10.CrossProduct(e20));
			dAssert(normal.m_w == dFloat32(0.0f));
			normal = normal.Normalize();

			points[i0].m_normal.m_x += normal.m_x;
			points[i0].m_normal.m_y += normal.m_y;
			points[i0].m_normal.m_z += normal.m_z;

			points[i1].m_normal.m_x += normal.m_x;
			points[i1].m_normal.m_y += normal.m_y;
			points[i1].m_normal.m_z += normal.m_z;

			points[i2].m_normal.m_x += normal.m_x;
			points[i2].m_normal.m_y += normal.m_y;
			points[i2].m_normal.m_z += normal.m_z;
		}

		for (dInt32 i = 0; i < points.GetCount(); i++)
		{
			dVector normal(points[i].m_normal.m_x, points[i].m_normal.m_y, points[i].m_normal.m_z, dFloat32(0.0f));
			normal = normal.Normalize();
			points[i].m_posit = ndMeshVector(gridMap[i].m_x, gridMap[i].m_y, gridMap[i].m_z);
			points[i].m_normal = ndMeshVector(normal.m_x, normal.m_y, normal.m_z);
			points[i].m_uv.m_u = dFloat32(0.0f);
			points[i].m_uv.m_v = dFloat32(0.0f);
		}
	}
};

static void MakeGridPoints(dArray<dVector>& gridPoints)
{
	gridPoints.SetCount(D_TERRAIN_WIDTH * D_TERRAIN_HEIGHT);

	dInt32 octaves = D_TERRAIN_NOISE_OCTAVES;
	dFloat32 amplitude = D_TERRAIN_NOISE_AMPLITUD;
	dFloat32 persistance = D_TERRAIN_NOISE_PERSISTANCE;
	dFloat32 frequency = D_TERRAIN_NOISE_FREQUENCY;
	dFloat32 lacunarity = D_TERRAIN_NOISE_LACUNARITY;

	dFloat32 cellSize = D_TERRAIN_GRID_SIZE;
	

	dFloat32 minHeight = dFloat32(1.0e10f);
	dFloat32 maxHight = dFloat32(-1.0e10f);
	for (dInt32 z = 0; z < D_TERRAIN_HEIGHT; z++)
	{
		for (dInt32 x = 0; x < D_TERRAIN_WIDTH; x++)
		{
			dFloat32 noiseVal = BrownianMotion(octaves, dFloat32(x), dFloat32(z), amplitude, persistance, frequency, lacunarity);
			gridPoints[z * D_TERRAIN_WIDTH + x] = dVector(x * cellSize, noiseVal, z * cellSize, dFloat32 (0.0f));
			minHeight = dMin(minHeight, noiseVal);
			maxHight = dMax(maxHight, noiseVal);
		}
	}

	dFloat32 highScale = D_TERRAIN_ELEVATION_SCALE;
	dFloat32 scale = dFloat32(2.0f) / (maxHight - minHeight);
	for (dInt32 i = 0; i < gridPoints.GetCapacity(); i++)
	{
		dFloat32 y = gridPoints[i].m_y;
		y = scale * (y - minHeight) - dFloat32(1.0f);
		gridPoints[i].m_y *= highScale;
	}
}

ndBodyKinematic* BuildHeightFieldTerrain(ndDemoEntityManager* const scene)
{
	dArray<dVector> gridMap;
	MakeGridPoints(gridMap);
	
	// create the visual mesh
	ndDemoMesh* const mesh = new ndHeightfieldMesh(gridMap, scene->GetShaderCache());

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit.m_z = -200.0f;
	matrix.m_posit.m_x = -200.0f;
	matrix.m_posit.m_y = -30.0f;

	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	scene->AddEntity(entity);
	entity->SetMesh(mesh, dGetIdentityMatrix());
	mesh->Release();
/*
	// create the height field collision and rigid body

	// create the attribute map
	int width = size;
	int height = size;
	char* const attibutes = new char [size * size];
	memset (attibutes, 0, width * height * sizeof (char));
	NewtonCollision* collision = NewtonCreateHeightFieldCollision (scene->GetWorld(), width, height, 1, 0, elevation, attibutes, 1.0f, cellSize, cellSize, 0);

	#ifdef USE_STATIC_MESHES_DEBUG_COLLISION
	NewtonStaticCollisionSetDebugCallback (collision, ShowMeshCollidingFaces);
	#endif

	NewtonCollisionInfoRecord collisionInfo;
	// keep the compiler happy
	memset (&collisionInfo, 0, sizeof (NewtonCollisionInfoRecord));
	NewtonCollisionGetInfo (collision, &collisionInfo);

	width = collisionInfo.m_heightField.m_width;
	height = collisionInfo.m_heightField.m_height;
	//elevations = collisionInfo.m_heightField.m_elevation;

	dVector boxP0;
	dVector boxP1;
	// get the position of the aabb of this geometry
	dMatrix matrix (entity->GetCurrentMatrix());
	NewtonCollisionCalculateAABB (collision, &matrix[0][0], &boxP0.m_x, &boxP1.m_x);
	matrix.m_posit = (boxP0 + boxP1).Scale (-0.5f);
	matrix.m_posit.m_w = 1.0f;
	//SetMatrix (matrix);
	entity->ResetMatrix (*scene, matrix);

	// create the terrainBody rigid body
	NewtonBody* const terrainBody = NewtonCreateDynamicBody(scene->GetWorld(), collision, &matrix[0][0]);

	// release the collision tree (this way the application does not have to do book keeping of Newton objects
	NewtonDestroyCollision (collision);

	// in newton 300 collision are instance, you need to ready it after you create a body, if you want to male call on the instance
	collision = NewtonBodyGetCollision(terrainBody);
	#if 0
	// uncomment this to test horizontal displacement
	unsigned short* const horizontalDisplacemnet = new unsigned short[size * size];
	for (dInt32 i = 0; i < size * size; i++) {
	horizontalDisplacemnet[i] = dRand();
	}
	NewtonHeightFieldSetHorizontalDisplacement(collision, horizontalDisplacemnet, 0.02f);
	delete horizontalDisplacemnet;
	#endif

	// save the pointer to the graphic object with the body.
	NewtonBodySetUserData (terrainBody, entity);

	// set the global position of this body
	//	NewtonBodySetMatrix (m_terrainBody, &matrix[0][0]);

	// set the destructor for this object
	//NewtonBodySetDestructorCallback (terrainBody, Destructor);

	// get the position of the aabb of this geometry
	//NewtonCollisionCalculateAABB (collision, &matrix[0][0], &boxP0.m_x, &boxP1.m_x);

	#ifdef USE_TEST_ALL_FACE_USER_RAYCAST_CALLBACK
	// set a ray cast callback for all face ray cast
	NewtonTreeCollisionSetUserRayCastCallback (collision, AllRayHitCallback);

	dVector p0 (0,  100, 0, 0);
	dVector p1 (0, -100, 0, 0);
	dVector normal;
	dLong id;
	dFloat32 parameter;
	parameter = NewtonCollisionRayCast (collision, &p0[0], &p1[0], &normal[0], &id);
	#endif

	delete[] attibutes;
	delete[] elevation;
	return terrainBody;
	*/

	return nullptr;
}