/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndDebugDisplay.h"
#include "ndPhysicsWorld.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoEntityManager.h"
#include "ndHeightFieldPrimitive.h"

//#define D_TERRAIN_WIDTH			1024 * 4
//#define D_TERRAIN_HEIGHT			1024 * 4
#define D_TERRAIN_WIDTH			1024
#define D_TERRAIN_HEIGHT		1024


#define D_TERRAIN_NOISE_OCTAVES		8
#define D_TERRAIN_NOISE_PERSISTANCE	0.5f
//#define D_TERRAIN_NOISE_GRID_SCALE  1.0f / (dFloat32 (D_TERRAIN_WIDTH) / 5)
#define D_TERRAIN_NOISE_GRID_SCALE  (1.0f / 500.0f)

#define D_TERRAIN_GRID_SIZE			4.0f
#define D_TERRAIN_ELEVATION_SCALE	150.0f

#define D_TERRAIN_TILE_SIZE			128

class ndHeightfieldMesh : public ndDemoMesh
{
	public: 
	ndHeightfieldMesh(const dArray<dVector>& heightfield, const ndShaderPrograms& shaderCache)
		:ndDemoMesh ("heightfield")
	{
		dArray<ndMeshPointUV> points(heightfield.GetCount());
		dArray<dInt32> indexList(6 * D_TERRAIN_WIDTH * D_TERRAIN_WIDTH + 1024);

		m_shader = shaderCache.m_diffuseEffect;

		BuildTilesArray(indexList);
		BuildVertexAndNormals(indexList, heightfield, points);
		//OptimizeForRender(points, indexList);
		OptimizeForRender(&points[0], points.GetCount(), &indexList[0], indexList.GetCount());
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

	void BuildVertexAndNormals(const dArray<dInt32>& indexList, const dArray<dVector>& heightfield, dArray<ndMeshPointUV>& points)
	{
		points.SetCount(heightfield.GetCount());
		memset(&points[0], 0, heightfield.GetCount() * sizeof(ndMeshPointUV));

		for (dInt32 i = 0; i < indexList.GetCount(); i += 3)
		{
			const dInt32 i0 = indexList[i + 0];
			const dInt32 i1 = indexList[i + 1];
			const dInt32 i2 = indexList[i + 2];
			const dVector& p0 = heightfield[i0];
			const dVector& p1 = heightfield[i1];
			const dVector& p2 = heightfield[i2];

			dVector e10(p1 - p0);
			dVector e20(p2 - p0);
			dVector normal(e10.CrossProduct(e20));
			dAssert(normal.m_w == dFloat32(0.0f));
			normal = normal.Normalize();

			points[i0].m_normal.m_x += GLfloat(normal.m_x);
			points[i0].m_normal.m_y += GLfloat(normal.m_y);
			points[i0].m_normal.m_z += GLfloat(normal.m_z);

			points[i1].m_normal.m_x += GLfloat(normal.m_x);
			points[i1].m_normal.m_y += GLfloat(normal.m_y);
			points[i1].m_normal.m_z += GLfloat(normal.m_z);

			points[i2].m_normal.m_x += GLfloat(normal.m_x);
			points[i2].m_normal.m_y += GLfloat(normal.m_y);
			points[i2].m_normal.m_z += GLfloat(normal.m_z);
		}

		for (dInt32 i = 0; i < points.GetCount(); i++)
		{
			dVector normal(points[i].m_normal.m_x, points[i].m_normal.m_y, points[i].m_normal.m_z, dFloat32(0.0f));
			normal = normal.Normalize();
			points[i].m_posit = ndMeshVector(GLfloat(heightfield[i].m_x), GLfloat(heightfield[i].m_y), GLfloat(heightfield[i].m_z));
			points[i].m_normal = ndMeshVector(GLfloat(normal.m_x), GLfloat(normal.m_y), GLfloat(normal.m_z));
			points[i].m_uv.m_u = dFloat32(0.0f);
			points[i].m_uv.m_v = dFloat32(0.0f);
		}
	}
};

static void MakeNoiseHeightfield(dArray<dVector>& heightfield)
{
	heightfield.SetCount(D_TERRAIN_WIDTH * D_TERRAIN_HEIGHT);
	
	const dInt32 octaves = D_TERRAIN_NOISE_OCTAVES;
	const dFloat32 cellSize = D_TERRAIN_GRID_SIZE;
	const dFloat32 persistance = D_TERRAIN_NOISE_PERSISTANCE;
	const dFloat32 noiseGridScale = D_TERRAIN_NOISE_GRID_SCALE;
	
	dFloat32 minHeight = dFloat32(1.0e10f);
	dFloat32 maxHight = dFloat32(-1.0e10f);
	for (dInt32 z = 0; z < D_TERRAIN_HEIGHT; z++)
	{
		for (dInt32 x = 0; x < D_TERRAIN_WIDTH; x++)
		{
			dFloat32 noiseVal = BrownianMotion(octaves, persistance, noiseGridScale * dFloat32(x), noiseGridScale * dFloat32(z));
			heightfield[z * D_TERRAIN_WIDTH + x] = dVector(x * cellSize, noiseVal, z * cellSize, dFloat32 (0.0f));
			minHeight = dMin(minHeight, noiseVal);
			maxHight = dMax(maxHight, noiseVal);
		}
	}

	dFloat32 highScale = D_TERRAIN_ELEVATION_SCALE;
	dFloat32 scale = dFloat32(2.0f) / (maxHight - minHeight);
	for (dInt32 i = 0; i < heightfield.GetCapacity(); i++)
	{
		dFloat32 y = heightfield[i].m_y;
		y = scale * (y - minHeight) - dFloat32(1.0f);
		heightfield[i].m_y *= highScale;
	}
}

ndBodyKinematic* BuildHeightFieldTerrain(ndDemoEntityManager* const scene)
{
	dArray<dVector> heightfield(D_TERRAIN_WIDTH * D_TERRAIN_HEIGHT);
	MakeNoiseHeightfield(heightfield);
	
	// create the visual mesh
	ndDemoMesh* const mesh = new ndHeightfieldMesh(heightfield, scene->GetShaderCache());

	dMatrix matrix(dGetIdentityMatrix());
	matrix.m_posit.m_z = -200.0f;
	matrix.m_posit.m_x = -200.0f;
	matrix.m_posit.m_y = -30.0f;

	ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
	entity->SetMesh(mesh, dGetIdentityMatrix());

	// create the height field collision and rigid body
	ndShapeInstance heighfieldInstance(new ndShapeHeightfield(D_TERRAIN_WIDTH, D_TERRAIN_WIDTH,
		ndShapeHeightfield::m_invertedDiagonals,
		1.0f / 100.0f, D_TERRAIN_GRID_SIZE, D_TERRAIN_GRID_SIZE));

	//#ifdef USE_STATIC_MESHES_DEBUG_COLLISION
	//NewtonStaticCollisionSetDebugCallback (collision, ShowMeshCollidingFaces);
	//#endif

	ndShapeHeightfield* const shape = heighfieldInstance.GetShape()->GetAsShapeHeightfield();
	dArray<dInt16>& hightMap = shape->GetElevationMap();
	dAssert(hightMap.GetCount() == heightfield.GetCount());
	//ndShapeInfo hightInfo(heighfieldInstance.GetShapeInfo());
	//dInt16* const highMap = hightInfo.m_heightfield.m_elevation;
	for (int i = 0; i < heightfield.GetCount(); i++)
	{
		dFloat32 high = heightfield[i].m_y * 100.0f;
		dAssert(high < dFloat32(1 << 15));
		dAssert(high > dFloat32(-1 << 15));
		hightMap[i] = dInt16(high);
	}

	shape->UpdateElevationMapAabb();

	//width = collisionInfo.m_heightField.m_width;
	//height = collisionInfo.m_heightField.m_height;
	//elevations = collisionInfo.m_heightField.m_elevation;

	dVector boxP0;
	dVector boxP1;
	// get the position of the aabb of this geometry
	dMatrix entMatrix (entity->GetCurrentMatrix());

	//NewtonCollisionCalculateAABB (collision, &matrix[0][0], &boxP0.m_x, &boxP1.m_x);
	heighfieldInstance.CalculateAABB(entMatrix, boxP0, boxP1);

	entMatrix.m_posit = (boxP0 + boxP1).Scale (-0.5f);
	entMatrix.m_posit.m_w = 1.0f;

	//SetMatrix (matrix);
	//entity->ResetMatrix (entMatrix);

	ndPhysicsWorld* const world = scene->GetWorld();
	ndBodyDynamic* const body = new ndBodyDynamic();
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(matrix);
	body->SetCollisionShape(heighfieldInstance);

	world->AddBody(body);
	scene->AddEntity(entity);
	mesh->Release();
	return body;
}