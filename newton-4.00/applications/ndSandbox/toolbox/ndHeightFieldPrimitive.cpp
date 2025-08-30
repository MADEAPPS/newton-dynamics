/* Copyright (c) <2003-2022> <Newton Game Dynamics>
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
#include "ndPngToOpenGl.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndHeightFieldPrimitive.h"


#define D_TERRAIN_WIDTH				1024
#define D_TERRAIN_HEIGHT			1024
#define D_TERRAIN_NOISE_OCTAVES		8
#define D_TERRAIN_NOISE_PERSISTANCE	0.5f
#define D_TERRAIN_NOISE_GRID_SCALE  (1.0f / 500.0f)
//#define D_TERRAIN_NOISE_GRID_SCALE  1.0f / (ndFloat32 (D_TERRAIN_WIDTH) / 5)

//#define D_TERRAIN_GRID_SIZE		4.0f
#define D_TERRAIN_GRID_SIZE			2.0f
#define D_TERRAIN_TILE_SIZE			128
#define D_TERRAIN_ELEVATION_SCALE	32.0f
//#define D_TERRAIN_ELEVATION_SCALE	0.5f

class ndHeightfieldMesh : public ndDemoMesh
{
	public: 
	ndHeightfieldMesh(const ndArray<ndVector>& heightfield, const ndShaderCache& shaderCache)
		:ndDemoMesh ("heightfield")
	{
		ndArray<glPositionNormalUV> points(heightfield.GetCount());
		ndArray<ndInt32> indexList(6 * D_TERRAIN_WIDTH * D_TERRAIN_WIDTH + 1024);

		m_shader = shaderCache.m_diffuseEffect;
		m_shaderShadow = shaderCache.m_diffuseShadowEffect;

		BuildTilesArray(indexList, "texture1.png");
		BuildVertexAndNormals(indexList, heightfield, points);
		OptimizeForRender(&points[0], ndInt32(points.GetCount()), &indexList[0], ndInt32(indexList.GetCount()));
	}

	private:
	void BuildTilesArray(ndArray<ndInt32>& indexList, const char* const texName)
	{
		for (ndInt32 z = 0; z < D_TERRAIN_HEIGHT - 1; z += D_TERRAIN_TILE_SIZE)
		{
			for (ndInt32 x = 0; x < D_TERRAIN_WIDTH - 1; x += D_TERRAIN_TILE_SIZE)
			{
				BuildTile(indexList, x, z, texName);
			}
		}
	}

	void BuildTile(ndArray<ndInt32>& indexList, ndInt32 x0, ndInt32 z0, const char* const texName)
	{
		const ndInt32 start = ndInt32(indexList.GetCount());
		const ndInt32 zMax = ((z0 + D_TERRAIN_TILE_SIZE) >= D_TERRAIN_HEIGHT) ? D_TERRAIN_HEIGHT - 1 : z0 + D_TERRAIN_TILE_SIZE;
		const ndInt32 xMax = ((x0 + D_TERRAIN_TILE_SIZE) >= D_TERRAIN_WIDTH) ? D_TERRAIN_WIDTH - 1 : x0 + D_TERRAIN_TILE_SIZE;
		
		for (ndInt32 z = z0; z < zMax; z ++)
		{
			for (ndInt32 x = x0; x < xMax; x ++)
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
		segment->m_material.SetTextureName(texName);
		ndInt32 texHandle = ndInt32(LoadTexture("texture1.png"));
		segment->m_material.SetTexture(texHandle);
		ReleaseTexture(GLuint(texHandle));
		
		segment->SetOpacity(1.0f);
		segment->m_segmentStart = start;
		segment->m_indexCount = ndInt32(indexList.GetCount()) - start;
	}

	void BuildVertexAndNormals(const ndArray<ndInt32>& indexList, const ndArray<ndVector>& heightfield, ndArray<glPositionNormalUV>& points)
	{
		points.SetCount(heightfield.GetCount());
		memset(&points[0], 0, heightfield.GetCount() * sizeof(glPositionNormalUV));

		for (ndInt32 i = 0; i < indexList.GetCount(); i += 3)
		{
			const ndInt32 i0 = indexList[i + 0];
			const ndInt32 i1 = indexList[i + 1];
			const ndInt32 i2 = indexList[i + 2];
			const ndVector& p0 = heightfield[i0];
			const ndVector& p1 = heightfield[i1];
			const ndVector& p2 = heightfield[i2];

			ndVector e10(p1 - p0);
			ndVector e20(p2 - p0);
			ndVector normal(e10.CrossProduct(e20));
			ndAssert(normal.m_w == ndFloat32(0.0f));
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

		ndFloat32 uvScale = 1.0f / 32.0f;
		for (ndInt32 i = 0; i < points.GetCount(); ++i)
		{
			ndVector normal(points[i].m_normal.m_x, points[i].m_normal.m_y, points[i].m_normal.m_z, ndFloat32(0.0f));
			normal = normal.Normalize();
			points[i].m_posit = glVector3(GLfloat(heightfield[i].m_x), GLfloat(heightfield[i].m_y), GLfloat(heightfield[i].m_z));
			points[i].m_normal = glVector3(GLfloat(normal.m_x), GLfloat(normal.m_y), GLfloat(normal.m_z));
			points[i].m_uv.m_u = GLfloat(points[i].m_posit.m_x * uvScale);
			points[i].m_uv.m_v = GLfloat(points[i].m_posit.m_z * uvScale);
		}
	}
};

static void MakeNoiseHeightfield(ndArray<ndVector>& heightfield)
{
	heightfield.SetCount(D_TERRAIN_WIDTH * D_TERRAIN_HEIGHT);
	
	const ndInt32 octaves = D_TERRAIN_NOISE_OCTAVES;
	const ndFloat32 cellSize = D_TERRAIN_GRID_SIZE;
	const ndFloat32 persistance = D_TERRAIN_NOISE_PERSISTANCE;
	const ndFloat32 noiseGridScale = D_TERRAIN_NOISE_GRID_SCALE;
	
	ndFloat32 minHeight = ndFloat32(1.0e10f);
	ndFloat32 maxHight = ndFloat32(-1.0e10f);
	for (ndInt32 z = 0; z < D_TERRAIN_HEIGHT; z++)
	{
		for (ndInt32 x = 0; x < D_TERRAIN_WIDTH; x++)
		{
			ndFloat32 noiseVal = BrownianMotion(octaves, persistance, noiseGridScale * ndFloat32(x), noiseGridScale * ndFloat32(z));
			heightfield[z * D_TERRAIN_WIDTH + x] = ndVector((ndFloat32)x * cellSize, noiseVal, (ndFloat32)z * cellSize, ndFloat32 (0.0f));
			minHeight = ndMin(minHeight, noiseVal);
			maxHight = ndMax(maxHight, noiseVal);
		}
	}

	ndFloat32 highScale = D_TERRAIN_ELEVATION_SCALE;
	ndFloat32 scale = ndFloat32(2.0f) / (maxHight - minHeight);
	for (ndInt32 i = 0; i < heightfield.GetCapacity(); ++i)
	{
		ndFloat32 y = heightfield[i].m_y;
		y = scale * (y - minHeight) - ndFloat32(1.0f);
		heightfield[i].m_y *= highScale;
	}
}

void AddHeightfieldSubShape(ndDemoEntityManager* const scene, ndShapeInstance& sceneInstance, const ndSharedPtr<ndDemoEntity>& rootEntity, const ndMatrix& matrix)
{
	ndArray<ndVector> heightfield(D_TERRAIN_WIDTH * D_TERRAIN_HEIGHT);
	MakeNoiseHeightfield(heightfield);

	ndSharedPtr<ndDemoMeshInterface> mesh (new ndHeightfieldMesh(heightfield, scene->GetShaderCache()));
	ndSharedPtr<ndDemoEntity>entity (new ndDemoEntity(matrix));
	rootEntity->AddChild(entity);
	entity->SetMesh(mesh);
	entity->SetShadowMode(false);

	ndShapeInstance heighfieldInstance(
		new ndShapeHeightfield(D_TERRAIN_WIDTH, D_TERRAIN_WIDTH,
		ndShapeHeightfield::m_invertedDiagonals, 
		D_TERRAIN_GRID_SIZE, D_TERRAIN_GRID_SIZE));

	ndShapeHeightfield* const shape = heighfieldInstance.GetShape()->GetAsShapeHeightfield();
	ndArray<ndReal>& hightMap = shape->GetElevationMap();
	ndAssert(hightMap.GetCount() == heightfield.GetCount());
	for (ndInt32 i = 0; i < heightfield.GetCount(); ++i)
	{
		ndFloat32 high = heightfield[i].m_y;
		hightMap[i] = ndReal(high);
	}
	shape->UpdateElevationMapAabb();

	// save the entity  with the sub shape
	ndShapeMaterial material(heighfieldInstance.GetMaterial());
	material.m_data.m_userData = *entity;
	heighfieldInstance.SetMaterial(material);
	heighfieldInstance.SetLocalMatrix(matrix);
	
	ndShapeCompound* const compound = sceneInstance.GetShape()->GetAsShapeCompound();
	compound->AddCollision(&heighfieldInstance);
}

ndSharedPtr<ndBody> BuildHeightFieldTerrain(ndDemoEntityManager* const scene, const ndMatrix& location)
{
	ndArray<ndVector> heightfield(D_TERRAIN_WIDTH * D_TERRAIN_HEIGHT);
	MakeNoiseHeightfield(heightfield);

	// create the visual mesh
	ndSharedPtr<ndDemoMeshInterface> mesh(new ndHeightfieldMesh(heightfield, scene->GetShaderCache()));
	ndSharedPtr<ndDemoEntity> entity (new ndDemoEntity(location));
	entity->SetShadowMode(false);
	entity->SetMesh(mesh);

	// create the height field collision and rigid body
	ndShapeInstance heighfieldInstance(
		new ndShapeHeightfield(D_TERRAIN_WIDTH, D_TERRAIN_WIDTH,
			ndShapeHeightfield::m_invertedDiagonals,
			D_TERRAIN_GRID_SIZE, D_TERRAIN_GRID_SIZE));

	ndShapeHeightfield* const shape = heighfieldInstance.GetShape()->GetAsShapeHeightfield();
	ndArray<ndReal>& hightMap = shape->GetElevationMap();
	ndAssert(hightMap.GetCount() == heightfield.GetCount());
	for (ndInt32 i = 0; i < heightfield.GetCount(); ++i)
	{
		ndFloat32 high = heightfield[i].m_y;
		hightMap[i] = ndReal(high);
	}
	shape->UpdateElevationMapAabb();

	ndPhysicsWorld* const world = scene->GetWorld();
	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(location);
	body->GetAsBodyDynamic()->SetCollisionShape(heighfieldInstance);

	world->AddBody(body);
	scene->AddEntity(entity);
	//return body->GetAsBodyDynamic();
	return body;
}