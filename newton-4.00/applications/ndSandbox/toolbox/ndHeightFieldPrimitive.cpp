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
#include "ndPhysicsWorld.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndHeightFieldPrimitive.h"

//#define D_TERRAIN_WIDTH			1024
//#define D_TERRAIN_HEIGHT			1024
#define D_TERRAIN_WIDTH				512
#define D_TERRAIN_HEIGHT			512
//#define D_TERRAIN_WIDTH			256
//#define D_TERRAIN_HEIGHT			256

#define D_TERRAIN_NOISE_OCTAVES		8
#define D_TERRAIN_NOISE_PERSISTANCE	0.5f
#define D_TERRAIN_NOISE_GRID_SCALE  (1.0f / 500.0f)
//#define D_TERRAIN_NOISE_GRID_SCALE  1.0f / (ndFloat32 (D_TERRAIN_WIDTH) / 5)

//#define D_TERRAIN_GRID_SIZE		4.0f
#define D_TERRAIN_GRID_SIZE			2.0f
#define D_TERRAIN_TILE_SIZE			128
#define D_TERRAIN_ELEVATION_SCALE	32.0f

#if 0
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
	ndArray<ndReal>& heightMap = shape->GetElevationMap();
	ndAssert(heightMap.GetCount() == heightfield.GetCount());
	for (ndInt32 i = 0; i < heightfield.GetCount(); ++i)
	{
		ndFloat32 high = heightfield[i].m_y;
		heightMap[i] = ndReal(high);
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
#endif

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
			heightfield[z * D_TERRAIN_WIDTH + x] = ndVector((ndFloat32)x * cellSize, noiseVal, (ndFloat32)z * cellSize, ndFloat32(0.0f));
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

class ndHeightfieldMesh : public ndRenderSceneNode
{
	public:
	ndHeightfieldMesh(ndRender* const render, const ndShapeHeightfield* const shape, const ndSharedPtr<ndRenderTexture>& texture, const ndMatrix& location)
		:ndRenderSceneNode(location)
	{
		ndMatrix uvMapping(ndGetIdentityMatrix());
		uvMapping[0][0] = 1.0f / 20.0f;
		uvMapping[1][1] = 1.0f / 20.0f;
		uvMapping[2][2] = 1.0f / 20.0f;

		for (ndInt32 z = 0; z < D_TERRAIN_HEIGHT - 1; z += D_TERRAIN_TILE_SIZE)
		{
			for (ndInt32 x = 0; x < D_TERRAIN_WIDTH - 1; x += D_TERRAIN_TILE_SIZE)
			{
				ndSharedPtr<ndShapeInstance> tileShape(BuildTile(shape, x, z));
				const ndShapeHeightfield* const heightfield = tileShape->GetShape()->GetAsShapeHeightfield();

				ndMatrix tileMatrix(ndGetIdentityMatrix());
				tileMatrix.m_posit += heightfield->GetLocation(0, 0);
				tileMatrix.m_posit.m_y = ndFloat32(0.0f);
				tileMatrix.m_posit.m_x += ndFloat32(x) * heightfield->GetWithScale();
				tileMatrix.m_posit.m_z += ndFloat32(z) * heightfield->GetHeightScale();

				ndSharedPtr<ndRenderSceneNode> tileNode(new ndRenderSceneNode(tileMatrix));
				AddChild(tileNode);

				ndRenderPrimitiveMesh::ndDescriptor descriptor(render);
				descriptor.m_collision = tileShape;
				descriptor.m_stretchMaping = false;
				descriptor.m_uvMatrix = uvMapping;
				descriptor.m_mapping = ndRenderPrimitiveMesh::m_box;
				ndRenderPrimitiveMeshMaterial& material = descriptor.AddMaterial(texture);
				material.m_castShadows = false;
				ndSharedPtr<ndRenderPrimitive> mesh(ndRenderPrimitiveMesh::CreateMeshPrimitive(descriptor));
				tileNode->SetPrimitive(mesh);
			}
		}
	}

	private:
	virtual void Render(const ndRender* const owner, ndFloat32 timeStep, const ndMatrix& parentMatrix, ndRenderPassMode renderMode) const
	{
		// make a tiled rendered node.
		// the terrain is a array of tile subtable for colling,
		// but in this demo we are just rendering the map brute force
		ndRenderSceneNode::Render(owner, timeStep, parentMatrix, renderMode);
	}

	ndSharedPtr<ndShapeInstance> BuildTile(const ndShapeHeightfield* const shape, ndInt32 x0, ndInt32 z0)
	{
		const ndInt32 xMax = ((x0 + D_TERRAIN_TILE_SIZE) >= D_TERRAIN_WIDTH) ? D_TERRAIN_TILE_SIZE : D_TERRAIN_TILE_SIZE + 1;
		const ndInt32 zMax = ((z0 + D_TERRAIN_TILE_SIZE) >= D_TERRAIN_HEIGHT) ? D_TERRAIN_TILE_SIZE : D_TERRAIN_TILE_SIZE + 1;

		// build a collision subtile
		const ndArray<ndReal>& heightMap = shape->GetElevationMap();
		ndSharedPtr<ndShapeInstance> tileInstance(new ndShapeInstance(new ndShapeHeightfield(xMax, zMax,
				ndShapeHeightfield::m_invertedDiagonals,
				D_TERRAIN_GRID_SIZE, D_TERRAIN_GRID_SIZE)));

		ndArray<ndReal>& tileHeightMap = tileInstance->GetShape()->GetAsShapeHeightfield()->GetElevationMap();
		for (ndInt32 z = 0; z < zMax; z++)
		{
			for (ndInt32 x = 0; x < xMax; x++)
			{
				ndReal h = heightMap[(z0 + z) * D_TERRAIN_WIDTH + x0 + x];
				tileHeightMap[z * xMax + x] = h;
			}
		}
		tileInstance->GetShape()->GetAsShapeHeightfield()->UpdateElevationMapAabb();
		return tileInstance;
	}
};

ndSharedPtr<ndBody> BuildHeightFieldTerrain(ndDemoEntityManager* const scene, const char* const textureName, const ndMatrix& location)
{
	ndArray<ndVector> heightfield(D_TERRAIN_WIDTH * D_TERRAIN_HEIGHT);
	MakeNoiseHeightfield(heightfield);

	// create the height field collision and rigid body
	ndShapeInstance heighfieldInstance(new ndShapeHeightfield(D_TERRAIN_WIDTH, D_TERRAIN_WIDTH,
			ndShapeHeightfield::m_invertedDiagonals, D_TERRAIN_GRID_SIZE, D_TERRAIN_GRID_SIZE));
	
	ndShapeHeightfield* const heighfield = heighfieldInstance.GetShape()->GetAsShapeHeightfield();
	ndArray<ndReal>& heightMap = heighfield->GetElevationMap();
	ndAssert(heightMap.GetCount() == heightfield.GetCount());
	for (ndInt32 i = 0; i < heightfield.GetCount(); ++i)
	{
		ndFloat32 high = heightfield[i].m_y;
		heightMap[i] = ndReal(high);
	}
	heighfield->UpdateElevationMapAabb();

	ndMatrix heighfieldLocation(location);
	heighfieldLocation.m_posit.m_x -= 0.5f * ndFloat32(heighfield->GetWith()) * heighfield->GetWithScale();
	heighfieldLocation.m_posit.m_z -= 0.5f * ndFloat32(heighfield->GetHeight()) * heighfield->GetHeightScale();

	// add tile base sence node
	ndRender* const render = *scene->GetRenderer();
	ndSharedPtr<ndRenderTexture> texture(render->GetTextureCache()->GetTexture(ndGetWorkingFileName(textureName)));
	ndSharedPtr<ndRenderSceneNode> entity(new ndHeightfieldMesh(render, heighfield, texture, heighfieldLocation));
	
	// generate a rigibody and added to the scene and world
	ndPhysicsWorld* const world = scene->GetWorld();
	ndSharedPtr<ndBody> body (new ndBodyDynamic());
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetMatrix(heighfieldLocation);
	body->GetAsBodyDynamic()->SetCollisionShape(heighfieldInstance);
	
	world->AddBody(body);
	scene->AddEntity(entity);
	return body;
}