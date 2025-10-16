/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndContact.h"
#include "ndBodyKinematic.h"
#include "ndShapeInstance.h"
#include "ndPolygonMeshDesc.h"
#include "ndShapeStaticProceduralMesh.h"

ndShapeStaticProceduralMesh::ndShapeStaticProceduralMesh(ndFloat32 sizex, ndFloat32 sizey, ndFloat32 sizez)
	:ndShapeStaticMesh(m_staticProceduralMesh)
{
	m_boxOrigin = ndVector::m_zero;
	m_boxSize = ndVector(sizex, sizey, sizez, ndFloat32 (0.0f)) * ndVector::m_half;
}

ndShapeStaticProceduralMesh::~ndShapeStaticProceduralMesh(void)
{
}

ndShapeInfo ndShapeStaticProceduralMesh::GetShapeInfo() const
{
	ndShapeInfo info(ndShapeStaticMesh::GetShapeInfo());
	info.m_procedural.m_noUsed = 0;
	return info;
}

void ndShapeStaticProceduralMesh::GetCollidingFaces(ndPolygonMeshDesc* const data) const
{
	ndPolygonMeshDesc::ndStaticMeshFaceQuery& query = *data->m_staticMeshQuery;
	ndPolygonMeshDesc::ndProceduralStaticMeshFaceQuery& meshPatch = *data->m_proceduralStaticMeshFaceQuery;

	ndArray<ndVector>& vertex = meshPatch.m_vertex;
	ndArray<ndInt32>& faceList = query.m_faceIndexCount;
	ndArray<ndInt32>& indexList = meshPatch.m_indexListList;
	ndArray<ndInt32>& faceMaterialList = meshPatch.m_faceMaterial;
	GetCollidingFaces(data->GetOrigin(), data->GetTarget(), vertex, faceList, faceMaterialList, indexList);

	if (faceList.GetCount() == 0)
	{
		return;
	}

	ndEdgeMap edgeMap;
	//ndInt32 index = 0;
	ndInt32 faceStart = 0;
	ndArray<ndInt32>& indices = query.m_faceVertexIndex;
	ndArray<ndInt32>& faceIndexCount = query.m_faceIndexCount;
	
	for (ndInt32 i = 0; i < faceList.GetCount(); ++i)
	{
		ndInt32 i0 = indexList[faceStart + 0];
		ndInt32 i1 = indexList[faceStart + 1];
		ndVector normal(ndVector::m_zero);
		ndVector edge0(vertex[i1] - vertex[i0]);

		ndFloat32 maxDiagonal2 = edge0.DotProduct(edge0).GetScalar();
		for (ndInt32 j = 2; j < faceList[i]; ++j)
		{
			ndInt32 i2 = indexList[faceStart + j];
			const ndVector edge1(vertex[i2] - vertex[i0]);
			maxDiagonal2 = ndMax(maxDiagonal2, edge1.DotProduct(edge1).GetScalar());
			normal += edge0.CrossProduct(edge1);
			edge0 = edge1;
		}

		ndInt32 normalIndex = ndInt32(vertex.GetCount());
		ndAssert(normal.m_w == ndFloat32(0.0f));
		vertex.PushBack(normal.Normalize());

		ndInt32 quantizedDiagSize = ndInt32(ndFloor(ndSqrt(maxDiagonal2) / D_FACE_CLIP_DIAGONAL_SCALE + ndFloat32(1.0f)));

		const ndPlane plane(normal, -normal.DotProduct(vertex[i0]).GetScalar());

		ndInt32 index = ndInt32(indices.GetCount());
		indices.SetCount(index + faceList[i] * 2 + 3);
		indices[index + faceList[i] + 0] = faceMaterialList[i];
		indices[index + faceList[i] + 1] = normalIndex;
		indices[index + 2 * faceList[i] + 2] = quantizedDiagSize;

		ndInt32 j0 = faceList[i] - 1;
		ndInt32 testIndex = j0 - 1;
		const ndInt32 faceVectexCount = faceList[i];
		faceIndexCount[i] = faceVectexCount;
		for (ndInt32 j1 = 0; j1 < faceVectexCount; ++j1)
		{
			ndInt32 k0 = indexList[faceStart + j0];
			ndInt32 k1 = indexList[faceStart + j1];
			ndInt32 test = indexList[faceStart + testIndex];
			const ndEdge edge(k0, k1, plane, test);
			ndInt32 normalEntryIndex = index + j1 + faceVectexCount + 2;
			edgeMap.Insert(normalEntryIndex, edge);

			indices[index + j1] = indexList[faceStart + j0];
			indices[normalEntryIndex] = normalIndex;

			testIndex = j0;
			j0 = j1;
		}
		faceStart += faceVectexCount;
	}

	ndEdgeMap::Iterator iter(edgeMap);
	for (iter.Begin(); iter; iter++)
	{
		ndEdgeMap::ndNode* const edgeNode = iter.GetNode();
		if (edgeNode->GetInfo() != -1)
		{
			ndEdge edge(iter.GetKey());
			ndSwap(edge.m_i0, edge.m_i1);
			ndEdgeMap::ndNode* const twinNode = edgeMap.Find(edge);
			if (twinNode)
			{
				const ndPlane& plane = twinNode->GetKey().m_plane;
				ndFloat32 dist = plane.Evalue(vertex[edge.m_testIndex]);
				if (dist < -ndFloat32(1.0e-3f))
				{
					ndInt32 i0 = edgeNode->GetInfo();
					ndInt32 i1 = twinNode->GetInfo();
					ndSwap(indices[i0], indices[i1]);
				}
				twinNode->GetInfo() = -1;
			}
		}
		edgeNode->GetInfo() = -1;
	}

	ndInt32 faceCount0 = 0;
	ndInt32 faceIndexCount0 = 0;
	ndInt32 faceIndexCount1 = 0;
	ndInt32 stride = sizeof(ndVector) / sizeof(ndFloat32);
	
	//ndInt32* const address = data->m_meshData.m_globalFaceIndexStart;
	ndArray<ndInt32>& address = query.m_faceIndexStart;
	ndArray<ndFloat32>& hitDistance = query.m_hitDistance;
	if (data->m_doContinueCollisionTest) 
	{
		ndFastRay ray(ndVector::m_zero, data->m_boxDistanceTravelInMeshSpace);
		for (ndInt32 i = 0; i < faceList.GetCount(); ++i)
		{
			const ndInt32 vertexCount = faceIndexCount[i];
			const ndInt32* const indexArray = &indices[faceIndexCount1];
			const ndVector& faceNormal = vertex[indexArray[4]];
			ndFloat32 dist = data->PolygonBoxRayDistance(faceNormal, 3, indexArray, stride, &vertex[0].m_x, ray);
			if (dist < ndFloat32(1.0f)) 
			{
				hitDistance.PushBack(dist);
				address.PushBack(faceIndexCount0);
				ndMemCpy(&indices[faceIndexCount0], indexArray, vertexCount * 2 + 3);
				faceCount0++;
				faceIndexCount0 += vertexCount * 2 + 3;
			}
			faceIndexCount1 += vertexCount * 2 + 3;
		}
	}
	else 
	{
		for (ndInt32 i = 0; i < faceList.GetCount(); ++i)
		{
			const ndInt32 vertexCount = faceIndexCount[i];
			const ndInt32* const indexArray = &indices[faceIndexCount1];
			const ndVector& faceNormal = vertex[indexArray[vertexCount + 1]];
			ndFloat32 dist = data->PolygonBoxDistance(faceNormal, vertexCount, indexArray, stride, &vertex[0].m_x);
			if (dist > ndFloat32(0.0f)) 
			{
				hitDistance.PushBack(dist);
				address.PushBack(faceIndexCount0);
				ndMemCpy(&indices[faceIndexCount0], indexArray, vertexCount * 2 + 3);
				faceCount0++;
				faceIndexCount0 += vertexCount * 2 + 3;
			}
			faceIndexCount1 += vertexCount * 2 + 3;
		}
	}

	// initialize the callback data structure
	faceIndexCount.SetCount(faceCount0);
	data->m_vertex = &vertex[0].m_x;
	data->m_vertexStrideInBytes = sizeof(ndVector);
}

ndUnsigned64 ndShapeStaticProceduralMesh::GetHash(ndUnsigned64 hash) const
{
	return hash + 1;
}