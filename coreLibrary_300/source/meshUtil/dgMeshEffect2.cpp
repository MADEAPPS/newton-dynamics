/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#include "dgPhysicsStdafx.h"
#include "dgBody.h"
#include "dgWorld.h"
#include "dgMeshEffect.h"
#include "dgCollisionConvexHull.h"


// create a convex hull
dgMeshEffect::dgMeshEffect (dgMemoryAllocator* const allocator, const dgFloat64* const vertexCloud, dgInt32 count, dgInt32 strideInByte, dgFloat64 distTol)
	:dgPolyhedra(allocator)
{
	Init();
	if (count >= 4) {
		dgConvexHull3d convexHull (allocator, vertexCloud, strideInByte, count, distTol);
		if (convexHull.GetCount()) {

			dgInt32 vertexCount = convexHull.GetVertexCount();
			dgStack<dgVector> pointsPool (convexHull.GetVertexCount());
			dgVector* const points = &pointsPool[0];
			for (dgInt32 i = 0; i < vertexCount; i ++) {
				points[i] = convexHull.GetVertex(i);
			}
			dgVector uv(dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
			dgVector normal (dgFloat32 (0.0f), dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

			dgInt32 triangleCount = convexHull.GetCount();
			dgStack<dgInt32> faceCountPool (triangleCount);
			dgStack<dgInt32> materialsPool (triangleCount);
			dgStack<dgInt32> vertexIndexListPool (triangleCount * 3);
			dgStack<dgInt32> normalIndexListPool (triangleCount * 3);


			memset (&materialsPool[0], 0, triangleCount * sizeof (dgInt32));
			memset (&normalIndexListPool[0], 0, 3 * triangleCount * sizeof (dgInt32));

			dgInt32 index = 0;
			dgInt32* const faceCount = &faceCountPool[0];
			dgInt32* const vertexIndexList = &vertexIndexListPool[0];
			for (dgConvexHull3d::dgListNode* faceNode = convexHull.GetFirst(); faceNode; faceNode = faceNode->GetNext()) {
				dgConvexHull3DFace& face = faceNode->GetInfo();
				faceCount[index] = 3;
				vertexIndexList[index * 3 + 0] = face.m_index[0]; 
				vertexIndexList[index * 3 + 1] = face.m_index[1]; 
				vertexIndexList[index * 3 + 2] = face.m_index[2]; 
				index ++;
			}

			BuildFromVertexListIndexList(triangleCount, faceCount, &materialsPool[0], 
				&points[0].m_x, sizeof (dgVector), vertexIndexList,
				&normal.m_x, sizeof (dgVector), &normalIndexListPool[0],
				&uv.m_x, sizeof (dgVector), &normalIndexListPool[0],
				&uv.m_x, sizeof (dgVector), &normalIndexListPool[0]);

            RepairTJoints ();
		}
	}
}


dgMeshEffect* dgMeshEffect::CreateDelaunayTetrahedralization (dgMemoryAllocator* const allocator, dgInt32 pointCount, dgInt32 pointStrideInBytes, const dgFloat32* const pointCloud, dgInt32 materialId, const dgMatrix& textureProjectionMatrix)
{
	dgAssert (0);
	return NULL;
}




dgMeshEffect* dgMeshEffect::CreateVoronoiConvexDecomposition (dgMemoryAllocator* const allocator, dgInt32 pointCount, dgInt32 pointStrideInBytes, const dgFloat32* const pointCloud, dgInt32 materialId, const dgMatrix& textureProjectionMatrix)
{
	dgFloat32 normalAngleInRadians = 30.0f * 3.1416f / 180.0f;

	dgStack<dgBigVector> buffer(pointCount + 16);
	dgBigVector* const pool = &buffer[0];
	dgInt32 count = 0;
	dgFloat64 quantizeFactor = dgFloat64 (16.0f);
	dgFloat64 invQuantizeFactor = dgFloat64 (1.0f) / quantizeFactor;
	dgInt32 stride = pointStrideInBytes / sizeof (dgFloat32); 

	dgBigVector pMin (dgFloat32 (1.0e10f), dgFloat32 (1.0e10f), dgFloat32 (1.0e10f), dgFloat32 (0.0f));
	dgBigVector pMax (dgFloat32 (-1.0e10f), dgFloat32 (-1.0e10f), dgFloat32 (-1.0e10f), dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < pointCount; i ++) {
		dgFloat64 x = pointCloud[i * stride + 0];
		dgFloat64 y	= pointCloud[i * stride + 1];
		dgFloat64 z	= pointCloud[i * stride + 2];
		x = floor (x * quantizeFactor) * invQuantizeFactor;
		y = floor (y * quantizeFactor) * invQuantizeFactor;
		z = floor (z * quantizeFactor) * invQuantizeFactor;
		dgBigVector p (x, y, z, dgFloat64 (0.0f));
		pMin = dgBigVector (dgMin (x, pMin.m_x), dgMin (y, pMin.m_y), dgMin (z, pMin.m_z), dgFloat64 (0.0f));
		pMax = dgBigVector (dgMax (x, pMax.m_x), dgMax (y, pMax.m_y), dgMax (z, pMax.m_z), dgFloat64 (0.0f));
		pool[count] = p;
		count ++;
	}
	// add the bbox as a barrier
	pool[count + 0] = dgBigVector ( pMin.m_x, pMin.m_y, pMin.m_z, dgFloat64 (0.0f));
	pool[count + 1] = dgBigVector ( pMax.m_x, pMin.m_y, pMin.m_z, dgFloat64 (0.0f));
	pool[count + 2] = dgBigVector ( pMin.m_x, pMax.m_y, pMin.m_z, dgFloat64 (0.0f));
	pool[count + 3] = dgBigVector ( pMax.m_x, pMax.m_y, pMin.m_z, dgFloat64 (0.0f));
	pool[count + 4] = dgBigVector ( pMin.m_x, pMin.m_y, pMax.m_z, dgFloat64 (0.0f));
	pool[count + 5] = dgBigVector ( pMax.m_x, pMin.m_y, pMax.m_z, dgFloat64 (0.0f));
	pool[count + 6] = dgBigVector ( pMin.m_x, pMax.m_y, pMax.m_z, dgFloat64 (0.0f));
	pool[count + 7] = dgBigVector ( pMax.m_x, pMax.m_y, pMax.m_z, dgFloat64 (0.0f));
	count += 8;

	dgStack<dgInt32> indexList(count);
	count = dgVertexListToIndexList(&pool[0].m_x, sizeof (dgBigVector), 3, count, &indexList[0], dgFloat64 (5.0e-2f));	
	dgAssert (count >= 8);

	dgFloat64 maxSize = dgMax(pMax.m_x - pMin.m_x, pMax.m_y - pMin.m_y, pMax.m_z - pMin.m_z);
	pMin -= dgBigVector (maxSize, maxSize, maxSize, dgFloat64 (0.0f));
	pMax += dgBigVector (maxSize, maxSize, maxSize, dgFloat64 (0.0f));

	// add the a guard zone, so that we do no have to clip
	dgInt32 guadVertexKey = count;
	pool[count + 0] = dgBigVector ( pMin.m_x, pMin.m_y, pMin.m_z, dgFloat64 (0.0f));
	pool[count + 1] = dgBigVector ( pMax.m_x, pMin.m_y, pMin.m_z, dgFloat64 (0.0f));
	pool[count + 2] = dgBigVector ( pMin.m_x, pMax.m_y, pMin.m_z, dgFloat64 (0.0f));
	pool[count + 3] = dgBigVector ( pMax.m_x, pMax.m_y, pMin.m_z, dgFloat64 (0.0f));
	pool[count + 4] = dgBigVector ( pMin.m_x, pMin.m_y, pMax.m_z, dgFloat64 (0.0f));
	pool[count + 5] = dgBigVector ( pMax.m_x, pMin.m_y, pMax.m_z, dgFloat64 (0.0f));
	pool[count + 6] = dgBigVector ( pMin.m_x, pMax.m_y, pMax.m_z, dgFloat64 (0.0f));
	pool[count + 7] = dgBigVector ( pMax.m_x, pMax.m_y, pMax.m_z, dgFloat64 (0.0f));
	count += 8; 


	dgDelaunayTetrahedralization delaunayTetrahedras (allocator, &pool[0].m_x, count, sizeof (dgBigVector), dgFloat32 (0.0f));
	delaunayTetrahedras.RemoveUpperHull ();

//	delaunayTetrahedras.Save("xxx0.txt");

	dgInt32 tetraCount = delaunayTetrahedras.GetCount();
	dgStack<dgBigVector> voronoiPoints(tetraCount + 32);
	dgStack<dgDelaunayTetrahedralization::dgListNode*> tetradrumNode(tetraCount);
	dgTree<dgList<dgInt32>, dgInt32> delanayNodes (allocator);	

	dgInt32 index = 0;
	const dgHullVector* const delanayPoints = delaunayTetrahedras.GetHullVertexArray();
	for (dgDelaunayTetrahedralization::dgListNode* node = delaunayTetrahedras.GetFirst(); node; node = node->GetNext()) {
		dgConvexHull4dTetraherum& tetra = node->GetInfo();
		voronoiPoints[index] = tetra.CircumSphereCenter (delanayPoints);
		tetradrumNode[index] = node;

		for (dgInt32 i = 0; i < 4; i ++) {
			dgTree<dgList<dgInt32>, dgInt32>::dgTreeNode* header = delanayNodes.Find(tetra.m_faces[0].m_index[i]);
			if (!header) {
				dgList<dgInt32> list (allocator);
				header = delanayNodes.Insert(list, tetra.m_faces[0].m_index[i]);
			}
			header->GetInfo().Append (index);
		}
		index ++;
	}

	dgMeshEffect* const voronoiPartition = new (allocator) dgMeshEffect (allocator);
	voronoiPartition->BeginPolygon();
	dgFloat64 layer = dgFloat64 (0.0f);

	dgTree<dgList<dgInt32>, dgInt32>::Iterator iter (delanayNodes);
	for (iter.Begin(); iter; iter ++) {
		dgTree<dgList<dgInt32>, dgInt32>::dgTreeNode* const nodeNode = iter.GetNode();
		const dgList<dgInt32>& list = nodeNode->GetInfo();
		dgInt32 key = nodeNode->GetKey();

		if (key < guadVertexKey) {
			dgBigVector pointArray[512];
			dgInt32 indexArray[512];
			
			dgInt32 count = 0;
			for (dgList<dgInt32>::dgListNode* ptr = list.GetFirst(); ptr; ptr = ptr->GetNext()) {
				dgInt32 i = ptr->GetInfo();
				pointArray[count] = voronoiPoints[i];
				count ++;
				dgAssert (count < dgInt32 (sizeof (pointArray) / sizeof (pointArray[0])));
			}

			count = dgVertexListToIndexList(&pointArray[0].m_x, sizeof (dgBigVector), 3, count, &indexArray[0], dgFloat64 (1.0e-3f));	
			if (count >= 4) {
				dgMeshEffect convexMesh (allocator, &pointArray[0].m_x, count, sizeof (dgBigVector), dgFloat64 (0.0f));
				if (convexMesh.GetCount()) {
					convexMesh.CalculateNormals(normalAngleInRadians);
					convexMesh.UniformBoxMapping (materialId, textureProjectionMatrix);

					for (dgInt32 i = 0; i < convexMesh.m_pointCount; i ++) {
						convexMesh.m_points[i].m_w = layer;
					}
					for (dgInt32 i = 0; i < convexMesh.m_atribCount; i ++) {
						convexMesh.m_attrib[i].m_vertex.m_w = layer;
					}
					voronoiPartition->MergeFaces(&convexMesh);
					layer += dgFloat64 (1.0f);

				}
			}
		}
	}
	voronoiPartition->EndPolygon(dgFloat64 (1.0e-8f), false);

//	voronoiPartition->SaveOFF("xxx0.off");

	//voronoiPartition->ConvertToPolygons();
	return voronoiPartition;
}
