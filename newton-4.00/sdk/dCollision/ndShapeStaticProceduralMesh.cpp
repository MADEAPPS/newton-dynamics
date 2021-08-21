/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#include "dCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndContact.h"
#include "ndBodyKinematic.h"
#include "ndShapeInstance.h"
#include "ndShapeStaticProceduralMesh.h"

template<class T>
class dTempArray : public dArray<T>
{
	public:
	dTempArray(dInt32 maxSize, T* const buffer) 
		:dArray<T>()
		,m_buffer(buffer)
	{
		dArray<T>::m_array = buffer;
		dArray<T>::m_capacity = maxSize;
	}

	~dTempArray()
	{
		dArray<T>::m_array = nullptr;
	}

	T* m_buffer;
};

ndShapeStaticProceduralMesh::ndShapeStaticProceduralMesh(dFloat32 sizex, dFloat32 sizey, dFloat32 sizez)
	:ndShapeStaticMesh(m_staticProceduralMesh)
	,m_minBox(dVector::m_negOne * dVector::m_half * dVector(sizex, sizey, sizez, dFloat32(0.0f)))
	,m_maxBox(dVector::m_half * dVector(sizex, sizey, sizez, dFloat32(0.0f)))
	,m_localData()
	,m_maxVertexCount(256)
	,m_maxFaceCount(64)
{
	CalculateLocalObb();
}

ndShapeStaticProceduralMesh::ndShapeStaticProceduralMesh(const dLoadSaveBase::dLoadDescriptor&)
	:ndShapeStaticMesh(m_staticProceduralMesh)
{
	dAssert(0);
}

ndShapeStaticProceduralMesh::~ndShapeStaticProceduralMesh(void)
{
}

void ndShapeStaticProceduralMesh::SetMaxVertexAndFaces(dInt32 maxVertex, dInt32 maxFaces)
{
	m_maxVertexCount = maxVertex;
	m_maxFaceCount = maxFaces;
}

//void ndShapeStaticProceduralMesh::Save(nd::TiXmlElement* const xmlNode, const char* const assetPath, dInt32 nodeid) const
void ndShapeStaticProceduralMesh::Save(const dLoadSaveBase::dSaveDescriptor&) const
{
	dAssert(0);
}

ndShapeInfo ndShapeStaticProceduralMesh::GetShapeInfo() const
{
	ndShapeInfo info(ndShapeStaticMesh::GetShapeInfo());
	info.m_procedural.m_noUsed = 0;
	return info;
}

void ndShapeStaticProceduralMesh::CalculateLocalObb()
{
	m_boxSize = (m_maxBox - m_minBox) * dVector::m_half;
	m_boxOrigin = (m_maxBox + m_minBox) * dVector::m_half;
}

void ndShapeStaticProceduralMesh::GetCollidingFaces(ndPolygonMeshDesc* const data) const
{
	dVector* const vertexBuffer = dAlloca(dVector, m_maxVertexCount);
	dInt32* const faceBuffer = dAlloca(dInt32, m_maxFaceCount);
	dInt32* const materialBuffer = dAlloca(dInt32, m_maxFaceCount);
	dInt32* const indexBuffer = dAlloca(dInt32, m_maxFaceCount * 4);

	dTempArray<dVector> vertexList(m_maxVertexCount, vertexBuffer);
	dTempArray<dInt32> faceList(m_maxFaceCount, faceBuffer);
	dTempArray<dInt32> faceMaterialList(m_maxFaceCount, materialBuffer);
	dTempArray<dInt32> indexList(m_maxFaceCount * 4, indexBuffer);

	GetCollidingFaces(data->GetOrigin(), data->GetTarget(), vertexList, faceList, faceMaterialList, indexList);
	if (faceList.GetCount() == 0)
	{
		return;
	}

	std::thread::id threadId = std::this_thread::get_id();
	dList<ndLocalThreadData>::dNode* localDataNode = nullptr;
	for (dList<ndLocalThreadData>::dNode* node = m_localData.GetFirst(); node; node = node->GetNext())
	{
		if (node->GetInfo().m_threadId == threadId)
		{
			localDataNode = node;
			break;
		}
	}
	
	if (!localDataNode)
	{
		localDataNode = m_localData.Append();
		localDataNode->GetInfo().m_threadId = threadId;
	}
	
	// scan the vertices's intersected by the box extend
	dArray<dVector>& vertex = localDataNode->GetInfo().m_vertex;
	vertex.SetCount(vertexList.GetCount() + faceList.GetCount());
	memcpy(&vertex[0], &vertexList[0], vertexList.GetCount() * sizeof(dVector));
	
	ndEdgeMap edgeMap;
	dInt32 index = 0;
	dInt32 faceStart = 0;
	dInt32* const indices = data->m_globalFaceVertexIndex;
	dInt32* const faceIndexCount = data->m_meshData.m_globalFaceIndexCount;
	
	for (dInt32 i = 0; i < faceList.GetCount(); i++)
	{
		dInt32 i0 = indexList[faceStart + 0];
		dInt32 i1 = indexList[faceStart + 1];
		dVector area(dVector::m_zero);
		dVector edge0(vertex[i1] - vertex[i0]);

		for (dInt32 j = 2; j < faceList[i]; j++)
		{
			dInt32 i2 = indexList[faceStart + j];
			const dVector edge1(vertex[i2] - vertex[i0]);
			area += edge0.CrossProduct(edge1);
			edge0 = edge1;
		}

		dFloat32 faceSize = dSqrt(area.DotProduct(area).GetScalar());
		dInt32 normalIndex = vertexList.GetCount() + i;

		vertex[normalIndex] = area.Scale (dFloat32 (1.0f) / faceSize);

		indices[index + faceList[i] + 0] = faceMaterialList[i];
		indices[index + faceList[i] + 1] = normalIndex;
		indices[index + 2 * faceList[i] + 2] = dInt32(faceSize * dFloat32(0.5f));

		dInt32 j0 = faceList[i] - 1;
		faceIndexCount[i] = faceList[i];
		for (dInt32 j = 0; j < faceList[i]; j++) 
		{
			ndEdge edge;
			edge.m_i0 = indexList[faceStart + j0];
			edge.m_i1 = indexList[faceStart + j];
			dInt32 normalEntryIndex = index + j + faceList[i] + 2;
			edgeMap.Insert(normalEntryIndex, edge);

			indices[index + j] = indexList[faceStart + j];
			indices[normalEntryIndex] = normalIndex;

			j0 = j;
		}
		faceStart += faceList[i];
		index += faceList[i] * 2 + 3;
	}

	ndEdgeMap::Iterator iter(edgeMap);
	for (iter.Begin(); iter; iter++)
	{
		ndEdgeMap::dNode* const edgeNode = iter.GetNode();
		if (edgeNode->GetInfo() != -1)
		{
			ndEdge twin(iter.GetKey());
			dSwap(twin.m_i0, twin.m_i1);
			ndEdgeMap::dNode* const twinNode = edgeMap.Find(twin);
			if (twinNode)
			{
				dInt32 i0 = edgeNode->GetInfo();
				dInt32 i1 = twinNode->GetInfo();
				dSwap(indices[i0], indices[i1]);
				twinNode->GetInfo() = -1;
			}
		}
		edgeNode->GetInfo() = -1;
	}

	dInt32 faceCount0 = 0;
	dInt32 faceIndexCount0 = 0;
	dInt32 faceIndexCount1 = 0;
	dInt32 stride = sizeof(dVector) / sizeof(dFloat32);
	
	dInt32* const address = data->m_meshData.m_globalFaceIndexStart;
	dFloat32* const hitDistance = data->m_meshData.m_globalHitDistance;
	if (data->m_doContinuesCollisionTest) 
	{
		dAssert(0);
		//dFastRay ray(dVector::m_zero, data->m_boxDistanceTravelInMeshSpace);
		//for (dInt32 i = 0; i < faceCount; i++) 
		//{
		//	const dInt32* const indexArray = &indices[faceIndexCount1];
		//	const dVector& faceNormal = vertex[indexArray[4]];
		//	dFloat32 dist = data->PolygonBoxRayDistance(faceNormal, 3, indexArray, stride, &vertex[0].m_x, ray);
		//	if (dist < dFloat32(1.0f)) 
		//	{
		//		hitDistance[faceCount0] = dist;
		//		address[faceCount0] = faceIndexCount0;
		//		memcpy(&indices[faceIndexCount0], indexArray, 9 * sizeof(dInt32));
		//		faceCount0++;
		//		faceIndexCount0 += 9;
		//	}
		//	faceIndexCount1 += 9;
		//}
	}
	else 
	{
		for (dInt32 i = 0; i < faceList.GetCount(); i++)
		{
			const dInt32 vertexCount = faceIndexCount[i];
			const dInt32* const indexArray = &indices[faceIndexCount1];
			const dVector& faceNormal = vertex[indexArray[vertexCount + 1]];
			dFloat32 dist = data->PolygonBoxDistance(faceNormal, vertexCount, indexArray, stride, &vertex[0].m_x);
			if (dist > dFloat32(0.0f)) 
			{
				hitDistance[faceCount0] = dist;
				address[faceCount0] = faceIndexCount0;
				memcpy(&indices[faceIndexCount0], indexArray, (vertexCount * 2 + 3) * sizeof(dInt32));
				faceCount0++;
				faceIndexCount0 += vertexCount * 2 + 3;
			}
			faceIndexCount1 += vertexCount * 2 + 3;
		}
	}
	
	if (faceCount0) 
	{
		// initialize the callback data structure
		data->m_faceCount = faceCount0;
		data->m_vertex = &vertex[0].m_x;
		data->m_faceVertexIndex = indices;
		data->m_faceIndexStart = address;
		data->m_hitDistance = hitDistance;
		data->m_faceIndexCount = faceIndexCount;
		data->m_vertexStrideInBytes = sizeof(dVector);
	}
}

