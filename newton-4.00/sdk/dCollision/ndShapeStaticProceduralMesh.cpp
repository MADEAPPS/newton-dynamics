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

#include "ndCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndContact.h"
#include "ndBodyKinematic.h"
#include "ndShapeInstance.h"
#include "ndShapeStaticProceduralMesh.h"

D_CLASS_REFLECTION_IMPLEMENT_LOADER(ndShapeStaticProceduralMesh)

template<class T>
class dTempArray : public ndArray<T>
{
	public:
	dTempArray(ndInt32 maxSize, T* const buffer) 
		:ndArray<T>()
		,m_buffer(buffer)
	{
		ndArray<T>::m_array = buffer;
		ndArray<T>::m_capacity = maxSize;
	}

	~dTempArray()
	{
		ndArray<T>::m_array = nullptr;
	}

	T* m_buffer;
};

ndShapeStaticProceduralMesh::ndShapeStaticProceduralMesh(ndFloat32 sizex, ndFloat32 sizey, ndFloat32 sizez)
	:ndShapeStaticMesh(m_staticProceduralMesh)
	,m_minBox(ndVector::m_negOne * ndVector::m_half * ndVector(sizex, sizey, sizez, ndFloat32(0.0f)))
	,m_maxBox(ndVector::m_half * ndVector(sizex, sizey, sizez, ndFloat32(0.0f)))
	,m_localData()
	,m_maxFaceCount(64)
	,m_maxVertexCount(256)
{
	CalculateLocalObb();
}

ndShapeStaticProceduralMesh::ndShapeStaticProceduralMesh(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndShapeStaticMesh(m_staticProceduralMesh)
{
	const nd::TiXmlNode* const xmlNode = desc.m_rootNode;

	//CalculateLocalObb();
	m_minBox = xmlGetVector3(xmlNode, "minBox");
	m_maxBox = xmlGetVector3(xmlNode, "maxBox");
	m_maxFaceCount = xmlGetInt(xmlNode, "maxFaceCount");
	m_maxVertexCount = xmlGetInt(xmlNode, "maxVertexCount");
}

ndShapeStaticProceduralMesh::~ndShapeStaticProceduralMesh(void)
{
}

void ndShapeStaticProceduralMesh::SetMaxVertexAndFaces(ndInt32 maxVertex, ndInt32 maxFaces)
{
	m_maxFaceCount = maxFaces;
	m_maxVertexCount = maxVertex;
}

void ndShapeStaticProceduralMesh::Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const
{
	nd::TiXmlElement* const childNode = new nd::TiXmlElement(ClassName());
	desc.m_rootNode->LinkEndChild(childNode);
	childNode->SetAttribute("hashId", desc.m_nodeNodeHash);
	ndShapeStaticMesh::Save(ndLoadSaveBase::ndSaveDescriptor(desc, childNode));

	xmlSaveParam(childNode, "minBox", m_minBox);
	xmlSaveParam(childNode, "maxBox", m_maxBox);
	xmlSaveParam(childNode, "maxFaceCount", m_maxFaceCount);
	xmlSaveParam(childNode, "maxVertexCount", m_maxVertexCount);
}

ndShapeInfo ndShapeStaticProceduralMesh::GetShapeInfo() const
{
	ndShapeInfo info(ndShapeStaticMesh::GetShapeInfo());
	info.m_procedural.m_noUsed = 0;
	return info;
}

void ndShapeStaticProceduralMesh::CalculateLocalObb()
{
	m_boxSize = (m_maxBox - m_minBox) * ndVector::m_half;
	m_boxOrigin = (m_maxBox + m_minBox) * ndVector::m_half;
}

void ndShapeStaticProceduralMesh::GetCollidingFaces(ndPolygonMeshDesc* const data) const
{
	ndVector* const vertexBuffer = dAlloca(ndVector, m_maxVertexCount);
	ndInt32* const faceBuffer = dAlloca(ndInt32, m_maxFaceCount);
	ndInt32* const materialBuffer = dAlloca(ndInt32, m_maxFaceCount);
	ndInt32* const indexBuffer = dAlloca(ndInt32, m_maxFaceCount * 4);

	dTempArray<ndVector> vertexList(m_maxVertexCount, vertexBuffer);
	dTempArray<ndInt32> faceList(m_maxFaceCount, faceBuffer);
	dTempArray<ndInt32> faceMaterialList(m_maxFaceCount, materialBuffer);
	dTempArray<ndInt32> indexList(m_maxFaceCount * 4, indexBuffer);

	GetCollidingFaces(data->GetOrigin(), data->GetTarget(), vertexList, faceList, faceMaterialList, indexList);
	if (faceList.GetCount() == 0)
	{
		return;
	}

	std::thread::id threadId = std::this_thread::get_id();
	ndList<ndLocalThreadData>::ndNode* localDataNode = nullptr;
	for (ndList<ndLocalThreadData>::ndNode* node = m_localData.GetFirst(); node; node = node->GetNext())
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
	ndArray<ndVector>& vertex = localDataNode->GetInfo().m_vertex;
	vertex.SetCount(vertexList.GetCount() + faceList.GetCount());
	memcpy(&vertex[0], &vertexList[0], vertexList.GetCount() * sizeof(ndVector));
	
	ndEdgeMap edgeMap;
	ndInt32 index = 0;
	ndInt32 faceStart = 0;
	ndInt32* const indices = data->m_globalFaceVertexIndex;
	ndInt32* const faceIndexCount = data->m_meshData.m_globalFaceIndexCount;
	
	for (ndInt32 i = 0; i < faceList.GetCount(); i++)
	{
		ndInt32 i0 = indexList[faceStart + 0];
		ndInt32 i1 = indexList[faceStart + 1];
		ndVector area(ndVector::m_zero);
		ndVector edge0(vertex[i1] - vertex[i0]);

		for (ndInt32 j = 2; j < faceList[i]; j++)
		{
			ndInt32 i2 = indexList[faceStart + j];
			const ndVector edge1(vertex[i2] - vertex[i0]);
			area += edge0.CrossProduct(edge1);
			edge0 = edge1;
		}

		ndFloat32 faceSize = ndSqrt(area.DotProduct(area).GetScalar());
		ndInt32 normalIndex = vertexList.GetCount() + i;

		vertex[normalIndex] = area.Scale (ndFloat32 (1.0f) / faceSize);

		indices[index + faceList[i] + 0] = faceMaterialList[i];
		indices[index + faceList[i] + 1] = normalIndex;
		indices[index + 2 * faceList[i] + 2] = ndInt32(faceSize * ndFloat32(0.5f));

		ndInt32 j0 = faceList[i] - 1;
		faceIndexCount[i] = faceList[i];
		for (ndInt32 j = 0; j < faceList[i]; j++) 
		{
			ndEdge edge;
			edge.m_i0 = indexList[faceStart + j0];
			edge.m_i1 = indexList[faceStart + j];
			ndInt32 normalEntryIndex = index + j + faceList[i] + 2;
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
		ndEdgeMap::ndNode* const edgeNode = iter.GetNode();
		if (edgeNode->GetInfo() != -1)
		{
			ndEdge twin(iter.GetKey());
			dSwap(twin.m_i0, twin.m_i1);
			ndEdgeMap::ndNode* const twinNode = edgeMap.Find(twin);
			if (twinNode)
			{
				ndInt32 i0 = edgeNode->GetInfo();
				ndInt32 i1 = twinNode->GetInfo();
				dSwap(indices[i0], indices[i1]);
				twinNode->GetInfo() = -1;
			}
		}
		edgeNode->GetInfo() = -1;
	}

	ndInt32 faceCount0 = 0;
	ndInt32 faceIndexCount0 = 0;
	ndInt32 faceIndexCount1 = 0;
	ndInt32 stride = sizeof(ndVector) / sizeof(ndFloat32);
	
	ndInt32* const address = data->m_meshData.m_globalFaceIndexStart;
	ndFloat32* const hitDistance = data->m_meshData.m_globalHitDistance;
	if (data->m_doContinuesCollisionTest) 
	{
		dAssert(0);
		//dFastRay ray(ndVector::m_zero, data->m_boxDistanceTravelInMeshSpace);
		//for (ndInt32 i = 0; i < faceCount; i++) 
		//{
		//	const ndInt32* const indexArray = &indices[faceIndexCount1];
		//	const ndVector& faceNormal = vertex[indexArray[4]];
		//	ndFloat32 dist = data->PolygonBoxRayDistance(faceNormal, 3, indexArray, stride, &vertex[0].m_x, ray);
		//	if (dist < ndFloat32(1.0f)) 
		//	{
		//		hitDistance[faceCount0] = dist;
		//		address[faceCount0] = faceIndexCount0;
		//		memcpy(&indices[faceIndexCount0], indexArray, 9 * sizeof(ndInt32));
		//		faceCount0++;
		//		faceIndexCount0 += 9;
		//	}
		//	faceIndexCount1 += 9;
		//}
	}
	else 
	{
		for (ndInt32 i = 0; i < faceList.GetCount(); i++)
		{
			const ndInt32 vertexCount = faceIndexCount[i];
			const ndInt32* const indexArray = &indices[faceIndexCount1];
			const ndVector& faceNormal = vertex[indexArray[vertexCount + 1]];
			ndFloat32 dist = data->PolygonBoxDistance(faceNormal, vertexCount, indexArray, stride, &vertex[0].m_x);
			if (dist > ndFloat32(0.0f)) 
			{
				hitDistance[faceCount0] = dist;
				address[faceCount0] = faceIndexCount0;
				memcpy(&indices[faceIndexCount0], indexArray, (vertexCount * 2 + 3) * sizeof(ndInt32));
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
		data->m_vertexStrideInBytes = sizeof(ndVector);
	}
}

