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

/****************************************************************************
*
*  Visual C++ 6.0 created by: Julio Jerez
*
****************************************************************************/
#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndList.h"
#include "ndTree.h"
#include "ndStack.h"
#include "ndPolyhedra.h"
#include "ndPolygonSoupBuilder.h"

#define ND_POINTS_RUN (512 * 1024)

class ndPolygonSoupBuilder::ndFaceInfo
{
	public:
	ndInt32 indexCount;
	ndInt32 indexStart;
};

class ndPolygonSoupBuilder::ndFaceBucket: public ndList<ndFaceInfo>
{
	public: 
	ndFaceBucket ()
		:ndList<ndFaceInfo>()
	{
	}
};

class ndPolygonSoupBuilder::ndFaceMap: public ndTree<ndFaceBucket, ndInt32>
{
	public:
	ndFaceMap (ndPolygonSoupBuilder& builder)
		:ndTree<ndFaceBucket, ndInt32>()
	{
		ndInt32 polygonIndex = 0;
		ndInt32 faceCount = ndInt32(builder.m_faceVertexCount.GetCount());
		const ndInt32* const faceVertexCounts = &builder.m_faceVertexCount[0];
		const ndInt32* const faceVertexIndex = &builder.m_vertexIndex[0];
		for (ndInt32 i = 0; i < faceCount; ++i) 
		{
			ndInt32 count = faceVertexCounts[i];
			ndInt32 attribute = faceVertexIndex[polygonIndex + count - 1];
			
			ndNode* node = Find(attribute);
			if (!node) 
			{
				ndFaceBucket tmp;
				node = Insert(tmp, attribute);
			}

			ndFaceBucket& bucket = node->GetInfo();
			ndFaceInfo& face = bucket.Append()->GetInfo();
			face.indexCount = count;
			face.indexStart = polygonIndex;
			polygonIndex += count;
		}
	}
};

class ndPolygonSoupBuilder::ndPolySoupFilterAllocator: public ndPolyhedra
{
	public: 
	ndPolySoupFilterAllocator ()
		:ndPolyhedra ()
	{
	}

	~ndPolySoupFilterAllocator ()
	{
	}

	ndInt32 AddFilterFace (ndUnsigned32 count, ndInt32* const pool)
	{
		BeginFace();
		ndAssert (count);
		bool reduction = true;
		while (reduction && !AddFace (ndInt32 (count), pool)) 
		{
			reduction = false;
			if (count >3) 
			{
				for (ndUnsigned32 i = 0; i < count; ++i) 
				{
					for (ndUnsigned32 j = i + 1; j < count; ++j) 
					{
						if (pool[j] == pool[i]) 
						{
							for (i = j; i < count - 1; ++i) 
							{
								pool[i] =  pool[i + 1];
							}
							count --;
							i = count;
							reduction = true;
							break;
						}
					}
				}
			}
		}
		EndFace();
		return reduction ? ndInt32 (count) : 0;
	}
};

ndPolygonSoupBuilder::ndPolygonSoupBuilder ()
	:m_faceVertexCount()
	,m_vertexIndex()
	,m_normalIndex()
	,m_vertexPoints()
	,m_normalPoints()
{
	m_run = ND_POINTS_RUN;
}

ndPolygonSoupBuilder::ndPolygonSoupBuilder (const ndPolygonSoupBuilder& source)
	:m_faceVertexCount(ndInt32(source.m_faceVertexCount.GetCount()))
	,m_vertexIndex(ndInt32(source.m_vertexIndex.GetCount()))
	,m_normalIndex()
	,m_vertexPoints(ndInt32(source.m_vertexPoints.GetCount()))
	,m_normalPoints()
{
	m_run = ND_POINTS_RUN;
	m_faceVertexCount.SetCount(source.m_faceVertexCount.GetCount());
	m_vertexIndex.SetCount(source.m_vertexIndex.GetCount());
	m_vertexPoints.SetCount(source.m_vertexPoints.GetCount());
	
	ndMemCpy(&m_vertexIndex[0], &source.m_vertexIndex[0], source.m_vertexIndex.GetCount());
	ndMemCpy(&m_faceVertexCount[0], &source.m_faceVertexCount[0], source.m_faceVertexCount.GetCount());
	ndMemCpy(&m_vertexPoints[0], &source.m_vertexPoints[0], source.m_vertexPoints.GetCount());

	if (m_normalPoints.GetCount())
	{
		m_normalIndex.Resize(source.m_normalIndex.GetCount());
		m_normalPoints.Resize(source.m_normalPoints.GetCount());
		m_normalIndex.SetCount(source.m_normalIndex.GetCount());
		m_normalPoints.SetCount(source.m_normalPoints.GetCount());

		ndMemCpy(&m_normalIndex[0], &source.m_normalIndex[0], source.m_normalIndex.GetCount());
		ndMemCpy(&m_normalPoints[0], &source.m_normalPoints[0], source.m_normalPoints.GetCount());
	}
}

ndPolygonSoupBuilder::~ndPolygonSoupBuilder ()
{
}

void ndPolygonSoupBuilder::Begin()
{
	m_run = ND_POINTS_RUN;
	m_vertexIndex.SetCount(0);
	m_normalIndex.SetCount(0);
	m_vertexPoints.SetCount(0);
	m_normalPoints.SetCount(0);
	m_faceVertexCount.SetCount(0);
}

void ndPolygonSoupBuilder::SavePLY(const char* const fileName) const
{
	FILE* const file = fopen(fileName, "wb");

	fprintf(file, "ply\n");
	fprintf(file, "format ascii 1.0\n");

	fprintf(file, "element vertex %d\n", ndInt32(m_vertexPoints.GetCount()));
	fprintf(file, "property float x\n");
	fprintf(file, "property float y\n");
	fprintf(file, "property float z\n");
	fprintf(file, "element face %d\n", ndInt32(m_faceVertexCount.GetCount()));
	fprintf(file, "property list uchar int vertex_index\n");
	fprintf(file, "end_header\n");

	for (ndInt32 i = 0; i < m_vertexPoints.GetCount(); ++i)
	{
		const ndBigVector& point = m_vertexPoints[i];
		fprintf(file, "%f %f %f\n", point.m_x, point.m_y, point.m_z);
	}

	ndInt32 index = 0;
	for (ndInt32 i = 0; i < m_faceVertexCount.GetCount(); ++i)
	{
		ndInt32 count = m_faceVertexCount[i];
		fprintf(file, "%d", count - 1);
		for (ndInt32 j = 0; j < count - 1; ++j) 
		{
			fprintf(file, " %d", m_vertexIndex[index + j]);
		}
		index += count;
		fprintf(file, "\n");
	}
	fclose(file);
}

void ndPolygonSoupBuilder::LoadPLY(const char* const fileName)
{
	FILE* const file = fopen(fileName, "rb");

	char line[1024];
	char* ret0 = nullptr;
	ndInt32 ret1 = 0;
	ret0 = fgets(line, sizeof (line) - 1, file);
	if (!strncmp(line, "ply", 3))
	{
		ndInt32 faceCount;
		ndInt32 vertexCount;
		ret0 = fgets(line, sizeof(line) - 1, file);
		ret1 = fscanf(file, "%s %s %d\n", line, line, &vertexCount);
		ret0 = fgets(line, sizeof(line) - 1, file);
		ret0 = fgets(line, sizeof(line) - 1, file);
		ret0 = fgets(line, sizeof(line) - 1, file);
		ret1 = fscanf(file, "%s %s %d\n", line, line, &faceCount);
		ret0 = fgets(line, sizeof(line) - 1, file);
		ret0 = fgets(line, sizeof(line) - 1, file);

		ndArray<ndVector> vertexArray;
		vertexArray.SetCount(vertexCount);
		for (ndInt32 i = 0; i < vertexCount; ++i)
		{
			ndReal x;
			ndReal y;
			ndReal z;
			ret1 = fscanf(file, "%f %f %f", &x, &y, &z);
			vertexArray[i] = ndBigVector(ndFloat32(x), ndFloat32(y), ndFloat32(z), ndFloat32(0.0f));
		}

		Begin();
		ndVector face[64];
		for (ndInt32 i = 0; i < faceCount; ++i)
		{
			ndInt32 indexCount;
			ret1 = fscanf(file, "%d", &indexCount);
			for (ndInt32 j = 0; j < indexCount; ++j)
			{
				ndInt32 index;
				ret1 = fscanf(file, "%d", &index);
				face[j] = vertexArray[index];
			}
			AddFace(&face[0].m_x, sizeof (ndVector), indexCount, 0);
		}
	}
	fclose(file);
}

void ndPolygonSoupBuilder::AddFaceIndirect(const ndFloat32* const vertex, ndInt32 strideInBytes, ndInt32 faceId, const ndInt32* const indexArray, ndInt32 indexCount)
{
	ndInt32 faces[32];
	ndInt32 pool[512];

	const ndInt32 vertexCount = ndInt32(m_vertexPoints.GetCount());
	const ndInt32 stride = ndInt32 (strideInBytes / sizeof(ndFloat32));
	for (ndInt32 i = 0; i < indexCount; ++i)
	{
		pool[i] = i + vertexCount;
		const ndInt32 j = indexArray[i] * stride;
		ndBigVector point(vertex[j + 0], vertex[j + 1], vertex[j + 2], ndFloat32(0.0f));
		m_vertexPoints.PushBack (point);
	}

	ndInt32 convexFaces = 0;
	if (indexCount == 3)
	{
		convexFaces = 1;
		ndBigVector p0(m_vertexPoints[pool[2]]);
		for (ndInt32 j = 0; j < 3; ++j)
		{
			ndBigVector p1(m_vertexPoints[pool[j]]);
			ndBigVector edge(p1 - p0);
			ndFloat64 mag2 = edge.DotProduct(edge).GetScalar();
			if (mag2 < ndFloat32(1.0e-12f))
			{
				ndAssert(0);
				ndTrace(("rejecting degenerated face, edge too small\n"));
				convexFaces = 0;
			}
			p0 = p1;
		}

		if (convexFaces)
		{
			ndBigVector edge0(m_vertexPoints[pool[2]] - m_vertexPoints[pool[0]]);
			ndBigVector edge1(m_vertexPoints[pool[1]] - m_vertexPoints[pool[0]]);
			ndAssert(edge0.m_w == ndFloat32(0.0f));
			ndAssert(edge1.m_w == ndFloat32(0.0f));
			ndBigVector normal(edge0.CrossProduct(edge1));
			ndFloat64 mag2 = normal.DotProduct(normal).GetScalar();
			if (mag2 < ndFloat32(1.0e-12f))
			{
				ndAssert(0);
				ndTrace(("rejecting degenerated face, area too small\n"));
				convexFaces = 0;
			}
		}

		if (convexFaces)
		{
			faces[0] = 3;
		}
	}
	else
	{
		convexFaces = AddConvexFace(indexCount, pool, faces);
	}

	ndInt32 indexAcc = 0;
	for (ndInt32 j = 0; j < convexFaces; ++j)
	{
		ndInt32 count1 = faces[j];
		for (ndInt32 m = 0; m < count1; m++)
		{
			m_vertexIndex.PushBack(pool[indexAcc + m]);
		}
		m_vertexIndex.PushBack(faceId);
		indexAcc += count1;
		count1++;
		m_faceVertexCount.PushBack(count1);
	}

	m_run -= indexCount;
	if (m_run <= 0)
	{
		PackArray();
	}
}

void ndPolygonSoupBuilder::AddFace(const ndFloat32* const vertex, ndInt32 strideInBytes, ndInt32 vertexCount, const ndInt32 faceId)
{
	ndInt32 indexArray[1024];
	ndAssert(vertexCount < ndInt32 (sizeof(indexArray)/sizeof (indexArray[0])));
	for (ndInt32 i = 0; i < vertexCount; ++i)
	{
		indexArray[i] = i;
	}
	AddFaceIndirect(vertex, strideInBytes, faceId, indexArray, vertexCount);
}

void ndPolygonSoupBuilder::PackArray()
{
	ndStack<ndInt32> indexMapPool (ndInt32(m_vertexPoints.GetCount()));
	ndInt32* const indexMap = &indexMapPool[0];
	ndInt32 vertexCount = ndVertexListToIndexList(&m_vertexPoints[0].m_x, sizeof (ndBigVector), 3, ndInt32(m_vertexPoints.GetCount()), &indexMap[0], ndFloat32 (1.0e-6f));

	ndInt32 k = 0;
	for (ndInt32 i = 0; i < m_faceVertexCount.GetCount(); ++i)
	{
		ndInt32 count = m_faceVertexCount[i] - 1;
		for (ndInt32 j = 0; j < count; ++j) 
		{
			ndInt32 index = m_vertexIndex[k];
			index = indexMap[index];
			m_vertexIndex[k] = index;
			k ++;
		}
		k ++;
	}

	m_vertexPoints.Resize(vertexCount);
	m_vertexPoints.SetCount(vertexCount);
	m_run = ND_POINTS_RUN;
}

void ndPolygonSoupBuilder::Finalize()
{
	const ndInt32 faceCount = ndInt32(m_faceVertexCount.GetCount());
	if (faceCount)
	{
		ndStack<ndInt32> indexMapPool(ndInt32(m_vertexPoints.GetCount()));

		ndInt32* const indexMap = &indexMapPool[0];
		ndInt32 vertexCount = ndVertexListToIndexList(&m_vertexPoints[0].m_x, sizeof (ndBigVector), 3, ndInt32(m_vertexPoints.GetCount()), &indexMap[0], ndFloat32 (1.0e-4f));
		ndAssert(vertexCount <= m_vertexPoints.GetCount());
		m_vertexPoints.SetCount(vertexCount);

		ndInt32 k = 0;
		for (ndInt32 i = 0; i < faceCount; ++i) 
		{
			ndInt32 count = m_faceVertexCount[i] - 1;
			for (ndInt32 j = 0; j < count; ++j) 
			{
				ndInt32 index = m_vertexIndex[k];
				index = indexMap[index];
				m_vertexIndex[k] = index;
				k ++;
			}
			k ++;
		}
		OptimizeByIndividualFaces();
	}
}

void ndPolygonSoupBuilder::FinalizeAndOptimize(ndInt32 id)
{
	Finalize();
	ndPolyhedra polyhedra;
	ndPolygonSoupBuilder source(*this);
	ndPolygonSoupBuilder leftOver;
	ndInt32 tmpIndexPool[1024];
	ndVector tmpVertexPool[1024];

	Begin();
	leftOver.Begin();
	polyhedra.BeginFace ();
	ndInt32 attribute = id;
	ndInt32 faceIndexNumber = 0;
	for (ndInt32 i = 0; i < source.m_faceVertexCount.GetCount(); ++i)
	{
		ndInt32 indexCount = source.m_faceVertexCount[i];
		ndAssert (indexCount < 1024);

		ndEdge* const face = polyhedra.AddFace(indexCount - 1, &source.m_vertexIndex[faceIndexNumber]);
		if (!face) 
		{
			for (ndInt32 j = 0; j < indexCount - 1; ++j) 
			{
				ndInt32 index = source.m_vertexIndex[faceIndexNumber + j];
				tmpVertexPool[j] = source.m_vertexPoints[index];
				tmpIndexPool[j] = j;
			}
			leftOver.AddFaceIndirect(&tmpVertexPool[0].m_x, sizeof (ndVector), attribute, tmpIndexPool, indexCount - 1);
		} 
		else 
		{
			// set the attribute
			ndEdge* ptr = face;
			do 
			{
				ptr->m_userData = ndUnsigned64 (attribute);
				ptr = ptr->m_next;
			} while (ptr != face);
		}
		faceIndexNumber += indexCount; 
	} 
	polyhedra.EndFace();

	ndPolyhedra facesLeft;
	facesLeft.BeginFace();
	polyhedra.ConvexPartition (&source.m_vertexPoints[0].m_x, sizeof (ndBigVector), &facesLeft);
	facesLeft.EndFace();

	ndInt32 mark = polyhedra.IncLRU();
	ndPolyhedra::Iterator iter (polyhedra);
	for (iter.Begin(); iter; iter ++) 
	{
		ndEdge* const edge = &(*iter);
		if (edge->m_incidentFace < 0) 
		{
			continue;
		}
		if (edge->m_mark == mark) 
		{
			continue;
		}

		ndEdge* ptr = edge;
		ndInt32 indexCount = 0;
		do 
		{
			ptr->m_mark = mark;
			tmpVertexPool[indexCount] = source.m_vertexPoints[ptr->m_incidentVertex];
			tmpIndexPool[indexCount] = indexCount;
			indexCount ++;
			ptr = ptr->m_next;
		} while (ptr != edge);

		if (indexCount >= 3) 
		{
			//AddMesh (&tmpVertexPool[0].m_x, indexCount, sizeof (tmpVertexPool[0]), 1, &indexCount, tmpIndexPool, &attribute, dGetIdentityMatrix());
			AddFaceIndirect(&tmpVertexPool[0].m_x, sizeof (ndVector), attribute, tmpIndexPool, indexCount);
		}
	}

	mark = facesLeft.IncLRU();
	ndPolyhedra::Iterator iter1 (facesLeft);
	for (iter1.Begin(); iter1; iter1 ++) 
	{
		ndEdge* const edge = &(*iter1);
		if (edge->m_incidentFace < 0) 
		{
			continue;
		}
		if (edge->m_mark == mark) 
		{
			continue;
		}

		ndEdge* ptr = edge;
		ndInt32 indexCount = 0;
		do 
		{
			ptr->m_mark = mark;
			tmpVertexPool[indexCount] = source.m_vertexPoints[ptr->m_incidentVertex];
			tmpIndexPool[indexCount] = indexCount;
			indexCount ++;
			ptr = ptr->m_next;
		} while (ptr != edge);
		if (indexCount >= 3) 
		{
			//AddMesh (&tmpVertexPool[0].m_x, indexCount, sizeof (ndVector), 1, &indexCount, tmpIndexPool, &attribute, dGetIdentityMatrix());
			AddFaceIndirect(&tmpVertexPool[0].m_x, sizeof(ndVector), attribute, tmpIndexPool, indexCount);
		}
	}

	faceIndexNumber = 0;
	for (ndInt32 i = 0; i < leftOver.m_faceVertexCount.GetCount(); ++i)
	{
		ndInt32 indexCount = leftOver.m_faceVertexCount[i] - 1;
		for (ndInt32 j = 0; j < indexCount; ++j) 
		{
			ndInt32 index = leftOver.m_vertexIndex[faceIndexNumber + j];
			tmpVertexPool[j] = leftOver.m_vertexPoints[index];
			tmpIndexPool[j] = j;
		}
		AddFaceIndirect(&tmpVertexPool[0].m_x, sizeof(ndVector), attribute, tmpIndexPool, indexCount);

		faceIndexNumber += (indexCount + 1); 
	}

	Finalize();
}

void ndPolygonSoupBuilder::OptimizeByIndividualFaces()
{
	ndInt32* const faceArray = &m_faceVertexCount[0];
	ndInt32* const indexArray = &m_vertexIndex[0];

	ndInt32* const oldFaceArray = &m_faceVertexCount[0];
	ndInt32* const oldIndexArray = &m_vertexIndex[0];

	ndInt32 polygonIndex = 0;
	ndInt32 newFaceCount = 0;
	ndInt32 newIndexCount = 0;
	for (ndInt32 i = 0; i < m_faceVertexCount.GetCount(); ++i)
	{
		ndInt32 oldCount = oldFaceArray[i];
		ndInt32 count = FilterFace (oldCount - 1, &oldIndexArray[polygonIndex]);
		if (count) 
		{
			faceArray[newFaceCount] = count + 1;
			for (ndInt32 j = 0; j < count; ++j) 
			{
				indexArray[newIndexCount + j] = oldIndexArray[polygonIndex + j];
			}
			indexArray[newIndexCount + count] = oldIndexArray[polygonIndex + oldCount - 1];
			newFaceCount ++;
			newIndexCount += (count + 1);
		}
		polygonIndex += oldCount;
	}
	ndAssert (polygonIndex == m_vertexIndex.GetCount());

	m_vertexIndex.Resize(newIndexCount);
	m_faceVertexCount.Resize(newFaceCount);
	m_vertexIndex.SetCount(newIndexCount);
	m_faceVertexCount.SetCount(newFaceCount);
}

void ndPolygonSoupBuilder::End(bool optimize)
{
	if (optimize) 
	{
		ndPolygonSoupBuilder copy (*this);
		ndFaceMap faceMap (copy);

		Begin();
		ndFaceMap::Iterator iter (faceMap);
		for (iter.Begin(); iter; iter ++) 
		{
			const ndFaceBucket& bucket = iter.GetNode()->GetInfo();
			Optimize(iter.GetNode()->GetKey(), bucket, copy);
		}
	}
	Finalize();

	// build the normal array and adjacency array
	ndInt32 indexCount = 0;
	const ndInt32 faceCount = ndInt32(m_faceVertexCount.GetCount());
	if (faceCount)
	{
		// calculate all face the normals
		m_normalPoints.Resize(faceCount);
		m_normalPoints.SetCount(faceCount);
		for (ndInt32 i = 0; i < faceCount; ++i)
		{
			ndInt32 faceIndexCount = m_faceVertexCount[i];

			const ndInt32* const ptr = &m_vertexIndex[indexCount];
			ndBigVector v0(&m_vertexPoints[ptr[0]].m_x);
			ndBigVector v1(&m_vertexPoints[ptr[1]].m_x);
			ndBigVector e0(v1 - v0);
			ndBigVector normal0(ndBigVector::m_zero);
			for (ndInt32 j = 2; j < faceIndexCount - 1; ++j)
			{
				ndBigVector v2(&m_vertexPoints[ptr[j]].m_x);
				ndBigVector e1(v2 - v0);
				normal0 += e0.CrossProduct(e1);
				e0 = e1;
			}
			ndBigVector normal(normal0.Normalize());

			m_normalPoints[i].m_x = normal.m_x;
			m_normalPoints[i].m_y = normal.m_y;
			m_normalPoints[i].m_z = normal.m_z;
			m_normalPoints[i].m_w = ndFloat32(0.0f);
			indexCount += faceIndexCount;
		}

		m_normalIndex.Resize(faceCount);;
		m_normalIndex.SetCount(faceCount);
		ndInt32 normalCount = ndVertexListToIndexList(&m_normalPoints[0].m_x, sizeof(ndBigVector), 3, faceCount, &m_normalIndex[0], ndFloat32(1.0e-6f));
		ndAssert(normalCount <= m_normalPoints.GetCount());
		m_normalPoints.SetCount(normalCount);
	}
}

void ndPolygonSoupBuilder::Optimize(ndInt32 faceId, const ndFaceBucket& faceBucket, const ndPolygonSoupBuilder& source)
{
	#define DG_MESH_PARTITION_SIZE (1024 * 4)

	const ndInt32* const indexArray = &source.m_vertexIndex[0];
	const ndBigVector* const points = &source.m_vertexPoints[0];

	ndVector face[256];
	ndInt32 faceIndex[256];
	if (faceBucket.GetCount() >= DG_MESH_PARTITION_SIZE) 
	{
		ndStack<ndFaceBucket::ndNode*> array(faceBucket.GetCount());
		ndInt32 count = 0;
		for (ndFaceBucket::ndNode* node = faceBucket.GetFirst(); node; node = node->GetNext()) 
		{
			array[count] = node;
			count ++;
		}

		ndInt32 stack = 1;
		ndInt32 segments[32][2];
			
		segments[0][0] = 0;
		segments[0][1] = count;
	
		while (stack) 
		{
			stack --;
			ndInt32 faceStart = segments[stack][0];
			ndInt32 faceCount = segments[stack][1];

			if (faceCount <= DG_MESH_PARTITION_SIZE) 
			{
				ndPolygonSoupBuilder tmpBuilder;
				for (ndInt32 i = 0; i < faceCount; ++i) 
				{
					const ndFaceInfo& faceInfo = array[faceStart + i]->GetInfo();

					ndInt32 count1 = faceInfo.indexCount - 1;
					ndInt32 start1 = faceInfo.indexStart;
					ndAssert (faceId == indexArray[start1 + count1]);
					for (ndInt32 j = 0; j < count1; ++j) 
					{
						ndInt32 index = indexArray[start1 + j];
						face[j] = points[index];
						faceIndex[j] = j;
					}
					tmpBuilder.AddFaceIndirect(&face[0].m_x, sizeof(ndVector), faceId, faceIndex, count1);
				}
				tmpBuilder.FinalizeAndOptimize (faceId);

				ndInt32 faceIndexNumber = 0;
				for (ndInt32 i = 0; i < ndInt32(tmpBuilder.m_faceVertexCount.GetCount()); ++i)
				{
					ndInt32 indexCount = tmpBuilder.m_faceVertexCount[i] - 1;
					for (ndInt32 j = 0; j < indexCount; ++j) 
					{
						ndInt32 index = tmpBuilder.m_vertexIndex[faceIndexNumber + j];
						face[j] = tmpBuilder.m_vertexPoints[index];
						faceIndex[j] = j;
					}
					AddFaceIndirect(&face[0].m_x, sizeof(ndVector), faceId, faceIndex, indexCount);
					faceIndexNumber += (indexCount + 1); 
				}
			} 
			else 
			{
				ndBigVector median (ndBigVector::m_zero);
				ndBigVector varian (ndBigVector::m_zero);
				for (ndInt32 i = 0; i < faceCount; ++i) 
				{
					const ndFaceInfo& faceInfo = array[faceStart + i]->GetInfo();
					ndInt32 count1 = faceInfo.indexCount - 1;
					ndInt32 start1 = faceInfo.indexStart;
					ndBigVector p0 (ndFloat32 ( 1.0e10f), ndFloat32 ( 1.0e10f), ndFloat32 ( 1.0e10f), ndFloat32 (0.0f));
					ndBigVector p1 (ndFloat32 (-1.0e10f), ndFloat32 (-1.0e10f), ndFloat32 (-1.0e10f), ndFloat32 (0.0f));
					for (ndInt32 j = 0; j < count1; ++j) 
					{
						ndInt32 index = indexArray[start1 + j];
						const ndBigVector& p = points[index];
						ndAssert(p.m_w == ndFloat32(0.0f));
						p0 = p0.GetMin(p);
						p1 = p1.GetMax(p);
					}
					ndBigVector p ((p0 + p1).Scale (0.5f));
					median += p;
					varian += p * p;
				}

				varian = varian.Scale (ndFloat32 (faceCount)) - median * median;

				ndInt32 axis = 0;
				ndFloat32 maxVarian = ndFloat32 (-1.0e10f);
				for (ndInt32 i = 0; i < 3; ++i) 
				{
					if (varian[i] > maxVarian) 
					{
						axis = i;
						maxVarian = ndFloat32 (varian[i]);
					}
				}
				ndBigVector center = median.Scale (ndFloat32 (1.0f) / ndFloat32 (faceCount));
				ndFloat64 axisVal = center[axis];

				ndInt32 leftCount = 0;
				ndInt32 lastFace = faceCount;

				for (ndInt32 i = 0; i < lastFace; ++i) 
				{
					ndInt32 side = 0;
					const ndFaceInfo& faceInfo = array[faceStart + i]->GetInfo();

					ndInt32 start1 = faceInfo.indexStart;
					ndInt32 count1 = faceInfo.indexCount - 1;
					for (ndInt32 j = 0; j < count1; ++j) 
					{
						ndInt32 index = indexArray[start1 + j];
						const ndBigVector& p = points[index];
						if (p[axis] > axisVal) 
						{
							side = 1;
							break;
						}
					}

					if (side) 
					{
						ndSwap (array[faceStart + i], array[faceStart + lastFace - 1]);
						lastFace --;
						i --;
					} 
					else 
					{
						leftCount ++;
					}
				}
				ndAssert (leftCount);
				ndAssert (leftCount < faceCount);

				segments[stack][0] = faceStart;
				segments[stack][1] = leftCount;
				stack ++;

				segments[stack][0] = faceStart + leftCount;
				segments[stack][1] = faceCount - leftCount;
				stack ++;
			}
		}
	} 
	else 
	{
		ndPolygonSoupBuilder tmpBuilder;
		for (ndFaceBucket::ndNode* node = faceBucket.GetFirst(); node; node = node->GetNext()) 
		{
			const ndFaceInfo& faceInfo = node->GetInfo();

			ndInt32 count = faceInfo.indexCount - 1;
			ndInt32 start = faceInfo.indexStart;
			ndAssert (faceId == indexArray[start + count]);
			for (ndInt32 j = 0; j < count; ++j) 
			{
				ndInt32 index = indexArray[start + j];
				face[j] = points[index];
				faceIndex[j] = j;
			}
			tmpBuilder.AddFaceIndirect(&face[0].m_x, sizeof(ndVector), faceId, faceIndex, count);
		}
		tmpBuilder.FinalizeAndOptimize (faceId);

		ndInt32 faceIndexNumber = 0;
		for (ndInt32 i = 0; i < ndInt32(tmpBuilder.m_faceVertexCount.GetCount()); ++i)
		{
			ndInt32 indexCount = tmpBuilder.m_faceVertexCount[i] - 1;
			for (ndInt32 j = 0; j < indexCount; ++j) 
			{
				ndInt32 index = tmpBuilder.m_vertexIndex[faceIndexNumber + j];
				face[j] = tmpBuilder.m_vertexPoints[index];
				faceIndex[j] = j;
			}
			AddFaceIndirect(&face[0].m_x, sizeof(ndVector), faceId, faceIndex, indexCount);
		
			faceIndexNumber += (indexCount + 1); 
		}
	}
}

ndInt32 ndPolygonSoupBuilder::FilterFace (ndInt32 count, ndInt32* const pool)
{
	if (count == 3) 
	{
		ndBigVector p0 (m_vertexPoints[pool[2]]);
		for (ndInt32 i = 0; i < 3; ++i) 
		{
			ndBigVector p1 (m_vertexPoints[pool[i]]);
			ndBigVector edge (p1 - p0);
			ndFloat64 mag2 = edge.DotProduct(edge).GetScalar();
			if (mag2 < ndFloat32 (1.0e-6f)) 
			{
				count = 0;
			}
			p0 = p1;
		}

		if (count == 3) 
		{
			ndBigVector edge0 (m_vertexPoints[pool[2]] - m_vertexPoints[pool[0]]);
			ndBigVector edge1 (m_vertexPoints[pool[1]] - m_vertexPoints[pool[0]]);
			ndBigVector normal (edge0.CrossProduct(edge1));

			ndAssert(edge0.m_w == ndFloat32(0.0f));
			ndAssert(edge1.m_w == ndFloat32(0.0f));
			ndAssert (normal.m_w == ndFloat32 (0.0f));
			ndFloat64 mag2 = normal.DotProduct(normal).GetScalar();
			if (mag2 < ndFloat32 (1.0e-8f)) 
			{
				count = 0;
			}
		}
	} 
	else 
	{
		ndPolySoupFilterAllocator polyhedra;
		count = polyhedra.AddFilterFace (ndUnsigned32 (count), pool);

		if (!count) 
		{
			return 0;
		}

		ndEdge* edge = &polyhedra.GetRoot()->GetInfo();
		if (edge->m_incidentFace < 0) 
		{
			edge = edge->m_twin;
		}

		bool flag = true;
		while (flag) 
		{
			flag = false;
			if (count >= 3) 
			{
				ndEdge* ptr = edge;
				ndBigVector p0 (&m_vertexPoints[ptr->m_incidentVertex].m_x);
				p0 = p0 & ndBigVector::m_triplexMask;
				do 
				{
					ndBigVector p1 (&m_vertexPoints[ptr->m_next->m_incidentVertex].m_x);
					p1 = p1 & ndBigVector::m_triplexMask;
					ndBigVector e0 (p1 - p0);
					ndFloat64 mag2 = e0.DotProduct(e0).GetScalar();
					if (mag2 < ndFloat32 (1.0e-6f)) 
					{
						count --;
						flag = true;
						edge = ptr->m_next;
						ptr->m_prev->m_next = ptr->m_next;
						ptr->m_next->m_prev = ptr->m_prev;
						ptr->m_twin->m_next->m_prev = ptr->m_twin->m_prev;
						ptr->m_twin->m_prev->m_next = ptr->m_twin->m_next;
						break;
					}
					p0 = p1;
					ptr = ptr->m_next;
				} while (ptr != edge);
			}
		}
		if (count >= 3) 
		{
			flag = true;
			ndBigVector normal (polyhedra.FaceNormal (edge, &m_vertexPoints[0].m_x, sizeof (ndBigVector)));
			ndAssert (normal.m_w == ndFloat32 (0.0f));

			ndAssert (normal.DotProduct(normal).GetScalar() > ndFloat32 (1.0e-10f)); 
			normal = normal.Scale (ndFloat64 (1.0f) / sqrt (normal.DotProduct(normal).GetScalar() + ndFloat32 (1.0e-24f)));

			while (flag) 
			{
				flag = false;
				if (count >= 3) 
				{
					ndEdge* ptr = edge;

					ndBigVector p0 (&m_vertexPoints[ptr->m_prev->m_incidentVertex].m_x);
					ndBigVector p1 (&m_vertexPoints[ptr->m_incidentVertex].m_x);

					p0 = p0 & ndBigVector::m_triplexMask;
					p1 = p1 & ndBigVector::m_triplexMask;
					ndBigVector e0 (p1 - p0);
					e0 = e0.Scale (ndFloat64 (1.0f) / sqrt (e0.DotProduct(e0).GetScalar() + ndFloat32(1.0e-24f)));
					do 
					{
						ndBigVector p2 (&m_vertexPoints[ptr->m_next->m_incidentVertex].m_x);
						p2 = p2 & ndBigVector::m_triplexMask;
						ndBigVector e1 (p2 - p1);

						e1 = e1.Scale (ndFloat64 (1.0f) / sqrt (e1.DotProduct(e1).GetScalar() + ndFloat32(1.0e-24f)));
						ndFloat64 mag2 = e1.DotProduct(e0).GetScalar();
						if (mag2 > ndFloat32 (0.9999f)) 
						{
							count --;
							flag = true;
							edge = ptr->m_next;
							ptr->m_prev->m_next = ptr->m_next;
							ptr->m_next->m_prev = ptr->m_prev;
							ptr->m_twin->m_next->m_prev = ptr->m_twin->m_prev;
							ptr->m_twin->m_prev->m_next = ptr->m_twin->m_next;
							break;
						}

						ndBigVector n (e0.CrossProduct(e1));
						ndAssert (n.m_w == ndFloat32 (0.0f));
						mag2 = n.DotProduct(normal).GetScalar();
						if (mag2 < ndFloat32 (1.0e-5f)) 
						{
							count --;
							flag = true;
							edge = ptr->m_next;
							ptr->m_prev->m_next = ptr->m_next;
							ptr->m_next->m_prev = ptr->m_prev;
							ptr->m_twin->m_next->m_prev = ptr->m_twin->m_prev;
							ptr->m_twin->m_prev->m_next = ptr->m_twin->m_next;
							break;
						}

						e0 = e1;
						p1 = p2;
						ptr = ptr->m_next;
					} while (ptr != edge);
				}
			}
		}

		ndEdge* first = edge;
		if (count >= 3) 
		{
			ndFloat64 best = ndFloat32 (2.0f);
			ndEdge* ptr = edge;

			ndBigVector p0 (&m_vertexPoints[ptr->m_incidentVertex].m_x);
			ndBigVector p1 (&m_vertexPoints[ptr->m_next->m_incidentVertex].m_x);
			p0 = p0 & ndBigVector::m_triplexMask;
			p1 = p1 & ndBigVector::m_triplexMask;
			ndBigVector e0 (p1 - p0);
			e0 = e0.Scale (ndFloat64 (1.0f) / sqrt (e0.DotProduct(e0).GetScalar() + ndFloat32(1.0e-24f)));
			do 
			{
				ndBigVector p2 (&m_vertexPoints[ptr->m_next->m_next->m_incidentVertex].m_x);
				p2 = p2 & ndBigVector::m_triplexMask;
				ndBigVector e1 (p2 - p1);

				e1 = e1.Scale (ndFloat64 (1.0f) / sqrt (e1.DotProduct(e1).GetScalar() + ndFloat32(1.0e-24f)));
				ndFloat64 mag2 = fabs (e1.DotProduct(e0).GetScalar());
				if (mag2 < best) 
				{
					best = mag2;
					first = ptr;
				}

				e0 = e1;
				p1 = p2;
				ptr = ptr->m_next;
			} while (ptr != edge);

			count = 0;
			ptr = first;
			do 
			{
				pool[count] = ptr->m_incidentVertex;
				count ++;
				ptr = ptr->m_next;
			} while (ptr != first);
		}

	#ifdef _DEBUG
		if (count >= 3) 
		{
			ndInt32 j0 = count - 2;  
			ndInt32 j1 = count - 1;  
			ndBigVector normal (polyhedra.FaceNormal (edge, &m_vertexPoints[0].m_x, sizeof (ndBigVector)));
			ndAssert (normal.m_w == ndFloat32 (0.0f));
			for (ndInt32 j2 = 0; j2 < count; j2 ++) 
			{ 
				ndBigVector p0 (&m_vertexPoints[pool[j0]].m_x);
				ndBigVector p1 (&m_vertexPoints[pool[j1]].m_x);
				ndBigVector p2 (&m_vertexPoints[pool[j2]].m_x);
				p0 = p0 & ndBigVector::m_triplexMask;
				p1 = p1 & ndBigVector::m_triplexMask;
				p2 = p2 & ndBigVector::m_triplexMask;

				ndBigVector e0 ((p0 - p1));
				ndBigVector e1 ((p2 - p1));

				ndBigVector n (e1.CrossProduct(e0));
				ndAssert (n.DotProduct(normal).GetScalar() > ndFloat32 (0.0f));
				j0 = j1;
				j1 = j2;
			}
		}
	#endif
	}

	return (count >= 3) ? count : 0;
}

ndInt32 ndPolygonSoupBuilder::AddConvexFace (ndInt32 count, ndInt32* const pool, ndInt32* const facesArray)
{
	ndPolySoupFilterAllocator polyhedra;

	count = polyhedra.AddFilterFace(ndUnsigned32 (count), pool);

	ndEdge* edge = &polyhedra.GetRoot()->GetInfo();
	if (edge->m_incidentFace < 0) 
	{
		edge = edge->m_twin;
	}
	
	ndInt32 isconvex = 1;
	ndInt32 facesCount = 0;

	ndInt32 flag = 1;
	while (flag) 
	{
		flag = 0;
		if (count >= 3) 
		{
			ndEdge* ptr = edge;

			ndBigVector p0 (&m_vertexPoints[ptr->m_incidentVertex].m_x);
			do 
			{
				ndBigVector p1 (&m_vertexPoints[ptr->m_next->m_incidentVertex].m_x);
				ndBigVector e0 (p1 - p0);
				ndFloat64 mag2 = e0.DotProduct(e0).GetScalar();
				if (mag2 < ndFloat32 (1.0e-6f)) 
				{
					count --;
					flag = 1;
					edge = ptr->m_next;
					ptr->m_prev->m_next = ptr->m_next;
					ptr->m_next->m_prev = ptr->m_prev;
					ptr->m_twin->m_next->m_prev = ptr->m_twin->m_prev;
					ptr->m_twin->m_prev->m_next = ptr->m_twin->m_next;
					break;
				}
				p0 = p1;
				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}
	if (count >= 3) 
	{
		flag = 1;

		while (flag) 
		{
			flag = 0;
			if (count >= 3) 
			{
				ndEdge* ptr = edge;

				ndBigVector p0 (&m_vertexPoints[ptr->m_prev->m_incidentVertex].m_x);
				ndBigVector p1 (&m_vertexPoints[ptr->m_incidentVertex].m_x);
				ndBigVector e0 (p1 - p0);
				e0 = e0.Scale (ndFloat64 (1.0f) / sqrt (e0.DotProduct(e0).GetScalar() + ndFloat32(1.0e-24f)));
				do 
				{
					ndBigVector p2 (&m_vertexPoints[ptr->m_next->m_incidentVertex].m_x);
					ndBigVector e1 (p2 - p1);

					e1 = e1.Scale (ndFloat64 (1.0f) / sqrt (e1.DotProduct(e1).GetScalar() + ndFloat32(1.0e-24f)));
					ndFloat64 mag2 = e1.DotProduct(e0).GetScalar();
					if (mag2 > ndFloat32 (0.9999f)) 
					{
						count --;
						flag = 1;
						edge = ptr->m_next;
						ptr->m_prev->m_next = ptr->m_next;
						ptr->m_next->m_prev = ptr->m_prev;
						ptr->m_twin->m_next->m_prev = ptr->m_twin->m_prev;
						ptr->m_twin->m_prev->m_next = ptr->m_twin->m_next;
						break;
					}

					e0 = e1;
					p1 = p2;
					ptr = ptr->m_next;
				} while (ptr != edge);
			}
		}

		ndBigVector normal (polyhedra.FaceNormal (edge, &m_vertexPoints[0].m_x, sizeof (ndBigVector)));
		ndFloat64 mag2 = normal.DotProduct(normal).GetScalar();
		if (mag2 < ndFloat32 (1.0e-8f)) 
		{
			return 0;
		}
		normal = normal.Scale (ndFloat64 (1.0f) / sqrt (mag2));

		if (count >= 3) 
		{
			ndEdge* ptr = edge;
			ndBigVector p0 (&m_vertexPoints[ptr->m_prev->m_incidentVertex].m_x);
			ndBigVector p1 (&m_vertexPoints[ptr->m_incidentVertex].m_x);
			ndBigVector e0 (p1 - p0);
			e0 = e0.Scale (ndFloat64 (1.0f) / sqrt (e0.DotProduct(e0).GetScalar() + ndFloat32(1.0e-24f)));
			do 
			{
				ndBigVector p2 (&m_vertexPoints[ptr->m_next->m_incidentVertex].m_x);
				ndBigVector e1 (p2 - p1);

				e1 = e1.Scale (ndFloat64 (1.0f) / sqrt (e1.DotProduct(e1).GetScalar() + ndFloat32(1.0e-24f)));

				ndBigVector n (e0.CrossProduct(e1));
				ndFloat64 magnitud2 = n.DotProduct(normal).GetScalar();
				if (magnitud2 < ndFloat32 (1.0e-5f)) 
				{
					isconvex = 0;
					break;
				}

				e0 = e1;
				p1 = p2;
				ptr = ptr->m_next;
			} while (ptr != edge);
		}
	}

	if (isconvex) 
	{
		ndEdge* const first = edge;
		if (count >= 3) 
		{
			count = 0;
			ndEdge* ptr = first;
			do 
			{
				pool[count] = ptr->m_incidentVertex;
				count ++;
				ptr = ptr->m_next;
			} while (ptr != first);
			facesArray[facesCount] = count;
			facesCount = 1;
		}
	} else {
		ndPolyhedra leftOver;
		ndPolyhedra polyhedra2;
		ndEdge* ptr = edge;
		count = 0;
		do 
		{
			pool[count] = ptr->m_incidentVertex;
			count ++;
			ptr = ptr->m_next;
		} while (ptr != edge);


		polyhedra2.BeginFace();
		polyhedra2.AddFace (count, pool);
		polyhedra2.EndFace();
		leftOver.BeginFace();
		polyhedra2.ConvexPartition (&m_vertexPoints[0].m_x, sizeof (ndBigVector), &leftOver);
		leftOver.EndFace();

#ifdef _DEBUG
		if (leftOver.GetCount()) 
		{
			ndTrace (("warning: %d faces with more that a one shared edge\n", leftOver.GetCount()));
			ndTrace (("         this mesh is not a manifold and may lead to collision malfunctions\n"));
		}
#endif

		ndInt32 mark = polyhedra2.IncLRU();
		ndInt32 index = 0;
		ndPolyhedra::Iterator iter (polyhedra2);
		for (iter.Begin(); iter; iter ++) 
		{
			ndEdge* const edge1 = &(*iter);
			if (edge1->m_incidentFace < 0) 
			{
				continue;
			}
			if (edge1->m_mark == mark) 
			{
				continue;
			}

			ptr = edge1;
			count = 0;
			do 
			{
				ptr->m_mark = mark;
				pool[index] = ptr->m_incidentVertex;
				index ++;
				count ++;
				ptr = ptr->m_next;
			} while (ptr != edge1);

			facesArray[facesCount] = count;
			facesCount ++;
		}
	}

	return facesCount;
}
