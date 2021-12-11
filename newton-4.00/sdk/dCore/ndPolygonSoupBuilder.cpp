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

//#include "ndMatrix.h"
//#include "dgMemory.h"
//#include "ndPolygonSoupBuilder.h"

#define DG_POINTS_RUN (512 * 1024)

class ndPolygonSoupBuilder::dgFaceInfo
{
	public:
	dInt32 indexCount;
	dInt32 indexStart;
};

class ndPolygonSoupBuilder::dgFaceBucket: public ndList<dgFaceInfo>
{
	public: 
	dgFaceBucket ()
		:ndList<dgFaceInfo>()
	{
	}
};

class ndPolygonSoupBuilder::dgFaceMap: public ndTree<dgFaceBucket, dInt32>
{
	public:
	dgFaceMap (ndPolygonSoupBuilder& builder)
		:ndTree<dgFaceBucket, dInt32>()
	{
		dInt32 polygonIndex = 0;
		//dInt32 faceCount = builder.m_faceCount;
		dInt32 faceCount = builder.m_faceVertexCount.GetCount();
		const dInt32* const faceVertexCounts = &builder.m_faceVertexCount[0];
		const dInt32* const faceVertexIndex = &builder.m_vertexIndex[0];
		for (dInt32 i = 0; i < faceCount; i ++) 
		{
			dInt32 count = faceVertexCounts[i];
			dInt32 attribute = faceVertexIndex[polygonIndex + count - 1];
			
			ndNode* node = Find(attribute);
			if (!node) 
			{
				dgFaceBucket tmp;
				node = Insert(tmp, attribute);
			}

			dgFaceBucket& bucket = node->GetInfo();
			dgFaceInfo& face = bucket.Append()->GetInfo();
			face.indexCount = count;
			face.indexStart = polygonIndex;
			polygonIndex += count;
		}
	}
};

class ndPolygonSoupBuilder::dgPolySoupFilterAllocator: public ndPolyhedra
{
	public: 
	dgPolySoupFilterAllocator ()
		:ndPolyhedra ()
	{
	}

	~dgPolySoupFilterAllocator ()
	{
	}

	dInt32 AddFilterFace (dUnsigned32 count, dInt32* const pool)
	{
		BeginFace();
		dAssert (count);
		bool reduction = true;
		while (reduction && !AddFace (dInt32 (count), pool)) 
		{
			reduction = false;
			if (count >3) 
			{
				for (dUnsigned32 i = 0; i < count; i ++) 
				{
					for (dUnsigned32 j = i + 1; j < count; j ++) 
					{
						if (pool[j] == pool[i]) 
						{
							for (i = j; i < count - 1; i ++) 
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
		return reduction ? dInt32 (count) : 0;
	}
};

ndPolygonSoupBuilder::ndPolygonSoupBuilder ()
	:m_faceVertexCount()
	,m_vertexIndex()
	,m_normalIndex()
	,m_vertexPoints()
	,m_normalPoints()
{
	m_run = DG_POINTS_RUN;
}

ndPolygonSoupBuilder::ndPolygonSoupBuilder (const ndPolygonSoupBuilder& source)
	:m_faceVertexCount(source.m_faceVertexCount.GetCount())
	,m_vertexIndex(source.m_vertexIndex.GetCount())
	,m_normalIndex()
	,m_vertexPoints(source.m_vertexPoints.GetCount())
	,m_normalPoints()
{
	m_run = DG_POINTS_RUN;
	m_faceVertexCount.SetCount(source.m_faceVertexCount.GetCount());
	m_vertexIndex.SetCount(source.m_vertexIndex.GetCount());
	m_vertexPoints.SetCount(source.m_vertexPoints.GetCount());
	
	memcpy (&m_vertexIndex[0], &source.m_vertexIndex[0], sizeof (dInt32) * source.m_vertexIndex.GetCount());
	memcpy (&m_faceVertexCount[0], &source.m_faceVertexCount[0], sizeof (dInt32) * source.m_faceVertexCount.GetCount());
	memcpy (&m_vertexPoints[0], &source.m_vertexPoints[0], sizeof (ndBigVector) * source.m_vertexPoints.GetCount());

	if (m_normalPoints.GetCount())
	{
		m_normalIndex.Resize(source.m_normalIndex.GetCount());
		m_normalPoints.Resize(source.m_normalPoints.GetCount());
		m_normalIndex.SetCount(source.m_normalIndex.GetCount());
		m_normalPoints.SetCount(source.m_normalPoints.GetCount());

		memcpy (&m_normalIndex[0], &source.m_normalIndex[0], sizeof (dInt32) * source.m_normalIndex.GetCount());
		memcpy (&m_normalPoints[0], &source.m_normalPoints[0], sizeof (ndBigVector) * source.m_normalPoints.GetCount());
	}
}

ndPolygonSoupBuilder::~ndPolygonSoupBuilder ()
{
}

void ndPolygonSoupBuilder::Begin()
{
	m_run = DG_POINTS_RUN;
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

	//dInt32 faceCount = 0;
	fprintf(file, "element vertex %d\n", m_vertexPoints.GetCount());
	fprintf(file, "property float x\n");
	fprintf(file, "property float y\n");
	fprintf(file, "property float z\n");
	fprintf(file, "element face %d\n", m_faceVertexCount.GetCount());
	fprintf(file, "property list uchar int vertex_index\n");
	fprintf(file, "end_header\n");

	for (dInt32 i = 0; i < m_vertexPoints.GetCount(); i ++)
	{
		const ndBigVector& point = m_vertexPoints[i];
		fprintf(file, "%f %f %f\n", point.m_x, point.m_y, point.m_z);
	}

	dInt32 index = 0;
	for (dInt32 i = 0; i < m_faceVertexCount.GetCount(); i ++)
	{
		dInt32 count = m_faceVertexCount[i];
		fprintf(file, "%d", count - 1);
		for (dInt32 j = 0; j < count - 1; j++) 
		{
			fprintf(file, " %d", m_vertexIndex[index + j]);
		}
		index += count;
		fprintf(file, "\n");
	}
	fclose(file);
}

void ndPolygonSoupBuilder::AddFaceIndirect(const dFloat32* const vertex, dInt32 strideInBytes, dInt32 faceId, const dInt32* const indexArray, dInt32 indexCount)
{
	dInt32 faces[32];
	dInt32 pool[512];

	const dInt32 vertexCount = m_vertexPoints.GetCount();
	const dInt32 stride = dInt32 (strideInBytes / sizeof(dFloat32));
	for (dInt32 i = 0; i < indexCount; i++)
	{
		pool[i] = i + vertexCount;
		const dInt32 j = indexArray[i] * stride;
		ndBigVector point(vertex[j + 0], vertex[j + 1], vertex[j + 2], dFloat32(0.0f));
		m_vertexPoints.PushBack (point);
	}

	dInt32 convexFaces = 0;
	if (indexCount == 3)
	{
		convexFaces = 1;
		ndBigVector p0(m_vertexPoints[pool[2]]);
		for (dInt32 j = 0; j < 3; j++)
		{
			ndBigVector p1(m_vertexPoints[pool[j]]);
			ndBigVector edge(p1 - p0);
			dFloat64 mag2 = edge.DotProduct(edge).GetScalar();
			if (mag2 < dFloat32(1.0e-6f))
			{
				convexFaces = 0;
			}
			p0 = p1;
		}

		if (convexFaces)
		{
			ndBigVector edge0(m_vertexPoints[pool[2]] - m_vertexPoints[pool[0]]);
			ndBigVector edge1(m_vertexPoints[pool[1]] - m_vertexPoints[pool[0]]);
			dAssert(edge0.m_w == dFloat32(0.0f));
			dAssert(edge1.m_w == dFloat32(0.0f));
			ndBigVector normal(edge0.CrossProduct(edge1));
			dFloat64 mag2 = normal.DotProduct(normal).GetScalar();
			if (mag2 < dFloat32(1.0e-8f))
			{
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

	dInt32 indexAcc = 0;
	for (dInt32 j = 0; j < convexFaces; j++)
	{
		dInt32 count1 = faces[j];
		for (dInt32 m = 0; m < count1; m++)
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

void ndPolygonSoupBuilder::AddFace(const dFloat32* const vertex, dInt32 strideInBytes, dInt32 vertexCount, const dInt32 faceId)
{
	dInt32 indexArray[1024];
	dAssert(vertexCount < dInt32 (sizeof(indexArray)/sizeof (indexArray[0])));
	for (dInt32 i = 0; i < vertexCount; i++)
	{
		indexArray[i] = i;
	}
	AddFaceIndirect(vertex, strideInBytes, faceId, indexArray, vertexCount);
}

void ndPolygonSoupBuilder::PackArray()
{
	dAssert(0);
	ndStack<dInt32> indexMapPool (m_vertexPoints.GetCount());
	dInt32* const indexMap = &indexMapPool[0];
	dInt32 vertexCount = dVertexListToIndexList(&m_vertexPoints[0].m_x, sizeof (ndBigVector), 3, m_vertexPoints.GetCount(), &indexMap[0], dFloat32 (1.0e-6f));

	dInt32 k = 0;
	for (dInt32 i = 0; i < m_faceVertexCount.GetCount(); i ++)
	{
		dInt32 count = m_faceVertexCount[i] - 1;
		for (dInt32 j = 0; j < count; j ++) 
		{
			dInt32 index = m_vertexIndex[k];
			index = indexMap[index];
			m_vertexIndex[k] = index;
			k ++;
		}
		k ++;
	}

	m_vertexPoints.Resize(vertexCount);
	m_vertexPoints.SetCount(vertexCount);
	m_run = DG_POINTS_RUN;
}

void ndPolygonSoupBuilder::Finalize()
{
	const dInt32 faceCount = m_faceVertexCount.GetCount();
	if (faceCount)
	{
		//ndStack<dInt32> indexMapPool (m_indexCount + m_vertexCount);
		ndStack<dInt32> indexMapPool(m_vertexIndex.GetCount());

		dInt32* const indexMap = &indexMapPool[0];
		dInt32 vertexCount = dVertexListToIndexList(&m_vertexPoints[0].m_x, sizeof (ndBigVector), 3, m_vertexPoints.GetCount(), &indexMap[0], dFloat32 (1.0e-4f));
		dAssert(vertexCount <= m_vertexPoints.GetCount());
		m_vertexPoints.SetCount(vertexCount);

		dInt32 k = 0;
		for (dInt32 i = 0; i < faceCount; i ++) 
		{
			dInt32 count = m_faceVertexCount[i] - 1;
			for (dInt32 j = 0; j < count; j ++) 
			{
				dInt32 index = m_vertexIndex[k];
				index = indexMap[index];
				m_vertexIndex[k] = index;
				k ++;
			}
			k ++;
		}
		OptimizeByIndividualFaces();
	}
}

void ndPolygonSoupBuilder::FinalizeAndOptimize(dInt32 id)
{
	Finalize();
	ndPolyhedra polyhedra;
	ndPolygonSoupBuilder source(*this);
	ndPolygonSoupBuilder leftOver;
	dInt32 tmpIndexPool[1024];
	ndVector tmpVertexPool[1024];

	Begin();
	leftOver.Begin();
	polyhedra.BeginFace ();
	dInt32 attribute = id;
	dInt32 faceIndexNumber = 0;
	for (dInt32 i = 0; i < source.m_faceVertexCount.GetCount(); i ++)
	{
		dInt32 indexCount = source.m_faceVertexCount[i];
		dAssert (indexCount < 1024);

		ndEdge* const face = polyhedra.AddFace(indexCount - 1, &source.m_vertexIndex[faceIndexNumber]);
		if (!face) 
		{
			for (dInt32 j = 0; j < indexCount - 1; j ++) 
			{
				dInt32 index = source.m_vertexIndex[faceIndexNumber + j];
				tmpVertexPool[j] = source.m_vertexPoints[index];
				tmpIndexPool[j] = j;
			}
			//dInt32 faceArray = indexCount - 1;
			//leftOver.AddMesh (&tmpVertexPool[0].m_x, indexCount, sizeof (tmpVertexPool[0]), 1, &faceArray, tmpIndexPool, &attribute, dGetIdentityMatrix());
			leftOver.AddFaceIndirect(&tmpVertexPool[0].m_x, sizeof (ndVector), attribute, tmpIndexPool, indexCount - 1);
		} 
		else 
		{
			// set the attribute
			ndEdge* ptr = face;
			do 
			{
				ptr->m_userData = dUnsigned64 (attribute);
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

	dInt32 mark = polyhedra.IncLRU();
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
		dInt32 indexCount = 0;
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
		dInt32 indexCount = 0;
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
	for (dInt32 i = 0; i < leftOver.m_faceVertexCount.GetCount(); i ++)
	{
		dInt32 indexCount = leftOver.m_faceVertexCount[i] - 1;
		for (dInt32 j = 0; j < indexCount; j ++) 
		{
			dInt32 index = leftOver.m_vertexIndex[faceIndexNumber + j];
			tmpVertexPool[j] = leftOver.m_vertexPoints[index];
			tmpIndexPool[j] = j;
		}
		//dInt32 faceArray = indexCount;
		//AddMesh (&tmpVertexPool[0].m_x, indexCount, sizeof (tmpVertexPool[0]), 1, &faceArray, tmpIndexPool, &attribute, dGetIdentityMatrix());
		AddFaceIndirect(&tmpVertexPool[0].m_x, sizeof(ndVector), attribute, tmpIndexPool, indexCount);

		faceIndexNumber += (indexCount + 1); 
	}

	Finalize();

}

void ndPolygonSoupBuilder::OptimizeByIndividualFaces()
{
	dInt32* const faceArray = &m_faceVertexCount[0];
	dInt32* const indexArray = &m_vertexIndex[0];

	dInt32* const oldFaceArray = &m_faceVertexCount[0];
	dInt32* const oldIndexArray = &m_vertexIndex[0];

	dInt32 polygonIndex = 0;
	dInt32 newFaceCount = 0;
	dInt32 newIndexCount = 0;
	for (dInt32 i = 0; i < m_faceVertexCount.GetCount(); i ++)
	{
		dInt32 oldCount = oldFaceArray[i];
		dInt32 count = FilterFace (oldCount - 1, &oldIndexArray[polygonIndex]);
		if (count) 
		{
			faceArray[newFaceCount] = count + 1;
			for (dInt32 j = 0; j < count; j ++) 
			{
				indexArray[newIndexCount + j] = oldIndexArray[polygonIndex + j];
			}
			indexArray[newIndexCount + count] = oldIndexArray[polygonIndex + oldCount - 1];
			newFaceCount ++;
			newIndexCount += (count + 1);
		}
		polygonIndex += oldCount;
	}
	dAssert (polygonIndex == m_vertexIndex.GetCount());

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
		dgFaceMap faceMap (copy);

		Begin();
		dgFaceMap::Iterator iter (faceMap);
		for (iter.Begin(); iter; iter ++) 
		{
			const dgFaceBucket& bucket = iter.GetNode()->GetInfo();
			Optimize(iter.GetNode()->GetKey(), bucket, copy);
		}
	}
	Finalize();

	// build the normal array and adjacency array
	dInt32 indexCount = 0;
	const dInt32 faceCount = m_faceVertexCount.GetCount();
	if (faceCount)
	{
		// calculate all face the normals
		m_normalPoints.Resize(faceCount);
		m_normalPoints.SetCount(faceCount);
		for (dInt32 i = 0; i < faceCount; i++)
		{
			dInt32 faceIndexCount = m_faceVertexCount[i];

			const dInt32* const ptr = &m_vertexIndex[indexCount];
			ndBigVector v0(&m_vertexPoints[ptr[0]].m_x);
			ndBigVector v1(&m_vertexPoints[ptr[1]].m_x);
			ndBigVector e0(v1 - v0);
			ndBigVector normal0(ndBigVector::m_zero);
			for (dInt32 j = 2; j < faceIndexCount - 1; j++)
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
			m_normalPoints[i].m_w = dFloat32(0.0f);
			indexCount += faceIndexCount;
		}
		// compress normals array
		//m_normalIndex[m_faceCount] = 0;

		m_normalIndex.Resize(faceCount);;
		m_normalIndex.SetCount(faceCount);
		dInt32 normalCount = dVertexListToIndexList(&m_normalPoints[0].m_x, sizeof(ndBigVector), 3, faceCount, &m_normalIndex[0], dFloat32(1.0e-6f));
		dAssert(normalCount <= m_normalPoints.GetCount());
		m_normalPoints.SetCount(normalCount);
	}
}

void ndPolygonSoupBuilder::Optimize(dInt32 faceId, const dgFaceBucket& faceBucket, const ndPolygonSoupBuilder& source)
{
	#define DG_MESH_PARTITION_SIZE (1024 * 4)

	const dInt32* const indexArray = &source.m_vertexIndex[0];
	const ndBigVector* const points = &source.m_vertexPoints[0];

	ndVector face[256];
	dInt32 faceIndex[256];
	if (faceBucket.GetCount() >= DG_MESH_PARTITION_SIZE) 
	{
		ndStack<dgFaceBucket::ndNode*> array(faceBucket.GetCount());
		dInt32 count = 0;
		for (dgFaceBucket::ndNode* node = faceBucket.GetFirst(); node; node = node->GetNext()) 
		{
			array[count] = node;
			count ++;
		}

		dInt32 stack = 1;
		dInt32 segments[32][2];
			
		segments[0][0] = 0;
		segments[0][1] = count;
	
		while (stack) 
		{
			stack --;
			dInt32 faceStart = segments[stack][0];
			dInt32 faceCount = segments[stack][1];

			if (faceCount <= DG_MESH_PARTITION_SIZE) 
			{
				ndPolygonSoupBuilder tmpBuilder;
				for (dInt32 i = 0; i < faceCount; i ++) 
				{
					const dgFaceInfo& faceInfo = array[faceStart + i]->GetInfo();

					dInt32 count1 = faceInfo.indexCount - 1;
					dInt32 start1 = faceInfo.indexStart;
					dAssert (faceId == indexArray[start1 + count1]);
					for (dInt32 j = 0; j < count1; j ++) 
					{
						dInt32 index = indexArray[start1 + j];
						face[j] = points[index];
						faceIndex[j] = j;
					}
					//dInt32 faceIndexCount = count1;
					//tmpBuilder.AddMesh (&face[0].m_x, count1, sizeof (ndVector), 1, &faceIndexCount, &faceIndex[0], &faceId, dGetIdentityMatrix()); 
					tmpBuilder.AddFaceIndirect(&face[0].m_x, sizeof(ndVector), faceId, faceIndex, count1);
				}
				tmpBuilder.FinalizeAndOptimize (faceId);

				dInt32 faceIndexNumber = 0;
				for (dInt32 i = 0; i < tmpBuilder.m_faceVertexCount.GetCount(); i ++)
				{
					dInt32 indexCount = tmpBuilder.m_faceVertexCount[i] - 1;
					for (dInt32 j = 0; j < indexCount; j ++) 
					{
						dInt32 index = tmpBuilder.m_vertexIndex[faceIndexNumber + j];
						face[j] = tmpBuilder.m_vertexPoints[index];
						faceIndex[j] = j;
					}
					//dInt32 faceArray = indexCount;
					//AddMesh (&face[0].m_x, indexCount, sizeof (ndVector), 1, &faceArray, faceIndex, &faceId, dGetIdentityMatrix());
					AddFaceIndirect(&face[0].m_x, sizeof(ndVector), faceId, faceIndex, indexCount);
					faceIndexNumber += (indexCount + 1); 
				}
			} 
			else 
			{
				ndBigVector median (ndBigVector::m_zero);
				ndBigVector varian (ndBigVector::m_zero);
				for (dInt32 i = 0; i < faceCount; i ++) 
				{
					const dgFaceInfo& faceInfo = array[faceStart + i]->GetInfo();
					dInt32 count1 = faceInfo.indexCount - 1;
					dInt32 start1 = faceInfo.indexStart;
					ndBigVector p0 (dFloat32 ( 1.0e10f), dFloat32 ( 1.0e10f), dFloat32 ( 1.0e10f), dFloat32 (0.0f));
					ndBigVector p1 (dFloat32 (-1.0e10f), dFloat32 (-1.0e10f), dFloat32 (-1.0e10f), dFloat32 (0.0f));
					for (dInt32 j = 0; j < count1; j ++) 
					{
						dInt32 index = indexArray[start1 + j];
						const ndBigVector& p = points[index];
						dAssert(p.m_w == dFloat32(0.0f));
						p0 = p0.GetMin(p);
						p1 = p1.GetMax(p);
					}
					ndBigVector p ((p0 + p1).Scale (0.5f));
					median += p;
					varian += p * p;
				}

				varian = varian.Scale (dFloat32 (faceCount)) - median * median;

				dInt32 axis = 0;
				dFloat32 maxVarian = dFloat32 (-1.0e10f);
				for (dInt32 i = 0; i < 3; i ++) 
				{
					if (varian[i] > maxVarian) 
					{
						axis = i;
						maxVarian = dFloat32 (varian[i]);
					}
				}
				ndBigVector center = median.Scale (dFloat32 (1.0f) / dFloat32 (faceCount));
				dFloat64 axisVal = center[axis];

				dInt32 leftCount = 0;
				dInt32 lastFace = faceCount;

				for (dInt32 i = 0; i < lastFace; i ++) 
				{
					dInt32 side = 0;
					const dgFaceInfo& faceInfo = array[faceStart + i]->GetInfo();

					dInt32 start1 = faceInfo.indexStart;
					dInt32 count1 = faceInfo.indexCount - 1;
					for (dInt32 j = 0; j < count1; j ++) 
					{
						dInt32 index = indexArray[start1 + j];
						const ndBigVector& p = points[index];
						if (p[axis] > axisVal) 
						{
							side = 1;
							break;
						}
					}

					if (side) 
					{
						dSwap (array[faceStart + i], array[faceStart + lastFace - 1]);
						lastFace --;
						i --;
					} 
					else 
					{
						leftCount ++;
					}
				}
				dAssert (leftCount);
				dAssert (leftCount < faceCount);

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
		for (dgFaceBucket::ndNode* node = faceBucket.GetFirst(); node; node = node->GetNext()) 
		{
			const dgFaceInfo& faceInfo = node->GetInfo();

			dInt32 count = faceInfo.indexCount - 1;
			dInt32 start = faceInfo.indexStart;
			dAssert (faceId == indexArray[start + count]);
			for (dInt32 j = 0; j < count; j ++) 
			{
				dInt32 index = indexArray[start + j];
				face[j] = points[index];
				faceIndex[j] = j;
			}
			//dInt32 faceIndexCount = count;
			//tmpBuilder.AddMesh (&face[0].m_x, count, sizeof (ndVector), 1, &faceIndexCount, &faceIndex[0], &faceId, dGetIdentityMatrix()); 
			tmpBuilder.AddFaceIndirect(&face[0].m_x, sizeof(ndVector), faceId, faceIndex, count);
		}
		tmpBuilder.FinalizeAndOptimize (faceId);

		dInt32 faceIndexNumber = 0;
		for (dInt32 i = 0; i < tmpBuilder.m_faceVertexCount.GetCount(); i ++)
		{
			dInt32 indexCount = tmpBuilder.m_faceVertexCount[i] - 1;
			for (dInt32 j = 0; j < indexCount; j ++) 
			{
				dInt32 index = tmpBuilder.m_vertexIndex[faceIndexNumber + j];
				face[j] = tmpBuilder.m_vertexPoints[index];
				faceIndex[j] = j;
			}
			//dInt32 faceArray = indexCount;
			//AddMesh (&face[0].m_x, indexCount, sizeof (ndVector), 1, &faceArray, faceIndex, &faceId, dGetIdentityMatrix());
			AddFaceIndirect(&face[0].m_x, sizeof(ndVector), faceId, faceIndex, indexCount);
		
			faceIndexNumber += (indexCount + 1); 
		}
	}
}

dInt32 ndPolygonSoupBuilder::FilterFace (dInt32 count, dInt32* const pool)
{
	if (count == 3) 
	{
		ndBigVector p0 (m_vertexPoints[pool[2]]);
		for (dInt32 i = 0; i < 3; i ++) 
		{
			ndBigVector p1 (m_vertexPoints[pool[i]]);
			ndBigVector edge (p1 - p0);
			dFloat64 mag2 = edge.DotProduct(edge).GetScalar();
			if (mag2 < dFloat32 (1.0e-6f)) 
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

			dAssert(edge0.m_w == dFloat32(0.0f));
			dAssert(edge1.m_w == dFloat32(0.0f));
			dAssert (normal.m_w == dFloat32 (0.0f));
			dFloat64 mag2 = normal.DotProduct(normal).GetScalar();
			if (mag2 < dFloat32 (1.0e-8f)) 
			{
				count = 0;
			}
		}
	} 
	else 
	{
		dgPolySoupFilterAllocator polyhedra;
		count = polyhedra.AddFilterFace (dUnsigned32 (count), pool);

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
					dFloat64 mag2 = e0.DotProduct(e0).GetScalar();
					if (mag2 < dFloat32 (1.0e-6f)) 
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
			dAssert (normal.m_w == dFloat32 (0.0f));

			dAssert (normal.DotProduct(normal).GetScalar() > dFloat32 (1.0e-10f)); 
			normal = normal.Scale (dFloat64 (1.0f) / sqrt (normal.DotProduct(normal).GetScalar() + dFloat32 (1.0e-24f)));

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
					e0 = e0.Scale (dFloat64 (1.0f) / sqrt (e0.DotProduct(e0).GetScalar() + dFloat32(1.0e-24f)));
					do 
					{
						ndBigVector p2 (&m_vertexPoints[ptr->m_next->m_incidentVertex].m_x);
						p2 = p2 & ndBigVector::m_triplexMask;
						ndBigVector e1 (p2 - p1);

						e1 = e1.Scale (dFloat64 (1.0f) / sqrt (e1.DotProduct(e1).GetScalar() + dFloat32(1.0e-24f)));
						dFloat64 mag2 = e1.DotProduct(e0).GetScalar();
						if (mag2 > dFloat32 (0.9999f)) 
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
						dAssert (n.m_w == dFloat32 (0.0f));
						mag2 = n.DotProduct(normal).GetScalar();
						if (mag2 < dFloat32 (1.0e-5f)) 
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
			dFloat64 best = dFloat32 (2.0f);
			ndEdge* ptr = edge;

			ndBigVector p0 (&m_vertexPoints[ptr->m_incidentVertex].m_x);
			ndBigVector p1 (&m_vertexPoints[ptr->m_next->m_incidentVertex].m_x);
			p0 = p0 & ndBigVector::m_triplexMask;
			p1 = p1 & ndBigVector::m_triplexMask;
			ndBigVector e0 (p1 - p0);
			e0 = e0.Scale (dFloat64 (1.0f) / sqrt (e0.DotProduct(e0).GetScalar() + dFloat32(1.0e-24f)));
			do 
			{
				ndBigVector p2 (&m_vertexPoints[ptr->m_next->m_next->m_incidentVertex].m_x);
				p2 = p2 & ndBigVector::m_triplexMask;
				ndBigVector e1 (p2 - p1);

				e1 = e1.Scale (dFloat64 (1.0f) / sqrt (e1.DotProduct(e1).GetScalar() + dFloat32(1.0e-24f)));
				dFloat64 mag2 = fabs (e1.DotProduct(e0).GetScalar());
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
			dInt32 j0 = count - 2;  
			dInt32 j1 = count - 1;  
			ndBigVector normal (polyhedra.FaceNormal (edge, &m_vertexPoints[0].m_x, sizeof (ndBigVector)));
			dAssert (normal.m_w == dFloat32 (0.0f));
			for (dInt32 j2 = 0; j2 < count; j2 ++) 
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
				dAssert (n.DotProduct(normal).GetScalar() > dFloat32 (0.0f));
				j0 = j1;
				j1 = j2;
			}
		}
	#endif
	}

	return (count >= 3) ? count : 0;
}

dInt32 ndPolygonSoupBuilder::AddConvexFace (dInt32 count, dInt32* const pool, dInt32* const facesArray)
{
	dgPolySoupFilterAllocator polyhedra;

	count = polyhedra.AddFilterFace(dUnsigned32 (count), pool);

	ndEdge* edge = &polyhedra.GetRoot()->GetInfo();
	if (edge->m_incidentFace < 0) 
	{
		edge = edge->m_twin;
	}
	
	dInt32 isconvex = 1;
	dInt32 facesCount = 0;

	dInt32 flag = 1;
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
				dFloat64 mag2 = e0.DotProduct(e0).GetScalar();
				if (mag2 < dFloat32 (1.0e-6f)) 
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
				e0 = e0.Scale (dFloat64 (1.0f) / sqrt (e0.DotProduct(e0).GetScalar() + dFloat32(1.0e-24f)));
				do 
				{
					ndBigVector p2 (&m_vertexPoints[ptr->m_next->m_incidentVertex].m_x);
					ndBigVector e1 (p2 - p1);

					e1 = e1.Scale (dFloat64 (1.0f) / sqrt (e1.DotProduct(e1).GetScalar() + dFloat32(1.0e-24f)));
					dFloat64 mag2 = e1.DotProduct(e0).GetScalar();
					if (mag2 > dFloat32 (0.9999f)) 
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
		dFloat64 mag2 = normal.DotProduct(normal).GetScalar();
		if (mag2 < dFloat32 (1.0e-8f)) 
		{
			return 0;
		}
		normal = normal.Scale (dFloat64 (1.0f) / sqrt (mag2));

		if (count >= 3) 
		{
			ndEdge* ptr = edge;
			ndBigVector p0 (&m_vertexPoints[ptr->m_prev->m_incidentVertex].m_x);
			ndBigVector p1 (&m_vertexPoints[ptr->m_incidentVertex].m_x);
			ndBigVector e0 (p1 - p0);
			e0 = e0.Scale (dFloat64 (1.0f) / sqrt (e0.DotProduct(e0).GetScalar() + dFloat32(1.0e-24f)));
			do 
			{
				ndBigVector p2 (&m_vertexPoints[ptr->m_next->m_incidentVertex].m_x);
				ndBigVector e1 (p2 - p1);

				e1 = e1.Scale (dFloat64 (1.0f) / sqrt (e1.DotProduct(e1).GetScalar() + dFloat32(1.0e-24f)));

				ndBigVector n (e0.CrossProduct(e1));
				dFloat64 magnitud2 = n.DotProduct(normal).GetScalar();
				if (magnitud2 < dFloat32 (1.0e-5f)) 
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

#if _DEBUG
		if (leftOver.GetCount()) 
		{
			dTrace (("warning: %d faces with more that a one shared edge\n", leftOver.GetCount()));
			dTrace (("         this mesh is not a manifold and may lead to collision malfunctions\n"));
		}
#endif

		dInt32 mark = polyhedra2.IncLRU();
		dInt32 index = 0;
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
