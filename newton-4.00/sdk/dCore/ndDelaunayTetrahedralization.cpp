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
#include "ndSort.h"
#include "ndStack.h"
#include "ndGoogol.h"
#include "ndSmallDeterminant.h"
#include "ndDelaunayTetrahedralization.h"

ndDelaunayTetrahedralization::ndDelaunayTetrahedralization(const ndFloat64* const vertexCloud, ndInt32 count, ndInt32 strideInByte, ndFloat64 distTol)
	:ndConvexHull4d()
{
	ndSetPrecisionDouble precision;
	ndStack<ndBigVector> pool(count + 2);

	ndBigVector* const points = &pool[0];
	ndInt32 stride = ndInt32 (strideInByte / sizeof (ndFloat64));
	for (ndInt32 i = 0; i < count; i ++) 
	{
		ndFloat64 x = dRoundToFloat (vertexCloud[i * stride + 0]);
		ndFloat64 y = dRoundToFloat (vertexCloud[i * stride + 1]);
		ndFloat64 z = dRoundToFloat (vertexCloud[i * stride + 2]);
		points[i] = ndBigVector (x, y, z, x * x + y * y + z * z);
	}

	ndInt32 oldCount = count;
	BuildHull (&pool[0].m_x, sizeof (ndBigVector), count, distTol);
	if (oldCount > m_count) 
	{
		// the mesh is convex, need to add two steiners point to make tractable
		ndBigVector origin (ndFloat64 (0.0f));
		ndFloat64 maxW = ndFloat64 (-1.0e20f);
		for (ndInt32 i = 0; i < count; i++) 
		{
			ndFloat64 x = dRoundToFloat(vertexCloud[i * stride + 0]);
			ndFloat64 y = dRoundToFloat(vertexCloud[i * stride + 1]);
			ndFloat64 z = dRoundToFloat(vertexCloud[i * stride + 2]);
			points[i] = ndBigVector(x, y, z, x * x + y * y + z * z);
			origin += points[i];
			maxW = dMax (points[i].m_w, maxW);
		}
		origin = origin.Scale (ndFloat64 (1.0f) / count);
		points[count + 0] = origin;
		points[count + 1] = origin;
		points[count + 0].m_w += ndFloat64 (1.0f);
		points[count + 1].m_w -= ndFloat64 (1.0f);
 		BuildHull (&pool[0].m_x, sizeof (ndBigVector), count + 2, distTol);
	}

	if (oldCount > m_count) 
	{
		// this is probably a regular convex solid, which will have a zero volume hull
		// add the rest of the points by incremental insertion with small perturbation
		ndInt32 hullCount = m_count;
		
		for (ndInt32 i = 0; i < count; i ++) 
		{
			bool inHull = false;
			const ndConvexHull4dVector* const hullPoints = &m_points[0];
			for (ndInt32 j = 0; j < hullCount; j ++) 
			{
				if (hullPoints[j].m_index == i) 
				{
					inHull = true;
					break;
				}			
			}
			if (!inHull) 
			{
				ndBigVector q (points[i]);
				ndInt32 index = AddVertex(q);
				if (index == -1) 
				{
					q.m_x += ndFloat64 (1.0e-3f);
					q.m_y += ndFloat64 (1.0e-3f);
					q.m_z += ndFloat64 (1.0e-3f);
					index = AddVertex(q);
					dAssert (index != -1);
				}
				dAssert (index != -1);
				m_points[index].m_index = i;
			}
		}
	}

	SortVertexArray ();

	ndTempList::FlushFreeList();
}

ndDelaunayTetrahedralization::~ndDelaunayTetrahedralization()
{
}

ndInt32 ndDelaunayTetrahedralization::AddVertex (const ndBigVector& vertex)
{
	ndSetPrecisionDouble precision;

	ndBigVector p (vertex);
	dAssert(p.m_w == ndFloat32(0.0f));
	p.m_w = p.DotProduct(p).GetScalar();
	ndInt32 index = ndConvexHull4d::AddVertex(p);

	return index;
}

void ndDelaunayTetrahedralization::SortVertexArray ()
{
	ndConvexHull4dVector* const points = &m_points[0];
	for (ndNode* node = GetFirst(); node; node = node->GetNext())
	{	
		ndConvexHull4dTetraherum* const tetra = &node->GetInfo();
		for (ndInt32 i = 0; i < 4; i ++) 
		{
			ndConvexHull4dTetraherum::ndTetrahedrumFace& face = tetra->m_faces[i];
			for (ndInt32 j = 0; j < 4; j ++) 
			{
				ndInt32 index = face.m_index[j];
				face.m_index[j] = points[index].m_index;
			}
		}
	}

	class CompareVertex
	{
		public:
		ndInt32 Compare(const ndConvexHull4dVector& elementA, const ndConvexHull4dVector& elementB, void* const) const
		{
			if (elementA.m_index < elementB.m_index)
			{
				return -1;
			}
			else if (elementA.m_index > elementB.m_index)
			{
				return 1;
			}
			return 0;
		}
	};
	ndSort<ndConvexHull4dVector, CompareVertex>(points, m_count);
}

void ndDelaunayTetrahedralization::RemoveUpperHull ()
{
	ndSetPrecisionDouble precision;

	ndNode* nextNode = NULL;
	for (ndNode* node = GetFirst(); node; node = nextNode) 
	{
		nextNode = node->GetNext();

		ndConvexHull4dTetraherum* const tetra = &node->GetInfo();
		tetra->SetMark(0);
		ndFloat64 w = GetTetraVolume (tetra);
		if (w >= ndFloat64 (0.0f)) 
		{
			DeleteFace(node);
		}
	}
}

void ndDelaunayTetrahedralization::DeleteFace (ndNode* const node)
{
	ndConvexHull4dTetraherum* const tetra = &node->GetInfo();
	for (ndInt32 i = 0; i < 4; i ++) 
	{
		ndNode* const twinNode = tetra->m_faces[i].m_twin;
		if (twinNode) 
		{
			ndConvexHull4dTetraherum* const twinTetra = &twinNode->GetInfo();
			for (ndInt32 j = 0; j < 4; j ++) 
			{
				if (twinTetra->m_faces[j].m_twin == node) 
				{
					twinTetra->m_faces[j].m_twin = NULL;
					break;
				}
			}
		}
	}
	ndConvexHull4d::DeleteFace (node);
}
