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
#include "ndTree.h"
#include "ndStack.h"
#include "ndGoogol.h"
#include "ndConvexHull3d.h"
#include "ndSmallDeterminant.h"


#define DG_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE		8

#ifdef	D_OLD_CONVEXHULL_3D
class ndConvexHull3d::ndNormalMap
{
	public:
	ndNormalMap()
		:m_count(sizeof(m_normal) / sizeof(m_normal[0]))
	{
		ndVector p0(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		ndVector p1(ndFloat32(-1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		ndVector p2(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		ndVector p3(ndFloat32(0.0f), ndFloat32(-1.0f), ndFloat32(0.0f), ndFloat32(0.0f));
		ndVector p4(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f));
		ndVector p5(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(-1.0f), ndFloat32(0.0f));

		ndInt32 count = 0;
		ndInt32 subdivitions = 2;
		TessellateTriangle(subdivitions, p4, p0, p2, count);
		TessellateTriangle(subdivitions, p0, p5, p2, count);
		TessellateTriangle(subdivitions, p5, p1, p2, count);
		TessellateTriangle(subdivitions, p1, p4, p2, count);
		TessellateTriangle(subdivitions, p0, p4, p3, count);
		TessellateTriangle(subdivitions, p5, p0, p3, count);
		TessellateTriangle(subdivitions, p1, p5, p3, count);
		TessellateTriangle(subdivitions, p4, p1, p3, count);
	}

	static const ndNormalMap& GetNormaMap()
	{
		static ndNormalMap normalMap;
		return normalMap;
	}

	void TessellateTriangle(ndInt32 level, const ndVector& p0, const ndVector& p1, const ndVector& p2, ndInt32& count)
	{
		if (level) 
		{
			dAssert(dAbs(p0.DotProduct(p0).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
			dAssert(dAbs(p1.DotProduct(p1).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
			dAssert(dAbs(p2.DotProduct(p2).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
			ndVector p01(p0 + p1);
			ndVector p12(p1 + p2);
			ndVector p20(p2 + p0);

			p01 = p01.Scale(ndRsqrt(p01.DotProduct(p01).GetScalar()));
			p12 = p12.Scale(ndRsqrt(p12.DotProduct(p12).GetScalar()));
			p20 = p20.Scale(ndRsqrt(p20.DotProduct(p20).GetScalar()));

			dAssert(dAbs(p01.DotProduct(p01).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
			dAssert(dAbs(p12.DotProduct(p12).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
			dAssert(dAbs(p20.DotProduct(p20).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));

			TessellateTriangle(level - 1, p0, p01, p20, count);
			TessellateTriangle(level - 1, p1, p12, p01, count);
			TessellateTriangle(level - 1, p2, p20, p12, count);
			TessellateTriangle(level - 1, p01, p12, p20, count);
		} 
		else 
		{
			ndBigPlane n(p0, p1, p2);
			n = n.Scale(ndFloat64(1.0f) / sqrt(n.DotProduct(n).GetScalar()));
			n.m_w = ndFloat64(0.0f);
			ndInt32 index = dBitReversal(count, sizeof(m_normal) / sizeof(m_normal[0]));
			m_normal[index] = n;
			count++;
			dAssert(count <= ndInt32 (sizeof(m_normal) / sizeof(m_normal[0])));
		}
	}

	ndBigVector m_normal[128];
	ndInt32 m_count;
};
#endif

class ndConvexHull3dVertex: public ndBigVector
{
	public:
	ndInt32 m_mark;
};

class ndConvexHull3dAABBTreeNode
{
	public:
	ndBigVector m_box[2];
	ndConvexHull3dAABBTreeNode* m_left;
	ndConvexHull3dAABBTreeNode* m_right;
	ndConvexHull3dAABBTreeNode* m_parent;
};

class ndConvexHull3dPointCluster: public ndConvexHull3dAABBTreeNode
{
	public:
	ndInt32 m_count;
	ndInt32 m_indices[DG_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE];
};


ndConvexHull3dFace::ndConvexHull3dFace()
{
	m_mark = 0;
	m_twin[0] = nullptr;
	m_twin[1] = nullptr;
	m_twin[2] = nullptr;
}

ndFloat64 ndConvexHull3dFace::Evalue (const ndBigVector* const pointArray, const ndBigVector& point) const
{
	const ndBigVector& p0 = pointArray[m_index[0]];
	const ndBigVector& p1 = pointArray[m_index[1]];
	const ndBigVector& p2 = pointArray[m_index[2]];

	ndFloat64 matrix[3][3];
	for (ndInt32 i = 0; i < 3; i ++) 
	{
		matrix[0][i] = p2[i] - p0[i];
		matrix[1][i] = p1[i] - p0[i];
		matrix[2][i] = point[i] - p0[i];
	}

	ndFloat64 error;
	ndFloat64 det = Determinant3x3 (matrix, &error);

	// the code use double, however the threshold for accuracy test is the machine precision of a float.
	// by changing this to a smaller number, the code should run faster since many small test will be considered valid
	// the precision must be a power of two no smaller than the machine precision of a double, (1<<48)
	// float64(1<<30) can be a good value

	// ndFloat64 precision	= ndFloat64 (1.0f) / ndFloat64 (1<<30);
	ndFloat64 precision	 = ndFloat64 (1.0f) / ndFloat64 (1<<24);
	ndFloat64 errbound = error * precision;
	if (fabs(det) > errbound) 
	{
		return det;
	}

	ndGoogol exactMatrix[3][3];
	for (ndInt32 i = 0; i < 3; i ++) 
	{
		exactMatrix[0][i] = ndGoogol(p2[i]) - ndGoogol(p0[i]);
		exactMatrix[1][i] = ndGoogol(p1[i]) - ndGoogol(p0[i]);
		exactMatrix[2][i] = ndGoogol(point[i]) - ndGoogol(p0[i]);
	}
	return Determinant3x3(exactMatrix);
}

ndBigPlane ndConvexHull3dFace::GetPlaneEquation (const ndBigVector* const pointArray) const
{
	const ndBigVector& p0 = pointArray[m_index[0]];
	const ndBigVector& p1 = pointArray[m_index[1]];
	const ndBigVector& p2 = pointArray[m_index[2]];
	ndBigPlane plane (p0, p1, p2);
	plane = plane.Scale (1.0f / sqrt (plane.DotProduct(plane & ndBigVector::m_triplexMask).GetScalar()));
	return plane;
}


ndConvexHull3d::ndConvexHull3d ()
	:ndList<ndConvexHull3dFace>()
	,m_aabbP0(ndBigVector (ndFloat64 (0.0f)))
	,m_aabbP1(ndBigVector (ndFloat64 (0.0f)))
	,m_count(0)
	,m_diag()
	,m_points()
{
}

ndConvexHull3d::ndConvexHull3d(const ndConvexHull3d& source)
	:ndList<ndConvexHull3dFace>()
	,m_aabbP0 (source.m_aabbP0)
	,m_aabbP1 (source.m_aabbP1)
	,m_count(source.m_count)
	,m_diag(source.m_diag)
	,m_points(source.m_count)
{
	m_points.SetCount(source.m_count);
	m_points[m_count-1].m_w = ndFloat64 (0.0f);
	for (ndInt32 i = 0; i < m_count; i ++) 
	{
		m_points[i] = source.m_points[i];
	}
	ndTree<ndNode*, ndNode*> map;
	for(ndNode* sourceNode = source.GetFirst(); sourceNode; sourceNode = sourceNode->GetNext() ) 
	{
		ndNode* const node = Append();
		map.Insert(node, sourceNode);
	}

	for(ndNode* sourceNode = source.GetFirst(); sourceNode; sourceNode = sourceNode->GetNext() ) {
		ndNode* const node = map.Find(sourceNode)->GetInfo();

		ndConvexHull3dFace& face = node->GetInfo();
		ndConvexHull3dFace& srcFace = sourceNode->GetInfo();

		face.m_mark = 0;
		for (ndInt32 i = 0; i < 3; i ++) {
			face.m_index[i] = srcFace.m_index[i];
			face.m_twin[i] = map.Find (srcFace.m_twin[i])->GetInfo();
		}
	}
}

ndConvexHull3d::ndConvexHull3d(const ndFloat64* const vertexCloud, ndInt32 strideInBytes, ndInt32 count, ndFloat64 distTol, ndInt32 maxVertexCount)
	:ndList<ndConvexHull3dFace>()
	,m_aabbP0(ndBigVector::m_zero)
	,m_aabbP1(ndBigVector::m_zero)
	,m_count(0)
	,m_diag()
	,m_points()
{
	BuildHull (vertexCloud, strideInBytes, count, distTol, maxVertexCount);
}

ndConvexHull3d::~ndConvexHull3d(void)
{
}

void ndConvexHull3d::BuildHull (const ndFloat64* const vertexCloud, ndInt32 strideInBytes, ndInt32 count, ndFloat64 distTol, ndInt32 maxVertexCount)
{
	ndSetPrecisionDouble precision;

	ndInt32 treeCount = count / (DG_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE>>1);
	if (treeCount < 4) 
	{
		treeCount = 4;
	}
	treeCount *= 2;

	ndStack<ndConvexHull3dVertex> points (count);
	ndStack<ndConvexHull3dPointCluster> treePool (treeCount + 256);
	count = InitVertexArray(&points[0], vertexCloud, strideInBytes, count, &treePool[0], treePool.GetSizeInBytes());

#ifdef	D_OLD_CONVEXHULL_3D
	if (m_count >= 4) 
	{
		CalculateConvexHull3d (&treePool[0], &points[0], count, distTol, maxVertexCount);
	}
#else
	if (m_count >= 3) 
	{
		if (CheckFlatSurface(&treePool[0], &points[0], count, distTol, maxVertexCount))
		{
			CalculateConvexHull2d(&treePool[0], &points[0], count, distTol, maxVertexCount);
		} 
		else 
		{
			dAssert(m_count == 4);
			CalculateConvexHull3d(&treePool[0], &points[0], count, distTol, maxVertexCount);
		}
	}
#endif
}

ndConvexHull3dAABBTreeNode* ndConvexHull3d::BuildTree (ndConvexHull3dAABBTreeNode* const parent, ndConvexHull3dVertex* const points, ndInt32 count, ndInt32 baseIndex, ndInt8** memoryPool, ndInt32& maxMemSize) const
{
	ndConvexHull3dAABBTreeNode* tree = nullptr;

	dAssert (count);
	ndBigVector minP ( ndFloat32 (1.0e15f));
	ndBigVector maxP (-ndFloat32 (1.0e15f));
	if (count <= DG_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE) 
	{
		ndConvexHull3dPointCluster* const clump = new (*memoryPool) ndConvexHull3dPointCluster;
		*memoryPool += sizeof (ndConvexHull3dPointCluster);
		maxMemSize -= sizeof (ndConvexHull3dPointCluster);
		dAssert (maxMemSize >= 0);

		dAssert (clump);
		clump->m_count = count;
		for (ndInt32 i = 0; i < count; i ++) 
		{
			clump->m_indices[i] = i + baseIndex;

			const ndBigVector& p = points[i];
			dAssert(p.m_w == ndFloat32(0.0f));
			minP = minP.GetMin(p);
			maxP = maxP.GetMax(p);
		}

		clump->m_left = nullptr;
		clump->m_right = nullptr;
		tree = clump;

	} 
	else 
	{
		ndBigVector median (ndBigVector::m_zero);
		ndBigVector varian (ndBigVector::m_zero);
		for (ndInt32 i = 0; i < count; i ++) 
		{
			const ndBigVector& p = points[i];
			dAssert(p.m_w == ndFloat32(0.0f));
			minP = minP.GetMin(p);
			maxP = maxP.GetMax(p);
			median += p;
			varian += p * p;
		}

		varian = varian.Scale(ndFloat32(count)) - median * median;
		ndInt32 index = 0;
		ndFloat64 maxVarian = ndFloat64 (-1.0e10f);
		for (ndInt32 i = 0; i < 3; i ++) 
		{
			if (varian[i] > maxVarian) 
			{
				index = i;
				maxVarian = varian[i];
			}
		}
		ndBigVector center (median.Scale (ndFloat64 (1.0f) / ndFloat64 (count)));

		ndFloat64 test = center[index];

		ndInt32 i0 = 0;
		ndInt32 i1 = count - 1;
		do 
		{
			for (; i0 <= i1; i0 ++) 
			{
				ndFloat64 val = points[i0][index];
				if (val > test) 
				{
					break;
				}
			}

			for (; i1 >= i0; i1 --) 
			{
				ndFloat64 val = points[i1][index];
				if (val < test) 
				{
					break;
				}
			}

			if (i0 < i1)
			{
				dSwap(points[i0], points[i1]);
				i0++;
				i1--;
			}
		} while (i0 <= i1);

		if (i0 == 0)
		{
			i0 = count / 2;
		}
		if (i0 >= (count - 1))
		{
			i0 = count / 2;
		}

		tree = new (*memoryPool) ndConvexHull3dAABBTreeNode;
		*memoryPool += sizeof (ndConvexHull3dAABBTreeNode);
		maxMemSize -= sizeof (ndConvexHull3dAABBTreeNode);
		dAssert (maxMemSize >= 0);

		dAssert (i0);
		dAssert (count - i0);

		tree->m_left = BuildTree (tree, points, i0, baseIndex, memoryPool, maxMemSize);
		tree->m_right = BuildTree (tree, &points[i0], count - i0, i0 + baseIndex, memoryPool, maxMemSize);
	}

	dAssert (tree);
	tree->m_parent = parent;
	tree->m_box[0] = (minP - ndBigVector (ndFloat64 (1.0e-3f))) & ndBigVector::m_triplexMask;
	tree->m_box[1] = (maxP + ndBigVector (ndFloat64 (1.0e-3f))) & ndBigVector::m_triplexMask;
	return tree;
}

ndInt32 ndConvexHull3d::GetUniquePoints(ndConvexHull3dVertex* const points, const ndFloat64* const vertexCloud, ndInt32 strideInBytes, ndInt32 count, void* const, ndInt32)
{
	const ndInt32 stride = ndInt32(strideInBytes / sizeof(ndFloat64));
	//if (stride >= 4) 
	//{
	//	for (ndInt32 i = 0; i < count; i++) 
	//	{
	//		ndInt32 index = i * stride;
	//		ndBigVector& vertex = points[i];
	//		vertex = ndBigVector(vertexCloud[index], vertexCloud[index + 1], vertexCloud[index + 2], vertexCloud[index + 3]);
	//		dAssert(dCheckVector(vertex));
	//		points[i].m_mark = 0;
	//	}
	//} 
	//else 
	//{
	//	for (ndInt32 i = 0; i < count; i++) 
	//	{
	//		ndInt32 index = i * stride;
	//		ndBigVector& vertex = points[i];
	//		vertex = ndBigVector(vertexCloud[index], vertexCloud[index + 1], vertexCloud[index + 2], ndFloat64(0.0f));
	//		dAssert(dCheckVector(vertex));
	//		points[i].m_mark = 0;
	//	}
	//}

	for (ndInt32 i = 0; i < count; i++) 
	{
		ndInt32 index = i * stride;
		ndBigVector& vertex = points[i];
		vertex = ndBigVector(vertexCloud[index], vertexCloud[index + 1], vertexCloud[index + 2], ndFloat64(0.0f));
		dAssert(dCheckVector(vertex));
		points[i].m_mark = 0;
	}

	class CompareVertex
	{
		public:
		ndInt32 Compare(const ndConvexHull3dVertex& elementA, const ndConvexHull3dVertex& elementB, void* const) const
		{
			for (ndInt32 i = 0; i < 3; i++) 
			{
				if (elementA[i] < elementB[i])
				{
					return -1;
				}
				else if (elementA[i] > elementB[i])
				{
					return 1;
				}
			}
			return 0;
		}
	};
	ndSort<ndConvexHull3dVertex, CompareVertex>(points, count);

	ndInt32 indexCount = 0;
	CompareVertex compareVetex;
	for (ndInt32 i = 1; i < count; i++) 
	{
		for (; i < count; i++) 
		{
			if (compareVetex.Compare(points[indexCount], points[i], nullptr))
			{
				indexCount++;
				points[indexCount] = points[i];
				break;
			}
		}
	}
	count = indexCount + 1;
	return count;
}

ndInt32 ndConvexHull3d::InitVertexArray(ndConvexHull3dVertex* const points, const ndFloat64* const vertexCloud, ndInt32 strideInBytes, ndInt32 count, void* const memoryPool, ndInt32 maxMemSize)
{
	count = GetUniquePoints(points, vertexCloud, strideInBytes, count, memoryPool, maxMemSize);
	if (count < 4) 
	{
		m_count = 0;
		return count;
	}
	ndConvexHull3dAABBTreeNode* tree = BuildTree (nullptr, points, count, 0, (ndInt8**) &memoryPool, maxMemSize);

	m_points.SetCount(count);
	m_aabbP0 = tree->m_box[0];
	m_aabbP1 = tree->m_box[1];

	ndBigVector boxSize (tree->m_box[1] - tree->m_box[0]);
	dAssert (boxSize.m_w == ndFloat32 (0.0f));
	m_diag = ndFloat32 (sqrt (boxSize.DotProduct(boxSize).GetScalar()));

#ifdef D_OLD_CONVEXHULL_3D
	const ndNormalMap& normalMap = ndNormalMap::GetNormaMap();

	ndInt32 index0 = SupportVertex (&tree, points, normalMap.m_normal[0]);
	m_points[0] = points[index0];
	points[index0].m_mark = 1;

	bool validTetrahedrum = false;
	ndBigVector e1 (ndBigVector::m_zero);
	for (ndInt32 i = 1; i < normalMap.m_count; i ++) 
	{
		ndInt32 index = SupportVertex (&tree, points, normalMap.m_normal[i]);
		dAssert (index >= 0);

		e1 = points[index] - m_points[0];
		dAssert (e1.m_w == ndFloat32 (0.0f));
		ndFloat64 error2 = e1.DotProduct(e1).GetScalar();
		if (error2 > (ndFloat32 (1.0e-4f) * m_diag * m_diag)) 
		{
			m_points[1] = points[index];
			points[index].m_mark = 1;
			validTetrahedrum = true;
			break;
		}
	}
	if (!validTetrahedrum) 
	{
		m_count = 0;
		dAssert (0);
		return count;
	}

	validTetrahedrum = false;
	ndBigVector e2(ndBigVector::m_zero);
	ndBigVector normal (ndBigVector::m_zero);
	for (ndInt32 i = 2; i < normalMap.m_count; i ++) 
	{
		ndInt32 index = SupportVertex (&tree, points, normalMap.m_normal[i]);
		dAssert (index >= 0);
		e2 = points[index] - m_points[0];
		normal = e1.CrossProduct(e2);
		dAssert (e2.m_w == ndFloat32 (0.0f));
		dAssert (normal.m_w == ndFloat32 (0.0f));
		ndFloat64 error2 = sqrt (normal.DotProduct(normal).GetScalar());
		if (error2 > (ndFloat32 (1.0e-4f) * m_diag * m_diag)) 
		{
			m_points[2] = points[index];
			points[index].m_mark = 1;
			validTetrahedrum = true;
			break;
		}
	}

	dAssert(normal.m_w == ndFloat32(0.0f));
	if (!validTetrahedrum) 
	{
		m_count = 0;
		dAssert (0);
		return count;
	}

	// find the largest possible tetrahedron
	validTetrahedrum = false;
	ndBigVector e3(ndBigVector::m_zero);

	index0 = SupportVertex (&tree, points, normal);
	e3 = points[index0] - m_points[0];
	dAssert (e3.m_w == ndFloat32 (0.0f));
	ndFloat64 err2 = normal.DotProduct(e3).GetScalar();
	if (fabs (err2) > (ndFloat64 (1.0e-6f) * m_diag * m_diag)) 
	{
		// we found a valid tetrahedral, about and start build the hull by adding the rest of the points
		m_points[3] = points[index0];
		points[index0].m_mark = 1;
		validTetrahedrum = true;
	}
	if (!validTetrahedrum) 
	{
		ndVector n (normal.Scale(ndFloat64 (-1.0f)));
		ndInt32 index = SupportVertex (&tree, points, n);
		e3 = points[index] - m_points[0];
		dAssert (e3.m_w == ndFloat32 (0.0f));
		ndFloat64 error2 = normal.DotProduct(e3).GetScalar();
		if (fabs (error2) > (ndFloat64 (1.0e-6f) * m_diag * m_diag)) 
		{
			// we found a valid tetrahedral, about and start build the hull by adding the rest of the points
			m_points[3] = points[index];
			points[index].m_mark = 1;
			validTetrahedrum = true;
		}
	}
	if (!validTetrahedrum) 
	{
		for (ndInt32 i = 3; i < normalMap.m_count; i ++) 
		{
			ndInt32 index = SupportVertex (&tree, points, normalMap.m_normal[i]);
			dAssert (index >= 0);

			//make sure the volume of the fist tetrahedral is no negative
			e3 = points[index] - m_points[0];
			dAssert (e3.m_w == ndFloat32 (0.0f));
			ndFloat64 error2 = normal.DotProduct(e3).GetScalar();
			if (fabs (error2) > (ndFloat64 (1.0e-6f) * m_diag * m_diag)) 
			{
				// we found a valid tetrahedral, about and start build the hull by adding the rest of the points
				m_points[3] = points[index];
				points[index].m_mark = 1;
				validTetrahedrum = true;
				break;
			}
		}
	}
	if (!validTetrahedrum) 
	{
		// the points do not form a convex hull
		m_count = 0;
		//dAssert (0);
		return count;
	}

	m_count = 4;
	ndFloat64 volume = TetrahedrumVolume (m_points[0], m_points[1], m_points[2], m_points[3]);
	if (volume > ndFloat64 (0.0f)) 
	{
		dSwap(m_points[2], m_points[3]);
	}
	dAssert (TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]) < ndFloat64(0.0f));

	return count;
#else
	
	ndBigVector origin((m_aabbP1 + m_aabbP0).Scale (0.5f));

	ndBigVector dir(m_aabbP1 - m_aabbP0);
	dAssert(dir.DotProduct3(dir) > ndFloat32(1.0e-4f));
	dir = dir.Normalize();
	ndInt32 index0 = SupportVertex(&tree, points, dir);
	m_points[0] = points[index0];
	points[index0].m_mark = 1;

	dir = origin - m_points[0];
	dAssert(dir.DotProduct3(dir) > ndFloat32(1.0e-4f));
	dir = dir.Normalize();
	ndInt32 index1 = SupportVertex(&tree, points, dir);
	m_points[1] = points[index1];
	points[index1].m_mark = 1;

	ndBigVector e0(m_points[1] - m_points[0]);
	dAssert(e0.DotProduct3(e0) > ndFloat32(1.0e-4f));
	ndFloat64 t = -e0.DotProduct3(origin - m_points[0]) / e0.DotProduct3(e0);
	dir = m_points[0] + e0.Scale(t) - origin;

	dAssert(dir.DotProduct3(dir) > ndFloat32(1.0e-4f));
	dir = dir.Normalize();
	ndInt32 index2 = SupportVertex(&tree, points, dir);
	m_points[2] = points[index2];
	points[index2].m_mark = 1;

	ndBigVector e1 (m_points[2] - m_points[0]);
	ndBigVector normal (e1.CrossProduct(e0));
	ndFloat64 error2 = sqrt(normal.DotProduct3(normal));
	if (error2 < (ndFloat32(1.0e-4f) * m_diag * m_diag)) 
	{
		dAssert(0);
//		m_points[2] = points[index];
//		points[index].m_mark = 1;
//		validTetrahedrum = true;
//		break;
	}

	m_count = 3;
	return count;
#endif
}

ndFloat64 ndConvexHull3d::TetrahedrumVolume (const ndBigVector& p0, const ndBigVector& p1, const ndBigVector& p2, const ndBigVector& p3) const
{
	const ndBigVector p1p0 (p1 - p0);
	const ndBigVector p2p0 (p2 - p0);
	const ndBigVector p3p0 (p3 - p0);
	dAssert (p1p0.m_w == ndFloat32 (0.0f));
	dAssert (p2p0.m_w == ndFloat32 (0.0f));
	dAssert (p3p0.m_w == ndFloat32 (0.0f));
	return p3p0.DotProduct(p1p0.CrossProduct(p2p0)).GetScalar();
}

ndInt32 ndConvexHull3d::SupportVertex (ndConvexHull3dAABBTreeNode** const treePointer, const ndConvexHull3dVertex* const points, const ndBigVector& dirPlane, const bool removeEntry) const
{
	#define DG_STACK_DEPTH_3D 64
	ndFloat64 aabbProjection[DG_STACK_DEPTH_3D];
	const ndConvexHull3dAABBTreeNode *stackPool[DG_STACK_DEPTH_3D];

	ndBigVector dir(dirPlane & ndBigPlane::m_triplexMask);
	dAssert (dir.m_w == ndFloat32 (0.0f));

	ndInt32 index = -1;
	ndInt32 stack = 1;
	stackPool[0] = *treePointer;
	aabbProjection[0] = ndFloat32 (1.0e20f);
	ndFloat64 maxProj = ndFloat64 (-1.0e20f);
	ndInt32 ix = (dir[0] > ndFloat64 (0.0f)) ? 1 : 0;
	ndInt32 iy = (dir[1] > ndFloat64 (0.0f)) ? 1 : 0;
	ndInt32 iz = (dir[2] > ndFloat64 (0.0f)) ? 1 : 0;
	while (stack) 
	{
		stack--;
		ndFloat64 boxSupportValue = aabbProjection[stack];
		if (boxSupportValue > maxProj) 
		{
			const ndConvexHull3dAABBTreeNode* const me = stackPool[stack];

			if (me->m_left && me->m_right) 
			{
				const ndBigVector leftSupportPoint (me->m_left->m_box[ix].m_x, me->m_left->m_box[iy].m_y, me->m_left->m_box[iz].m_z, ndFloat32 (0.0f));
				ndFloat64 leftSupportDist = leftSupportPoint.DotProduct(dir).GetScalar();

				const ndBigVector rightSupportPoint (me->m_right->m_box[ix].m_x, me->m_right->m_box[iy].m_y, me->m_right->m_box[iz].m_z, ndFloat32 (0.0f));
				ndFloat64 rightSupportDist = rightSupportPoint.DotProduct(dir).GetScalar();

				if (rightSupportDist >= leftSupportDist) 
				{
					aabbProjection[stack] = leftSupportDist;
					stackPool[stack] = me->m_left;
					stack++;
					dAssert (stack < DG_STACK_DEPTH_3D);
					aabbProjection[stack] = rightSupportDist;
					stackPool[stack] = me->m_right;
					stack++;
					dAssert (stack < DG_STACK_DEPTH_3D);
				} 
				else 
				{
					aabbProjection[stack] = rightSupportDist;
					stackPool[stack] = me->m_right;
					stack++;
					dAssert (stack < DG_STACK_DEPTH_3D);
					aabbProjection[stack] = leftSupportDist;
					stackPool[stack] = me->m_left;
					stack++;
					dAssert (stack < DG_STACK_DEPTH_3D);
				}
			} 
			else 
			{
				ndConvexHull3dPointCluster* const cluster = (ndConvexHull3dPointCluster*) me;
				for (ndInt32 i = 0; i < cluster->m_count; i ++) 
				{
					const ndConvexHull3dVertex& p = points[cluster->m_indices[i]];
					dAssert (p.m_x >= cluster->m_box[0].m_x);
					dAssert (p.m_x <= cluster->m_box[1].m_x);
					dAssert (p.m_y >= cluster->m_box[0].m_y);
					dAssert (p.m_y <= cluster->m_box[1].m_y);
					dAssert (p.m_z >= cluster->m_box[0].m_z);
					dAssert (p.m_z <= cluster->m_box[1].m_z);
					if (!p.m_mark) 
					{
						dAssert (p.m_w == ndFloat32 (0.0f));
						ndFloat64 dist = p.DotProduct(dir).GetScalar();
						if (dist > maxProj) 
						{
							maxProj = dist;
							index = cluster->m_indices[i];
						}
					} 
					else if (removeEntry) 
					{
						cluster->m_indices[i] = cluster->m_indices[cluster->m_count - 1];
						cluster->m_count = cluster->m_count - 1;
						i --;
					}
				}

				if (cluster->m_count == 0) 
				{
					ndConvexHull3dAABBTreeNode* const parent = cluster->m_parent;
					if (parent) 
					{
						ndConvexHull3dAABBTreeNode* const sibling = (parent->m_left != cluster) ? parent->m_left : parent->m_right;
						dAssert (sibling != cluster);
						ndConvexHull3dAABBTreeNode* const grandParent = parent->m_parent;
						if (grandParent) 
						{
							sibling->m_parent = grandParent;
							if (grandParent->m_right == parent) 
							{
								grandParent->m_right = sibling;
							} 
							else 
							{
								grandParent->m_left = sibling;
							}
						} 
						else 
						{
							sibling->m_parent = nullptr;
							*treePointer = sibling;
						}
					}
				}
			}
		}
	}

	dAssert (index != -1);
	return index;
}

ndConvexHull3d::ndNode* ndConvexHull3d::AddFace (ndInt32 i0, ndInt32 i1, ndInt32 i2)
{
	ndNode* const node = Append();
	ndConvexHull3dFace& face = node->GetInfo();

	face.m_index[0] = i0;
	face.m_index[1] = i1;
	face.m_index[2] = i2;
	return node;
}

void ndConvexHull3d::DeleteFace (ndNode* const node)
{
	Remove (node);
}

bool ndConvexHull3d::Sanity() const
{
/*
	for (ndNode* node = GetFirst(); node; node = node->GetNext()) {
		ndConvexHull3dFace* const face = &node->GetInfo();
		for (ndInt32 i = 0; i < 3; i ++) {
			ndNode* const twinNode = face->m_twin[i];
			if (!twinNode) {
				return false;
			}

			ndInt32 count = 0;
			ndNode* me = nullptr;
			ndConvexHull3dFace* const twinFace = &twinNode->GetInfo();
			for (ndInt32 j = 0; j < 3; j ++) {
				if (twinFace->m_twin[j] == node) {
					count ++;
					me = twinFace->m_twin[j];
				}
			}
			if (count != 1) {
				return false;
			}
			if (me != node) {
				return false;
			}
		}
	}
*/
	return true;
}

bool ndConvexHull3d::CheckFlatSurface(ndConvexHull3dAABBTreeNode* tree, ndConvexHull3dVertex* const points, ndInt32, ndFloat64, ndInt32)
{
	ndBigVector e0(m_points[1] - m_points[0]);
	ndBigVector e1(m_points[2] - m_points[0]);
	dAssert(e0.m_w == ndFloat32(0.0f));
	dAssert(e1.m_w == ndFloat32(0.0f));
	dAssert(e0.DotProduct(e0).GetScalar() > ndFloat32(1.0e-4f));
	dAssert(e1.DotProduct(e1).GetScalar() > ndFloat32(1.0e-4f));
	ndBigVector normal(e1.CrossProduct(e0));
	dAssert(normal.m_w == ndFloat32(0.0f));
	dAssert(normal.DotProduct(normal).GetScalar() > ndFloat32(1.0e-6f));
	normal = normal.Normalize();

	ndInt32 index = SupportVertex(&tree, points, normal);
	m_points[3] = points[index];

	ndFloat64 volume = TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]);
	if (dAbs(volume) < ndFloat32(1.0e-9f)) {
		normal = normal.Scale(ndFloat32(-1.0f));
		index = SupportVertex(&tree, points, normal);
		m_points[3] = points[index];
		volume = TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]);
		if (dAbs(volume) < ndFloat32(1.0e-9f)) {
			return true;
		}
	}
	points[index].m_mark = 1;
	if (volume > ndFloat64(0.0f)) {
		dSwap(m_points[2], m_points[3]);
	}
	dAssert(TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]) < ndFloat64(0.0f));
	m_count = 4;
	return false;
}


void ndConvexHull3d::CalculateConvexHull2d(ndConvexHull3dAABBTreeNode*, ndConvexHull3dVertex* const, ndInt32, ndFloat64, ndInt32)
{

}

void ndConvexHull3d::CalculateConvexHull3d (ndConvexHull3dAABBTreeNode* vertexTree, ndConvexHull3dVertex* const points, ndInt32 count, ndFloat64 distTol, ndInt32 maxVertexCount)
{
	distTol = dAbs (distTol) * m_diag;
	ndNode* const f0Node = AddFace (0, 1, 2);
	ndNode* const f1Node = AddFace (0, 2, 3);
	ndNode* const f2Node = AddFace (2, 1, 3);
	ndNode* const f3Node = AddFace (1, 0, 3);

	ndConvexHull3dFace* const f0 = &f0Node->GetInfo();
	ndConvexHull3dFace* const f1 = &f1Node->GetInfo();
	ndConvexHull3dFace* const f2 = &f2Node->GetInfo();
	ndConvexHull3dFace* const f3 = &f3Node->GetInfo();

	f0->m_twin[0] = (ndList<ndConvexHull3dFace>::ndNode*)f3Node;
	f0->m_twin[1] = (ndList<ndConvexHull3dFace>::ndNode*)f2Node;
	f0->m_twin[2] = (ndList<ndConvexHull3dFace>::ndNode*)f1Node;

	f1->m_twin[0] = (ndList<ndConvexHull3dFace>::ndNode*)f0Node;
	f1->m_twin[1] = (ndList<ndConvexHull3dFace>::ndNode*)f2Node;
	f1->m_twin[2] = (ndList<ndConvexHull3dFace>::ndNode*)f3Node;

	f2->m_twin[0] = (ndList<ndConvexHull3dFace>::ndNode*)f0Node;
	f2->m_twin[1] = (ndList<ndConvexHull3dFace>::ndNode*)f3Node;
	f2->m_twin[2] = (ndList<ndConvexHull3dFace>::ndNode*)f1Node;

	f3->m_twin[0] = (ndList<ndConvexHull3dFace>::ndNode*)f0Node;
	f3->m_twin[1] = (ndList<ndConvexHull3dFace>::ndNode*)f1Node;
	f3->m_twin[2] = (ndList<ndConvexHull3dFace>::ndNode*)f2Node;

	ndList<ndNode*> boundaryFaces;

	boundaryFaces.Append(f0Node);
	boundaryFaces.Append(f1Node);
	boundaryFaces.Append(f2Node);
	boundaryFaces.Append(f3Node);
	count -= 4;
	maxVertexCount -= 4;
	ndInt32 currentIndex = 4;

	ndStack<ndNode*> stackPool(1024 + m_count);
	ndStack<ndNode*> coneListPool(1024 + m_count);
	ndStack<ndNode*> deleteListPool(1024 + m_count);

	ndNode** const stack = &stackPool[0];
	ndNode** const coneList = &stackPool[0];
	ndNode** const deleteList = &deleteListPool[0];

	while (boundaryFaces.GetCount() && count && (maxVertexCount > 0)) 
	{
		// my definition of the optimal convex hull of a given vertex count,
		// is the convex hull formed by a subset of the input vertex that minimizes the volume difference
		// between the perfect hull formed from all input vertex and the hull of the sub set of vertex.
		// When using a priority heap this algorithms will generate the an optimal of a fix vertex count.
		// Since all Newton's tools do not have a limit on the point count of a convex hull, I can use either a stack or a queue.
		// a stack maximize construction speed, a Queue tend to maximize the volume of the generated Hull approaching a perfect Hull.
		// For now we use a queue.
		// For general hulls it does not make a difference if we use a stack, queue, or a priority heap.
		// perfect optimal hull only apply for when build hull of a limited vertex count.
		//
		// Also when building Hulls of a limited vertex count, this function runs in constant time.
		// yes that is correct, it does not makes a difference if you build a N point hull from 100 vertex
		// or from 100000 vertex input array.

		#if 0
			// using stack (faster)
			dNode* const faceNode = boundaryFaces.GetFirst()->GetInfo();
		#else
			// using a queue (some what slower by better hull when reduced vertex count is desired)
			ndNode* const faceNode = boundaryFaces.GetLast()->GetInfo();
		#endif

		ndConvexHull3dFace* const face = &faceNode->GetInfo();
		ndBigPlane planeEquation (face->GetPlaneEquation (&m_points[0]));

		ndInt32 index = SupportVertex (&vertexTree, points, planeEquation);
		const ndBigVector& p = points[index];
		ndFloat64 dist = planeEquation.Evalue(p);

		if ((dist >= distTol) && (face->Evalue(&m_points[0], p) > ndFloat64(0.0f))) 
		{
			dAssert (Sanity());

			dAssert (faceNode);
			stack[0] = faceNode;

			ndInt32 stackIndex = 1;
			ndInt32 deletedCount = 0;

			while (stackIndex) 
			{
				stackIndex --;
				ndNode* const node1 = stack[stackIndex];
				ndConvexHull3dFace* const face1 = &node1->GetInfo();

				if (!face1->m_mark && (face1->Evalue(&m_points[0], p) > ndFloat64(0.0f))) 
				{
					#ifdef _DEBUG
					for (ndInt32 i = 0; i < deletedCount; i ++) 
					{
						dAssert (deleteList[i] != node1);
					}
					#endif

					deleteList[deletedCount] = node1;
					deletedCount ++;
					dAssert (deletedCount < ndInt32 (deleteListPool.GetElementsCount()));
					face1->m_mark = 1;
					for (ndInt32 i = 0; i < 3; i ++) 
					{
						ndNode* const twinNode = (ndNode*)face1->m_twin[i];
						dAssert (twinNode);
						ndConvexHull3dFace* const twinFace = &twinNode->GetInfo();
						if (!twinFace->m_mark) 
						{
							stack[stackIndex] = twinNode;
							stackIndex ++;
							dAssert (stackIndex < ndInt32 (stackPool.GetElementsCount()));
						}
					}
				}
			}

			m_points[currentIndex] = points[index];
			points[index].m_mark = 1;

			ndInt32 newCount = 0;
			for (ndInt32 i = 0; i < deletedCount; i ++) 
			{
				ndNode* const node1 = deleteList[i];
				ndConvexHull3dFace* const face1 = &node1->GetInfo();
				dAssert (face1->m_mark == 1);
				for (ndInt32 j0 = 0; j0 < 3; j0 ++) 
				{
					ndNode* const twinNode = face1->m_twin[j0];
					ndConvexHull3dFace* const twinFace = &twinNode->GetInfo();
					if (!twinFace->m_mark) 
					{
						ndInt32 j1 = (j0 == 2) ? 0 : j0 + 1;
						ndNode* const newNode = AddFace (currentIndex, face1->m_index[j0], face1->m_index[j1]);
						boundaryFaces.Addtop(newNode);

						ndConvexHull3dFace* const newFace = &newNode->GetInfo();
						newFace->m_twin[1] = twinNode;
						for (ndInt32 k = 0; k < 3; k ++) 
						{
							if (twinFace->m_twin[k] == node1) 
							{
								twinFace->m_twin[k] = newNode;
							}
						}
						coneList[newCount] = newNode;
						newCount ++;
						dAssert (newCount < ndInt32 (coneListPool.GetElementsCount()));
					}
				}
			}

			for (ndInt32 i = 0; i < newCount - 1; i ++) 
			{
				ndNode* const nodeA = coneList[i];
				ndConvexHull3dFace* const faceA = &nodeA->GetInfo();
				dAssert (faceA->m_mark == 0);
				for (ndInt32 j = i + 1; j < newCount; j ++) {
					ndNode* const nodeB = coneList[j];
					ndConvexHull3dFace* const faceB = &nodeB->GetInfo();
					dAssert (faceB->m_mark == 0);
					if (faceA->m_index[2] == faceB->m_index[1]) 
					{
						faceA->m_twin[2] = nodeB;
						faceB->m_twin[0] = nodeA;
						break;
					}
				}

				for (ndInt32 j = i + 1; j < newCount; j ++) 
				{
					ndNode* const nodeB = coneList[j];
					ndConvexHull3dFace* const faceB = &nodeB->GetInfo();
					dAssert (faceB->m_mark == 0);
					if (faceA->m_index[1] == faceB->m_index[2]) 
					{
						faceA->m_twin[0] = nodeB;
						faceB->m_twin[2] = nodeA;
						break;
					}
				}
			}

			for (ndInt32 i = 0; i < deletedCount; i ++) 
			{
				ndNode* const node = deleteList[i];
				boundaryFaces.Remove (node);
				DeleteFace (node);
			}

			maxVertexCount --;
			currentIndex ++;
			count --;
		} 
		else 
		{
			boundaryFaces.Remove (faceNode);
		}
	}
	m_count = currentIndex;
}


void ndConvexHull3d::CalculateVolumeAndSurfaceArea (ndFloat64& volume, ndFloat64& surfaceArea) const
{
	ndFloat64 areaAcc = ndFloat32 (0.0f);
	ndFloat64  volumeAcc = ndFloat32 (0.0f);
	for (ndNode* node = GetFirst(); node; node = node->GetNext()) {
		const ndConvexHull3dFace* const face = &node->GetInfo();
		ndInt32 i0 = face->m_index[0];
		ndInt32 i1 = face->m_index[1];
		ndInt32 i2 = face->m_index[2];
		const ndBigVector& p0 = m_points[i0];
		const ndBigVector& p1 = m_points[i1];
		const ndBigVector& p2 = m_points[i2];
		dAssert(p0.m_w == ndFloat32(0.0f));
		dAssert(p1.m_w == ndFloat32(0.0f));
		dAssert(p2.m_w == ndFloat32(0.0f));
		ndBigVector normal ((p1 - p0).CrossProduct(p2 - p0));
		dAssert(normal.m_w == ndFloat32(0.0f));
		ndFloat64 area = sqrt (normal.DotProduct(normal).GetScalar());
		areaAcc += area;
		volumeAcc += p2.DotProduct(p0.CrossProduct(p1)).GetScalar();
	}
	dAssert (volumeAcc >= ndFloat64 (0.0f));
	volume = volumeAcc * ndFloat64 (1.0f/6.0f);
	surfaceArea = areaAcc * ndFloat64 (0.5f);
}

// this code has linear time complexity on the number of faces
ndFloat64 ndConvexHull3d::RayCast (const ndBigVector& localP0, const ndBigVector& localP1) const
{
	ndFloat64 interset = ndFloat32 (1.2f);
	ndFloat64 tE = ndFloat64 (0.0f);	// for the maximum entering segment parameter;
	ndFloat64 tL = ndFloat64 (1.0f);	// for the minimum leaving segment parameter;
	ndBigVector dS (localP1 - localP0); // is the segment direction vector;

	dAssert(dS.m_w == ndFloat32(0.0f));
	ndInt32 hasHit = 0;

	for (ndNode* node = GetFirst(); node; node = node->GetNext()) {
		const ndConvexHull3dFace* const face = &node->GetInfo();

		ndInt32 i0 = face->m_index[0];
		ndInt32 i1 = face->m_index[1];
		ndInt32 i2 = face->m_index[2];

		const ndBigVector& p0 = m_points[i0];
		dAssert(p0.m_w == ndFloat32(0.0f));
		ndBigVector normal ((m_points[i1] - p0).CrossProduct(m_points[i2] - p0));

		dAssert(normal.m_w == ndFloat32(0.0f));
		dAssert(localP0.m_w == ndFloat32(0.0f));

		//ndFloat64 N = -((localP0 - p0) % normal);
		ndFloat64 D =  normal.DotProduct(dS).GetScalar();
		ndFloat64 N = -normal.DotProduct(localP0 - p0).GetScalar();

		if (fabs(D) < ndFloat64 (1.0e-12f)) { //
			if (N < ndFloat64 (0.0f)) {
				return ndFloat64 (1.2f);
			} else {
				continue;
			}
		}

		ndFloat64 t = N / D;
		if (D < ndFloat64 (0.0f)) {
			if (t > tE) {
				tE = t;
				hasHit = 1;
			}
			if (tE > tL) {
				return ndFloat64 (1.2f);
			}
		} else {
			dAssert (D >= ndFloat64 (0.0f));
			tL = dMin (tL, t);
			if (tL < tE) {
				return ndFloat64 (1.2f);
			}
		}
	}

	if (hasHit) {
		interset = tE;
	}

	return interset;
}

void ndConvexHull3d::Save (const char* const filename) const
{
	FILE* const file = fopen(filename, "wb");
	ndInt32 index = 0;
//	fprintf(file, "final\n");
	for (ndNode* nodePtr = GetFirst(); nodePtr; nodePtr = nodePtr->GetNext()) {
		fprintf(file, "triangle %d\n", index);
		index++;
		const ndConvexHull3dFace& face = nodePtr->GetInfo();
		const ndBigVector& p0 = m_points[face.m_index[0]];
		const ndBigVector& p1 = m_points[face.m_index[1]];
		const ndBigVector& p2 = m_points[face.m_index[2]];

		fprintf(file, "p0(%f %f %f)\n", p0[0], p0[1], p0[2]);
		fprintf(file, "p1(%f %f %f)\n", p1[0], p1[1], p1[2]);
		fprintf(file, "p2(%f %f %f)\n", p2[0], p2[1], p2[2]);
	}
	fprintf(file, "\n");

	fclose(file);
}