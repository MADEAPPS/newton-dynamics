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
class dConvexHull3d::dNormalMap
{
	public:
	dNormalMap()
		:m_count(sizeof(m_normal) / sizeof(m_normal[0]))
	{
		dVector p0(dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
		dVector p1(dFloat32(-1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f));
		dVector p2(dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f));
		dVector p3(dFloat32(0.0f), dFloat32(-1.0f), dFloat32(0.0f), dFloat32(0.0f));
		dVector p4(dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f));
		dVector p5(dFloat32(0.0f), dFloat32(0.0f), dFloat32(-1.0f), dFloat32(0.0f));

		dInt32 count = 0;
		dInt32 subdivitions = 2;
		TessellateTriangle(subdivitions, p4, p0, p2, count);
		TessellateTriangle(subdivitions, p0, p5, p2, count);
		TessellateTriangle(subdivitions, p5, p1, p2, count);
		TessellateTriangle(subdivitions, p1, p4, p2, count);
		TessellateTriangle(subdivitions, p0, p4, p3, count);
		TessellateTriangle(subdivitions, p5, p0, p3, count);
		TessellateTriangle(subdivitions, p1, p5, p3, count);
		TessellateTriangle(subdivitions, p4, p1, p3, count);
	}

	static const dNormalMap& GetNormaMap()
	{
		static dNormalMap normalMap;
		return normalMap;
	}

	void TessellateTriangle(dInt32 level, const dVector& p0, const dVector& p1, const dVector& p2, dInt32& count)
	{
		if (level) {
			dAssert(dAbs(p0.DotProduct(p0).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
			dAssert(dAbs(p1.DotProduct(p1).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
			dAssert(dAbs(p2.DotProduct(p2).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
			dVector p01(p0 + p1);
			dVector p12(p1 + p2);
			dVector p20(p2 + p0);

			p01 = p01.Scale(dRsqrt(p01.DotProduct(p01).GetScalar()));
			p12 = p12.Scale(dRsqrt(p12.DotProduct(p12).GetScalar()));
			p20 = p20.Scale(dRsqrt(p20.DotProduct(p20).GetScalar()));

			dAssert(dAbs(p01.DotProduct(p01).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
			dAssert(dAbs(p12.DotProduct(p12).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
			dAssert(dAbs(p20.DotProduct(p20).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));

			TessellateTriangle(level - 1, p0, p01, p20, count);
			TessellateTriangle(level - 1, p1, p12, p01, count);
			TessellateTriangle(level - 1, p2, p20, p12, count);
			TessellateTriangle(level - 1, p01, p12, p20, count);
		} else {
			dBigPlane n(p0, p1, p2);
			n = n.Scale(dFloat64(1.0f) / sqrt(n.DotProduct(n).GetScalar()));
			n.m_w = dFloat64(0.0f);
			dInt32 index = dBitReversal(count, sizeof(m_normal) / sizeof(m_normal[0]));
			m_normal[index] = n;
			count++;
			dAssert(count <= dInt32 (sizeof(m_normal) / sizeof(m_normal[0])));
		}
	}

	dBigVector m_normal[128];
	dInt32 m_count;
};
#endif

class dConvexHull3dVertex: public dBigVector
{
	public:
	dInt32 m_mark;
};

class dConvexHull3dAABBTreeNode
{
	public:
	dBigVector m_box[2];
	dConvexHull3dAABBTreeNode* m_left;
	dConvexHull3dAABBTreeNode* m_right;
	dConvexHull3dAABBTreeNode* m_parent;
};

class dgConvexHull3dPointCluster: public dConvexHull3dAABBTreeNode
{
	public:
	dInt32 m_count;
	dInt32 m_indices[DG_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE];
};


dConvexHull3dFace::dConvexHull3dFace()
{
	m_mark = 0;
	m_twin[0] = nullptr;
	m_twin[1] = nullptr;
	m_twin[2] = nullptr;
}

dFloat64 dConvexHull3dFace::Evalue (const dBigVector* const pointArray, const dBigVector& point) const
{
	const dBigVector& p0 = pointArray[m_index[0]];
	const dBigVector& p1 = pointArray[m_index[1]];
	const dBigVector& p2 = pointArray[m_index[2]];

	dFloat64 matrix[3][3];
	for (dInt32 i = 0; i < 3; i ++) {
		matrix[0][i] = p2[i] - p0[i];
		matrix[1][i] = p1[i] - p0[i];
		matrix[2][i] = point[i] - p0[i];
	}

	dFloat64 error;
	dFloat64 det = Determinant3x3 (matrix, &error);

	// the code use double, however the threshold for accuracy test is the machine precision of a float.
	// by changing this to a smaller number, the code should run faster since many small test will be considered valid
	// the precision must be a power of two no smaller than the machine precision of a double, (1<<48)
	// float64(1<<30) can be a good value

	// dFloat64 precision	= dFloat64 (1.0f) / dFloat64 (1<<30);
	dFloat64 precision	 = dFloat64 (1.0f) / dFloat64 (1<<24);
	dFloat64 errbound = error * precision;
	if (fabs(det) > errbound) {
		return det;
	}

	dGoogol exactMatrix[3][3];
	for (dInt32 i = 0; i < 3; i ++) {
		exactMatrix[0][i] = dGoogol(p2[i]) - dGoogol(p0[i]);
		exactMatrix[1][i] = dGoogol(p1[i]) - dGoogol(p0[i]);
		exactMatrix[2][i] = dGoogol(point[i]) - dGoogol(p0[i]);
	}
	return Determinant3x3(exactMatrix);
}

dBigPlane dConvexHull3dFace::GetPlaneEquation (const dBigVector* const pointArray) const
{
	const dBigVector& p0 = pointArray[m_index[0]];
	const dBigVector& p1 = pointArray[m_index[1]];
	const dBigVector& p2 = pointArray[m_index[2]];
	dBigPlane plane (p0, p1, p2);
	plane = plane.Scale (1.0f / sqrt (plane.DotProduct(plane & dBigVector::m_triplexMask).GetScalar()));
	return plane;
}


dConvexHull3d::dConvexHull3d ()
	:dList<dConvexHull3dFace>()
	,m_aabbP0(dBigVector (dFloat64 (0.0f)))
	,m_aabbP1(dBigVector (dFloat64 (0.0f)))
	,m_count(0)
	,m_diag()
	,m_points()
{
}

dConvexHull3d::dConvexHull3d(const dConvexHull3d& source)
	:dList<dConvexHull3dFace>()
	,m_aabbP0 (source.m_aabbP0)
	,m_aabbP1 (source.m_aabbP1)
	,m_count(source.m_count)
	,m_diag(source.m_diag)
	,m_points(source.m_count)
{
	m_points.SetCount(source.m_count);
	m_points[m_count-1].m_w = dFloat64 (0.0f);
	for (dInt32 i = 0; i < m_count; i ++) 
	{
		m_points[i] = source.m_points[i];
	}
	dTree<dNode*, dNode*> map;
	for(dNode* sourceNode = source.GetFirst(); sourceNode; sourceNode = sourceNode->GetNext() ) 
	{
		dNode* const node = Append();
		map.Insert(node, sourceNode);
	}

	for(dNode* sourceNode = source.GetFirst(); sourceNode; sourceNode = sourceNode->GetNext() ) {
		dNode* const node = map.Find(sourceNode)->GetInfo();

		dConvexHull3dFace& face = node->GetInfo();
		dConvexHull3dFace& srcFace = sourceNode->GetInfo();

		face.m_mark = 0;
		for (dInt32 i = 0; i < 3; i ++) {
			face.m_index[i] = srcFace.m_index[i];
			face.m_twin[i] = map.Find (srcFace.m_twin[i])->GetInfo();
		}
	}
}

dConvexHull3d::dConvexHull3d(const dFloat64* const vertexCloud, dInt32 strideInBytes, dInt32 count, dFloat64 distTol, dInt32 maxVertexCount)
	:dList<dConvexHull3dFace>()
	,m_aabbP0(dBigVector::m_zero)
	,m_aabbP1(dBigVector::m_zero)
	,m_count(0)
	,m_diag()
	,m_points()
{
	BuildHull (vertexCloud, strideInBytes, count, distTol, maxVertexCount);
}

dConvexHull3d::~dConvexHull3d(void)
{
}

void dConvexHull3d::BuildHull (const dFloat64* const vertexCloud, dInt32 strideInBytes, dInt32 count, dFloat64 distTol, dInt32 maxVertexCount)
{
	dSetPrecisionDouble precision;

	dInt32 treeCount = count / (DG_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE>>1);
	if (treeCount < 4) 
	{
		treeCount = 4;
	}
	treeCount *= 2;

	dStack<dConvexHull3dVertex> points (count);
	dStack<dgConvexHull3dPointCluster> treePool (treeCount + 256);
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

dConvexHull3dAABBTreeNode* dConvexHull3d::BuildTree (dConvexHull3dAABBTreeNode* const parent, dConvexHull3dVertex* const points, dInt32 count, dInt32 baseIndex, dInt8** memoryPool, dInt32& maxMemSize) const
{
	dConvexHull3dAABBTreeNode* tree = nullptr;

	dAssert (count);
	dBigVector minP ( dFloat32 (1.0e15f));
	dBigVector maxP (-dFloat32 (1.0e15f));
	if (count <= DG_CONVEXHULL_3D_VERTEX_CLUSTER_SIZE) 
	{
		dgConvexHull3dPointCluster* const clump = new (*memoryPool) dgConvexHull3dPointCluster;
		*memoryPool += sizeof (dgConvexHull3dPointCluster);
		maxMemSize -= sizeof (dgConvexHull3dPointCluster);
		dAssert (maxMemSize >= 0);

		dAssert (clump);
		clump->m_count = count;
		for (dInt32 i = 0; i < count; i ++) 
		{
			clump->m_indices[i] = i + baseIndex;

			const dBigVector& p = points[i];
			dAssert(p.m_w == dFloat32(0.0f));
			minP = minP.GetMin(p);
			maxP = maxP.GetMax(p);
		}

		clump->m_left = nullptr;
		clump->m_right = nullptr;
		tree = clump;

	} 
	else 
	{
		dBigVector median (dBigVector::m_zero);
		dBigVector varian (dBigVector::m_zero);
		for (dInt32 i = 0; i < count; i ++) 
		{
			const dBigVector& p = points[i];
			dAssert(p.m_w == dFloat32(0.0f));
			minP = minP.GetMin(p);
			maxP = maxP.GetMax(p);
			median += p;
			varian += p * p;
		}

		varian = varian.Scale(dFloat32(count)) - median * median;
		dInt32 index = 0;
		dFloat64 maxVarian = dFloat64 (-1.0e10f);
		for (dInt32 i = 0; i < 3; i ++) 
		{
			if (varian[i] > maxVarian) 
			{
				index = i;
				maxVarian = varian[i];
			}
		}
		dBigVector center (median.Scale (dFloat64 (1.0f) / dFloat64 (count)));

		dFloat64 test = center[index];

		dInt32 i0 = 0;
		dInt32 i1 = count - 1;
		do 
		{
			for (; i0 <= i1; i0 ++) 
			{
				dFloat64 val = points[i0][index];
				if (val > test) 
				{
					break;
				}
			}

			for (; i1 >= i0; i1 --) 
			{
				dFloat64 val = points[i1][index];
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

		tree = new (*memoryPool) dConvexHull3dAABBTreeNode;
		*memoryPool += sizeof (dConvexHull3dAABBTreeNode);
		maxMemSize -= sizeof (dConvexHull3dAABBTreeNode);
		dAssert (maxMemSize >= 0);

		dAssert (i0);
		dAssert (count - i0);

		tree->m_left = BuildTree (tree, points, i0, baseIndex, memoryPool, maxMemSize);
		tree->m_right = BuildTree (tree, &points[i0], count - i0, i0 + baseIndex, memoryPool, maxMemSize);
	}

	dAssert (tree);
	tree->m_parent = parent;
	tree->m_box[0] = (minP - dBigVector (dFloat64 (1.0e-3f))) & dBigVector::m_triplexMask;
	tree->m_box[1] = (maxP + dBigVector (dFloat64 (1.0e-3f))) & dBigVector::m_triplexMask;
	return tree;
}

dInt32 dConvexHull3d::GetUniquePoints(dConvexHull3dVertex* const points, const dFloat64* const vertexCloud, dInt32 strideInBytes, dInt32 count, void* const, dInt32)
{
	const dInt32 stride = dInt32(strideInBytes / sizeof(dFloat64));
	//if (stride >= 4) 
	//{
	//	for (dInt32 i = 0; i < count; i++) 
	//	{
	//		dInt32 index = i * stride;
	//		dBigVector& vertex = points[i];
	//		vertex = dBigVector(vertexCloud[index], vertexCloud[index + 1], vertexCloud[index + 2], vertexCloud[index + 3]);
	//		dAssert(dCheckVector(vertex));
	//		points[i].m_mark = 0;
	//	}
	//} 
	//else 
	//{
	//	for (dInt32 i = 0; i < count; i++) 
	//	{
	//		dInt32 index = i * stride;
	//		dBigVector& vertex = points[i];
	//		vertex = dBigVector(vertexCloud[index], vertexCloud[index + 1], vertexCloud[index + 2], dFloat64(0.0f));
	//		dAssert(dCheckVector(vertex));
	//		points[i].m_mark = 0;
	//	}
	//}

	for (dInt32 i = 0; i < count; i++) 
	{
		dInt32 index = i * stride;
		dBigVector& vertex = points[i];
		vertex = dBigVector(vertexCloud[index], vertexCloud[index + 1], vertexCloud[index + 2], dFloat64(0.0f));
		dAssert(dCheckVector(vertex));
		points[i].m_mark = 0;
	}

	class CompareVertex
	{
		public:
		dInt32 Compare(const dConvexHull3dVertex& elementA, const dConvexHull3dVertex& elementB, void* const) const
		{
			for (dInt32 i = 0; i < 3; i++) 
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
	dSort<dConvexHull3dVertex, CompareVertex>(points, count);

	dInt32 indexCount = 0;
	CompareVertex compareVetex;
	for (dInt32 i = 1; i < count; i++) 
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

dInt32 dConvexHull3d::InitVertexArray(dConvexHull3dVertex* const points, const dFloat64* const vertexCloud, dInt32 strideInBytes, dInt32 count, void* const memoryPool, dInt32 maxMemSize)
{
	count = GetUniquePoints(points, vertexCloud, strideInBytes, count, memoryPool, maxMemSize);
	if (count < 4) 
	{
		m_count = 0;
		return count;
	}
	dConvexHull3dAABBTreeNode* tree = BuildTree (nullptr, points, count, 0, (dInt8**) &memoryPool, maxMemSize);

	m_points.SetCount(count);
	m_aabbP0 = tree->m_box[0];
	m_aabbP1 = tree->m_box[1];

	dBigVector boxSize (tree->m_box[1] - tree->m_box[0]);
	dAssert (boxSize.m_w == dFloat32 (0.0f));
	m_diag = dFloat32 (sqrt (boxSize.DotProduct(boxSize).GetScalar()));

#ifdef D_OLD_CONVEXHULL_3D
	const dNormalMap& normalMap = dNormalMap::GetNormaMap();

	dInt32 index0 = SupportVertex (&tree, points, normalMap.m_normal[0]);
	m_points[0] = points[index0];
	points[index0].m_mark = 1;

	bool validTetrahedrum = false;
	dBigVector e1 (dBigVector::m_zero);
	for (dInt32 i = 1; i < normalMap.m_count; i ++) 
	{
		dInt32 index = SupportVertex (&tree, points, normalMap.m_normal[i]);
		dAssert (index >= 0);

		e1 = points[index] - m_points[0];
		dAssert (e1.m_w == dFloat32 (0.0f));
		dFloat64 error2 = e1.DotProduct(e1).GetScalar();
		if (error2 > (dFloat32 (1.0e-4f) * m_diag * m_diag)) 
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
	dBigVector e2(dBigVector::m_zero);
	dBigVector normal (dBigVector::m_zero);
	for (dInt32 i = 2; i < normalMap.m_count; i ++) 
	{
		dInt32 index = SupportVertex (&tree, points, normalMap.m_normal[i]);
		dAssert (index >= 0);
		e2 = points[index] - m_points[0];
		normal = e1.CrossProduct(e2);
		dAssert (e2.m_w == dFloat32 (0.0f));
		dAssert (normal.m_w == dFloat32 (0.0f));
		dFloat64 error2 = sqrt (normal.DotProduct(normal).GetScalar());
		if (error2 > (dFloat32 (1.0e-4f) * m_diag * m_diag)) 
		{
			m_points[2] = points[index];
			points[index].m_mark = 1;
			validTetrahedrum = true;
			break;
		}
	}

	dAssert(normal.m_w == dFloat32(0.0f));
	if (!validTetrahedrum) 
	{
		m_count = 0;
		dAssert (0);
		return count;
	}

	// find the largest possible tetrahedron
	validTetrahedrum = false;
	dBigVector e3(dBigVector::m_zero);

	index0 = SupportVertex (&tree, points, normal);
	e3 = points[index0] - m_points[0];
	dAssert (e3.m_w == dFloat32 (0.0f));
	dFloat64 err2 = normal.DotProduct(e3).GetScalar();
	if (fabs (err2) > (dFloat64 (1.0e-6f) * m_diag * m_diag)) 
	{
		// we found a valid tetrahedral, about and start build the hull by adding the rest of the points
		m_points[3] = points[index0];
		points[index0].m_mark = 1;
		validTetrahedrum = true;
	}
	if (!validTetrahedrum) 
	{
		dVector n (normal.Scale(dFloat64 (-1.0f)));
		dInt32 index = SupportVertex (&tree, points, n);
		e3 = points[index] - m_points[0];
		dAssert (e3.m_w == dFloat32 (0.0f));
		dFloat64 error2 = normal.DotProduct(e3).GetScalar();
		if (fabs (error2) > (dFloat64 (1.0e-6f) * m_diag * m_diag)) 
		{
			// we found a valid tetrahedral, about and start build the hull by adding the rest of the points
			m_points[3] = points[index];
			points[index].m_mark = 1;
			validTetrahedrum = true;
		}
	}
	if (!validTetrahedrum) 
	{
		for (dInt32 i = 3; i < normalMap.m_count; i ++) 
		{
			dInt32 index = SupportVertex (&tree, points, normalMap.m_normal[i]);
			dAssert (index >= 0);

			//make sure the volume of the fist tetrahedral is no negative
			e3 = points[index] - m_points[0];
			dAssert (e3.m_w == dFloat32 (0.0f));
			dFloat64 error2 = normal.DotProduct(e3).GetScalar();
			if (fabs (error2) > (dFloat64 (1.0e-6f) * m_diag * m_diag)) 
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
	dFloat64 volume = TetrahedrumVolume (m_points[0], m_points[1], m_points[2], m_points[3]);
	if (volume > dFloat64 (0.0f)) 
	{
		dSwap(m_points[2], m_points[3]);
	}
	dAssert (TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]) < dFloat64(0.0f));

	return count;
#else
	
	dBigVector origin((m_aabbP1 + m_aabbP0).Scale (0.5f));

	dBigVector dir(m_aabbP1 - m_aabbP0);
	dAssert(dir.DotProduct3(dir) > dFloat32(1.0e-4f));
	dir = dir.Normalize();
	dInt32 index0 = SupportVertex(&tree, points, dir);
	m_points[0] = points[index0];
	points[index0].m_mark = 1;

	dir = origin - m_points[0];
	dAssert(dir.DotProduct3(dir) > dFloat32(1.0e-4f));
	dir = dir.Normalize();
	dInt32 index1 = SupportVertex(&tree, points, dir);
	m_points[1] = points[index1];
	points[index1].m_mark = 1;

	dBigVector e0(m_points[1] - m_points[0]);
	dAssert(e0.DotProduct3(e0) > dFloat32(1.0e-4f));
	dFloat64 t = -e0.DotProduct3(origin - m_points[0]) / e0.DotProduct3(e0);
	dir = m_points[0] + e0.Scale(t) - origin;

	dAssert(dir.DotProduct3(dir) > dFloat32(1.0e-4f));
	dir = dir.Normalize();
	dInt32 index2 = SupportVertex(&tree, points, dir);
	m_points[2] = points[index2];
	points[index2].m_mark = 1;

	dBigVector e1 (m_points[2] - m_points[0]);
	dBigVector normal (e1.CrossProduct(e0));
	dFloat64 error2 = sqrt(normal.DotProduct3(normal));
	if (error2 < (dFloat32(1.0e-4f) * m_diag * m_diag)) 
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

dFloat64 dConvexHull3d::TetrahedrumVolume (const dBigVector& p0, const dBigVector& p1, const dBigVector& p2, const dBigVector& p3) const
{
	dBigVector p1p0 (p1 - p0);
	dBigVector p2p0 (p2 - p0);
	dBigVector p3p0 (p3 - p0);
	dAssert (p1p0.m_w == dFloat32 (0.0f));
	dAssert (p2p0.m_w == dFloat32 (0.0f));
	dAssert (p3p0.m_w == dFloat32 (0.0f));
	return p3p0.DotProduct(p1p0.CrossProduct(p2p0)).GetScalar();
}

dInt32 dConvexHull3d::SupportVertex (dConvexHull3dAABBTreeNode** const treePointer, const dConvexHull3dVertex* const points, const dBigVector& dirPlane, const bool removeEntry) const
{
	#define DG_STACK_DEPTH_3D 64
	dFloat64 aabbProjection[DG_STACK_DEPTH_3D];
	const dConvexHull3dAABBTreeNode *stackPool[DG_STACK_DEPTH_3D];

	dBigVector dir(dirPlane & dBigPlane::m_triplexMask);
	dAssert (dir.m_w == dFloat32 (0.0f));

	dInt32 index = -1;
	dInt32 stack = 1;
	stackPool[0] = *treePointer;
	aabbProjection[0] = dFloat32 (1.0e20f);
	dFloat64 maxProj = dFloat64 (-1.0e20f);
	dInt32 ix = (dir[0] > dFloat64 (0.0f)) ? 1 : 0;
	dInt32 iy = (dir[1] > dFloat64 (0.0f)) ? 1 : 0;
	dInt32 iz = (dir[2] > dFloat64 (0.0f)) ? 1 : 0;
	while (stack) 
	{
		stack--;
		dFloat64 boxSupportValue = aabbProjection[stack];
		if (boxSupportValue > maxProj) 
		{
			const dConvexHull3dAABBTreeNode* const me = stackPool[stack];

			if (me->m_left && me->m_right) 
			{
				dBigVector leftSupportPoint (me->m_left->m_box[ix].m_x, me->m_left->m_box[iy].m_y, me->m_left->m_box[iz].m_z, dFloat32 (0.0f));
				dFloat64 leftSupportDist = leftSupportPoint.DotProduct(dir).GetScalar();

				dBigVector rightSupportPoint (me->m_right->m_box[ix].m_x, me->m_right->m_box[iy].m_y, me->m_right->m_box[iz].m_z, dFloat32 (0.0f));
				dFloat64 rightSupportDist = rightSupportPoint.DotProduct(dir).GetScalar();

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
				dgConvexHull3dPointCluster* const cluster = (dgConvexHull3dPointCluster*) me;
				for (dInt32 i = 0; i < cluster->m_count; i ++) 
				{
					const dConvexHull3dVertex& p = points[cluster->m_indices[i]];
					dAssert (p.m_x >= cluster->m_box[0].m_x);
					dAssert (p.m_x <= cluster->m_box[1].m_x);
					dAssert (p.m_y >= cluster->m_box[0].m_y);
					dAssert (p.m_y <= cluster->m_box[1].m_y);
					dAssert (p.m_z >= cluster->m_box[0].m_z);
					dAssert (p.m_z <= cluster->m_box[1].m_z);
					if (!p.m_mark) 
					{
						dAssert (p.m_w == dFloat32 (0.0f));
						dFloat64 dist = p.DotProduct(dir).GetScalar();
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
					dConvexHull3dAABBTreeNode* const parent = cluster->m_parent;
					if (parent) 
					{
						dConvexHull3dAABBTreeNode* const sibling = (parent->m_left != cluster) ? parent->m_left : parent->m_right;
						dAssert (sibling != cluster);
						dConvexHull3dAABBTreeNode* const grandParent = parent->m_parent;
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

dConvexHull3d::dNode* dConvexHull3d::AddFace (dInt32 i0, dInt32 i1, dInt32 i2)
{
	dNode* const node = Append();
	dConvexHull3dFace& face = node->GetInfo();

	face.m_index[0] = i0;
	face.m_index[1] = i1;
	face.m_index[2] = i2;
	return node;
}

void dConvexHull3d::DeleteFace (dNode* const node)
{
	Remove (node);
}

bool dConvexHull3d::Sanity() const
{
/*
	for (dNode* node = GetFirst(); node; node = node->GetNext()) {
		dConvexHull3dFace* const face = &node->GetInfo();
		for (dInt32 i = 0; i < 3; i ++) {
			dNode* const twinNode = face->m_twin[i];
			if (!twinNode) {
				return false;
			}

			dInt32 count = 0;
			dNode* me = nullptr;
			dConvexHull3dFace* const twinFace = &twinNode->GetInfo();
			for (dInt32 j = 0; j < 3; j ++) {
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

bool dConvexHull3d::CheckFlatSurface(dConvexHull3dAABBTreeNode* tree, dConvexHull3dVertex* const points, dInt32, dFloat64, dInt32)
{
	dBigVector e0(m_points[1] - m_points[0]);
	dBigVector e1(m_points[2] - m_points[0]);
	dAssert(e0.m_w == dFloat32(0.0f));
	dAssert(e1.m_w == dFloat32(0.0f));
	dAssert(e0.DotProduct(e0).GetScalar() > dFloat32(1.0e-4f));
	dAssert(e1.DotProduct(e1).GetScalar() > dFloat32(1.0e-4f));
	dBigVector normal(e1.CrossProduct(e0));
	dAssert(normal.m_w == dFloat32(0.0f));
	dAssert(normal.DotProduct(normal).GetScalar() > dFloat32(1.0e-6f));
	normal = normal.Normalize();

	dInt32 index = SupportVertex(&tree, points, normal);
	m_points[3] = points[index];

	dFloat64 volume = TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]);
	if (dAbs(volume) < dFloat32(1.0e-9f)) {
		normal = normal.Scale(dFloat32(-1.0f));
		index = SupportVertex(&tree, points, normal);
		m_points[3] = points[index];
		volume = TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]);
		if (dAbs(volume) < dFloat32(1.0e-9f)) {
			return true;
		}
	}
	points[index].m_mark = 1;
	if (volume > dFloat64(0.0f)) {
		dSwap(m_points[2], m_points[3]);
	}
	dAssert(TetrahedrumVolume(m_points[0], m_points[1], m_points[2], m_points[3]) < dFloat64(0.0f));
	m_count = 4;
	return false;
}


void dConvexHull3d::CalculateConvexHull2d(dConvexHull3dAABBTreeNode*, dConvexHull3dVertex* const, dInt32, dFloat64, dInt32)
{

}

void dConvexHull3d::CalculateConvexHull3d (dConvexHull3dAABBTreeNode* vertexTree, dConvexHull3dVertex* const points, dInt32 count, dFloat64 distTol, dInt32 maxVertexCount)
{
	distTol = dAbs (distTol) * m_diag;
	dNode* const f0Node = AddFace (0, 1, 2);
	dNode* const f1Node = AddFace (0, 2, 3);
	dNode* const f2Node = AddFace (2, 1, 3);
	dNode* const f3Node = AddFace (1, 0, 3);

	dConvexHull3dFace* const f0 = &f0Node->GetInfo();
	dConvexHull3dFace* const f1 = &f1Node->GetInfo();
	dConvexHull3dFace* const f2 = &f2Node->GetInfo();
	dConvexHull3dFace* const f3 = &f3Node->GetInfo();

	f0->m_twin[0] = (dList<dConvexHull3dFace>::dNode*)f3Node;
	f0->m_twin[1] = (dList<dConvexHull3dFace>::dNode*)f2Node;
	f0->m_twin[2] = (dList<dConvexHull3dFace>::dNode*)f1Node;

	f1->m_twin[0] = (dList<dConvexHull3dFace>::dNode*)f0Node;
	f1->m_twin[1] = (dList<dConvexHull3dFace>::dNode*)f2Node;
	f1->m_twin[2] = (dList<dConvexHull3dFace>::dNode*)f3Node;

	f2->m_twin[0] = (dList<dConvexHull3dFace>::dNode*)f0Node;
	f2->m_twin[1] = (dList<dConvexHull3dFace>::dNode*)f3Node;
	f2->m_twin[2] = (dList<dConvexHull3dFace>::dNode*)f1Node;

	f3->m_twin[0] = (dList<dConvexHull3dFace>::dNode*)f0Node;
	f3->m_twin[1] = (dList<dConvexHull3dFace>::dNode*)f1Node;
	f3->m_twin[2] = (dList<dConvexHull3dFace>::dNode*)f2Node;

	dList<dNode*> boundaryFaces;

	boundaryFaces.Append(f0Node);
	boundaryFaces.Append(f1Node);
	boundaryFaces.Append(f2Node);
	boundaryFaces.Append(f3Node);
	count -= 4;
	maxVertexCount -= 4;
	dInt32 currentIndex = 4;

	dStack<dNode*> stackPool(1024 + m_count);
	dStack<dNode*> coneListPool(1024 + m_count);
	dStack<dNode*> deleteListPool(1024 + m_count);

	dNode** const stack = &stackPool[0];
	dNode** const coneList = &stackPool[0];
	dNode** const deleteList = &deleteListPool[0];

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
			dNode* const faceNode = boundaryFaces.GetLast()->GetInfo();
		#endif

		dConvexHull3dFace* const face = &faceNode->GetInfo();
		dBigPlane planeEquation (face->GetPlaneEquation (&m_points[0]));

		dInt32 index = SupportVertex (&vertexTree, points, planeEquation);
		const dBigVector& p = points[index];
		dFloat64 dist = planeEquation.Evalue(p);

		if ((dist >= distTol) && (face->Evalue(&m_points[0], p) > dFloat64(0.0f))) 
		{
			dAssert (Sanity());

			dAssert (faceNode);
			stack[0] = faceNode;

			dInt32 stackIndex = 1;
			dInt32 deletedCount = 0;

			while (stackIndex) 
			{
				stackIndex --;
				dNode* const node1 = stack[stackIndex];
				dConvexHull3dFace* const face1 = &node1->GetInfo();

				if (!face1->m_mark && (face1->Evalue(&m_points[0], p) > dFloat64(0.0f))) 
				{
					#ifdef _DEBUG
					for (dInt32 i = 0; i < deletedCount; i ++) 
					{
						dAssert (deleteList[i] != node1);
					}
					#endif

					deleteList[deletedCount] = node1;
					deletedCount ++;
					dAssert (deletedCount < dInt32 (deleteListPool.GetElementsCount()));
					face1->m_mark = 1;
					for (dInt32 i = 0; i < 3; i ++) 
					{
						dNode* const twinNode = (dNode*)face1->m_twin[i];
						dAssert (twinNode);
						dConvexHull3dFace* const twinFace = &twinNode->GetInfo();
						if (!twinFace->m_mark) 
						{
							stack[stackIndex] = twinNode;
							stackIndex ++;
							dAssert (stackIndex < dInt32 (stackPool.GetElementsCount()));
						}
					}
				}
			}

			m_points[currentIndex] = points[index];
			points[index].m_mark = 1;

			dInt32 newCount = 0;
			for (dInt32 i = 0; i < deletedCount; i ++) 
			{
				dNode* const node1 = deleteList[i];
				dConvexHull3dFace* const face1 = &node1->GetInfo();
				dAssert (face1->m_mark == 1);
				for (dInt32 j0 = 0; j0 < 3; j0 ++) 
				{
					dNode* const twinNode = face1->m_twin[j0];
					dConvexHull3dFace* const twinFace = &twinNode->GetInfo();
					if (!twinFace->m_mark) 
					{
						dInt32 j1 = (j0 == 2) ? 0 : j0 + 1;
						dNode* const newNode = AddFace (currentIndex, face1->m_index[j0], face1->m_index[j1]);
						boundaryFaces.Addtop(newNode);

						dConvexHull3dFace* const newFace = &newNode->GetInfo();
						newFace->m_twin[1] = twinNode;
						for (dInt32 k = 0; k < 3; k ++) 
						{
							if (twinFace->m_twin[k] == node1) 
							{
								twinFace->m_twin[k] = newNode;
							}
						}
						coneList[newCount] = newNode;
						newCount ++;
						dAssert (newCount < dInt32 (coneListPool.GetElementsCount()));
					}
				}
			}

			for (dInt32 i = 0; i < newCount - 1; i ++) 
			{
				dNode* const nodeA = coneList[i];
				dConvexHull3dFace* const faceA = &nodeA->GetInfo();
				dAssert (faceA->m_mark == 0);
				for (dInt32 j = i + 1; j < newCount; j ++) {
					dNode* const nodeB = coneList[j];
					dConvexHull3dFace* const faceB = &nodeB->GetInfo();
					dAssert (faceB->m_mark == 0);
					if (faceA->m_index[2] == faceB->m_index[1]) 
					{
						faceA->m_twin[2] = nodeB;
						faceB->m_twin[0] = nodeA;
						break;
					}
				}

				for (dInt32 j = i + 1; j < newCount; j ++) 
				{
					dNode* const nodeB = coneList[j];
					dConvexHull3dFace* const faceB = &nodeB->GetInfo();
					dAssert (faceB->m_mark == 0);
					if (faceA->m_index[1] == faceB->m_index[2]) 
					{
						faceA->m_twin[0] = nodeB;
						faceB->m_twin[2] = nodeA;
						break;
					}
				}
			}

			for (dInt32 i = 0; i < deletedCount; i ++) 
			{
				dNode* const node = deleteList[i];
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


void dConvexHull3d::CalculateVolumeAndSurfaceArea (dFloat64& volume, dFloat64& surfaceArea) const
{
	dFloat64 areaAcc = dFloat32 (0.0f);
	dFloat64  volumeAcc = dFloat32 (0.0f);
	for (dNode* node = GetFirst(); node; node = node->GetNext()) {
		const dConvexHull3dFace* const face = &node->GetInfo();
		dInt32 i0 = face->m_index[0];
		dInt32 i1 = face->m_index[1];
		dInt32 i2 = face->m_index[2];
		const dBigVector& p0 = m_points[i0];
		const dBigVector& p1 = m_points[i1];
		const dBigVector& p2 = m_points[i2];
		dAssert(p0.m_w == dFloat32(0.0f));
		dAssert(p1.m_w == dFloat32(0.0f));
		dAssert(p2.m_w == dFloat32(0.0f));
		dBigVector normal ((p1 - p0).CrossProduct(p2 - p0));
		dAssert(normal.m_w == dFloat32(0.0f));
		dFloat64 area = sqrt (normal.DotProduct(normal).GetScalar());
		areaAcc += area;
		volumeAcc += p2.DotProduct(p0.CrossProduct(p1)).GetScalar();
	}
	dAssert (volumeAcc >= dFloat64 (0.0f));
	volume = volumeAcc * dFloat64 (1.0f/6.0f);
	surfaceArea = areaAcc * dFloat64 (0.5f);
}

// this code has linear time complexity on the number of faces
dFloat64 dConvexHull3d::RayCast (const dBigVector& localP0, const dBigVector& localP1) const
{
	dFloat64 interset = dFloat32 (1.2f);
	dFloat64 tE = dFloat64 (0.0f);	// for the maximum entering segment parameter;
	dFloat64 tL = dFloat64 (1.0f);	// for the minimum leaving segment parameter;
	dBigVector dS (localP1 - localP0); // is the segment direction vector;

	dAssert(dS.m_w == dFloat32(0.0f));
	dInt32 hasHit = 0;

	for (dNode* node = GetFirst(); node; node = node->GetNext()) {
		const dConvexHull3dFace* const face = &node->GetInfo();

		dInt32 i0 = face->m_index[0];
		dInt32 i1 = face->m_index[1];
		dInt32 i2 = face->m_index[2];

		const dBigVector& p0 = m_points[i0];
		dAssert(p0.m_w == dFloat32(0.0f));
		dBigVector normal ((m_points[i1] - p0).CrossProduct(m_points[i2] - p0));

		dAssert(normal.m_w == dFloat32(0.0f));
		dAssert(localP0.m_w == dFloat32(0.0f));

		//dFloat64 N = -((localP0 - p0) % normal);
		dFloat64 D =  normal.DotProduct(dS).GetScalar();
		dFloat64 N = -normal.DotProduct(localP0 - p0).GetScalar();

		if (fabs(D) < dFloat64 (1.0e-12f)) { //
			if (N < dFloat64 (0.0f)) {
				return dFloat64 (1.2f);
			} else {
				continue;
			}
		}

		dFloat64 t = N / D;
		if (D < dFloat64 (0.0f)) {
			if (t > tE) {
				tE = t;
				hasHit = 1;
			}
			if (tE > tL) {
				return dFloat64 (1.2f);
			}
		} else {
			dAssert (D >= dFloat64 (0.0f));
			tL = dMin (tL, t);
			if (tL < tE) {
				return dFloat64 (1.2f);
			}
		}
	}

	if (hasHit) {
		interset = tE;
	}

	return interset;
}

void dConvexHull3d::Save (const char* const filename) const
{
	FILE* const file = fopen(filename, "wb");
	dInt32 index = 0;
//	fprintf(file, "final\n");
	for (dNode* nodePtr = GetFirst(); nodePtr; nodePtr = nodePtr->GetNext()) {
		fprintf(file, "triangle %d\n", index);
		index++;
		const dConvexHull3dFace& face = nodePtr->GetInfo();
		const dBigVector& p0 = m_points[face.m_index[0]];
		const dBigVector& p1 = m_points[face.m_index[1]];
		const dBigVector& p2 = m_points[face.m_index[2]];

		fprintf(file, "p0(%f %f %f)\n", p0[0], p0[1], p0[2]);
		fprintf(file, "p1(%f %f %f)\n", p1[0], p1[1], p1[2]);
		fprintf(file, "p2(%f %f %f)\n", p2[0], p2[1], p2[2]);
	}
	fprintf(file, "\n");

	fclose(file);
}