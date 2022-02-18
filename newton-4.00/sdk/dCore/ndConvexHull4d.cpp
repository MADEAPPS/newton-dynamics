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
#include "ndHeap.h"
#include "ndStack.h"
#include "ndGoogol.h"
#include "ndConvexHull4d.h"
#include "ndSmallDeterminant.h"

#define D_VERTEX_CLUMP_SIZE_4D		8 


ndConvexHull4d::ndNormalMap::ndNormalMap()
	:m_count(sizeof (m_normal) / sizeof (m_normal[0]))
{
	ndVector p0(ndFloat32( 1.0f), ndFloat32( 0.0f), ndFloat32( 0.0f), ndFloat32(0.0f));
	ndVector p1(ndFloat32(-1.0f), ndFloat32( 0.0f), ndFloat32( 0.0f), ndFloat32(0.0f));
	ndVector p2(ndFloat32( 0.0f), ndFloat32( 1.0f), ndFloat32( 0.0f), ndFloat32(0.0f));
	ndVector p3(ndFloat32( 0.0f), ndFloat32(-1.0f), ndFloat32( 0.0f), ndFloat32(0.0f));
	ndVector p4(ndFloat32( 0.0f), ndFloat32( 0.0f), ndFloat32( 1.0f), ndFloat32(0.0f));
	ndVector p5(ndFloat32( 0.0f), ndFloat32( 0.0f), ndFloat32(-1.0f), ndFloat32(0.0f));

	ndInt32 count = 0;
	ndInt32 subdivitions = 2;

	ndBigVector tmp[128];
	TessellateTriangle(subdivitions, p4, p0, p2, tmp, count);
	TessellateTriangle(subdivitions, p0, p5, p2, tmp, count);
	TessellateTriangle(subdivitions, p5, p1, p2, tmp, count);
	TessellateTriangle(subdivitions, p1, p4, p2, tmp, count);
	TessellateTriangle(subdivitions, p0, p4, p3, tmp, count);
	TessellateTriangle(subdivitions, p5, p0, p3, tmp, count);
	TessellateTriangle(subdivitions, p1, p5, p3, tmp, count);
	TessellateTriangle(subdivitions, p4, p1, p3, tmp, count);

	count = 0;
	for (ndInt32 j = 0; j < 8; j++) 
	{
		ndFloat64 beta = (j - 4) * ndFloat64 (22.5f * ndDegreeToRad) + ndFloat64 (10.5f * ndDegreeToRad);
		ndFloat64 sinBeta = sin(beta);
		ndFloat64 cosBeta = cos(beta);

		ndFloat64 w = sinBeta;
		for (ndInt32 i = 0; i < 128; i ++) 
		{
			ndFloat64 z = cosBeta * tmp[i].m_z;
			ndFloat64 y = cosBeta * tmp[i].m_y;
			ndFloat64 x = cosBeta * tmp[i].m_x;
			ndInt32 index = dBitReversal(count, sizeof (m_normal) / sizeof (m_normal[0]));
			dAssert (index < ndInt32 (sizeof (m_normal) / sizeof (m_normal[0])));
			m_normal[index] = ndBigVector (x, y, z, w);
			count ++;
		}
	}
}

void ndConvexHull4d::ndNormalMap::TessellateTriangle(ndInt32 level, const ndVector& p0, const ndVector& p1, const ndVector& p2, ndBigVector* const buffer, ndInt32& count)
{
	dAssert(p0.m_w == ndFloat32(0.0f));
	dAssert(p1.m_w == ndFloat32(0.0f));
	dAssert(p2.m_w == ndFloat32(0.0f));
	if (level) 
	{
		dAssert(dAbs(p0.DotProduct(p0).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
		dAssert(dAbs(p1.DotProduct(p1).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
		dAssert(dAbs(p2.DotProduct(p2).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
		ndVector p01(p0 + p1);
		ndVector p12(p1 + p2);
		ndVector p20(p2 + p0);

		dAssert (p01.m_w == ndFloat32 (0.0f));
		dAssert (p12.m_w == ndFloat32 (0.0f));
		dAssert (p20.m_w == ndFloat32 (0.0f));
		//p01 = p01.Scale(dgRsqrt(p01.DotProduct(p01).GetScalar()));
		//p12 = p12.Scale(dgRsqrt(p12.DotProduct(p12).GetScalar()));
		//p20 = p20.Scale(dgRsqrt(p20.DotProduct(p20).GetScalar()));
		p01 = p01.Normalize();
		p12 = p12.Normalize();
		p20 = p20.Normalize();

		dAssert(dAbs(p01.DotProduct(p01).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
		dAssert(dAbs(p12.DotProduct(p12).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
		dAssert(dAbs(p20.DotProduct(p20).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));

		TessellateTriangle(level - 1, p0, p01, p20, buffer, count);
		TessellateTriangle(level - 1, p1, p12, p01, buffer, count);
		TessellateTriangle(level - 1, p2, p20, p12, buffer, count);
		TessellateTriangle(level - 1, p01, p12, p20, buffer, count);
	} 
	else 
	{
		ndBigPlane n(p0, p1, p2);
		n = n.Scale(ndFloat64(1.0f) / sqrt(n.DotProduct(n).GetScalar()));
		n.m_w = ndFloat64(0.0f);
		ndInt32 index = dBitReversal(count, 128);
		buffer[index] = n;
		dAssert(count < 128);
		count++;
	}
}

class ndConvexHull4dAABBTreeNode
{
	public:
	#ifdef _DEBUG
	ndConvexHull4dAABBTreeNode()
	{
		static ndInt32 id = 0;
		m_id = id;
		id ++;
	}
	ndInt32 m_id;
	#endif

		ndBigVector m_box[2];
		ndConvexHull4dAABBTreeNode* m_left;
		ndConvexHull4dAABBTreeNode* m_right;
		ndConvexHull4dAABBTreeNode* m_parent;
};

class dgConvexHull4dPointCluster: public ndConvexHull4dAABBTreeNode
{
	public:
	ndInt32 m_count;
	ndInt32 m_indices[D_VERTEX_CLUMP_SIZE_4D];
};

ndConvexHull4dTetraherum::ndTetrahedrumPlane::ndTetrahedrumPlane (const ndBigVector& p0, const ndBigVector& p1, const ndBigVector& p2, const ndBigVector& p3)
	:ndBigVector ((p1 - p0).CrossProduct (p2 - p0, p3 - p0))
{
	ndBigVector& me = *this;
	ndFloat64 invMag2 = ndFloat32 (0.0f);
	ndFloat64 val = me.DotProduct(me).m_x;
	if (val > ndFloat64 (1.0e-38)) {
		invMag2 = ndFloat64 (1.0f) / sqrt (val);
	} else {
		invMag2 = ndFloat32 (0.0f);
	}

	me.m_x *= invMag2;
	me.m_y *= invMag2;
	me.m_z *= invMag2;
	me.m_w *= invMag2;
	m_dist = - me.DotProduct(p0).m_x;
}

ndFloat64 ndConvexHull4dTetraherum::ndTetrahedrumPlane::Evalue (const ndBigVector& point) const
{
	const ndBigVector& me = *this;
	return me.DotProduct(point).m_x + m_dist;
}


ndConvexHull4dTetraherum::ndConvexHull4dTetraherum()
{
#ifdef _DEBUG
	static ndInt32 debugID;
	m_debugID = debugID;
	debugID ++;
#endif
	static ndInt32 m_monotonicID;

	m_uniqueID = m_monotonicID;
	m_monotonicID ++;
}

void ndConvexHull4dTetraherum::Init (const ndConvexHull4dVector* const, ndInt32 v0, ndInt32 v1, ndInt32 v2, ndInt32 v3)
{
	m_faces[0].m_index[0] = v0;
	m_faces[0].m_index[1] = v1;
	m_faces[0].m_index[2] = v2;
	m_faces[0].m_index[3] = v3;

	m_faces[1].m_index[0] = v3;
	m_faces[1].m_index[1] = v0;
	m_faces[1].m_index[2] = v2;
	m_faces[1].m_index[3] = v1;

	m_faces[2].m_index[0] = v3;
	m_faces[2].m_index[1] = v2;
	m_faces[2].m_index[2] = v1;
	m_faces[2].m_index[3] = v0;

	m_faces[3].m_index[0] = v3;
	m_faces[3].m_index[1] = v1;
	m_faces[3].m_index[2] = v0;
	m_faces[3].m_index[3] = v2;

	SetMark (0); 
	for (ndInt32 i = 0; i < 4; i ++) {
		m_faces[i].m_twin = nullptr;
	}

#ifdef _DEBUG
	//ndBigVector p1p0 (points[v1] - points[v0]);
	//ndBigVector p2p0 (points[v2] - points[v0]);
	//ndBigVector p3p0 (points[v3] - points[v0]);
	//ndBigVector normal (p1p0.CrossProduct(p2p0, p3p0));
	//ndFloat64 volume = normal.DotProduct(normal).m_x;
	//dAssert (volume > ndFloat64 (0.0f));
#endif
}

ndFloat64 ndConvexHull4dTetraherum::Evalue (const ndConvexHull4dVector* const pointArray, const ndBigVector& point) const
{
	const ndBigVector &p0 = pointArray[m_faces[0].m_index[0]];
	const ndBigVector &p1 = pointArray[m_faces[0].m_index[1]];
	const ndBigVector &p2 = pointArray[m_faces[0].m_index[2]];
	const ndBigVector &p3 = pointArray[m_faces[0].m_index[3]];

	ndFloat64 matrix[4][4];
	for (ndInt32 i = 0; i < 4; i ++) 
	{
		matrix[0][i] = p1[i] - p0[i];
		matrix[1][i] = p2[i] - p0[i];
		matrix[2][i] = p3[i] - p0[i];
		matrix[3][i] = point[i] - p0[i];
	}

	ndFloat64 error;
	ndFloat64 det = Determinant4x4 (matrix, &error);
	ndFloat64 precision  = ndFloat64 (1.0f) / ndFloat64 (1<<24);
	ndFloat64 errbound = error * precision; 
	if (fabs(det) > errbound) 
	{
		return det;
	}

	ndGoogol exactMatrix[4][4];
	for (ndInt32 i = 0; i < 4; i ++) 
	{
		exactMatrix[0][i] = ndGoogol(p1[i]) - ndGoogol(p0[i]);
		exactMatrix[1][i] = ndGoogol(p2[i]) - ndGoogol(p0[i]);
		exactMatrix[2][i] = ndGoogol(p3[i]) - ndGoogol(p0[i]);
		exactMatrix[3][i] = ndGoogol(point[i]) - ndGoogol(p0[i]);
	}
	return Determinant4x4(exactMatrix);
}

ndFloat64 ndConvexHull4dTetraherum::GetTetraVolume(const ndConvexHull4dVector* const points) const
{
	const ndBigVector &p0 = points[m_faces[0].m_index[0]];
	const ndBigVector &p1 = points[m_faces[0].m_index[1]];
	const ndBigVector &p2 = points[m_faces[0].m_index[2]];
	const ndBigVector &p3 = points[m_faces[0].m_index[3]];

	ndFloat64 matrix[3][3];
	for (ndInt32 i = 0; i < 3; i++) 
	{
		matrix[0][i] = p2[i] - p0[i];
		matrix[1][i] = p1[i] - p0[i];
		matrix[2][i] = p3[i] - p0[i];
	}

	ndFloat64 error;
	ndFloat64 det = Determinant3x3(matrix, &error);

	ndFloat64 precision = ndFloat64(1.0f) / ndFloat64(1 << 24);
	ndFloat64 errbound = error * precision;
	if (fabs(det) > errbound) 
	{
		return det;
	}

	ndGoogol exactMatrix[3][3];
	for (ndInt32 i = 0; i < 3; i++) 
	{
		exactMatrix[0][i] = ndGoogol(p2[i]) - ndGoogol(p0[i]);
		exactMatrix[1][i] = ndGoogol(p1[i]) - ndGoogol(p0[i]);
		exactMatrix[2][i] = ndGoogol(p3[i]) - ndGoogol(p0[i]);
	}
	return Determinant3x3(exactMatrix);
}

ndBigVector ndConvexHull4dTetraherum::CircumSphereCenter (const ndConvexHull4dVector* const pointArray) const
{
	ndGoogol matrix[4][4];

	ndBigVector points[4];
	points[0] = pointArray[m_faces[0].m_index[0]];
	points[1] = pointArray[m_faces[0].m_index[1]];
	points[2] = pointArray[m_faces[0].m_index[2]];
	points[3] = pointArray[m_faces[0].m_index[3]];

	for (ndInt32 i = 0; i < 4; i ++) {
		for (ndInt32 j = 0; j < 3; j ++) {
			matrix[i][j] = ndGoogol (points[i][j]);
		}
		matrix[i][3] = ndGoogol (1.0f);
	}
	ndGoogol det (Determinant4x4(matrix));
	ndFloat64 invDen = ndFloat64 (1.0f) / (ndFloat64(det) * ndFloat64 (2.0f));

	ndBigVector centerOut;
	ndFloat64 sign = ndFloat64 (1.0f);
	for (ndInt32 k = 0; k < 3; k ++) {
		for (ndInt32 i = 0; i < 4; i ++) {
			matrix[i][0] = ndGoogol (points[i][3]);
			for (ndInt32 j = 0; j < 2; j ++) {
				ndInt32 j1 = (j < k) ? j : j + 1; 
				matrix[i][j + 1] = ndGoogol (points[i][j1]);
			}
			matrix[i][3] = ndGoogol (1.0f);
		}
		ndGoogol det1 (Determinant4x4(matrix));
		ndFloat64 val = ndFloat64 (det1) * sign;
		sign *= ndFloat64 (-1.0f);
		centerOut[k] = val * invDen; 
	}
	centerOut[3] = ndFloat32 (0.0f);
	return centerOut;
}

ndConvexHull4dTetraherum::ndTetrahedrumPlane ndConvexHull4dTetraherum::GetPlaneEquation (const ndConvexHull4dVector* const points) const
{
	const ndBigVector &p0 = points[m_faces[0].m_index[0]];
	const ndBigVector &p1 = points[m_faces[0].m_index[1]];
	const ndBigVector &p2 = points[m_faces[0].m_index[2]];
	const ndBigVector &p3 = points[m_faces[0].m_index[3]];
	return ndTetrahedrumPlane (p0, p1, p2, p3);
}

ndConvexHull4d::ndConvexHull4d ()
	:ndList<ndConvexHull4dTetraherum>()
	,m_mark(0)
	,m_count(0)
	,m_diag(ndFloat32 (0.0f))
	,m_points() 
{
}

ndConvexHull4d::ndConvexHull4d(const ndConvexHull4d&)
	:ndList<ndConvexHull4dTetraherum>()
{
	dAssert(0);
}

ndConvexHull4d::ndConvexHull4d (const ndFloat64* const vertexCloud, ndInt32 strideInBytes, ndInt32 count, ndFloat64 distTol)
	:ndList<ndConvexHull4dTetraherum>()
	,m_mark(0)
	,m_count(0)
	,m_diag(ndFloat32(0.0f))
	,m_points() 
{
	BuildHull (vertexCloud, strideInBytes, count, distTol);
}

ndConvexHull4d::~ndConvexHull4d(void)
{
}

const ndConvexHull4d::ndNormalMap& ndConvexHull4d::GetNormaMap()
{
	static ndNormalMap normalMap;
	return normalMap;
}

void ndConvexHull4d::Save (const char* const filename) const
{
	FILE* const file = fopen (filename, "wb");
	ndInt32 index = 0;
//	fprintf (file, "final\n");
	for (ndNode* nodePtr = GetFirst(); nodePtr; nodePtr = nodePtr->GetNext()) {
		fprintf (file, "tetra %d\n", index);
		index ++;
		const ndConvexHull4dTetraherum& face = nodePtr->GetInfo();
		const ndBigVector& p0 = m_points[face.m_faces[0].m_index[0]];
		const ndBigVector& p1 = m_points[face.m_faces[0].m_index[1]];
		const ndBigVector& p2 = m_points[face.m_faces[0].m_index[2]];
		const ndBigVector& p3 = m_points[face.m_faces[0].m_index[3]];
		fprintf (file, "p0(%f %f %f %f)\n", p0[0], p0[1], p0[2], p0[3]);
		fprintf (file, "p1(%f %f %f %f)\n", p1[0], p1[1], p1[2], p1[3]);
		fprintf (file, "p2(%f %f %f %f)\n", p2[0], p2[1], p2[2], p2[3]);
		fprintf (file, "p3(%f %f %f %f)\n", p3[0], p3[1], p3[2], p3[3]);
	}
	fprintf (file, "\n");

	fclose (file);
}

ndInt32 ndConvexHull4d::SupportVertex (ndConvexHull4dAABBTreeNode** const treePointer, const ndConvexHull4dVector* const points, const ndBigVector& dir, const bool removeEntry) const
{
	#define DG_STACK_DEPTH_4D	64
	ndFloat64 aabbProjection[DG_STACK_DEPTH_4D];
	const ndConvexHull4dAABBTreeNode *stackPool[DG_STACK_DEPTH_4D];

	ndInt32 index = -1;
	ndInt32 stack = 1;
	stackPool[0] = *treePointer;
	aabbProjection[0] = ndFloat32 (1.0e20f);
	ndFloat64 maxProj = ndFloat64 (-1.0e20f); 
	ndInt32 ix = (dir[0] > ndFloat64 (0.0f)) ? 1 : 0;
	ndInt32 iy = (dir[1] > ndFloat64 (0.0f)) ? 1 : 0;
	ndInt32 iz = (dir[2] > ndFloat64 (0.0f)) ? 1 : 0;
	ndInt32 iw = (dir[3] > ndFloat64 (0.0f)) ? 1 : 0;
	while (stack) 
	{
		stack--;
		ndFloat64 boxSupportValue = aabbProjection[stack];
		if (boxSupportValue > maxProj) 
		{
			const ndConvexHull4dAABBTreeNode* const me = stackPool[stack];

			if (me->m_left && me->m_right) 
			{
				ndBigVector leftSupportPoint (me->m_left->m_box[ix].m_x, me->m_left->m_box[iy].m_y, me->m_left->m_box[iz].m_z, me->m_left->m_box[iw].m_w);
				ndFloat64 leftSupportDist = leftSupportPoint.DotProduct(dir).m_x;

				ndBigVector rightSupportPoint (me->m_right->m_box[ix].m_x, me->m_right->m_box[iy].m_y, me->m_right->m_box[iz].m_z, me->m_right->m_box[iw].m_w);
				ndFloat64 rightSupportDist = rightSupportPoint.DotProduct(dir).m_x;

				if (rightSupportDist >= leftSupportDist) 
				{
					aabbProjection[stack] = leftSupportDist;
					stackPool[stack] = me->m_left;
					stack++;
					dAssert (stack < DG_STACK_DEPTH_4D);
					aabbProjection[stack] = rightSupportDist;
					stackPool[stack] = me->m_right;
					stack++;
					dAssert (stack < DG_STACK_DEPTH_4D);
				} 
				else 
				{
					aabbProjection[stack] = rightSupportDist;
					stackPool[stack] = me->m_right;
					stack++;
					dAssert (stack < DG_STACK_DEPTH_4D);
					aabbProjection[stack] = leftSupportDist;
					stackPool[stack] = me->m_left;
					stack++;
					dAssert (stack < DG_STACK_DEPTH_4D);
				}
			} 
			else 
			{
				dgConvexHull4dPointCluster* const cluster = (dgConvexHull4dPointCluster*) me;
				for (ndInt32 i = 0; i < cluster->m_count; i ++) 
				{
					const ndConvexHull4dVector& p = points[cluster->m_indices[i]];
					dAssert (p.m_x >= cluster->m_box[0].m_x);
					dAssert (p.m_x <= cluster->m_box[1].m_x);
					dAssert (p.m_y >= cluster->m_box[0].m_y);
					dAssert (p.m_y <= cluster->m_box[1].m_y);
					dAssert (p.m_z >= cluster->m_box[0].m_z);
					dAssert (p.m_z <= cluster->m_box[1].m_z);
					dAssert (p.m_w >= cluster->m_box[0].m_w);
					dAssert (p.m_w <= cluster->m_box[1].m_w);
					if (!p.m_mark) 
					{
						ndFloat64 dist = p.DotProduct(dir).m_x;
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
					ndConvexHull4dAABBTreeNode* const parent = cluster->m_parent;
					if (parent) 
					{	
						ndConvexHull4dAABBTreeNode* const sibling = (parent->m_left != cluster) ? parent->m_left : parent->m_right;
						dAssert (sibling != cluster);
						ndConvexHull4dAABBTreeNode* const grandParent = parent->m_parent;
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


ndConvexHull4dAABBTreeNode* ndConvexHull4d::BuildTree (ndConvexHull4dAABBTreeNode* const parent, ndConvexHull4dVector* const points, ndInt32 count, ndInt32 baseIndex, ndInt8** memoryPool, ndInt32& maxMemSize) const
{
	ndConvexHull4dAABBTreeNode* tree = nullptr;

	dAssert (count);
	ndBigVector minP ( ndFloat32 (1.0e15f)); 
	ndBigVector maxP (-ndFloat32 (1.0e15f)); 
	if (count <= D_VERTEX_CLUMP_SIZE_4D) 
	{
		dgConvexHull4dPointCluster* const clump = new (*memoryPool) dgConvexHull4dPointCluster;
		*memoryPool += sizeof (dgConvexHull4dPointCluster);
		maxMemSize -= sizeof (dgConvexHull4dPointCluster);
		dAssert (maxMemSize >= 0);

		dAssert (clump);
		clump->m_count = count;
		for (ndInt32 i = 0; i < count; i ++) 
		{
			clump->m_indices[i] = i + baseIndex;
			const ndBigVector& p = points[i];
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
			minP = minP.GetMin(p);
			maxP = maxP.GetMax(p);
			median += p;
			varian += p * p;
		}

		varian = varian.Scale (ndFloat32 (count)) - median * median;

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
		ndBigVector center = median.Scale (ndFloat64 (1.0f) / ndFloat64 (count));

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

		tree = new (*memoryPool) ndConvexHull4dAABBTreeNode;
		*memoryPool += sizeof (ndConvexHull4dAABBTreeNode);
		maxMemSize -= sizeof (ndConvexHull4dAABBTreeNode);
		dAssert (maxMemSize >= 0);

		dAssert (i0);
		dAssert (count - i0);

		tree->m_left = BuildTree (tree, points, i0, baseIndex, memoryPool, maxMemSize);
		tree->m_right = BuildTree (tree, &points[i0], count - i0, i0 + baseIndex, memoryPool, maxMemSize);
	}

	dAssert (tree);
	tree->m_parent = parent;
	tree->m_box[0] = minP - ndBigVector (ndFloat64 (1.0e-3f));
	tree->m_box[1] = maxP + ndBigVector (ndFloat64 (1.0e-3f));
	return tree;
}

ndInt32 ndConvexHull4d::InitVertexArray(ndConvexHull4dVector* const points, const ndFloat64* const vertexCloud, ndInt32 strideInBytes, ndInt32 count, void* const memoryPool, ndInt32 maxMemSize)
{
	ndInt32 stride = ndInt32(strideInBytes / sizeof (ndFloat64));
	for (ndInt32 i = 0; i < count; i ++) 
	{
		points[i] = ndBigVector (vertexCloud[i * stride + 0], vertexCloud[i * stride + 1], vertexCloud[i * stride + 2], vertexCloud[i * stride + 3]);
		points[i].m_index = i;
		points[i].m_mark = 0;
	}

	class CompareVertex
	{
		public:
		ndInt32 Compare(const ndConvexHull4dVector& elementA, const ndConvexHull4dVector& elementB, void* const) const
		{
			for (ndInt32 i = 0; i < 4; i++)
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
	ndSort<ndConvexHull4dVector, CompareVertex>(points, count);

	ndInt32 indexCount = 0;
	CompareVertex compareVetex;
	for (ndInt32 i = 1; i < count; i ++) 
	{
		for (; i < count; i ++) 
		{
			if (compareVetex.Compare (points[indexCount], points[i], nullptr))
			{
				indexCount ++;
				points[indexCount] = points[i];
				break;
			}
		}
	}
	count = indexCount + 1;
	if (count < 4) 
	{
		m_count = 0;
		return count;
	}

	ndConvexHull4dAABBTreeNode* tree = BuildTree (nullptr, points, count, 0, (ndInt8**) &memoryPool, maxMemSize);

	ndBigVector boxSize (tree->m_box[1] - tree->m_box[0]);	
	m_diag = ndFloat32 (sqrt (boxSize.DotProduct(boxSize).m_x));

	ndInt32 marks[4];
	m_points.SetCount(count);
	bool validTetrahedrum = false;
	ndConvexHull4dVector* const convexPoints = &m_points[0]; 
	const ndFloat64 testVol = ndFloat32 (1.0e-6f) * m_diag * m_diag * m_diag;

	const ndNormalMap& normalMap = GetNormaMap();
	for (ndInt32 i = 0; !validTetrahedrum && (i < normalMap.m_count); i++) 
	{
		ndInt32 index = SupportVertex(&tree, points, normalMap.m_normal[i], false);
		convexPoints[0] = points[index];
		marks[0] = index;
		for (ndInt32 j = i + 1; !validTetrahedrum && (j < normalMap.m_count); j++) 
		{
			ndInt32 index1 = SupportVertex(&tree, points, normalMap.m_normal[j], false);
			convexPoints[1] = points[index1];
			ndBigVector p10(convexPoints[1] - convexPoints[0]);
			if (p10.DotProduct(p10).GetScalar() >(ndFloat32(1.0e-3f) * m_diag)) 
			{
				marks[1] = index1;
				for (ndInt32 k = j + 1; !validTetrahedrum && (k < normalMap.m_count); k++) 
				{
					ndInt32 index2 = SupportVertex(&tree, points, normalMap.m_normal[k], false);
					convexPoints[2] = points[index2];
					ndBigVector p20(convexPoints[2] - convexPoints[0]);
					ndBigVector p21(convexPoints[2] - convexPoints[1]);
					bool test = p20.DotProduct(p20).GetScalar() > (ndFloat32(1.0e-3f) * m_diag);
					test = test && (p21.DotProduct(p21).GetScalar() > (ndFloat32(1.0e-3f) * m_diag));
					if (test) 
					{
						marks[2] = index2;
						for (ndInt32 l = k + 1; !validTetrahedrum && (l < normalMap.m_count); l++) 
						{
							ndInt32 index3 = SupportVertex(&tree, points, normalMap.m_normal[l], false);
							convexPoints[3] = points[index3];
							ndBigVector p30(convexPoints[3] - convexPoints[0]);
							ndBigVector plane(p10.CrossProduct(p20, p30));
							ndFloat64 volume = plane.DotProduct(plane).GetScalar();
							if (volume > testVol) 
							{
								validTetrahedrum = true;
								marks[3] = index3;
							}
						}
					}
				}
			}
		}
	}

	m_count = 4;
	if (!validTetrahedrum) 
	{
		m_count = 0;
	}

	if (validTetrahedrum) 
	{
		for (ndInt32 i = 0; i < 4; i ++) 
		{
			points[marks[i]].m_mark = 1;
		}
	}

	return count;
}

ndConvexHull4d::ndNode* ndConvexHull4d::AddFace (ndInt32 i0, ndInt32 i1, ndInt32 i2, ndInt32 i3)
{
	ndNode* const node = Append();
	ndConvexHull4dTetraherum& face = node->GetInfo();
	face.Init (&m_points[0], i0, i1, i2, i3);
	return node;
}

void ndConvexHull4d::DeleteFace (ndNode* const node) 
{
	Remove (node);
}

bool ndConvexHull4d::Sanity() const
{
	for (ndNode* node = GetFirst(); node; node = node->GetNext()) {
		ndConvexHull4dTetraherum* const tetra = &node->GetInfo();

		for (ndInt32 i = 0; i < 4; i ++) {
			ndConvexHull4dTetraherum::ndTetrahedrumFace* const face = &tetra->m_faces[i];
			ndNode* const twinNode = face->m_twin;
			if (!twinNode) {
				return false;
			}
		}
	}

/*
	ndList<ndNode*> tetraList(GetAllocator());
	const dgHullVector* const points = &m_points[0];
	for (ndNode* node = GetFirst(); node; node = node->GetNext()) {
		ndConvexHull4dTetraherum* const tetra = &node->GetInfo();
		const ndBigVector &p0 = points[tetra->m_faces[0].m_index[0]];
		const ndBigVector &p1 = points[tetra->m_faces[0].m_index[1]];
		const ndBigVector &p2 = points[tetra->m_faces[0].m_index[2]];
		const ndBigVector &p3 = points[tetra->m_faces[0].m_index[3]];

		ndBigVector p1p0 (p1.Sub4(p0));
		ndBigVector p2p0 (p2.Sub4(p0));
		ndBigVector p3p0 (p3.Sub4(p0));
		ndBigVector normal (p1p0.CrossProduct (p2p0, p3p0));

		if (normal.m_w < ndFloat64 (0.0f)) {
			tetraList.Append(node);
		}
	}

	for (ndList<ndNode*>::ndNode* node0 = tetraList.GetFirst(); node0; node0 = node0->GetNext()) {
		ndNode* const tetraNode0 = node0->GetInfo();
		ndConvexHull4dTetraherum* const tetra0 = &tetraNode0->GetInfo();

		ndInt32 index0[4];
		index0[0] = tetra0->m_faces[0].m_index[0];
		index0[1] = tetra0->m_faces[0].m_index[1];
		index0[2] = tetra0->m_faces[0].m_index[2];
		index0[3] = tetra0->m_faces[0].m_index[3];

		const ndBigVector &p0 = points[index0[0]];
		const ndBigVector &p1 = points[index0[1]];
		const ndBigVector &p2 = points[index0[2]];
		const ndBigVector &p3 = points[index0[3]];
		for (ndList<ndNode*>::ndNode* node1 = node0->GetNext(); node1; node1 = node1->GetNext()) {
			ndNode* const tetraNode1 = node1->GetInfo();
			ndConvexHull4dTetraherum* const tetra1 = &tetraNode1->GetInfo();

			ndInt32 index1[4];
			index1[0] = tetra1->m_faces[0].m_index[0];
			index1[1] = tetra1->m_faces[0].m_index[1];
			index1[2] = tetra1->m_faces[0].m_index[2];
			index1[3] = tetra1->m_faces[0].m_index[3];

			for (ndInt32 i = 0; i < 4; i ++) {
				ndInt32 count = 0;
				ndInt32 k = index1[i];
				for (ndInt32 j = 0; j < 4; j ++) {
					count += (k == index0[j]);
				}
				if (!count){
//					const ndBigVector &p = points[k];
//					ndFloat64 size = -insphere(&p0.m_x, &p1.m_x, &p2.m_x, &p3.m_x, &p.m_x);
//					if (size < ndFloat64 (0.0f)) {
//						return false;
//					}
				}
			}
		}
	}
*/
	return true;
}

void ndConvexHull4d::LinkSibling (ndNode* node0, ndNode* node1)	const
{
	ndConvexHull4dTetraherum* const tetra0 = &node0->GetInfo();
	ndConvexHull4dTetraherum* const tetra1 = &node1->GetInfo();
	for (ndInt32 i = 0; i < 4; i ++) {
		ndConvexHull4dTetraherum::ndTetrahedrumFace* const face0 = &tetra0->m_faces[i];
		if (!face0->m_twin) {
			ndInt32 i0 = face0->m_index[0];
			ndInt32 i1 = face0->m_index[1];
			ndInt32 i2 = face0->m_index[2];
			for (ndInt32 j = 0; j < 4; j ++) {
				ndConvexHull4dTetraherum::ndTetrahedrumFace* const face1 = &tetra1->m_faces[j];
				if (!face1->m_twin) {
					ndInt32 j2 = face1->m_index[0];
					ndInt32 j1 = face1->m_index[1];
					ndInt32 j0 = face1->m_index[2];

					if (((i0 == j0) && (i1 == j1) && (i2 == j2)) || ((i1 == j0) && (i2 == j1) && (i0 == j2)) || ((i2 == j0) && (i0 == j1) && (i1 == j2))) {
						face0->m_twin = node1;
						face1->m_twin = node0;
						return;
					}
				}
			}
		}
	}
}

void ndConvexHull4d::InsertNewVertex(ndInt32 vertexIndex, ndNode* const frontFace, ndTempList& deletedFaces, ndTempList& newFaces)
{
	dAssert (Sanity());

	ndTempList stack;
	
	ndInt32 mark = IncMark();
	stack.Append(frontFace);
	ndConvexHull4dVector* const hullVertexArray = &m_points[0];
	const ndBigVector& p = hullVertexArray[vertexIndex];
	while (stack.GetCount()) 
	{
		ndTempList::ndNode* const stackNode = stack.GetLast();
		ndNode* const node = stackNode->GetInfo();
		stack.Remove(stackNode);
		ndConvexHull4dTetraherum* const face = &node->GetInfo();
		if ((face->GetMark() != mark) && (face->Evalue(hullVertexArray, p) > ndFloat64(0.0f))) 
		{ 
			#ifdef _DEBUG
			for (ndTempList::ndNode* deleteNode = deletedFaces.GetFirst(); deleteNode; deleteNode = deleteNode->GetNext())
			{
				dAssert (deleteNode->GetInfo() != node);
			}
			#endif
			deletedFaces.Append(node);

			face->SetMark(mark);
			for (ndInt32 i = 0; i < 4; i ++) 
			{
				ndNode* const twinNode = (ndNode*)face->m_faces[i].m_twin;
				dAssert (twinNode);
				ndConvexHull4dTetraherum* const twinFace = &twinNode->GetInfo();

				if (twinFace->GetMark() != mark) 
				{
					stack.Append(twinNode);
				}
			}
		}
	}

    ndTree<ndNode*, ndInt32> perimeter;
	for (ndTempList::ndNode* deleteNode = deletedFaces.GetFirst(); deleteNode; deleteNode = deleteNode->GetNext()) 
	{
		ndNode* const deleteTetraNode = deleteNode->GetInfo();
		ndConvexHull4dTetraherum* const deletedTetra = &deleteTetraNode->GetInfo();
		dAssert (deletedTetra->GetMark() == mark);
		for (ndInt32 i = 0; i < 4; i ++) 
		{
			ndNode* const twinNode = deletedTetra->m_faces[i].m_twin;
			ndConvexHull4dTetraherum* const twinTetra = &twinNode->GetInfo();

			if (twinTetra->GetMark() != mark) 
			{
				if (!perimeter.Find(twinTetra->m_uniqueID)) 
				{
					perimeter.Insert(twinNode, twinTetra->m_uniqueID);
				}
			}
			deletedTetra->m_faces[i].m_twin = nullptr;
		}
	}

	ndTempList coneList;
	ndTree<ndNode*, ndInt32>::Iterator iter (perimeter);
	for (iter.Begin(); iter; iter ++) 
	{
		ndNode* const perimeterNode = iter.GetNode()->GetInfo();
		ndConvexHull4dTetraherum* const perimeterTetra = &perimeterNode->GetInfo();
		for (ndInt32 i = 0; i < 4; i ++) 
		{
			ndConvexHull4dTetraherum::ndTetrahedrumFace* const perimeterFace = &perimeterTetra->m_faces[i];

			if (perimeterFace->m_twin->GetInfo().GetMark() == mark) 
			{
				ndNode* const newNode = AddFace (vertexIndex, perimeterFace->m_index[0], perimeterFace->m_index[1], perimeterFace->m_index[2]);
				newFaces.Addtop(newNode);

				ndConvexHull4dTetraherum* const newTetra = &newNode->GetInfo();
				newTetra->m_faces[2].m_twin = perimeterNode;
				perimeterFace->m_twin = newNode;
				coneList.Append (newNode);
			}
		}
	}

	for (ndTempList::ndNode* coneNode = coneList.GetFirst(); coneNode->GetNext(); coneNode = coneNode->GetNext())
	{
		ndNode* const coneNodeA = coneNode->GetInfo();
		for (ndTempList::ndNode* nextConeNode = coneNode->GetNext(); nextConeNode; nextConeNode = nextConeNode->GetNext())
		{
			ndNode* const coneNodeB = nextConeNode->GetInfo();
			LinkSibling (coneNodeA, coneNodeB);
		}
	}
}

ndConvexHull4d::ndNode* ndConvexHull4d::FindFacingNode(const ndBigVector& vertex)
{
	const ndConvexHull4dVector* const hullVertexArray = &m_points[0];

	ndNode* bestNode = GetFirst();
	ndConvexHull4dTetraherum* const tetra = &bestNode->GetInfo();
	ndConvexHull4dTetraherum::ndTetrahedrumPlane plane (tetra->GetPlaneEquation (hullVertexArray));
	ndFloat64 dist = plane.Evalue(vertex);
	ndInt32 mark = IncMark();
	tetra->SetMark(mark);

	ndInt8 buffer[1024 * 2 * sizeof (ndFloat64)];
	ndDownHeap<ndNode*, ndFloat64> heap (buffer, sizeof (buffer));

	heap.Push(bestNode, dist);
	ndInt32 maxCount = heap.GetMaxCount() - 1;
	ndInt32 releafCount = maxCount >> 3;
	while (heap.GetCount()) {
		ndNode* const node1 = heap[0];
		ndFloat64 dist1 = heap.Value();
		if (dist1 > ndFloat64 (1.0e-5f)) {
			return node1;
		}
		heap.Pop();
		ndConvexHull4dTetraherum* const tetra1 = &node1->GetInfo();
		for (ndInt32 i = 0; i < 4; i ++) {
			ndNode* neigborghNode = tetra1->m_faces[i].m_twin;
			ndConvexHull4dTetraherum* const neighborgh = &neigborghNode->GetInfo();
			if (neighborgh->GetMark() != mark) {
				neighborgh->SetMark(mark);
				if (heap.GetCount() >= maxCount) {
					for (ndInt32 j = 0; j < releafCount; j ++) {
						heap.Remove(heap.GetCount() - 1);
					}
				}
				ndConvexHull4dTetraherum::ndTetrahedrumPlane plane1 (neighborgh->GetPlaneEquation (hullVertexArray));
				heap.Push(neigborghNode, plane1.Evalue(vertex));
			}
		}
	}

	for (ndNode* node1 = GetFirst(); node1; node1 = node1->GetNext()) {
		ndConvexHull4dTetraherum* const tetra1 = &node1->GetInfo();
		ndFloat64 dist1 = tetra1->Evalue(hullVertexArray, vertex);
		if (dist1 > ndFloat64(0.0f)) {
			return node1;
		}
	}

	return nullptr;
}

ndInt32 ndConvexHull4d::AddVertex (const ndBigVector& vertex)
{
	ndSetPrecisionDouble precision;
	ndInt32 index = -1;
	ndNode* const faceNode = FindFacingNode(vertex);
	if (faceNode) {
		index = m_count;
		m_points[index] = vertex;
		m_points[index].m_index = index;
		m_count ++;

		dAssert(0);
		ndTempList newFaces;
		ndTempList deleteList;
		
		InsertNewVertex(index, faceNode, deleteList, newFaces);
		for (ndTempList::ndNode* deleteNode = deleteList.GetFirst(); deleteNode; deleteNode = deleteNode->GetNext())
		{
			ndNode* const node = deleteNode->GetInfo();
			DeleteFace (node); 
		}
	}
	return index;
}

void ndConvexHull4d::BuildHull (const ndFloat64* const vertexCloud, ndInt32 strideInBytes, ndInt32 count, ndFloat64 distTol)
{
	ndInt32 treeCount = count / (D_VERTEX_CLUMP_SIZE_4D>>1); 
	if (treeCount < 4) 
	{
		treeCount = 4;
	}
	treeCount *= 2;

	ndStack<ndConvexHull4dVector> points (count);
	ndStack<dgConvexHull4dPointCluster> treePool (treeCount + 256);

	count = InitVertexArray(&points[0], vertexCloud, strideInBytes, count, &treePool[0], treePool.GetSizeInBytes());
	if (m_count >= 4) 
	{
		CalculateConvexHull (&treePool[0], &points[0], count, distTol);
	}
}

void ndConvexHull4d::CalculateConvexHull (ndConvexHull4dAABBTreeNode* vertexTree, ndConvexHull4dVector* const points, ndInt32 count, ndFloat64 distTol)
{
	distTol = fabs (distTol) * m_diag;
	ndNode* const nodes0 = AddFace (0, 1, 2, 3);
	ndNode* const nodes1 = AddFace (0, 1, 3, 2);

	//ndList<ndNode*> boundaryFaces;
	ndTempList boundaryFaces;
	boundaryFaces.Append(nodes0);
	boundaryFaces.Append(nodes1);

	LinkSibling (nodes0, nodes1);
	LinkSibling (nodes0, nodes1);
	LinkSibling (nodes0, nodes1);
	LinkSibling (nodes0, nodes1);

	IncMark();
	count -= 4;
	ndInt32 currentIndex = 4;
	while (boundaryFaces.GetCount() && count) 
	{
		ndConvexHull4dVector* const hullVertexArray = &m_points[0]; 
		ndNode* const faceNode = boundaryFaces.GetFirst()->GetInfo();
		ndConvexHull4dTetraherum* const face = &faceNode->GetInfo();
		ndConvexHull4dTetraherum::ndTetrahedrumPlane planeEquation (face->GetPlaneEquation (hullVertexArray));

		ndInt32 index = SupportVertex (&vertexTree, points, planeEquation);

		const ndConvexHull4dVector& p = points[index];
		ndFloat64 dist = planeEquation.Evalue(p);
		if ((dist > distTol) && (face->Evalue(hullVertexArray, p) > ndFloat64(0.0f))) 
		{
			m_points[currentIndex] = p;
			points[index].m_mark = 1;

			ndTempList deleteList;
			InsertNewVertex(currentIndex, faceNode, deleteList, boundaryFaces);

			for (ndTempList::ndNode* deleteNode = deleteList.GetFirst(); deleteNode; deleteNode = deleteNode->GetNext()) 
			{
				ndNode* const node = deleteNode->GetInfo();
				boundaryFaces.Remove (node);
				DeleteFace (node); 
			}

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
