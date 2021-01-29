/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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
#include "dSort.h"
#include "dTree.h"
#include "dHeap.h"
#include "dStack.h"
#include "dGoogol.h"
#include "dConvexHull4d.h"
#include "dSmallDeterminant.h"

#define D_VERTEX_CLUMP_SIZE_4D		8 


dConvexHull4d::dgNormalMap::dgNormalMap()
	:m_count(sizeof (m_normal) / sizeof (m_normal[0]))
{
	dVector p0(dFloat32( 1.0f), dFloat32( 0.0f), dFloat32( 0.0f), dFloat32(0.0f));
	dVector p1(dFloat32(-1.0f), dFloat32( 0.0f), dFloat32( 0.0f), dFloat32(0.0f));
	dVector p2(dFloat32( 0.0f), dFloat32( 1.0f), dFloat32( 0.0f), dFloat32(0.0f));
	dVector p3(dFloat32( 0.0f), dFloat32(-1.0f), dFloat32( 0.0f), dFloat32(0.0f));
	dVector p4(dFloat32( 0.0f), dFloat32( 0.0f), dFloat32( 1.0f), dFloat32(0.0f));
	dVector p5(dFloat32( 0.0f), dFloat32( 0.0f), dFloat32(-1.0f), dFloat32(0.0f));

	dInt32 count = 0;
	dInt32 subdivitions = 2;

	dBigVector tmp[128];
	TessellateTriangle(subdivitions, p4, p0, p2, tmp, count);
	TessellateTriangle(subdivitions, p0, p5, p2, tmp, count);
	TessellateTriangle(subdivitions, p5, p1, p2, tmp, count);
	TessellateTriangle(subdivitions, p1, p4, p2, tmp, count);
	TessellateTriangle(subdivitions, p0, p4, p3, tmp, count);
	TessellateTriangle(subdivitions, p5, p0, p3, tmp, count);
	TessellateTriangle(subdivitions, p1, p5, p3, tmp, count);
	TessellateTriangle(subdivitions, p4, p1, p3, tmp, count);

	count = 0;
	for (dInt32 j = 0; j < 8; j++) 
	{
		dFloat64 beta = (j - 4) * dFloat64 (22.5f * dDegreeToRad) + dFloat64 (10.5f * dDegreeToRad);
		dFloat64 sinBeta = sin(beta);
		dFloat64 cosBeta = cos(beta);

		dFloat64 w = sinBeta;
		for (dInt32 i = 0; i < 128; i ++) 
		{
			dFloat64 z = cosBeta * tmp[i].m_z;
			dFloat64 y = cosBeta * tmp[i].m_y;
			dFloat64 x = cosBeta * tmp[i].m_x;
			dInt32 index = dBitReversal(count, sizeof (m_normal) / sizeof (m_normal[0]));
			dAssert (index < sizeof (m_normal) / sizeof (m_normal[0]));
			dAssert (count < sizeof (m_normal) / sizeof (m_normal[0]));
			m_normal[index] = dBigVector (x, y, z, w);
			count ++;
		}
	}
}

void dConvexHull4d::dgNormalMap::TessellateTriangle(dInt32 level, const dVector& p0, const dVector& p1, const dVector& p2, dBigVector* const buffer, dInt32& count)
{
	dAssert(p0.m_w == dFloat32(0.0f));
	dAssert(p1.m_w == dFloat32(0.0f));
	dAssert(p2.m_w == dFloat32(0.0f));
	if (level) 
	{
		dAssert(dAbs(p0.DotProduct(p0).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
		dAssert(dAbs(p1.DotProduct(p1).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
		dAssert(dAbs(p2.DotProduct(p2).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
		dVector p01(p0 + p1);
		dVector p12(p1 + p2);
		dVector p20(p2 + p0);

		dAssert (p01.m_w == dFloat32 (0.0f));
		dAssert (p12.m_w == dFloat32 (0.0f));
		dAssert (p20.m_w == dFloat32 (0.0f));
		//p01 = p01.Scale(dgRsqrt(p01.DotProduct(p01).GetScalar()));
		//p12 = p12.Scale(dgRsqrt(p12.DotProduct(p12).GetScalar()));
		//p20 = p20.Scale(dgRsqrt(p20.DotProduct(p20).GetScalar()));
		p01 = p01.Normalize();
		p12 = p12.Normalize();
		p20 = p20.Normalize();

		dAssert(dAbs(p01.DotProduct(p01).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
		dAssert(dAbs(p12.DotProduct(p12).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
		dAssert(dAbs(p20.DotProduct(p20).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));

		TessellateTriangle(level - 1, p0, p01, p20, buffer, count);
		TessellateTriangle(level - 1, p1, p12, p01, buffer, count);
		TessellateTriangle(level - 1, p2, p20, p12, buffer, count);
		TessellateTriangle(level - 1, p01, p12, p20, buffer, count);
	} 
	else 
	{
		dBigPlane n(p0, p1, p2);
		n = n.Scale(dFloat64(1.0f) / sqrt(n.DotProduct(n).GetScalar()));
		n.m_w = dFloat64(0.0f);
		dInt32 index = dBitReversal(count, 128);
		buffer[index] = n;
		dAssert(count < 128);
		count++;
	}
}

class dConvexHull4dAABBTreeNode
{
	public:
	#ifdef _DEBUG
	dConvexHull4dAABBTreeNode()
	{
		static dInt32 id = 0;
		m_id = id;
		id ++;
	}
	dInt32 m_id;
	#endif

		dBigVector m_box[2];
		dConvexHull4dAABBTreeNode* m_left;
		dConvexHull4dAABBTreeNode* m_right;
		dConvexHull4dAABBTreeNode* m_parent;
};

class dgConvexHull4dPointCluster: public dConvexHull4dAABBTreeNode
{
	public:
	dInt32 m_count;
	dInt32 m_indices[D_VERTEX_CLUMP_SIZE_4D];
};

dConvexHull4dTetraherum::dgTetrahedrumPlane::dgTetrahedrumPlane (const dBigVector& p0, const dBigVector& p1, const dBigVector& p2, const dBigVector& p3)
	:dBigVector ((p1 - p0).CrossProduct (p2 - p0, p3 - p0))
{
	dBigVector& me = *this;
	dFloat64 invMag2 = dFloat32 (0.0f);
	dFloat64 val = me.DotProduct(me).m_x;
	if (val > dFloat64 (1.0e-38)) {
		invMag2 = dFloat64 (1.0f) / sqrt (val);
	} else {
		invMag2 = dFloat32 (0.0f);
	}

	me.m_x *= invMag2;
	me.m_y *= invMag2;
	me.m_z *= invMag2;
	me.m_w *= invMag2;
	m_dist = - me.DotProduct(p0).m_x;
}

dFloat64 dConvexHull4dTetraherum::dgTetrahedrumPlane::Evalue (const dBigVector& point) const
{
	const dBigVector& me = *this;
	return me.DotProduct(point).m_x + m_dist;
}


dConvexHull4dTetraherum::dConvexHull4dTetraherum()
{
#ifdef _DEBUG
	static dInt32 debugID;
	m_debugID = debugID;
	debugID ++;
#endif
	static dInt32 m_monotonicID;

	m_uniqueID = m_monotonicID;
	m_monotonicID ++;
}

void dConvexHull4dTetraherum::Init (const dConvexHull4dVector* const points, dInt32 v0, dInt32 v1, dInt32 v2, dInt32 v3)
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
	for (dInt32 i = 0; i < 4; i ++) {
		m_faces[i].m_twin = nullptr;
	}

#ifdef _DEBUG
	dBigVector p1p0 (points[v1] - points[v0]);
	dBigVector p2p0 (points[v2] - points[v0]);
	dBigVector p3p0 (points[v3] - points[v0]);
	dBigVector normal (p1p0.CrossProduct(p2p0, p3p0));
	dFloat64 volume = normal.DotProduct(normal).m_x;
	dAssert (volume > dFloat64 (0.0f));
#endif
}

dFloat64 dConvexHull4dTetraherum::Evalue (const dConvexHull4dVector* const pointArray, const dBigVector& point) const
{
	const dBigVector &p0 = pointArray[m_faces[0].m_index[0]];
	const dBigVector &p1 = pointArray[m_faces[0].m_index[1]];
	const dBigVector &p2 = pointArray[m_faces[0].m_index[2]];
	const dBigVector &p3 = pointArray[m_faces[0].m_index[3]];

	dFloat64 matrix[4][4];
	for (dInt32 i = 0; i < 4; i ++) {
		matrix[0][i] = p1[i] - p0[i];
		matrix[1][i] = p2[i] - p0[i];
		matrix[2][i] = p3[i] - p0[i];
		matrix[3][i] = point[i] - p0[i];
	}

	dFloat64 error;
	dFloat64 det = Determinant4x4 (matrix, &error);
	dFloat64 precision  = dFloat64 (1.0f) / dFloat64 (1<<24);
	dFloat64 errbound = error * precision; 
	if (fabs(det) > errbound) {
		return det;
	}

	dGoogol exactMatrix[4][4];
	for (dInt32 i = 0; i < 4; i ++) {
		exactMatrix[0][i] = dGoogol(p1[i]) - dGoogol(p0[i]);
		exactMatrix[1][i] = dGoogol(p2[i]) - dGoogol(p0[i]);
		exactMatrix[2][i] = dGoogol(p3[i]) - dGoogol(p0[i]);
		exactMatrix[3][i] = dGoogol(point[i]) - dGoogol(p0[i]);
	}
	return Determinant4x4(exactMatrix);
}

dFloat64 dConvexHull4dTetraherum::GetTetraVolume(const dConvexHull4dVector* const points) const
{
	const dBigVector &p0 = points[m_faces[0].m_index[0]];
	const dBigVector &p1 = points[m_faces[0].m_index[1]];
	const dBigVector &p2 = points[m_faces[0].m_index[2]];
	const dBigVector &p3 = points[m_faces[0].m_index[3]];

	dFloat64 matrix[3][3];
	for (dInt32 i = 0; i < 3; i++) {
		matrix[0][i] = p2[i] - p0[i];
		matrix[1][i] = p1[i] - p0[i];
		matrix[2][i] = p3[i] - p0[i];
	}

	dFloat64 error;
	dFloat64 det = Determinant3x3(matrix, &error);

	dFloat64 precision = dFloat64(1.0f) / dFloat64(1 << 24);
	dFloat64 errbound = error * precision;
	if (fabs(det) > errbound) {
		return det;
	}

	dGoogol exactMatrix[3][3];
	for (dInt32 i = 0; i < 3; i++) {
		exactMatrix[0][i] = dGoogol(p2[i]) - dGoogol(p0[i]);
		exactMatrix[1][i] = dGoogol(p1[i]) - dGoogol(p0[i]);
		exactMatrix[2][i] = dGoogol(p3[i]) - dGoogol(p0[i]);
	}
	return Determinant3x3(exactMatrix);
}

dBigVector dConvexHull4dTetraherum::CircumSphereCenter (const dConvexHull4dVector* const pointArray) const
{
	dGoogol matrix[4][4];

	dBigVector points[4];
	points[0] = pointArray[m_faces[0].m_index[0]];
	points[1] = pointArray[m_faces[0].m_index[1]];
	points[2] = pointArray[m_faces[0].m_index[2]];
	points[3] = pointArray[m_faces[0].m_index[3]];

	for (dInt32 i = 0; i < 4; i ++) {
		for (dInt32 j = 0; j < 3; j ++) {
			matrix[i][j] = dGoogol (points[i][j]);
		}
		matrix[i][3] = dGoogol (1.0f);
	}
	dGoogol det (Determinant4x4(matrix));
	dFloat64 invDen = dFloat64 (1.0f) / (dFloat64(det) * dFloat64 (2.0f));

	dBigVector centerOut;
	dFloat64 sign = dFloat64 (1.0f);
	for (dInt32 k = 0; k < 3; k ++) {
		for (dInt32 i = 0; i < 4; i ++) {
			matrix[i][0] = dGoogol (points[i][3]);
			for (dInt32 j = 0; j < 2; j ++) {
				dInt32 j1 = (j < k) ? j : j + 1; 
				matrix[i][j + 1] = dGoogol (points[i][j1]);
			}
			matrix[i][3] = dGoogol (1.0f);
		}
		dGoogol det1 (Determinant4x4(matrix));
		dFloat64 val = dFloat64 (det1) * sign;
		sign *= dFloat64 (-1.0f);
		centerOut[k] = val * invDen; 
	}
	centerOut[3] = dFloat32 (0.0f);
	return centerOut;
}

dConvexHull4dTetraherum::dgTetrahedrumPlane dConvexHull4dTetraherum::GetPlaneEquation (const dConvexHull4dVector* const points) const
{
	const dBigVector &p0 = points[m_faces[0].m_index[0]];
	const dBigVector &p1 = points[m_faces[0].m_index[1]];
	const dBigVector &p2 = points[m_faces[0].m_index[2]];
	const dBigVector &p3 = points[m_faces[0].m_index[3]];
	return dgTetrahedrumPlane (p0, p1, p2, p3);
}

dConvexHull4d::dConvexHull4d ()
	:dList<dConvexHull4dTetraherum>()
	,m_mark(0)
	,m_count(0)
	,m_diag(dFloat32 (0.0f))
	,m_points() 
{
}

dConvexHull4d::dConvexHull4d(const dConvexHull4d& source)
	:dList<dConvexHull4dTetraherum>()
{
	dAssert(0);
}

dConvexHull4d::dConvexHull4d (const dFloat64* const vertexCloud, dInt32 strideInBytes, dInt32 count, dFloat64 distTol)
	:dList<dConvexHull4dTetraherum>()
	,m_mark(0)
	,m_count(0)
	,m_diag(dFloat32(0.0f))
	,m_points() 
{
	BuildHull (vertexCloud, strideInBytes, count, distTol);
}

dConvexHull4d::~dConvexHull4d(void)
{
}

const dConvexHull4d::dgNormalMap& dConvexHull4d::GetNormaMap()
{
	static dgNormalMap normalMap;
	return normalMap;
}

void dConvexHull4d::Save (const char* const filename) const
{
	FILE* const file = fopen (filename, "wb");
	dInt32 index = 0;
//	fprintf (file, "final\n");
	for (dListNode* nodePtr = GetFirst(); nodePtr; nodePtr = nodePtr->GetNext()) {
		fprintf (file, "tetra %d\n", index);
		index ++;
		const dConvexHull4dTetraherum& face = nodePtr->GetInfo();
		const dBigVector& p0 = m_points[face.m_faces[0].m_index[0]];
		const dBigVector& p1 = m_points[face.m_faces[0].m_index[1]];
		const dBigVector& p2 = m_points[face.m_faces[0].m_index[2]];
		const dBigVector& p3 = m_points[face.m_faces[0].m_index[3]];
		fprintf (file, "p0(%f %f %f %f)\n", p0[0], p0[1], p0[2], p0[3]);
		fprintf (file, "p1(%f %f %f %f)\n", p1[0], p1[1], p1[2], p1[3]);
		fprintf (file, "p2(%f %f %f %f)\n", p2[0], p2[1], p2[2], p2[3]);
		fprintf (file, "p3(%f %f %f %f)\n", p3[0], p3[1], p3[2], p3[3]);
	}
	fprintf (file, "\n");

	fclose (file);
}

dInt32 dConvexHull4d::SupportVertex (dConvexHull4dAABBTreeNode** const treePointer, const dConvexHull4dVector* const points, const dBigVector& dir, const bool removeEntry) const
{
	#define DG_STACK_DEPTH_4D	64
	dFloat64 aabbProjection[DG_STACK_DEPTH_4D];
	const dConvexHull4dAABBTreeNode *stackPool[DG_STACK_DEPTH_4D];

	dInt32 index = -1;
	dInt32 stack = 1;
	stackPool[0] = *treePointer;
	aabbProjection[0] = dFloat32 (1.0e20f);
	dFloat64 maxProj = dFloat64 (-1.0e20f); 
	dInt32 ix = (dir[0] > dFloat64 (0.0f)) ? 1 : 0;
	dInt32 iy = (dir[1] > dFloat64 (0.0f)) ? 1 : 0;
	dInt32 iz = (dir[2] > dFloat64 (0.0f)) ? 1 : 0;
	dInt32 iw = (dir[3] > dFloat64 (0.0f)) ? 1 : 0;
	while (stack) 
	{
		stack--;
		dFloat64 boxSupportValue = aabbProjection[stack];
		if (boxSupportValue > maxProj) 
		{
			const dConvexHull4dAABBTreeNode* const me = stackPool[stack];

			if (me->m_left && me->m_right) 
			{
				dBigVector leftSupportPoint (me->m_left->m_box[ix].m_x, me->m_left->m_box[iy].m_y, me->m_left->m_box[iz].m_z, me->m_left->m_box[iw].m_w);
				dFloat64 leftSupportDist = leftSupportPoint.DotProduct(dir).m_x;

				dBigVector rightSupportPoint (me->m_right->m_box[ix].m_x, me->m_right->m_box[iy].m_y, me->m_right->m_box[iz].m_z, me->m_right->m_box[iw].m_w);
				dFloat64 rightSupportDist = rightSupportPoint.DotProduct(dir).m_x;

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
				for (dInt32 i = 0; i < cluster->m_count; i ++) 
				{
					const dConvexHull4dVector& p = points[cluster->m_indices[i]];
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
						dFloat64 dist = p.DotProduct(dir).m_x;
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
					dConvexHull4dAABBTreeNode* const parent = cluster->m_parent;
					if (parent) 
					{	
						dConvexHull4dAABBTreeNode* const sibling = (parent->m_left != cluster) ? parent->m_left : parent->m_right;
						dAssert (sibling != cluster);
						dConvexHull4dAABBTreeNode* const grandParent = parent->m_parent;
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

dInt32 dConvexHull4d::ConvexCompareVertex(const dConvexHull4dVector* const  A, const dConvexHull4dVector* const B, void* const context)
{
	for (dInt32 i = 0; i < 4; i ++) {
		if ((*A)[i] < (*B)[i]) {
			return -1;
		} else if ((*A)[i] > (*B)[i]) {
			return 1;
		}
	}
	return 0;
}

dConvexHull4dAABBTreeNode* dConvexHull4d::BuildTree (dConvexHull4dAABBTreeNode* const parent, dConvexHull4dVector* const points, dInt32 count, dInt32 baseIndex, dInt8** memoryPool, dInt32& maxMemSize) const
{
	dConvexHull4dAABBTreeNode* tree = nullptr;

	dAssert (count);
	dBigVector minP ( dFloat32 (1.0e15f)); 
	dBigVector maxP (-dFloat32 (1.0e15f)); 
	if (count <= D_VERTEX_CLUMP_SIZE_4D) 
	{
		dgConvexHull4dPointCluster* const clump = new (*memoryPool) dgConvexHull4dPointCluster;
		*memoryPool += sizeof (dgConvexHull4dPointCluster);
		maxMemSize -= sizeof (dgConvexHull4dPointCluster);
		dAssert (maxMemSize >= 0);

		dAssert (clump);
		clump->m_count = count;
		for (dInt32 i = 0; i < count; i ++) 
		{
			clump->m_indices[i] = i + baseIndex;
			const dBigVector& p = points[i];
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
			minP = minP.GetMin(p);
			maxP = maxP.GetMax(p);
			median += p;
			varian += p * p;
		}

		varian = varian.Scale (dFloat32 (count)) - median * median;

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
		dBigVector center = median.Scale (dFloat64 (1.0f) / dFloat64 (count));

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

		tree = new (*memoryPool) dConvexHull4dAABBTreeNode;
		*memoryPool += sizeof (dConvexHull4dAABBTreeNode);
		maxMemSize -= sizeof (dConvexHull4dAABBTreeNode);
		dAssert (maxMemSize >= 0);

		dAssert (i0);
		dAssert (count - i0);

		tree->m_left = BuildTree (tree, points, i0, baseIndex, memoryPool, maxMemSize);
		tree->m_right = BuildTree (tree, &points[i0], count - i0, i0 + baseIndex, memoryPool, maxMemSize);
	}

	dAssert (tree);
	tree->m_parent = parent;
	tree->m_box[0] = minP - dBigVector (dFloat64 (1.0e-3f));
	tree->m_box[1] = maxP + dBigVector (dFloat64 (1.0e-3f));
	return tree;
}

dInt32 dConvexHull4d::InitVertexArray(dConvexHull4dVector* const points, const dFloat64* const vertexCloud, dInt32 strideInBytes, dInt32 count, void* const memoryPool, dInt32 maxMemSize)
{
	dInt32 stride = dInt32(strideInBytes / sizeof (dFloat64));
	for (dInt32 i = 0; i < count; i ++) 
	{
		points[i] = dBigVector (vertexCloud[i * stride + 0], vertexCloud[i * stride + 1], vertexCloud[i * stride + 2], vertexCloud[i * stride + 3]);
		points[i].m_index = i;
		points[i].m_mark = 0;
	}

	dSort(points, count, ConvexCompareVertex);
	dInt32 indexCount = 0;
	for (dInt32 i = 1; i < count; i ++) 
	{
		for (; i < count; i ++) 
		{
			if (ConvexCompareVertex (&points[indexCount], &points[i], nullptr)) 
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

	dConvexHull4dAABBTreeNode* tree = BuildTree (nullptr, points, count, 0, (dInt8**) &memoryPool, maxMemSize);

	dBigVector boxSize (tree->m_box[1] - tree->m_box[0]);	
	m_diag = dFloat32 (sqrt (boxSize.DotProduct(boxSize).m_x));

	dInt32 marks[4];
	m_points.SetCount(count);
	bool validTetrahedrum = false;
	dConvexHull4dVector* const convexPoints = &m_points[0]; 
	const dFloat64 testVol = dFloat32 (1.0e-6f) * m_diag * m_diag * m_diag;

	const dgNormalMap& normalMap = GetNormaMap();
	for (dInt32 i = 0; !validTetrahedrum && (i < normalMap.m_count); i++) 
	{
		dInt32 index = SupportVertex(&tree, points, normalMap.m_normal[i], false);
		convexPoints[0] = points[index];
		marks[0] = index;
		for (dInt32 j = i + 1; !validTetrahedrum && (j < normalMap.m_count); j++) 
		{
			dInt32 index1 = SupportVertex(&tree, points, normalMap.m_normal[j], false);
			convexPoints[1] = points[index1];
			dBigVector p10(convexPoints[1] - convexPoints[0]);
			if (p10.DotProduct(p10).GetScalar() >(dFloat32(1.0e-3f) * m_diag)) 
			{
				marks[1] = index1;
				for (dInt32 k = j + 1; !validTetrahedrum && (k < normalMap.m_count); k++) 
				{
					dInt32 index2 = SupportVertex(&tree, points, normalMap.m_normal[k], false);
					convexPoints[2] = points[index2];
					dBigVector p20(convexPoints[2] - convexPoints[0]);
					dBigVector p21(convexPoints[2] - convexPoints[1]);
					bool test = p20.DotProduct(p20).GetScalar() > (dFloat32(1.0e-3f) * m_diag);
					test = test && (p21.DotProduct(p21).GetScalar() > (dFloat32(1.0e-3f) * m_diag));
					if (test) 
					{
						marks[2] = index2;
						for (dInt32 l = k + 1; !validTetrahedrum && (l < normalMap.m_count); l++) 
						{
							dInt32 index3 = SupportVertex(&tree, points, normalMap.m_normal[l], false);
							convexPoints[3] = points[index3];
							dBigVector p30(convexPoints[3] - convexPoints[0]);
							dBigVector plane(p10.CrossProduct(p20, p30));
							dFloat64 volume = plane.DotProduct(plane).GetScalar();
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
		for (dInt32 i = 0; i < 4; i ++) 
		{
			points[marks[i]].m_mark = 1;
		}
	}

	return count;
}

dConvexHull4d::dListNode* dConvexHull4d::AddFace (dInt32 i0, dInt32 i1, dInt32 i2, dInt32 i3)
{
	dListNode* const node = Append();
	dConvexHull4dTetraherum& face = node->GetInfo();
	face.Init (&m_points[0], i0, i1, i2, i3);
	return node;
}

void dConvexHull4d::DeleteFace (dListNode* const node) 
{
	Remove (node);
}

bool dConvexHull4d::Sanity() const
{
	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		dConvexHull4dTetraherum* const tetra = &node->GetInfo();

		for (dInt32 i = 0; i < 4; i ++) {
			dConvexHull4dTetraherum::dgTetrahedrumFace* const face = &tetra->m_faces[i];
			dListNode* const twinNode = face->m_twin;
			if (!twinNode) {
				return false;
			}
		}
	}

/*
	dList<dListNode*> tetraList(GetAllocator());
	const dgHullVector* const points = &m_points[0];
	for (dListNode* node = GetFirst(); node; node = node->GetNext()) {
		dConvexHull4dTetraherum* const tetra = &node->GetInfo();
		const dBigVector &p0 = points[tetra->m_faces[0].m_index[0]];
		const dBigVector &p1 = points[tetra->m_faces[0].m_index[1]];
		const dBigVector &p2 = points[tetra->m_faces[0].m_index[2]];
		const dBigVector &p3 = points[tetra->m_faces[0].m_index[3]];

		dBigVector p1p0 (p1.Sub4(p0));
		dBigVector p2p0 (p2.Sub4(p0));
		dBigVector p3p0 (p3.Sub4(p0));
		dBigVector normal (p1p0.CrossProduct (p2p0, p3p0));

		if (normal.m_w < dFloat64 (0.0f)) {
			tetraList.Append(node);
		}
	}

	for (dList<dListNode*>::dListNode* node0 = tetraList.GetFirst(); node0; node0 = node0->GetNext()) {
		dListNode* const tetraNode0 = node0->GetInfo();
		dConvexHull4dTetraherum* const tetra0 = &tetraNode0->GetInfo();

		dInt32 index0[4];
		index0[0] = tetra0->m_faces[0].m_index[0];
		index0[1] = tetra0->m_faces[0].m_index[1];
		index0[2] = tetra0->m_faces[0].m_index[2];
		index0[3] = tetra0->m_faces[0].m_index[3];

		const dBigVector &p0 = points[index0[0]];
		const dBigVector &p1 = points[index0[1]];
		const dBigVector &p2 = points[index0[2]];
		const dBigVector &p3 = points[index0[3]];
		for (dList<dListNode*>::dListNode* node1 = node0->GetNext(); node1; node1 = node1->GetNext()) {
			dListNode* const tetraNode1 = node1->GetInfo();
			dConvexHull4dTetraherum* const tetra1 = &tetraNode1->GetInfo();

			dInt32 index1[4];
			index1[0] = tetra1->m_faces[0].m_index[0];
			index1[1] = tetra1->m_faces[0].m_index[1];
			index1[2] = tetra1->m_faces[0].m_index[2];
			index1[3] = tetra1->m_faces[0].m_index[3];

			for (dInt32 i = 0; i < 4; i ++) {
				dInt32 count = 0;
				dInt32 k = index1[i];
				for (dInt32 j = 0; j < 4; j ++) {
					count += (k == index0[j]);
				}
				if (!count){
//					const dBigVector &p = points[k];
//					dFloat64 size = -insphere(&p0.m_x, &p1.m_x, &p2.m_x, &p3.m_x, &p.m_x);
//					if (size < dFloat64 (0.0f)) {
//						return false;
//					}
				}
			}
		}
	}
*/
	return true;
}

void dConvexHull4d::LinkSibling (dListNode* node0, dListNode* node1)	const
{
	dConvexHull4dTetraherum* const tetra0 = &node0->GetInfo();
	dConvexHull4dTetraherum* const tetra1 = &node1->GetInfo();
	for (dInt32 i = 0; i < 4; i ++) {
		dConvexHull4dTetraherum::dgTetrahedrumFace* const face0 = &tetra0->m_faces[i];
		if (!face0->m_twin) {
			dInt32 i0 = face0->m_index[0];
			dInt32 i1 = face0->m_index[1];
			dInt32 i2 = face0->m_index[2];
			for (dInt32 j = 0; j < 4; j ++) {
				dConvexHull4dTetraherum::dgTetrahedrumFace* const face1 = &tetra1->m_faces[j];
				if (!face1->m_twin) {
					dInt32 j2 = face1->m_index[0];
					dInt32 j1 = face1->m_index[1];
					dInt32 j0 = face1->m_index[2];

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

void dConvexHull4d::InsertNewVertex(dInt32 vertexIndex, dListNode* const frontFace, ndTempList& deletedFaces, ndTempList& newFaces)
{
	dAssert (Sanity());

	ndTempList stack;
	
	dInt32 mark = IncMark();
	stack.Append(frontFace);
	dConvexHull4dVector* const hullVertexArray = &m_points[0];
	const dBigVector& p = hullVertexArray[vertexIndex];
	while (stack.GetCount()) 
	{
		ndTempList::dListNode* const stackNode = stack.GetLast();
		dListNode* const node = stackNode->GetInfo();
		stack.Remove(stackNode);
		dConvexHull4dTetraherum* const face = &node->GetInfo();
		if ((face->GetMark() != mark) && (face->Evalue(hullVertexArray, p) > dFloat64(0.0f))) 
		{ 
			#ifdef _DEBUG
			for (ndTempList::dListNode* deleteNode = deletedFaces.GetFirst(); deleteNode; deleteNode = deleteNode->GetNext())
			{
				dAssert (deleteNode->GetInfo() != node);
			}
			#endif
			deletedFaces.Append(node);

			face->SetMark(mark);
			for (dInt32 i = 0; i < 4; i ++) 
			{
				dListNode* const twinNode = (dListNode*)face->m_faces[i].m_twin;
				dAssert (twinNode);
				dConvexHull4dTetraherum* const twinFace = &twinNode->GetInfo();

				if (twinFace->GetMark() != mark) 
				{
					stack.Append(twinNode);
				}
			}
		}
	}

    dTree<dListNode*, dInt32> perimeter;
	for (ndTempList::dListNode* deleteNode = deletedFaces.GetFirst(); deleteNode; deleteNode = deleteNode->GetNext()) 
	{
		dListNode* const deleteTetraNode = deleteNode->GetInfo();
		dConvexHull4dTetraherum* const deletedTetra = &deleteTetraNode->GetInfo();
		dAssert (deletedTetra->GetMark() == mark);
		for (dInt32 i = 0; i < 4; i ++) 
		{
			dListNode* const twinNode = deletedTetra->m_faces[i].m_twin;
			dConvexHull4dTetraherum* const twinTetra = &twinNode->GetInfo();

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
	dTree<dListNode*, dInt32>::Iterator iter (perimeter);
	for (iter.Begin(); iter; iter ++) 
	{
		dListNode* const perimeterNode = iter.GetNode()->GetInfo();
		dConvexHull4dTetraherum* const perimeterTetra = &perimeterNode->GetInfo();
		for (dInt32 i = 0; i < 4; i ++) 
		{
			dConvexHull4dTetraherum::dgTetrahedrumFace* const perimeterFace = &perimeterTetra->m_faces[i];

			if (perimeterFace->m_twin->GetInfo().GetMark() == mark) 
			{
				dListNode* const newNode = AddFace (vertexIndex, perimeterFace->m_index[0], perimeterFace->m_index[1], perimeterFace->m_index[2]);
				newFaces.Addtop(newNode);

				dConvexHull4dTetraherum* const newTetra = &newNode->GetInfo();
				newTetra->m_faces[2].m_twin = perimeterNode;
				perimeterFace->m_twin = newNode;
				coneList.Append (newNode);
			}
		}
	}

	for (ndTempList::dListNode* coneNode = coneList.GetFirst(); coneNode->GetNext(); coneNode = coneNode->GetNext())
	{
		dListNode* const coneNodeA = coneNode->GetInfo();
		for (ndTempList::dListNode* nextConeNode = coneNode->GetNext(); nextConeNode; nextConeNode = nextConeNode->GetNext())
		{
			dListNode* const coneNodeB = nextConeNode->GetInfo();
			LinkSibling (coneNodeA, coneNodeB);
		}
	}
}

dConvexHull4d::dListNode* dConvexHull4d::FindFacingNode(const dBigVector& vertex)
{
	const dConvexHull4dVector* const hullVertexArray = &m_points[0];

	dListNode* bestNode = GetFirst();
	dConvexHull4dTetraherum* const tetra = &bestNode->GetInfo();
	dConvexHull4dTetraherum::dgTetrahedrumPlane plane (tetra->GetPlaneEquation (hullVertexArray));
	dFloat64 dist = plane.Evalue(vertex);
	dInt32 mark = IncMark();
	tetra->SetMark(mark);

	dInt8 buffer[1024 * 2 * sizeof (dFloat64)];
	dDownHeap<dListNode*, dFloat64> heap (buffer, sizeof (buffer));

	heap.Push(bestNode, dist);
	dInt32 maxCount = heap.GetMaxCount() - 1;
	dInt32 releafCount = maxCount >> 3;
	while (heap.GetCount()) {
		dListNode* const node1 = heap[0];
		dFloat64 dist1 = heap.Value();
		if (dist1 > dFloat64 (1.0e-5f)) {
			return node1;
		}
		heap.Pop();
		dConvexHull4dTetraherum* const tetra1 = &node1->GetInfo();
		for (dInt32 i = 0; i < 4; i ++) {
			dListNode* neigborghNode = tetra1->m_faces[i].m_twin;
			dConvexHull4dTetraherum* const neighborgh = &neigborghNode->GetInfo();
			if (neighborgh->GetMark() != mark) {
				neighborgh->SetMark(mark);
				if (heap.GetCount() >= maxCount) {
					for (dInt32 j = 0; j < releafCount; j ++) {
						heap.Remove(heap.GetCount() - 1);
					}
				}
				dConvexHull4dTetraherum::dgTetrahedrumPlane plane1 (neighborgh->GetPlaneEquation (hullVertexArray));
				heap.Push(neigborghNode, plane1.Evalue(vertex));
			}
		}
	}

	for (dListNode* node1 = GetFirst(); node1; node1 = node1->GetNext()) {
		dConvexHull4dTetraherum* const tetra1 = &node1->GetInfo();
		dFloat64 dist1 = tetra1->Evalue(hullVertexArray, vertex);
		if (dist1 > dFloat64(0.0f)) {
			return node1;
		}
	}

	return nullptr;
}

dInt32 dConvexHull4d::AddVertex (const dBigVector& vertex)
{
	dSetPrecisionDouble precision;
	dInt32 index = -1;
	dListNode* const faceNode = FindFacingNode(vertex);
	if (faceNode) {
		index = m_count;
		m_points[index] = vertex;
		m_points[index].m_index = index;
		m_count ++;

		dAssert(0);
		ndTempList newFaces;
		ndTempList deleteList;
		
		InsertNewVertex(index, faceNode, deleteList, newFaces);
		for (ndTempList::dListNode* deleteNode = deleteList.GetFirst(); deleteNode; deleteNode = deleteNode->GetNext())
		{
			dListNode* const node = deleteNode->GetInfo();
			DeleteFace (node); 
		}
	}
	return index;
}

void dConvexHull4d::BuildHull (const dFloat64* const vertexCloud, dInt32 strideInBytes, dInt32 count, dFloat64 distTol)
{
	dInt32 treeCount = count / (D_VERTEX_CLUMP_SIZE_4D>>1); 
	if (treeCount < 4) 
	{
		treeCount = 4;
	}
	treeCount *= 2;

	dStack<dConvexHull4dVector> points (count);
	dStack<dgConvexHull4dPointCluster> treePool (treeCount + 256);

	count = InitVertexArray(&points[0], vertexCloud, strideInBytes, count, &treePool[0], treePool.GetSizeInBytes());
	if (m_count >= 4) 
	{
		CalculateConvexHull (&treePool[0], &points[0], count, distTol);
	}
}

void dConvexHull4d::CalculateConvexHull (dConvexHull4dAABBTreeNode* vertexTree, dConvexHull4dVector* const points, dInt32 count, dFloat64 distTol)
{
	distTol = fabs (distTol) * m_diag;
	dListNode* const nodes0 = AddFace (0, 1, 2, 3);
	dListNode* const nodes1 = AddFace (0, 1, 3, 2);

	//dList<dListNode*> boundaryFaces;
	ndTempList boundaryFaces;
	boundaryFaces.Append(nodes0);
	boundaryFaces.Append(nodes1);

	LinkSibling (nodes0, nodes1);
	LinkSibling (nodes0, nodes1);
	LinkSibling (nodes0, nodes1);
	LinkSibling (nodes0, nodes1);

	IncMark();
	count -= 4;
	dInt32 currentIndex = 4;
	while (boundaryFaces.GetCount() && count) 
	{
		dConvexHull4dVector* const hullVertexArray = &m_points[0]; 
		dListNode* const faceNode = boundaryFaces.GetFirst()->GetInfo();
		dConvexHull4dTetraherum* const face = &faceNode->GetInfo();
		dConvexHull4dTetraherum::dgTetrahedrumPlane planeEquation (face->GetPlaneEquation (hullVertexArray));

		dInt32 index = SupportVertex (&vertexTree, points, planeEquation);

		const dConvexHull4dVector& p = points[index];
		dFloat64 dist = planeEquation.Evalue(p);
		if ((dist > distTol) && (face->Evalue(hullVertexArray, p) > dFloat64(0.0f))) 
		{
			m_points[currentIndex] = p;
			points[index].m_mark = 1;

			ndTempList deleteList;
			InsertNewVertex(currentIndex, faceNode, deleteList, boundaryFaces);

			for (ndTempList::dListNode* deleteNode = deleteList.GetFirst(); deleteNode; deleteNode = deleteNode->GetNext()) 
			{
				dListNode* const node = deleteNode->GetInfo();
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
