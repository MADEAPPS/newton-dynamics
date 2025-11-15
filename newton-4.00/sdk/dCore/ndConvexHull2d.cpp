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

#include "ndCoreStdafx.h"
#include "ndSort.h"
#include "ndConvexHull2d.h"

//ndInt32 ndConvexHull2d_new(ndVector* const vertexCloud2d, ndInt32 count)
//{
//	auto Cross = [](const ndVector& O, const ndVector& A, const ndVector& B)
//	{
//		const ndVector a0(A - O);
//		const ndVector b0(B - O);
//		const ndFloat32 sign = a0.m_y * b0.m_x - a0.m_x * b0.m_y;
//		return sign;
//	};
//
//	if (count <= 3)
//	{
//		if (count == 3)
//		{
//			ndFloat32 area = Cross(vertexCloud2d[0], vertexCloud2d[1], vertexCloud2d[2]);
//			if (area < ndFloat32(0.0f))
//			{
//				ndSwap(vertexCloud2d[1], vertexCloud2d[2]);
//			}
//		}
//		return count;
//	}
//
//
//	class ndConvexFaceNode
//	{
//	public:
//		ndVector m_point2d;
//		ndConvexFaceNode* m_next;
//		ndConvexFaceNode* m_prev;
//		ndInt32 m_mask;
//	};
//
//	ndFixSizeArray<ndVector, 256> points(256);
//	ndFixSizeArray<ndConvexFaceNode, 256> hull(256);
//
//	ndInt32 j0 = 0;
//	ndInt32 j1 = 0;
//	ndFloat32 minValue = ndFloat32(1.0e10f);
//	ndFloat32 maxValue = ndFloat32(-1.0e10f);
//	for (ndInt32 i = count - 1; i >= 0; --i)
//	{
//		points[i] = vertexCloud2d[i];
//		ndFloat32 dist = vertexCloud2d[i].m_x;
//		if (dist > maxValue)
//		{
//			j0 = i;
//			maxValue = dist;
//		}
//		if (dist < minValue)
//		{
//			j1 = i;
//			minValue = dist;
//		}
//	}
//
//	ndConvexFaceNode& edge0 = hull[0];
//	ndConvexFaceNode& edge1 = hull[1];
//
//	edge0.m_point2d = vertexCloud2d[j0];
//	edge1.m_point2d = vertexCloud2d[j1];
//
//	edge0.m_next = &edge1;
//	edge0.m_prev = &edge1;
//	edge1.m_next = &edge0;
//	edge1.m_prev = &edge0;
//
//	points[j0] = points[count - 1];
//	points[j1] = points[count - 2];
//	count -= 2;
//
//	ndInt32 head = 0;
//	ndInt32 tail = 0;
//	ndFixSizeArray<ndConvexFaceNode*, 256> queue(256);
//
//	queue[head++] = &edge0;
//	queue[head++] = &edge1;
//
//	while (tail != head)
//	{
//		ndConvexFaceNode* const edge = queue[tail++];
//		const ndVector p0(edge->m_point2d);
//		const ndVector p1(edge->m_next->m_point2d);
//	}
//
//
//	return 0;
//}

//static ndFloat32 Cross(const ndVector &O, const ndVector &A, const ndVector &B)
//{
//	const ndVector A0(A - O);
//	const ndVector B0(B - O);
//	const ndFloat32 sign = A0.m_y * B0.m_x - A0.m_x * B0.m_y;
//	return sign;
//}

ndInt32 ndConvexHull2d(ndVector* const vertexCloud2d, ndInt32 count)
{
	auto Cross = [](const ndVector& O, const ndVector& A, const ndVector& B)
	{
		const ndVector A0(A - O);
		const ndVector B0(B - O);
		const ndFloat32 sign = A0.m_y * B0.m_x - A0.m_x * B0.m_y;
		return sign;
	};

	if (count <= 3)
	{
		if (count == 3)
		{
			ndFloat32 area = Cross(vertexCloud2d[0], vertexCloud2d[1], vertexCloud2d[2]);
			if (area < ndFloat32(0.0f)) 
			{
				ndSwap(vertexCloud2d[1], vertexCloud2d[2]);
			}
		}
		return count;
	}

	// Sort points lexicographically
	class CompareVertex
	{
		public:
		CompareVertex(void*)
		{
		}

		ndInt32 Compare(const ndVector& elementA, const ndVector& elementB) const
		{
			if (elementA.m_x < elementB.m_x)
			{
				return -1;
			}
			else if (elementA.m_x > elementB.m_x)
			{
				return 1;
			}
			else
			{
				if (elementA.m_y < elementB.m_y)
				{
					return -1;
				}
				else if (elementA.m_y > elementB.m_y)
				{
					return 1;
				}
			}
			return 0;
		}
	};
	ndSort<ndVector, CompareVertex>(vertexCloud2d, count, nullptr);

	ndFixSizeArray<ndVector, 256> hull(2 * count);

	// Build lower hull
	ndInt32 k = 0;
	for (ndInt32 i = 0; i < count; ++i) 
	{
		while (k >= 2 && Cross(hull[k - 2], hull[k - 1], vertexCloud2d[i]) <= 0.0f)
		{
			--k;
		}
		hull[k] = vertexCloud2d[i];
		k++;
	}
	
	// Build upper hull
	for (ndInt32 i = count - 1, t = k + 1; i > 0; --i) 
	{
		while (k >= t && Cross(hull[k - 2], hull[k - 1], vertexCloud2d[i - 1]) <= 0.0f)
		{
			--k;
		}
		hull[k] = vertexCloud2d[i - 1];
		k++;
	}

	k--;
	for (ndInt32 i = k - 1; i >= 0; --i)
	{
		vertexCloud2d[i] = hull[i];
	}
	return k;
}

