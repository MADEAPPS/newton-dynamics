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
#include "ndConvexHull2d.h"

static ndFloat32 Cross(const ndVector &O, const ndVector &A, const ndVector &B)
{
	const ndVector A0(A - O);
	const ndVector B0(B - O);
	const ndFloat32 sign = A0.m_y * B0.m_x - A0.m_x * B0.m_y;
	return sign;
}

ndInt32 dConvexHull2d(ndVector* const vertexCloud2d, ndInt32 count)
{
	if (count <= 3)
	{
		if (count == 3)
		{
			ndFloat32 area = Cross(vertexCloud2d[0], vertexCloud2d[1], vertexCloud2d[2]);
			if (area < ndFloat32(0.0f)) 
			{
				dSwap(vertexCloud2d[1], vertexCloud2d[2]);
			}
		}
		return count;
	}

	ndVector* const hull = dAlloca(ndVector, 2 * count);

	// Sort points lexicographically
	class CompareVertex
	{
		public:
		ndInt32 Compare(const ndVector& elementA, const ndVector& elementB, void* const) const
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
	ndSort<ndVector, CompareVertex>(vertexCloud2d, count);

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

	for (ndInt32 i = 0; i < k; ++i)
	{
		vertexCloud2d[i] = hull[i];
	}
	return k - 1;
}
