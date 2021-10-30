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

#include "dCoreStdafx.h"
#include "dSort.h"
#include "dConvexHull2d.h"

static dFloat32 Cross(const dVector &O, const dVector &A, const dVector &B)
{
	dVector A0(A - O);
	dVector B0(B - O);
	return A0.m_x * B0.m_y - A0.m_y * B0.m_x;
}

dInt32 dConvexHull2d(dVector* const vertexCloud2d, dInt32 count)
{
	if (count <= 3)
	{
		return count;
	}

	dVector* const hull = dAlloca(dVector, 2 * count);

	// Sort points lexicographically
	class CompareVertex
	{
		public:
		dInt32 Compare(const dVector& elementA, const dVector& elementB, void* const)
		{
			if (elementA.m_x < elementB.m_x)
			{
				return 1;
			}
			else if (elementA.m_x > elementB.m_x)
			{
				return -1;
			}
			else
			{
				if (elementA.m_y < elementB.m_y)
				{
					return 1;
				}
				else if (elementA.m_y > elementB.m_y)
				{
					return -1;
				}
			}
			return 0;
		}
	};
	dSort<dVector, CompareVertex>(vertexCloud2d, count);

	// Build lower hull
	dInt32 k = 0;
	for (dInt32 i = 0; i < count; i++) 
	{
		while (k >= 2 && Cross(hull[k - 2], hull[k - 1], vertexCloud2d[i]) <= 0.0f)
		{
			k--;
		}
		hull[k] = vertexCloud2d[i];
		k++;
	}
	
	// Build upper hull
	for (dInt32 i = count - 1, t = k + 1; i > 0; i--) 
	{
		while (k >= t && Cross(hull[k - 2], hull[k - 1], vertexCloud2d[i - 1]) <= 0.0f)
		{
			k--;
		}
		hull[k] = vertexCloud2d[i - 1];
		k++;
	}

	memcpy(vertexCloud2d, hull, k * sizeof(dVector));
	return k - 1;
}
