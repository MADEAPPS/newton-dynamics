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

#ifndef __ND_SPATIAL_MATRIX_H__
#define __ND_SPATIAL_MATRIX_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndVector.h"
#include "ndSpatialVector.h"

class ndSpatialMatrix
{
	public:
	D_OPERATOR_NEW_AND_DELETE

	inline ndSpatialMatrix()
	{
	}

	inline ndSpatialMatrix(ndFloat32 val)
	{
		const ndSpatialVector row (val);
		for (ndInt32 i = 0; i < 6; i++) 
		{
			m_rows[i] = row;
		}
	}

	inline ~ndSpatialMatrix()
	{
	}
	
	inline ndSpatialVector& operator[] (ndInt32 i)
	{
		dAssert(i < 6);
		dAssert(i >= 0);
		return m_rows[i];
	}

	inline const ndSpatialVector& operator[] (ndInt32 i) const
	{
		dAssert(i < 6);
		dAssert(i >= 0);
		return m_rows[i];
	}

	D_CORE_API ndSpatialMatrix Inverse(ndInt32 rows) const;

	inline ndSpatialVector VectorTimeMatrix(const ndSpatialVector& jacobian) const
	{
		ndSpatialVector tmp(m_rows[0].Scale (jacobian[0]));
		for (ndInt32 i = 1; i < 6; i++) 
		{
			tmp = tmp + m_rows[i].Scale(jacobian[i]);
		}
		return tmp;
	}

	inline ndSpatialVector VectorTimeMatrix(const ndSpatialVector& jacobian, ndInt32 dof) const
	{
		ndSpatialVector tmp(ndFloat32 (0.0f));
		for (ndInt32 i = 0; i < dof; i++) 
		{
			tmp = tmp + m_rows[i].Scale(jacobian[i]);
		}
		return tmp;
	}

	ndSpatialVector m_rows[6];
};

#endif

