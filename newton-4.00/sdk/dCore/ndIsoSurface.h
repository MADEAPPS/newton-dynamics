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

#ifndef __ND_ISO_SURFACE_H__
#define __ND_ISO_SURFACE_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndArray.h"
#include "ndTree.h"

class ndIsoSurface: public ndClassAlloc
{
	public:
	class ndCalculateIsoValue
	{
		public:
		ndCalculateIsoValue()
		{
		}

		virtual ~ndCalculateIsoValue()
		{
		}

		virtual ndFloat32 CalculateIsoValue(const ndVector& point) const = 0;
	};

	class ndImplementation;

	ndIsoSurface();
	D_CORE_API ~ndIsoSurface();

	ndVector GetOrigin() const;
	const ndArray<ndVector>& GetPoints() const;

	D_CORE_API void GenerateMesh(const ndArray<ndVector>& pointCloud, ndFloat32 gridSize, ndCalculateIsoValue* const computeIsoValue = nullptr);
	D_CORE_API ndInt32 GenerateListIndexList(ndInt32 * const indexList, ndInt32 strideInFloat32, ndReal* const posit, ndReal* const normals) const;

	private:
	ndImplementation& GetImplementation() const;

	ndVector m_origin;
	ndArray<ndVector> m_points;
	ndFloat32 m_gridSize;
	ndInt32 m_volumeSizeX;
	ndInt32 m_volumeSizeY;
	ndInt32 m_volumeSizeZ;
	bool m_isLowRes;
	friend class ndImplementation;
};

inline ndIsoSurface::ndIsoSurface()
	:m_origin(ndVector::m_zero)
	,m_points(1024)
	,m_gridSize(ndFloat32 (1.0f))
	,m_volumeSizeX(1)
	,m_volumeSizeY(1)
	,m_volumeSizeZ(1)
	,m_isLowRes(true)
{
}

inline const ndArray<ndVector>& ndIsoSurface::GetPoints() const
{
	return m_points;
}

inline ndVector ndIsoSurface::GetOrigin() const
{
	return m_origin;
}

#endif

