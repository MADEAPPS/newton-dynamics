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

// adapted from code by written by Paul Bourke may 1994
//http://paulbourke.net/geometry/polygonise/

#ifndef __ND_ISO_SURFACE_H__
#define __ND_ISO_SURFACE_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndArray.h"
#include "ndTree.h"

class ndIsoSurface: public ndClassAlloc
{
	public:
	class ndIsoCell
	{
		public:
		ndFloat32 m_isoValues[2][2][2];
		ndInt32 m_x;
		ndInt32 m_y;
		ndInt32 m_z;
	};

	class ndIsoTriangle
	{
		public:
		ndUnsigned64 m_pointId[3];
	};

	class ndIsoVertexMap: public ndTree<ndVector, ndUnsigned64, ndContainersFreeListAlloc<ndVector>>
	{
		public: 
	};

	D_CORE_API ndIsoSurface();
	D_CORE_API ~ndIsoSurface();

	D_CORE_API void Begin(const ndVector& origin, ndFloat32 isovalue, ndFloat32 gridSize, ndInt32 sizex, ndInt32 sizey, ndInt32 sizez);
	D_CORE_API void ProcessCell(const ndIsoCell& cell);
	D_CORE_API void End();

	ndInt32 GetIndexCount() const;
	ndInt32 GetVertexCount() const;
	const ndVector* GetPoints() const;
	const ndVector* GetNormals() const;
	const ndUnsigned64* GetIndexList() const;

	private:
	ndUnsigned64 GetVertexID(ndInt32 x, ndInt32 y, ndInt32 z);
	ndUnsigned64 GetEdgeID(const ndIsoCell& cell, ndInt32 edgeCode);
	ndVector CalculateIntersection(const ndIsoCell& cell, ndInt32 edgeCode);
	ndVector InterpolateEdge(ndFloat32 fX1, ndFloat32 fY1, ndFloat32 fZ1, ndFloat32 fX2, ndFloat32 fY2, ndFloat32 fZ2, ndFloat32 tVal1, ndFloat32 tVal2);

	void RemapIndexList();
	void CalculateNormals();

	ndVector m_origin;
	ndArray<ndVector> m_points;
	ndArray<ndVector> m_normals;
	ndArray<ndIsoTriangle> m_trianglesList;

	ndIsoVertexMap m_vertexMap;
	ndInt32 m_xCellSize;
	ndInt32 m_yCellSize;
	ndInt32 m_zCellSize;
	ndFloat32 m_gridSize;
	ndFloat32 m_isoValue;

	static const ndInt32 m_edgeTable[];
	static const ndInt32 m_triangleTable[][16];
};

inline ndInt32 ndIsoSurface::GetIndexCount() const
{
	return m_trianglesList.GetCount() * 3;
}

inline ndInt32 ndIsoSurface::GetVertexCount() const
{
	return m_points.GetCount();
}

inline const ndVector* ndIsoSurface::GetPoints() const
{
	return m_points.GetCount() ? &m_points[0] : nullptr;
}

inline const ndVector* ndIsoSurface::GetNormals() const
{
	return m_normals.GetCount() ? &m_normals[0] : nullptr;
}

inline const ndUnsigned64* ndIsoSurface::GetIndexList() const
{
	return m_trianglesList.GetCount() ? &m_trianglesList[0].m_pointId[0] : nullptr;
}

#endif

