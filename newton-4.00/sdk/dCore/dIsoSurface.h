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

#ifndef __D_ISO_SURFACE_H__
#define __D_ISO_SURFACE_H__

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dArray.h"
#include "dTree.h"

class dIsoSurface: public dClassAlloc
{
	public:
	class dIsoCell
	{
		public:
		dFloat32 m_isoValues[2][2][2];
		dInt32 m_x;
		dInt32 m_y;
		dInt32 m_z;
	};

	class dIsoTriangle
	{
		public:
		dUnsigned64 m_pointId[3];
	};

	class dIsoVertexMap: public dTree<dVector, dUnsigned64, dContainersFreeListAlloc<dVector>>
	{
		public: 
	};

	D_CORE_API dIsoSurface();
	D_CORE_API ~dIsoSurface();

	D_CORE_API void Begin(const dVector& origin, dFloat32 isovalue, dFloat32 gridSize, dInt32 sizex, dInt32 sizey, dInt32 sizez);
	D_CORE_API void ProcessCell(const dIsoCell& cell);
	D_CORE_API void End();

	dInt32 GetIndexCount() const;
	dInt32 GetVertexCount() const;
	const dVector* GetPoints() const;
	const dVector* GetNormals() const;
	const dUnsigned64* GetIndexList() const;

	private:
	dUnsigned64 GetVertexID(dInt32 x, dInt32 y, dInt32 z);
	dUnsigned64 GetEdgeID(const dIsoCell& cell, dInt32 edgeCode);
	dVector CalculateIntersection(const dIsoCell& cell, dInt32 edgeCode);
	dVector InterpolateEdge(dFloat32 fX1, dFloat32 fY1, dFloat32 fZ1, dFloat32 fX2, dFloat32 fY2, dFloat32 fZ2, dFloat32 tVal1, dFloat32 tVal2);

	void RemapIndexList();
	void CalculateNormals();

	dVector m_origin;
	dArray<dVector> m_points;
	dArray<dVector> m_normals;
	dArray<dIsoTriangle> m_trianglesList;

	dIsoVertexMap m_vertexMap;
	dInt32 m_xCellSize;
	dInt32 m_yCellSize;
	dInt32 m_zCellSize;
	dFloat32 m_gridSize;
	dFloat32 m_isoValue;

	static const dInt32 m_edgeTable[];
	static const dInt32 m_triangleTable[][16];
};

inline dInt32 dIsoSurface::GetIndexCount() const
{
	return m_trianglesList.GetCount() * 3;
}

inline dInt32 dIsoSurface::GetVertexCount() const
{
	return m_points.GetCount();
}

inline const dVector* dIsoSurface::GetPoints() const
{
	return m_points.GetCount() ? &m_points[0] : nullptr;
}

inline const dVector* dIsoSurface::GetNormals() const
{
	return m_normals.GetCount() ? &m_normals[0] : nullptr;
}

inline const dUnsigned64* dIsoSurface::GetIndexList() const
{
	return m_trianglesList.GetCount() ? &m_trianglesList[0].m_pointId[0] : nullptr;
}

#endif

