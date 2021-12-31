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
	class ndIsoCell;

	class ndIsoTriangle
	{
		public:
		ndIsoTriangle() {}
		ndIsoTriangle(ndInt32) {}
		bool operator<(const ndIsoTriangle& triangle) const;
		bool operator>(const ndIsoTriangle& triangle) const;

		ndUnsigned64 m_pointId[3];
	};

	class ndIsoVertexMap: public ndTree<ndVector, ndUnsigned64, ndContainersFreeListAlloc<ndVector>>
	{
		public: 
	};

	class ndIsoTriangleMap : public ndTree<ndInt32, ndIsoTriangle, ndContainersFreeListAlloc<ndIsoTriangle>>
	{
		public:
	};

	class ndOctreeInterface : public ndContainersFreeListAlloc<ndOctreeInterface>
	{
		public:
		ndOctreeInterface(ndOctreeInterface* const parent)
			:ndContainersFreeListAlloc<ndOctreeInterface>() 
			,m_parent(parent)
			,m_isLeaf(0)
			,m_fullMask(0)
		{
		}

		virtual ~ndOctreeInterface() {}

		virtual void ProccessCells(ndIsoSurface* const isoSurface) = 0;
		virtual void Insert(const ndVector&, ndInt32) {}
		virtual bool Find(const ndVector&) const { dAssert(0); return false; }
		ndInt32 CheckInside(const ndVector& point, const ndVector& box0, const ndVector& box1) const;
		
		ndOctreeInterface* m_parent;
		ndUnsigned8 m_isLeaf;
		ndUnsigned8 m_fullMask;

		static ndVector m_neighbors[3][3][3];
	};

	class ndOctree : public ndOctreeInterface
	{
		public: 
		ndOctree(const ndVector& box0, const ndVector& box1, ndOctreeInterface* const parent);
		virtual ~ndOctree();

		virtual bool Find(const ndVector& point) const;
		virtual void ProccessCells(ndIsoSurface* const isoSurface);
		virtual void Insert(const ndVector& point, ndInt32 parentQuadrant);

		ndVector m_box0;
		ndVector m_box1;
		static ndVector m_quadrantCode;
		ndOctreeInterface* m_children[8];
	};

	class ndOctreeLeaf : public ndOctreeInterface
	{
		public:
		ndOctreeLeaf(const ndVector& point, ndOctreeInterface* const parent)
			:ndOctreeInterface(parent)
			,m_point(point)
		{
			m_isLeaf = 1;
		}
		virtual ~ndOctreeLeaf() {}
		virtual void ProccessCells(ndIsoSurface* const isoSurface);

		ndVector m_point;
	};

	D_CORE_API ndIsoSurface();
	D_CORE_API ~ndIsoSurface();

	D_CORE_API void Begin(const ndVector& boxP0, const ndVector& boxP1, ndFloat32 gridSize);
	D_CORE_API void	AddPoint(const ndVector& point);
	D_CORE_API void End();

	ndInt32 GetIndexCount() const;
	ndInt32 GetVertexCount() const;
	const ndVector* GetPoints() const;
	const ndVector* GetNormals() const;
	const ndUnsigned64* GetIndexList() const;

	private:
	void ProcessCell(const ndIsoCell& cell);
	ndUnsigned64 GetVertexID(ndInt32 x, ndInt32 y, ndInt32 z);
	ndUnsigned64 GetEdgeID(const ndIsoCell& cell, ndInt32 edgeCode);
	ndVector CalculateIntersection(const ndIsoCell& cell, ndInt32 edgeCode);
	ndVector InterpolateEdge(ndFloat32 fX1, ndFloat32 fY1, ndFloat32 fZ1, ndFloat32 fX2, ndFloat32 fY2, ndFloat32 fZ2, ndFloat32 tVal1, ndFloat32 tVal2);

	void RemapIndexList();
	void CalculateNormals();

	ndVector m_origin;
	ndVector m_gridSize;
	ndVector m_invGridSize;
	ndArray<ndVector> m_points;
	ndArray<ndVector> m_normals;
	ndArray<ndIsoTriangle> m_trianglesList;

	ndIsoVertexMap m_vertexMap;
	ndIsoTriangleMap m_triangleMap;

	ndOctreeInterface* m_octree;
	ndInt32 m_xCellSize;
	ndInt32 m_yCellSize;
	ndInt32 m_zCellSize;
	
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

inline ndInt32 ndIsoSurface::ndOctreeInterface::CheckInside(const ndVector& point, const ndVector& box0, const ndVector& box1) const
{
	const ndVector p0(box0 - point);
	const ndVector p1(box1 - point);
	const ndVector diff(p1 * p0);
	const ndVector mask(diff <= ndVector::m_zero);
	ndInt32 isInsize = mask.m_ix & mask.m_iy & mask.m_iz;
	return isInsize;
}

inline bool ndIsoSurface::ndIsoTriangle::operator<(const ndIsoTriangle& triangle) const
{
	if (m_pointId[2] < triangle.m_pointId[2])
	{
		return true;
	}
	if (m_pointId[1] < triangle.m_pointId[1])
	{
		return true;
	}
	return m_pointId[0] < triangle.m_pointId[0];
}

inline bool ndIsoSurface::ndIsoTriangle::operator>(const ndIsoTriangle& triangle) const
{
	if (m_pointId[2] > triangle.m_pointId[2])
	{
		return true;
	}
	if (m_pointId[1] > triangle.m_pointId[1])
	{
		return true;
	}
	return m_pointId[0] > triangle.m_pointId[0];
}

#endif

