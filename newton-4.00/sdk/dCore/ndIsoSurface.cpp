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
#include "ndDebug.h"
#include "ndVector.h"
#include "ndMatrix.h"
#include "ndProfiler.h"
#include "ndIsoSurface.h"

#define D_RADIX_DIGIT_SIZE 8

// adapted from code by written by Paul Bourke may 1994
//http://paulbourke.net/geometry/polygonise/

class ndIsoSurface::ndImplementation : public ndClassAlloc
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

	class ndIsoCell_1
	{
		public:
		ndVector m_isoValues[8];
	};

	class ndIsoTriangle_1
	{
		public:
		ndVector m_p[3];
	};
	
	class ndIsoTriangle
	{
		public:
		ndIsoTriangle() {}
		ndIsoTriangle(ndInt32) {}
		bool operator<(const ndIsoTriangle& triangle) const
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

		bool operator>(const ndIsoTriangle& triangle) const
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
		ndUnsigned64 m_pointId[3];
	};

	class ndIsoTriangleMap : public ndTree<ndInt32, ndIsoTriangle, ndContainersFreeListAlloc<ndIsoTriangle>>
	{
		public:
	};

	class ndIsoVertexMap : public ndTree<ndVector, ndUnsigned64, ndContainersFreeListAlloc<ndVector>>
	{
		public:
	};

	ndImplementation();
	~ndImplementation();

	void BuildIndexListMesh(ndIsoSurface* const me, const ndArray<ndVector>& pointCloud, ndFloat32 gridSize);
	void BuildTriangleListMesh(ndIsoSurface* const me, const ndArray<ndVector>& pointCloud, ndFloat32 gridSize);

	private:
	class ndGridHash
	{
		public:
		ndGridHash()
		{
		}

		ndGridHash(const ndGridHash& src, ndInt8 cellType)
			:m_gridCellHash(src.m_gridCellHash)
			,m_cellType(cellType)
		{
		}

		ndGridHash(ndInt32 x, ndInt32 y, ndInt32 z)
		{
			m_gridCellHash = 0;
			m_x = ndInt16(x);
			m_y = ndInt16(y);
			m_z = ndInt16(z);
		}

		ndGridHash(const ndVector& grid)
		{
			dAssert(grid.m_x >= ndFloat32(0.0f));
			dAssert(grid.m_y >= ndFloat32(0.0f));
			dAssert(grid.m_z >= ndFloat32(0.0f));
			dAssert(grid.m_x < ndFloat32(1 << (D_RADIX_DIGIT_SIZE * 2)));
			dAssert(grid.m_y < ndFloat32(1 << (D_RADIX_DIGIT_SIZE * 2)));
			dAssert(grid.m_z < ndFloat32(1 << (D_RADIX_DIGIT_SIZE * 2)));
			
			ndVector hash(grid.GetInt());
			m_gridCellHash = 0;
			m_x = ndInt16(hash.m_ix);
			m_y = ndInt16(hash.m_iy);
			m_z = ndInt16(hash.m_iz);
		}

		union
		{
			struct
			{
				ndUnsigned16 m_x;
				ndUnsigned16 m_y;
				ndUnsigned16 m_z;
				ndUnsigned8 m_cellType;
			};
			struct
			{
				ndUnsigned8 m_xLow;
				ndUnsigned8 m_xHigh;
				ndUnsigned8 m_yLow;
				ndUnsigned8 m_yHigh;
				ndUnsigned8 m_zLow;
				ndUnsigned8 m_zHigh;
			};
			ndUnsigned64 m_gridCellHash : 48;
			ndUnsigned64 m_gridFullHash;
		};
	};

	class ndGridHashSteps
	{
		public:
		ndGridHashSteps()
		{
			m_steps[0] = ndGridHash(-1, -1, -1);
			m_steps[1] = ndGridHash( 0, -1, -1);
			m_steps[2] = ndGridHash(-1,  0, -1);
			m_steps[3] = ndGridHash( 0,  0, -1);
			m_steps[4] = ndGridHash(-1, -1,  0);
			m_steps[5] = ndGridHash( 0, -1,  0);
			m_steps[6] = ndGridHash(-1,  0,  0);
			m_steps[7] = ndGridHash( 0,  0,  0);

			m_cellType[0] = 7;
			m_cellType[1] = 6;
			m_cellType[2] = 5;
			m_cellType[3] = 4;
			m_cellType[4] = 3;
			m_cellType[5] = 2;
			m_cellType[6] = 1;
			m_cellType[7] = 0;
		}

		ndGridHash m_steps[8];
		ndUnsigned8 m_cellType[8];
	};

	class ndGridHashSteps_1
	{
		public:
		ndGridHashSteps_1()
		{
			m_steps[0] = ndGridHash(-1, -1, -1);
			m_steps[1] = ndGridHash(0, -1, -1);
			m_steps[2] = ndGridHash(-1, 0, -1);
			m_steps[3] = ndGridHash(0, 0, -1);
			m_steps[4] = ndGridHash(-1, -1, 0);
			m_steps[5] = ndGridHash(0, -1, 0);
			m_steps[6] = ndGridHash(-1, 0, 0);
			m_steps[7] = ndGridHash(0, 0, 0);

			m_cellType[0] = 5;
			m_cellType[1] = 6;
			m_cellType[2] = 1;
			m_cellType[3] = 2;
			m_cellType[4] = 4;
			m_cellType[5] = 7;
			m_cellType[6] = 0;
			m_cellType[7] = 3;
		}

		ndGridHash m_steps[8];
		ndUnsigned8 m_cellType[8];
	};
	
	class ndUpperDigit
	{
		public:
		ndUpperDigit()
			:m_x(0)
			,m_y(0)
			,m_z(0)
		{
		}
		ndInt32 m_x;
		ndInt32 m_y;
		ndInt32 m_z;
	};

	void ClearBuffers();
	void ClearBuffers_1();
	void RemoveDuplicates();
	void SortCellBuckects();
	void GenerateIsoSurface();
	void GenerateIsoSurface_1();
	void RemapIndexArray(ndIsoSurface* const me);
	void RemapVertexArray(ndIsoSurface* const me);
	void CalculateNormals(ndIsoSurface* const me);
	void GenerateIndexList(ndIsoSurface* const me);
	void ProcessIndexListCell(const ndIsoCell& cell);
	void CreateGrids(const ndArray<ndVector>& points);
	void CreateGrids_1(const ndArray<ndVector>& points);
	void ProcessTriangleListCell(const ndIsoCell_1& cell);
	ndUnsigned64 GetEdgeID(const ndIsoCell& cell, ndInt32 edgeCode);
	ndUnsigned64 GetVertexID(ndInt32 gridX, ndInt32 gridY, ndInt32 gridZ);
	ndVector CalculateIntersection(const ndIsoCell& cell, ndInt32 edgeCode);
	void CalculatedAabb(const ndArray<ndVector>& points, ndFloat32 gridSize);

	ndVector VertexInterp(ndFloat32 isolevel, const ndVector& p1, const ndVector& p2) const;
	ndVector InterpolateEdge(ndFloat32 fX1, ndFloat32 fY1, ndFloat32 fZ1, ndFloat32 fX2, ndFloat32 fY2, ndFloat32 fZ2, ndFloat32 tVal1, ndFloat32 tVal2);

	ndVector m_boxP0;
	ndVector m_boxP1;
	ndVector m_origin;
	ndVector m_gridSize;
	ndVector m_invGridSize;
	ndIsoVertexMap m_vertexMap;
	ndIsoTriangleMap m_triangleMap;
	ndFloat32 m_isoValue;
	ndInt32 m_xCellSize;
	ndInt32 m_yCellSize;
	ndInt32 m_zCellSize;

	ndArray<ndGridHash> m_hashGridMap;
	ndArray<ndGridHash> m_hashGridMapScratchBuffer;
	ndUpperDigit m_upperDigitsIsValid;

	ndArray<ndIsoTriangle_1> m_triangles;

	static const ndInt32 m_edgeTable[];
	static const ndInt32 m_triangleTable[][16];
};

const ndInt32 ndIsoSurface::ndImplementation::m_edgeTable[256] =
{
	0x0  , 0x109, 0x203, 0x30a, 0x406, 0x50f, 0x605, 0x70c,
	0x80c, 0x905, 0xa0f, 0xb06, 0xc0a, 0xd03, 0xe09, 0xf00,
	0x190, 0x99 , 0x393, 0x29a, 0x596, 0x49f, 0x795, 0x69c,
	0x99c, 0x895, 0xb9f, 0xa96, 0xd9a, 0xc93, 0xf99, 0xe90,
	0x230, 0x339, 0x33 , 0x13a, 0x636, 0x73f, 0x435, 0x53c,
	0xa3c, 0xb35, 0x83f, 0x936, 0xe3a, 0xf33, 0xc39, 0xd30,
	0x3a0, 0x2a9, 0x1a3, 0xaa , 0x7a6, 0x6af, 0x5a5, 0x4ac,
	0xbac, 0xaa5, 0x9af, 0x8a6, 0xfaa, 0xea3, 0xda9, 0xca0,
	0x460, 0x569, 0x663, 0x76a, 0x66 , 0x16f, 0x265, 0x36c,
	0xc6c, 0xd65, 0xe6f, 0xf66, 0x86a, 0x963, 0xa69, 0xb60,
	0x5f0, 0x4f9, 0x7f3, 0x6fa, 0x1f6, 0xff , 0x3f5, 0x2fc,
	0xdfc, 0xcf5, 0xfff, 0xef6, 0x9fa, 0x8f3, 0xbf9, 0xaf0,
	0x650, 0x759, 0x453, 0x55a, 0x256, 0x35f, 0x55 , 0x15c,
	0xe5c, 0xf55, 0xc5f, 0xd56, 0xa5a, 0xb53, 0x859, 0x950,
	0x7c0, 0x6c9, 0x5c3, 0x4ca, 0x3c6, 0x2cf, 0x1c5, 0xcc ,
	0xfcc, 0xec5, 0xdcf, 0xcc6, 0xbca, 0xac3, 0x9c9, 0x8c0,
	0x8c0, 0x9c9, 0xac3, 0xbca, 0xcc6, 0xdcf, 0xec5, 0xfcc,
	0xcc , 0x1c5, 0x2cf, 0x3c6, 0x4ca, 0x5c3, 0x6c9, 0x7c0,
	0x950, 0x859, 0xb53, 0xa5a, 0xd56, 0xc5f, 0xf55, 0xe5c,
	0x15c, 0x55 , 0x35f, 0x256, 0x55a, 0x453, 0x759, 0x650,
	0xaf0, 0xbf9, 0x8f3, 0x9fa, 0xef6, 0xfff, 0xcf5, 0xdfc,
	0x2fc, 0x3f5, 0xff , 0x1f6, 0x6fa, 0x7f3, 0x4f9, 0x5f0,
	0xb60, 0xa69, 0x963, 0x86a, 0xf66, 0xe6f, 0xd65, 0xc6c,
	0x36c, 0x265, 0x16f, 0x66 , 0x76a, 0x663, 0x569, 0x460,
	0xca0, 0xda9, 0xea3, 0xfaa, 0x8a6, 0x9af, 0xaa5, 0xbac,
	0x4ac, 0x5a5, 0x6af, 0x7a6, 0xaa , 0x1a3, 0x2a9, 0x3a0,
	0xd30, 0xc39, 0xf33, 0xe3a, 0x936, 0x83f, 0xb35, 0xa3c,
	0x53c, 0x435, 0x73f, 0x636, 0x13a, 0x33 , 0x339, 0x230,
	0xe90, 0xf99, 0xc93, 0xd9a, 0xa96, 0xb9f, 0x895, 0x99c,
	0x69c, 0x795, 0x49f, 0x596, 0x29a, 0x393, 0x99 , 0x190,
	0xf00, 0xe09, 0xd03, 0xc0a, 0xb06, 0xa0f, 0x905, 0x80c,
	0x70c, 0x605, 0x50f, 0x406, 0x30a, 0x203, 0x109, 0x0
};

const ndInt32 ndIsoSurface::ndImplementation::m_triangleTable[256][16] =
{
	{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 1, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 8, 3, 9, 8, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 8, 3, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 2, 10, 0, 2, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 8, 3, 2, 10, 8, 10, 9, 8, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 11, 2, 8, 11, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 9, 0, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 11, 2, 1, 9, 11, 9, 8, 11, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 10, 1, 11, 10, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 10, 1, 0, 8, 10, 8, 11, 10, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 9, 0, 3, 11, 9, 11, 10, 9, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 3, 0, 7, 3, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 1, 9, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 1, 9, 4, 7, 1, 7, 3, 1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 2, 10, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 4, 7, 3, 0, 4, 1, 2, 10, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 2, 10, 9, 0, 2, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 10, 9, 2, 9, 7, 2, 7, 3, 7, 9, 4, -1, -1, -1, -1 },
	{ 8, 4, 7, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 11, 4, 7, 11, 2, 4, 2, 0, 4, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 0, 1, 8, 4, 7, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 7, 11, 9, 4, 11, 9, 11, 2, 9, 2, 1, -1, -1, -1, -1 },
	{ 3, 10, 1, 3, 11, 10, 7, 8, 4, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 11, 10, 1, 4, 11, 1, 0, 4, 7, 11, 4, -1, -1, -1, -1 },
	{ 4, 7, 8, 9, 0, 11, 9, 11, 10, 11, 0, 3, -1, -1, -1, -1 },
	{ 4, 7, 11, 4, 11, 9, 9, 11, 10, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 5, 4, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 5, 4, 1, 5, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 8, 5, 4, 8, 3, 5, 3, 1, 5, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 2, 10, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 0, 8, 1, 2, 10, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1 },
	{ 5, 2, 10, 5, 4, 2, 4, 0, 2, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 10, 5, 3, 2, 5, 3, 5, 4, 3, 4, 8, -1, -1, -1, -1 },
	{ 9, 5, 4, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 11, 2, 0, 8, 11, 4, 9, 5, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 5, 4, 0, 1, 5, 2, 3, 11, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 1, 5, 2, 5, 8, 2, 8, 11, 4, 8, 5, -1, -1, -1, -1 },
	{ 10, 3, 11, 10, 1, 3, 9, 5, 4, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 9, 5, 0, 8, 1, 8, 10, 1, 8, 11, 10, -1, -1, -1, -1 },
	{ 5, 4, 0, 5, 0, 11, 5, 11, 10, 11, 0, 3, -1, -1, -1, -1 },
	{ 5, 4, 8, 5, 8, 10, 10, 8, 11, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 7, 8, 5, 7, 9, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 3, 0, 9, 5, 3, 5, 7, 3, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 7, 8, 0, 1, 7, 1, 5, 7, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 7, 8, 9, 5, 7, 10, 1, 2, -1, -1, -1, -1, -1, -1, -1 },
	{ 10, 1, 2, 9, 5, 0, 5, 3, 0, 5, 7, 3, -1, -1, -1, -1 },
	{ 8, 0, 2, 8, 2, 5, 8, 5, 7, 10, 5, 2, -1, -1, -1, -1 },
	{ 2, 10, 5, 2, 5, 3, 3, 5, 7, -1, -1, -1, -1, -1, -1, -1 },
	{ 7, 9, 5, 7, 8, 9, 3, 11, 2, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 5, 7, 9, 7, 2, 9, 2, 0, 2, 7, 11, -1, -1, -1, -1 },
	{ 2, 3, 11, 0, 1, 8, 1, 7, 8, 1, 5, 7, -1, -1, -1, -1 },
	{ 11, 2, 1, 11, 1, 7, 7, 1, 5, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 5, 8, 8, 5, 7, 10, 1, 3, 10, 3, 11, -1, -1, -1, -1 },
	{ 5, 7, 0, 5, 0, 9, 7, 11, 0, 1, 0, 10, 11, 10, 0, -1 },
	{ 11, 10, 0, 11, 0, 3, 10, 5, 0, 8, 0, 7, 5, 7, 0, -1 },
	{ 11, 10, 5, 7, 11, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 8, 3, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 0, 1, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 8, 3, 1, 9, 8, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 6, 5, 2, 6, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 6, 5, 1, 2, 6, 3, 0, 8, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 6, 5, 9, 0, 6, 0, 2, 6, -1, -1, -1, -1, -1, -1, -1 },
	{ 5, 9, 8, 5, 8, 2, 5, 2, 6, 3, 2, 8, -1, -1, -1, -1 },
	{ 2, 3, 11, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 11, 0, 8, 11, 2, 0, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 1, 9, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1, -1, -1, -1 },
	{ 5, 10, 6, 1, 9, 2, 9, 11, 2, 9, 8, 11, -1, -1, -1, -1 },
	{ 6, 3, 11, 6, 5, 3, 5, 1, 3, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 8, 11, 0, 11, 5, 0, 5, 1, 5, 11, 6, -1, -1, -1, -1 },
	{ 3, 11, 6, 0, 3, 6, 0, 6, 5, 0, 5, 9, -1, -1, -1, -1 },
	{ 6, 5, 9, 6, 9, 11, 11, 9, 8, -1, -1, -1, -1, -1, -1, -1 },
	{ 5, 10, 6, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 3, 0, 4, 7, 3, 6, 5, 10, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 9, 0, 5, 10, 6, 8, 4, 7, -1, -1, -1, -1, -1, -1, -1 },
	{ 10, 6, 5, 1, 9, 7, 1, 7, 3, 7, 9, 4, -1, -1, -1, -1 },
	{ 6, 1, 2, 6, 5, 1, 4, 7, 8, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 2, 5, 5, 2, 6, 3, 0, 4, 3, 4, 7, -1, -1, -1, -1 },
	{ 8, 4, 7, 9, 0, 5, 0, 6, 5, 0, 2, 6, -1, -1, -1, -1 },
	{ 7, 3, 9, 7, 9, 4, 3, 2, 9, 5, 9, 6, 2, 6, 9, -1 },
	{ 3, 11, 2, 7, 8, 4, 10, 6, 5, -1, -1, -1, -1, -1, -1, -1 },
	{ 5, 10, 6, 4, 7, 2, 4, 2, 0, 2, 7, 11, -1, -1, -1, -1 },
	{ 0, 1, 9, 4, 7, 8, 2, 3, 11, 5, 10, 6, -1, -1, -1, -1 },
	{ 9, 2, 1, 9, 11, 2, 9, 4, 11, 7, 11, 4, 5, 10, 6, -1 },
	{ 8, 4, 7, 3, 11, 5, 3, 5, 1, 5, 11, 6, -1, -1, -1, -1 },
	{ 5, 1, 11, 5, 11, 6, 1, 0, 11, 7, 11, 4, 0, 4, 11, -1 },
	{ 0, 5, 9, 0, 6, 5, 0, 3, 6, 11, 6, 3, 8, 4, 7, -1 },
	{ 6, 5, 9, 6, 9, 11, 4, 7, 9, 7, 11, 9, -1, -1, -1, -1 },
	{ 10, 4, 9, 6, 4, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 10, 6, 4, 9, 10, 0, 8, 3, -1, -1, -1, -1, -1, -1, -1 },
	{ 10, 0, 1, 10, 6, 0, 6, 4, 0, -1, -1, -1, -1, -1, -1, -1 },
	{ 8, 3, 1, 8, 1, 6, 8, 6, 4, 6, 1, 10, -1, -1, -1, -1 },
	{ 1, 4, 9, 1, 2, 4, 2, 6, 4, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 0, 8, 1, 2, 9, 2, 4, 9, 2, 6, 4, -1, -1, -1, -1 },
	{ 0, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 8, 3, 2, 8, 2, 4, 4, 2, 6, -1, -1, -1, -1, -1, -1, -1 },
	{ 10, 4, 9, 10, 6, 4, 11, 2, 3, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 8, 2, 2, 8, 11, 4, 9, 10, 4, 10, 6, -1, -1, -1, -1 },
	{ 3, 11, 2, 0, 1, 6, 0, 6, 4, 6, 1, 10, -1, -1, -1, -1 },
	{ 6, 4, 1, 6, 1, 10, 4, 8, 1, 2, 1, 11, 8, 11, 1, -1 },
	{ 9, 6, 4, 9, 3, 6, 9, 1, 3, 11, 6, 3, -1, -1, -1, -1 },
	{ 8, 11, 1, 8, 1, 0, 11, 6, 1, 9, 1, 4, 6, 4, 1, -1 },
	{ 3, 11, 6, 3, 6, 0, 0, 6, 4, -1, -1, -1, -1, -1, -1, -1 },
	{ 6, 4, 8, 11, 6, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 7, 10, 6, 7, 8, 10, 8, 9, 10, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 7, 3, 0, 10, 7, 0, 9, 10, 6, 7, 10, -1, -1, -1, -1 },
	{ 10, 6, 7, 1, 10, 7, 1, 7, 8, 1, 8, 0, -1, -1, -1, -1 },
	{ 10, 6, 7, 10, 7, 1, 1, 7, 3, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 2, 6, 1, 6, 8, 1, 8, 9, 8, 6, 7, -1, -1, -1, -1 },
	{ 2, 6, 9, 2, 9, 1, 6, 7, 9, 0, 9, 3, 7, 3, 9, -1 },
	{ 7, 8, 0, 7, 0, 6, 6, 0, 2, -1, -1, -1, -1, -1, -1, -1 },
	{ 7, 3, 2, 6, 7, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 3, 11, 10, 6, 8, 10, 8, 9, 8, 6, 7, -1, -1, -1, -1 },
	{ 2, 0, 7, 2, 7, 11, 0, 9, 7, 6, 7, 10, 9, 10, 7, -1 },
	{ 1, 8, 0, 1, 7, 8, 1, 10, 7, 6, 7, 10, 2, 3, 11, -1 },
	{ 11, 2, 1, 11, 1, 7, 10, 6, 1, 6, 7, 1, -1, -1, -1, -1 },
	{ 8, 9, 6, 8, 6, 7, 9, 1, 6, 11, 6, 3, 1, 3, 6, -1 },
	{ 0, 9, 1, 11, 6, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 7, 8, 0, 7, 0, 6, 3, 11, 0, 11, 6, 0, -1, -1, -1, -1 },
	{ 7, 11, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 0, 8, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 1, 9, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 8, 1, 9, 8, 3, 1, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1 },
	{ 10, 1, 2, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 2, 10, 3, 0, 8, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 9, 0, 2, 10, 9, 6, 11, 7, -1, -1, -1, -1, -1, -1, -1 },
	{ 6, 11, 7, 2, 10, 3, 10, 8, 3, 10, 9, 8, -1, -1, -1, -1 },
	{ 7, 2, 3, 6, 2, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 7, 0, 8, 7, 6, 0, 6, 2, 0, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 7, 6, 2, 3, 7, 0, 1, 9, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 6, 2, 1, 8, 6, 1, 9, 8, 8, 7, 6, -1, -1, -1, -1 },
	{ 10, 7, 6, 10, 1, 7, 1, 3, 7, -1, -1, -1, -1, -1, -1, -1 },
	{ 10, 7, 6, 1, 7, 10, 1, 8, 7, 1, 0, 8, -1, -1, -1, -1 },
	{ 0, 3, 7, 0, 7, 10, 0, 10, 9, 6, 10, 7, -1, -1, -1, -1 },
	{ 7, 6, 10, 7, 10, 8, 8, 10, 9, -1, -1, -1, -1, -1, -1, -1 },
	{ 6, 8, 4, 11, 8, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 6, 11, 3, 0, 6, 0, 4, 6, -1, -1, -1, -1, -1, -1, -1 },
	{ 8, 6, 11, 8, 4, 6, 9, 0, 1, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 4, 6, 9, 6, 3, 9, 3, 1, 11, 3, 6, -1, -1, -1, -1 },
	{ 6, 8, 4, 6, 11, 8, 2, 10, 1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 2, 10, 3, 0, 11, 0, 6, 11, 0, 4, 6, -1, -1, -1, -1 },
	{ 4, 11, 8, 4, 6, 11, 0, 2, 9, 2, 10, 9, -1, -1, -1, -1 },
	{ 10, 9, 3, 10, 3, 2, 9, 4, 3, 11, 3, 6, 4, 6, 3, -1 },
	{ 8, 2, 3, 8, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 4, 2, 4, 6, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 9, 0, 2, 3, 4, 2, 4, 6, 4, 3, 8, -1, -1, -1, -1 },
	{ 1, 9, 4, 1, 4, 2, 2, 4, 6, -1, -1, -1, -1, -1, -1, -1 },
	{ 8, 1, 3, 8, 6, 1, 8, 4, 6, 6, 10, 1, -1, -1, -1, -1 },
	{ 10, 1, 0, 10, 0, 6, 6, 0, 4, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 6, 3, 4, 3, 8, 6, 10, 3, 0, 3, 9, 10, 9, 3, -1 },
	{ 10, 9, 4, 6, 10, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 9, 5, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 8, 3, 4, 9, 5, 11, 7, 6, -1, -1, -1, -1, -1, -1, -1 },
	{ 5, 0, 1, 5, 4, 0, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1 },
	{ 11, 7, 6, 8, 3, 4, 3, 5, 4, 3, 1, 5, -1, -1, -1, -1 },
	{ 9, 5, 4, 10, 1, 2, 7, 6, 11, -1, -1, -1, -1, -1, -1, -1 },
	{ 6, 11, 7, 1, 2, 10, 0, 8, 3, 4, 9, 5, -1, -1, -1, -1 },
	{ 7, 6, 11, 5, 4, 10, 4, 2, 10, 4, 0, 2, -1, -1, -1, -1 },
	{ 3, 4, 8, 3, 5, 4, 3, 2, 5, 10, 5, 2, 11, 7, 6, -1 },
	{ 7, 2, 3, 7, 6, 2, 5, 4, 9, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 5, 4, 0, 8, 6, 0, 6, 2, 6, 8, 7, -1, -1, -1, -1 },
	{ 3, 6, 2, 3, 7, 6, 1, 5, 0, 5, 4, 0, -1, -1, -1, -1 },
	{ 6, 2, 8, 6, 8, 7, 2, 1, 8, 4, 8, 5, 1, 5, 8, -1 },
	{ 9, 5, 4, 10, 1, 6, 1, 7, 6, 1, 3, 7, -1, -1, -1, -1 },
	{ 1, 6, 10, 1, 7, 6, 1, 0, 7, 8, 7, 0, 9, 5, 4, -1 },
	{ 4, 0, 10, 4, 10, 5, 0, 3, 10, 6, 10, 7, 3, 7, 10, -1 },
	{ 7, 6, 10, 7, 10, 8, 5, 4, 10, 4, 8, 10, -1, -1, -1, -1 },
	{ 6, 9, 5, 6, 11, 9, 11, 8, 9, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 6, 11, 0, 6, 3, 0, 5, 6, 0, 9, 5, -1, -1, -1, -1 },
	{ 0, 11, 8, 0, 5, 11, 0, 1, 5, 5, 6, 11, -1, -1, -1, -1 },
	{ 6, 11, 3, 6, 3, 5, 5, 3, 1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 2, 10, 9, 5, 11, 9, 11, 8, 11, 5, 6, -1, -1, -1, -1 },
	{ 0, 11, 3, 0, 6, 11, 0, 9, 6, 5, 6, 9, 1, 2, 10, -1 },
	{ 11, 8, 5, 11, 5, 6, 8, 0, 5, 10, 5, 2, 0, 2, 5, -1 },
	{ 6, 11, 3, 6, 3, 5, 2, 10, 3, 10, 5, 3, -1, -1, -1, -1 },
	{ 5, 8, 9, 5, 2, 8, 5, 6, 2, 3, 8, 2, -1, -1, -1, -1 },
	{ 9, 5, 6, 9, 6, 0, 0, 6, 2, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 5, 8, 1, 8, 0, 5, 6, 8, 3, 8, 2, 6, 2, 8, -1 },
	{ 1, 5, 6, 2, 1, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 3, 6, 1, 6, 10, 3, 8, 6, 5, 6, 9, 8, 9, 6, -1 },
	{ 10, 1, 0, 10, 0, 6, 9, 5, 0, 5, 6, 0, -1, -1, -1, -1 },
	{ 0, 3, 8, 5, 6, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 10, 5, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 11, 5, 10, 7, 5, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 11, 5, 10, 11, 7, 5, 8, 3, 0, -1, -1, -1, -1, -1, -1, -1 },
	{ 5, 11, 7, 5, 10, 11, 1, 9, 0, -1, -1, -1, -1, -1, -1, -1 },
	{ 10, 7, 5, 10, 11, 7, 9, 8, 1, 8, 3, 1, -1, -1, -1, -1 },
	{ 11, 1, 2, 11, 7, 1, 7, 5, 1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 8, 3, 1, 2, 7, 1, 7, 5, 7, 2, 11, -1, -1, -1, -1 },
	{ 9, 7, 5, 9, 2, 7, 9, 0, 2, 2, 11, 7, -1, -1, -1, -1 },
	{ 7, 5, 2, 7, 2, 11, 5, 9, 2, 3, 2, 8, 9, 8, 2, -1 },
	{ 2, 5, 10, 2, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1 },
	{ 8, 2, 0, 8, 5, 2, 8, 7, 5, 10, 2, 5, -1, -1, -1, -1 },
	{ 9, 0, 1, 5, 10, 3, 5, 3, 7, 3, 10, 2, -1, -1, -1, -1 },
	{ 9, 8, 2, 9, 2, 1, 8, 7, 2, 10, 2, 5, 7, 5, 2, -1 },
	{ 1, 3, 5, 3, 7, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 8, 7, 0, 7, 1, 1, 7, 5, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 0, 3, 9, 3, 5, 5, 3, 7, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 8, 7, 5, 9, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 5, 8, 4, 5, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1 },
	{ 5, 0, 4, 5, 11, 0, 5, 10, 11, 11, 3, 0, -1, -1, -1, -1 },
	{ 0, 1, 9, 8, 4, 10, 8, 10, 11, 10, 4, 5, -1, -1, -1, -1 },
	{ 10, 11, 4, 10, 4, 5, 11, 3, 4, 9, 4, 1, 3, 1, 4, -1 },
	{ 2, 5, 1, 2, 8, 5, 2, 11, 8, 4, 5, 8, -1, -1, -1, -1 },
	{ 0, 4, 11, 0, 11, 3, 4, 5, 11, 2, 11, 1, 5, 1, 11, -1 },
	{ 0, 2, 5, 0, 5, 9, 2, 11, 5, 4, 5, 8, 11, 8, 5, -1 },
	{ 9, 4, 5, 2, 11, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 5, 10, 3, 5, 2, 3, 4, 5, 3, 8, 4, -1, -1, -1, -1 },
	{ 5, 10, 2, 5, 2, 4, 4, 2, 0, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 10, 2, 3, 5, 10, 3, 8, 5, 4, 5, 8, 0, 1, 9, -1 },
	{ 5, 10, 2, 5, 2, 4, 1, 9, 2, 9, 4, 2, -1, -1, -1, -1 },
	{ 8, 4, 5, 8, 5, 3, 3, 5, 1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 4, 5, 1, 0, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 8, 4, 5, 8, 5, 3, 9, 0, 5, 0, 3, 5, -1, -1, -1, -1 },
	{ 9, 4, 5, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 11, 7, 4, 9, 11, 9, 10, 11, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 8, 3, 4, 9, 7, 9, 11, 7, 9, 10, 11, -1, -1, -1, -1 },
	{ 1, 10, 11, 1, 11, 4, 1, 4, 0, 7, 4, 11, -1, -1, -1, -1 },
	{ 3, 1, 4, 3, 4, 8, 1, 10, 4, 7, 4, 11, 10, 11, 4, -1 },
	{ 4, 11, 7, 9, 11, 4, 9, 2, 11, 9, 1, 2, -1, -1, -1, -1 },
	{ 9, 7, 4, 9, 11, 7, 9, 1, 11, 2, 11, 1, 0, 8, 3, -1 },
	{ 11, 7, 4, 11, 4, 2, 2, 4, 0, -1, -1, -1, -1, -1, -1, -1 },
	{ 11, 7, 4, 11, 4, 2, 8, 3, 4, 3, 2, 4, -1, -1, -1, -1 },
	{ 2, 9, 10, 2, 7, 9, 2, 3, 7, 7, 4, 9, -1, -1, -1, -1 },
	{ 9, 10, 7, 9, 7, 4, 10, 2, 7, 8, 7, 0, 2, 0, 7, -1 },
	{ 3, 7, 10, 3, 10, 2, 7, 4, 10, 1, 10, 0, 4, 0, 10, -1 },
	{ 1, 10, 2, 8, 7, 4, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 9, 1, 4, 1, 7, 7, 1, 3, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 9, 1, 4, 1, 7, 0, 8, 1, 8, 7, 1, -1, -1, -1, -1 },
	{ 4, 0, 3, 7, 4, 3, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 4, 8, 7, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 10, 8, 10, 11, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 0, 9, 3, 9, 11, 11, 9, 10, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 1, 10, 0, 10, 8, 8, 10, 11, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 1, 10, 11, 3, 10, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 2, 11, 1, 11, 9, 9, 11, 8, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 0, 9, 3, 9, 11, 1, 2, 9, 2, 11, 9, -1, -1, -1, -1 },
	{ 0, 2, 11, 8, 0, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 3, 2, 11, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 3, 8, 2, 8, 10, 10, 8, 9, -1, -1, -1, -1, -1, -1, -1 },
	{ 9, 10, 2, 0, 9, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 2, 3, 8, 2, 8, 10, 0, 1, 8, 1, 10, 8, -1, -1, -1, -1 },
	{ 1, 10, 2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 1, 3, 8, 9, 1, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 9, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ 0, 3, 8, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 },
	{ -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 }
};

inline ndIsoSurface::ndImplementation::ndImplementation()
	:ndClassAlloc()
{
}

inline ndIsoSurface::ndImplementation::~ndImplementation()
{
}


ndIsoSurface::ndImplementation& ndIsoSurface::GetImplementation() const
{
	static ndImplementation implementation;
	return implementation;
}

void ndIsoSurface::ndImplementation::CalculatedAabb(const ndArray<ndVector>& points, ndFloat32 gridSize)
{
	D_TRACKTIME();
	ndVector boxP0(ndFloat32(1.0e10f));
	ndVector boxP1(ndFloat32(-1.0e10f));
	for (ndInt32 i = 0; i < points.GetCount(); ++i)
	{
		boxP0 = boxP0.GetMin(points[i]);
		boxP1 = boxP1.GetMax(points[i]);
	}
	m_boxP0 = boxP0 & ndVector::m_triplexMask;
	m_boxP1 = boxP1 & ndVector::m_triplexMask;

	m_isoValue = ndFloat32(0.5f);
	m_gridSize = ndVector::m_triplexMask & ndVector(gridSize);
	m_invGridSize = ndVector::m_triplexMask & ndVector(ndFloat32(1.0f) / gridSize);

	m_boxP0 = m_boxP0 - m_gridSize;
	m_origin = m_boxP0;
	ndVector size((m_boxP1 - m_origin) * m_invGridSize + ndVector::m_half);

	ndVector cells(size.Floor().GetInt());
	m_xCellSize = cells.m_ix + 2;
	m_yCellSize = cells.m_iy + 2;
	m_zCellSize = cells.m_iz + 2;
}

ndVector ndIsoSurface::ndImplementation::VertexInterp(ndFloat32 isolevel, const ndVector& p0, const ndVector& p1) const
{
	//ndVector p;
	//ndFloat32 mu = (isolevel - p0.m_w) / (p1.m_w - p0.m_w);
	//p.m_x = p0.m_x + mu * (p1.m_x - p0.m_x);
	//p.m_y = p0.m_y + mu * (p1.m_y - p0.m_y);
	//p.m_z = p0.m_z + mu * (p1.m_z - p0.m_z);
	//p.m_w = ndFloat32(0.0f);
	//return p;

	dAssert(isolevel == ndFloat32(0.5f));
	dAssert(dAbs(p1.m_w - p0.m_w) == ndFloat32(1.0f));
	const ndVector p1p0(p1 - p0);
	return ndVector(p0 + p1p0 * ndVector::m_half);
}
ndVector ndIsoSurface::ndImplementation::InterpolateEdge(ndFloat32 fX1, ndFloat32 fY1, ndFloat32 fZ1, ndFloat32 fX2, ndFloat32 fY2, ndFloat32 fZ2, ndFloat32 tVal1, ndFloat32 tVal2)
{
	ndFloat32 mu = (m_isoValue - tVal1) / (tVal2 - tVal1);
	ndFloat32 x = fX1 + mu*(fX2 - fX1);
	ndFloat32 y = fY1 + mu*(fY2 - fY1);
	ndFloat32 z = fZ1 + mu*(fZ2 - fZ1);
	return ndVector(x, y, z, ndFloat32(0.0f));
}

ndVector ndIsoSurface::ndImplementation::CalculateIntersection(const ndIsoCell& cell, ndInt32 edgeCode)
{
	ndInt32 v1x = cell.m_x;
	ndInt32 v1y = cell.m_y;
	ndInt32 v1z = cell.m_z;
	ndInt32 v2x = cell.m_x;
	ndInt32 v2y = cell.m_y;
	ndInt32 v2z = cell.m_z;

	switch (edgeCode)
	{
		case 0:
			v2y += 1;
			break;
		case 1:
			v1y += 1;
			v2x += 1;
			v2y += 1;
			break;
		case 2:
			v1x += 1;
			v1y += 1;
			v2x += 1;
			break;
		case 3:
			v1x += 1;
			break;
		case 4:
			v1z += 1;
			v2y += 1;
			v2z += 1;
			break;
		case 5:
			v1y += 1;
			v1z += 1;
			v2x += 1;
			v2y += 1;
			v2z += 1;
			break;
		case 6:
			v1x += 1;
			v1y += 1;
			v1z += 1;
			v2x += 1;
			v2z += 1;
			break;
		case 7:
			v1x += 1;
			v1z += 1;
			v2z += 1;
			break;
		case 8:
			v2z += 1;
			break;
		case 9:
			v1y += 1;
			v2y += 1;
			v2z += 1;
			break;
		case 10:
			v1x += 1;
			v1y += 1;
			v2x += 1;
			v2y += 1;
			v2z += 1;
			break;
		case 11:
			v1x += 1;
			v2x += 1;
			v2z += 1;
			break;
	}

	ndFloat32 x1 = v1x * m_gridSize.m_x;
	ndFloat32 y1 = v1y * m_gridSize.m_y;
	ndFloat32 z1 = v1z * m_gridSize.m_z;
	ndFloat32 x2 = v2x * m_gridSize.m_x;
	ndFloat32 y2 = v2y * m_gridSize.m_y;
	ndFloat32 z2 = v2z * m_gridSize.m_z;

	ndFloat32 val1 = cell.m_isoValues[v1z - cell.m_z][v1y - cell.m_y][v1x - cell.m_x];
	ndFloat32 val2 = cell.m_isoValues[v2z - cell.m_z][v2y - cell.m_y][v2x - cell.m_x];
	return InterpolateEdge(x1, y1, z1, x2, y2, z2, val1, val2);
}

ndUnsigned64 ndIsoSurface::ndImplementation::GetVertexID(ndInt32 gridX, ndInt32 gridY, ndInt32 gridZ)
{
	//return 3 * (gridZ*(m_nCellsY + 1)*(m_nCellsX + 1) + gridY*(m_nCellsX + 1) + gridX);
	return 3 * (m_xCellSize * ndUnsigned64(gridZ * m_yCellSize + gridY) + gridX);
}

ndUnsigned64 ndIsoSurface::ndImplementation::GetEdgeID(const ndIsoCell& cell, ndInt32 edgeCode)
{
	const ndInt32 gridX = cell.m_x;
	const ndInt32 gridY = cell.m_y;
	const ndInt32 gridZ = cell.m_z;
	switch (edgeCode)
	{
		case 0:
			return GetVertexID(gridX, gridY, gridZ) + 1;
		case 1:
			return GetVertexID(gridX, gridY + 1, gridZ);
		case 2:
			return GetVertexID(gridX + 1, gridY, gridZ) + 1;
		case 3:
			return GetVertexID(gridX, gridY, gridZ);
		case 4:
			return GetVertexID(gridX, gridY, gridZ + 1) + 1;
		case 5:
			return GetVertexID(gridX, gridY + 1, gridZ + 1);
		case 6:
			return GetVertexID(gridX + 1, gridY, gridZ + 1) + 1;
		case 7:
			return GetVertexID(gridX, gridY, gridZ + 1);
		case 8:
			return GetVertexID(gridX, gridY, gridZ) + 2;
		case 9:
			return GetVertexID(gridX, gridY + 1, gridZ) + 2;
		case 10:
			return GetVertexID(gridX + 1, gridY + 1, gridZ) + 2;
		case 11:
			return GetVertexID(gridX + 1, gridY, gridZ) + 2;
		default:
			// Invalid edge no.
			return ndUnsigned64(-1);
	}
}

void ndIsoSurface::ndImplementation::ProcessIndexListCell(const ndIsoCell& cell)
{
	dAssert(cell.m_x < (m_xCellSize));
	dAssert(cell.m_y < (m_yCellSize));
	dAssert(cell.m_z < (m_zCellSize));

	ndInt32 tableIndex = 0;
	if (cell.m_isoValues[0][0][0] > m_isoValue)
	{
		tableIndex |= 1;
	}
	if (cell.m_isoValues[0][1][0] > m_isoValue)
	{
		tableIndex |= 2;
	}
	if (cell.m_isoValues[0][1][1] > m_isoValue)
	{
		tableIndex |= 4;
	}
	if (cell.m_isoValues[0][0][1] > m_isoValue)
	{
		tableIndex |= 8;
	}
	if (cell.m_isoValues[1][0][0] > m_isoValue)
	{
		tableIndex |= 16;
	}
	if (cell.m_isoValues[1][1][0] > m_isoValue)
	{
		tableIndex |= 32;
	}
	if (cell.m_isoValues[1][1][1] > m_isoValue)
	{
		tableIndex |= 64;
	}
	if (cell.m_isoValues[1][0][1] > m_isoValue)
	{
		tableIndex |= 128;
	}

	// Now create a triangulation of the iso surface in this cell.
	ndInt32 edgeBits = m_edgeTable[tableIndex];
	if (edgeBits != 0)
	{
		if (edgeBits & 8)
		{
			ndVector pt(CalculateIntersection(cell, 3));
			ndUnsigned64 id = GetEdgeID(cell, 3);
			dAssert(!m_vertexMap.Find(id));
			m_vertexMap.Insert(pt, id);
		}
		if (edgeBits & 1)
		{
			ndVector pt(CalculateIntersection(cell, 0));
			ndUnsigned64 id = GetEdgeID(cell, 0);
			dAssert(!m_vertexMap.Find(id));
			m_vertexMap.Insert(pt, id);
		}
		if (edgeBits & 256)
		{
			ndVector pt(CalculateIntersection(cell, 8));
			ndUnsigned64 id = GetEdgeID(cell, 8);
			dAssert(!m_vertexMap.Find(id));
			m_vertexMap.Insert(pt, id);
		}

		for (ndInt32 i = 0; m_triangleTable[tableIndex][i] != -1; i += 3)
		{
			ndIsoTriangle triangle;
			ndUnsigned64 pointID0 = GetEdgeID(cell, m_triangleTable[tableIndex][i + 0]);
			ndUnsigned64 pointID1 = GetEdgeID(cell, m_triangleTable[tableIndex][i + 1]);
			ndUnsigned64 pointID2 = GetEdgeID(cell, m_triangleTable[tableIndex][i + 2]);
			triangle.m_pointId[0] = pointID0;
			triangle.m_pointId[1] = pointID1;
			triangle.m_pointId[2] = pointID2;
			m_triangleMap.Insert(0, triangle);
			#ifdef _DEBUG
				ndIsoTriangle triangle1;
				triangle1.m_pointId[0] = triangle.m_pointId[1];
				triangle1.m_pointId[1] = triangle.m_pointId[2];
				triangle1.m_pointId[2] = triangle.m_pointId[0];
				dAssert(!m_triangleMap.Find(triangle1));

				triangle1.m_pointId[0] = triangle.m_pointId[2];
				triangle1.m_pointId[1] = triangle.m_pointId[0];
				triangle1.m_pointId[2] = triangle.m_pointId[1];
				dAssert(!m_triangleMap.Find(triangle1));
			#endif
		}
	}
}

void ndIsoSurface::ndImplementation::ProcessTriangleListCell(const ndIsoCell_1& cell)
{
	ndInt32 tableIndex = 0;
	for (ndInt32 i = 0; i < 8; i++)
	{
		tableIndex |= (cell.m_isoValues[i].m_w > m_isoValue) << i;
	}
	
	// Now create a triangulation of the iso surface in this cell.
	ndInt32 edgeBits = m_edgeTable[tableIndex];
	if (edgeBits == 0)
	{
		return;
	}
	
	ndVector vertlist[12];
	// Find the vertices where the surface intersects the cube
	if (edgeBits & 1)
	{
		vertlist[0] = VertexInterp(m_isoValue, cell.m_isoValues[0], cell.m_isoValues[1]);
	}
	if (edgeBits & 2)
	{
		vertlist[1] = VertexInterp(m_isoValue, cell.m_isoValues[1], cell.m_isoValues[2]);
	}
	if (edgeBits & 4)
	{
		vertlist[2] = VertexInterp(m_isoValue, cell.m_isoValues[2], cell.m_isoValues[3]);
	}
	if (edgeBits & 8)
	{
		vertlist[3] = VertexInterp(m_isoValue, cell.m_isoValues[3], cell.m_isoValues[0]);
	}
	if (edgeBits & 16)
	{
		vertlist[4] = VertexInterp(m_isoValue, cell.m_isoValues[4], cell.m_isoValues[5]);
	}
	if (edgeBits & 32)
	{
		vertlist[5] = VertexInterp(m_isoValue, cell.m_isoValues[6], cell.m_isoValues[5]);
	}
	if (edgeBits & 64)
	{
		vertlist[6] = VertexInterp(m_isoValue, cell.m_isoValues[7], cell.m_isoValues[6]);
	}
	if (edgeBits & 128)
	{
		vertlist[7] = VertexInterp(m_isoValue, cell.m_isoValues[7], cell.m_isoValues[4]);
	}
	if (edgeBits & 256)
	{
		vertlist[8] = VertexInterp(m_isoValue, cell.m_isoValues[0], cell.m_isoValues[4]);
	}
	if (edgeBits & 512)
	{
		vertlist[9] = VertexInterp(m_isoValue, cell.m_isoValues[1], cell.m_isoValues[5]);
	}
	if (edgeBits & 1024)
	{
		vertlist[10] = VertexInterp(m_isoValue, cell.m_isoValues[2], cell.m_isoValues[6]);
	}
	if (edgeBits & 2048)
	{
		vertlist[11] = VertexInterp(m_isoValue, cell.m_isoValues[3], cell.m_isoValues[7]);
	}
	
	for (ndInt32 i = 0; m_triangleTable[tableIndex][i] != -1; i += 3)
	{
		ndIsoTriangle_1 triangle;
		ndInt32 j0 = m_triangleTable[tableIndex][i + 0];
		ndInt32 j1 = m_triangleTable[tableIndex][i + 1];
		ndInt32 j2 = m_triangleTable[tableIndex][i + 2];
		triangle.m_p[0] = vertlist[j0];
		triangle.m_p[1] = vertlist[j1];
		triangle.m_p[2] = vertlist[j2];
		m_triangles.PushBack(triangle);
	}
}

void ndIsoSurface::ndImplementation::RemapVertexArray(ndIsoSurface* const me)
{
	D_TRACKTIME();
	ndArray<ndVector>& points = me->m_points;
	points.SetCount(m_vertexMap.GetCount());
	ndIsoVertexMap::Iterator iter(m_vertexMap);

	ndInt32 nextID = 0;
	for (iter.Begin(); iter; iter++)
	{
		ndVector& point = iter.GetNode()->GetInfo();
		points[nextID] = point + m_origin;
		point.m_w = ndFloat32(nextID);
		nextID++;
	}
}

void ndIsoSurface::ndImplementation::RemapIndexArray(ndIsoSurface* const me)
{
	D_TRACKTIME();
	ndArray<ndTriangle>& triangles = me->m_triangles;
	triangles.SetCount(m_triangleMap.GetCount());
	ndIsoTriangleMap::Iterator iter(m_triangleMap);

	ndInt32 index = 0;
	for (iter.Begin(); iter; iter++)
	{
		const ndIsoTriangle& triangle = iter.GetKey();
		for (ndInt32 i = 0; i < 3; ++i)
		{
			ndIsoVertexMap::ndNode* const node = m_vertexMap.Find(triangle.m_pointId[i]);
			dAssert(node);
			ndInt32 id = ndInt32(node->GetInfo().m_w);
			triangles[index].m_index[i] = id;
		}
		index++;
	}
}

void ndIsoSurface::ndImplementation::CalculateNormals(ndIsoSurface* const me)
{
	D_TRACKTIME();
	ndArray<ndVector>& normals = me->m_normals;
	const ndArray<ndVector>& points = me->m_points;
	const ndArray<ndTriangle>& triangles = me->m_triangles;
	normals.SetCount(points.GetCount());

	// Set all normals to 0.
	if (normals.GetCount())
	{
		memset(&normals[0], 0, normals.GetCount() * sizeof(ndVector));

		for (ndInt32 i = 0; i < triangles.GetCount(); ++i)
		{
			ndInt32 id0 = triangles[i].m_index[0];
			ndInt32 id1 = triangles[i].m_index[1];
			ndInt32 id2 = triangles[i].m_index[2];
			ndVector vec1(points[id1] - points[id0]);
			ndVector vec2(points[id2] - points[id0]);
			ndVector normal = vec1.CrossProduct(vec2);
			normals[id0] += normal;
			normals[id1] += normal;
			normals[id2] += normal;
		}

		// Normalize normals.
		for (ndInt32 i = 0; i < normals.GetCount(); ++i)
		{
			//m_normals[i] = m_normals[i].Normalize();
			normals[i] = normals[i] * normals[i].InvMagSqrt();
		}
	}
}

void ndIsoSurface::ndImplementation::SortCellBuckects()
{
	D_TRACKTIME();
	class ndKey_xlow
	{
		public:
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return cell.m_xLow;
		}
	};

	class ndKey_ylow
	{
		public:
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return cell.m_yLow;
		}
	};

	class ndKey_zlow
	{
		public:
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return cell.m_zLow;
		}
	};

	class ndKey_xhigh
	{
		public:
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return cell.m_xHigh;
		}
	};

	class ndKey_yhigh
	{
		public:
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return cell.m_yHigh;
		}
	};

	class ndKey_zhigh
	{
		public:
		ndInt32 GetKey(const ndGridHash& cell) const
		{
			return cell.m_zHigh;
		}
	};

	ndCountingSort<ndGridHash, ndKey_xlow, D_RADIX_DIGIT_SIZE>(m_hashGridMap, m_hashGridMapScratchBuffer);
	if (m_upperDigitsIsValid.m_x)
	{
		dAssert(0);
		ndCountingSort<ndGridHash, ndKey_xhigh, D_RADIX_DIGIT_SIZE>(m_hashGridMap, m_hashGridMapScratchBuffer);
	}

	ndCountingSort<ndGridHash, ndKey_ylow, D_RADIX_DIGIT_SIZE>(m_hashGridMap, m_hashGridMapScratchBuffer);
	if (m_upperDigitsIsValid.m_y)
	{
		dAssert(0);
		ndCountingSort<ndGridHash, ndKey_yhigh, D_RADIX_DIGIT_SIZE>(m_hashGridMap, m_hashGridMapScratchBuffer);
	}

	ndCountingSort<ndGridHash, ndKey_zlow, D_RADIX_DIGIT_SIZE>(m_hashGridMap, m_hashGridMapScratchBuffer);
	if (m_upperDigitsIsValid.m_z)
	{
		dAssert(0);
		ndCountingSort<ndGridHash, ndKey_zhigh, D_RADIX_DIGIT_SIZE>(m_hashGridMap, m_hashGridMapScratchBuffer);
	}
}

void ndIsoSurface::ndImplementation::RemoveDuplicates()
{
	D_TRACKTIME();
	ndInt32 gridCount = 0;
	ndUpperDigit upperDigits;
	for (ndInt32 i = 1; i < m_hashGridMap.GetCount(); ++i)
	{
		const ndGridHash cell(m_hashGridMap[i]);
		gridCount += (m_hashGridMap[i].m_gridFullHash != m_hashGridMap[i - 1].m_gridFullHash);
		m_hashGridMap[gridCount] = cell;
	}
	gridCount++;
	m_hashGridMap.SetCount(gridCount);
}

void ndIsoSurface::ndImplementation::GenerateIsoSurface()
{
	D_TRACKTIME();
	ndInt32 end = 0;
	const ndInt32 gridCount = m_hashGridMap.GetCount();
	m_hashGridMap.PushBack(ndGridHash(0xffff, 0xffff, 0xffff));

	for (ndInt32 i = 0; i < gridCount; i = end)
	{
		end = i + 1;
		ndInt32 start = i;
		const ndGridHash startGrid(m_hashGridMap[start], 0);
		while (startGrid.m_gridCellHash == ndGridHash(m_hashGridMap[end], 0).m_gridCellHash)
		{
			end++;
		}
		ndInt32 count = end - start;
		if (count < 8)
		{
			ndIsoCell cell;
			cell.m_x = startGrid.m_x;
			cell.m_y = startGrid.m_y;
			cell.m_z = startGrid.m_z;
			ndFloat32* const isoValue = &cell.m_isoValues[0][0][0];
			for (ndInt32 j = 0; j < 8; j++)
			{
				isoValue[j] = ndFloat32(0.0f);
			}
			for (ndInt32 j = 0; j < count; j++)
			{
				ndInt32 index = m_hashGridMap[start + j].m_cellType;
				isoValue[index] = ndFloat32 (1.0f);
			}
			cell.m_x++;
			cell.m_y++;
			cell.m_z++;
			ProcessIndexListCell(cell);
		}
	}
}
	
void ndIsoSurface::ndImplementation::GenerateIsoSurface_1()
{
	D_TRACKTIME();
	ndInt32 end = 0;
	const ndInt32 gridCount = m_hashGridMap.GetCount();
	m_hashGridMap.PushBack(ndGridHash(0xffff, 0xffff, 0xffff));

	static ndVector gridCorners[] =
	{
		ndVector(ndFloat32( 0.0f), ndFloat32(-1.0f), ndFloat32(-1.0f), ndFloat32(0.0f)),
		ndVector(ndFloat32( 0.0f), ndFloat32(-1.0f), ndFloat32( 0.0f), ndFloat32(0.0f)),
		ndVector(ndFloat32(-1.0f), ndFloat32(-1.0f), ndFloat32( 0.0f), ndFloat32(0.0f)),
		ndVector(ndFloat32(-1.0f), ndFloat32(-1.0f), ndFloat32(-1.0f), ndFloat32(0.0f)),
		ndVector(ndFloat32( 0.0f), ndFloat32( 0.0f), ndFloat32(-1.0f), ndFloat32(0.0f)),
		ndVector(ndFloat32( 0.0f), ndFloat32( 0.0f), ndFloat32( 0.0f), ndFloat32(0.0f)),
		ndVector(ndFloat32(-1.0f), ndFloat32( 0.0f), ndFloat32( 0.0f), ndFloat32(0.0f)),
		ndVector(ndFloat32(-1.0f), ndFloat32( 0.0f), ndFloat32(-1.0f), ndFloat32(0.0f))
	};

	for (ndInt32 i = 0; i < gridCount; i = end)
	{
		end = i + 1;
		ndInt32 start = i;
		const ndGridHash startGrid(m_hashGridMap[start], 0);
		while (startGrid.m_gridCellHash == ndGridHash(m_hashGridMap[end], 0).m_gridCellHash)
		{
			end++;
		}
		ndInt32 count = end - start;
		if (count < 8)
		{
			ndIsoCell_1 cell;
			ndVector* const isoValue = &cell.m_isoValues[0];
			ndVector origin(ndFloat32(startGrid.m_x + 1), ndFloat32(startGrid.m_y + 1), ndFloat32(startGrid.m_z + 1), ndFloat32(0.0f));
			for (ndInt32 j = 0; j < 8; j++)
			{
				isoValue[j] = origin + gridCorners[j];
			}
			for (ndInt32 j = 0; j < count; j++)
			{
				ndInt32 index = m_hashGridMap[start + j].m_cellType;
				isoValue[index].m_w = ndFloat32(1.0f);
			}
			ProcessTriangleListCell(cell);
		}
	}
}

void ndIsoSurface::ndImplementation::GenerateIndexList(ndIsoSurface* const me)
{
	D_TRACKTIME();
	
	ndArray<ndVector>& points = me->m_points;
	ndArray<ndTriangle>& indexList = me->m_triangles;
	const ndArray<ndIsoTriangle_1>& triangleList = m_triangles;
	indexList.SetCount(triangleList.GetCount());
	points.SetCount(triangleList.GetCount() * 3);
	for (ndInt32 i = 0; i < triangleList.GetCount(); ++i)
	{
		const ndIsoTriangle_1& triag = triangleList[i];
		indexList[i].m_index[0] = i * 3 + 0;
		indexList[i].m_index[1] = i * 3 + 1;
		indexList[i].m_index[2] = i * 3 + 2;

		points[i * 3 + 0] = triag.m_p[0] * m_gridSize + m_origin;
		points[i * 3 + 1] = triag.m_p[1] * m_gridSize + m_origin;
		points[i * 3 + 2] = triag.m_p[2] * m_gridSize + m_origin;
	}
}
	
void ndIsoSurface::ndImplementation::CreateGrids(const ndArray<ndVector>& points)
{
	D_TRACKTIME();
	const ndVector origin(m_boxP0);
	const ndVector invGridSize(m_invGridSize);

	ndUpperDigit upperDigits;
	const ndGridHashSteps steps;
	m_hashGridMap.SetCount(points.GetCount() * 8);
	for (ndInt32 i = 0; i < points.GetCount(); ++i)
	{
		const ndVector r(points[i] - origin);
		const ndVector p(r * invGridSize);
		const ndGridHash hashKey(p);
	
		upperDigits.m_x = dMax(upperDigits.m_x, ndInt32(hashKey.m_xHigh));
		upperDigits.m_y = dMax(upperDigits.m_y, ndInt32(hashKey.m_yHigh));
		upperDigits.m_z = dMax(upperDigits.m_z, ndInt32(hashKey.m_zHigh));
		
		for (ndInt32 j = 0; j < 8; ++j)
		{
			ndGridHash cell(hashKey);
			cell.m_x += steps.m_steps[j].m_x;
			cell.m_y += steps.m_steps[j].m_y;
			cell.m_z += steps.m_steps[j].m_z;
			cell.m_cellType = steps.m_cellType[j];
			m_hashGridMap[i * 8 + j] = cell;
		}
	}
	m_upperDigitsIsValid = upperDigits;
}

void ndIsoSurface::ndImplementation::CreateGrids_1(const ndArray<ndVector>& points)
{
	D_TRACKTIME();
	const ndVector origin(m_boxP0);
	const ndVector invGridSize(m_invGridSize);

	ndUpperDigit upperDigits;
	const ndGridHashSteps_1 steps;
	m_hashGridMap.SetCount(points.GetCount() * 8);
	for (ndInt32 i = 0; i < points.GetCount(); ++i)
	{
		const ndVector r(points[i] - origin);
		const ndVector p(r * invGridSize);
		const ndGridHash hashKey(p);

		upperDigits.m_x = dMax(upperDigits.m_x, ndInt32(hashKey.m_xHigh));
		upperDigits.m_y = dMax(upperDigits.m_y, ndInt32(hashKey.m_yHigh));
		upperDigits.m_z = dMax(upperDigits.m_z, ndInt32(hashKey.m_zHigh));
		
		for (ndInt32 j = 0; j < 8; ++j)
		{
			ndGridHash cell(hashKey);
			cell.m_x += steps.m_steps[j].m_x;
			cell.m_y += steps.m_steps[j].m_y;
			cell.m_z += steps.m_steps[j].m_z;
			cell.m_cellType = steps.m_cellType[j];
			m_hashGridMap[i * 8 + j] = cell;
		}
	}
	m_upperDigitsIsValid = upperDigits;
}


void ndIsoSurface::ndImplementation::ClearBuffers()
{
	D_TRACKTIME();
	m_vertexMap.RemoveAll();
	m_triangleMap.RemoveAll();
}

void ndIsoSurface::ndImplementation::ClearBuffers_1()
{
	D_TRACKTIME();
	m_triangles.SetCount(0);
}

void ndIsoSurface::ndImplementation::BuildIndexListMesh(ndIsoSurface* const me, const ndArray<ndVector>& points, ndFloat32 gridSize)
{
	D_TRACKTIME();
	CalculatedAabb(points, gridSize);
	CreateGrids(points);
	SortCellBuckects();
	RemoveDuplicates();
	GenerateIsoSurface();
	RemapVertexArray(me);
	RemapIndexArray(me);
	CalculateNormals(me);
	ClearBuffers();
}

void ndIsoSurface::ndImplementation::BuildTriangleListMesh(ndIsoSurface* const me, const ndArray<ndVector>& points, ndFloat32 gridSize)
{
	D_TRACKTIME();
	CalculatedAabb(points, gridSize);
	CreateGrids_1(points);
	SortCellBuckects();
	RemoveDuplicates();
	GenerateIsoSurface_1();
	GenerateIndexList(me);
	CalculateNormals(me);
	ClearBuffers_1();
}

void ndIsoSurface::GenerateMesh(const ndArray<ndVector>& pointCloud, ndFloat32 gridSize)
{
	ndImplementation& implementation = GetImplementation();
	implementation.BuildTriangleListMesh(this, pointCloud, gridSize);
	//implementation.BuildIndexListMesh(this, pointCloud, gridSize);
}
