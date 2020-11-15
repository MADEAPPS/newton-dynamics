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

// adapted from code by written by Paul Bourke may 1994
//http://paulbourke.net/geometry/polygonise/

#ifndef __D_ISO_SURFACE_H__
#define __D_ISO_SURFACE_H__

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dArray.h"
#include "dTree.h"

class dIsoSurfacePointId 
{
	public:
	dFloat32 m_x, m_y, m_z;
	dInt32 m_newId;
};

class dIsoSurfaceTriangle 
{
	public:
	dInt32 m_pointId[3];
};

class dIsoSurfaceOld 
{
	public:
	D_CORE_API dIsoSurfaceOld();
	D_CORE_API ~dIsoSurfaceOld();
	
	// Generates the iso surface from the scalar field contained in the
	// buffer ptScalarField[].
	D_CORE_API void GenerateSurface(const dFloat32* ptScalarField, dFloat32 tIsoLevel, dInt32 nCellsX, dInt32 nCellsY,  dInt32 nCellsZ, dFloat32 fCellLengthX, dFloat32 fCellLengthY, dFloat32 fCellLengthZ);

	// Returns true if a valid surface has been generated.
	bool IsSurfaceValid();

	// Deletes the iso surface.
	void DeleteSurface();

	// Returns the length, width, and height of the volume in which the
	// iso surface in enclosed in.  Returns -1 if the surface is not
	// valid.
	int GetVolumeLengths(dFloat32& fVolLengthX, dFloat32& fVolLengthY, dFloat32& fVolLengthZ);

	protected:
	// The number of vertices which make up the iso surface.
	dInt32 m_nVertices;

	// The vertices which make up the iso surface.
	dVector* m_ppt3dVertices;

	// The number of triangles which make up the iso surface.
	dInt32 m_nTriangles;

	// The indices of the vertices which make up the triangles.
	dInt32* m_piTriangleIndices;

	// The number of normals.
	dInt32 m_nNormals;

	// The normals.
	dVector* m_pvec3dNormals;

	// List of POINT3Ds which form the iso surface.
	dTree<dIsoSurfacePointId, dInt32> m_i2pt3idVertices;

	// List of TRIANGLES which form the triangulation of the iso surface.
	dArray<dIsoSurfaceTriangle> m_trivecTriangles;

	// Returns the edge ID.
	dInt32 GetEdgeID(dInt32 nX, dInt32 nY, dInt32 nZ, dInt32 nEdgeNo);

	// Returns the vertex ID.
	dInt32 GetVertexID(dInt32 nX, dInt32 nY, dInt32 nZ);

	// Calculates the intersection point of the iso surface with an
	// edge.
	dIsoSurfacePointId CalculateIntersection(dInt32 nX, dInt32 nY, dInt32 nZ, dInt32 nEdgeNo);

	// Interpolates between two grid points to produce the point at which
	// the iso surface intersects an edge.
	dIsoSurfacePointId Interpolate(dFloat32 fX1, dFloat32 fY1, dFloat32 fZ1, dFloat32 fX2, dFloat32 fY2, dFloat32 fZ2, dFloat32 tVal1, dFloat32 tVal2);
 
	// Renames vertices and triangles so that they can be accessed more
	// efficiently.
	void RenameVerticesAndTriangles();

	// Calculates the normals.
	void CalculateNormals();

	// No. of cells in m_x, m_y, and m_z directions.
	dInt32 m_nCellsX, m_nCellsY, m_nCellsZ;

	// Cell length in m_x, m_y, and m_z directions.
	dFloat32 m_fCellLengthX, m_fCellLengthY, m_fCellLengthZ;

	// The buffer holding the scalar field.
	const dFloat32* m_ptScalarField;

	// The iso surface value.
	dFloat32 m_tIsoLevel;

	// Indicates whether a valid surface is present.
	bool m_bValidSurface;

	// Lookup tables used in the construction of the iso surface.
	static const dInt32 m_edgeTable[256];
	static const dInt32 m_triTable[256][16];
};


class dIsoSurface
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

	class dIsoSurfaceTriangle
	{
		public:
		dUnsigned64 m_pointId[3];
	};


	D_CORE_API dIsoSurface();
	D_CORE_API ~dIsoSurface();

	D_CORE_API void Begin(dFloat32 isovalue, dFloat32 gridSize, dInt32 sizex, dInt32 sizey, dInt32 sizez);

	D_CORE_API void ProcessCell(const dIsoCell& cell);

	D_CORE_API void End();

	private:
	dUnsigned64 GetVertexID(dInt32 x, dInt32 y, dInt32 z);
	dUnsigned64 GetEdgeID(const dIsoCell& cell, dInt32 edgeCode);
	dVector CalculateIntersection(const dIsoCell& cell, dInt32 edgeCode);
	dVector Interpolate(dFloat32 fX1, dFloat32 fY1, dFloat32 fZ1, dFloat32 fX2, dFloat32 fY2, dFloat32 fZ2, dFloat32 tVal1, dFloat32 tVal2);

	void RemapIndexList();
	void CalculateNormals();

	dArray<dVector> m_points;
	dArray<dVector> m_normals;
	dTree<dVector, dUnsigned64> m_vertexMap;
	dArray<dIsoSurfaceTriangle> m_trivecTriangles;
	dInt32 m_xCellSize;
	dInt32 m_yCellSize;
	dInt32 m_zCellSize;
	dFloat32 m_gridSize;
	dFloat32 m_isoValue;

	static const dInt32 m_edgeTable[];
	static const dInt32 m_triangleTable[][16];
};


#endif

