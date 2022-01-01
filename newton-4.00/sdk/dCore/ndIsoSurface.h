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
	class ndTriangle
	{
		public:
		ndInt32 m_index[3];
	};

	class ndImplementation;

	ndIsoSurface();
	~ndIsoSurface();
	const ndArray<ndVector>& GetPoints() const;
	const ndArray<ndVector>& GetNormals() const;
	const ndArray<ndTriangle>& GetTriangles() const;

	D_CORE_API void GenerateMesh(const ndArray<ndVector>& pointCloud, ndFloat32 gridSize);
	D_CORE_API void GenerateMeshNaive(const ndArray<ndVector>& pointCloud, ndFloat32 gridSize);

	private:
	ndImplementation& GetImplementation() const;

	ndArray<ndVector> m_points;
	ndArray<ndVector> m_normals;
	ndArray<ndTriangle> m_triangles;
	friend class ndImplementation;
};

inline ndIsoSurface::ndIsoSurface()
	:m_points(1024)
	,m_normals(1024)
	,m_triangles(1024)
{
}

inline ndIsoSurface::~ndIsoSurface()
{
}

inline const ndArray<ndVector>& ndIsoSurface::GetPoints() const
{
	return m_points;
}

inline const ndArray<ndVector>& ndIsoSurface::GetNormals() const
{
	return m_normals;
}

inline const ndArray<ndIsoSurface::ndTriangle>& ndIsoSurface::GetTriangles() const
{
	return m_triangles;
}

#endif

