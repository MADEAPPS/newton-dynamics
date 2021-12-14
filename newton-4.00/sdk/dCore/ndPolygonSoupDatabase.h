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

#ifndef __ND_POLYGONSOUP_DATABASE_H_
#define __ND_POLYGONSOUP_DATABASE_H_

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndClassAlloc.h"

class ndPolygonSoupDatabase: public ndClassAlloc
{
	public:
	ndFloat32 GetRadius() const;
	ndInt32 GetVertexCount() const;
	ndInt32 GetStrideInBytes() const;
	ndFloat32* GetLocalVertexPool() const;

	ndUnsigned32 GetTagId(const ndInt32* const face, ndInt32 indexCount) const;
	void SetTagId(const ndInt32* const face, ndInt32 indexCount, ndUnsigned32 newID) const;
		
	protected:
	ndPolygonSoupDatabase(const char* const name = nullptr);
	virtual ~ndPolygonSoupDatabase ();

	ndInt32 m_vertexCount;
	ndInt32 m_strideInBytes;
	ndFloat32* m_localVertex;
};

inline ndInt32 ndPolygonSoupDatabase::GetVertexCount()	const
{
	return m_vertexCount;
}

inline ndFloat32* ndPolygonSoupDatabase::GetLocalVertexPool() const
{
	return m_localVertex;
}

inline ndInt32 ndPolygonSoupDatabase::GetStrideInBytes() const
{
	return m_strideInBytes;
}

inline ndFloat32 ndPolygonSoupDatabase::GetRadius() const
{
	return ndFloat32 (0.0f);
}

inline ndUnsigned32 ndPolygonSoupDatabase::GetTagId(const ndInt32* const face, ndInt32 indexCount) const
{
	return ndUnsigned32(face[indexCount]);
}

#endif

