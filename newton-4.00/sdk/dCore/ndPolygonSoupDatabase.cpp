/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

/****************************************************************************
*
*  Visual C++ 6.0 created by: Julio Jerez
*
****************************************************************************/

#include "ndCoreStdafx.h"
#include "ndMemory.h"
#include "ndPolygonSoupDatabase.h"

ndPolygonSoupDatabase::ndPolygonSoupDatabase(const char* const)
{
	m_vertexCount = 0;
	m_strideInBytes = 0;
	m_localVertex = nullptr;
}

ndPolygonSoupDatabase::~ndPolygonSoupDatabase ()
{
	if (m_localVertex) 
	{
		ndMemory::Free (m_localVertex);
	}
}

void ndPolygonSoupDatabase::SetTagId(const ndInt32* const facePtr, ndInt32 indexCount, ndUnsigned32 newID) const
{
	ndUnsigned32* const face = (ndUnsigned32*) facePtr;
	face[indexCount] = newID;
}

ndInt32 ndPolygonSoupDatabase::GetVertexCount()	const
{
	return m_vertexCount;
}

ndFloat32* ndPolygonSoupDatabase::GetLocalVertexPool() const
{
	return m_localVertex;
}

ndInt32 ndPolygonSoupDatabase::GetStrideInBytes() const
{
	return m_strideInBytes;
}

ndFloat32 ndPolygonSoupDatabase::GetRadius() const
{
	return ndFloat32(0.0f);
}

ndUnsigned32 ndPolygonSoupDatabase::GetTagId(const ndInt32* const face, ndInt32 indexCount) const
{
	return ndUnsigned32(face[indexCount]);
}

