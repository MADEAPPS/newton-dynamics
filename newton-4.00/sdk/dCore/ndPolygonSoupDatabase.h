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

#ifndef __ND_POLYGONSOUP_DATABASE_H_
#define __ND_POLYGONSOUP_DATABASE_H_

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndMemory.h"
#include "ndVector.h"
#include "ndClassAlloc.h"

D_MSV_NEWTON_CLASS_ALIGN_32
class ndPolygonSoupDatabase: public ndClassAlloc
{
	public:
	D_CORE_API ndFloat32 GetRadius() const;
	D_CORE_API ndInt32 GetVertexCount() const;
	D_CORE_API ndInt32 GetStrideInBytes() const;
	D_CORE_API ndFloat32* GetLocalVertexPool() const;

	D_CORE_API ndUnsigned32 GetTagId(const ndInt32* const face, ndInt32 indexCount) const;
	D_CORE_API void SetTagId(const ndInt32* const face, ndInt32 indexCount, ndUnsigned32 newID) const;
		
	protected:
	D_CORE_API ndPolygonSoupDatabase(const char* const name = nullptr);
	D_CORE_API virtual ~ndPolygonSoupDatabase ();

	ndFloat32* m_localVertex;
	ndInt32 m_vertexCount;
	ndInt32 m_strideInBytes;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif

