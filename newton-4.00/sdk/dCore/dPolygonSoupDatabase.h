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

#ifndef __D_POLYGONSOUP_DATABASE_H_
#define __D_POLYGONSOUP_DATABASE_H_

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dClassAlloc.h"

class dPolygonSoupDatabase: public dClassAlloc
{
	public:
	dFloat32 GetRadius() const;
	dInt32 GetVertexCount() const;
	dInt32 GetStrideInBytes() const;
	dFloat32* GetLocalVertexPool() const;

	dUnsigned32 GetTagId(const dInt32* const face, dInt32 indexCount) const;
	void SetTagId(const dInt32* const face, dInt32 indexCount, dUnsigned32 newID) const;

	//virtual void Serialize (dgSerialize callback, void* const userData) const = 0;
	//virtual void Deserialize (dgDeserialize callback, void* const userData, dInt32 revisionNumber) = 0;
		
	protected:
	dPolygonSoupDatabase(const char* const name = nullptr);
	virtual ~dPolygonSoupDatabase ();

	dInt32 m_vertexCount;
	dInt32 m_strideInBytes;
	dFloat32* m_localVertex;
};

#endif

