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

#ifndef __ND_FAST_AABB_H__
#define __ND_FAST_AABB_H__

#include "ndCoreStdafx.h"
#include "ndDebug.h"
#include "ndVector.h"
#include "ndMatrix.h"

D_MSV_NEWTON_CLASS_ALIGN_32 
class ndFastAabb : public ndMatrix
{
	public:
	D_CORE_API ndFastAabb();
	D_CORE_API ndFastAabb(const ndVector& p0, const ndVector& p1);
	D_CORE_API ndFastAabb(const ndMatrix& matrix, const ndVector& size);

	D_CORE_API const ndVector& GetOrigin() const;
	D_CORE_API const ndVector& GetTarget() const;

	D_CORE_API void SetSeparatingDistance(const ndFloat32 distance);
	D_CORE_API void SetTransposeAbsMatrix(const ndMatrix& matrix);

	D_CORE_API ndFloat32 PolygonBoxDistance(const ndVector& faceNormal, ndInt32 indexCount, const ndInt32* const indexArray, ndInt32 stride, const ndFloat32* const vertexArray) const;
	D_CORE_API ndFloat32 PolygonBoxRayDistance(const ndVector& faceNormal, ndInt32 indexCount, const ndInt32* const indexArray, ndInt32 stride, const ndFloat32* const vertexArray, const ndFastRay& ray) const;

	private:
	ndMatrix MakeFaceMatrix(const ndVector& faceNormal, ndInt32, const ndInt32* const indexArray, ndInt32 stride, const ndFloat32* const vertexArray) const;
	void MakeBox1(ndInt32 indexCount, const ndInt32* const indexArray, ndInt32 stride, const ndFloat32* const vertexArray, ndVector& minBox, ndVector& maxBox) const;
	void MakeBox2(const ndMatrix& faceMatrix, ndInt32 indexCount, const ndInt32* const indexArray, ndInt32 stride, const ndFloat32* const vertexArray, ndVector& minBox, ndVector& maxBox) const;

	protected:
	ndMatrix m_absDir;
	ndVector m_p0;
	ndVector m_p1;
	ndVector m_size;
	mutable ndVector m_separationDistance;

	friend class ndAabbPolygonSoup;
} D_GCC_NEWTON_CLASS_ALIGN_32 ;


#endif

