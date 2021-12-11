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

#ifndef __ND_FAST_AABB_H__
#define __ND_FAST_AABB_H__

#include "ndCoreStdafx.h"
#include "ndDebug.h"
#include "ndVector.h"
#include "ndMatrix.h"

D_MSV_NEWTON_ALIGN_32 
class ndFastAabb : public ndMatrix
{
	public:
	ndFastAabb();
	ndFastAabb(const ndVector& p0, const ndVector& p1);
	ndFastAabb(const ndMatrix& matrix, const ndVector& size);

	const ndVector& GetOrigin() const;
	const ndVector& GetTarget() const;

	void SetSeparatingDistance(const dFloat32 distance);
	void SetTransposeAbsMatrix(const ndMatrix& matrix);

	D_CORE_API dFloat32 PolygonBoxDistance(const ndVector& faceNormal, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray) const;
	D_CORE_API dFloat32 PolygonBoxRayDistance(const ndVector& faceNormal, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray, const ndFastRay& ray) const;

	private:
	ndMatrix MakeFaceMatrix(const ndVector& faceNormal, dInt32, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray) const;
	void MakeBox1(dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray, ndVector& minBox, ndVector& maxBox) const;
	void MakeBox2(const ndMatrix& faceMatrix, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray, ndVector& minBox, ndVector& maxBox) const;

	protected:
	ndMatrix m_absDir;
	ndVector m_p0;
	ndVector m_p1;
	ndVector m_size;
	mutable ndVector m_separationDistance;

	friend class ndAabbPolygonSoup;
} D_GCC_NEWTON_ALIGN_32 ;

inline ndFastAabb::ndFastAabb()
	:ndMatrix(dGetIdentityMatrix())
	,m_absDir(dGetIdentityMatrix())
	,m_p0(ndVector::m_zero)
	,m_p1(ndVector::m_zero)
	,m_size(ndVector::m_zero)
	,m_separationDistance(dFloat32(1.0e10f))
{
}

inline ndFastAabb::ndFastAabb(const ndMatrix& matrix, const ndVector& size)
	:ndMatrix(matrix)
	,m_separationDistance(dFloat32(1.0e10f))
{
	SetTransposeAbsMatrix(matrix);
	m_size = ndVector(matrix[0].Abs().Scale(size.m_x) + matrix[1].Abs().Scale(size.m_y) + matrix[2].Abs().Scale(size.m_z));
	m_p0 = (matrix[3] - m_size) & ndVector::m_triplexMask;
	m_p1 = (matrix[3] + m_size) & ndVector::m_triplexMask;
}

inline ndFastAabb::ndFastAabb(const ndVector& p0, const ndVector& p1)
	:ndMatrix(dGetIdentityMatrix())
	,m_absDir(dGetIdentityMatrix())
	,m_p0(p0)
	,m_p1(p1)
	,m_size(ndVector::m_half * (p1 - p0))
	,m_separationDistance(dFloat32(1.0e10f))
{
	m_posit = (ndVector::m_half * (p1 + p0)) | ndVector::m_wOne;
	dAssert(m_size.m_w == dFloat32(0.0f));
	dAssert(m_posit.m_w == dFloat32(1.0f));
}

inline const ndVector& ndFastAabb::GetOrigin() const
{
	return m_p0;
}

inline const ndVector& ndFastAabb::GetTarget() const
{
	return m_p1;
}

inline void ndFastAabb::SetSeparatingDistance(const dFloat32 distance)
{
	m_separationDistance = distance;
}

inline void ndFastAabb::SetTransposeAbsMatrix(const ndMatrix& matrix)
{
	m_absDir = matrix.Transpose();
	m_absDir[0] = m_absDir[0].Abs();
	m_absDir[1] = m_absDir[1].Abs();
	m_absDir[2] = m_absDir[2].Abs();
}

#endif

