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

#ifndef __D_FAST_AABB_H__
#define __D_FAST_AABB_H__

#include "dCoreStdafx.h"
#include "dDebug.h"
#include "dVector.h"
#include "dMatrix.h"

D_MSV_NEWTON_ALIGN_32 
class dFastAabb : public dMatrix
{
	public:
	dFastAabb();
	dFastAabb(const dVector& p0, const dVector& p1);
	dFastAabb(const dMatrix& matrix, const dVector& size);

	const dVector& GetOrigin() const;
	const dVector& GetTarget() const;

	void SetSeparatingDistance(const dFloat32 distance);
	void SetTransposeAbsMatrix(const dMatrix& matrix);

	D_CORE_API dFloat32 PolygonBoxDistance(const dVector& faceNormal, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray) const;
	D_CORE_API dFloat32 PolygonBoxRayDistance(const dVector& faceNormal, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray, const dFastRay& ray) const;

	private:
	dMatrix MakeFaceMatrix(const dVector& faceNormal, dInt32, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray) const;
	void MakeBox1(dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray, dVector& minBox, dVector& maxBox) const;
	void MakeBox2(const dMatrix& faceMatrix, dInt32 indexCount, const dInt32* const indexArray, dInt32 stride, const dFloat32* const vertexArray, dVector& minBox, dVector& maxBox) const;

	protected:
	dMatrix m_absDir;
	dVector m_p0;
	dVector m_p1;
	dVector m_size;
	mutable dVector m_separationDistance;

	friend class dAabbPolygonSoup;
} D_GCC_NEWTON_ALIGN_32 ;

inline dFastAabb::dFastAabb()
	:dMatrix(dGetIdentityMatrix())
	,m_absDir(dGetIdentityMatrix())
	,m_p0(dVector::m_zero)
	,m_p1(dVector::m_zero)
	,m_size(dVector::m_zero)
	,m_separationDistance(dFloat32(1.0e10f))
{
}

inline dFastAabb::dFastAabb(const dMatrix& matrix, const dVector& size)
	:dMatrix(matrix)
	,m_separationDistance(dFloat32(1.0e10f))
{
	SetTransposeAbsMatrix(matrix);
	m_size = dVector(matrix[0].Abs().Scale(size.m_x) + matrix[1].Abs().Scale(size.m_y) + matrix[2].Abs().Scale(size.m_z));
	m_p0 = (matrix[3] - m_size) & dVector::m_triplexMask;
	m_p1 = (matrix[3] + m_size) & dVector::m_triplexMask;
}

inline dFastAabb::dFastAabb(const dVector& p0, const dVector& p1)
	:dMatrix(dGetIdentityMatrix())
	,m_absDir(dGetIdentityMatrix())
	,m_p0(p0)
	,m_p1(p1)
	,m_size(dVector::m_half * (p1 - p0))
	,m_separationDistance(dFloat32(1.0e10f))
{
	m_posit = (dVector::m_half * (p1 + p0)) | dVector::m_wOne;
	dAssert(m_size.m_w == dFloat32(0.0f));
	dAssert(m_posit.m_w == dFloat32(1.0f));
}

inline const dVector& dFastAabb::GetOrigin() const
{
	return m_p0;
}

inline const dVector& dFastAabb::GetTarget() const
{
	return m_p1;
}

inline void dFastAabb::SetSeparatingDistance(const dFloat32 distance)
{
	m_separationDistance = distance;
}

inline void dFastAabb::SetTransposeAbsMatrix(const dMatrix& matrix)
{
	m_absDir = matrix.Transpose();
	m_absDir[0] = m_absDir[0].Abs();
	m_absDir[1] = m_absDir[1].Abs();
	m_absDir[2] = m_absDir[2].Abs();
}

#endif

