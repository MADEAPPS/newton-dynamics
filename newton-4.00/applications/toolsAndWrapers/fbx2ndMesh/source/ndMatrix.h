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

#ifndef __ND_MATRIX_H__
#define __ND_MATRIX_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndDebug.h"
#include "ndPlane.h"
#include "ndVector.h"

class ndMatrix;
class ndQuaternion;

D_CORE_API const ndMatrix& ndGetZeroMatrix ();
D_CORE_API const ndMatrix& ndGetIdentityMatrix();
D_CORE_API ndMatrix ndYawMatrix(ndFloat32 ang);
D_CORE_API ndMatrix ndRollMatrix(ndFloat32 ang);
D_CORE_API ndMatrix ndPitchMatrix(ndFloat32 ang);
D_CORE_API ndMatrix ndGramSchmidtMatrix(const ndVector& dir);
D_CORE_API ndMatrix ndCovarianceMatrix(const ndVector& p, const ndVector& q);
D_CORE_API ndMatrix ndCalculateMatrix(const ndQuaternion& rotation, const ndVector& position = ndVector::m_wOne);

D_MSV_NEWTON_CLASS_ALIGN_32
class ndMatrix
{
	public:
	D_OPERATOR_NEW_AND_DELETE

	ndMatrix ();
	ndMatrix (const ndFloat32* const array);
	ndMatrix (const ndVector &front, const ndVector &up, const ndVector &right, const ndVector &posit);

	// please use function ndCalculateMatrix()
	//D_CORE_API ndMatrix (const ndQuaternion &rotation, const ndVector &position);

	// create a orthonormal normal vector basis, front become m_front vector, and m_up and m_right are mutualiperpendicular to fron and to each other
	// please use function ndGramSchmidtMatrix
	//ndMatrix (const ndVector &front);

	// create a covariance Matrix = transpose(p) * q 
	// please use function ndCovariance
	//ndMatrix (const ndVector& p, const ndVector& q);

	ndVector& operator[] (ndInt32 i);
	const ndVector& operator[] (ndInt32 i) const;
	
	D_CORE_API ndMatrix Inverse() const;
	D_CORE_API ndMatrix OrthoInverse() const;
	D_CORE_API ndMatrix Transpose3x3 () const;
	D_CORE_API ndMatrix Transpose4X4 () const;
	D_CORE_API ndVector RotateVector (const ndVector &v) const;
	D_CORE_API ndVector UnrotateVector (const ndVector &v) const;
	D_CORE_API ndVector TransformVector (const ndVector &v) const;
	D_CORE_API ndVector UntransformVector (const ndVector &v) const;
	D_CORE_API ndVector TransformVector1x4(const ndVector& v) const;
	D_CORE_API ndPlane TransformPlane (const ndPlane &localPlane) const;
	D_CORE_API ndPlane UntransformPlane (const ndPlane &globalPlane) const;

	D_CORE_API ndMatrix Inverse4x4() const;
	D_CORE_API ndVector SolveByGaussianElimination(const ndVector &v) const;
	D_CORE_API void TransformBBox (const ndVector& p0local, const ndVector& p1local, ndVector& p0, ndVector& p1) const;

	D_CORE_API ndVector CalcPitchYawRoll (ndVector& euler) const;
	D_CORE_API void TransformTriplex (ndFloat32* const dst, ndInt32 dstStrideInBytes, const ndFloat32* const src, ndInt32 srcStrideInBytes, ndInt32 count) const;

#ifndef D_NEWTON_USE_DOUBLE
	D_CORE_API void TransformTriplex (
		ndFloat64* const dst, ndInt32 dstStrideInBytes,
		const ndFloat64* const src, ndInt32 srcStrideInBytes, ndInt32 count) const;

	D_CORE_API void TransformTriplex (
		ndFloat64* const dst, ndInt32 dstStrideInBytes,
		const ndFloat32* const src, ndInt32 srcStrideInBytes, ndInt32 count) const;
#endif
	D_CORE_API bool SanityCheck() const;
	D_CORE_API bool TestIdentity() const;
	D_CORE_API bool TestSymetric3x3() const;
	D_CORE_API bool TestOrthogonal(ndFloat32 tol = ndFloat32 (1.0e-4f)) const;

	D_CORE_API ndMatrix Multiply3X3 (const ndMatrix &B) const;
	D_CORE_API ndMatrix operator* (const ndMatrix &B) const;

	// these function can only be called when ndMatrix is a PDS matrix
	//void EigenVectors ();
	D_CORE_API ndVector EigenVectors ();
	D_CORE_API void PolarDecomposition (ndMatrix& transformMatrix, ndVector& scale, ndMatrix& stretchAxis) const;

	// constructor for polar composition
	D_CORE_API ndMatrix (const ndMatrix& transformMatrix, const ndVector& scale, const ndMatrix& stretchAxis);

	ndVector m_front;
	ndVector m_up;
	ndVector m_right;
	ndVector m_posit;
} D_GCC_NEWTON_CLASS_ALIGN_32 ;

inline ndMatrix::ndMatrix ()
{
}

inline ndMatrix::ndMatrix (const ndFloat32* const array)
{
	ndMemCpy(&m_front.m_x, array, sizeof(ndMatrix) / sizeof (ndFloat32));
}

inline ndMatrix::ndMatrix (const ndVector &front, const ndVector &up, const ndVector &right, const ndVector &posit)
	:m_front (front), m_up(up), m_right(right), m_posit(posit)
{
}

inline ndVector& ndMatrix::operator[] (ndInt32  i)
{
	ndAssert (i < 4);
	ndAssert (i >= 0);
	return (&m_front)[i];
}

inline const ndVector& ndMatrix::operator[] (ndInt32  i) const
{
	ndAssert (i < 4);
	ndAssert (i >= 0);
	return (&m_front)[i];
}

#endif

