/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __dgMatrix__
#define __dgMatrix__


#include "dgStdafx.h"
#include "dgDebug.h"
#include "dgVector.h"
#include "dgPlane.h"
#include "dgSimd.h"

#include <math.h>

class dgMatrix;
class dgQuaternion;

const dgMatrix& dgGetZeroMatrix ();
const dgMatrix& dgGetIdentityMatrix();


DG_MSC_VECTOR_AVX_ALIGMENT
class dgMatrix
{
	public:
	DG_CLASS_ALLOCATOR(allocator)

	dgMatrix ();
	dgMatrix (const dgFloat32* const array);
	dgMatrix (const dgVector &front, const dgVector &up, const dgVector &right, const dgVector &posit);
	dgMatrix (const dgQuaternion &rotation, const dgVector &position);

	// create a orthonormal normal vector basis
	dgMatrix (const dgVector &front);


	

	dgVector& operator[] (dgInt32 i);
	const dgVector& operator[] (dgInt32 i) const;

	dgMatrix Inverse () const;
	dgMatrix Inverse4x4 () const;
	dgMatrix Transpose () const;
	dgMatrix Transpose4X4 () const;
	dgMatrix Symetric3by3Inverse () const;
	dgVector RotateVector (const dgVector &v) const;
	dgVector UnrotateVector (const dgVector &v) const;
	dgVector TransformVector (const dgVector &v) const;
	dgVector UntransformVector (const dgVector &v) const;
	dgPlane TransformPlane (const dgPlane &localPlane) const;
	dgPlane UntransformPlane (const dgPlane &globalPlane) const;
	void TransformBBox (const dgVector& p0local, const dgVector& p1local, dgVector& p0, dgVector& p1) const; 

	dgVector CalcPitchYawRoll () const;
	void TransformTriplex (dgFloat32* const dst, dgInt32 dstStrideInBytes,
						   const dgFloat32* const src, dgInt32 srcStrideInBytes, dgInt32 count) const;

	void TransformTriplex (dgFloat64* const dst, dgInt32 dstStrideInBytes,
						   const dgFloat64* const src, dgInt32 srcStrideInBytes, dgInt32 count) const;

	void TransformTriplex (dgFloat64* const dst, dgInt32 dstStrideInBytes,
						   const dgFloat32* const src, dgInt32 srcStrideInBytes, dgInt32 count) const;


	dgMatrix operator* (const dgMatrix &B) const;
	

	// this function can not be a member of dgMatrix, because
	// dgMatrix a define to handle only orthogonal matrices
	// and this function take a parameter to a symmetric matrix
	void EigenVectors (dgVector &eigenValues, const dgMatrix& initialGuess = dgGetIdentityMatrix());
	void EigenVectors (const dgMatrix& initialGuess = dgGetIdentityMatrix());


	// simd operations
	dgMatrix InverseSimd () const;
	dgMatrix MultiplySimd (const dgMatrix& B) const;
	dgSimd RotateVectorSimd (const dgVector &v) const;
	dgSimd UnrotateVectorSimd (const dgVector &v) const;
	dgSimd TransformVectorSimd (const dgVector &v) const;
	dgSimd UntransformVectorSimd (const dgVector &v) const;
	void TransformVectorsSimd (dgVector* const dst, const dgVector* const src, dgInt32 count) const;

	dgVector m_front;
	dgVector m_up;
	dgVector m_right;
	dgVector m_posit;
} DG_GCC_VECTOR_AVX_ALIGMENT;



DG_INLINE dgMatrix::dgMatrix ()
{
}

DG_INLINE dgMatrix::dgMatrix (const dgFloat32* const array)
{
	memcpy (&m_front.m_x, array, sizeof (dgMatrix)) ;
}

DG_INLINE dgMatrix::dgMatrix (const dgVector &front, const dgVector &up, const dgVector &right, const dgVector &posit)
	:m_front (front), m_up(up), m_right(right), m_posit(posit)
{
}

DG_INLINE dgMatrix::dgMatrix (const dgVector& front)
{
	m_front = front; 
	if (dgAbsf (front.m_z) > dgFloat32 (0.577f)) {
		m_right = front * dgVector (-front.m_y, front.m_z, dgFloat32(0.0f), dgFloat32(0.0f));
	} else {
	  	m_right = front * dgVector (-front.m_y, front.m_x, dgFloat32(0.0f), dgFloat32(0.0f));
	}
  	m_right = m_right.Scale3 (dgRsqrt (m_right % m_right));
  	m_up = m_right * m_front;

	m_front.m_w = dgFloat32(0.0f);
	m_up.m_w = dgFloat32(0.0f);
	m_right.m_w = dgFloat32(0.0f);
	m_posit = dgVector (dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(1.0f));

	dgAssert ((dgAbsf (m_front % m_front) - dgFloat32(1.0f)) < dgFloat32(1.0e-5f)); 
	dgAssert ((dgAbsf (m_up % m_up) - dgFloat32(1.0f)) < dgFloat32(1.0e-5f)); 
	dgAssert ((dgAbsf (m_right % m_right) - dgFloat32(1.0f)) < dgFloat32(1.0e-5f)); 
	dgAssert ((dgAbsf (m_right % (m_front * m_up)) - dgFloat32(1.0f)) < dgFloat32(1.0e-5f)); 
}


DG_INLINE dgVector& dgMatrix::operator[] (dgInt32  i)
{
	dgAssert (i < 4);
	dgAssert (i >= 0);
	return (&m_front)[i];
}

DG_INLINE const dgVector& dgMatrix::operator[] (dgInt32  i) const
{
	dgAssert (i < 4);
	dgAssert (i >= 0);
	return (&m_front)[i];
}


DG_INLINE dgMatrix dgMatrix::Transpose () const
{
#ifdef DG_SIMD_VECTOR_CLASS
	dgMatrix tmp;
	dgVector::Transpose4x4 (tmp.m_front, tmp.m_up, tmp.m_right, tmp.m_posit, m_front, m_up, m_right, dgVector::m_wOne); 
	return tmp;
#else
	return dgMatrix (dgVector (m_front.m_x, m_up.m_x, m_right.m_x, dgFloat32(0.0f)),
		dgVector (m_front.m_y, m_up.m_y, m_right.m_y, dgFloat32(0.0f)),
		dgVector (m_front.m_z, m_up.m_z, m_right.m_z, dgFloat32(0.0f)),
		dgVector (dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(1.0f)));
#endif
}

DG_INLINE dgMatrix dgMatrix::Transpose4X4 () const
{
#ifdef DG_SIMD_VECTOR_CLASS
	dgMatrix tmp;
	dgVector::Transpose4x4 (tmp.m_front, tmp.m_up, tmp.m_right, tmp.m_posit, m_front, m_up, m_right, m_posit); 
	return tmp;
#else 
	return dgMatrix (dgVector (m_front.m_x, m_up.m_x, m_right.m_x, m_posit.m_x),
					 dgVector (m_front.m_y, m_up.m_y, m_right.m_y, m_posit.m_y),
					 dgVector (m_front.m_z, m_up.m_z, m_right.m_z, m_posit.m_z),
					 dgVector (m_front.m_w, m_up.m_w, m_right.m_w, m_posit.m_w));
#endif
}

DG_INLINE dgVector dgMatrix::RotateVector (const dgVector &v) const
{
	return dgVector (m_front.Scale4(v.m_x) + m_up.Scale4(v.m_y) + m_right.Scale4(v.m_z));
}


DG_INLINE dgVector dgMatrix::UnrotateVector (const dgVector &v) const
{
#ifdef DG_SIMD_VECTOR_CLASS
	dgVector x (v.CompProduct4(m_front));
	dgVector y (v.CompProduct4(m_up));
	dgVector z (v.CompProduct4(m_right));
	dgVector w;
	dgVector::Transpose4x4 (x, y, z, w, x, y, z, w); 
	return x + y + z;
#else
	return dgVector (v % m_front, v % m_up, v % m_right, dgFloat32 (0.0f));
#endif
}


DG_INLINE dgVector dgMatrix::TransformVector (const dgVector &v) const
{
/*
	dgVector tmp0 (v.m_x * m_front.m_x + v.m_y * m_up.m_x + v.m_z * m_right.m_x + m_posit.m_x,
				   v.m_x * m_front.m_y + v.m_y * m_up.m_y + v.m_z * m_right.m_y + m_posit.m_y,
				   v.m_x * m_front.m_z + v.m_y * m_up.m_z + v.m_z * m_right.m_z + m_posit.m_z, v.m_w);
	reurn dgVector (m_front.Scale4(v.m_x) + m_up.Scale4(v.m_y) + m_right.Scale4(v.m_z) + m_posit.Scale4(v.m_w));
*/
	return dgVector (m_front.Scale4(v.m_x) + m_up.Scale4(v.m_y) + m_right.Scale4(v.m_z) + m_posit);
}

DG_INLINE dgVector dgMatrix::UntransformVector (const dgVector &v) const
{
	return UnrotateVector(v - m_posit);
}

DG_INLINE dgPlane dgMatrix::TransformPlane (const dgPlane &localPlane) const
{
	return dgPlane (RotateVector (localPlane), localPlane.m_w - (localPlane % UnrotateVector (m_posit)));  
}

DG_INLINE dgPlane dgMatrix::UntransformPlane (const dgPlane &globalPlane) const
{
	return dgPlane (UnrotateVector (globalPlane), globalPlane.Evalue(m_posit));
}

DG_INLINE void dgMatrix::EigenVectors (const dgMatrix& initialGuess)
{
	dgVector eigenValues;
	EigenVectors (eigenValues, initialGuess);
}


DG_INLINE dgMatrix dgPitchMatrix(dgFloat32 ang)
{
	dgFloat32 cosAng;
	dgFloat32 sinAng;
	sinAng = dgSin (ang);
	cosAng = dgCos (ang);
	return dgMatrix (dgVector (dgFloat32(1.0f),  dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f)), 
					 dgVector (dgFloat32(0.0f),  cosAng,          sinAng,          dgFloat32(0.0f)),
					 dgVector (dgFloat32(0.0f), -sinAng,          cosAng,          dgFloat32(0.0f)), 
					 dgVector (dgFloat32(0.0f),  dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(1.0f))); 

}

DG_INLINE dgMatrix dgYawMatrix(dgFloat32 ang)
{
	dgFloat32 cosAng;
	dgFloat32 sinAng;
	sinAng = dgSin (ang);
	cosAng = dgCos (ang);
	return dgMatrix (dgVector (cosAng,          dgFloat32(0.0f), -sinAng,          dgFloat32(0.0f)), 
					 dgVector (dgFloat32(0.0f), dgFloat32(1.0f),  dgFloat32(0.0f), dgFloat32(0.0f)), 
					 dgVector (sinAng,          dgFloat32(0.0f),  cosAng,          dgFloat32(0.0f)), 
					 dgVector (dgFloat32(0.0f), dgFloat32(0.0f),  dgFloat32(0.0f), dgFloat32(1.0f))); 
}

DG_INLINE dgMatrix dgRollMatrix(dgFloat32 ang)
{
	dgFloat32 cosAng;
	dgFloat32 sinAng;
	sinAng = dgSin (ang);
	cosAng = dgCos (ang);
	return dgMatrix (dgVector ( cosAng,          sinAng,          dgFloat32(0.0f), dgFloat32(0.0f)), 
					 dgVector (-sinAng,          cosAng,          dgFloat32(0.0f), dgFloat32(0.0f)),
					 dgVector ( dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(1.0f), dgFloat32(0.0f)), 
					 dgVector ( dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(1.0f))); 
}																		 


DG_INLINE dgMatrix dgMatrix::Inverse () const
{
	return dgMatrix (dgVector (m_front.m_x, m_up.m_x, m_right.m_x, dgFloat32(0.0f)),
					 dgVector (m_front.m_y, m_up.m_y, m_right.m_y, dgFloat32(0.0f)),
					 dgVector (m_front.m_z, m_up.m_z, m_right.m_z, dgFloat32(0.0f)),
					 dgVector (- (m_posit % m_front), - (m_posit % m_up), - (m_posit % m_right), dgFloat32(1.0f)));
}

DG_INLINE dgSimd dgMatrix::TransformVectorSimd (const dgVector &v) const
{
	const dgMatrix& source = *this;
	return (dgSimd&)source[0] * dgSimd(v[0]) + (dgSimd&)source[1] * dgSimd(v[1]) + (dgSimd&)source[2] * dgSimd(v[2]) + (dgSimd&)source[3];
}

DG_INLINE dgSimd dgMatrix::UntransformVectorSimd (const dgVector &v) const
{
	return UnrotateVectorSimd((dgSimd&)v - (dgSimd&)m_posit);
}

DG_INLINE void dgMatrix::TransformVectorsSimd (dgVector* const dst, const dgVector* const src, dgInt32 count) const
{
	for (dgInt32 i = 0; i < count; i ++) {
		dst[i] = TransformVectorSimd (src[i]);
	}
}


DG_INLINE dgSimd dgMatrix::RotateVectorSimd (const dgVector &v) const
{
	const dgMatrix& source = *this;
	return (dgSimd&)source[0] * dgSimd(v[0])  + (dgSimd&)source[1] * dgSimd(v[1]) + (dgSimd&)source[2] * dgSimd(v[2]);
}



DG_INLINE dgSimd dgMatrix::UnrotateVectorSimd (const dgVector &v) const
{
	dgSimd x ((dgSimd&)v * (dgSimd&)m_front);
	dgSimd y ((dgSimd&)v * (dgSimd&)m_up);
	dgSimd z ((dgSimd&)v * (dgSimd&)m_right);

	x = x.AddHorizontal();
	y = y.AddHorizontal();
	z = z.AddHorizontal();
	return x.PackLow(y).MoveLow(z);
}




DG_INLINE dgMatrix dgMatrix::InverseSimd () const
{
	dgMatrix matrix;
	const dgMatrix& source = *this;

	dgSimd tmp2 (dgFloat32 (0.0f));
	dgSimd tmp0 (((dgSimd&)source[0]).PackLow((dgSimd&)source[1]));
	dgSimd tmp1 (((dgSimd&)source[2]).PackLow(tmp2));
	matrix[0] = tmp0.MoveLow (tmp1);
	matrix[1] = tmp1.MoveHigh (tmp0);

	tmp0 = ((dgSimd&)source[0]).PackHigh((dgSimd&)source[1]);
	tmp1 = ((dgSimd&)source[2]).PackHigh(tmp2);
	matrix[2] = tmp0.MoveLow (tmp1);

	matrix[3] = tmp2 - dgSimd (source.m_posit.m_x) * (dgSimd&)matrix[0] - dgSimd (source.m_posit.m_y) * (dgSimd&)matrix[1] - dgSimd (source.m_posit.m_z) * (dgSimd&)matrix[2];
	matrix[3][3] = dgFloat32 (1.0f);
	return matrix;
}

DG_INLINE dgMatrix dgMatrix::MultiplySimd (const dgMatrix& B) const
{
	dgMatrix matrix;
	const dgMatrix& A = *this;
//	matrix[0] = ((dgSimd&)B[0]) * dgSimd (A[0][0]) + ((dgSimd&)B[1]) * dgSimd (A[0][1]) + ((dgSimd&)B[2]) * dgSimd (A[0][2]) + ((dgSimd&)B[3]) * dgSimd (A[0][3]);
//	matrix[1] = ((dgSimd&)B[0]) * dgSimd (A[1][0]) + ((dgSimd&)B[1]) * dgSimd (A[1][1]) + ((dgSimd&)B[2]) * dgSimd (A[1][2]) + ((dgSimd&)B[3]) * dgSimd (A[1][3]);
//	matrix[2] = ((dgSimd&)B[0]) * dgSimd (A[2][0]) + ((dgSimd&)B[1]) * dgSimd (A[2][1]) + ((dgSimd&)B[2]) * dgSimd (A[2][2]) + ((dgSimd&)B[3]) * dgSimd (A[2][3]);
//	matrix[3] = ((dgSimd&)B[0]) * dgSimd (A[3][0]) + ((dgSimd&)B[1]) * dgSimd (A[3][1]) + ((dgSimd&)B[2]) * dgSimd (A[3][2]) + ((dgSimd&)B[3]) * dgSimd (A[3][3]);
	for (dgInt32 i = 0; i < 4; i ++) {
		matrix[i] = ((dgSimd&)B[0]) * dgSimd (A[i][0]) + ((dgSimd&)B[1]) * dgSimd (A[i][1]) + ((dgSimd&)B[2]) * dgSimd (A[i][2]) + ((dgSimd&)B[3]) * dgSimd (A[i][3]);
	}
	return matrix;
}

#endif

