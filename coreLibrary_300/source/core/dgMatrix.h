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
#include <math.h>

class dgMatrix;
class dgQuaternion;

const dgMatrix& dgGetZeroMatrix ();
const dgMatrix& dgGetIdentityMatrix();


DG_MSC_VECTOR_ALIGMENT
class dgMatrix
{
	public:
	DG_CLASS_ALLOCATOR(allocator)

	dgMatrix ();
	dgMatrix (const dgFloat32* const array);
	dgMatrix (const dgVector &front, const dgVector &up, const dgVector &right, const dgVector &posit);
	dgMatrix (const dgQuaternion &rotation, const dgVector &position);

	// create a orthonormal normal vector basis, front become m_front vector, and m_up and m_right are mutualiperpendicular to fron and to each other
	dgMatrix (const dgVector &front);

	// create a covariance Matrix = transpose(p) * q 
	dgMatrix (const dgVector& p, const dgVector& q);

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

	void CalcPitchYawRoll (dgVector& euler0, dgVector& euler1) const;
	void TransformTriplex (dgFloat32* const dst, dgInt32 dstStrideInBytes,
						   const dgFloat32* const src, dgInt32 srcStrideInBytes, dgInt32 count) const;

#ifndef _NEWTON_USE_DOUBLE
	void TransformTriplex (dgFloat64* const dst, dgInt32 dstStrideInBytes,
						   const dgFloat64* const src, dgInt32 srcStrideInBytes, dgInt32 count) const;

	void TransformTriplex (dgFloat64* const dst, dgInt32 dstStrideInBytes,
						   const dgFloat32* const src, dgInt32 srcStrideInBytes, dgInt32 count) const;
#endif

	bool TestOrthogonal() const;
	bool TestSymetric3x3() const;

	dgMatrix operator* (const dgMatrix &B) const;


	
	

	// these function can only be called when dgMatrix is a PDS matrix
	void EigenVectors (dgVector &eigenValues, const dgMatrix& initialGuess = dgGetIdentityMatrix());
	void EigenVectors (const dgMatrix& initialGuess = dgGetIdentityMatrix());
	void PolarDecomposition (dgMatrix& transformMatrix, dgVector& scale, dgMatrix& stretchAxis, const dgMatrix& initialStretchAxis = dgGetIdentityMatrix()) const;

	// constructor for polar composition
	dgMatrix (const dgMatrix& transformMatrix, const dgVector& scale, const dgMatrix& stretchAxis);

	dgVector m_front;
	dgVector m_up;
	dgVector m_right;
	dgVector m_posit;

	static dgMatrix m_zeroMatrix;
	static dgMatrix m_identityMatrix;
} DG_GCC_VECTOR_ALIGMENT;



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

DG_INLINE dgMatrix::dgMatrix (const dgVector& p, const dgVector& q)
	:m_front(q.CompProduct4(p.BroadcastX()))
	,m_up(q.CompProduct4(p.BroadcastY()))
	,m_right(q.CompProduct4(p.BroadcastZ()))
	,m_posit (dgVector::m_wOne)
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
#ifdef DG_SCALAR_VECTOR_CLASS
	return dgMatrix (dgVector (m_front.m_x, m_up.m_x, m_right.m_x, dgFloat32(0.0f)),
					 dgVector (m_front.m_y, m_up.m_y, m_right.m_y, dgFloat32(0.0f)),
					 dgVector (m_front.m_z, m_up.m_z, m_right.m_z, dgFloat32(0.0f)),
					 dgVector (dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(1.0f)));
#else
	dgMatrix tmp;
	dgVector::Transpose4x4 (tmp.m_front, tmp.m_up, tmp.m_right, tmp.m_posit, m_front, m_up, m_right, dgVector::m_wOne); 
	return tmp;
#endif
}

DG_INLINE dgMatrix dgMatrix::Transpose4X4 () const
{
#ifdef DG_SCALAR_VECTOR_CLASS
	return dgMatrix (dgVector (m_front.m_x, m_up.m_x, m_right.m_x, m_posit.m_x),
					 dgVector (m_front.m_y, m_up.m_y, m_right.m_y, m_posit.m_y),
					 dgVector (m_front.m_z, m_up.m_z, m_right.m_z, m_posit.m_z),
					 dgVector (m_front.m_w, m_up.m_w, m_right.m_w, m_posit.m_w));
#else 
	dgMatrix tmp;
	dgVector::Transpose4x4 (tmp.m_front, tmp.m_up, tmp.m_right, tmp.m_posit, m_front, m_up, m_right, m_posit); 
	return tmp;
#endif
}

DG_INLINE dgVector dgMatrix::RotateVector (const dgVector &v) const
{
	return dgVector (m_front.Scale4(v.m_x) + m_up.Scale4(v.m_y) + m_right.Scale4(v.m_z));
}


DG_INLINE dgVector dgMatrix::UnrotateVector (const dgVector &v) const
{
#ifdef DG_SCALAR_VECTOR_CLASS
	return dgVector (v % m_front, v % m_up, v % m_right, dgFloat32 (0.0f));
#else
	dgVector x (v.CompProduct4(m_front));
	dgVector y (v.CompProduct4(m_up));
	dgVector z (v.CompProduct4(m_right));
	dgVector w (dgFloat32 (0.0f));
	dgVector::Transpose4x4 (x, y, z, w, x, y, z, w); 
	return x + y + z;

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


DG_INLINE dgMatrix dgMatrix::Inverse () const
{
	return dgMatrix (dgVector (m_front.m_x, m_up.m_x, m_right.m_x, dgFloat32(0.0f)),
					 dgVector (m_front.m_y, m_up.m_y, m_right.m_y, dgFloat32(0.0f)),
					 dgVector (m_front.m_z, m_up.m_z, m_right.m_z, dgFloat32(0.0f)),
					 dgVector (- (m_posit % m_front), - (m_posit % m_up), - (m_posit % m_right), dgFloat32(1.0f)));
}


DG_INLINE bool dgMatrix::TestOrthogonal() const
{
	dgVector n (m_front * m_up);
	dgFloat32 a = m_right % m_right;
	dgFloat32 b = m_up % m_up;
	dgFloat32 c = m_front % m_front;
	dgFloat32 d = n % m_right;

	return (m_front[3] == dgFloat32 (0.0f)) & 
		   (m_up[3] == dgFloat32 (0.0f)) & 
		   (m_right[3] == dgFloat32 (0.0f)) & 
		   (m_posit[3] == dgFloat32 (1.0f)) &
		   (dgAbsf(a - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f)) & 
		   (dgAbsf(b - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f)) &
		   (dgAbsf(c - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f)) &
		   (dgAbsf(d - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f)); 
}

DG_INLINE bool dgMatrix::TestSymetric3x3() const
{
	const dgMatrix& me = *this;
	return (dgAbsf (me[0][1] - me[1][0]) < dgFloat32 (1.0e-5f)) && 
		   (dgAbsf (me[0][2] - me[2][0]) < dgFloat32 (1.0e-5f)) &&
		   (dgAbsf (me[1][2] - me[2][1]) < dgFloat32 (1.0e-5f)) &&
		   (me[0][3] == dgFloat32 (0.0f)) &&
		   (me[1][3] == dgFloat32 (0.0f)) &&
		   (me[2][3] == dgFloat32 (0.0f)) &&
		   (me[3][0] == dgFloat32 (0.0f)) &&
		   (me[3][1] == dgFloat32 (0.0f)) &&
		   (me[3][2] == dgFloat32 (0.0f)) &&
		   (me[3][3] == dgFloat32 (1.0f));
}


DG_INLINE dgMatrix dgPitchMatrix(dgFloat32 ang)
{
	dgFloat32 sinAng = dgSin (ang);
	dgFloat32 cosAng = dgCos (ang);
	return dgMatrix (dgVector (dgFloat32(1.0f),  dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f)), 
		dgVector (dgFloat32(0.0f),  cosAng,          sinAng,          dgFloat32(0.0f)),
		dgVector (dgFloat32(0.0f), -sinAng,          cosAng,          dgFloat32(0.0f)), 
		dgVector (dgFloat32(0.0f),  dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(1.0f))); 

}

DG_INLINE dgMatrix dgYawMatrix(dgFloat32 ang)
{
	dgFloat32 sinAng = dgSin (ang);
	dgFloat32 cosAng = dgCos (ang);
	return dgMatrix (dgVector (cosAng,          dgFloat32(0.0f), -sinAng,          dgFloat32(0.0f)), 
		dgVector (dgFloat32(0.0f), dgFloat32(1.0f),  dgFloat32(0.0f), dgFloat32(0.0f)), 
		dgVector (sinAng,          dgFloat32(0.0f),  cosAng,          dgFloat32(0.0f)), 
		dgVector (dgFloat32(0.0f), dgFloat32(0.0f),  dgFloat32(0.0f), dgFloat32(1.0f))); 
}

DG_INLINE dgMatrix dgRollMatrix(dgFloat32 ang)
{
	dgFloat32 sinAng = dgSin (ang);
	dgFloat32 cosAng = dgCos (ang);
	return dgMatrix (dgVector ( cosAng,          sinAng,          dgFloat32(0.0f), dgFloat32(0.0f)), 
		dgVector (-sinAng,          cosAng,          dgFloat32(0.0f), dgFloat32(0.0f)),
		dgVector ( dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(1.0f), dgFloat32(0.0f)), 
		dgVector ( dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(1.0f))); 
}																		 



#endif

