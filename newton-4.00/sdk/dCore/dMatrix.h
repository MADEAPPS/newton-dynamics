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

#ifndef __D_MATRIX_H__
#define __D_MATRIX_H__


#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dPlane.h"
#include "dVector.h"
#include "dClassAlloc.h"

//#include "dDebug.h"
//#include <math.h>

class dMatrix;
class dQuaternion;

D_CORE_API const dMatrix& dGetZeroMatrix ();
D_CORE_API const dMatrix& dGetIdentityMatrix();

D_MSC_VECTOR_ALIGNMENT
class dMatrix: public dClassAlloc
{
	public:
	dMatrix ();
	dMatrix (const dFloat32* const array);
	dMatrix (const dVector &front, const dVector &up, const dVector &right, const dVector &posit);
	D_CORE_API dMatrix (const dQuaternion &rotation, const dVector &position);

	// create a orthonormal normal vector basis, front become m_front vector, and m_up and m_right are mutualiperpendicular to fron and to each other
	dMatrix (const dVector &front);

	// create a covariance Matrix = transpose(p) * q 
	dMatrix (const dVector& p, const dVector& q);

	dVector& operator[] (dInt32 i);
	const dVector& operator[] (dInt32 i) const;

	dMatrix Inverse () const;
	D_CORE_API dMatrix Inverse4x4 () const;
	dMatrix Transpose () const;
	dMatrix Transpose4X4 () const;
	dVector RotateVector (const dVector &v) const;
	dVector UnrotateVector (const dVector &v) const;
	dVector TransformVector (const dVector &v) const;
	dVector UntransformVector (const dVector &v) const;
	dPlane TransformPlane (const dPlane &localPlane) const;
	dPlane UntransformPlane (const dPlane &globalPlane) const;
	D_CORE_API dVector SolveByGaussianElimination(const dVector &v) const;
	D_CORE_API void TransformBBox (const dVector& p0local, const dVector& p1local, dVector& p0, dVector& p1) const;

	D_CORE_API void CalcPitchYawRoll (dVector& euler0, dVector& euler1) const;
	D_CORE_API void TransformTriplex (
		dFloat32* const dst, dInt32 dstStrideInBytes,
		const dFloat32* const src, dInt32 srcStrideInBytes, dInt32 count) const;

#ifndef _NEWTON_USE_DOUBLE
	D_CORE_API void TransformTriplex (
		dFloat64* const dst, dInt32 dstStrideInBytes,
		const dFloat64* const src, dInt32 srcStrideInBytes, dInt32 count) const;

	D_CORE_API void TransformTriplex (
		dFloat64* const dst, dInt32 dstStrideInBytes,
		const dFloat32* const src, dInt32 srcStrideInBytes, dInt32 count) const;
#endif

	bool TestIdentity() const;
	bool TestSymetric3x3() const;
	bool TestOrthogonal(dFloat32 tol = dFloat32 (1.0e-4f)) const;

	D_CORE_API dMatrix Multiply3X3 (const dMatrix &B) const;
	D_CORE_API dMatrix operator* (const dMatrix &B) const;

	// these function can only be called when dMatrix is a PDS matrix
	//void EigenVectors ();
	D_CORE_API dVector EigenVectors ();
	D_CORE_API void PolarDecomposition (dMatrix& transformMatrix, dVector& scale, dMatrix& stretchAxis) const;

	// constructor for polar composition
	D_CORE_API dMatrix (const dMatrix& transformMatrix, const dVector& scale, const dMatrix& stretchAxis);

	dVector m_front;
	dVector m_up;
	dVector m_right;
	dVector m_posit;

	static dMatrix m_zeroMatrix;
	static dMatrix m_identityMatrix;
} D_GCC_VECTOR_ALIGNMENT;

D_INLINE dMatrix::dMatrix ()
{
}

D_INLINE dMatrix::dMatrix (const dFloat32* const array)
{
	memcpy (&m_front.m_x, array, sizeof (dMatrix)) ;
}

D_INLINE dMatrix::dMatrix (const dVector &front, const dVector &up, const dVector &right, const dVector &posit)
	:m_front (front), m_up(up), m_right(right), m_posit(posit)
{
}

D_INLINE dMatrix::dMatrix (const dVector& p, const dVector& q)
	:m_front(q * p.BroadcastX())
	,m_up   (q * p.BroadcastY())
	,m_right(q * p.BroadcastZ())
	,m_posit (dVector::m_wOne)
{
}

D_INLINE dMatrix::dMatrix (const dVector& front)
{
	m_front = front; 
	if (dAbs (front.m_z) > dFloat32 (0.577f)) 
	{
		m_right = front.CrossProduct(dVector (-front.m_y, front.m_z, dFloat32(0.0f), dFloat32(0.0f)));
	} 
	else 
	{
	  	m_right = front.CrossProduct(dVector (-front.m_y, front.m_x, dFloat32(0.0f), dFloat32(0.0f)));
	}
  	//m_right = m_right.Scale (dgRsqrt (m_right.DotProduct(m_right).GetScalar()));
	m_right = m_right.Normalize();
  	m_up = m_right.CrossProduct(m_front);

	m_front.m_w = dFloat32(0.0f);
	m_up.m_w = dFloat32(0.0f);
	m_right.m_w = dFloat32(0.0f);
	m_posit = dVector (dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f));

	dAssert ((dAbs (m_front.DotProduct(m_front).GetScalar()) - dFloat32(1.0f)) < dFloat32(1.0e-5f)); 
	dAssert ((dAbs (m_up.DotProduct(m_up).GetScalar()) - dFloat32(1.0f)) < dFloat32(1.0e-5f)); 
	dAssert ((dAbs (m_right.DotProduct(m_right).GetScalar()) - dFloat32(1.0f)) < dFloat32(1.0e-5f)); 
	dAssert ((dAbs (m_right.DotProduct(m_front.CrossProduct(m_up)).GetScalar()) - dFloat32(1.0f)) < dFloat32(1.0e-5f)); 
}

D_INLINE dVector& dMatrix::operator[] (dInt32  i)
{
	dAssert (i < 4);
	dAssert (i >= 0);
	return (&m_front)[i];
}

D_INLINE const dVector& dMatrix::operator[] (dInt32  i) const
{
	dAssert (i < 4);
	dAssert (i >= 0);
	return (&m_front)[i];
}


D_INLINE dMatrix dMatrix::Transpose () const
{
	dMatrix inv;
	dVector::Transpose4x4(inv[0], inv[1], inv[2], inv[3], m_front, m_up, m_right, dVector::m_wOne);
	return inv;
}

D_INLINE dMatrix dMatrix::Transpose4X4 () const
{
	dMatrix inv;
	dVector::Transpose4x4(inv[0], inv[1], inv[2], inv[3], m_front, m_up, m_right, m_posit);
	return inv;
}

D_INLINE dVector dMatrix::RotateVector (const dVector &v) const
{
	return m_front * v.BroadcastX() + m_up * v.BroadcastY() + m_right * v.BroadcastZ();
}

D_INLINE dVector dMatrix::UnrotateVector (const dVector &v) const
{
	return dVector ((m_front * v).AddHorizontal().GetScalar(), (m_up * v).AddHorizontal().GetScalar(), (m_right * v).AddHorizontal().GetScalar(), dFloat32 (0.0f));
}

D_INLINE dVector dMatrix::TransformVector (const dVector &v) const
{
	return RotateVector(v) + m_posit;
}

D_INLINE dVector dMatrix::UntransformVector (const dVector &v) const
{
	return UnrotateVector(v - m_posit);
}

D_INLINE dPlane dMatrix::TransformPlane (const dPlane &localPlane) const
{
	return dPlane (RotateVector (localPlane), localPlane.m_w - (localPlane.DotProduct(UnrotateVector (m_posit)).GetScalar()));  
}

D_INLINE dPlane dMatrix::UntransformPlane (const dPlane &globalPlane) const
{
	return dPlane (UnrotateVector (globalPlane), globalPlane.Evalue(m_posit));
}

/*
D_INLINE void dMatrix::EigenVectors ()
{
	dVector eigenValues;
	EigenVectors (eigenValues);
}
*/

D_INLINE dMatrix dMatrix::Inverse () const
{
	// much faster inverse
	dMatrix inv;
	dVector::Transpose4x4 (inv[0], inv[1], inv[2], inv[3], m_front, m_up, m_right, dVector::m_wOne);
	inv.m_posit -= inv[0] * m_posit.BroadcastX() + inv[1] * m_posit.BroadcastY() + inv[2] * m_posit.BroadcastZ();
	return inv;
}

D_INLINE bool dMatrix::TestIdentity() const
{
	const dMatrix& me = *this;
	for (int i = 0; i < 4; i++) 
	{
		if (me[i][i] != dFloat32 (1.0f)) 
		{
			return false;
		}
		for (int j = i + 1; j < 4; j++) 
		{
			if (me[i][j] != dFloat32 (0.0f)) 
			{
				return false;
			}
			if (me[j][i] != dFloat32(0.0f)) 
			{
				return false;
			}
		}
	}
	return true;
}

D_INLINE bool dMatrix::TestOrthogonal(dFloat32 tol) const
{
	dVector n (m_front.CrossProduct(m_up));
	dFloat32 a = m_right.DotProduct(m_right).GetScalar();
	dFloat32 b = m_up.DotProduct(m_up).GetScalar();
	dFloat32 c = m_front.DotProduct(m_front).GetScalar();
	dFloat32 d = n.DotProduct(m_right).GetScalar();

#ifdef _DEBUG
	const dMatrix& me = *this;
	for (int i = 0; i < 4; i++) 
	{
		for (int j = 0; j < 4; j++) 
		{
			dAssert(dCheckFloat(me[i][j]));
		}
	}
#endif

	return (m_front[3] == dFloat32 (0.0f)) & 
		   (m_up[3] == dFloat32 (0.0f)) & 
		   (m_right[3] == dFloat32 (0.0f)) & 
		   (m_posit[3] == dFloat32 (1.0f)) &
		   (dAbs(a - dFloat32 (1.0f)) < tol) & 
		   (dAbs(b - dFloat32 (1.0f)) < tol) &
		   (dAbs(c - dFloat32 (1.0f)) < tol) &
		   (dAbs(d - dFloat32 (1.0f)) < tol); 
}

D_INLINE bool dMatrix::TestSymetric3x3() const
{
	const dMatrix& me = *this;
	return (dAbs (me[0][1] - me[1][0]) < dFloat32 (1.0e-5f)) && 
		   (dAbs (me[0][2] - me[2][0]) < dFloat32 (1.0e-5f)) &&
		   (dAbs (me[1][2] - me[2][1]) < dFloat32 (1.0e-5f)) &&
		   (me[0][3] == dFloat32 (0.0f)) &&
		   (me[1][3] == dFloat32 (0.0f)) &&
		   (me[2][3] == dFloat32 (0.0f)) &&
		   (me[3][0] == dFloat32 (0.0f)) &&
		   (me[3][1] == dFloat32 (0.0f)) &&
		   (me[3][2] == dFloat32 (0.0f)) &&
		   (me[3][3] == dFloat32 (1.0f));
}

D_INLINE dMatrix dPitchMatrix(dFloat32 ang)
{
	dFloat32 sinAng = dSin (ang);
	dFloat32 cosAng = dCos (ang);
	return dMatrix (
		dVector (dFloat32(1.0f),  dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f)), 
		dVector (dFloat32(0.0f),  cosAng,          sinAng,        dFloat32(0.0f)),
		dVector (dFloat32(0.0f), -sinAng,          cosAng,        dFloat32(0.0f)), 
		dVector (dFloat32(0.0f),  dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f))); 
}

D_INLINE dMatrix dYawMatrix(dFloat32 ang)
{
	dFloat32 sinAng = dSin (ang);
	dFloat32 cosAng = dCos (ang);
	return dMatrix (
		dVector (cosAng,          dFloat32(0.0f), -sinAng,        dFloat32(0.0f)), 
		dVector (dFloat32(0.0f), dFloat32(1.0f),  dFloat32(0.0f), dFloat32(0.0f)), 
		dVector (sinAng,          dFloat32(0.0f),  cosAng,        dFloat32(0.0f)), 
		dVector (dFloat32(0.0f), dFloat32(0.0f),  dFloat32(0.0f), dFloat32(1.0f))); 
}

D_INLINE dMatrix dRollMatrix(dFloat32 ang)
{
	dFloat32 sinAng = dSin (ang);
	dFloat32 cosAng = dCos (ang);
	return dMatrix (dVector ( cosAng,          sinAng,          dFloat32(0.0f), dFloat32(0.0f)), 
					dVector (-sinAng,          cosAng,          dFloat32(0.0f), dFloat32(0.0f)),
					dVector ( dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f)), 
					dVector ( dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f))); 
}																		 

#endif

