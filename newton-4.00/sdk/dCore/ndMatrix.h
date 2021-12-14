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

#ifndef __ND_MATRIX_H__
#define __ND_MATRIX_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndDebug.h"
#include "ndPlane.h"
#include "ndVector.h"

class ndMatrix;
class ndQuaternion;

D_CORE_API const ndMatrix& dGetZeroMatrix ();
D_CORE_API const ndMatrix& dGetIdentityMatrix();

D_MSV_NEWTON_ALIGN_32
class ndMatrix
{
	public:
	D_OPERATOR_NEW_AND_DELETE

	ndMatrix ();
	ndMatrix (const ndFloat32* const array);
	ndMatrix (const ndVector &front, const ndVector &up, const ndVector &right, const ndVector &posit);
	D_CORE_API ndMatrix (const ndQuaternion &rotation, const ndVector &position);

	~ndMatrix();

	// create a orthonormal normal vector basis, front become m_front vector, and m_up and m_right are mutualiperpendicular to fron and to each other
	ndMatrix (const ndVector &front);

	// create a covariance Matrix = transpose(p) * q 
	ndMatrix (const ndVector& p, const ndVector& q);

	ndVector& operator[] (ndInt32 i);
	const ndVector& operator[] (ndInt32 i) const;

	ndMatrix Inverse () const;
	D_CORE_API ndMatrix Inverse4x4 () const;
	ndMatrix Transpose () const;
	ndMatrix Transpose4X4 () const;
	ndVector RotateVector (const ndVector &v) const;
	ndVector UnrotateVector (const ndVector &v) const;
	ndVector TransformVector (const ndVector &v) const;
	ndVector UntransformVector (const ndVector &v) const;
	ndPlane TransformPlane (const ndPlane &localPlane) const;
	ndPlane UntransformPlane (const ndPlane &globalPlane) const;
	ndVector TransformVector1x4(const ndVector &v) const;
	D_CORE_API ndVector SolveByGaussianElimination(const ndVector &v) const;
	D_CORE_API void TransformBBox (const ndVector& p0local, const ndVector& p1local, ndVector& p0, ndVector& p1) const;

	D_CORE_API void CalcPitchYawRoll (ndVector& euler0, ndVector& euler1) const;
	D_CORE_API void TransformTriplex (
		ndFloat32* const dst, ndInt32 dstStrideInBytes,
		const ndFloat32* const src, ndInt32 srcStrideInBytes, ndInt32 count) const;

#ifndef D_NEWTON_USE_DOUBLE
	D_CORE_API void TransformTriplex (
		ndFloat64* const dst, ndInt32 dstStrideInBytes,
		const ndFloat64* const src, ndInt32 srcStrideInBytes, ndInt32 count) const;

	D_CORE_API void TransformTriplex (
		ndFloat64* const dst, ndInt32 dstStrideInBytes,
		const ndFloat32* const src, ndInt32 srcStrideInBytes, ndInt32 count) const;
#endif

	bool TestIdentity() const;
	bool TestSymetric3x3() const;
	bool TestOrthogonal(ndFloat32 tol = ndFloat32 (1.0e-4f)) const;

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

	//private:
	//static ndMatrix m_zeroMatrix;
	//static ndMatrix m_identityMatrix;
} D_GCC_NEWTON_ALIGN_32 ;

inline ndMatrix::ndMatrix ()
{
}

inline ndMatrix::ndMatrix (const ndFloat32* const array)
{
	memcpy (&m_front.m_x, array, sizeof (ndMatrix)) ;
}

inline ndMatrix::ndMatrix (const ndVector &front, const ndVector &up, const ndVector &right, const ndVector &posit)
	:m_front (front), m_up(up), m_right(right), m_posit(posit)
{
}

inline ndMatrix::~ndMatrix() 
{
}

inline ndMatrix::ndMatrix (const ndVector& p, const ndVector& q)
	:m_front(q * p.BroadcastX())
	,m_up   (q * p.BroadcastY())
	,m_right(q * p.BroadcastZ())
	,m_posit (ndVector::m_wOne)
{
}

inline ndMatrix::ndMatrix (const ndVector& front)
	:m_front((front & ndVector::m_triplexMask).Normalize())
	,m_posit(ndVector::m_wOne)
{
	if (dAbs(m_front.m_z) > ndFloat32 (0.577f)) 
	{
		m_right = m_front.CrossProduct(ndVector(-m_front.m_y, m_front.m_z, ndFloat32(0.0f), ndFloat32(0.0f)));
	}
	else 
	{
		m_right = m_front.CrossProduct(ndVector(-m_front.m_y, m_front.m_x, ndFloat32(0.0f), ndFloat32(0.0f)));
	}
	m_right = m_right.Normalize();
	m_up = m_right.CrossProduct(m_front);
	dAssert(TestOrthogonal());
}

inline ndVector& ndMatrix::operator[] (ndInt32  i)
{
	dAssert (i < 4);
	dAssert (i >= 0);
	return (&m_front)[i];
}

inline const ndVector& ndMatrix::operator[] (ndInt32  i) const
{
	dAssert (i < 4);
	dAssert (i >= 0);
	return (&m_front)[i];
}

inline ndMatrix ndMatrix::Transpose () const
{
	ndMatrix inv;
	ndVector::Transpose4x4(inv[0], inv[1], inv[2], inv[3], m_front, m_up, m_right, ndVector::m_wOne);
	return inv;
}

inline ndMatrix ndMatrix::Transpose4X4 () const
{
	ndMatrix inv;
	ndVector::Transpose4x4(inv[0], inv[1], inv[2], inv[3], m_front, m_up, m_right, m_posit);
	return inv;
}

inline ndVector ndMatrix::RotateVector (const ndVector &v) const
{
	return m_front * v.BroadcastX() + m_up * v.BroadcastY() + m_right * v.BroadcastZ();
}

inline ndVector ndMatrix::UnrotateVector (const ndVector &v) const
{
	return ndVector ((m_front * v).AddHorizontal().GetScalar(), (m_up * v).AddHorizontal().GetScalar(), (m_right * v).AddHorizontal().GetScalar(), ndFloat32 (0.0f));
}

inline ndVector ndMatrix::TransformVector (const ndVector &v) const
{
	return RotateVector(v) + m_posit;
}

inline ndVector ndMatrix::TransformVector1x4(const ndVector &v) const
{
	return m_front * v.BroadcastX() + m_up * v.BroadcastY() +
		   m_right * v.BroadcastZ() + m_posit * v.BroadcastW();
}

inline ndVector ndMatrix::UntransformVector (const ndVector &v) const
{
	return UnrotateVector(v - m_posit) | ndVector::m_wOne;
}

inline ndPlane ndMatrix::TransformPlane (const ndPlane &localPlane) const
{
	return ndPlane (RotateVector (localPlane), localPlane.m_w - (localPlane.DotProduct(UnrotateVector (m_posit)).GetScalar()));  
}

inline ndPlane ndMatrix::UntransformPlane (const ndPlane &globalPlane) const
{
	return ndPlane (UnrotateVector (globalPlane), globalPlane.Evalue(m_posit));
}

/*
inline void ndMatrix::EigenVectors ()
{
	ndVector eigenValues;
	EigenVectors (eigenValues);
}
*/

inline ndMatrix ndMatrix::Inverse () const
{
	// much faster inverse
	ndMatrix inv;
	ndVector::Transpose4x4 (inv[0], inv[1], inv[2], inv[3], m_front, m_up, m_right, ndVector::m_wOne);
	inv.m_posit -= inv[0] * m_posit.BroadcastX() + inv[1] * m_posit.BroadcastY() + inv[2] * m_posit.BroadcastZ();
	return inv;
}

inline bool ndMatrix::TestIdentity() const
{
	const ndMatrix& me = *this;
	for (ndInt32 i = 0; i < 4; i++) 
	{
		if (me[i][i] != ndFloat32 (1.0f)) 
		{
			return false;
		}
		for (ndInt32 j = i + 1; j < 4; j++) 
		{
			if (me[i][j] != ndFloat32 (0.0f)) 
			{
				return false;
			}
			if (me[j][i] != ndFloat32(0.0f)) 
			{
				return false;
			}
		}
	}
	return true;
}

inline bool ndMatrix::TestOrthogonal(ndFloat32 tol) const
{
	#ifdef _DEBUG
		const ndMatrix& me = *this;
		for (ndInt32 i = 0; i < 4; i++)
		{
			for (ndInt32 j = 0; j < 4; j++)
			{
				dAssert(dCheckFloat(me[i][j]));
			}
		}
	#endif

	ndVector n (m_front.CrossProduct(m_up));
	ndFloat32 a = m_right.DotProduct(m_right).GetScalar();
	ndFloat32 b = m_up.DotProduct(m_up).GetScalar();
	ndFloat32 c = m_front.DotProduct(m_front).GetScalar();
	ndFloat32 d = n.DotProduct(m_right).GetScalar();
	bool ret =  (m_front[3] == ndFloat32(0.0f)) &&
			    (m_up[3] == ndFloat32(0.0f)) &&
				(m_right[3] == ndFloat32(0.0f)) &&
				(m_posit[3] == ndFloat32(1.0f)) &&
				(dAbs(a - ndFloat32(1.0f)) < tol) &&
				(dAbs(b - ndFloat32(1.0f)) < tol) &&
				(dAbs(c - ndFloat32(1.0f)) < tol) &&
				(dAbs(d - ndFloat32(1.0f)) < tol);
	if (!ret)
	{
		dAssert (0);
	}
	return ret;
}

inline bool ndMatrix::TestSymetric3x3() const
{
	const ndMatrix& me = *this;
	return (dAbs (me[0][1] - me[1][0]) < ndFloat32 (1.0e-5f)) && 
		   (dAbs (me[0][2] - me[2][0]) < ndFloat32 (1.0e-5f)) &&
		   (dAbs (me[1][2] - me[2][1]) < ndFloat32 (1.0e-5f)) &&
		   (me[0][3] == ndFloat32 (0.0f)) &&
		   (me[1][3] == ndFloat32 (0.0f)) &&
		   (me[2][3] == ndFloat32 (0.0f)) &&
		   (me[3][0] == ndFloat32 (0.0f)) &&
		   (me[3][1] == ndFloat32 (0.0f)) &&
		   (me[3][2] == ndFloat32 (0.0f)) &&
		   (me[3][3] == ndFloat32 (1.0f));
}

inline ndMatrix dPitchMatrix(ndFloat32 ang)
{
	ndFloat32 sinAng = ndSin (ang);
	ndFloat32 cosAng = ndCos (ang);
	return ndMatrix (
		ndVector (ndFloat32(1.0f),  ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f)), 
		ndVector (ndFloat32(0.0f),  cosAng,          sinAng,        ndFloat32(0.0f)),
		ndVector (ndFloat32(0.0f), -sinAng,          cosAng,        ndFloat32(0.0f)), 
		ndVector (ndFloat32(0.0f),  ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f))); 
}

inline ndMatrix dYawMatrix(ndFloat32 ang)
{
	ndFloat32 sinAng = ndSin (ang);
	ndFloat32 cosAng = ndCos (ang);
	return ndMatrix (
		ndVector (cosAng,          ndFloat32(0.0f), -sinAng,        ndFloat32(0.0f)), 
		ndVector (ndFloat32(0.0f), ndFloat32(1.0f),  ndFloat32(0.0f), ndFloat32(0.0f)), 
		ndVector (sinAng,          ndFloat32(0.0f),  cosAng,        ndFloat32(0.0f)), 
		ndVector (ndFloat32(0.0f), ndFloat32(0.0f),  ndFloat32(0.0f), ndFloat32(1.0f))); 
}

inline ndMatrix dRollMatrix(ndFloat32 ang)
{
	ndFloat32 sinAng = ndSin (ang);
	ndFloat32 cosAng = ndCos (ang);
	return ndMatrix (ndVector ( cosAng,          sinAng,          ndFloat32(0.0f), ndFloat32(0.0f)), 
					ndVector (-sinAng,          cosAng,          ndFloat32(0.0f), ndFloat32(0.0f)),
					ndVector ( ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f)), 
					ndVector ( ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f))); 
}																		 

#endif

