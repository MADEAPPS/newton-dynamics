/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


#ifndef __dMatrix__
#define __dMatrix__

#include "dVector.h"

class dMatrix ;
class dQuaternion;

// small but fully operational 4x4 matrix class
class dQuaternion;

dMatrix dGetZeroMatrix ();
dMatrix dGetIdentityMatrix();

D_MSC_VECTOR_ALIGMENT
class dMatrix
{
	public:
	dMatrix ();
	dMatrix (const dFloat* const array);
#ifndef _NEWTON_USE_DOUBLE
	dMatrix (const dFloat64* const array);
#endif
	dMatrix (const dVector &front, const dVector &up, const dVector &right, const dVector &posit);
	dMatrix (const dQuaternion &rotation, const dVector &position);
	dMatrix (dFloat pitch, dFloat yaw, dFloat roll, const dVector& location);

	dVector& operator[] (int i);
	const dVector& operator[] (int i) const;

	dMatrix Inverse () const;
	dMatrix Transpose () const;
	dMatrix Transpose4X4 () const;
	
	dVector RotateVector (const dVector &v) const;
	dVector UnrotateVector (const dVector &v) const;
	dVector TransformVector (const dVector &v) const;
	dVector UntransformVector (const dVector &v) const;
	dVector TransformPlane (const dVector &localPlane) const;
	dVector UntransformPlane (const dVector &globalPlane) const;
	//dVector GetEulerAngles(dEulerAngleOrder order = m_pitchYawRoll) const;
    void GetEulerAngles(dVector& euler1, dVector& euler2, dEulerAngleOrder order = m_pitchYawRoll) const;

	bool TestIdentity() const; 
	bool TestOrthogonal() const; 
	dMatrix Inverse4x4 () const;
	dVector RotateVector4x4 (const dVector &v) const;
	dMatrix JacobiDiagonalization (dVector& eigenValues, const dMatrix& initialMatrix = dGetIdentityMatrix()) const;

	// decompose this matrix into [this = transpose(stretchAxis) * matrix(scale) * stretchAxis * transformMatrix];
	void PolarDecomposition (dMatrix& transformMatrix, dVector& scale, dMatrix& stretchAxis, const dMatrix& initialStretchAxis = dGetIdentityMatrix()) const;
	
	// constructor for polar composition
	dMatrix (const dMatrix& transformMatrix, const dVector& scale, const dMatrix& stretchAxis);

	void TransformTriplex (dFloat* const dst, int dstStrideInBytes, const dFloat* const src, int srcStrideInBytes, int count) const;
#ifndef _NEWTON_USE_DOUBLE
	void TransformTriplex (dFloat64* const dst, int dstStrideInBytes, const dFloat64* const src, int srcStrideInBytes, int count) const;
#endif

	dMatrix operator* (const dMatrix & B) const;


	bool SanityCheck() const;

	dVector m_front;
	dVector m_up;
	dVector m_right;
	dVector m_posit;
};



inline dMatrix::dMatrix ()
{
}

inline dMatrix::dMatrix (
	const dVector &front, 
	const dVector &up,
	const dVector &right,
	const dVector &posit)
	:m_front (front), m_up(up), m_right(right), m_posit(posit)
{
}

inline dMatrix::dMatrix (const dFloat* const array)
{
	memcpy (&(*this)[0][0], array, sizeof (dMatrix));
}

#ifndef _NEWTON_USE_DOUBLE
inline dMatrix::dMatrix (const dFloat64* const array)
{
	dFloat* const ptr = &(*this)[0][0];
	for (int i = 0; i < 16; i ++) {
		ptr[i] = dFloat (array[i]);
	}
}
#endif

inline dVector& dMatrix::operator[] (int  i)
{
	return (&m_front)[i];
}

inline const dVector& dMatrix::operator[] (int  i) const
{
	return (&m_front)[i];
}


dMatrix dRollMatrix(dFloat ang);
dMatrix dYawMatrix(dFloat ang);
dMatrix dPitchMatrix(dFloat ang);
dMatrix dGrammSchmidt(const dVector& dir);
#endif

