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

#include "dgStdafx.h"
#include "dgMatrix.h"
#include "dgQuaternion.h"


dgVector dgVector::m_xMask (0xffffffff, 0, 0, 0);
dgVector dgVector::m_yMask (0,          0xffffffff, 0, 0);
dgVector dgVector::m_zMask (0,          0,          0xffffffff, 0);
dgVector dgVector::m_wMask (0,          0,          0,          0xffffffff);
dgVector dgVector::m_triplexMask (0xffffffff, 0xffffffff, 0xffffffff, 0);
dgVector dgVector::m_signMask (0x7fffffff, 0x7fffffff, 0x7fffffff, 0x7fffffff);

dgVector dgVector::m_zero (dgFloat32 (0.0f));
dgVector dgVector::m_one  (dgFloat32 (1.0f));
dgVector dgVector::m_two  (dgFloat32 (2.0f));
dgVector dgVector::m_wOne (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (1.0f));
dgVector dgVector::m_half (dgFloat32 (0.5f));
dgVector dgVector::m_three (dgFloat32 (3.0f));
dgVector dgVector::m_negOne (dgFloat32 (-1.0f));

dgMatrix dgMatrix::m_zeroMatrix (dgVector (dgFloat32(0.0f)),
								 dgVector (dgFloat32(0.0f)),
								 dgVector (dgFloat32(0.0f)),
								 dgVector (dgFloat32(0.0f)));

dgMatrix dgMatrix::m_identityMatrix (dgVector (dgFloat32(1.0f), dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f)),
									 dgVector (dgFloat32(0.0f), dgFloat32(1.0f), dgFloat32(0.0f), dgFloat32(0.0f)),
									 dgVector (dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(1.0f), dgFloat32(0.0f)),
									 dgVector (dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(1.0f)));

const dgMatrix& dgGetIdentityMatrix()
{
	return dgMatrix::m_identityMatrix;
}

const dgMatrix& dgGetZeroMatrix ()
{
	return dgMatrix::m_zeroMatrix;
}


dgMatrix::dgMatrix (const dgQuaternion &rotation, const dgVector &position)
{
	dgFloat32 x2 = dgFloat32 (2.0f) * rotation.m_q1 * rotation.m_q1;
	dgFloat32 y2 = dgFloat32 (2.0f) * rotation.m_q2 * rotation.m_q2;
	dgFloat32 z2 = dgFloat32 (2.0f) * rotation.m_q3 * rotation.m_q3;

#ifdef _DEBUG
	dgFloat32 w2 = dgFloat32 (2.0f) * rotation.m_q0 * rotation.m_q0;
	dgAssert (dgAbsf (w2 + x2 + y2 + z2 - dgFloat32(2.0f)) <dgFloat32 (1.0e-3f));
#endif

	dgFloat32 xy = dgFloat32 (2.0f) * rotation.m_q1 * rotation.m_q2;
	dgFloat32 xz = dgFloat32 (2.0f) * rotation.m_q1 * rotation.m_q3;
	dgFloat32 xw = dgFloat32 (2.0f) * rotation.m_q1 * rotation.m_q0;
	dgFloat32 yz = dgFloat32 (2.0f) * rotation.m_q2 * rotation.m_q3;
	dgFloat32 yw = dgFloat32 (2.0f) * rotation.m_q2 * rotation.m_q0;
	dgFloat32 zw = dgFloat32 (2.0f) * rotation.m_q3 * rotation.m_q0;

	m_front = dgVector (dgFloat32(1.0f) - y2 - z2, xy + zw,                   xz - yw				    , dgFloat32(0.0f));
	m_up    = dgVector (xy - zw,                   dgFloat32(1.0f) - x2 - z2, yz + xw					, dgFloat32(0.0f));
	m_right = dgVector (xz + yw,                   yz - xw,                   dgFloat32(1.0f) - x2 - y2 , dgFloat32(0.0f));


	m_posit.m_x = position.m_x;
	m_posit.m_y = position.m_y;
	m_posit.m_z = position.m_z;
	m_posit.m_w = dgFloat32(1.0f);
}


dgMatrix::dgMatrix (const dgMatrix& transformMatrix, const dgVector& scale, const dgMatrix& stretchAxis)
{
	dgMatrix scaledAxis;
	scaledAxis[0] = stretchAxis[0].Scale4 (scale[0]);
	scaledAxis[1] = stretchAxis[1].Scale4 (scale[1]);
	scaledAxis[2] = stretchAxis[2].Scale4 (scale[2]);
	scaledAxis[3] = stretchAxis[3];

	*this = stretchAxis.Transpose() * scaledAxis * transformMatrix;
}

dgMatrix dgMatrix::Multiply3X3 (const dgMatrix &B) const
{
	return dgMatrix (B.m_front.CompProduct4(m_front.BroadcastX()) + B.m_up.CompProduct4(m_front.BroadcastY()) + B.m_right.CompProduct4(m_front.BroadcastZ()), 
					 B.m_front.CompProduct4(m_up.BroadcastX())    + B.m_up.CompProduct4(m_up.BroadcastY())    + B.m_right.CompProduct4(m_up.BroadcastZ()), 
					 B.m_front.CompProduct4(m_right.BroadcastX()) + B.m_up.CompProduct4(m_right.BroadcastY()) + B.m_right.CompProduct4(m_right.BroadcastZ()), 
					 dgVector::m_wOne); 
}

dgMatrix dgMatrix::operator* (const dgMatrix &B) const
{
#if 0
	return dgMatrix (B.m_front.Scale4(m_front.m_x) + B.m_up.Scale4(m_front.m_y) + B.m_right.Scale4(m_front.m_z) + B.m_posit.Scale4 (m_front.m_w), 
					 B.m_front.Scale4(m_up.m_x)    + B.m_up.Scale4(m_up.m_y)    + B.m_right.Scale4(m_up.m_z)    + B.m_posit.Scale4 (m_up.m_w), 
					 B.m_front.Scale4(m_right.m_x) + B.m_up.Scale4(m_right.m_y) + B.m_right.Scale4(m_right.m_z) + B.m_posit.Scale4 (m_right.m_w), 
					 B.m_front.Scale4(m_posit.m_x) + B.m_up.Scale4(m_posit.m_y) + B.m_right.Scale4(m_posit.m_z) + B.m_posit.Scale4 (m_posit.m_w)); 
#else
	return dgMatrix (B.m_front.CompProduct4(m_front.BroadcastX()) + B.m_up.CompProduct4(m_front.BroadcastY()) + B.m_right.CompProduct4(m_front.BroadcastZ()) + B.m_posit.CompProduct4 (m_front.BroadcastW()), 
					 B.m_front.CompProduct4(m_up.BroadcastX())    + B.m_up.CompProduct4(m_up.BroadcastY())    + B.m_right.CompProduct4(m_up.BroadcastZ())    + B.m_posit.CompProduct4 (m_up.BroadcastW()), 
					 B.m_front.CompProduct4(m_right.BroadcastX()) + B.m_up.CompProduct4(m_right.BroadcastY()) + B.m_right.CompProduct4(m_right.BroadcastZ()) + B.m_posit.CompProduct4 (m_right.BroadcastW()), 
					 B.m_front.CompProduct4(m_posit.BroadcastX()) + B.m_up.CompProduct4(m_posit.BroadcastY()) + B.m_right.CompProduct4(m_posit.BroadcastZ()) + B.m_posit.CompProduct4 (m_posit.BroadcastW())); 
#endif
}



void dgMatrix::TransformTriplex (dgFloat32* const dst, dgInt32 dstStrideInBytes, const dgFloat32* const src, dgInt32 srcStrideInBytes, dgInt32 count) const
{
	dgInt32 dstStride = dstStrideInBytes /sizeof (dgFloat32);
	dgInt32 srcStride = srcStrideInBytes / sizeof (dgFloat32);

	dgInt32 dstIndex = 0;
	dgInt32 srcIndex = 0;
	for (dgInt32 i = 0 ; i < count; i ++ ) {
		dgFloat32 x = src[srcIndex + 0];
		dgFloat32 y = src[srcIndex + 1];
		dgFloat32 z = src[srcIndex + 2];
		srcIndex += srcStride;
		dst[dstIndex + 0] = x * m_front.m_x + y * m_up.m_x + z * m_right.m_x + m_posit.m_x;
		dst[dstIndex + 1] = x * m_front.m_y + y * m_up.m_y + z * m_right.m_y + m_posit.m_y;
		dst[dstIndex + 2] = x * m_front.m_z + y * m_up.m_z + z * m_right.m_z + m_posit.m_z;
		dstIndex += dstStride;
	}
}

#ifndef _NEWTON_USE_DOUBLE
void dgMatrix::TransformTriplex (dgFloat64* const dst, dgInt32 dstStrideInBytes, const dgFloat64* const src, dgInt32 srcStrideInBytes, dgInt32 count) const
{
	dgInt32 dstStride = dstStrideInBytes /sizeof (dgFloat64);
	dgInt32 srcStride = srcStrideInBytes / sizeof (dgFloat64);

	dgInt32 dstIndex = 0;
	dgInt32 srcIndex = 0;
	for (dgInt32 i = 0 ; i < count; i ++ ) {
		dgFloat64 x = src[srcIndex + 0];
		dgFloat64 y = src[srcIndex + 1];
		dgFloat64 z = src[srcIndex + 2];
		srcIndex += srcStride;
		dst[dstIndex + 0] = x * m_front.m_x + y * m_up.m_x + z * m_right.m_x + m_posit.m_x;
		dst[dstIndex + 1] = x * m_front.m_y + y * m_up.m_y + z * m_right.m_y + m_posit.m_y;
		dst[dstIndex + 2] = x * m_front.m_z + y * m_up.m_z + z * m_right.m_z + m_posit.m_z;
		dstIndex += dstStride;
	}
}

void dgMatrix::TransformTriplex (dgFloat64* const dst, dgInt32 dstStrideInBytes, const dgFloat32* const src, dgInt32 srcStrideInBytes, dgInt32 count) const
{
	dgInt32 dstStride = dstStrideInBytes /sizeof (dgFloat64);
	dgInt32 srcStride = srcStrideInBytes / sizeof (dgFloat32);

	dgInt32 dstIndex = 0;
	dgInt32 srcIndex = 0;
	for (dgInt32 i = 0 ; i < count; i ++ ) {
		dgFloat64 x = src[srcIndex + 0];
		dgFloat64 y = src[srcIndex + 1];
		dgFloat64 z = src[srcIndex + 2];
		srcIndex += srcStride;
		dst[dstIndex + 0] = x * m_front.m_x + y * m_up.m_x + z * m_right.m_x + m_posit.m_x;
		dst[dstIndex + 1] = x * m_front.m_y + y * m_up.m_y + z * m_right.m_y + m_posit.m_y;
		dst[dstIndex + 2] = x * m_front.m_z + y * m_up.m_z + z * m_right.m_z + m_posit.m_z;
		dstIndex += dstStride;
	}
}
#endif

void dgMatrix::TransformBBox (const dgVector& p0local, const dgVector& p1local, dgVector& p0, dgVector& p1) const
{
	const dgMatrix& matrix = *this;
	dgVector size ((p1local - p0local).Scale3 (dgFloat32 (0.5f)));
	dgVector center (TransformVector ((p1local + p0local).Scale3 (dgFloat32 (0.5f))));
	dgVector extends (size.m_x * dgAbsf(matrix[0][0]) + size.m_y * dgAbsf(matrix[1][0]) + size.m_z * dgAbsf(matrix[2][0]),  
					  size.m_x * dgAbsf(matrix[0][1]) + size.m_y * dgAbsf(matrix[1][1]) + size.m_z * dgAbsf(matrix[2][1]),  
	                  size.m_x * dgAbsf(matrix[0][2]) + size.m_y * dgAbsf(matrix[1][2]) + size.m_z * dgAbsf(matrix[2][2]), dgFloat32 (0.0f));  

	p0 = center - extends;
	p1 = center + extends;

}

dgMatrix dgMatrix::Inverse4x4 () const
{
	const dgFloat32 tol = 1.0e-4f;
	dgMatrix tmp (*this);
	dgMatrix inv (dgGetIdentityMatrix());
	for (dgInt32 i = 0; i < 4; i ++) {
		dgFloat32 diag = tmp[i][i];
		if (dgAbsf (diag) < tol) {
			dgInt32 j = 0;
			for (j = i + 1; j < 4; j ++) {
				dgFloat32 val = tmp[j][i];
				if (dgAbsf (val) > tol) {
					break;
				}
			}
			dgAssert (j < 4);
			for (dgInt32 k = 0; k < 4; k ++) {
				tmp[i][k] += tmp[j][k];
				inv[i][k] += inv[j][k];
			}
			diag = tmp[i][i];
		}
		dgFloat32 invDiag = dgFloat32 (1.0f) / diag;
		for (dgInt32 j = 0; j < 4; j ++) {
			tmp[i][j] *= invDiag;
			inv[i][j] *= invDiag;
		}
		tmp[i][i] = dgFloat32 (1.0f);

		for (dgInt32 j = 0; j < 4; j ++) {
			if (j != i) {
				dgFloat32 pivot = tmp[j][i];
				for (dgInt32 k = 0; k < 4; k ++) {
					tmp[j][k] -= pivot * tmp[i][k];
					inv[j][k] -= pivot * inv[i][k];
				}
				tmp[j][i] = dgFloat32 (0.0f);
			}
		}
	}
	return inv;
}

dgMatrix dgMatrix::Symetric3by3Inverse () const
{
	dgMatrix copy(*this);
	dgMatrix inverse(dgGetIdentityMatrix());
	for (dgInt32 i = 0; i < 3; i++) {
		dgVector den(dgFloat32(1.0f) / copy[i][i]);
		copy[i] = copy[i].CompProduct4(den);
		inverse[i] = inverse[i].CompProduct4(den);
		for (dgInt32 j = 0; j < 3; j++) {
			if (j != i) {
				dgVector pivot(copy[j][i]);
				copy[j] -= copy[i].CompProduct4(pivot);
				inverse[j] -= inverse[i].CompProduct4(pivot);
			}
		}
	}

#ifdef _DEBUG
	dgMatrix test(*this * inverse);
	dgAssert (dgAbsf (test[0][0] - dgFloat32(1.0f)) < dgFloat32(0.01f));
	dgAssert (dgAbsf (test[1][1] - dgFloat32(1.0f)) < dgFloat32(0.01f));
	dgAssert (dgAbsf (test[2][2] - dgFloat32(1.0f)) < dgFloat32(0.01f));
#endif

	return inverse;
}





void dgMatrix::CalcPitchYawRoll (dgVector& euler0, dgVector& euler1) const
{
	const dgMatrix& matrix = *this;
	dgAssert (dgAbsf (((matrix[0] * matrix[1]) % matrix[2]) - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));

	// Assuming the angles are in radians.
	if (matrix[0][2] > dgFloat32 (0.99995f)) {
		dgFloat32 picth0 = dgFloat32 (0.0f);
		dgFloat32 yaw0 = dgFloat32 (-3.141592f * 0.5f);
		dgFloat32 roll0 = - dgAtan2(matrix[2][1], matrix[1][1]);
		euler0[0] = picth0;
		euler0[1] = yaw0;
		euler0[2] = roll0;

		euler1[0] = picth0;
		euler1[1] = yaw0;
		euler1[2] = roll0;

	} else if (matrix[0][2] < dgFloat32 (-0.99995f)) {
		dgFloat32 picth0 = dgFloat32 (0.0f);
		dgFloat32 yaw0 = dgFloat32 (3.141592f * 0.5f);
		dgFloat32 roll0 = dgAtan2(matrix[2][1], matrix[1][1]);
		euler0[0] = picth0;
		euler0[1] = yaw0;
		euler0[2] = roll0;

		euler1[0] = picth0;
		euler1[1] = yaw0;
		euler1[2] = roll0;
	} else {
		dgFloat32 yaw0 = -dgAsin ( matrix[0][2]);
		dgFloat32 yaw1 = dgFloat32 (3.141592f) - yaw0;
		dgFloat32 sign0 = dgSign(dgCos (yaw0));
		dgFloat32 sign1 = dgSign(dgCos (yaw1));

		dgFloat32 picth0 = dgAtan2(matrix[1][2] * sign0, matrix[2][2] * sign0);
		dgFloat32 picth1 = dgAtan2(matrix[1][2] * sign1, matrix[2][2] * sign1);

		dgFloat32 roll0 = dgAtan2(matrix[0][1] * sign0, matrix[0][0] * sign0);
		dgFloat32 roll1 = dgAtan2(matrix[0][1] * sign1, matrix[0][0] * sign1);

		if (yaw1 > dgFloat32 (3.141592f)) {
			yaw1 -= dgFloat32 (2.0f * 3.141592f);
		}

		euler0[0] = picth0;
		euler0[1] = yaw0;
		euler0[2] = roll0;

		euler1[0] = picth1;
		euler1[1] = yaw1;
		euler1[2] = roll1;
	}
	euler0[3] = dgFloat32(0.0f);
	euler1[3] = dgFloat32(0.0f);

#ifdef _DEBUG
	dgMatrix m0 (dgPitchMatrix (euler0[0]) * dgYawMatrix(euler0[1]) * dgRollMatrix(euler0[2]));
	dgMatrix m1 (dgPitchMatrix (euler1[0]) * dgYawMatrix(euler1[1]) * dgRollMatrix(euler1[2]));
	for (int i = 0; i < 3; i ++) {
		for (int j = 0; j < 3; j ++) {
			dgFloat32 error = dgAbsf (m0[i][j] - matrix[i][j]);
			dgAssert (error < 5.0e-2f);
			error = dgAbsf (m1[i][j] - matrix[i][j]);
			dgAssert (error < 5.0e-2f);
		}
	}
#endif
}



void dgMatrix::PolarDecomposition (dgMatrix& transformMatrix, dgVector& scale, dgMatrix& stretchAxis, const dgMatrix& initialStretchAxis) const
{
	// a polar decomposition decompose matrix A = O * S
	// where S = sqrt (transpose (L) * L)

/*
	// calculate transpose (L) * L 
	dgMatrix LL ((*this) * Transpose());

	// check is this is a pure uniformScale * rotation * translation
	dgFloat32 det2 = (LL[0][0] + LL[1][1] + LL[2][2]) * dgFloat32 (1.0f / 3.0f);

	dgFloat32 invdet2 = 1.0f / det2;

	dgMatrix pureRotation (LL);
	pureRotation[0] = pureRotation[0].Scale3 (invdet2);
	pureRotation[1] = pureRotation[1].Scale3 (invdet2);
	pureRotation[2] = pureRotation[2].Scale3 (invdet2);

	dgFloat32 sign = ((((*this)[0] * (*this)[1]) % (*this)[2]) > 0.0f) ? 1.0f : -1.0f;
	dgFloat32 det = (pureRotation[0] * pureRotation[1]) % pureRotation[2];
	if (dgAbsf (det - dgFloat32 (1.0f)) < dgFloat32 (1.0e-5f)) {
		// this is a pure scale * rotation * translation
		det = sign * dgSqrt (det2);
		scale[0] = det;
		scale[1] = det;
		scale[2] = det;
		det = dgFloat32 (1.0f)/ det;
		transformMatrix.m_front = m_front.Scale3 (det);
		transformMatrix.m_up = m_up.Scale3 (det);
		transformMatrix.m_right = m_right.Scale3 (det);
		transformMatrix[0][3] = dgFloat32 (0.0f);
		transformMatrix[1][3] = dgFloat32 (0.0f);
		transformMatrix[2][3] = dgFloat32 (0.0f);
		transformMatrix.m_posit = m_posit;
		stretchAxis = dgGetIdentityMatrix();

	} else {
		stretchAxis = LL;
		stretchAxis.EigenVectors (scale);

		// I need to deal with by seeing of some of the Scale are duplicated
		// do this later (maybe by a given rotation around the non uniform axis but I do not know if it will work)
		// for now just us the matrix

		scale[0] = sign * dgSqrt (scale[0]);
		scale[1] = sign * dgSqrt (scale[1]);
		scale[2] = sign * dgSqrt (scale[2]);
		scale[3] = dgFloat32 (0.0f);

		dgMatrix scaledAxis;
		scaledAxis[0] = stretchAxis[0].Scale3 (dgFloat32 (1.0f) / scale[0]);
		scaledAxis[1] = stretchAxis[1].Scale3 (dgFloat32 (1.0f) / scale[1]);
		scaledAxis[2] = stretchAxis[2].Scale3 (dgFloat32 (1.0f) / scale[2]);
		scaledAxis[3] = stretchAxis[3];
		dgMatrix symetricInv (stretchAxis.Transpose() * scaledAxis);

		transformMatrix = symetricInv * (*this);
		transformMatrix.m_posit = m_posit;
	}
*/

	//dgFloat32 sign = ((((*this)[0] * (*this)[1]) % (*this)[2]) > 0.0f) ? 1.0f : -1.0f;
	dgFloat32 sign = dgSign(((*this)[0] * (*this)[1]) % (*this)[2]);
	stretchAxis = (*this) * Transpose();
	stretchAxis.EigenVectors (scale);

	// I need to deal with by seeing of some of the Scale are duplicated
	// do this later (maybe by a given rotation around the non uniform axis but I do not know if it will work)
	// for now just us the matrix

	scale[0] = sign * dgSqrt (scale[0]);
	scale[1] = sign * dgSqrt (scale[1]);
	scale[2] = sign * dgSqrt (scale[2]);
	scale[3] = dgFloat32 (0.0f);

	dgMatrix scaledAxis;
	scaledAxis[0] = stretchAxis[0].Scale3 (dgFloat32 (1.0f) / scale[0]);
	scaledAxis[1] = stretchAxis[1].Scale3 (dgFloat32 (1.0f) / scale[1]);
	scaledAxis[2] = stretchAxis[2].Scale3 (dgFloat32 (1.0f) / scale[2]);
	scaledAxis[3] = stretchAxis[3];
	dgMatrix symetricInv (stretchAxis.Transpose() * scaledAxis);

	transformMatrix = symetricInv * (*this);
	transformMatrix.m_posit = m_posit;

}


class dgEigenVectorMatrix: public dgMatrix
{
	void HouseholderFactorization()
	{
		dgMatrix& me = *this;
		m_eigenValues[0] = me[0][0];
		m_offDiagonal[2] = dgFloat32 (0.0f);
		if (dgAbsf (me[0][2]) > dgFloat32 (1.0e-6f)) {
			dgFloat32 a01 = me[0][1];
			dgFloat32 a02 = me[0][2];

			dgFloat32 mag = dgSqrt(a01 * a01 + a02 * a02);
			dgFloat32 invMag = (1.0f) / mag;
			a01 *= invMag;
			a02 *= invMag;

			dgFloat32 q = dgFloat32 (2.0f) * a01 * me[1][2] + a02 * (me[2][2] - me[1][1]);

			m_offDiagonal[0] = mag;
			m_offDiagonal[1] = me[1][2] - a01 * q;
			m_eigenValues[1] = me[1][1] + a02 * q;
			m_eigenValues[2] = me[2][2] - a02 * q;

			me[0] = dgVector (dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)); 
			me[1] = dgVector (dgFloat32 (0.0f), a01,  a02, dgFloat32 (0.0f)); 
			me[2] = dgVector (dgFloat32 (0.0f), a02, -a01, dgFloat32 (0.0f)); 
		} else {
			m_eigenValues[1] = me[1][1];
			m_eigenValues[2] = me[2][2];
			m_offDiagonal[0] = me[0][1];
			m_offDiagonal[1] = me[1][2];
			me = dgGetIdentityMatrix();
		}
	}

	bool QLFactorizatiuon(void)
	{
		const int iMaxIter = 32;

		for (int i0 = 0; i0 <3; i0++)
		{
			int i1;
			for (i1 = 0; i1 < iMaxIter; i1++)
			{
				int i2;
				for (i2 = i0; i2 <= (3-2); i2++)
				{
					dgFloat32 fTmp = fabsf(m_afDiag[i2]) + fabsf(m_afDiag[i2+1]);
					if ( fabsf(m_afSubd[i2]) + fTmp == fTmp )
						break;
				}
				if (i2 == i0)
				{
					break;
				}

				dgFloat32 fG = (m_afDiag[i0+1] - m_afDiag[i0])/((2.0f) * m_afSubd[i0]);
				dgFloat32 fR = sqrtf(fG*fG+1.0f);
				if (fG < 0.0f)
				{
					fG = m_afDiag[i2]-m_afDiag[i0]+m_afSubd[i0]/(fG-fR);
				}
				else
				{
					fG = m_afDiag[i2]-m_afDiag[i0]+m_afSubd[i0]/(fG+fR);
				}
				dgFloat32 fSin = 1.0f, fCos = 1.0f, fP = 0.0f;
				for (int i3 = i2-1; i3 >= i0; i3--)
				{
					dgFloat32 fF = fSin*m_afSubd[i3];
					dgFloat32 fB = fCos*m_afSubd[i3];
					if (fabsf(fF) >= fabsf(fG))
					{
						fCos = fG/fF;
						fR = sqrtf(fCos*fCos+1.0f);
						m_afSubd[i3+1] = fF*fR;
						fSin = (1.0f)/fR;
						fCos *= fSin;
					}
					else
					{
						fSin = fF/fG;
						fR = sqrtf(fSin*fSin+1.0f);
						m_afSubd[i3+1] = fG*fR;
						fCos = (1.0f)/fR;
						fSin *= fCos;
					}
					fG = m_afDiag[i3+1]-fP;
					fR = (m_afDiag[i3]-fG)*fSin+(2.0f)*fB*fCos;
					fP = fSin*fR;
					m_afDiag[i3+1] = fG+fP;
					fG = fCos*fR-fB;
					for (int i4 = 0; i4 < 3; i4++)
					{
						fF = mElement[i4][i3+1];
						mElement[i4][i3+1] = fSin*mElement[i4][i3]+fCos*fF;
						mElement[i4][i3] = fCos*mElement[i4][i3]-fSin*fF;
					}
				}
				m_afDiag[i0] -= fP;
				m_afSubd[i0] = fG;
				m_afSubd[i2] = 0.0f;
			}
			if (i1 == iMaxIter)
			{
				return false;
			}
		}
		return true;
	}

	public:
	dgEigenVectorMatrix(const dgMatrix& matrix)
		:dgMatrix (matrix)
	{
		HouseholderFactorization(); 

		dgMatrix xxx (dgGetIdentityMatrix());
		xxx[0][0] = m_eigenValues[0];
		xxx[1][1] = m_eigenValues[1];
		xxx[2][2] = m_eigenValues[2];

		xxx[0][1] = m_offDiagonal[0];
		xxx[1][0] = m_offDiagonal[0];
		xxx[1][2] = m_offDiagonal[1];
		xxx[2][1] = m_offDiagonal[1];

		dgMatrix& me = *this;
		dgMatrix xxx1 (me.Inverse() * xxx * me);
		dgMatrix xxx2 (me * xxx * me.Inverse());
		dgMatrix xxx3 (me * xxx * me.Inverse());

//		QLFactorizatiuon();
	}

	private:
	dgVector m_eigenValues;
	dgVector m_offDiagonal;
	dgFloat32 mElement[3][3];
	dgFloat32 m_afDiag[3];
	dgFloat32 m_afSubd[3];
};


void dgMatrix::EigenVectors (dgVector &eigenValues, const dgMatrix& initialGuess)
{
	dgEigenVectorMatrix(*this);

	dgFloat32 b[3];
	dgFloat32 z[3];
	dgFloat32 d[3];

	dgMatrix& mat = *this;
	dgMatrix eigenVectors (initialGuess.Transpose4X4());
	mat = initialGuess * mat * eigenVectors;

	b[0] = mat[0][0]; 
	b[1] = mat[1][1];
	b[2] = mat[2][2];

	d[0] = mat[0][0]; 
	d[1] = mat[1][1]; 
	d[2] = mat[2][2]; 

	z[0] = dgFloat32 (0.0f);
	z[1] = dgFloat32 (0.0f);
	z[2] = dgFloat32 (0.0f);

	for (dgInt32 i = 0; i < 50; i++) {
		dgFloat32 sm = dgAbsf(mat[0][1]) + dgAbsf(mat[0][2]) + dgAbsf(mat[1][2]);

		if (sm < dgFloat32 (1.0e-12f)) {
			//dgAssert (dgAbsf((eigenVectors.m_front % eigenVectors.m_front) - dgFloat32(1.0f)) < dgEPSILON);
			//dgAssert (dgAbsf((eigenVectors.m_up % eigenVectors.m_up) - dgFloat32(1.0f)) < dgEPSILON);
			//dgAssert (dgAbsf((eigenVectors.m_right % eigenVectors.m_right) - dgFloat32(1.0f)) < dgEPSILON);

			// order the eigenvalue vectors	
			dgVector tmp (eigenVectors.m_front * eigenVectors.m_up);
			if (tmp % eigenVectors.m_right < dgFloat32(0.0f)) {
				dgAssert (0.0f);
				eigenVectors.m_right = eigenVectors.m_right.Scale3 (-dgFloat32(1.0f));
			}

			//eigenValues = dgVector (d[0], d[1], d[2], dgFloat32 (1.0f));
			//*this = eigenVectors.Inverse();
			//return;
			break;
		}

		dgFloat32 thresh = dgFloat32 (0.0f);
		if (i < 3) {
			thresh = (dgFloat32)(0.2f / 9.0f) * sm;
		}

		for (dgInt32 ip = 0; ip < 2; ip ++) {
			for (dgInt32 iq = ip + 1; iq < 3; iq ++) {
				dgFloat32 g = dgFloat32 (100.0f) * dgAbsf(mat[ip][iq]);
				if ((i > 3) && ((dgAbsf(d[ip]) + g) == dgAbsf(d[ip])) && ((dgAbsf(d[iq]) + g) == dgAbsf(d[iq]))) {
					mat[ip][iq] = dgFloat32 (0.0f);
				} else if (dgAbsf(mat[ip][iq]) > thresh) {

					dgFloat32 t;
					dgFloat32 h = d[iq] - d[ip];
					if (dgAbsf(h) + g == dgAbsf(h)) {
						t = mat[ip][iq] / h;
					} else {
						dgFloat32 theta = dgFloat32 (0.5f) * h / mat[ip][iq];
						t = dgFloat32(1.0f) / (dgAbsf(theta) + dgSqrt(dgFloat32(1.0f) + theta * theta));
						if (theta < dgFloat32 (0.0f)) {
							t = -t;
						}
					}
					dgFloat32 c = dgRsqrt (dgFloat32 (1.0f) + t * t); 
					dgFloat32 s = t * c; 
					dgFloat32 tau = s / (dgFloat32(1.0f) + c); 
					h = t * mat[ip][iq];
					z[ip] -= h; 
					z[iq] += h; 
					d[ip] -= h; 
					d[iq] += h;
					mat[ip][iq] = dgFloat32(0.0f);

					for (dgInt32 j = 0; j <= ip - 1; j ++) {
						dgFloat32 g = mat[j][ip]; 
						dgFloat32 h = mat[j][iq]; 
						mat[j][ip] = g - s * (h + g * tau); 
						mat[j][iq] = h + s * (g - h * tau);

					}
					for (dgInt32 j = ip + 1; j <= iq - 1; j ++) {
						dgFloat32 g = mat[ip][j]; 
						dgFloat32 h = mat[j][iq]; 
						mat[ip][j] = g - s * (h + g * tau); 
						mat[j][iq] = h + s * (g - h * tau);
					}
					for (dgInt32 j = iq + 1; j < 3; j ++) {
						dgFloat32 g = mat[ip][j]; 
						dgFloat32 h = mat[iq][j]; 
						mat[ip][j] = g - s * (h + g * tau); 
						mat[iq][j] = h + s * (g - h * tau);
					}

					for (dgInt32 j = 0; j < 3; j ++) {
						dgFloat32 g = eigenVectors[j][ip]; 
						dgFloat32 h = eigenVectors[j][iq]; 
						eigenVectors[j][ip] = g - s * (h + g * tau); 
						eigenVectors[j][iq] = h + s * (g - h * tau);
					}
				}
			}
		}
		b[0] += z[0]; d[0] = b[0]; z[0] = dgFloat32 (0.0f);
		b[1] += z[1]; d[1] = b[1]; z[1] = dgFloat32 (0.0f);
		b[2] += z[2]; d[2] = b[2]; z[2] = dgFloat32 (0.0f);
	}

	eigenValues = dgVector (d[0], d[1], d[2], dgFloat32 (1.0f));
	*this = eigenVectors.Inverse();
}


