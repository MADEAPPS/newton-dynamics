/* Copyright (c) <2003-2016> <Julio Jerez, Newton Game Dynamics>
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
#include "dgGeneralMatrix.h"

#ifndef _NEWTON_USE_DOUBLE

	dgVector dgVector::m_xMask(dgInt32(-1), dgInt32(0), dgInt32(0), dgInt32(0));
	dgVector dgVector::m_yMask(dgInt32(0), dgInt32(-1), dgInt32(0), dgInt32(0));
	dgVector dgVector::m_zMask(dgInt32(0), dgInt32(0), dgInt32(-1), dgInt32(0));
	dgVector dgVector::m_wMask(dgInt32(0), dgInt32(0), dgInt32(0), dgInt32(-1));
	dgVector dgVector::m_triplexMask(dgInt32(-1), dgInt32(-1), dgInt32(-1), dgInt32(0));
	dgVector dgVector::m_signMask(dgVector(dgInt32(-1), dgInt32(-1), dgInt32(-1), dgInt32(-1)).ShiftRightLogical(1));

	dgVector dgVector::m_zero(dgFloat32(0.0f));
	dgVector dgVector::m_one(dgFloat32(1.0f));
	dgVector dgVector::m_two(dgFloat32(2.0f));
	dgVector dgVector::m_half(dgFloat32(0.5f));
	dgVector dgVector::m_three(dgFloat32(3.0f));
	dgVector dgVector::m_negOne(dgFloat32(-1.0f));
	dgVector dgVector::m_epsilon(dgFloat32(1.0e-20f));
	dgVector dgVector::m_wOne(dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(1.0f));
#endif

dgBigVector dgBigVector::m_zero (dgFloat64 (0.0f));
dgBigVector dgBigVector::m_one (dgFloat64 (1.0f));
dgBigVector dgBigVector::m_two (dgFloat64 (2.0f));
dgBigVector dgBigVector::m_half (dgFloat32 (0.5f));
dgBigVector dgBigVector::m_three (dgFloat32 (3.0f));
dgBigVector dgBigVector::m_negOne (dgFloat32 (-1.0f));
dgBigVector dgBigVector::m_epsilon(dgFloat32(1.0e-20f));
dgBigVector dgBigVector::m_wOne (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (1.0f));
dgBigVector dgBigVector::m_triplexMask (dgInt32 (-1), dgInt32 (-1),	dgInt32 (-1), dgInt32 (0));
dgBigVector dgBigVector::m_signMask (dgBigVector(dgInt32 (-1), dgInt32 (-1), dgInt32 (-1), dgInt32 (-1)).ShiftRightLogical(1));

dgBigVector dgBigVector::m_xMask (dgInt32 (-1), dgInt32 ( 0),	dgInt32 ( 0), dgInt32 ( 0));
dgBigVector dgBigVector::m_yMask (dgInt32 ( 0), dgInt32 (-1),	dgInt32 ( 0), dgInt32 ( 0));
dgBigVector dgBigVector::m_zMask (dgInt32 ( 0), dgInt32 ( 0),	dgInt32 (-1), dgInt32 ( 0));
dgBigVector dgBigVector::m_wMask (dgInt32 ( 0), dgInt32 ( 0),	dgInt32 ( 0), dgInt32 (-1));

dgSpatialVector dgSpatialVector::m_zero (dgFloat32 (0.0f));


dgMatrix dgMatrix::m_zeroMatrix (dgVector (dgFloat32(0.0f)),
								 dgVector (dgFloat32(0.0f)),
								 dgVector (dgFloat32(0.0f)),
								 dgVector (dgFloat32(0.0f)));

dgMatrix dgMatrix::m_identityMatrix (dgVector (dgFloat32(1.0f), dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f)),
									 dgVector (dgFloat32(0.0f), dgFloat32(1.0f), dgFloat32(0.0f), dgFloat32(0.0f)),
									 dgVector (dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(1.0f), dgFloat32(0.0f)),
									 dgVector (dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(0.0f), dgFloat32(1.0f)));


#ifndef _NEWTON_USE_DOUBLE
#endif

const dgMatrix& dgGetIdentityMatrix()
{
	return dgMatrix::m_identityMatrix;
}

const dgMatrix& dgGetZeroMatrix ()
{
	return dgMatrix::m_zeroMatrix;
}


dgMatrix::dgMatrix (const dgQuaternion &quat0, const dgVector &position)
{
	dgQuaternion quat1 (quat0);
	quat1.Scale(dgFloat32 (2.0f));

	dgFloat32 x2 = quat0.m_x * quat1.m_x;
	dgFloat32 y2 = quat0.m_y * quat1.m_y;
	dgFloat32 z2 = quat0.m_z * quat1.m_z;

#ifdef _DEBUG
	dgFloat32 w2 = quat0.m_w * quat1.m_w;
	dgAssert (dgAbs (w2 + x2 + y2 + z2 - dgFloat32(2.0f)) <dgFloat32 (1.0e-3f));
#endif

	dgFloat32 xy = quat0.m_x * quat1.m_y;
	dgFloat32 xz = quat0.m_x * quat1.m_z;
	dgFloat32 xw = quat0.m_x * quat1.m_w;
	dgFloat32 yz = quat0.m_y * quat1.m_z;
	dgFloat32 yw = quat0.m_y * quat1.m_w;
	dgFloat32 zw = quat0.m_z * quat1.m_w;

	m_front = dgVector (dgFloat32(1.0f) - y2 - z2, xy + zw, xz - yw, dgFloat32(0.0f));
	m_up    = dgVector (xy - zw, dgFloat32(1.0f) - x2 - z2, yz + xw, dgFloat32(0.0f));
	m_right = dgVector (xz + yw, yz - xw, dgFloat32(1.0f) - x2 - y2, dgFloat32(0.0f));

	m_posit.m_x = position.m_x;
	m_posit.m_y = position.m_y;
	m_posit.m_z = position.m_z;
	m_posit.m_w = dgFloat32(1.0f);
}


dgMatrix::dgMatrix (const dgMatrix& transformMatrix, const dgVector& scale, const dgMatrix& stretchAxis)
{
	dgMatrix scaledAxis;
	scaledAxis[0] = stretchAxis[0].Scale (scale[0]);
	scaledAxis[1] = stretchAxis[1].Scale (scale[1]);
	scaledAxis[2] = stretchAxis[2].Scale (scale[2]);
	scaledAxis[3] = stretchAxis[3];

	*this = stretchAxis.Transpose() * scaledAxis * transformMatrix;
}

dgMatrix dgMatrix::Multiply3X3 (const dgMatrix &B) const
{
	return dgMatrix (B.m_front * m_front.BroadcastX() + B.m_up * m_front.BroadcastY() + B.m_right * m_front.BroadcastZ(), 
					 B.m_front * m_up.BroadcastX()    + B.m_up * m_up.BroadcastY()    + B.m_right * m_up.BroadcastZ(), 
					 B.m_front * m_right.BroadcastX() + B.m_up * m_right.BroadcastY() + B.m_right * m_right.BroadcastZ(), 
					 dgVector::m_wOne); 
}

dgMatrix dgMatrix::operator* (const dgMatrix &B) const
{
	return dgMatrix (B.m_front * m_front.BroadcastX() + B.m_up * m_front.BroadcastY() + B.m_right * m_front.BroadcastZ() + B.m_posit * m_front.BroadcastW(), 
					 B.m_front * m_up.BroadcastX()    + B.m_up * m_up.BroadcastY()    + B.m_right * m_up.BroadcastZ()    + B.m_posit * m_up.BroadcastW(), 
					 B.m_front * m_right.BroadcastX() + B.m_up * m_right.BroadcastY() + B.m_right * m_right.BroadcastZ() + B.m_posit * m_right.BroadcastW(), 
					 B.m_front * m_posit.BroadcastX() + B.m_up * m_posit.BroadcastY() + B.m_right * m_posit.BroadcastZ() + B.m_posit * m_posit.BroadcastW()); 
}



void dgMatrix::TransformTriplex (dgFloat32* const dst, dgInt32 dstStrideInBytes, const dgFloat32* const src, dgInt32 srcStrideInBytes, dgInt32 count) const
{
	dgInt32 dstStride = dgInt32 (dstStrideInBytes /sizeof (dgFloat32));
	dgInt32 srcStride = dgInt32 (srcStrideInBytes / sizeof (dgFloat32));

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
	dgInt32 dstStride = dgInt32 (dstStrideInBytes /sizeof (dgFloat64));
	dgInt32 srcStride = dgInt32 (srcStrideInBytes / sizeof (dgFloat64));

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
	dgInt32 dstStride = dgInt32 (dstStrideInBytes /sizeof (dgFloat64));
	dgInt32 srcStride = dgInt32 (srcStrideInBytes / sizeof (dgFloat32));

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
	//dgVector size ((p1local - p0local).Scale (dgFloat32 (0.5f)));
	//dgVector center (TransformVector ((p1local + p0local).Scale (dgFloat32 (0.5f))));
	dgVector size ((p1local - p0local) * dgVector::m_half);
	dgVector center (TransformVector ((p1local + p0local) * dgVector::m_half));
	dgVector extends (size.m_x * dgAbs(matrix[0][0]) + size.m_y * dgAbs(matrix[1][0]) + size.m_z * dgAbs(matrix[2][0]),  
					  size.m_x * dgAbs(matrix[0][1]) + size.m_y * dgAbs(matrix[1][1]) + size.m_z * dgAbs(matrix[2][1]),  
	                  size.m_x * dgAbs(matrix[0][2]) + size.m_y * dgAbs(matrix[1][2]) + size.m_z * dgAbs(matrix[2][2]), dgFloat32 (0.0f));  

	p0 = center - extends;
	p1 = center + extends;
}

dgMatrix dgMatrix::Inverse4x4 () const
{
	dgMatrix tmp (*this);
	dgMatrix inv (dgGetIdentityMatrix());
	for (dgInt32 i = 0; i < 4; i++) {
		dgFloat32 pivot = dgAbs(tmp[i][i]);
		if (pivot < dgFloat32(0.01f)) {
			dgInt32 permute = i;
			for (dgInt32 j = i + 1; j < 4; j++) {
				dgFloat32 pivot1 = dgAbs(tmp[j][i]);
				if (pivot1 > pivot) {
					permute = j;
					pivot = pivot1;
				}
			}
			if (permute != i) {
				dgAssert(pivot > dgFloat32(0.0f));
				dgAssert((pivot > dgFloat32(1.0e-6f)) || (dgConditionNumber(4, 4, (dgFloat32*)&(*this)[0][0]) < dgFloat32(1.0e5f)));
				dgSwap(inv[i], inv[permute]);
				dgSwap(tmp[i], tmp[permute]);
			}
		}

		for (dgInt32 j = i + 1; j < 4; j++) {
			dgVector scale (tmp[j][i] / tmp[i][i]);
			tmp[j] -= tmp[i] * scale;
			inv[j] -= inv[i] * scale;
			tmp[j][i] = dgFloat32 (0.0f);
		}
	}

	for (dgInt32 i = 3; i >= 0; i--) {
		dgVector acc(dgVector::m_zero);
		for (dgInt32 j = i + 1; j < 4; j++) {
			dgVector pivot(tmp[i][j]);
			acc += pivot * inv[j];
		}
		dgVector den(dgFloat32(1.0f) / tmp[i][i]);
		inv[i] = den * (inv[i] - acc);
	}

#ifdef _DEBUG
	tmp = *this * inv;
	for (dgInt32 i = 0; i < 4; i++) {
		dgAssert(dgAbs(tmp[i][i] - dgFloat32(1.0f)) < dgFloat32(1.0e-6f));
		for (dgInt32 j = i + 1; j < 4; j++) {
			dgAssert(dgAbs(tmp[i][j]) < dgFloat32(1.0e-6f));
			dgAssert(dgAbs(tmp[j][i]) < dgFloat32(1.0e-6f));
		}
	}
#endif

	return inv;
}

dgVector dgMatrix::SolveByGaussianElimination(const dgVector &v) const
{
//	return  Inverse4x4().UnrotateVector(v);
	dgMatrix tmp(*this);
	dgVector ret(v);
	for (dgInt32 i = 0; i < 4; i++) {
		dgFloat32 pivot = dgAbs(tmp[i][i]);
		if (pivot < dgFloat32(0.01f)) {
			dgInt32 permute = i;
			for (dgInt32 j = i + 1; j < 4; j++) {
				dgFloat32 pivot1 = dgAbs(tmp[j][i]);
				if (pivot1 > pivot) {
					permute = j;
					pivot = pivot1;
				}
			}
			
			if (permute != i) {
				dgAssert(pivot > dgFloat32(1.0e-6f));
				dgSwap(ret[i], ret[permute]);
				dgSwap(tmp[i], tmp[permute]);
			}
		}

		for (dgInt32 j = i + 1; j < 4; j++) {
			dgVector scale(tmp[j][i] / tmp[i][i]);
			tmp[j] -= tmp[i] * scale;
			ret[j] -= ret[i] * scale.GetScalar();
			tmp[j][i] = dgFloat32(0.0f);
		}
	}

	for (dgInt32 i = 3; i >= 0; i--) {
		dgVector pivot(tmp[i] * ret);
		ret[i] = (ret[i] - pivot.AddHorizontal().GetScalar() + tmp[i][i] * ret[i]) / tmp[i][i];
	}

	return ret;
}

void dgMatrix::CalcPitchYawRoll (dgVector& euler0, dgVector& euler1) const
{
	const dgMatrix& matrix = *this;
	dgAssert (matrix[2].DotProduct(matrix[0].CrossProduct(matrix[1])).GetScalar() > 0.0f);
	dgAssert (dgAbs (matrix[2].DotProduct(matrix[0].CrossProduct(matrix[1])).GetScalar() - dgFloat32 (1.0f)) < dgFloat32 (1.0e-4f));
/*
	// Assuming the angles are in radians.
	if (matrix[0][2] > dgFloat32 (0.99995f)) {
		dgFloat32 picth0 = dgFloat32 (0.0f);
		dgFloat32 yaw0 = dgFloat32 (-dgPI * 0.5f);
		dgFloat32 roll0 = - dgAtan2(matrix[2][1], matrix[1][1]);
		euler0[0] = picth0;
		euler0[1] = yaw0;
		euler0[2] = roll0;

		euler1[0] = picth0;
		euler1[1] = yaw0;
		euler1[2] = roll0;

	} else if (matrix[0][2] < dgFloat32 (-0.99995f)) {
		dgFloat32 picth0 = dgFloat32 (0.0f);
		dgFloat32 yaw0 = dgFloat32 (dgPI * 0.5f);
		dgFloat32 roll0 = dgAtan2(matrix[2][1], matrix[1][1]);
		euler0[0] = picth0;
		euler0[1] = yaw0;
		euler0[2] = roll0;

		euler1[0] = picth0;
		euler1[1] = yaw0;
		euler1[2] = roll0;
	} else {
		dgFloat32 yaw0 = -dgAsin ( matrix[0][2]);
		dgFloat32 yaw1 = dgFloat32 (dgPI) - yaw0;
		dgFloat32 sign0 = dgSign(dgCos (yaw0));
		dgFloat32 sign1 = dgSign(dgCos (yaw1));

		dgFloat32 picth0 = dgAtan2(matrix[1][2] * sign0, matrix[2][2] * sign0);
		dgFloat32 picth1 = dgAtan2(matrix[1][2] * sign1, matrix[2][2] * sign1);

		dgFloat32 roll0 = dgAtan2(matrix[0][1] * sign0, matrix[0][0] * sign0);
		dgFloat32 roll1 = dgAtan2(matrix[0][1] * sign1, matrix[0][0] * sign1);

		if (yaw1 > dgFloat32 (dgPI)) {
			yaw1 -= dgFloat32 (2.0f * dgPI);
		}

		euler0[0] = picth0;
		euler0[1] = yaw0;
		euler0[2] = roll0;

		euler1[0] = picth1;
		euler1[1] = yaw1;
		euler1[2] = roll1;
	}
*/

	// Assuming the angles are in radians.
	if (matrix[0][2] > dgFloat32 (0.99995f)) {
		dgFloat32 picth0 = dgFloat32(0.0f);
		dgFloat32 yaw0 = dgFloat32(-dgPi * 0.5f);
		dgFloat32 roll0 = -dgAtan2(matrix[2][1], matrix[1][1]);
		euler0[0] = picth0;
		euler0[1] = yaw0;
		euler0[2] = roll0;

		euler1[0] = picth0;
		euler1[1] = yaw0;
		euler1[2] = roll0;

	} else if (matrix[0][2] < dgFloat32 (-0.99995f)) {
		dgFloat32 picth0 = dgFloat32 (0.0f);
		dgFloat32 yaw0 = dgFloat32(dgPi * 0.5f);
		dgFloat32 roll0 = dgAtan2(matrix[2][1], matrix[1][1]);
		euler0[0] = picth0;
		euler0[1] = yaw0;
		euler0[2] = roll0;

		euler1[0] = picth0;
		euler1[1] = yaw0;
		euler1[2] = roll0;

	} else {
		dgFloat32 yaw0 = -dgAsin(matrix[0][2]);
		dgFloat32 yaw1 = dgFloat32(dgPi) - yaw0;

		dgFloat32 picth0 = dgAtan2( matrix[1][2],  matrix[2][2]);
		dgFloat32 picth1 = dgAtan2(-matrix[1][2], -matrix[2][2]);

		dgFloat32 roll0 = dgAtan2( matrix[0][1],  matrix[0][0]);
		dgFloat32 roll1 = dgAtan2(-matrix[0][1], -matrix[0][0]);

		if (yaw1 > dgFloat32 (dgPi)) {
			yaw1 -= dgFloat32 (2.0f * dgPi);
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
	for (dgInt32 i = 0; i < 3; i ++) {
		for (dgInt32 j = 0; j < 3; j ++) {
			dgFloat32 error = dgAbs (m0[i][j] - matrix[i][j]);
			dgAssert (error < 5.0e-2f);
			error = dgAbs (m1[i][j] - matrix[i][j]);
			dgAssert (error < 5.0e-2f);
		}
	}
#endif
}


void dgMatrix::PolarDecomposition (dgMatrix& transformMatrix, dgVector& scale, dgMatrix& stretchAxis, const dgMatrix* const initialStretchAxis) const
{
	// a polar decomposition decompose matrix A = O * S
	// where S = sqrt (transpose (L) * L)

	const dgMatrix& me = *this;
	dgFloat32 sign = dgSign (me[2].DotProduct(me[0].CrossProduct(me[1])).GetScalar());
	stretchAxis = me * Transpose();
	//stretchAxis.EigenVectors (scale);
	scale = stretchAxis.EigenVectors();

	// I need to deal with by seeing of some of the Scale are duplicated
	// do this later (maybe by a given rotation around the non uniform axis but I do not know if it will work)
	// for now just us the matrix

	scale[0] = sign * dgSqrt (scale[0]);
	scale[1] = sign * dgSqrt (scale[1]);
	scale[2] = sign * dgSqrt (scale[2]);
	scale[3] = dgFloat32 (0.0f);

	dgMatrix scaledAxis;
	scaledAxis[0] = stretchAxis[0].Scale (dgFloat32 (1.0f) / scale[0]);
	scaledAxis[1] = stretchAxis[1].Scale (dgFloat32 (1.0f) / scale[1]);
	scaledAxis[2] = stretchAxis[2].Scale (dgFloat32 (1.0f) / scale[2]);
	scaledAxis[3] = stretchAxis[3];
	dgMatrix symetricInv (stretchAxis.Transpose() * scaledAxis);

	transformMatrix = symetricInv * (*this);
	transformMatrix.m_posit = m_posit;
}


dgVector dgMatrix::EigenVectors ()
{
#if 0
	// still have teh same problem I had in teh pass, becaually QR algorith is really bad 
	// at converging for matroces with very dioffrent eiegen values. 
	// the solution is to use RD with double shift. 
	dgMatrix eigenValues (*this);
	dgMatrix& eigenVectors = *this;
	eigenVectors = dgGetIdentityMatrix();
	if (dgAbs(eigenValues.m_front.m_z) > dgFloat32(1.0e-6f)) {
		// calcualte initial guess by convert to tridiagonal matrix using householder
		dgVector u(eigenValues.m_front);
		u.m_x = dgFloat32(0.0f);
		u.m_y -= dgSqrt(u.DotProduct(u).GetScalar());
		dgAssert(dgAbs(u.m_y) > dgFloat32(1.0e-6f));
		dgVector v(u.Scale(-dgFloat32(2.0f) / u.DotProduct(u).GetScalar()));

		eigenVectors = dgMatrix(v, u);
		eigenVectors[0][0] += dgFloat32(1.0f);
		eigenVectors[1][1] += dgFloat32(1.0f);
		eigenVectors[2][2] += dgFloat32(1.0f);
		eigenValues = eigenVectors.Transpose() * eigenValues * eigenVectors;
	}

	eigenValues[0][2] = dgFloat32(0.0f);
	eigenValues[2][0] = dgFloat32(0.0f);
	for (dgInt32 i = 0; (i < 16) && ((m_front.m_y * m_front.m_y + m_front.m_z * m_front.m_z + m_up.m_z * m_up.m_z) > dgFloat32(1.0e-12f)); i++) {
		dgMatrix householder(dgGetIdentityMatrix());
		dgFloat32 mag2 = eigenValues.m_front.m_y * eigenValues.m_front.m_y + eigenValues.m_front.m_z * eigenValues.m_front.m_z;
		if (mag2 > dgFloat32(1.0e-12f)) {
			// convert to tridiagonal matrix using householder
			dgVector u(eigenValues.m_front);
			u.m_x -= dgSqrt(eigenValues.m_front.m_x * eigenValues.m_front.m_x + mag2);
			dgAssert(dgAbs(u.m_x) > dgFloat32(1.0e-12f));
			dgVector v(u.Scale(-dgFloat32(2.0f) / u.DotProduct(u).GetScalar()));

			householder = dgMatrix(v, u);
			householder[0][0] += dgFloat32(1.0f);
			householder[1][1] += dgFloat32(1.0f);
			householder[2][2] += dgFloat32(1.0f);
			eigenValues = eigenValues * householder.Transpose();
		}

		if (dgAbs(eigenValues.m_up.m_z) > 1.0e-6f) {
			dgVector u(eigenValues.m_up);
			u.m_x = dgFloat32(0.0f);
			u.m_y -= dgSqrt(u.DotProduct(u).GetScalar());
			dgVector v(u.Scale(-dgFloat32(2.0f) / u.DotProduct(u).GetScalar()));

			dgMatrix househoulder1(dgGetIdentityMatrix());
			househoulder1.m_up.m_y += v.m_y * u.m_y;
			househoulder1.m_up.m_z = v.m_y * u.m_z;
			househoulder1.m_right.m_y = v.m_y * u.m_z;
			househoulder1.m_right.m_z += v.m_z * u.m_z;
			householder = househoulder1 * householder;
			eigenValues = eigenValues * househoulder1.Transpose();
		}
		eigenVectors = householder * eigenVectors;
		eigenValues = householder * eigenValues;
	}

	return dgVector(0.0f);

#else
	dgMatrix& mat = *this;
	dgMatrix eigenVectors(dgGetIdentityMatrix());
	if (dgAbs(m_front.m_z) > dgFloat32(1.0e-6f)) {
		// calculate initial guess by convert to tridiagonal matrix using householder
		dgVector u(m_front);
		u.m_x = dgFloat32(0.0f);
		u.m_y -= dgSqrt(u.DotProduct(u).GetScalar());
		//dgAssert(dgAbs(u.m_y) > dgFloat32(1.0e-12f));
		dgAssert(u.DotProduct(u).GetScalar() > dgFloat32(0.0f));
		dgVector v(u.Scale(-dgFloat32(2.0f) / u.DotProduct(u).GetScalar()));

		eigenVectors = dgMatrix(v, u);
		eigenVectors[0][0] += dgFloat32(1.0f);
		eigenVectors[1][1] += dgFloat32(1.0f);
		eigenVectors[2][2] += dgFloat32(1.0f);
		mat = eigenVectors.Transpose() * mat * eigenVectors;
	}

	mat[0][2] = dgFloat32(0.0f);
	mat[2][0] = dgFloat32(0.0f);

	dgVector d (mat[0][0], mat[1][1], mat[2][2], dgFloat32 (0.0f)); 
	dgVector b (d);
	for (dgInt32 i = 0; i < 50; i++) {
		dgFloat32 sm = mat[0][1] * mat[0][1] + mat[0][2] * mat[0][2] + mat[1][2] * mat[0][2];
		if (sm < dgFloat32 (1.0e-12f)) {
			// order the eigenvalue vectors	
			dgVector tmp (eigenVectors.m_front.CrossProduct(eigenVectors.m_up));
			if (tmp.DotProduct(eigenVectors.m_right).GetScalar() < dgFloat32(0.0f)) {
				eigenVectors.m_right = eigenVectors.m_right * dgVector::m_negOne;
			}
			break;
		}

		dgFloat32 thresh = dgFloat32 (0.0f);
		if (i < 3) {
			thresh = (dgFloat32)(0.2f / 9.0f) * sm;
		}

		dgVector z (dgVector::m_zero);
		for (dgInt32 ip = 0; ip < 2; ip ++) {
			for (dgInt32 iq = ip + 1; iq < 3; iq ++) {
				dgFloat32 g = dgFloat32 (100.0f) * dgAbs(mat[ip][iq]);
				if ((i > 3) && ((dgAbs(d[ip]) + g) == dgAbs(d[ip])) && ((dgAbs(d[iq]) + g) == dgAbs(d[iq]))) {
					mat[ip][iq] = dgFloat32 (0.0f);
				} else if (dgAbs(mat[ip][iq]) > thresh) {

					dgFloat32 t;
					dgFloat32 h = d[iq] - d[ip];
					if (dgAbs(h) + g == dgAbs(h)) {
						t = mat[ip][iq] / h;
					} else {
						dgFloat32 theta = dgFloat32 (0.5f) * h / mat[ip][iq];
						t = dgFloat32(1.0f) / (dgAbs(theta) + dgSqrt(dgFloat32(1.0f) + theta * theta));
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
						dgFloat32 g0 = mat[j][ip]; 
						dgFloat32 h0 = mat[j][iq]; 
						mat[j][ip] = g0 - s * (h0 + g0 * tau); 
						mat[j][iq] = h0 + s * (g0 - h0 * tau);
					}
					for (dgInt32 j = ip + 1; j <= iq - 1; j ++) {
						dgFloat32 g0 = mat[ip][j]; 
						dgFloat32 h0 = mat[j][iq]; 
						mat[ip][j] = g0 - s * (h0 + g0 * tau); 
						mat[j][iq] = h0 + s * (g0 - h0 * tau);
					}
					for (dgInt32 j = iq + 1; j < 3; j ++) {
						dgFloat32 g0 = mat[ip][j]; 
						dgFloat32 h0 = mat[iq][j]; 
						mat[ip][j] = g0 - s * (h0 + g0 * tau); 
						mat[iq][j] = h0 + s * (g0 - h0 * tau);
					}

					dgVector sv (s);
					dgVector tauv (tau);
					dgVector gv (eigenVectors[ip]);
					dgVector hv (eigenVectors[iq]);
					eigenVectors[ip] -= sv * (hv + gv * tauv); 
					eigenVectors[iq] += sv * (gv - hv * tauv);
				}
			}
		}

		b += z; 
		d = b; 
	}

	//eigenValues = d;
	*this = eigenVectors;
	return d;
#endif
}

dgSpatialMatrix dgSpatialMatrix::Inverse(dgInt32 rows) const
{
	dgSpatialMatrix tmp(*this);
	dgSpatialMatrix inv(dgFloat64(0.0f));
	for (dgInt32 i = 0; i < rows; i++) {
		inv[i][i] = dgFloat32(1.0f);
	}

	for (dgInt32 i = 0; i < rows; i++) {
		dgFloat64 pivot = dgAbs(tmp[i][i]);
		if (pivot < dgFloat64(0.01f)) {
			int permute = i;
			for (dgInt32 j = i + 1; j < rows; j++) {
				dgFloat64 pivot1 = dgAbs(tmp[j][i]);
				if (pivot1 > pivot) {
					permute = j;
					pivot = pivot1;
				}
			}
			dgAssert(pivot > dgFloat32(0.0f));
			dgAssert((pivot > dgFloat32(1.0e-6f)) || (dgConditionNumber(rows, 6, (dgFloat64*)&m_rows[0]) < dgFloat32(1.0e5f)));
			//if (!((pivot > dgFloat32(1.0e-6f)) || (dgConditionNumber(rows, 6, (dgFloat64*)&m_rows[0]) < dgFloat32(1.0e5f))))
			//{
			//	for (dgInt32 m = 0; m < rows; m++) {
			//		for (dgInt32 n = 0; n < rows; n++) {
			//			dgTrace(("%f ", m_rows[m][n]));
			//		}
			//		dgTrace(("\n"));
			//	}
			//	dgAssert(0);
			//}

			if (permute != i) {
				for (dgInt32 j = 0; j < rows; j++) {
					dgSwap(tmp[i][j], tmp[permute][j]);
					dgSwap(tmp[i][j], tmp[permute][j]);
				}
			}
		}

		for (dgInt32 j = i + 1; j < rows; j++) {
			dgFloat64 scale = tmp[j][i] / tmp[i][i];
			tmp[j][i] = dgFloat64(0.0f);
			for (int k = i + 1; k < rows; k++) {
				tmp[j][k] -= scale * tmp[i][k];
			}
			for (int k = 0; k <= i; k++) {
				inv[j][k] -= scale * inv[i][k];
			}
		}
	}

	for (dgInt32 i = rows - 1; i >= 0; i--) {
		dgSpatialVector acc(dgFloat64(0.0f));
		for (dgInt32 j = i + 1; j < rows; j++) {
			dgFloat64 pivot = tmp[i][j];
			for (int k = 0; k < rows; k++) {
				acc[k] += pivot * inv[j][k];
			}
		}
		dgFloat64 den = dgFloat64(1.0f) / tmp[i][i];
		for (dgInt32 k = 0; k < rows; k++) {
			inv[i][k] = den * (inv[i][k] - acc[k]);
		}
	}


#ifdef _DEBUG
	for (dgInt32 i = 0; i < rows; i++) {
		for (dgInt32 j = 0; j < rows; j++) {
			tmp[i][j] = m_rows[j][i];
		}
	}
	for (dgInt32 i = 0; i < rows; i++) {
		dgSpatialVector v(inv.VectorTimeMatrix(tmp[i], rows));
		dgAssert(dgAbs(v[i] - dgFloat64(1.0f)) < dgFloat64(1.0e-6f));
		for (dgInt32 j = 0; j < rows; j++) {
			if (j != i) {
				dgAssert(dgAbs(v[j]) < dgFloat64(1.0e-6f));
			}
		}
	}
#endif

	return inv;
}
