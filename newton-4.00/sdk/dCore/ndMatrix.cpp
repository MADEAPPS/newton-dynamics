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

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndMatrix.h"
#include "ndQuaternion.h"
#include "ndGeneralMatrix.h"

const dMatrix& dGetIdentityMatrix()
{
	static dMatrix identityMatrix(
		dVector(dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f)),
		dVector(dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f), dFloat32(0.0f)),
		dVector(dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f), dFloat32(0.0f)),
		dVector(dFloat32(0.0f), dFloat32(0.0f), dFloat32(0.0f), dFloat32(1.0f)));
	return identityMatrix;
}

const dMatrix& dGetZeroMatrix ()
{
	static dMatrix zeroMatrix(dVector::m_zero, dVector::m_zero, dVector::m_zero, dVector::m_zero);
	return zeroMatrix;
}

dMatrix::dMatrix (const dQuaternion &quat, const dVector &position)
{
	dAssert((quat.DotProduct(quat).GetScalar() - dFloat32(1.0f)) < dFloat32(1.0e-4f));
	const dQuaternion quat0(quat);
	const dQuaternion quat1(quat0.Scale (dFloat32(2.0f)));

	dFloat32 x2 = quat0.m_x * quat1.m_x;
	dFloat32 y2 = quat0.m_y * quat1.m_y;
	dFloat32 z2 = quat0.m_z * quat1.m_z;

#ifdef _DEBUG
	dFloat32 w2 = quat0.m_w * quat1.m_w;
	dAssert (dAbs (w2 + x2 + y2 + z2 - dFloat32(2.0f)) <dFloat32 (1.0e-3f));
#endif

	dFloat32 xy = quat0.m_x * quat1.m_y;
	dFloat32 xz = quat0.m_x * quat1.m_z;
	dFloat32 xw = quat0.m_x * quat1.m_w;
	dFloat32 yz = quat0.m_y * quat1.m_z;
	dFloat32 yw = quat0.m_y * quat1.m_w;
	dFloat32 zw = quat0.m_z * quat1.m_w;

	m_front = dVector (dFloat32(1.0f) - y2 - z2, xy + zw, xz - yw, dFloat32(0.0f));
	m_up    = dVector (xy - zw, dFloat32(1.0f) - x2 - z2, yz + xw, dFloat32(0.0f));
	m_right = dVector (xz + yw, yz - xw, dFloat32(1.0f) - x2 - y2, dFloat32(0.0f));

	m_posit.m_x = position.m_x;
	m_posit.m_y = position.m_y;
	m_posit.m_z = position.m_z;
	m_posit.m_w = dFloat32(1.0f);
}

dMatrix::dMatrix (const dMatrix& transformMatrix, const dVector& scale, const dMatrix& stretchAxis)
{
	//dMatrix scaledAxis;
	//scaledAxis[0] = stretchAxis[0].Scale (scale[0]);
	//scaledAxis[1] = stretchAxis[1].Scale (scale[1]);
	//scaledAxis[2] = stretchAxis[2].Scale (scale[2]);
	//scaledAxis[3] = stretchAxis[3];
	dMatrix scaledAxis(
		stretchAxis[0].Scale(scale[0]),
		stretchAxis[1].Scale(scale[1]),
		stretchAxis[2].Scale(scale[2]),
		stretchAxis[3]);
	*this = stretchAxis.Transpose() * scaledAxis * transformMatrix;
}

dMatrix dMatrix::Multiply3X3 (const dMatrix &B) const
{
	return dMatrix (B.m_front * m_front.BroadcastX() + B.m_up * m_front.BroadcastY() + B.m_right * m_front.BroadcastZ(), 
					B.m_front * m_up.BroadcastX()    + B.m_up * m_up.BroadcastY()    + B.m_right * m_up.BroadcastZ(), 
					B.m_front * m_right.BroadcastX() + B.m_up * m_right.BroadcastY() + B.m_right * m_right.BroadcastZ(), 
					dVector::m_wOne); 
}

dMatrix dMatrix::operator* (const dMatrix &B) const
{
	return dMatrix (B.m_front * m_front.BroadcastX() + B.m_up * m_front.BroadcastY() + B.m_right * m_front.BroadcastZ() + B.m_posit * m_front.BroadcastW(), 
					B.m_front * m_up.BroadcastX()    + B.m_up * m_up.BroadcastY()    + B.m_right * m_up.BroadcastZ()    + B.m_posit * m_up.BroadcastW(), 
					B.m_front * m_right.BroadcastX() + B.m_up * m_right.BroadcastY() + B.m_right * m_right.BroadcastZ() + B.m_posit * m_right.BroadcastW(), 
					B.m_front * m_posit.BroadcastX() + B.m_up * m_posit.BroadcastY() + B.m_right * m_posit.BroadcastZ() + B.m_posit * m_posit.BroadcastW()); 
}

void dMatrix::TransformTriplex (dFloat32* const dst, dInt32 dstStrideInBytes, const dFloat32* const src, dInt32 srcStrideInBytes, dInt32 count) const
{
	dInt32 dstStride = dInt32 (dstStrideInBytes /sizeof (dFloat32));
	dInt32 srcStride = dInt32 (srcStrideInBytes / sizeof (dFloat32));

	dInt32 dstIndex = 0;
	dInt32 srcIndex = 0;
	for (dInt32 i = 0 ; i < count; i ++ ) 
	{
		dFloat32 x = src[srcIndex + 0];
		dFloat32 y = src[srcIndex + 1];
		dFloat32 z = src[srcIndex + 2];
		srcIndex += srcStride;
		dst[dstIndex + 0] = x * m_front.m_x + y * m_up.m_x + z * m_right.m_x + m_posit.m_x;
		dst[dstIndex + 1] = x * m_front.m_y + y * m_up.m_y + z * m_right.m_y + m_posit.m_y;
		dst[dstIndex + 2] = x * m_front.m_z + y * m_up.m_z + z * m_right.m_z + m_posit.m_z;
		dstIndex += dstStride;
	}
}

#ifndef D_NEWTON_USE_DOUBLE
void dMatrix::TransformTriplex (dFloat64* const dst, dInt32 dstStrideInBytes, const dFloat64* const src, dInt32 srcStrideInBytes, dInt32 count) const
{
	dInt32 dstStride = dInt32 (dstStrideInBytes /sizeof (dFloat64));
	dInt32 srcStride = dInt32 (srcStrideInBytes / sizeof (dFloat64));

	dInt32 dstIndex = 0;
	dInt32 srcIndex = 0;
	for (dInt32 i = 0 ; i < count; i ++ ) 
	{
		dFloat64 x = src[srcIndex + 0];
		dFloat64 y = src[srcIndex + 1];
		dFloat64 z = src[srcIndex + 2];
		srcIndex += srcStride;
		dst[dstIndex + 0] = x * m_front.m_x + y * m_up.m_x + z * m_right.m_x + m_posit.m_x;
		dst[dstIndex + 1] = x * m_front.m_y + y * m_up.m_y + z * m_right.m_y + m_posit.m_y;
		dst[dstIndex + 2] = x * m_front.m_z + y * m_up.m_z + z * m_right.m_z + m_posit.m_z;
		dstIndex += dstStride;
	}
}

void dMatrix::TransformTriplex (dFloat64* const dst, dInt32 dstStrideInBytes, const dFloat32* const src, dInt32 srcStrideInBytes, dInt32 count) const
{
	dInt32 dstStride = dInt32 (dstStrideInBytes /sizeof (dFloat64));
	dInt32 srcStride = dInt32 (srcStrideInBytes / sizeof (dFloat32));

	dInt32 dstIndex = 0;
	dInt32 srcIndex = 0;
	for (dInt32 i = 0 ; i < count; i ++ ) 
	{
		dFloat64 x = src[srcIndex + 0];
		dFloat64 y = src[srcIndex + 1];
		dFloat64 z = src[srcIndex + 2];
		srcIndex += srcStride;
		dst[dstIndex + 0] = x * m_front.m_x + y * m_up.m_x + z * m_right.m_x + m_posit.m_x;
		dst[dstIndex + 1] = x * m_front.m_y + y * m_up.m_y + z * m_right.m_y + m_posit.m_y;
		dst[dstIndex + 2] = x * m_front.m_z + y * m_up.m_z + z * m_right.m_z + m_posit.m_z;
		dstIndex += dstStride;
	}
}
#endif

void dMatrix::TransformBBox (const dVector& p0local, const dVector& p1local, dVector& p0, dVector& p1) const
{
	const dMatrix& matrix = *this;
	dVector size ((p1local - p0local) * dVector::m_half);
	dVector center (TransformVector ((p1local + p0local) * dVector::m_half));
	dVector extends (size.m_x * dAbs(matrix[0][0]) + size.m_y * dAbs(matrix[1][0]) + size.m_z * dAbs(matrix[2][0]),  
					 size.m_x * dAbs(matrix[0][1]) + size.m_y * dAbs(matrix[1][1]) + size.m_z * dAbs(matrix[2][1]),  
	                 size.m_x * dAbs(matrix[0][2]) + size.m_y * dAbs(matrix[1][2]) + size.m_z * dAbs(matrix[2][2]), dFloat32 (0.0f));  

	p0 = center - extends;
	p1 = center + extends;
}

dMatrix dMatrix::Inverse4x4 () const
{
	dMatrix tmp (*this);
	dMatrix inv (dGetIdentityMatrix());
	for (dInt32 i = 0; i < 4; i++) 
	{
		dFloat32 pivot = dAbs(tmp[i][i]);
		if (pivot < dFloat32(0.1f)) 
		{
			dInt32 permute = i;
			for (dInt32 j = i + 1; j < 4; j++) 
			{
				dFloat32 pivot1 = dAbs(tmp[j][i]);
				if (pivot1 > pivot) 
				{
					permute = j;
					pivot = pivot1;
				}
			}
			if (permute != i) 
			{
				dAssert(pivot > dFloat32(0.0f));
				dAssert((pivot > dFloat32(1.0e-6f)) || (dConditionNumber(4, 4, (dFloat32*)&(*this)[0][0]) < dFloat32(1.0e5f)));
				dSwap(inv[i], inv[permute]);
				dSwap(tmp[i], tmp[permute]);
			}
		}

		for (dInt32 j = i + 1; j < 4; j++) 
		{
			dVector scale (tmp[j][i] / tmp[i][i]);
			tmp[j] -= tmp[i] * scale;
			inv[j] -= inv[i] * scale;
			tmp[j][i] = dFloat32 (0.0f);
		}
	}

	for (dInt32 i = 3; i >= 0; i--) 
	{
		dVector acc(dVector::m_zero);
		for (dInt32 j = i + 1; j < 4; j++) 
		{
			dVector pivot(tmp[i][j]);
			acc += pivot * inv[j];
		}
		dVector den(dFloat32(1.0f) / tmp[i][i]);
		inv[i] = den * (inv[i] - acc);
	}

#ifdef _DEBUG
	tmp = *this * inv;
	for (dInt32 i = 0; i < 4; i++) 
	{
		dFloat32 error = tmp[i][i] - dFloat32(1.0f);
		dAssert(dAbs(error) < dFloat32(1.0e-3f));
		for (dInt32 j = i + 1; j < 4; j++) 
		{
			dAssert(dAbs(tmp[i][j]) < dFloat32(1.0e-3f));
			dAssert(dAbs(tmp[j][i]) < dFloat32(1.0e-3f));
		}
	}
#endif

	return inv;
}

dVector dMatrix::SolveByGaussianElimination(const dVector &v) const
{
	dMatrix tmp(*this);
	dVector ret(v);
	for (dInt32 i = 0; i < 4; i++) 
	{
		dFloat32 pivot = dAbs(tmp[i][i]);
		if (pivot < dFloat32(0.01f)) 
		{
			dInt32 permute = i;
			for (dInt32 j = i + 1; j < 4; j++) 
			{
				dFloat32 pivot1 = dAbs(tmp[j][i]);
				if (pivot1 > pivot) 
				{
					permute = j;
					pivot = pivot1;
				}
			}
			
			if (permute != i) 
			{
				dAssert(pivot > dFloat32(1.0e-6f));
				dSwap(ret[i], ret[permute]);
				dSwap(tmp[i], tmp[permute]);
			}
		}

		for (dInt32 j = i + 1; j < 4; j++) 
		{
			dVector scale(tmp[j][i] / tmp[i][i]);
			tmp[j] -= tmp[i] * scale;
			ret[j] -= ret[i] * scale.GetScalar();
			tmp[j][i] = dFloat32(0.0f);
		}
	}

	for (dInt32 i = 3; i >= 0; i--) 
	{
		dVector pivot(tmp[i] * ret);
		ret[i] = (ret[i] - pivot.AddHorizontal().GetScalar() + tmp[i][i] * ret[i]) / tmp[i][i];
	}

	return ret;
}

void dMatrix::CalcPitchYawRoll (dVector& euler0, dVector& euler1) const
{
	const dMatrix& matrix = *this;
	dAssert (matrix[2].DotProduct(matrix[0].CrossProduct(matrix[1])).GetScalar() > 0.0f);
	dAssert (dAbs (matrix[2].DotProduct(matrix[0].CrossProduct(matrix[1])).GetScalar() - dFloat32 (1.0f)) < dFloat32 (1.0e-4f));

	// Assuming the angles are in radians.
	if (matrix[0][2] > dFloat32 (0.99995f)) 
	{
		dFloat32 picth0 = -dAtan2(matrix[2][1], matrix[1][1]);
		dFloat32 yaw0 = -dFloat32(dPi * dFloat32 (0.5f));
		dFloat32 roll0 = dFloat32(0.0f);

		euler0[0] = picth0;
		euler0[1] = yaw0;
		euler0[2] = roll0;

		euler1[0] = picth0;
		euler1[1] = yaw0;
		euler1[2] = roll0;
		//dMatrix xxxx(dPitchMatrix(picth0) * dYawMatrix(yaw0) * dRollMatrix(roll0));
		//dMatrix xxxx1(dPitchMatrix(picth0) * dYawMatrix(yaw0) * dRollMatrix(roll0));
	} 
	else if (matrix[0][2] < dFloat32 (-0.99995f)) 
	{
		dFloat32 picth0 = -dAtan2(matrix[2][1], matrix[1][1]);
		dFloat32 yaw0 = dFloat32(dPi * dFloat32(0.5f));
		dFloat32 roll0 = dFloat32(0.0f);
		euler0[0] = picth0;
		euler0[1] = yaw0;
		euler0[2] = roll0;

		euler1[0] = picth0;
		euler1[1] = yaw0;
		euler1[2] = roll0;
		//dMatrix xxxx(dPitchMatrix(picth0) * dYawMatrix(yaw0) * dRollMatrix(roll0));
		//dMatrix xxxx1(dPitchMatrix(picth0) * dYawMatrix(yaw0) * dRollMatrix(roll0));
	} 
	else 
	{
		dFloat32 yaw0 = -dAsin(matrix[0][2]);
		dFloat32 yaw1 = dFloat32(dPi) - yaw0;

		dFloat32 picth0 = dAtan2( matrix[1][2],  matrix[2][2]);
		dFloat32 picth1 = dAtan2(-matrix[1][2], -matrix[2][2]);

		dFloat32 roll0 = dAtan2( matrix[0][1],  matrix[0][0]);
		dFloat32 roll1 = dAtan2(-matrix[0][1], -matrix[0][0]);

		if (yaw1 > dFloat32 (dPi)) 
		{
			yaw1 -= dFloat32 (2.0f * dPi);
		}

		euler0[0] = picth0;
		euler0[1] = yaw0;
		euler0[2] = roll0;

		euler1[0] = picth1;
		euler1[1] = yaw1;
		euler1[2] = roll1;
	}

	euler0[3] = dFloat32(0.0f);
	euler1[3] = dFloat32(0.0f);

#ifdef _DEBUG
	dMatrix m0 (dPitchMatrix (euler0[0]) * dYawMatrix(euler0[1]) * dRollMatrix(euler0[2]));
	dMatrix m1 (dPitchMatrix (euler1[0]) * dYawMatrix(euler1[1]) * dRollMatrix(euler1[2]));
	for (dInt32 i = 0; i < 3; i ++) 
	{
		for (dInt32 j = 0; j < 3; j ++) 
		{
			dFloat32 error = dAbs (m0[i][j] - matrix[i][j]);
			dAssert (error < 5.0e-2f);
			error = dAbs (m1[i][j] - matrix[i][j]);
			dAssert (error < 5.0e-2f);
		}
	}
#endif
}

void dMatrix::PolarDecomposition (dMatrix& transformMatrix, dVector& scale, dMatrix& stretchAxis) const
{
	// a polar decomposition decompose matrix A = O * S
	// where S = sqrt (transpose (L) * L)

	const dMatrix& me = *this;
	dFloat32 sign = dSign (me[2].DotProduct(me[0].CrossProduct(me[1])).GetScalar());
	stretchAxis = me * Transpose();
	scale = stretchAxis.EigenVectors();

	// I need to deal with by seeing of some of the Scale are duplicated
	// do this later (maybe by a given rotation around the non uniform axis but I do not know if it will work)
	// for now just us the matrix

	scale[0] = sign * dSqrt (scale[0]);
	scale[1] = sign * dSqrt (scale[1]);
	scale[2] = sign * dSqrt (scale[2]);
	scale[3] = dFloat32 (0.0f);

	dMatrix scaledAxis;
	scaledAxis[0] = stretchAxis[0].Scale (dFloat32 (1.0f) / scale[0]);
	scaledAxis[1] = stretchAxis[1].Scale (dFloat32 (1.0f) / scale[1]);
	scaledAxis[2] = stretchAxis[2].Scale (dFloat32 (1.0f) / scale[2]);
	scaledAxis[3] = stretchAxis[3];
	dMatrix symetricInv (stretchAxis.Transpose() * scaledAxis);

	transformMatrix = symetricInv * (*this);
	transformMatrix.m_posit = m_posit;
}

#if 0
dVector dMatrix::EigenVectors()
{
	dMatrix& mat = *this;
	dMatrix eigenVectors(dGetIdentityMatrix());

	dVector d(mat[0][0], mat[1][1], mat[2][2], dFloat32(0.0f));
	dVector b(d);
	for (dInt32 i = 0; i < 50; i++) {
		dFloat32 sm = mat[0][1] * mat[0][1] + mat[0][2] * mat[0][2] + mat[1][2] * mat[1][2];
		if (sm < dFloat32(1.0e-12f)) {
			// check the eigenvalue vectors	
			//dVector tmp(eigenVectors.m_front.CrossProduct(eigenVectors.m_up));
			//if (tmp.DotProduct(eigenVectors.m_right).GetScalar() < dFloat32(0.0f)) {
			//	eigenVectors.m_right = eigenVectors.m_right * dVector::m_negOne;
			//}
			dAssert (eigenVectors[0].DotProduct(eigenVectors[1].CrossProduct(eigenVectors[2])).GetScalar() > dFloat32 (0.0f));
			break;
		}

		dFloat32 thresh = dFloat32(0.0f);
		if (i < 3) {
			thresh = (dFloat32)(0.2f / 9.0f) * sm;
		}

		dVector z(dVector::m_zero);
		for (dInt32 ip = 0; ip < 2; ip++) {
			for (dInt32 iq = ip + 1; iq < 3; iq++) {
				dFloat32 g = dFloat32(100.0f) * dAbs(mat[ip][iq]);
				if ((i > 3) && ((dAbs(d[ip]) + g) == dAbs(d[ip])) && ((dAbs(d[iq]) + g) == dAbs(d[iq]))) {
					mat[ip][iq] = dFloat32(0.0f);
				} else if (dAbs(mat[ip][iq]) > thresh) {

					dFloat32 t;
					dFloat32 h = d[iq] - d[ip];
					if (dAbs(h) + g == dAbs(h)) {
						t = mat[ip][iq] / h;
					} else {
						dFloat32 theta = dFloat32(0.5f) * h / mat[ip][iq];
						t = dFloat32(1.0f) / (dAbs(theta) + dSqrt(dFloat32(1.0f) + theta * theta));
						if (theta < dFloat32(0.0f)) {
							t = -t;
						}
					}
					dFloat32 c = dRsqrt(dFloat32(1.0f) + t * t);
					dFloat32 s = t * c;
					dFloat32 tau = s / (dFloat32(1.0f) + c);
					h = t * mat[ip][iq];
					z[ip] -= h;
					z[iq] += h;
					d[ip] -= h;
					d[iq] += h;
					mat[ip][iq] = dFloat32(0.0f);

					for (dInt32 j = 0; j <= ip - 1; j++) {
						dFloat32 g0 = mat[j][ip];
						dFloat32 h0 = mat[j][iq];
						mat[j][ip] = g0 - s * (h0 + g0 * tau);
						mat[j][iq] = h0 + s * (g0 - h0 * tau);
					}
					for (dInt32 j = ip + 1; j <= iq - 1; j++) {
						dFloat32 g0 = mat[ip][j];
						dFloat32 h0 = mat[j][iq];
						mat[ip][j] = g0 - s * (h0 + g0 * tau);
						mat[j][iq] = h0 + s * (g0 - h0 * tau);
					}
					for (dInt32 j = iq + 1; j < 3; j++) {
						dFloat32 g0 = mat[ip][j];
						dFloat32 h0 = mat[iq][j];
						mat[ip][j] = g0 - s * (h0 + g0 * tau);
						mat[iq][j] = h0 + s * (g0 - h0 * tau);
					}

					dVector sv(s);
					dVector tauv(tau);
					dVector gv(eigenVectors[ip]);
					dVector hv(eigenVectors[iq]);
					eigenVectors[ip] -= sv * (hv + gv * tauv);
					eigenVectors[iq] += sv * (gv - hv * tauv);
				}
			}
		}

		b += z;
		d = b;
	}

	*this = eigenVectors;
	return d;
}

#else
dVector dMatrix::EigenVectors ()
{
	dMatrix matrix (*this);
	dMatrix eigenVectors(dGetIdentityMatrix());

#if 0
	if (dAbs(m_front.m_z) > dFloat32(1.0e-6f)) {
		// calculate initial guess by convert to tridiagonal matrix using householder
		// but this fail since it changes the oder of the Eigen values and Eigen vectors
		dVector u(m_front);
		u.m_x = dFloat32(0.0f);
		dVector v(dVector::m_zero);
		v.m_y = dSqrt(u.DotProduct(u).GetScalar());
		dVector w(u - v);
		w = w.Normalize();
		eigenVectors = dMatrix(w, w);
		dMatrix ident(dGetIdentityMatrix());
		eigenVectors[0] = ident[0] - eigenVectors[0] * dVector::m_two;
		eigenVectors[1] = ident[1] - eigenVectors[1] * dVector::m_two;
		eigenVectors[2] = ident[2] - eigenVectors[2] * dVector::m_two;
		matrix = eigenVectors * matrix * eigenVectors;
	}
	matrix[0][2] = dFloat32(0.0f);
	matrix[2][0] = dFloat32(0.0f);
#endif

	// QR algorithm is really bad at converging matrices with very different eigenvalue. 
	// the solution is to use RD with double shift which I do not feel like implementing. 
	// using Jacobi diagonalize instead
	dVector d (matrix[0][0], matrix[1][1], matrix[2][2], dFloat32 (0.0f)); 
	dVector b (d);
	for (dInt32 i = 0; i < 50; i++) 
	{
		dFloat32 sm = matrix[0][1] * matrix[0][1] + matrix[0][2] * matrix[0][2] + matrix[1][2] * matrix[1][2];
		if (sm < dFloat32 (1.0e-12f)) 
		{
			// make sure the the Eigen vectors are orthonormal
			//dVector tmp (eigenVectors.m_front.CrossProduct(eigenVectors.m_up));
			//if (tmp.DotProduct(eigenVectors.m_right).GetScalar() < dFloat32(0.0f)) {
			//	eigenVectors.m_right = eigenVectors.m_right * dVector::m_negOne;
			//}
			dAssert (eigenVectors[0].DotProduct(eigenVectors[1].CrossProduct(eigenVectors[2])).GetScalar() > dFloat32 (0.0f));
			break;
		}

		dFloat32 thresh = dFloat32 (0.0f);
		if (i < 3) 
		{
			thresh = (dFloat32)(0.2f / 9.0f) * sm;
		}

		dVector z (dVector::m_zero);
		for (dInt32 j = 0; j < 2; j ++) 
		{
			for (dInt32 k = j + 1; k < 3; k ++) 
			{
				dFloat32 g = dFloat32 (100.0f) * dAbs(matrix[j][k]);
				if ((i > 3) && ((dAbs(d[j]) + g) == dAbs(d[j])) && ((dAbs(d[k]) + g) == dAbs(d[k]))) 
				{
					matrix[j][k] = dFloat32 (0.0f);
				} 
				else if (dAbs(matrix[j][k]) > thresh) 
				{
					dFloat32 t;
					dFloat32 h = d[k] - d[j];
					if (dAbs(h) + g == dAbs(h)) 
					{
						t = matrix[j][k] / h;
					} 
					else 
					{
						dFloat32 theta = dFloat32 (0.5f) * h / matrix[j][k];
						t = dFloat32(1.0f) / (dAbs(theta) + dSqrt(dFloat32(1.0f) + theta * theta));
						if (theta < dFloat32 (0.0f)) 
						{
							t = -t;
						}
					}
					dFloat32 c = dRsqrt (dFloat32 (1.0f) + t * t); 
					dFloat32 s = t * c; 
					dFloat32 tau = s / (dFloat32(1.0f) + c); 
					h = t * matrix[j][k];
					z[j] -= h; 
					z[k] += h; 
					d[j] -= h; 
					d[k] += h;
					matrix[j][k] = dFloat32(0.0f);

					for (dInt32 n = 0; n <= j - 1; n ++) 
					{
						dFloat32 g0 = matrix[n][j]; 
						dFloat32 h0 = matrix[n][k]; 
						matrix[n][j] = g0 - s * (h0 + g0 * tau); 
						matrix[n][k] = h0 + s * (g0 - h0 * tau);
					}
					for (dInt32 n = j + 1; n <= k - 1; n ++) 
					{
						dFloat32 g0 = matrix[j][n]; 
						dFloat32 h0 = matrix[n][k]; 
						matrix[j][n] = g0 - s * (h0 + g0 * tau); 
						matrix[n][k] = h0 + s * (g0 - h0 * tau);
					}
					for (dInt32 n = k + 1; n < 3; n ++) 
					{
						dFloat32 g0 = matrix[j][n]; 
						dFloat32 h0 = matrix[k][n]; 
						matrix[j][n] = g0 - s * (h0 + g0 * tau); 
						matrix[k][n] = h0 + s * (g0 - h0 * tau);
					}

					dVector sv (s);
					dVector tauv (tau);
					dVector gv (eigenVectors[j]);
					dVector hv (eigenVectors[k]);
					eigenVectors[j] -= sv * (hv + gv * tauv); 
					eigenVectors[k] += sv * (gv - hv * tauv);
				}
			}
		}

		b += z; 
		d = b; 
	}

	#ifdef _DEBUG___
		dMatrix diag(dGetIdentityMatrix());
		diag[0][0] = d[0];
		diag[1][1] = d[1];
		diag[2][2] = d[2];
		dMatrix E(eigenVectors.Transpose());
		dMatrix originalMatrix(*this);
		dMatrix tempMatrix(E * diag * E.Transpose());
		for (dInt32 j = 0; j < 3; j++) 
		{
			for (dInt32 k = 0; k < 3; k++) 
			{
				dAssert(dAreEqual(originalMatrix[j][k], tempMatrix[j][k], dFloat32(1.0e-3f)));
			}
		}
	#endif

	*this = eigenVectors;
	return d;
}
#endif

