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

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndMatrix.h"
#include "ndQuaternion.h"
#include "ndGeneralMatrix.h"

ndMatrix::ndMatrix (const ndMatrix& transformMatrix, const ndVector& scale, const ndMatrix& stretchAxis)
{
	const ndMatrix scaledAxis(
		stretchAxis[0].Scale(scale[0]),
		stretchAxis[1].Scale(scale[1]),
		stretchAxis[2].Scale(scale[2]),
		stretchAxis[3]);
	*this = stretchAxis.Transpose3x3() * scaledAxis * transformMatrix;
}

ndMatrix ndMatrix::Multiply3X3 (const ndMatrix &B) const
{
	return ndMatrix (B.m_front * m_front.BroadcastX() + B.m_up * m_front.BroadcastY() + B.m_right * m_front.BroadcastZ(), 
					 B.m_front * m_up.BroadcastX()    + B.m_up * m_up.BroadcastY()    + B.m_right * m_up.BroadcastZ(), 
					 B.m_front * m_right.BroadcastX() + B.m_up * m_right.BroadcastY() + B.m_right * m_right.BroadcastZ(), 
					 ndVector::m_wOne); 
}

ndMatrix ndMatrix::operator* (const ndMatrix &B) const
{
	return ndMatrix (B.m_front * m_front.BroadcastX() + B.m_up * m_front.BroadcastY() + B.m_right * m_front.BroadcastZ() + B.m_posit * m_front.BroadcastW(), 
					 B.m_front * m_up.BroadcastX()    + B.m_up * m_up.BroadcastY()    + B.m_right * m_up.BroadcastZ()    + B.m_posit * m_up.BroadcastW(), 
					 B.m_front * m_right.BroadcastX() + B.m_up * m_right.BroadcastY() + B.m_right * m_right.BroadcastZ() + B.m_posit * m_right.BroadcastW(), 
					 B.m_front * m_posit.BroadcastX() + B.m_up * m_posit.BroadcastY() + B.m_right * m_posit.BroadcastZ() + B.m_posit * m_posit.BroadcastW()); 
}

void ndMatrix::TransformTriplex (ndFloat32* const dst, ndInt32 dstStrideInBytes, const ndFloat32* const src, ndInt32 srcStrideInBytes, ndInt32 count) const
{
	ndInt32 dstStride = ndInt32 (dstStrideInBytes /sizeof (ndFloat32));
	ndInt32 srcStride = ndInt32 (srcStrideInBytes / sizeof (ndFloat32));

	ndInt32 dstIndex = 0;
	ndInt32 srcIndex = 0;
	for (ndInt32 i = 0 ; i < count; i ++ ) 
	{
		ndFloat32 x = src[srcIndex + 0];
		ndFloat32 y = src[srcIndex + 1];
		ndFloat32 z = src[srcIndex + 2];
		srcIndex += srcStride;
		dst[dstIndex + 0] = x * m_front.m_x + y * m_up.m_x + z * m_right.m_x + m_posit.m_x;
		dst[dstIndex + 1] = x * m_front.m_y + y * m_up.m_y + z * m_right.m_y + m_posit.m_y;
		dst[dstIndex + 2] = x * m_front.m_z + y * m_up.m_z + z * m_right.m_z + m_posit.m_z;
		dstIndex += dstStride;
	}
}

#ifndef D_NEWTON_USE_DOUBLE
void ndMatrix::TransformTriplex (ndFloat64* const dst, ndInt32 dstStrideInBytes, const ndFloat64* const src, ndInt32 srcStrideInBytes, ndInt32 count) const
{
	ndInt32 dstStride = ndInt32 (dstStrideInBytes /sizeof (ndFloat64));
	ndInt32 srcStride = ndInt32 (srcStrideInBytes / sizeof (ndFloat64));

	ndInt32 dstIndex = 0;
	ndInt32 srcIndex = 0;
	for (ndInt32 i = 0 ; i < count; i ++ ) 
	{
		ndFloat64 x = src[srcIndex + 0];
		ndFloat64 y = src[srcIndex + 1];
		ndFloat64 z = src[srcIndex + 2];
		srcIndex += srcStride;
		dst[dstIndex + 0] = x * m_front.m_x + y * m_up.m_x + z * m_right.m_x + m_posit.m_x;
		dst[dstIndex + 1] = x * m_front.m_y + y * m_up.m_y + z * m_right.m_y + m_posit.m_y;
		dst[dstIndex + 2] = x * m_front.m_z + y * m_up.m_z + z * m_right.m_z + m_posit.m_z;
		dstIndex += dstStride;
	}
}

void ndMatrix::TransformTriplex (ndFloat64* const dst, ndInt32 dstStrideInBytes, const ndFloat32* const src, ndInt32 srcStrideInBytes, ndInt32 count) const
{
	ndInt32 dstStride = ndInt32 (dstStrideInBytes /sizeof (ndFloat64));
	ndInt32 srcStride = ndInt32 (srcStrideInBytes / sizeof (ndFloat32));

	ndInt32 dstIndex = 0;
	ndInt32 srcIndex = 0;
	for (ndInt32 i = 0 ; i < count; i ++ ) 
	{
		ndFloat64 x = src[srcIndex + 0];
		ndFloat64 y = src[srcIndex + 1];
		ndFloat64 z = src[srcIndex + 2];
		srcIndex += srcStride;
		dst[dstIndex + 0] = x * m_front.m_x + y * m_up.m_x + z * m_right.m_x + m_posit.m_x;
		dst[dstIndex + 1] = x * m_front.m_y + y * m_up.m_y + z * m_right.m_y + m_posit.m_y;
		dst[dstIndex + 2] = x * m_front.m_z + y * m_up.m_z + z * m_right.m_z + m_posit.m_z;
		dstIndex += dstStride;
	}
}
#endif

void ndMatrix::TransformBBox (const ndVector& p0local, const ndVector& p1local, ndVector& p0, ndVector& p1) const
{
	const ndMatrix& matrix = *this;
	ndVector size ((p1local - p0local) * ndVector::m_half);
	ndVector center (TransformVector ((p1local + p0local) * ndVector::m_half));
	ndVector extends (size.m_x * ndAbs(matrix[0][0]) + size.m_y * ndAbs(matrix[1][0]) + size.m_z * ndAbs(matrix[2][0]),  
					  size.m_x * ndAbs(matrix[0][1]) + size.m_y * ndAbs(matrix[1][1]) + size.m_z * ndAbs(matrix[2][1]),  
	                  size.m_x * ndAbs(matrix[0][2]) + size.m_y * ndAbs(matrix[1][2]) + size.m_z * ndAbs(matrix[2][2]), ndFloat32 (0.0f));  

	p0 = center - extends;
	p1 = center + extends;
}

ndMatrix ndMatrix::Inverse4x4 () const
{
	ndMatrix tmp (*this);
	ndMatrix inv (ndGetIdentityMatrix());
	for (ndInt32 i = 0; i < 4; ++i) 
	{
		ndFloat32 pivot = ndAbs(tmp[i][i]);
		if (pivot < ndFloat32(0.01f)) 
		{
			ndInt32 permute = i;
			for (ndInt32 j = i + 1; j < 4; ++j) 
			{
				ndFloat32 pivot1 = ndAbs(tmp[j][i]);
				if (pivot1 > pivot) 
				{
					permute = j;
					pivot = pivot1;
				}
			}
			if (permute != i) 
			{
				ndAssert(pivot > ndFloat32(0.0f));
				ndAssert((pivot > ndFloat32(1.0e-6f)) || (ndConditionNumber(4, 4, (ndFloat32*)&(*this)[0][0]) < ndFloat32(1.0e5f)));
				ndSwap(inv[i], inv[permute]);
				ndSwap(tmp[i], tmp[permute]);
			}
		}

		for (ndInt32 j = i + 1; j < 4; ++j) 
		{
			ndVector scale (tmp[j][i] / tmp[i][i]);
			tmp[j] -= tmp[i] * scale;
			inv[j] -= inv[i] * scale;
			tmp[j][i] = ndFloat32 (0.0f);
		}
	}

	for (ndInt32 i = 3; i >= 0; --i) 
	{
		ndVector acc(ndVector::m_zero);
		for (ndInt32 j = i + 1; j < 4; ++j) 
		{
			ndVector pivot(tmp[i][j]);
			acc += pivot * inv[j];
		}
		ndVector den(ndFloat32(1.0f) / tmp[i][i]);
		inv[i] = den * (inv[i] - acc);
	}

#ifdef _DEBUG
	tmp = *this * inv;
	for (ndInt32 i = 0; i < 4; ++i) 
	{
		ndFloat32 error = tmp[i][i] - ndFloat32(1.0f);
		ndAssert(ndAbs(error) < ndFloat32(1.0e-3f));
		for (ndInt32 j = i + 1; j < 4; ++j) 
		{
			ndAssert(ndAbs(tmp[i][j]) < ndFloat32(1.0e-3f));
			ndAssert(ndAbs(tmp[j][i]) < ndFloat32(1.0e-3f));
		}
	}
#endif

	return inv;
}

ndVector ndMatrix::SolveByGaussianElimination(const ndVector &v) const
{
	ndMatrix tmp(*this);
	ndVector ret(v);
	for (ndInt32 i = 0; i < 4; ++i) 
	{
		ndFloat32 pivot = ndAbs(tmp[i][i]);
		if (pivot < ndFloat32(0.01f)) 
		{
			ndInt32 permute = i;
			for (ndInt32 j = i + 1; j < 4; ++j) 
			{
				ndFloat32 pivot1 = ndAbs(tmp[j][i]);
				if (pivot1 > pivot) 
				{
					permute = j;
					pivot = pivot1;
				}
			}
			
			if (permute != i) 
			{
				ndAssert(pivot > ndFloat32(1.0e-6f));
				ndSwap(ret[i], ret[permute]);
				ndSwap(tmp[i], tmp[permute]);
			}
		}

		for (ndInt32 j = i + 1; j < 4; ++j) 
		{
			const ndVector scale(tmp[j][i] / tmp[i][i]);
			tmp[j] -= tmp[i] * scale;
			ret[j] -= ret[i] * scale.GetScalar();
			tmp[j][i] = ndFloat32(0.0f);
		}
	}

	for (ndInt32 i = 3; i >= 0; --i) 
	{
		const ndVector pivot(tmp[i] * ret);
		ret[i] = (ret[i] - pivot.AddHorizontal().GetScalar() + tmp[i][i] * ret[i]) / tmp[i][i];
	}

	return ret;
}

ndVector ndMatrix::CalcPitchYawRoll (ndVector& euler1) const
{
	const ndMatrix& matrix = *this;
	ndAssert (matrix[2].DotProduct(matrix[0].CrossProduct(matrix[1])).GetScalar() > 0.0f);
	ndAssert (ndAbs (matrix[2].DotProduct(matrix[0].CrossProduct(matrix[1])).GetScalar() - ndFloat32 (1.0f)) < ndFloat32 (1.0e-4f));

	euler1 = ndVector::m_zero;
	ndVector euler0(ndVector::m_zero);

	// Assuming the angles are in radians.
	if (matrix[0][2] > ndFloat32 (0.99995f)) 
	{
		ndFloat32 picth0 = -ndAtan2(matrix[2][1], matrix[1][1]);
		ndFloat32 yaw0 = -ndFloat32(ndPi * ndFloat32 (0.5f));
		ndFloat32 roll0 = ndFloat32(0.0f);

		euler0[0] = picth0;
		euler0[1] = yaw0;
		euler0[2] = roll0;

		euler1[0] = picth0;
		euler1[1] = yaw0;
		euler1[2] = roll0;
	} 
	else if (matrix[0][2] < ndFloat32 (-0.99995f)) 
	{
		ndFloat32 picth0 = -ndAtan2(matrix[2][1], matrix[1][1]);
		ndFloat32 yaw0 = ndFloat32(ndPi * ndFloat32(0.5f));
		ndFloat32 roll0 = ndFloat32(0.0f);
		euler0[0] = picth0;
		euler0[1] = yaw0;
		euler0[2] = roll0;

		euler1[0] = picth0;
		euler1[1] = yaw0;
		euler1[2] = roll0;
	} 
	else 
	{
		ndFloat32 yaw0 = -ndAsin(matrix[0][2]);
		ndFloat32 yaw1 = ndFloat32(ndPi) - yaw0;

		ndFloat32 picth0 = ndAtan2( matrix[1][2],  matrix[2][2]);
		ndFloat32 picth1 = ndAtan2(-matrix[1][2], -matrix[2][2]);

		ndFloat32 roll0 = ndAtan2( matrix[0][1],  matrix[0][0]);
		ndFloat32 roll1 = ndAtan2(-matrix[0][1], -matrix[0][0]);

		if (yaw1 > ndFloat32 (ndPi)) 
		{
			yaw1 -= ndFloat32 (2.0f * ndPi);
		}

		euler0[0] = picth0;
		euler0[1] = yaw0;
		euler0[2] = roll0;

		euler1[0] = picth1;
		euler1[1] = yaw1;
		euler1[2] = roll1;
	}

	euler0[3] = ndFloat32(0.0f);
	euler1[3] = ndFloat32(0.0f);

#ifdef _DEBUG
	ndMatrix m0 (ndPitchMatrix (euler0[0]) * ndYawMatrix(euler0[1]) * ndRollMatrix(euler0[2]));
	ndMatrix m1 (ndPitchMatrix (euler1[0]) * ndYawMatrix(euler1[1]) * ndRollMatrix(euler1[2]));
	for (ndInt32 i = 0; i < 3; ++i) 
	{
		for (ndInt32 j = 0; j < 3; ++j) 
		{
			ndFloat32 error = ndAbs (m0[i][j] - matrix[i][j]);
			ndAssert (error < 5.0e-2f);
			error = ndAbs (m1[i][j] - matrix[i][j]);
			ndAssert (error < 5.0e-2f);
		}
	}
#endif
	return euler0;
}

void ndMatrix::PolarDecomposition (ndMatrix& transformMatrix, ndVector& scale, ndMatrix& stretchAxis) const
{
	// a polar decomposition decompose matrix A = O * S
	// where S = sqrt (transpose (L) * L)

	const ndMatrix& me = *this;
	ndFloat32 sign = ndSign (me[2].DotProduct(me[0].CrossProduct(me[1])).GetScalar());
	stretchAxis = me * Transpose3x3();
	scale = stretchAxis.EigenVectors();

	// I need to deal with by seeing of some of the Scale are duplicated
	// do this later (maybe by a given rotation around the non uniform axis but I do not know if it will work)
	// for now just us the matrix

	scale[0] = sign * ndSqrt (scale[0]);
	scale[1] = sign * ndSqrt (scale[1]);
	scale[2] = sign * ndSqrt (scale[2]);
	scale[3] = ndFloat32 (0.0f);

	ndMatrix scaledAxis;
	scaledAxis[0] = stretchAxis[0].Scale (ndFloat32 (1.0f) / scale[0]);
	scaledAxis[1] = stretchAxis[1].Scale (ndFloat32 (1.0f) / scale[1]);
	scaledAxis[2] = stretchAxis[2].Scale (ndFloat32 (1.0f) / scale[2]);
	scaledAxis[3] = stretchAxis[3];
	ndMatrix symetricInv (stretchAxis.Transpose3x3() * scaledAxis);

	transformMatrix = symetricInv * (*this);
	transformMatrix.m_posit = m_posit;
}

ndVector ndMatrix::EigenVectors ()
{
	ndMatrix matrix (*this);
	ndMatrix eigenVectors(ndGetIdentityMatrix());

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
	ndVector d (matrix[0][0], matrix[1][1], matrix[2][2], ndFloat32 (0.0f)); 
	ndVector b (d);
	for (ndInt32 i = 0; i < 50; ++i) 
	{
		ndFloat32 sm = matrix[0][1] * matrix[0][1] + matrix[0][2] * matrix[0][2] + matrix[1][2] * matrix[1][2];
		if (sm < ndFloat32 (1.0e-12f)) 
		{
			// make sure the the Eigen vectors are orthonormal
			//ndVector tmp (eigenVectors.m_front.CrossProduct(eigenVectors.m_up));
			//if (tmp.DotProduct(eigenVectors.m_right).GetScalar() < ndFloat32(0.0f)) {
			//	eigenVectors.m_right = eigenVectors.m_right * ndVector::m_negOne;
			//}
			ndAssert (eigenVectors[0].DotProduct(eigenVectors[1].CrossProduct(eigenVectors[2])).GetScalar() > ndFloat32 (0.0f));
			break;
		}

		ndFloat32 thresh = ndFloat32 (0.0f);
		if (i < 3) 
		{
			thresh = (ndFloat32)(0.2f / 9.0f) * sm;
		}

		ndVector z (ndVector::m_zero);
		for (ndInt32 j = 0; j < 2; ++j) 
		{
			for (ndInt32 k = j + 1; k < 3; ++k) 
			{
				ndFloat32 g = ndFloat32 (100.0f) * ndAbs(matrix[j][k]);
				if ((i > 3) && ((ndAbs(d[j]) + g) == ndAbs(d[j])) && ((ndAbs(d[k]) + g) == ndAbs(d[k]))) 
				{
					matrix[j][k] = ndFloat32 (0.0f);
				} 
				else if (ndAbs(matrix[j][k]) > thresh) 
				{
					ndFloat32 t;
					ndFloat32 h = d[k] - d[j];
					if (ndAbs(h) + g == ndAbs(h)) 
					{
						t = matrix[j][k] / h;
					} 
					else 
					{
						ndFloat32 theta = ndFloat32 (0.5f) * h / matrix[j][k];
						t = ndFloat32(1.0f) / (ndAbs(theta) + ndSqrt(ndFloat32(1.0f) + theta * theta));
						if (theta < ndFloat32 (0.0f)) 
						{
							t = -t;
						}
					}
					ndFloat32 c = ndRsqrt (ndFloat32 (1.0f) + t * t); 
					ndFloat32 s = t * c; 
					ndFloat32 tau = s / (ndFloat32(1.0f) + c); 
					h = t * matrix[j][k];
					z[j] -= h; 
					z[k] += h; 
					d[j] -= h; 
					d[k] += h;
					matrix[j][k] = ndFloat32(0.0f);

					for (ndInt32 n = 0; n <= j - 1; n ++) 
					{
						ndFloat32 g0 = matrix[n][j]; 
						ndFloat32 h0 = matrix[n][k]; 
						matrix[n][j] = g0 - s * (h0 + g0 * tau); 
						matrix[n][k] = h0 + s * (g0 - h0 * tau);
					}
					for (ndInt32 n = j + 1; n <= k - 1; n ++) 
					{
						ndFloat32 g0 = matrix[j][n]; 
						ndFloat32 h0 = matrix[n][k]; 
						matrix[j][n] = g0 - s * (h0 + g0 * tau); 
						matrix[n][k] = h0 + s * (g0 - h0 * tau);
					}
					for (ndInt32 n = k + 1; n < 3; n ++) 
					{
						ndFloat32 g0 = matrix[j][n]; 
						ndFloat32 h0 = matrix[k][n]; 
						matrix[j][n] = g0 - s * (h0 + g0 * tau); 
						matrix[k][n] = h0 + s * (g0 - h0 * tau);
					}

					ndVector sv (s);
					ndVector tauv (tau);
					ndVector gv (eigenVectors[j]);
					ndVector hv (eigenVectors[k]);
					eigenVectors[j] -= sv * (hv + gv * tauv); 
					eigenVectors[k] += sv * (gv - hv * tauv);
				}
			}
		}

		b += z; 
		d = b; 
	}

#ifdef _DEBUG___
		ndMatrix diag(ndGetIdentityMatrix());
		diag[0][0] = d[0];
		diag[1][1] = d[1];
		diag[2][2] = d[2];
		ndMatrix E(eigenVectors.Transpose3x3());
		ndMatrix originalMatrix(*this);
		ndMatrix tempMatrix(E * diag * E.Transpose3x3());
		tempMatrix = tempMatrix.Inverse4x4();
		ndMatrix unitMatrix(tempMatrix* originalMatrix);

		for (ndInt32 j = 0; j < 3; ++j) 
		{
			ndAssert(ndAbs(unitMatrix[j][j] - ndFloat32(1.0f)) < ndFloat32(1.0e-6f));
			for (ndInt32 k = j + 1; k < 3; ++k) 
			{
				ndAssert(ndAbs(unitMatrix[k][j]) < ndFloat32(1.0e-6f));
				ndAssert(ndAbs(unitMatrix[j][k]) < ndFloat32(1.0e-6f));
			}
		}
	#endif

	*this = eigenVectors;
	return d;
}

static ndMatrix identityMatrix(
	ndVector(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f)));

static ndMatrix zeroMatrix(
	ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f)),
	ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f)));

const ndMatrix& ndGetIdentityMatrix()
{
	return identityMatrix;
}

const ndMatrix& ndGetZeroMatrix()
{
	return zeroMatrix;
}

ndMatrix ndPitchMatrix(ndFloat32 ang)
{
	ndFloat32 sinAng = ndSin(ang);
	ndFloat32 cosAng = ndCos(ang);
	return ndMatrix(
		ndVector(ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(0.0f)),
		ndVector(ndFloat32(0.0f), cosAng, sinAng, ndFloat32(0.0f)),
		ndVector(ndFloat32(0.0f), -sinAng, cosAng, ndFloat32(0.0f)),
		ndVector::m_wOne);
}

ndMatrix ndYawMatrix(ndFloat32 ang)
{
	ndFloat32 sinAng = ndSin(ang);
	ndFloat32 cosAng = ndCos(ang);
	return ndMatrix(
		ndVector(cosAng, ndFloat32(0.0f), -sinAng, ndFloat32(0.0f)),
		ndVector(ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f), ndFloat32(0.0f)),
		ndVector(sinAng, ndFloat32(0.0f), cosAng, ndFloat32(0.0f)),
		ndVector::m_wOne);
}

ndMatrix ndRollMatrix(ndFloat32 ang)
{
	ndFloat32 sinAng = ndSin(ang);
	ndFloat32 cosAng = ndCos(ang);
	return ndMatrix(ndVector(cosAng, sinAng, ndFloat32(0.0f), ndFloat32(0.0f)),
		ndVector(-sinAng, cosAng, ndFloat32(0.0f), ndFloat32(0.0f)),
		ndVector(ndFloat32(0.0f), ndFloat32(0.0f), ndFloat32(1.0f), ndFloat32(0.0f)),
		ndVector::m_wOne);
}

ndMatrix ndCalculateMatrix(const ndQuaternion& quat, const ndVector& position)
{
	ndAssert((quat.DotProduct(quat).GetScalar() - ndFloat32(1.0f)) < ndFloat32(1.0e-4f));
	const ndQuaternion quat0(quat);
	const ndQuaternion quat1(quat0.Scale(ndFloat32(2.0f)));

	const ndFloat32 x2 = quat0.m_x * quat1.m_x;
	const ndFloat32 y2 = quat0.m_y * quat1.m_y;
	const ndFloat32 z2 = quat0.m_z * quat1.m_z;

#ifdef _DEBUG
	ndFloat32 w2 = quat0.m_w * quat1.m_w;
	ndAssert(ndAbs(w2 + x2 + y2 + z2 - ndFloat32(2.0f)) < ndFloat32(1.0e-3f));
#endif

	const ndFloat32 xy = quat0.m_x * quat1.m_y;
	const ndFloat32 xz = quat0.m_x * quat1.m_z;
	const ndFloat32 xw = quat0.m_x * quat1.m_w;
	const ndFloat32 yz = quat0.m_y * quat1.m_z;
	const ndFloat32 yw = quat0.m_y * quat1.m_w;
	const ndFloat32 zw = quat0.m_z * quat1.m_w;

	ndVector front(ndFloat32(1.0f) - y2 - z2, xy + zw, xz - yw, ndFloat32(0.0f));
	ndVector up(xy - zw, ndFloat32(1.0f) - x2 - z2, yz + xw, ndFloat32(0.0f));
	ndVector right(xz + yw, yz - xw, ndFloat32(1.0f) - x2 - y2, ndFloat32(0.0f));
	//ndVector posit (position);
	return ndMatrix(front, up, right, position);
}

ndMatrix ndCovarianceMatrix(const ndVector& p, const ndVector& q)
{
	return ndMatrix(q * p.BroadcastX(), q * p.BroadcastY(), q * p.BroadcastZ(), ndVector::m_wOne);
}

ndMatrix ndGramSchmidtMatrix(const ndVector& dir)
{
	ndVector right;
	ndVector front((dir & ndVector::m_triplexMask).Normalize());
	
	if (ndAbs(front.m_z) > ndFloat32(0.577f))
	{
		right = front.CrossProduct(ndVector(-front.m_y, front.m_z, ndFloat32(0.0f), ndFloat32(0.0f)));
	}
	else
	{
		right = front.CrossProduct(ndVector(-front.m_y, front.m_x, ndFloat32(0.0f), ndFloat32(0.0f)));
	}
	right = right.Normalize();
	ndVector up (right.CrossProduct(front));

	ndAssert(ndMatrix(front, up, right, ndVector::m_wOne).TestOrthogonal());
	return ndMatrix(front, up, right, ndVector::m_wOne);
}

ndMatrix ndMatrix::Transpose3x3() const
{
	ndMatrix inv;
	ndVector::Transpose4x4(inv[0], inv[1], inv[2], inv[3], m_front, m_up, m_right, ndVector::m_wOne);
	return inv;
}

ndMatrix ndMatrix::Transpose4X4() const
{
	ndMatrix inv;
	ndVector::Transpose4x4(inv[0], inv[1], inv[2], inv[3], m_front, m_up, m_right, m_posit);
	return inv;
}

ndVector ndMatrix::RotateVector(const ndVector& v) const
{
	return m_front * v.BroadcastX() + m_up * v.BroadcastY() + m_right * v.BroadcastZ();
}

ndVector ndMatrix::UnrotateVector(const ndVector& v) const
{
	return v.OptimizedVectorUnrotate(m_front, m_up, m_right);
}

ndVector ndMatrix::TransformVector(const ndVector& v) const
{
	return m_front * v.BroadcastX() + m_up * v.BroadcastY() + m_right * v.BroadcastZ() + m_posit;
}

ndVector ndMatrix::TransformVector1x4(const ndVector& v) const
{
	return m_front * v.BroadcastX() + m_up * v.BroadcastY() + m_right * v.BroadcastZ() + m_posit * v.BroadcastW();
}

ndVector ndMatrix::UntransformVector(const ndVector& v) const
{
	return UnrotateVector(v - m_posit) | ndVector::m_wOne;
}

ndPlane ndMatrix::TransformPlane(const ndPlane& localPlane) const
{
	return ndPlane(RotateVector(localPlane), localPlane.m_w - (localPlane.DotProduct(UnrotateVector(m_posit)).GetScalar()));
}

ndPlane ndMatrix::UntransformPlane(const ndPlane& globalPlane) const
{
	return ndPlane(UnrotateVector(globalPlane), globalPlane.Evalue(m_posit));
}

ndMatrix ndMatrix::Inverse() const
{
	ndTrace(("funtion: %s deprecated, please use ndMatrix::OrthoInverse instead", __FUNCTION__));
	ndAssert(0);
	return OrthoInverse();
}

ndMatrix ndMatrix::OrthoInverse() const
{
	ndMatrix inv;
	ndVector::Transpose4x4(inv[0], inv[1], inv[2], inv[3], m_front, m_up, m_right, ndVector::m_wOne);
	inv.m_posit -= inv[0] * m_posit.BroadcastX() + inv[1] * m_posit.BroadcastY() + inv[2] * m_posit.BroadcastZ();
	return inv;
}

bool ndMatrix::TestIdentity() const
{
	const ndMatrix& me = *this;
	for (ndInt32 i = 0; i < 4; ++i)
	{
		if (me[i][i] != ndFloat32(1.0f))
		{
			return false;
		}
		for (ndInt32 j = i + 1; j < 4; ++j)
		{
			if (me[i][j] != ndFloat32(0.0f))
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

bool ndMatrix::TestOrthogonal(ndFloat32 tol) const
{
#ifdef _DEBUG
	for (ndInt32 i = 0; i < 4; ++i)
	{
		for (ndInt32 j = 0; j < 4; ++j)
		{
			ndAssert(ndCheckFloat((*this)[i][j]));
		}
	}
#endif

	ndVector n(m_front.CrossProduct(m_up));
	ndFloat32 a = m_right.DotProduct(m_right).GetScalar();
	ndFloat32 b = m_up.DotProduct(m_up).GetScalar();
	ndFloat32 c = m_front.DotProduct(m_front).GetScalar();
	ndFloat32 d = n.DotProduct(m_right).GetScalar();
	bool ret = (m_front[3] == ndFloat32(0.0f)) &&
		(m_up[3] == ndFloat32(0.0f)) &&
		(m_right[3] == ndFloat32(0.0f)) &&
		(m_posit[3] == ndFloat32(1.0f)) &&
		(ndAbs(a - ndFloat32(1.0f)) < tol) &&
		(ndAbs(b - ndFloat32(1.0f)) < tol) &&
		(ndAbs(c - ndFloat32(1.0f)) < tol) &&
		(ndAbs(d - ndFloat32(1.0f)) < tol);
	if (!ret)
	{
		ndAssert(0);
	}
	return ret;
}

bool ndMatrix::TestSymetric3x3() const
{
	const ndMatrix& me = *this;
	return (ndAbs(me[0][1] - me[1][0]) < ndFloat32(1.0e-5f)) &&
		(ndAbs(me[0][2] - me[2][0]) < ndFloat32(1.0e-5f)) &&
		(ndAbs(me[1][2] - me[2][1]) < ndFloat32(1.0e-5f)) &&
		(me[0][3] == ndFloat32(0.0f)) &&
		(me[1][3] == ndFloat32(0.0f)) &&
		(me[2][3] == ndFloat32(0.0f)) &&
		(me[3][0] == ndFloat32(0.0f)) &&
		(me[3][1] == ndFloat32(0.0f)) &&
		(me[3][2] == ndFloat32(0.0f)) &&
		(me[3][3] == ndFloat32(1.0f));
}

bool ndMatrix::SanityCheck() const
{
	if (ndAbs(m_right.m_w) > ndFloat32(0.0f))
	{
		return false;
	}
	if (ndAbs(m_up.m_w) > ndFloat32(0.0f))
	{
		return false;
	}
	if (ndAbs(m_right.m_w) > ndFloat32(0.0f))
	{
		return false;
	}
	if (ndAbs(m_posit.m_w) != ndFloat32(1.0f))
	{
		return false;
	}

	ndVector right(m_front.CrossProduct(m_up));
	if (ndAbs(right.DotProduct(m_right).GetScalar()) < ndFloat32(0.9999f))
	{
		return false;
	}
	return true;
}
