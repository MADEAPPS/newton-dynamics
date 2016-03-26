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

#include "dStdAfxMath.h"
#include "dMathDefines.h"
#include "dVector.h"
#include "dMatrix.h"
#include "dQuaternion.h"

// calculate an orthonormal matrix with the front vector pointing on the 
// dir direction, and the up and right are determined by using the GramSchidth procedure


dMatrix dGetIdentityMatrix()
{
	return dMatrix (dVector (1.0f, 0.0f, 0.0f, 0.0f),
					dVector (0.0f, 1.0f, 0.0f, 0.0f),
					dVector (0.0f, 0.0f, 1.0f, 0.0f),
					dVector (0.0f, 0.0f, 0.0f, 1.0f));
}

dMatrix dGetZeroMatrix ()
{
	return dMatrix (dVector (0.0f, 0.0f, 0.0f, 0.0f),
					dVector (0.0f, 0.0f, 0.0f, 0.0f),
					dVector (0.0f, 0.0f, 0.0f, 0.0f),
					dVector (0.0f, 0.0f, 0.0f, 0.0f));
}



dMatrix dGrammSchmidt(const dVector& dir)
{
	dVector up;
	dVector right;
	dVector front (dir); 

	front = front.Scale(1.0f / dSqrt (front % front));
	if (dAbs (front.m_z) > 0.577f) {
		right = front * dVector (-front.m_y, front.m_z, 0.0f);
	} else {
		right = front * dVector (-front.m_y, front.m_x, 0.0f);
	}
	right = right.Scale (1.0f / dSqrt (right % right));
	up = right * front;

	front.m_w = 0.0f;
	up.m_w = 0.0f;
	right.m_w = 0.0f;
	return dMatrix (front, up, right, dVector (0.0f, 0.0f, 0.0f, 1.0f));
}


dMatrix dPitchMatrix(dFloat ang)
{
	dFloat cosAng;
	dFloat sinAng;
	sinAng = dSin (ang);
	cosAng = dCos (ang);
	return dMatrix (dVector (1.0f,    0.0f,    0.0f, 0.0f), 
					dVector (0.0f,  cosAng,  sinAng, 0.0f),
					dVector (0.0f, -sinAng,  cosAng, 0.0f), 
					dVector (0.0f,    0.0f,    0.0f, 1.0f)); 

}

dMatrix dYawMatrix(dFloat ang)
{
	dFloat cosAng;
	dFloat sinAng;
	sinAng = dSin (ang);
	cosAng = dCos (ang);
	return dMatrix (dVector (cosAng, 0.0f, -sinAng, 0.0f), 
					dVector (0.0f,   1.0f,    0.0f, 0.0f), 
					dVector (sinAng, 0.0f,  cosAng, 0.0f), 
					dVector (0.0f,   0.0f,    0.0f, 1.0f)); 
}

dMatrix dRollMatrix(dFloat ang)
{
	dFloat cosAng;
	dFloat sinAng;
	sinAng = dSin (ang);
	cosAng = dCos (ang);
	return dMatrix (dVector ( cosAng, sinAng, 0.0f, 0.0f), 
					dVector (-sinAng, cosAng, 0.0f, 0.0f),
					dVector (   0.0f,   0.0f, 1.0f, 0.0f), 
					dVector (   0.0f,   0.0f, 0.0f, 1.0f)); 
}																		 



dMatrix::dMatrix (const dQuaternion &rotation, const dVector &position)
{

	dFloat x2 = dFloat (2.0f) * rotation.m_q1 * rotation.m_q1;
	dFloat y2 = dFloat (2.0f) * rotation.m_q2 * rotation.m_q2;
	dFloat z2 = dFloat (2.0f) * rotation.m_q3 * rotation.m_q3;
#ifdef _DEBUG
	dFloat w2 = dFloat (2.0f) * rotation.m_q0 * rotation.m_q0;
	dAssert (dAbs (w2 + x2 + y2 + z2 - dFloat(2.0f)) < dFloat (1.e-2f));
#endif

	dFloat xy = dFloat (2.0f) * rotation.m_q1 * rotation.m_q2;
	dFloat xz = dFloat (2.0f) * rotation.m_q1 * rotation.m_q3;
	dFloat xw = dFloat (2.0f) * rotation.m_q1 * rotation.m_q0;
	dFloat yz = dFloat (2.0f) * rotation.m_q2 * rotation.m_q3;
	dFloat yw = dFloat (2.0f) * rotation.m_q2 * rotation.m_q0;
	dFloat zw = dFloat (2.0f) * rotation.m_q3 * rotation.m_q0;

	m_front = dVector (dFloat(1.0f) - y2 - z2, xy + zw				 , xz - yw				  , dFloat(0.0f));
	m_up    = dVector (xy - zw				 , dFloat(1.0f) - x2 - z2, yz + xw				  , dFloat(0.0f));
	m_right = dVector (xz + yw				 , yz - xw				 , dFloat(1.0f) - x2 - y2 , dFloat(0.0f));

	m_posit.m_x = position.m_x;
	m_posit.m_y = position.m_y;
	m_posit.m_z = position.m_z;
	m_posit.m_w = dFloat(1.0f);
}

dMatrix::dMatrix (dFloat pitch, dFloat yaw, dFloat roll, const dVector& location)
{
	dMatrix& me = *this;
	me = dPitchMatrix(pitch) * dYawMatrix(yaw) * dRollMatrix(roll);
	me.m_posit = location;
	me.m_posit.m_w = 1.0f;
}


bool dMatrix::TestIdentity() const 
{
	const dMatrix& matrix = *this;
	const dMatrix& identity = dGetIdentityMatrix();
	
	bool isIdentity = true;
	for (int i = 0; isIdentity && (i < 3); i ++) {
		isIdentity &= dAbs (matrix[3][i]) < 1.0e-4f;
		for (int j = i; isIdentity && (j < 3); j ++) {
			isIdentity &= dAbs (matrix[i][j]-identity[i][j]) < 1.0e-4f;
		}
	}
	return isIdentity;
}


bool dMatrix::TestOrthogonal() const
{
	dVector n (m_front * m_up);
	dFloat a = m_right % m_right;
	dFloat b = m_up % m_up;
	dFloat c = m_front % m_front;
	dFloat d = n % m_right;

	return (m_front[3] == dFloat (0.0f)) & 
		(m_up[3] == dFloat (0.0f)) & 
		(m_right[3] == dFloat (0.0f)) & 
		(m_posit[3] == dFloat (1.0f)) &
		(dAbs(a - dFloat (1.0f)) < dFloat (1.0e-4f)) & 
		(dAbs(b - dFloat (1.0f)) < dFloat (1.0e-4f)) &
		(dAbs(c - dFloat (1.0f)) < dFloat (1.0e-4f)) &
		(dAbs(d - dFloat (1.0f)) < dFloat (1.0e-4f)); 
}


//dVector dMatrix::GetEulerAngles (dEulerAngleOrder order) const
void dMatrix::GetEulerAngles(dVector& euler0, dVector& euler1, dEulerAngleOrder order) const
{
	int a0 = (order>>8)&3;
	int a1 = (order>>4)&3;
	int a2 = (order>>0)&3;
	const dMatrix& matrix = *this;

	// Assuming the angles are in radians.
	if (matrix[a0][a2] > 0.99995f) {
		dFloat picth0 = 0.0f;
		dFloat yaw0 = -3.141592f * 0.5f;
		dFloat roll0 = - dAtan2(matrix[a2][a1], matrix[a1][a1]);
		euler0[a0] = picth0;
		euler0[a1] = yaw0;
		euler0[a2] = roll0;

		euler1[a0] = picth0;
		euler1[a1] = yaw0;
		euler1[a2] = roll0;

	} else if (matrix[a0][a2] < -0.99995f) {
		dFloat picth0 = 0.0f;
		dFloat yaw0 = 3.141592f * 0.5f;
		dFloat roll0 = dAtan2(matrix[a2][a1], matrix[a1][a1]);
		euler0[a0] = picth0;
		euler0[a1] = yaw0;
		euler0[a2] = roll0;

		euler1[a0] = picth0;
		euler1[a1] = yaw0;
		euler1[a2] = roll0;
	} else {
		//euler[a0] = -dAtan2(-matrix[a1][a2], matrix[a2][a2]);
		//euler[a1] = -dAsin ( matrix[a0][a2]);
		//euler[a2] = -dAtan2(-matrix[a0][a1], matrix[a0][a0]);

		dFloat yaw0 = -dAsin ( matrix[a0][a2]);
		dFloat yaw1 = 3.141592f - yaw0;
		dFloat sign0 = dSign(dCos (yaw0));
		dFloat sign1 = dSign(dCos (yaw1));

		dFloat picth0 = dAtan2(matrix[a1][a2] * sign0, matrix[a2][a2] * sign0);
		dFloat picth1 = dAtan2(matrix[a1][a2] * sign1, matrix[a2][a2] * sign1);

		dFloat roll0 = dAtan2(matrix[a0][a1] * sign0, matrix[a0][a0] * sign0);
		dFloat roll1 = dAtan2(matrix[a0][a1] * sign1, matrix[a0][a0] * sign1);

		if (yaw1 > 3.141592f) {
			yaw1 -= 2.0f * 3.141592f;
		}

		euler0[a0] = picth0;
		euler0[a1] = yaw0;
		euler0[a2] = roll0;

		euler1[a0] = picth1;
		euler1[a1] = yaw1;
		euler1[a2] = roll1;
	}
	euler0[3] = dFloat(0.0f);
	euler1[3] = dFloat(0.0f);

	#ifdef _DEBUG
		if (order == m_pitchYawRoll) {
			dMatrix m0 (dPitchMatrix (euler0[0]) * dYawMatrix(euler0[1]) * dRollMatrix(euler0[2]));
			dMatrix m1 (dPitchMatrix (euler1[0]) * dYawMatrix(euler1[1]) * dRollMatrix(euler1[2]));
			for (int i = 0; i < 3; i ++) {
				for (int j = 0; j < 3; j ++) {
					dFloat error = dAbs (m0[i][j] - matrix[i][j]);
					dAssert (error < 5.0e-2f);
					error = dAbs (m1[i][j] - matrix[i][j]);
					dAssert (error < 5.0e-2f);
				}
			}
		}
	#endif
}



dMatrix dMatrix::Inverse () const
{
	return dMatrix (dVector (m_front.m_x, m_up.m_x, m_right.m_x, 0.0f),
					dVector (m_front.m_y, m_up.m_y, m_right.m_y, 0.0f),
		            dVector (m_front.m_z, m_up.m_z, m_right.m_z, 0.0f),
		            dVector (- (m_posit % m_front), - (m_posit % m_up), - (m_posit % m_right), 1.0f));
}

dMatrix dMatrix::Transpose () const
{
	return dMatrix (dVector (m_front.m_x, m_up.m_x, m_right.m_x, 0.0f),
					dVector (m_front.m_y, m_up.m_y, m_right.m_y, 0.0f),
					dVector (m_front.m_z, m_up.m_z, m_right.m_z, 0.0f),
					dVector (0.0f, 0.0f, 0.0f, 1.0f));
}

dMatrix dMatrix::Transpose4X4 () const
{
	return dMatrix (dVector (m_front.m_x, m_up.m_x, m_right.m_x, m_posit.m_x),
					dVector (m_front.m_y, m_up.m_y, m_right.m_y, m_posit.m_y),
					dVector (m_front.m_z, m_up.m_z, m_right.m_z, m_posit.m_z),
					dVector (m_front.m_w, m_up.m_w, m_right.m_w, m_posit.m_w));
							
}

dVector dMatrix::RotateVector (const dVector &v) const
{
	return dVector (v.m_x * m_front.m_x + v.m_y * m_up.m_x + v.m_z * m_right.m_x,
					v.m_x * m_front.m_y + v.m_y * m_up.m_y + v.m_z * m_right.m_y,
					v.m_x * m_front.m_z + v.m_y * m_up.m_z + v.m_z * m_right.m_z, 0.0f);
}

dVector dMatrix::UnrotateVector (const dVector &v) const
{
	return dVector (v % m_front, v % m_up, v % m_right, 0.0f);
}

dVector dMatrix::RotateVector4x4 (const dVector &v) const
{
	dVector tmp;
	const dMatrix& me = *this;
	for (int i = 0; i < 4; i ++) {
		tmp[i] = v[0] * me[0][i] + v[1] * me[1][i] +  v[2] * me[2][i] +  v[3] * me[3][i];
	}
	return tmp;
}


dVector dMatrix::TransformVector (const dVector &v) const
{
	return m_posit + RotateVector(v);
}

dVector dMatrix::UntransformVector (const dVector &v) const
{
	return UnrotateVector(v - m_posit);
}


void dMatrix::TransformTriplex (dFloat* const dst, int dstStrideInBytes, const dFloat* const src, int srcStrideInBytes, int count) const
{
	dstStrideInBytes /= sizeof (dFloat);
	srcStrideInBytes /= sizeof (dFloat);
	for (int i = 0 ; i < count; i ++ ) {
		dFloat x = src[srcStrideInBytes * i + 0];
		dFloat y = src[srcStrideInBytes * i + 1];
		dFloat z = src[srcStrideInBytes * i + 2];
		dst[dstStrideInBytes * i + 0] = x * m_front.m_x + y * m_up.m_x + z * m_right.m_x + m_posit.m_x;
		dst[dstStrideInBytes * i + 1] = x * m_front.m_y + y * m_up.m_y + z * m_right.m_y + m_posit.m_y;
		dst[dstStrideInBytes * i + 2] = x * m_front.m_z + y * m_up.m_z + z * m_right.m_z + m_posit.m_z;
	}
}

#ifndef _NEWTON_USE_DOUBLE
void dMatrix::TransformTriplex (dFloat64* const dst, int dstStrideInBytes, const dFloat64* const src, int srcStrideInBytes, int count) const
{
	dstStrideInBytes /= sizeof (dFloat64);
	srcStrideInBytes /= sizeof (dFloat64);
	for (int i = 0 ; i < count; i ++ ) {
		dFloat64 x = src[srcStrideInBytes * i + 0];
		dFloat64 y = src[srcStrideInBytes * i + 1];
		dFloat64 z = src[srcStrideInBytes * i + 2];
		dst[dstStrideInBytes * i + 0] = x * m_front.m_x + y * m_up.m_x + z * m_right.m_x + m_posit.m_x;
		dst[dstStrideInBytes * i + 1] = x * m_front.m_y + y * m_up.m_y + z * m_right.m_y + m_posit.m_y;
		dst[dstStrideInBytes * i + 2] = x * m_front.m_z + y * m_up.m_z + z * m_right.m_z + m_posit.m_z;
	}
}
#endif

dMatrix dMatrix::operator* (const dMatrix &B) const
{
	const dMatrix& A = *this;
	return dMatrix (dVector (A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0] + A[0][3] * B[3][0],
							 A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1] + A[0][3] * B[3][1],
							 A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2] + A[0][3] * B[3][2],
	                         A[0][0] * B[0][3] + A[0][1] * B[1][3] + A[0][2] * B[2][3] + A[0][3] * B[3][3]),
					dVector (A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0] + A[1][3] * B[3][0],
						     A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1] + A[1][3] * B[3][1],
							 A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2] + A[1][3] * B[3][2],
							 A[1][0] * B[0][3] + A[1][1] * B[1][3] + A[1][2] * B[2][3] + A[1][3] * B[3][3]),
					dVector (A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0] + A[2][3] * B[3][0],
							 A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1] + A[2][3] * B[3][1],
							 A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2] + A[2][3] * B[3][2],
							 A[2][0] * B[0][3] + A[2][1] * B[1][3] + A[2][2] * B[2][3] + A[2][3] * B[3][3]),
					dVector (A[3][0] * B[0][0] + A[3][1] * B[1][0] + A[3][2] * B[2][0] + A[3][3] * B[3][0],
							 A[3][0] * B[0][1] + A[3][1] * B[1][1] + A[3][2] * B[2][1] + A[3][3] * B[3][1],
							 A[3][0] * B[0][2] + A[3][1] * B[1][2] + A[3][2] * B[2][2] + A[3][3] * B[3][2],
							 A[3][0] * B[0][3] + A[3][1] * B[1][3] + A[3][2] * B[2][3] + A[3][3] * B[3][3]));
}





dVector dMatrix::TransformPlane (const dVector &localPlane) const
{
	dVector tmp (RotateVector (localPlane));  
	tmp.m_w = localPlane.m_w - (localPlane % UnrotateVector (m_posit));  
	return tmp;  
}

dVector dMatrix::UntransformPlane (const dVector &globalPlane) const
{
	dVector tmp (UnrotateVector (globalPlane));
	tmp.m_w = globalPlane % m_posit + globalPlane.m_w;
	return tmp;
}

bool dMatrix::SanityCheck() const
{
	dVector right (m_front * m_up);
	if (dAbs (right % m_right) < 0.9999f) {
		return false;
	}
	if (dAbs (m_right.m_w) > 0.0f) {
		return false;
	}
	if (dAbs (m_up.m_w) > 0.0f) {
		return false;
	}
	if (dAbs (m_right.m_w) > 0.0f) {
		return false;
	}

	if (dAbs (m_posit.m_w) != 1.0f) {
		return false;
	}

	return true;
}


dMatrix dMatrix::Inverse4x4 () const
{
	const dFloat tol = 1.0e-4f;
	dMatrix tmp (*this);
	dMatrix inv (dGetIdentityMatrix());
	for (int i = 0; i < 4; i ++) {
		dFloat diag = tmp[i][i];
		if (dAbs (diag) < tol) {
			int j = 0;
			for (j = i + 1; j < 4; j ++) {
				dFloat val = tmp[j][i];
				if (dAbs (val) > tol) {
					break;
				}
			}
			dAssert (j < 4);
			for (int k = 0; k < 4; k ++) {
				tmp[i][k] += tmp[j][k];
				inv[i][k] += inv[j][k];
			}
			diag = tmp[i][i];
		}
		dFloat invDiag = 1.0f / diag;
		for (int j = 0; j < 4; j ++) {
			tmp[i][j] *= invDiag;
			inv[i][j] *= invDiag;
		}
		tmp[i][i] = 1.0f;

		
		for (int j = 0; j < 4; j ++) {
			if (j != i) {
				dFloat pivot = tmp[j][i];
				for (int k = 0; k < 4; k ++) {
					tmp[j][k] -= pivot * tmp[i][k];
					inv[j][k] -= pivot * inv[i][k];
				}
				tmp[j][i] = 0.0f;
			}
		}
	}
	return inv;
}




#if 0
class XXXX 
{
public:
	XXXX ()
	{
		dMatrix s;
		dFloat m = 2.0f;
		for (int i = 0; i < 3; i ++) {
			for (int j = 0; j < 3; j ++) {
				s[i][j] = m; 
				m += (i + 1) + j;
			}
		}
		s.m_posit = dVector (1, 2, 3, 1);
		dMatrix matrix;
		dVector scale;
		dMatrix stretch;
		s.PolarDecomposition (matrix, scale, stretch);
		dMatrix s1 (matrix, scale, stretch);

		dMatrix xxx (dPitchMatrix(30.0f * 3.14159f/180.0f) * dRollMatrix(30.0f * 3.14159f/180.0f));
		dMatrix xxxx (GetIdentityMatrix());
		xxx[0] = xxx[0].Scale (-1.0f);
		dFloat mmm = (xxx[0] * xxx[1]) % xxx[2];
		xxxx[0][0] = 3.0f;
		xxxx[1][1] = 3.0f;
		xxxx[2][2] = 4.0f;

		dMatrix xxx2 (xxx * xxxx);
		mmm = (xxx2[0] * xxx2[1]) % xxx2[2];
		xxx2.PolarDecomposition (matrix, scale, stretch);

		s1 = dMatrix (matrix, scale, stretch);
		s1 = dMatrix (matrix, scale, stretch);

	}
};
XXXX xxx;
#endif


static inline void ROT(dMatrix &a, int i, int j, int k, int l, dFloat s, dFloat tau) 
{
	dFloat g;
	dFloat h;
	g = a[i][j]; 
	h = a[k][l]; 
	a[i][j] = g - s * (h + g * tau); 
	a[k][l] = h + s * (g - h * tau);
}

// from numerical recipes in c
// Jacobian method for computing the eigenvectors of a symmetric matrix
dMatrix dMatrix::JacobiDiagonalization (dVector &eigenValues, const dMatrix& initialMatrix) const
{
	dFloat thresh;
	dFloat b[3];
	dFloat z[3];
	dFloat d[3];
	dFloat EPSILON = 1.0e-5f;

	dMatrix mat (*this);
	dMatrix eigenVectors (initialMatrix);
	
	b[0] = mat[0][0]; 
	b[1] = mat[1][1];
	b[2] = mat[2][2];

	d[0] = mat[0][0]; 
	d[1] = mat[1][1]; 
	d[2] = mat[2][2]; 

	z[0] = 0.0f;
	z[1] = 0.0f;
	z[2] = 0.0f;

	int nrot = 0;
	for (int i = 0; i < 50; i++) {
		dFloat sm = dAbs(mat[0][1]) + dAbs(mat[0][2]) + dAbs(mat[1][2]);

		if (sm < EPSILON * 1e-5) {
			dAssert (dAbs((eigenVectors.m_front % eigenVectors.m_front) - 1.0f) <EPSILON);
			dAssert (dAbs((eigenVectors.m_up % eigenVectors.m_up) - 1.0f) < EPSILON);
			dAssert (dAbs((eigenVectors.m_right % eigenVectors.m_right) - 1.0f) < EPSILON);
			eigenValues = dVector (d[0], d[1], d[2], dFloat (0.0f));
			return eigenVectors.Inverse();
		}

		if (i < 3) {
			thresh = (dFloat)(0.2f / 9.0f) * sm;
		}	else {
			thresh = 0.0;
		}


		// First row
		dFloat g = 100.0f * dAbs(mat[0][1]);
		if ((i > 3) && (dAbs(d[0]) + g == dAbs(d[0])) && (dAbs(d[1]) + g == dAbs(d[1]))) {
			mat[0][1] = 0.0f;
		} else if (dAbs(mat[0][1]) > thresh) {
			dFloat t;
			dFloat h = d[1] - d[0];
			if (dAbs(h) + g == dAbs(h)) {
				t = mat[0][1] / h;
			} else {
				dFloat theta = dFloat (0.5f) * h / mat[0][1];
				t = dFloat(1.0f) / (dAbs(theta) + dSqrt(dFloat(1.0f) + theta * theta));
				if (theta < 0.0f) {
					t = -t;
				}
			}
			dFloat c = dFloat(1.0f) / dSqrt (1.0f + t * t); 
			dFloat s = t * c; 
			dFloat tau = s / (dFloat(1.0f) + c); 
			h = t * mat[0][1];
			z[0] -= h; 
			z[1] += h; 
			d[0] -= h; 
			d[1] += h;
			mat[0][1] = 0.0f;
			ROT (mat, 0, 2, 1, 2, s, tau); 
			ROT (eigenVectors, 0, 0, 0, 1, s, tau); 
			ROT (eigenVectors, 1, 0, 1, 1, s, tau); 
			ROT (eigenVectors, 2, 0, 2, 1, s, tau); 

			nrot++;
		}


		// second row
		g = 100.0f * dAbs(mat[0][2]);
		if ((i > 3) && (dAbs(d[0]) + g == dAbs(d[0])) && (dAbs(d[2]) + g == dAbs(d[2]))) {
			mat[0][2] = 0.0f;
		} else if (dAbs(mat[0][2]) > thresh) {
			dFloat t;
			dFloat h = d[2] - d[0];
			if (dAbs(h) + g == dAbs(h)) {
				t = (mat[0][2]) / h;
			}	else {
				dFloat theta = dFloat (0.5f) * h / mat[0][2];
				t = dFloat(1.0f) / (dAbs(theta) + dSqrt(dFloat(1.0f) + theta * theta));
				if (theta < 0.0f) {
					t = -t;
				}
			}
			dFloat c = dFloat(1.0f) / dSqrt(1 + t * t); 
			dFloat s = t * c; 
			dFloat tau = s / (dFloat(1.0f) + c); 
			h = t * mat[0][2];
			z[0] -= h; 
			z[2] += h; 
			d[0] -= h; 
			d[2] += h;
			mat[0][2]=0.0;
			ROT (mat, 0, 1, 1, 2, s, tau); 
			ROT (eigenVectors, 0, 0, 0, 2, s, tau); 
			ROT (eigenVectors, 1, 0, 1, 2, s, tau); 
			ROT (eigenVectors, 2, 0, 2, 2, s, tau); 
		}

		// trird row
		g = 100.0f * dAbs(mat[1][2]);
		if ((i > 3) && (dAbs(d[1]) + g == dAbs(d[1])) && (dAbs(d[2]) + g == dAbs(d[2]))) {
			mat[1][2] = 0.0f;
		} else if (dAbs(mat[1][2]) > thresh) {
			dFloat t;
			dFloat h = d[2] - d[1];
			if (dAbs(h) + g == dAbs(h)) {
				t = mat[1][2] / h;
			}	else {
				dFloat theta = dFloat (0.5f) * h / mat[1][2];
				t = dFloat(1.0f) / (dAbs(theta) + dSqrt(dFloat(1.0f) + theta * theta));
				if (theta < 0.0f) {
					t = -t;
				}
			}
			dFloat c = dFloat(1.0f) / dSqrt(1 + t*t); 
			dFloat s = t * c; 
			dFloat tau = s / (dFloat(1.0f) + c); 

			h = t * mat[1][2];
			z[1] -= h; 
			z[2] += h; 
			d[1] -= h; 
			d[2] += h;
			mat[1][2] = 0.0f;
			ROT (mat, 0, 1, 0, 2, s, tau); 
			ROT (eigenVectors, 0, 1, 0, 2, s, tau); 
			ROT (eigenVectors, 1, 1, 1, 2, s, tau); 
			ROT (eigenVectors, 2, 1, 2, 2, s, tau); 
			nrot++;
		}

		b[0] += z[0]; d[0] = b[0]; z[0] = 0.0f;
		b[1] += z[1]; d[1] = b[1]; z[1] = 0.0f;
		b[2] += z[2]; d[2] = b[2]; z[2] = 0.0f;
	}

	dAssert (0);
	eigenValues = dVector (d[0], d[1], d[2], dFloat (0.0f));
	return dGetIdentityMatrix();
} 	




//void dMatrix::PolarDecomposition (dMatrix& orthogonal, dMatrix& symetric) const
void dMatrix::PolarDecomposition (dMatrix& transformMatrix, dVector& scale, dMatrix& stretchAxis, const dMatrix& initialStretchAxis) const
{
	// a polar decomposition decompose matrix A = O * S
	// where S = sqrt (transpose (L) * L)

	// calculate transpose (L) * L 
	dMatrix LL ((*this) * Transpose());


	// check is this si a pure uniformScale * rotation * translation
	dFloat det2 = (LL[0][0] + LL[1][1] + LL[2][2]) * (1.0f / 3.0f);

	dFloat invdet2 = 1.0f / det2;

	dMatrix pureRotation (LL);
	pureRotation[0] = pureRotation[0].Scale (invdet2);
	pureRotation[1] = pureRotation[1].Scale (invdet2);
	pureRotation[2] = pureRotation[2].Scale (invdet2);

	//dFloat soureSign = ((*this)[0] * (*this)[1]) % (*this)[2];
	dFloat sign = ((((*this)[0] * (*this)[1]) % (*this)[2]) > 0.0f) ? 1.0f : -1.0f;
	dFloat det = (pureRotation[0] * pureRotation[1]) % pureRotation[2];
	if (dAbs (det - 1.0f) < 1.e-5f){
		// this is a pure scale * rotation * translation
		det = sign * dSqrt (det2);
		scale[0] = det;
		scale[1] = det;
		scale[2] = det;
		scale[3] = 1.0f;
		det = 1.0f/ det;
		transformMatrix.m_front = m_front.Scale (det);
		transformMatrix.m_up = m_up.Scale (det);
		transformMatrix.m_right = m_right.Scale (det);
		transformMatrix[0][3] = 0.0f;
		transformMatrix[1][3] = 0.0f;
		transformMatrix[2][3] = 0.0f;
		transformMatrix.m_posit = m_posit;
		stretchAxis = dGetIdentityMatrix();
		
	} else {
		stretchAxis = LL.JacobiDiagonalization(scale, initialStretchAxis);

		// I need to deal with buy seeing of some of the Scale are duplicated
		// do this later (maybe by a given rotation around the non uniform axis but I do not know if it will work)
		// for now just us the matrix

		scale[0] = sign * dSqrt (scale[0]);
		scale[1] = sign * dSqrt (scale[1]);
		scale[2] = sign * dSqrt (scale[2]);
		scale[3] = 1.0f;

		dMatrix scaledAxis;
		scaledAxis[0] = stretchAxis[0].Scale (1.0f / scale[0]);
		scaledAxis[1] = stretchAxis[1].Scale (1.0f / scale[1]);
		scaledAxis[2] = stretchAxis[2].Scale (1.0f / scale[2]);
		scaledAxis[3] = stretchAxis[3];
		dMatrix symetricInv (stretchAxis.Transpose() * scaledAxis);

		transformMatrix = symetricInv * (*this);
		transformMatrix.m_posit = m_posit;
	}
}

dMatrix::dMatrix (const dMatrix& transformMatrix, const dVector& scale, const dMatrix& stretchAxis)
{
	dMatrix scaledAxis;
	scaledAxis[0] = stretchAxis[0].Scale (scale[0]);
	scaledAxis[1] = stretchAxis[1].Scale (scale[1]);
	scaledAxis[2] = stretchAxis[2].Scale (scale[2]);
	scaledAxis[3] = stretchAxis[3];

	*this = stretchAxis.Transpose() * scaledAxis * transformMatrix;
}






