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

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dMatrix.h"
#include "dQuaternion.h"

enum QUAT_INDEX
{
	X_INDEX = 0,
	Y_INDEX = 1,
	Z_INDEX = 2
};
static QUAT_INDEX QIndex[] = { Y_INDEX, Z_INDEX, X_INDEX };

dQuaternion::dQuaternion(const dMatrix& matrix)
{
	dFloat32 trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
	if (trace > dFloat32(0.0f))
	{
		trace = dSqrt(trace + dFloat32(1.0f));
		m_w = dFloat32(0.5f) * trace;
		trace = dFloat32(0.5f) / trace;
		m_x = (matrix[1][2] - matrix[2][1]) * trace;
		m_y = (matrix[2][0] - matrix[0][2]) * trace;
		m_z = (matrix[0][1] - matrix[1][0]) * trace;
	}
	else
	{
		QUAT_INDEX i = X_INDEX;
		if (matrix[Y_INDEX][Y_INDEX] > matrix[X_INDEX][X_INDEX])
		{
			i = Y_INDEX;
		}
		if (matrix[Z_INDEX][Z_INDEX] > matrix[i][i])
		{
			i = Z_INDEX;
		}
		QUAT_INDEX j = QIndex[i];
		QUAT_INDEX k = QIndex[j];

		trace = dFloat32(1.0f) + matrix[i][i] - matrix[j][j] - matrix[k][k];
		trace = dSqrt(trace);

		dFloat32* const ptr = &m_x;
		ptr[i] = dFloat32(0.5f) * trace;
		trace = dFloat32(0.5f) / trace;
		m_w = (matrix[j][k] - matrix[k][j]) * trace;
		ptr[j] = (matrix[i][j] + matrix[j][i]) * trace;
		ptr[k] = (matrix[i][k] + matrix[k][i]) * trace;
	}

#ifdef _DEBUG

	dMatrix tmp (*this, matrix.m_posit);
	dMatrix unitMatrix (tmp * matrix.Inverse());
	for (dInt32 i = 0; i < 4; i ++) 
	{
		dFloat32 err = dAbs (unitMatrix[i][i] - dFloat32(1.0f));
		dAssert (err < dFloat32 (1.0e-2f));
	}

	dFloat32 err = dAbs (DotProduct(*this).GetScalar() - dFloat32(1.0f));
	dAssert (err < dFloat32(dEpsilon * 100.0f));
#endif
}

dQuaternion::dQuaternion (const dVector &unitAxis, dFloat32 angle)
{
	angle *= dFloat32 (0.5f);
	m_w = dCos (angle);
	dFloat32 sinAng = dSin (angle);

#ifdef _DEBUG
	if (dAbs (angle) > dFloat32(dEpsilon / 10.0f)) {
		dAssert (dAbs (dFloat32(1.0f) - unitAxis.DotProduct(unitAxis & dVector::m_triplexMask).GetScalar()) < dFloat32(dEpsilon * 10.0f));
	} 
#endif
	m_x = unitAxis.m_x * sinAng;
	m_y = unitAxis.m_y * sinAng;
	m_z = unitAxis.m_z * sinAng;
}

dQuaternion dQuaternion::operator* (const dQuaternion &q) const
{
	//return dQuaternion(
	//	q.m_x * m_w + q.m_w * m_x - q.m_z * m_y + q.m_y * m_z,
	//	q.m_y * m_w + q.m_z * m_x + q.m_w * m_y - q.m_x * m_z,
	//	q.m_z * m_w - q.m_y * m_x + q.m_x * m_y + q.m_w * m_z,
	//	q.m_w * m_w - q.m_x * m_x - q.m_y * m_y - q.m_z * m_z);

	const dVector x( q.m_w,  q.m_z, -q.m_y, -q.m_x);
	const dVector y(-q.m_z,  q.m_w,  q.m_x, -q.m_y);
	const dVector z( q.m_y, -q.m_x,  q.m_w, -q.m_z);
	const dVector w( q.m_x,  q.m_y,  q.m_z,  q.m_w);

//#ifdef _DEBUG
//	dQuaternion xxx0(x * dVector(m_x) + y * dVector(m_y) + z * dVector(m_z) + w * dVector(m_w));
//	dQuaternion xxx1(
//		q.m_x * m_w + q.m_w * m_x - q.m_z * m_y + q.m_y * m_z,
//		q.m_y * m_w + q.m_z * m_x + q.m_w * m_y - q.m_x * m_z,
//		q.m_z * m_w - q.m_y * m_x + q.m_x * m_y + q.m_w * m_z,
//		q.m_w * m_w - q.m_x * m_x - q.m_y * m_y - q.m_z * m_z);
//
//	dFloat32 mag0 = xxx0.DotProduct(xxx0).GetScalar();
//	dFloat32 mag1 = xxx1.DotProduct(xxx1).GetScalar();
//	dFloat32 error = mag0 - mag1;
//	dAssert(dAbs(error) < dFloat32(1.0e-5f));
//#endif

	return x * dVector(m_x) + y * dVector(m_y) + z * dVector(m_z) + w * dVector(m_w);
}

dVector dQuaternion::CalcAverageOmega (const dQuaternion &q1, dFloat32 invdt) const
{
	dQuaternion q0 (*this);
	if (q0.DotProduct (q1).GetScalar() < dFloat32 (0.0f)) 
	{
		q0 = q0.Scale(dFloat32 (-1.0f));
	}
	dQuaternion dq (q0.Inverse() * q1);
	dVector omegaDir (dq.m_x, dq.m_y, dq.m_z, dFloat32 (0.0f));

	dFloat32 dirMag2 = omegaDir.DotProduct(omegaDir).GetScalar();
	if (dirMag2	< dFloat32(dFloat32 (1.0e-5f) * dFloat32 (1.0e-5f))) 
	{
		return dVector (dFloat32(0.0f));
	}

	dFloat32 dirMagInv = dRsqrt (dirMag2);
	dFloat32 dirMag = dirMag2 * dirMagInv;

	dFloat32 omegaMag = dFloat32(2.0f) * dAtan2 (dirMag, dq.m_w) * invdt;
	return omegaDir.Scale (dirMagInv * omegaMag);
}

dQuaternion dQuaternion::Slerp (const dQuaternion &q1, dFloat32 t) const 
{
	dQuaternion q0;

	dFloat32 dot = DotProduct(q1).GetScalar();
	if ((dot + dFloat32(1.0f)) > dEpsilon) 
	{
		dFloat32 Sclp;
		dFloat32 Sclq;
		if (dot < (dFloat32(1.0f) - dEpsilon) ) 
		{
			dFloat32 ang = dAcos (dot);

			dFloat32 sinAng = dSin (ang);
			dFloat32 den = dFloat32(1.0f) / sinAng;

			Sclp = dSin ((dFloat32(1.0f) - t ) * ang) * den;
			Sclq = dSin (t * ang) * den;
		} 
		else
		{
			Sclp = dFloat32(1.0f) - t;
			Sclq = t;
		}

		q0.m_w = m_w * Sclp + q1.m_w * Sclq;
		q0.m_x = m_x * Sclp + q1.m_x * Sclq;
		q0.m_y = m_y * Sclp + q1.m_y * Sclq;
		q0.m_z = m_z * Sclp + q1.m_z * Sclq;
	} 
	else 
	{
		q0.m_w =  m_z;
		q0.m_x = -m_y;
		q0.m_y =  m_x;
		q0.m_z =  m_w;

		dFloat32 Sclp = dSin ((dFloat32(1.0f) - t) * dPi * dFloat32 (0.5f));
		dFloat32 Sclq = dSin (t * dPi * dFloat32 (0.5f));

		q0.m_w = m_w * Sclp + q0.m_w * Sclq;
		q0.m_x = m_x * Sclp + q0.m_x * Sclq;
		q0.m_y = m_y * Sclp + q0.m_y * Sclq;
		q0.m_z = m_z * Sclp + q0.m_z * Sclq;
	}

	dot = q0.DotProduct (q0).GetScalar();
	if ((dot) < dFloat32(1.0f - dEpsilon * 10.0f) ) 
	{
		//dot = dFloat32(1.0f) / dSqrt (dot);
		dot = dRsqrt (dot);
		q0.m_w *= dot;
		q0.m_x *= dot;
		q0.m_y *= dot;
		q0.m_z *= dot;
	}
	return q0;
}


