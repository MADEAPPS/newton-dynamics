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

enum QUAT_INDEX
{
	X_INDEX = 0,
	Y_INDEX = 1,
	Z_INDEX = 2
};
static QUAT_INDEX QIndex[] = { Y_INDEX, Z_INDEX, X_INDEX };

ndQuaternion::ndQuaternion(const ndMatrix& matrix)
{
	ndFloat32 trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
	if (trace > ndFloat32(0.0f))
	{
		trace = ndSqrt(trace + ndFloat32(1.0f));
		m_w = ndFloat32(0.5f) * trace;
		trace = ndFloat32(0.5f) / trace;
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

		trace = ndFloat32(1.0f) + matrix[i][i] - matrix[j][j] - matrix[k][k];
		trace = ndSqrt(trace);

		ndFloat32* const ptr = &m_x;
		ptr[i] = ndFloat32(0.5f) * trace;
		trace = ndFloat32(0.5f) / trace;
		m_w = (matrix[j][k] - matrix[k][j]) * trace;
		ptr[j] = (matrix[i][j] + matrix[j][i]) * trace;
		ptr[k] = (matrix[i][k] + matrix[k][i]) * trace;
	}

#ifdef _DEBUG

	ndMatrix tmp (*this, matrix.m_posit);
	ndMatrix unitMatrix (tmp * matrix.Inverse());
	for (ndInt32 i = 0; i < 4; i ++) 
	{
		ndFloat32 err = dAbs (unitMatrix[i][i] - ndFloat32(1.0f));
		dAssert (err < ndFloat32 (1.0e-2f));
	}

	ndFloat32 err = dAbs (DotProduct(*this).GetScalar() - ndFloat32(1.0f));
	dAssert (err < ndFloat32(ndEpsilon * 100.0f));
#endif
}

ndQuaternion::ndQuaternion (const ndVector &unitAxis, ndFloat32 angle)
{
	angle *= ndFloat32 (0.5f);
	m_w = ndCos (angle);
	ndFloat32 sinAng = ndSin (angle);

#ifdef _DEBUG
	if (dAbs (angle) > ndFloat32(ndEpsilon / 10.0f)) {
		dAssert (dAbs (ndFloat32(1.0f) - unitAxis.DotProduct(unitAxis & ndVector::m_triplexMask).GetScalar()) < ndFloat32(ndEpsilon * 10.0f));
	} 
#endif
	m_x = unitAxis.m_x * sinAng;
	m_y = unitAxis.m_y * sinAng;
	m_z = unitAxis.m_z * sinAng;
}

ndQuaternion ndQuaternion::operator* (const ndQuaternion &q) const
{
	//return ndQuaternion(
	//	q.m_x * m_w + q.m_w * m_x - q.m_z * m_y + q.m_y * m_z,
	//	q.m_y * m_w + q.m_z * m_x + q.m_w * m_y - q.m_x * m_z,
	//	q.m_z * m_w - q.m_y * m_x + q.m_x * m_y + q.m_w * m_z,
	//	q.m_w * m_w - q.m_x * m_x - q.m_y * m_y - q.m_z * m_z);

	const ndVector x( q.m_w,  q.m_z, -q.m_y, -q.m_x);
	const ndVector y(-q.m_z,  q.m_w,  q.m_x, -q.m_y);
	const ndVector z( q.m_y, -q.m_x,  q.m_w, -q.m_z);
	//const ndVector w( q.m_x,  q.m_y,  q.m_z,  q.m_w);
	const ndVector w(q);

//#ifdef _DEBUG
//	ndQuaternion xxx0(x * ndVector(m_x) + y * ndVector(m_y) + z * ndVector(m_z) + w * ndVector(m_w));
//	ndQuaternion xxx1(
//		q.m_x * m_w + q.m_w * m_x - q.m_z * m_y + q.m_y * m_z,
//		q.m_y * m_w + q.m_z * m_x + q.m_w * m_y - q.m_x * m_z,
//		q.m_z * m_w - q.m_y * m_x + q.m_x * m_y + q.m_w * m_z,
//		q.m_w * m_w - q.m_x * m_x - q.m_y * m_y - q.m_z * m_z);
//
//	ndFloat32 mag0 = xxx0.DotProduct(xxx0).GetScalar();
//	ndFloat32 mag1 = xxx1.DotProduct(xxx1).GetScalar();
//	ndFloat32 error = mag0 - mag1;
//	dAssert(dAbs(error) < ndFloat32(1.0e-5f));
//#endif

	return x * ndVector(m_x) + y * ndVector(m_y) + z * ndVector(m_z) + w * ndVector(m_w);
}

ndVector ndQuaternion::CalcAverageOmega (const ndQuaternion &q1, ndFloat32 invdt) const
{
	ndQuaternion q0 (*this);
	if (q0.DotProduct (q1).GetScalar() < ndFloat32 (0.0f)) 
	{
		q0 = q0.Scale(ndFloat32 (-1.0f));
	}
	ndQuaternion dq (q0.Inverse() * q1);
	ndVector omegaDir (dq.m_x, dq.m_y, dq.m_z, ndFloat32 (0.0f));

	ndFloat32 dirMag2 = omegaDir.DotProduct(omegaDir).GetScalar();
	if (dirMag2	< ndFloat32(ndFloat32 (1.0e-5f) * ndFloat32 (1.0e-5f))) 
	{
		return ndVector (ndFloat32(0.0f));
	}

	ndFloat32 dirMagInv = ndRsqrt (dirMag2);
	ndFloat32 dirMag = dirMag2 * dirMagInv;

	ndFloat32 omegaMag = ndFloat32(2.0f) * ndAtan2 (dirMag, dq.m_w) * invdt;
	return omegaDir.Scale (dirMagInv * omegaMag);
}

ndQuaternion ndQuaternion::Slerp (const ndQuaternion &q1, ndFloat32 t) const 
{
	ndQuaternion q0;

	ndFloat32 dot = DotProduct(q1).GetScalar();
	if ((dot + ndFloat32(1.0f)) > ndEpsilon) 
	{
		ndFloat32 Sclp;
		ndFloat32 Sclq;
		if (dot < (ndFloat32(1.0f) - ndEpsilon) ) 
		{
			ndFloat32 ang = ndAcos (dot);

			ndFloat32 sinAng = ndSin (ang);
			ndFloat32 den = ndFloat32(1.0f) / sinAng;

			Sclp = ndSin ((ndFloat32(1.0f) - t ) * ang) * den;
			Sclq = ndSin (t * ang) * den;
		} 
		else
		{
			Sclp = ndFloat32(1.0f) - t;
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

		ndFloat32 Sclp = ndSin ((ndFloat32(1.0f) - t) * ndPi * ndFloat32 (0.5f));
		ndFloat32 Sclq = ndSin (t * ndPi * ndFloat32 (0.5f));

		q0.m_w = m_w * Sclp + q0.m_w * Sclq;
		q0.m_x = m_x * Sclp + q0.m_x * Sclq;
		q0.m_y = m_y * Sclp + q0.m_y * Sclq;
		q0.m_z = m_z * Sclp + q0.m_z * Sclq;
	}

	dot = q0.DotProduct (q0).GetScalar();
	if (dAbs (dot - ndFloat32(1.0f)) > ndEpsilon) 
	{
		dot = ndRsqrt (dot);
		q0 = q0.Scale(dot);
	}
	return q0;
}


