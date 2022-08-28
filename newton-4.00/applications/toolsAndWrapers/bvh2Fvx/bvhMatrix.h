#pragma once
#include "bvhVector.h"

class bvhMatrix
{
	public:

	bvhMatrix();
	~bvhMatrix();
	bvhMatrix(const bvhVector &front, const bvhVector &up, const bvhVector &right, const bvhVector &posit);
	
	bvhVector& operator[] (int i);
	const bvhVector& operator[] (int i) const;

	bvhMatrix Inverse () const;
	bvhVector CalcPitchYawRoll() const;
	bvhVector UntransformVector(const bvhVector &v) const;
	
	bvhMatrix operator* (const bvhMatrix &B) const;

	bvhVector m_front;
	bvhVector m_up;
	bvhVector m_right;
	bvhVector m_posit;
};


inline bvhMatrix::bvhMatrix()
	:m_front(1.0f, 0.0f, 0.0f, 0.0f)
	,m_up   (0.0f, 1.0f, 0.0f, 0.0f)
	,m_right(0.0f, 0.0f, 1.0f, 0.0f)
	,m_posit(0.0f, 0.0f, 0.0f, 1.0f)
{
}

inline bvhMatrix::bvhMatrix(const bvhVector &front, const bvhVector &up, const bvhVector &right, const bvhVector &posit)
	:m_front(front)
	,m_up(up)
	,m_right(right)
	,m_posit(posit)
{
}

inline bvhMatrix::~bvhMatrix()
{
}

inline bvhVector& bvhMatrix::operator[] (int i)
{
	return (&m_front)[i];
}

inline const bvhVector& bvhMatrix::operator[] (int i) const
{
	return (&m_front)[i];
}

inline bvhMatrix bvhMatrix::Inverse() const
{
	bvhMatrix matrix;
	const bvhMatrix& me = *this;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			matrix[i][j] = me[j][i];
		}
	}
	matrix.m_posit = (matrix[0].Scale(m_posit.m_x) + matrix[1].Scale(m_posit.m_y) + matrix[2].Scale(m_posit.m_z)).Scale (-1.0f);
	matrix.m_posit.m_w = 1.0f;
	return matrix;
}

inline bvhVector bvhMatrix::CalcPitchYawRoll() const
{
	const bvhMatrix& matrix = *this;
	//ndAssert(matrix[2].DotProduct(matrix[0].CrossProduct(matrix[1])).GetScalar() > 0.0f);
	//ndAssert(ndAbs(matrix[2].DotProduct(matrix[0].CrossProduct(matrix[1])).GetScalar() - float(1.0f)) < float(1.0e-4f));

	bvhVector euler0;
	bvhVector euler1;
	float ndPi = 3.14159265f;
	// Assuming the angles are in radians.
	if (matrix[0][2] > float(0.99995f))
	{
		float picth0 = -atan2(matrix[2][1], matrix[1][1]);
		float yaw0 = -float(ndPi * float(0.5f));
		float roll0 = float(0.0f);

		euler0[0] = picth0;
		euler0[1] = yaw0;
		euler0[2] = roll0;

		euler1[0] = picth0;
		euler1[1] = yaw0;
		euler1[2] = roll0;
		//bvhMatrix xxxx(ndPitchMatrix(picth0) * ndYawMatrix(yaw0) * ndRollMatrix(roll0));
		//bvhMatrix xxxx1(ndPitchMatrix(picth0) * ndYawMatrix(yaw0) * ndRollMatrix(roll0));
	}
	else if (matrix[0][2] < float(-0.99995f))
	{
		float picth0 = -atan2(matrix[2][1], matrix[1][1]);
		float yaw0 = float(ndPi * float(0.5f));
		float roll0 = float(0.0f);
		euler0[0] = picth0;
		euler0[1] = yaw0;
		euler0[2] = roll0;

		euler1[0] = picth0;
		euler1[1] = yaw0;
		euler1[2] = roll0;
		//bvhMatrix xxxx(ndPitchMatrix(picth0) * ndYawMatrix(yaw0) * ndRollMatrix(roll0));
		//bvhMatrix xxxx1(ndPitchMatrix(picth0) * ndYawMatrix(yaw0) * ndRollMatrix(roll0));
	}
	else
	{
		float yaw0 = -asinf(matrix[0][2]);
		float yaw1 = float(ndPi) - yaw0;

		float picth0 = atan2(matrix[1][2], matrix[2][2]);
		float picth1 = atan2(-matrix[1][2], -matrix[2][2]);

		float roll0 = atan2(matrix[0][1], matrix[0][0]);
		float roll1 = atan2(-matrix[0][1], -matrix[0][0]);

		if (yaw1 > float(ndPi))
		{
			yaw1 -= float(2.0f * ndPi);
		}

		euler0[0] = picth0;
		euler0[1] = yaw0;
		euler0[2] = roll0;

		euler1[0] = picth1;
		euler1[1] = yaw1;
		euler1[2] = roll1;
	}

	euler0[3] = float(0.0f);
	euler1[3] = float(0.0f);
	return euler0;
}

inline bvhVector bvhMatrix::UntransformVector(const bvhVector &v) const
{
	bvhVector out;
	bvhVector v1(v - m_posit);
	const bvhMatrix& me = *this;
	out.m_x = v1.DotProduct(me[0]);
	out.m_y = v1.DotProduct(me[1]);
	out.m_z = v1.DotProduct(me[2]);
	out.m_w = 1.0f;
	return out;
}

inline bvhMatrix bvhMatrix::operator* (const bvhMatrix &B) const
{
	bvhMatrix matrix;
	const bvhMatrix& me = *this;
	for (int i = 0; i < 4; i++)
	{
		bvhVector row(B[0][i], B[1][i], B[2][i], B[3][i]);
		for (int j = 0; j < 4; j++)
		{
			matrix[j][i] = row.DotProduct(me[j]);
		}
	}
	return matrix;
}

inline bvhMatrix ndPitchMatrix(float ang)
{
	float sinAng = sinf(ang);
	float cosAng = cosf(ang);
	return bvhMatrix(
		bvhVector(float(1.0f), float(0.0f), float(0.0f), float(0.0f)),
		bvhVector(float(0.0f), cosAng, sinAng, float(0.0f)),
		bvhVector(float(0.0f), -sinAng, cosAng, float(0.0f)),
		bvhVector(float(0.0f), float(0.0f), float(0.0f), float(1.0f)));
}

inline bvhMatrix ndYawMatrix(float ang)
{
	float sinAng = sinf(ang);
	float cosAng = cosf(ang);
	return bvhMatrix(
		bvhVector(cosAng, float(0.0f), -sinAng, float(0.0f)),
		bvhVector(float(0.0f), float(1.0f), float(0.0f), float(0.0f)),
		bvhVector(sinAng, float(0.0f), cosAng, float(0.0f)),
		bvhVector(float(0.0f), float(0.0f), float(0.0f), float(1.0f)));
}

inline bvhMatrix ndRollMatrix(float ang)
{
	float sinAng = sinf(ang);
	float cosAng = cosf(ang);
	return bvhMatrix(bvhVector(cosAng, sinAng, float(0.0f), float(0.0f)),
		bvhVector(-sinAng, cosAng, float(0.0f), float(0.0f)),
		bvhVector(float(0.0f), float(0.0f), float(1.0f), float(0.0f)),
		bvhVector(float(0.0f), float(0.0f), float(0.0f), float(1.0f)));
}

