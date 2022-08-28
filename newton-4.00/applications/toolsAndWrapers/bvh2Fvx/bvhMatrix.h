#pragma once
#include "bvhVector.h"

class bvhMatrix
{
	public:

	bvhMatrix();
	~bvhMatrix();

	//bvhMatrix(const float* const array);
	//bvhMatrix(const bvhVector &front, const bvhVector &up, const bvhVector &right, const bvhVector &posit);
	//bvhMatrix(const ndQuaternion &rotation, const bvhVector &position);

	//// create a orthonormal normal vector basis, front become m_front vector, and m_up and m_right are mutualiperpendicular to fron and to each other
	//bvhMatrix(const bvhVector &front);
	//
	//// create a covariance Matrix = transpose(p) * q 
	//bvhMatrix(const bvhVector& p, const bvhVector& q);
	
	bvhVector& operator[] (int i);
	const bvhVector& operator[] (int i) const;
	
	//bvhMatrix Inverse() const;
	//bvhMatrix Inverse4x4() const;
	//bvhMatrix Transpose() const;
	//bvhMatrix Transpose4X4() const;
	//bvhVector RotateVector(const bvhVector &v) const;
	//bvhVector UnrotateVector(const bvhVector &v) const;
	//bvhVector TransformVector(const bvhVector &v) const;
	//bvhVector UntransformVector(const bvhVector &v) const;
	//ndPlane TransformPlane(const ndPlane &localPlane) const;
	//ndPlane UntransformPlane(const ndPlane &globalPlane) const;
	//bvhVector TransformVector1x4(const bvhVector &v) const;
	//bvhVector SolveByGaussianElimination(const bvhVector &v) const;
	//void TransformBBox(const bvhVector& p0local, const bvhVector& p1local, bvhVector& p0, bvhVector& p1) const;
	//
	//void CalcPitchYawRoll(bvhVector& euler0, bvhVector& euler1) const;
	//void TransformTriplex(
	//	float* const dst, int dstStrideInBytes,
	//	const float* const src, int srcStrideInBytes, int count) const;
	//
	//bool TestIdentity() const;
	//bool TestSymetric3x3() const;
	//bool TestOrthogonal(float tol = float(1.0e-4f)) const;
	//
	//bvhMatrix Multiply3X3(const bvhMatrix &B) const;
	//bvhMatrix operator* (const bvhMatrix &B) const;
	//
	//// these function can only be called when bvhMatrix is a PDS matrix
	////void EigenVectors ();
	//bvhVector EigenVectors();
	//void PolarDecomposition(bvhMatrix& transformMatrix, bvhVector& scale, bvhMatrix& stretchAxis) const;
	//
	//// constructor for polar composition
	//bvhMatrix(const bvhMatrix& transformMatrix, const bvhVector& scale, const bvhMatrix& stretchAxis);

	bvhVector m_front;
	bvhVector m_up;
	bvhVector m_right;
	bvhVector m_posit;
};


inline bvhMatrix::bvhMatrix()
	:m_front(1.0f, 0.0f, 0.0f, 0.0f)
	,m_up(0.0f, 0.0f, 1.0f, 0.0f)
	,m_right(0.0f, 1.0f, 0.0f, 0.0f)
	,m_posit(1.0f, 0.0f, 0.0f, 1.0f)
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
