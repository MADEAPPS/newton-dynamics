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

#ifndef _ND_BRAIN_VECTOR_H__
#define _ND_BRAIN_VECTOR_H__

#include "ndBrainStdafx.h"

class ndBrainVector: public ndArray<ndBrainFloat> 
{
	public: 
	ndBrainVector();
	ndBrainVector(const ndBrainVector& src);
	~ndBrainVector();

	ndInt64 ArgMax() const;

	void Set(ndBrainFloat value);
	void Set(const ndBrainVector& data);

	void Add(const ndBrainVector& a);
	void Sub(const ndBrainVector& a);
	void Mul(const ndBrainVector& a);
	void Min(const ndBrainVector& a);
	void Max(const ndBrainVector& a);
	void LessEqual(ndBrainFloat test);
	void GreaterEqual(ndBrainFloat test);
	void LessEqual(const ndBrainVector& a);
	void GreaterEqual(const ndBrainVector& a);

	void FlushToZero();
	void SoftMaxNormalize();
	void GaussianNormalize();
	void Clamp(ndBrainFloat min, ndBrainFloat max);
	void InitGaussianWeights(ndBrainFloat variance);

	void Min(ndBrainFloat b);
	void Max(ndBrainFloat b);
	void Scale(ndBrainFloat b);
	void ScaleSet(const ndBrainVector& a, ndBrainFloat b);
	void ScaleAdd(const ndBrainVector& a, ndBrainFloat b);
	void Blend(const ndBrainVector& target, ndBrainFloat blend);
	void Blend(const ndBrainVector& target, const ndBrainVector& blend);
	void Blend(const ndBrainVector& blend, ndBrainFloat a, ndBrainFloat b);
	void MulAdd(const ndBrainVector& a, const ndBrainVector& b);
	void MulSub(const ndBrainVector& a, const ndBrainVector& b);
	
	ndBrainFloat Dot(const ndBrainVector& a) const;
	void StandardNormalDistribution();
	void CalculateMeanAndVariance(ndBrainFloat& mean, ndBrainFloat& variance) const;
	void CategoricalSample(const ndBrainVector& probability, ndBrainFloat beta = ndBrainFloat(0.5f));
	void CalculateMeanAndDeviation____(const ndBrainVector& mean, const ndBrainVector& variance);
};

class ndBrainMemVector: public ndBrainVector
{
	public:
	ndBrainMemVector();
	ndBrainMemVector(const ndBrainFloat* const mem, ndInt64 size);
	~ndBrainMemVector();

	void SetSize(ndInt64 size);
	void Swap(ndBrainMemVector& src);
	void SetPointer(ndBrainFloat* const memmory);
};

inline ndBrainVector::ndBrainVector()
	:ndArray<ndBrainFloat>()
{
}

inline ndBrainVector::ndBrainVector(const ndBrainVector& src)
	: ndArray<ndBrainFloat>(src)
{
}

inline ndBrainVector::~ndBrainVector()
{
}

inline ndBrainMemVector::ndBrainMemVector()
	:ndBrainVector()
{
	m_size = 0;
	m_capacity = 0;
	m_array = nullptr;
}

inline ndBrainMemVector::ndBrainMemVector(const ndBrainFloat* const mem, ndInt64 size)
	:ndBrainVector()
{
	m_size = size;
	m_capacity = size + 1;
	m_array = (ndBrainFloat*)mem;
}

inline ndBrainMemVector::~ndBrainMemVector()
{
	m_size = 0;
	m_capacity = 0;
	m_array = nullptr;
}

template<ndInt32 size>
class ndBrainFixSizeVector: public ndBrainMemVector
{
	public:
	ndBrainFixSizeVector();
	ndBrainFixSizeVector(ndInt32 count);
	ndBrainFixSizeVector(const ndBrainFixSizeVector& src);
	ndBrainFixSizeVector& operator=(const ndBrainFixSizeVector& src);

	private:
	ndBrainFloat m_buffer[size];
};

template<ndInt32 size>
ndBrainFixSizeVector<size>::ndBrainFixSizeVector()
	:ndBrainMemVector(m_buffer, size)
{
}

template<ndInt32 size>
ndBrainFixSizeVector<size>::ndBrainFixSizeVector(ndInt32 count)
	: ndBrainMemVector(m_buffer, size)
{
	ndAssert(count <= size);
	SetCount(ndMin(count, size));
}

template<ndInt32 size>
ndBrainFixSizeVector<size>::ndBrainFixSizeVector(const ndBrainFixSizeVector& src)
	:ndBrainMemVector(m_buffer, size)
{
	Set(src);
}

template<ndInt32 size>
ndBrainFixSizeVector<size>& ndBrainFixSizeVector<size>::operator=(const ndBrainFixSizeVector& src)
{
	new (this) ndBrainFixSizeVector(src);
	return*this;
}

#endif 
