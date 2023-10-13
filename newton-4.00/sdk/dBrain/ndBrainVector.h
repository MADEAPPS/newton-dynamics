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

	void InitGaussianWeights(ndBrainFloat variance);

	void Set(ndBrainFloat value);
	void Set(const ndBrainVector& data);

	
	ndBrainFloat Dot(const ndBrainVector& a) const;

	void Scale(ndBrainFloat b);
	void Add(const ndBrainVector& a);
	void Sub(const ndBrainVector& a);
	void Mul(const ndBrainVector& a);

	void FlushToZero();
	void Clamp(ndBrainFloat min, ndBrainFloat max);
	void ScaleSet(const ndBrainVector& a, ndBrainFloat b);
	void ScaleAdd(const ndBrainVector& a, ndBrainFloat b);
	void Blend(const ndBrainVector& target, ndBrainFloat blend);
	void MulAdd(const ndBrainVector& a, const ndBrainVector& b);
	void MulSub(const ndBrainVector& a, const ndBrainVector& b);

	ndInt32 ArgMax() const;
	void CalculateMeanAndDeviation(ndBrainFloat& mean, ndBrainFloat& deviation) const;

	void GaussianNormalize();
	void CategoricalSample(const ndBrainVector& probability, ndBrainFloat beta = ndBrainFloat(0.5f));
};

class ndBrainMemVector: public ndBrainVector
{
	public:
	ndBrainMemVector();
	ndBrainMemVector(const ndBrainFloat* const mem, ndInt32 size);
	~ndBrainMemVector();

	void SetSize(ndInt32 size);
	void Swap(ndBrainMemVector& src);
	void SetPointer(ndBrainFloat* const memmory);
};

template<ndInt32 size>
class ndBrainFixSizeVector: public ndBrainMemVector
{
	public:
	ndBrainFixSizeVector();
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
