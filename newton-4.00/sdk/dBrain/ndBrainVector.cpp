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

#include "ndBrainStdafx.h"
#include "ndBrainVector.h"

ndDeepBrainMemVector::ndDeepBrainMemVector()
	:ndBrainVector()
{
	m_size = 0;
	m_capacity = 0;
	m_array = nullptr;
}

ndDeepBrainMemVector::ndDeepBrainMemVector(const ndReal* const mem, ndInt32 size)
	:ndBrainVector()
{
	m_size = size;
	m_capacity = size + 1;
	m_array = (ndReal*)mem;
}

ndDeepBrainMemVector::~ndDeepBrainMemVector()
{
	m_size = 0;
	m_capacity = 0;
	m_array = nullptr;
}

void ndDeepBrainMemVector::SetSize(ndInt32 size)
{
	m_size = size;
	m_capacity = size + 1;
}

void ndDeepBrainMemVector::SetPointer(ndReal* const memmory)
{
	m_array = memmory;
}

ndBrainVector::ndBrainVector()
	:ndArray<ndReal>()
{
}

ndBrainVector::~ndBrainVector()
{
}

void ndBrainVector::InitGaussianWeights(ndReal variance)
{
	for (ndInt32 i = GetCount() - 1; i >= 0 ; --i)
	{
		(*this)[i] = ndReal(ndGaussianRandom(ndReal (0.0f), variance));
	}
}

void ndBrainVector::Set(ndReal value)
{
	for (ndInt32 i = GetCount() - 1; i >= 0; --i)
	{
		(*this)[i] = value;
	}
}

void ndBrainVector::Set(const ndBrainVector& data)
{
	ndAssert(GetCount() == data.GetCount());
	for (ndInt32 i = GetCount() - 1; i >= 0; --i)
	{
		(*this)[i] = data[i];
	}
}

ndInt32 ndBrainVector::GetMaxIndex() const
{
	ndInt32 index = 0;
	ndFloat32 maxValue = (*this)[0];
	for (ndInt32 i = GetCount() - 1; i > 0; --i)
	{
		ndFloat32 val = (*this)[i];
		if (val > maxValue)
		{
			index = i;
			maxValue = val;
		}
	}
	return index;
}

void ndBrainVector::ScaleSet(const ndBrainVector& a, ndReal scale)
{
	ndAssert(GetCount() == a.GetCount());
	ndScaleSet(GetCount(), &(*this)[0], &a[0], scale);
}

void ndBrainVector::ScaleAdd(const ndBrainVector& a, ndReal b)
{
	ndAssert(GetCount() == a.GetCount());
	ndScaleAdd(GetCount(), &(*this)[0], &(*this)[0], &a[0], b);
}

void ndBrainVector::Add(const ndBrainVector& a, const ndBrainVector& b)
{
	ndAssert(GetCount() == a.GetCount());
	ndAssert(GetCount() == b.GetCount());
	ndAdd(GetCount(), &(*this)[0], &a[0], &b[0]);
}

void ndBrainVector::Sub(const ndBrainVector& a, const ndBrainVector& b)
{
	ndAssert(GetCount() == a.GetCount());
	ndAssert(GetCount() == b.GetCount());
	ndSub(GetCount(), &(*this)[0], &a[0], &b[0]);
}

void ndBrainVector::Mul(const ndBrainVector& a, const ndBrainVector& b)
{
	ndAssert(GetCount() == a.GetCount());
	ndAssert(GetCount() == b.GetCount());
	ndMul(GetCount(), &(*this)[0], &a[0], &b[0]);
}

void ndBrainVector::MulAdd(const ndBrainVector& a, const ndBrainVector& b)
{
	ndAssert(GetCount() == a.GetCount());
	ndAssert(GetCount() == b.GetCount());
	ndMulAdd(GetCount(), &(*this)[0], &(*this)[0], &a[0], &b[0]);
}

void ndBrainVector::MulSub(const ndBrainVector& a, const ndBrainVector& b)
{
	ndAssert(GetCount() == a.GetCount());
	ndAssert(GetCount() == b.GetCount());
	ndMulSub(GetCount(), &(*this)[0], &(*this)[0], &a[0], &b[0]);
}

ndReal ndBrainVector::Dot(const ndBrainVector& a) const
{
	ndAssert(GetCount() == a.GetCount());
	return ndDotProduct(GetCount(), &(*this)[0], &a[0]);
}