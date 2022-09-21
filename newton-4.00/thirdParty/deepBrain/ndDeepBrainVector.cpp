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

#include "ndDeepBrainStdafx.h"
#include "ndDeepBrainVector.h"

ndDeepBrainMemVector::ndDeepBrainMemVector()
	:ndDeepBrainVector()
{
	m_size = 0;
	m_capacity = 0;
	m_array = nullptr;
}

ndDeepBrainMemVector::ndDeepBrainMemVector(const ndReal* const mem, ndInt32 size)
	:ndDeepBrainVector()
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

ndDeepBrainVector::ndDeepBrainVector()
	:ndArray<ndReal>()
{
}

ndDeepBrainVector::~ndDeepBrainVector()
{
}

void ndDeepBrainVector::InitGaussianWeights(ndReal mean, ndReal variance)
{
	for (ndInt32 i = GetCount() - 1; i >= 0 ; --i)
	{
		(*this)[i] = ndReal(ndGaussianRandom(mean, variance));
	}
}

void ndDeepBrainVector::Set(ndReal value)
{
	for (ndInt32 i = GetCount() - 1; i >= 0; --i)
	{
		(*this)[i] = value;
	}
}

void ndDeepBrainVector::Set(const ndDeepBrainVector& data)
{
	ndAssert(GetCount() == data.GetCount());
	for (ndInt32 i = GetCount() - 1; i >= 0; --i)
	{
		(*this)[i] = data[i];
	}
}

void ndDeepBrainVector::ScaleSet(const ndDeepBrainVector& a, ndReal scale)
{
	ndAssert(GetCount() == a.GetCount());
	ndScaleSet(GetCount(), &(*this)[0], &a[0], scale);
}

void ndDeepBrainVector::ScaleAdd(const ndDeepBrainVector& a, ndReal b)
{
	ndAssert(GetCount() == a.GetCount());
	ndScaleAdd(GetCount(), &(*this)[0], &(*this)[0], &a[0], b);
}

void ndDeepBrainVector::Add(const ndDeepBrainVector& a, const ndDeepBrainVector& b)
{
	ndAssert(GetCount() == a.GetCount());
	ndAssert(GetCount() == b.GetCount());
	ndAdd(GetCount(), &(*this)[0], &a[0], &b[0]);
}

void ndDeepBrainVector::Sub(const ndDeepBrainVector& a, const ndDeepBrainVector& b)
{
	ndAssert(GetCount() == a.GetCount());
	ndAssert(GetCount() == b.GetCount());
	ndSub(GetCount(), &(*this)[0], &a[0], &b[0]);
}

void ndDeepBrainVector::Mul(const ndDeepBrainVector& a, const ndDeepBrainVector& b)
{
	ndAssert(GetCount() == a.GetCount());
	ndAssert(GetCount() == b.GetCount());
	ndMul(GetCount(), &(*this)[0], &a[0], &b[0]);
}

void ndDeepBrainVector::MulAdd(const ndDeepBrainVector& a, const ndDeepBrainVector& b)
{
	ndAssert(GetCount() == a.GetCount());
	ndAssert(GetCount() == b.GetCount());
	ndMulAdd(GetCount(), &(*this)[0], &(*this)[0], &a[0], &b[0]);
}

void ndDeepBrainVector::MulSub(const ndDeepBrainVector& a, const ndDeepBrainVector& b)
{
	ndAssert(GetCount() == a.GetCount());
	ndAssert(GetCount() == b.GetCount());
	ndMulSub(GetCount(), &(*this)[0], &(*this)[0], &a[0], &b[0]);
}

ndReal ndDeepBrainVector::Dot(const ndDeepBrainVector& a) const
{
	ndAssert(GetCount() == a.GetCount());
	return ndDotProduct(GetCount(), &(*this)[0], &a[0]);
}