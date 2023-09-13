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

ndBrainMemVector::ndBrainMemVector()
	:ndBrainVector()
{
	m_size = 0;
	m_capacity = 0;
	m_array = nullptr;
}

ndBrainMemVector::ndBrainMemVector(const ndReal* const mem, ndInt32 size)
	:ndBrainVector()
{
	m_size = size;
	m_capacity = size + 1;
	m_array = (ndReal*)mem;
}

ndBrainMemVector::~ndBrainMemVector()
{
	m_size = 0;
	m_capacity = 0;
	m_array = nullptr;
}

void ndBrainMemVector::SetSize(ndInt32 size)
{
	m_size = size;
	m_capacity = size + 1;
}

void ndBrainMemVector::SetPointer(ndReal* const memmory)
{
	m_array = memmory;
}

ndBrainVector::ndBrainVector()
	:ndArray<ndReal>()
{
}

ndBrainVector::ndBrainVector(const ndBrainVector& src)
	:ndArray<ndReal>(src)
{
}

ndBrainVector::~ndBrainVector()
{
}

void ndBrainVector::Swap(ndBrainVector& src)
{
	ndSwap(m_size, src.m_size);
	ndSwap(m_array, src.m_array);
	ndSwap(m_capacity, src.m_capacity);
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
	ndMemSet(&(*this)[0], value, GetCount());
}

void ndBrainVector::Set(const ndBrainVector& data)
{
	ndAssert(GetCount() == data.GetCount());
	ndMemCpy(&(*this)[0], &data[0], GetCount());
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

void ndBrainVector::Scale(ndReal scale)
{
	ndScale(GetCount(), &(*this)[0], scale);
}

void ndBrainVector::Clamp(ndReal min, ndReal max)
{
	for (ndInt32 i = GetCount() - 1; i >= 0; --i)
	{
		ndReal val = (*this)[i];
		(*this)[i] = ndClamp(val, min, max);
	}
}

void ndBrainVector::FlushToZero()
{
	for (ndInt32 i = GetCount() - 1; i >= 0; --i)
	{
		(*this)[i] = ndFlushToZero((*this)[i]);
	}
}

void ndBrainVector::ScaleAdd(const ndBrainVector& a, ndReal b)
{
	ndAssert(GetCount() == a.GetCount());
	ndScaleAdd(GetCount(), &(*this)[0], &a[0], b);
}

void ndBrainVector::Add(const ndBrainVector& a)
{
	ndAssert(GetCount() == a.GetCount());
	ndAdd(GetCount(), &(*this)[0], &a[0]);
}

void ndBrainVector::Sub(const ndBrainVector& a)
{
	ndAssert(GetCount() == a.GetCount());
	ndSub(GetCount(), &(*this)[0], &a[0]);
}

void ndBrainVector::Mul(const ndBrainVector& a)
{
	ndAssert(GetCount() == a.GetCount());
	ndMul(GetCount(), &(*this)[0], &a[0]);
}

void ndBrainVector::MulAdd(const ndBrainVector& a, const ndBrainVector& b)
{
	ndAssert(GetCount() == a.GetCount());
	ndAssert(GetCount() == b.GetCount());
	ndMulAdd(GetCount(), &(*this)[0], &a[0], &b[0]);
}

void ndBrainVector::MulSub(const ndBrainVector& a, const ndBrainVector& b)
{
	ndAssert(GetCount() == a.GetCount());
	ndAssert(GetCount() == b.GetCount());
	ndMulSub(GetCount(), &(*this)[0], &a[0], &b[0]);
}

ndReal ndBrainVector::Dot(const ndBrainVector& a) const
{
	ndAssert(GetCount() == a.GetCount());
	return ndDotProduct(GetCount(), &(*this)[0], &a[0]);
}

void ndBrainVector::Blend(const ndBrainVector& target, ndReal blend)
{
	Scale(ndReal(1.0f) - blend);
	ScaleAdd(target, blend);
}