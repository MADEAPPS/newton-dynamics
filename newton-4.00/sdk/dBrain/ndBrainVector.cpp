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
#include "ndBrainSimdFloat8.h"

void ndBrainVector::InitGaussianWeights(ndBrainFloat variance)
{
	if (variance > ndBrainFloat(1.0e-6f))
	{
		for (ndInt64 i = GetCount() - 1; i >= 0; --i)
		{
			(*this)[i] = ndBrainFloat(ndGaussianRandom(ndFloat32(0.0f), ndFloat32(variance)));
			//(*this)[i] = 0.0f;
		}
		//(*this)[0] = 1.0f;
		//(*this)[GetCount() - 1] = 2.0f;
	}
	else
	{
		Set(ndFloat32(0.0f));
	}
}

void ndBrainVector::Set(ndBrainFloat value)
{
	ndMemSet(&(*this)[0], value, GetCount());
}

void ndBrainVector::Set(const ndBrainVector& data)
{
	ndAssert(GetCount() == data.GetCount());
	ndMemCpy(&(*this)[0], &data[0], GetCount());
}

ndInt64 ndBrainVector::ArgMax() const
{
	ndInt64 index = 0;
	ndBrainFloat maxValue = (*this)[0];
	for (ndInt64 i = 1; i < GetCount(); ++i)
	{
		ndBrainFloat val = (*this)[i];
		if (val > maxValue)
		{
			index = i;
			maxValue = val;
		}
	}
	return index;
}

// using Random variate generation from https://en.wikipedia.org/wiki/Gumbel_distribution 
void ndBrainVector::CategoricalSample(const ndBrainVector& probability, ndBrainFloat beta)
{
	#ifdef _DEBUG
		ndBrainFloat acc = ndBrainFloat(0.0f);
		for (ndInt32 i = 0; i < GetCount(); ++i)
		{
			acc += probability[i];
		}
		ndAssert(ndAbs(acc - ndFloat32(1.0f)) < ndBrainFloat(1.0e-5f));
	#endif	

	//ndBrainFloat xxx = 0.0f;
	//for (ndInt32 i = 0; i < 20; ++i)
	//{
	//	ndBrainFloat map = ndBrainFloat(0.930159f) * xxx + ndBrainFloat(1.0e-6f);
	//	ndBrainFloat xxx1 = -ndLog (-ndLog(map));
	//	xxx += 1.0f / 20.0f;
	//}

	ndBrainFloat sum = ndBrainFloat(0.0f);
	for (ndInt32 i = 0; i < GetCount(); ++i)
	{
		ndBrainFloat num = ndClamp(probability[i], ndBrainFloat(0.0000001f), ndBrainFloat(0.9999999f));;
		ndBrainFloat den = ndBrainFloat(1.0f) - num;
		ndBrainFloat logits = ndBrainFloat (ndLog(num / den));
		ndBrainFloat r = ndBrainFloat(0.930159f) * ndBrainFloat (ndRand()) + ndBrainFloat(1.0e-6f);
		ndBrainFloat noise = beta * ndBrainFloat (-ndLog(-ndLog(r)));
		ndBrainFloat sample = ndBrainFloat (ndExp (logits + noise));
		sum += sample;
		(*this)[i] = sample;
	}
	Scale(ndBrainFloat(1.0f) / sum);
}

void ndBrainVector::CalculateMeanAndDeviation(ndBrainFloat& mean, ndBrainFloat& deviation) const
{
	ndFloat64 sum = ndFloat64(0.0f);
	ndFloat64 sum2 = ndFloat64(0.0f);
	for (ndInt64 i = GetCount() - 1; i >= 0; --i)
	{
		ndFloat64 x = (*this)[i];
		sum += x;
		sum2 += x * x;
	}
	ndFloat64 den = ndFloat64(1.0f) / ndBrainFloat(GetCount());
	ndFloat64 average = sum * den;
	ndFloat64 variance2 = ndMax((sum2 * den - average * average), ndFloat64(1.0e-12f));

	mean = ndBrainFloat(average);
	deviation = ndBrainFloat(ndSqrt (variance2));
}

void ndBrainVector::CalculateMeanAndDeviation(const ndBrainVector& deviation, const ndBrainVector& uniformRand)
{
	for (ndInt64 i = GetCount() - 1; i >= 0; --i)
	{
		(*this)[i] = ndGaussianRandom((*this)[i], deviation[i], uniformRand[i]);
	}
}

void ndBrainVector::GaussianNormalize()
{
	ndBrainFloat mean;
	ndBrainFloat variance;
	CalculateMeanAndDeviation(mean, variance);
	ndBrainFloat invVariance = ndBrainFloat(1.0f) / variance;
	for (ndInt64 i = GetCount() - 1; i >= 0; --i)
	{
		ndBrainFloat x = (*this)[i];
		(*this)[i] = (x - mean) * invVariance;
	}
}

void ndBrainVector::SoftMaxNormalize()
{
	ndBrainFloat max = ndBrainFloat(0.0f);
	for (ndInt32 i = ndInt32(GetCount() - 1); i >= 0; --i)
	{
		max = ndMax((*this)[i], max);
	}

	ndBrainFloat acc = ndBrainFloat(0.0f);
	for (ndInt32 i = ndInt32(GetCount() - 1); i >= 0; --i)
	{
		ndBrainFloat in = ndMax(((*this)[i] - max), ndBrainFloat(-30.0f));
		ndAssert(in <= ndBrainFloat(0.0f));
		ndBrainFloat prob = ndBrainFloat(ndExp(in));
		(*this)[i] = prob;
		acc += prob;
	}

	ndAssert(acc > ndBrainFloat(0.0f));
	Scale(ndBrainFloat(1.0f) / acc);
}

void ndBrainVector::Scale(ndBrainFloat scale)
{
	ndAssert(GetCount() < (1ll << 32));
	ndScale(ndInt32(GetCount()), &(*this)[0], scale);
}

void ndBrainVector::Clamp(ndBrainFloat min, ndBrainFloat max)
{
	for (ndInt64 i = GetCount() - 1; i >= 0; --i)
	{
		ndBrainFloat val = (*this)[i];
		(*this)[i] = ndClamp(val, min, max);
	}
}

void ndBrainVector::FlushToZero()
{
	const ndBrainSimdFloat8 max(ndBrainFloat(1.0e-16f));
	const ndBrainSimdFloat8 min(ndBrainFloat(-1.0e-16f));
	const ndInt32 roundCount = ndInt32(GetCount()) & -8;
	ndBrainFloat* const data = &(*this)[0];
	for (ndInt32 i = 0; i < roundCount; i += 8)
	{
		const ndBrainSimdFloat8 x(&data[i]);
		const ndBrainSimdFloat8 mask((x < min) | (x > max));
		(x & mask).Store(&data[i]);
	}
	for (ndInt32 i = ndInt32(GetCount() - 1); i >= roundCount; --i)
	{
		ndBrainFloat val = data[i];
		data[i] = (val > max.m_f[0]) ? val : ((val < min.m_f[0]) ? val : ndBrainFloat(0.0f));
	}
}

void ndBrainVector::Min(const ndBrainVector& a)
{
	ndAssert(GetCount() < (1ll << 32));
	ndAssert(GetCount() == a.GetCount());
	ndMin(ndInt32(GetCount()), &(*this)[0], &a[0]);
}

void ndBrainVector::Max(const ndBrainVector& a)
{
	ndAssert(GetCount() < (1ll << 32));
	ndAssert(GetCount() == a.GetCount());
	ndMax(ndInt32(GetCount()), &(*this)[0], &a[0]);
}

void ndBrainVector::LessEqual(const ndBrainVector& a)
{
	ndAssert(GetCount() < (1ll << 32));
	ndAssert(GetCount() == a.GetCount());
	ndLessEqualMask(ndInt32(GetCount()), &(*this)[0], &a[0]);
}

void ndBrainVector::GreaterEqual(const ndBrainVector& a)
{
	ndAssert(GetCount() < (1ll << 32));
	ndAssert(GetCount() == a.GetCount());
	ndGreaterEqualMask(ndInt32(GetCount()), &(*this)[0], &a[0]);
}

void ndBrainVector::ScaleSet(const ndBrainVector& a, ndBrainFloat b)
{
	ndAssert(GetCount() < (1ll << 32));
	ndAssert(GetCount() == a.GetCount());
	ndScaleSet(ndInt32 (GetCount()), &(*this)[0], &a[0], b);
}

void ndBrainVector::ScaleAdd(const ndBrainVector& a, ndBrainFloat b)
{
	ndAssert(GetCount() < (1ll << 32));
	ndAssert(GetCount() == a.GetCount());
	ndScaleAdd(ndInt32(GetCount()), &(*this)[0], &a[0], b);
}

void ndBrainVector::Add(const ndBrainVector& a)
{
	ndAssert(GetCount() < (1ll << 32));
	ndAssert(GetCount() == a.GetCount());
	ndAdd(ndInt32(GetCount()), &(*this)[0], &a[0]);
}

void ndBrainVector::Sub(const ndBrainVector& a)
{
	ndAssert(GetCount() < (1ll << 32));
	ndAssert(GetCount() == a.GetCount());
	ndSub(ndInt32(GetCount()), &(*this)[0], &a[0]);
}

void ndBrainVector::Mul(const ndBrainVector& a)
{
	ndAssert(GetCount() < (1ll << 32));
	ndAssert(GetCount() == a.GetCount());
	ndMul(ndInt32(GetCount()), &(*this)[0], &a[0]);
}

void ndBrainVector::MulAdd(const ndBrainVector& a, const ndBrainVector& b)
{
	ndAssert(GetCount() < (1ll << 32));
	ndAssert(GetCount() == a.GetCount());
	ndAssert(GetCount() == b.GetCount());
	ndMulAdd(ndInt32(GetCount()), &(*this)[0], &a[0], &b[0]);
}

void ndBrainVector::MulSub(const ndBrainVector& a, const ndBrainVector& b)
{
	ndAssert(GetCount() < (1ll << 32));
	ndAssert(GetCount() == a.GetCount());
	ndAssert(GetCount() == b.GetCount());
	ndMulSub(ndInt32(GetCount()), &(*this)[0], &a[0], &b[0]);
}

ndBrainFloat ndBrainVector::Dot(const ndBrainVector& a) const
{
	ndAssert(GetCount() < (1ll << 32));
	ndAssert(GetCount() == a.GetCount());
	return ndDotProduct(ndInt32(GetCount()), &(*this)[0], &a[0]);
}

void ndBrainVector::Blend(const ndBrainVector& target, ndBrainFloat blend)
{
	ndAssert(GetCount() < (1ll << 32));
	Scale(ndBrainFloat(1.0f) - blend);
	ScaleAdd(target, blend);
}

void ndBrainVector::Blend(const ndBrainVector& target, const ndBrainVector& blend)
{
	ndAssert(GetCount() < (1ll << 32));
	//Scale(ndBrainFloat(1.0f) - blend);
	//ScaleAdd(target, blend);
	ndAssert(0);
	for (ndInt64 i = GetCount() - 1; i >= 0; --i)
	{
		ndBrainFloat s0 = blend[i];
		ndBrainFloat s1 = ndBrainFloat(1.0f) - s0;
		(*this)[i] = (*this)[i] * s1 + target[i] * s0;
	}
}

void ndBrainMemVector::SetSize(ndInt64 size)
{
	m_size = size;
	m_capacity = size + 1;
}

void ndBrainMemVector::SetPointer(ndBrainFloat* const memmory)
{
	m_array = memmory;
}

void ndBrainMemVector::Swap(ndBrainMemVector& src)
{
	ndSwap(m_size, src.m_size);
	ndSwap(m_array, src.m_array);
	ndSwap(m_capacity, src.m_capacity);
}
