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

#include "ndCoreStdafx.h"
#include "ndTypes.h"
#include "ndUtils.h"
#include "ndProbability.h"
#include "ndThreadSyncUtils.h"

static ndRand127& GetRandomGenerator()
{
	static ndRand127 generator;
	return generator;
}

void ndSetRandSeed(ndUnsigned32 seed)
{
	GetRandomGenerator().Init(seed);
}

ndUnsigned32 ndRandInt()
{
	return GetRandomGenerator().Generate();
}

ndFloat32 ndRand()
{
	static ndUniformDistribution uniform;
	ndFloat32 value = uniform();
	return value;
}

//ndFloat32 ndGaussianRandom(ndFloat32 mean, ndFloat32 sigma, ndFloat32 randomVariable)
ndFloat32 ndStandardNormalGaussian(ndFloat32 randomVariable)
{
	//// It seems the standard library normal random is based of the Box–Muller transform
	//// https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
	//// this is probably more accurate than Abramowitz and Stegun which is based on the 
	//// inverse cumulative, however Box–Muller is quite problematic, since it uses memory, 
	//// and more calls to rand and transcendental functions, 
	//// therefore, I am sticking with inverse cumulative method.
	//// beside, all my test show that inverse cumulative seems in fact more accurate.
	//// 
	//
	//// from Abramowitz and Stegun formula 26.2.23.
	//// calculate a normal value with 0.0 mean and 1.0 deviation 
	//// the absolute value of the error should be less than 4.5e-4.
	////auto NormalCumulativeDistibutionInverseOld = [](ndFloat32 r)
	////{
	////	auto RationalApproximation = [](ndFloat32 t)
	////	{
	////		ndFloat32 c[] = { ndFloat32(2.515517f), ndFloat32(0.802853f), ndFloat32(0.010328f) };
	////		ndFloat32 d[] = { ndFloat32(1.432788f), ndFloat32(0.189269f), ndFloat32(0.001308f) };
	////		ndFloat32 numerator = c[0] + (c[2] * t + c[1]) * t;
	////		ndFloat32 denominator = ndFloat32(1.0f) + ((d[2] * t + d[1]) * t + d[0]) * t;
	////		return t - numerator / denominator;
	////	};
	////
	////	ndFloat32 value = ndFloat32(0.0f);
	////	if (r < ndFloat32(0.5f))
	////	{
	////		// F^-1(p) = - G^-1(p)
	////		value = -RationalApproximation(ndSqrt(ndFloat32(-2.0f) * ndLog(r)));
	////	}
	////	else
	////	{
	////		// F^-1(p) = G^-1(1-p)
	////		value = RationalApproximation(ndSqrt(ndFloat32(-2.0f) * ndLog(ndFloat32(1.0f) - r)));
	////	}
	////
	////	// Taylor expansion does not produces good approximation for value far from the center. 
	////	// not sure how the Abramowitz and Stegun formula is derived but is far better than Taylor
	////	// in both accuracy and performance 
	////	// ndFloat32 value1 = (r >= ndFloat32(0.5f)) ? TailorExpansion(r - ndFloat32(0.5f)) : -TailorExpansion(ndFloat32(0.5f) - r);
	////	return value;
	////};
	//
	//// more GPU portable version
	//auto NormalCumulativeDistibutionInverse = [](ndFloat32 r)
	//{
	//	auto RationalApproximation = [](ndFloat32 t)
	//	{
	//		const ndFloat32 c0 = ndFloat32(2.515517f);
	//		const ndFloat32 c1 = ndFloat32(0.802853f);
	//		const ndFloat32 c2 = ndFloat32(0.010328f);
	//		const ndFloat32 d0 = ndFloat32(1.432788f);
	//		const ndFloat32 d1 = ndFloat32(0.189269f);
	//		const ndFloat32 d2 = ndFloat32(0.001308f);
	//		ndFloat32 numerator = c0 + (c2 * t + c1) * t;
	//		ndFloat32 denominator = ndFloat32(1.0f) + ((d2 * t + d1) * t + d0) * t;
	//		ndFloat32 r = t - numerator / denominator;
	//		return r;
	//	};
	//
	//	ndFloat32 value = ndFloat32(0.0f);
	//	if (r >= ndFloat32(0.5f))
	//	{
	//		// F^-1(p) = G^-1(1-p)
	//		ndFloat32 arg = ndFloat32(-2.0f) * ndLog(ndFloat32(1.0f) - r);
	//		ndAssert(arg > 0.0f);
	//		value = RationalApproximation(ndSqrt(arg));
	//	}
	//	else
	//	{
	//		// F^-1(p) = - G^-1(p)
	//		ndFloat32 arg = ndFloat32(-2.0f) * ndLog(r);
	//		ndAssert(arg > 0.0f);
	//		value = -RationalApproximation(ndSqrt(arg));
	//	}
	//	return value;
	//};
	//
	//ndAssert(randomVariable >= ndFloat32(0.0f));
	//ndAssert(randomVariable <= ndFloat32(1.0f));
	//ndFloat32 r = ndClamp(randomVariable, ndFloat32(1.0e-6f), ndFloat32(1.0f - 1.0e-6f));
	//ndFloat32 normal = NormalCumulativeDistibutionInverse(r);
	//return normal;

	static ndNomalDistribution gaussian;
	return gaussian.NormalGaussian(randomVariable);
}

#define TINYMT32_MUL (1.0f / 16777216.0f)
ndReal ndUniformDistribution::operator()()
{
	uint32_t rand = (Generate() >> 8);
	return ndReal(rand) * TINYMT32_MUL;
}

D_CORE_API ndReal ndNomalDistribution::NormalGaussian(ndReal uniform)
{
	// It seems the standard library normal random is based of the Box–Muller transform
	// https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
	// this is probably more accurate than Abramowitz and Stegun which is based on the 
	// inverse cumulative, however Box–Muller is quite problematic, since it uses memory, 
	// and more calls to rand and transcendental functions, 
	// therefore, I am sticking with inverse cumulative method.
	// beside, all my test show that inverse cumulative seems in fact more accurate.
	// 

	// from Abramowitz and Stegun formula 26.2.23.
	// calculate a normal value with 0.0 mean and 1.0 deviation 
	// the absolute value of the error should be less than 4.5e-4.
	//auto NormalCumulativeDistibutionInverseOld = [](ndFloat32 r)
	//{
	//	auto RationalApproximation = [](ndFloat32 t)
	//	{
	//		ndFloat32 c[] = { ndFloat32(2.515517f), ndFloat32(0.802853f), ndFloat32(0.010328f) };
	//		ndFloat32 d[] = { ndFloat32(1.432788f), ndFloat32(0.189269f), ndFloat32(0.001308f) };
	//		ndFloat32 numerator = c[0] + (c[2] * t + c[1]) * t;
	//		ndFloat32 denominator = ndFloat32(1.0f) + ((d[2] * t + d[1]) * t + d[0]) * t;
	//		return t - numerator / denominator;
	//	};
	//
	//	ndFloat32 value = ndFloat32(0.0f);
	//	if (r < ndFloat32(0.5f))
	//	{
	//		// F^-1(p) = - G^-1(p)
	//		value = -RationalApproximation(ndSqrt(ndFloat32(-2.0f) * ndLog(r)));
	//	}
	//	else
	//	{
	//		// F^-1(p) = G^-1(1-p)
	//		value = RationalApproximation(ndSqrt(ndFloat32(-2.0f) * ndLog(ndFloat32(1.0f) - r)));
	//	}
	//
	//	// Taylor expansion does not produces good approximation for value far from the center. 
	//	// not sure how the Abramowitz and Stegun formula is derived but is far better than Taylor
	//	// in both accuracy and performance 
	//	// ndFloat32 value1 = (r >= ndFloat32(0.5f)) ? TailorExpansion(r - ndFloat32(0.5f)) : -TailorExpansion(ndFloat32(0.5f) - r);
	//	return value;
	//};

	// more GPU portable version
	auto NormalCumulativeDistibutionInverse = [](ndFloat32 r)
	{
		auto RationalApproximation = [](ndFloat32 t)
		{
			const ndFloat32 c0 = ndFloat32(2.515517f);
			const ndFloat32 c1 = ndFloat32(0.802853f);
			const ndFloat32 c2 = ndFloat32(0.010328f);
			const ndFloat32 d0 = ndFloat32(1.432788f);
			const ndFloat32 d1 = ndFloat32(0.189269f);
			const ndFloat32 d2 = ndFloat32(0.001308f);
			ndFloat32 numerator = c0 + (c2 * t + c1) * t;
			ndFloat32 denominator = ndFloat32(1.0f) + ((d2 * t + d1) * t + d0) * t;
			ndFloat32 r = t - numerator / denominator;
			return r;
		};

		ndFloat32 value = ndFloat32(0.0f);
		if (r >= ndFloat32(0.5f))
		{
			// F^-1(p) = G^-1(1-p)
			ndFloat32 arg = ndFloat32(-2.0f) * ndLog(ndFloat32(1.0f) - r);
			ndAssert(arg > 0.0f);
			value = RationalApproximation(ndSqrt(arg));
		}
		else
		{
			// F^-1(p) = - G^-1(p)
			ndFloat32 arg = ndFloat32(-2.0f) * ndLog(r);
			ndAssert(arg > 0.0f);
			value = -RationalApproximation(ndSqrt(arg));
		}
		return value;
	};

	ndAssert(uniform >= ndFloat32(0.0f));
	ndAssert(uniform <= ndFloat32(1.0f));
	ndFloat32 r = ndClamp(uniform, ndFloat32(1.0e-6f), ndFloat32(1.0f - 1.0e-6f));
	ndFloat32 normal = NormalCumulativeDistibutionInverse(r);
	return normal;
}

ndReal ndNomalDistribution::operator()()
{
	uint32_t rand = (Generate() >> 8);
	ndReal uniform = ndReal(rand) * TINYMT32_MUL;
	return ndReal(NormalGaussian(uniform));
}