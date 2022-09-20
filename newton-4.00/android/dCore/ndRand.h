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

#ifndef __ND_RAND_H__
#define __ND_RAND_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"

/// Returns the time in micro seconds since application started 
D_CORE_API ndUnsigned64 ndGetTimeInMicroseconds();

/// Returns a float random value between 0.0 top 1.0 
D_CORE_API ndFloat32 ndRand();

/// Returns a unsigned integer random value between 0 top 0xffffffff 
D_CORE_API ndUnsigned32 ndRandInt();

/// Returns sets a new random seed 
D_CORE_API void ndSetRandSeed(ndUnsigned32 seed);

/// Returns a guardian distributed random value with center at mean and standard deviation sigma
D_CORE_API ndFloat32 ndGaussianRandom(ndFloat32 mean, ndFloat32 sigma);

/// generate a Gaussian process of centered at mean, deviation sigma and uncertainty range theta.
/// this class is used to randomly explore the environment when using reinforcement learning training.
class ndOUNoise
{
	public: 
	D_CORE_API ndOUNoise(ndFloat32 value, ndFloat32 theta, ndFloat32 mean, ndFloat32 sigma);
	D_CORE_API void Reset(ndFloat32 value);
	D_CORE_API ndFloat32 Evaluate(ndFloat32 step);

	private:
	ndFloat32 m_value;
	ndFloat32 m_mean;
	ndFloat32 m_sigma;
	ndFloat32 m_theta;
};

#endif

