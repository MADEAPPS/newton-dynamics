/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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
#include "ndDeepBrainNeuron.h"

ndDeepBrainNeuron::ndDeepBrainNeuron(ndInt32 inputs)
	:ndDeepBrainVector()
	,m_bias(0.0f)
{
	SetCount(inputs);
}

ndDeepBrainNeuron::~ndDeepBrainNeuron()
{
}

ndReal ndDeepBrainNeuron::GetBias() const
{
	return m_bias;
}

void ndDeepBrainNeuron::SetBias(ndReal bias)
{
	m_bias = bias;
}

void ndDeepBrainNeuron::InitGaussianWeights(ndReal mean, ndReal variance)
{
	m_bias = ndReal(ndGaussianRandom(mean, variance));
	ndDeepBrainVector::InitGaussianWeights(mean, variance);
}

ndReal ndDeepBrainNeuron::LinearPredict(const ndDeepBrainVector& input)
{
	dAssert(input.GetCount() >= GetCount());
	return m_bias + Dot(input);
}