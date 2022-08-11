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
#include "ndDeepBrainLayer.h"

ndDeepBrainLayer::ndDeepBrainLayer(ndInt32 outputCount, ndDeepBrainActivationType activation)
	:ndClassAlloc()
	,m_activation(activation)
	,m_bias()
{
	m_bias.SetCount(outputCount);
	m_bias.SetValue(0.0f);
}

ndDeepBrainLayer::~ndDeepBrainLayer()
{
}

ndDeepBrainVector& ndDeepBrainLayer::GetBias()
{
	return m_bias;
}

ndInt32 ndDeepBrainLayer::GetOuputSize() const
{
	return m_bias.GetCount();
}

void ndDeepBrainLayer::ReluActivation(ndDeepBrainVector& output) const
{
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		output[i] = ndMax(ndReal(0.0f), output[i]);
	}
}

void ndDeepBrainLayer::SigmoidActivation(ndDeepBrainVector& output) const
{
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		const ndReal exp = ndReal(ndPow(ndEXP, output[i]));
		output[i] = exp / (exp + 1.0f);
	}
}

void ndDeepBrainLayer::HyperbolicTanActivation(ndDeepBrainVector& output) const
{
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		const ndReal exp = ndReal(ndPow(ndEXP, 2.0f * output[i]));
		output[i] = (exp - 1.0f) / (exp + 1.0f);
	}
}

void ndDeepBrainLayer::SoftmaxActivation(ndDeepBrainVector& output) const
{
	ndReal acc = 0.0f;
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		const ndReal exp = ndReal(ndPow(ndEXP, output[i]));
		output[i] = exp;
		acc += exp;
	}
	
	dAssert(acc > 0.0f);
	ndReal invAcc = 1.0f / acc;
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		output[i] *= invAcc;
	}
}

void ndDeepBrainLayer::SigmoidDerivative(const ndDeepBrainVector& input, ndDeepBrainVector& derivativeOutput) const
{
	dAssert(input.GetCount() == derivativeOutput.GetCount());
	for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	{
		ndReal val = input[i];
		derivativeOutput[i] = val * (ndReal(1.0f) - val);
	}
}

void ndDeepBrainLayer::HyperbolicTanDerivative(const ndDeepBrainVector& input, ndDeepBrainVector& derivativeOutput) const
{
	dAssert(input.GetCount() == derivativeOutput.GetCount());
	for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	{
		ndReal val = input[i];
		derivativeOutput[i] = ndReal(1.0f) - val * val;
	}
}


void ndDeepBrainLayer::ApplyActivation(ndDeepBrainVector& output) const
{
	switch (m_activation)
	{
		case m_relu:
		{
			ReluActivation(output);
			break;
		}

		case m_tanh:
		{
			HyperbolicTanActivation(output);
			break;
		}

		case m_sigmoid:
		{
			SigmoidActivation(output);
			break;
		}

		case m_softmax:
		{
			SoftmaxActivation(output);
			break;
		}

		default:
			dAssert(0);
	}
}

void ndDeepBrainLayer::ActivationDerivative(const ndDeepBrainVector& input, ndDeepBrainVector& derivativeOutput) const
{
	switch (m_activation)
	{
		case m_relu:
		{
			dAssert(0);
			//ReluActivation(output);
			break;
		}

		case m_tanh:
		{
			HyperbolicTanDerivative(input, derivativeOutput);
			break;
		}

		case m_sigmoid:
		{
			SigmoidDerivative(input, derivativeOutput);
			break;
		}

		case m_softmax:
		{
			dAssert(0);
			//SoftmaxActivation(output);
			break;
		}

		default:
			dAssert(0);
	}

}