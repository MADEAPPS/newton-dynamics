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

ndDeepBrainLayer::ndDeepBrainLayer(ndInt32 inputCount, ndInt32 outputCount, ActivationType activation)
	:ndClassAlloc()
	,m_activation(activation)
	,m_neurons()
{
	for (ndInt32 i = 0; i < outputCount; ++i)
	{
		ndDeepBrainNeuron* const neuron = new ndDeepBrainNeuron(inputCount);
		m_neurons.PushBack(neuron);
	}
}

ndDeepBrainLayer::~ndDeepBrainLayer()
{
	for (ndInt32 i = 0; i < m_neurons.GetCount(); ++i)
	{
		delete m_neurons[i];
	}
}

ndInt32 ndDeepBrainLayer::GetInputSize() const
{
	ndDeepBrainNeuron* const neuron = m_neurons[0];
	return neuron->GetWeights().GetCount();
}

ndArray<ndDeepBrainNeuron*>& ndDeepBrainLayer::GetNeurons()
{
	return m_neurons;
}

void ndDeepBrainLayer::InitGaussianWeights(ndReal mean, ndReal variance)
{
	for (ndInt32 i = 0; i < m_neurons.GetCount(); ++i)
	{
		m_neurons[i]->InitGaussianWeights(mean, variance);
	}
}

void ndDeepBrainLayer::ReluActivation(ndDeepBrainVector& output)
{
	for (ndInt32 i = 0; i < m_neurons.GetCount(); ++i)
	{
		output[i] = ndMax(ndReal(0.0f), output[i]);
	}
}

void ndDeepBrainLayer::SigmoidActivation(ndDeepBrainVector& output)
{
	for (ndInt32 i = 0; i < m_neurons.GetCount(); ++i)
	{
		output[i] = 1.0f / (1.0f + ndReal (ndPow (ndEXP, -output[i])));
	}
}

void ndDeepBrainLayer::HyperbolicTanActivation(ndDeepBrainVector& output)
{
	for (ndInt32 i = 0; i < m_neurons.GetCount(); ++i)
	{
		ndReal positivePow = ndReal(ndPow(ndEXP, output[i]));
		ndReal negativePow = ndReal(ndPow(ndEXP, -output[i]));
		output[i] = (positivePow - negativePow) / (positivePow + negativePow);
	}
}

void ndDeepBrainLayer::SoftmaxActivation(ndDeepBrainVector& output)
{
	ndReal acc = 0.0f;
	for (ndInt32 i = 0; i < m_neurons.GetCount(); ++i)
	{
		output[i] = ndReal(ndPow(ndEXP, output[i]));
		acc += output[i];
	}

	dAssert(acc > 0.0f);
	ndReal invAcc = 1.0f / acc;
	for (ndInt32 i = 0; i < m_neurons.GetCount(); ++i)
	{
		output[i] *= invAcc;
	}
}


void ndDeepBrainLayer::FowardPass(const ndDeepBrainVector& input, ndDeepBrainVector& output)
{
	for (ndInt32 i = 0; i < m_neurons.GetCount(); ++i)
	{
		output[i] = m_neurons[i]->FowardPass(input);
	}

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
			SoftmaxActivation(output);
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