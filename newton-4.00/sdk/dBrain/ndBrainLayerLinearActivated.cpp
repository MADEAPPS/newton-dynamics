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
#include "ndBrain.h"
#include "ndBrainSaveLoad.h"
#include "ndBrainLayerLinearActivated.h"

ndBrainLayerLinearActivated::ndBrainLayerLinearActivated(ndInt32 inputCount, ndInt32 outputCount, ndBrainActivationType activation)
	:ndBrainLayer()
	,m_bias()
	,m_weights(outputCount, inputCount)
	,m_activation(activation)
	,m_columns(inputCount)
{
	ndAssert(0);
	m_bias.SetCount(outputCount);
}

ndBrainLayerLinearActivated::ndBrainLayerLinearActivated(const ndBrainLayerLinearActivated& src)
	:ndBrainLayer(src)
	,m_bias(src.m_bias)
	,m_weights(src.m_weights)
	,m_activation(src.m_activation)
	,m_columns(src.m_columns)
{
	ndAssert(0);
}

ndBrainLayerLinearActivated::~ndBrainLayerLinearActivated()
{
}

ndBrainLayer* ndBrainLayerLinearActivated::Clone() const
{
	return new ndBrainLayerLinearActivated(*this);
}

void ndBrainLayerLinearActivated::CopyFrom(const ndBrainLayer& src__)
{
	const ndBrainLayerLinearActivated& src = (const ndBrainLayerLinearActivated&)src__;
	m_bias.Set(src.m_bias);
	m_weights.Set(src.m_weights);
}

void ndBrainLayerLinearActivated::Blend(const ndBrainLayer& src__, ndReal blend)
{
	const ndBrainLayerLinearActivated& src = (const ndBrainLayerLinearActivated&)src__;
	m_bias.Blend(src.m_bias, blend);
	m_weights.Blend(src.m_weights, blend);
}

bool ndBrainLayerLinearActivated::Compare(const ndBrainLayer& src__) const
{
	const ndBrainLayerLinearActivated& src = (const ndBrainLayerLinearActivated&)src__;
	if (m_activation != src.m_activation)
	{
		ndAssert(0);
		return false;
	}
	
	if (m_bias.GetCount() != src.m_bias.GetCount())
	{
		ndAssert(0);
		return false;
	}
	
	for (ndInt32 i = 0; i < m_bias.GetCount(); ++i)
	{
		ndReal error = m_bias[i] - src.m_bias[i];
		if (ndAbs(error) > ndFloat32(1.0e-6f))
		{
			ndAssert(0);
			return false;
		}
	}
	
	const ndBrainMatrix& me = m_weights;
	for (ndInt32 i = 0; i < me.GetCount(); ++i)
	{
		const ndBrainVector& row0 = me[i];
		const ndBrainVector& row1 = src.m_weights[i];
		if (row0.GetCount() != row1.GetCount())
		{
			ndAssert(0);
			return 0;
		}
		for (ndInt32 j = 0; j < row0.GetCount(); j++)
		{
			ndReal error = row0[j] - row1[j];
			if (ndAbs(error) > ndFloat32 (1.0e-6f))
			{
				ndAssert(0);
				return false;
			}
		}
	}

	return true;
}

void ndBrainLayerLinearActivated::InitGaussianBias(ndReal variance)
{
	m_bias.InitGaussianWeights(variance);
}

void ndBrainLayerLinearActivated::InitGaussianWeights(ndReal variance)
{
	for (ndInt32 i = m_weights.GetCount() - 1; i >= 0; --i)
	{
		m_weights[i].InitGaussianWeights(variance);
	}
}

void ndBrainLayerLinearActivated::InitWeights(ndReal weighVariance, ndReal biasVariance)
{
	biasVariance = ndMin(biasVariance, ndReal(0.5f));
	weighVariance = ndMin(weighVariance, ndReal(0.5f));
	InitGaussianBias(biasVariance);
	InitGaussianWeights(weighVariance);
}

void ndBrainLayerLinearActivated::InitWeightsXavierMethod()
{
	ndReal biasVariance = ndReal (0.0f);
	ndReal weighVariance = ndReal(0.0f);
	switch (m_activation)
	{
		case m_relu:
		case m_lineal:
		case m_sigmoid:
		case m_softmax:
		case m_noActivation:
		{
			// this seems to be some huge bull shit.
			//biasVariance = ndReal(ndSqrt(ndFloat32(2.0f) / ndFloat32(GetInputSize())));
			//weighVariance = biasVariance;

			biasVariance = ndReal(0.1f);
			weighVariance = ndReal(0.1f);
			break;
		}

		case m_tanh:
		{
			weighVariance = ndReal(ndSqrt (ndFloat32(6.0f) / ndFloat32(GetInputSize() + GetOuputSize())));
			break;
		}
	}

	biasVariance = ndMin(biasVariance, ndReal(0.5f));
	weighVariance = ndMin(weighVariance, ndReal(0.5f));
	InitGaussianBias(biasVariance);
	InitGaussianWeights(weighVariance);
}

void ndBrainLayerLinearActivated::LinealActivation(ndBrainVector&) const
{
}

void ndBrainLayerLinearActivated::ReluActivation(ndBrainVector& output) const
{
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		output[i] = (output[i] > ndReal (0.0f)) ? output[i] : ndReal(0.0f);
		ndAssert(ndCheckFloat(output[i]));
	}
}

void ndBrainLayerLinearActivated::SigmoidActivation(ndBrainVector& output) const
{
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		ndReal val = ndClamp (output[i], ndReal(-50.0f), ndReal(50.0f));
		ndReal ep = ndReal(ndExp(val));
		ndReal out = ndFlushToZero(ep / (ep + ndReal(1.0f)));
		ndAssert(ndCheckFloat(out));
		ndAssert(out <= ndReal(1.0f));
		ndAssert(out >= ndReal(0.0f));
		output[i] = out;
	}
}

void ndBrainLayerLinearActivated::HyperbolicTanActivation(ndBrainVector& output) const
{
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		ndReal value = ndClamp(output[i], ndReal(-25.0f), ndReal(25.0f));
		ndReal ep = ndReal(ndExp(ndReal(2.0f) * value));
		output[i] = ndFlushToZero(ep - ndReal(1.0f)) / (ep + ndReal(1.0f));
		ndAssert(ndCheckFloat(output[i]));
		ndAssert(output[i] <= ndReal(1.0f));
		ndAssert(output[i] >= ndReal(-1.0f));
	}
}

void ndBrainLayerLinearActivated::SoftmaxActivation(ndBrainVector& output) const
{
	ndAssert(0);
	ndReal acc = 0.0f;
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		ndReal ep = ndReal(ndExp(output[i]));
		output[i] = ep;
		ndAssert(ndCheckFloat(output[i]));
		acc += ep;
	}
	
	ndAssert(acc > 0.0f);
	ndReal invAcc = ndReal(1.0f) / acc;
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		output[i] *= invAcc;
		ndAssert(output[i] <= ndReal(1.0f));
		ndAssert(output[i] >= ndReal(0.0f));
	}
}

void ndBrainLayerLinearActivated::LinealActivationDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const
{
	ndAssert(input.GetCount() == derivativeOutput.GetCount());
	for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	{
		derivativeOutput[i] = ndReal(1.0f);
	}
}

void ndBrainLayerLinearActivated::ReluActivationDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const
{
	ndAssert(input.GetCount() == derivativeOutput.GetCount());
	for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	{
		ndReal val = input[i];
		derivativeOutput[i] = (val > ndReal(0.0f)) ? ndReal(1.0f) : ndReal(0.0f);
	}
}

void ndBrainLayerLinearActivated::SigmoidActivationDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const
{
	ndAssert(input.GetCount() == derivativeOutput.GetCount());
	for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	{
		ndReal val = input[i];
		ndReal out = val * (ndReal(1.0f) - val);
		derivativeOutput[i] = ndFlushToZero (out);
	}
}

void ndBrainLayerLinearActivated::TanhActivationDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const
{
	ndAssert(input.GetCount() == derivativeOutput.GetCount());
	for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	{
		ndReal val = input[i];
		derivativeOutput[i] = ndFlushToZero (ndReal(1.0f) - val * val);
	}
}

void ndBrainLayerLinearActivated::SoftmaxDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const
{
	ndAssert(0);
	ndAssert(input.GetCount() == derivativeOutput.GetCount());
	
	// this is not correct, for now use SigmoidActivationDerivative, 
	// Categorical Cross-Entropy Loss
	SigmoidActivationDerivative(input, derivativeOutput);

	//for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	//{
	//	ndReal val = input[i];
	//	ndReal acc = input[i];
	//	for (ndInt32 j = input.GetCount() - 1; j >= 0; --j)
	//	{
	//		acc -= val * input[j];
	//	}
	//	derivativeOutput[i] = acc;
	//}
}

void ndBrainLayerLinearActivated::ApplyActivation(ndBrainVector& output) const
{
	switch (m_activation)
	{
		case m_lineal:
		{
			LinealActivation(output);
			break;
		}

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
			ndAssert(0);
	}
}

void ndBrainLayerLinearActivated::ActivationDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const
{
	switch (m_activation)
	{
		case m_lineal:
		{
			LinealActivationDerivative(input, derivativeOutput);
			break;
		}

		case m_relu:
		{
			ReluActivationDerivative(input, derivativeOutput);
			break;
		}

		case m_tanh:
		{
			TanhActivationDerivative(input, derivativeOutput);
			break;
		}

		case m_sigmoid:
		{
			SigmoidActivationDerivative(input, derivativeOutput);
			break;
		}

		case m_softmax:
		{
			SoftmaxDerivative(input, derivativeOutput);
			break;
		}

		default:
			ndAssert(0);
	}
}

void ndBrainLayerLinearActivated::MakePrediction(const ndBrainVector& input, ndBrainVector& output)
{
	m_weights.Mul(input, output);
	output.Add(m_bias);
	ApplyActivation(output);
}

void ndBrainLayerLinearActivated::Save(const ndBrainSave* const loadSave) const
{
	char buffer[1024];
	auto Save = [this, &buffer, &loadSave](const char* const fmt, ...)
	{
		va_list v_args;
		buffer[0] = 0;
		va_start(v_args, fmt);
		vsprintf(buffer, fmt, v_args);
		va_end(v_args);
		loadSave->WriteData(buffer);
	};

	Save("\tbiasWeights ", m_bias.GetCount());
	for (ndInt32 i = 0; i < m_bias.GetCount(); ++i)
	{
		Save("%g ", m_bias[i]);
	}
	Save("\n");

	Save("\tinputWeights\n");
	for (ndInt32 i = 0; i < m_weights.GetCount(); ++i)
	{
		Save("\t\tweights_%d ", i);
		const ndBrainVector& row = m_weights[i];
		for (ndInt32 j = 0; j < GetInputSize(); ++j)
		{
			Save("%g ", row[j]);
		}
		Save("\n");
	}
}

void ndBrainLayerLinearActivated::Load(const ndBrainLoad* const loader)
{
	char buffer[1024];
	loader->ReadString(buffer);

	for (ndInt32 i = 0; i < m_bias.GetCount(); ++i)
	{
		ndFloat32 weight = loader->ReadFloat();
		m_bias[i] = ndReal(weight);
	}

	loader->ReadString(buffer);
	for (ndInt32 i = 0; i < GetOuputSize(); ++i)
	{
		loader->ReadString(buffer);
		ndBrainVector& row = m_weights[i];
		for (ndInt32 j = 0; j < GetInputSize(); ++j)
		{
			ndFloat32 weight = loader->ReadFloat();
			row[j] = ndReal(weight);
		}
	}
}
