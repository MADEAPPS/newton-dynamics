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
#include "ndBrainLayer.h"
#include "ndBrainSaveLoad.h"

#define RELUE_SATURATION ndReal(100.0f)

ndBrainLayer::ndBrainLayer(ndInt32 inputCount, ndInt32 outputCount, ndBrainActivationType activation)
	:ndBrainMatrix()
	,m_bias()
	,m_activation(activation)
	,m_columns(inputCount)
{
	m_size = outputCount;
	m_capacity = outputCount + 1;
	m_bias.SetSize(outputCount);
}

ndBrainLayer::ndBrainLayer(const ndBrainLayer& src)
	:ndBrainMatrix()
	,m_bias()
	,m_activation(src.m_activation)
	,m_columns(src.m_columns)
{
	m_size = src.GetOuputSize();
	m_capacity = src.GetOuputSize() + 1;
	m_bias.SetSize(src.GetOuputSize());
}

ndBrainLayer::~ndBrainLayer()
{
	m_size = 0;
	m_capacity = 0;
}

ndUnsigned8* ndBrainLayer::SetPointers(ndUnsigned8* const memPtr)
{
	return ndBrainMatrix::SetPointer(memPtr);
}

ndReal* ndBrainLayer::SetFloatPointers(ndReal* const memPtr)
{
	ndInt32 columns = memPtr ? m_columns : 0;
	ndReal* memory = ndBrainMatrix::SetFloatPointers(memPtr, columns);
	m_bias.SetPointer(memory);
	ndInt32 count = (m_bias.GetCount() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	return &memory[count];
}

ndBrainLayer* ndBrainLayer::Clone() const
{
	return new ndBrainLayer(*this);
}

void ndBrainLayer::CopyFrom(const ndBrainLayer& src)
{
	Set(src);
	m_bias.Set(src.m_bias);
}

void ndBrainLayer::Blend(const ndBrainLayer& src, ndReal blend)
{
	ndBrainMatrix::Blend(src, blend);
	m_bias.Blend(src.m_bias, blend);
}

bool ndBrainLayer::Compare(const ndBrainLayer& src) const
{
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
	
	const ndBrainMatrix& me = (*this);
	for (ndInt32 i = 0; i < me.GetCount(); ++i)
	{
		const ndBrainVector& row0 = me[i];
		const ndBrainVector& row1 = src[i];
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

void ndBrainLayer::InitGaussianBias(ndReal variance)
{
	m_bias.InitGaussianWeights(variance);
}

void ndBrainLayer::InitGaussianWeights(ndReal variance)
{
	for (ndInt32 i = GetCount() - 1; i >= 0; --i)
	{
		(*this)[i].InitGaussianWeights(variance);
	}
}

void ndBrainLayer::InitWeightsXavierMethod(ndReal weighVariance, ndReal biasVariance)
{
	//variance = ndReal(ndSqrt (ndFloat32(2.0f) / ndFloat32(GetInputSize() + GetOuputSize())));
	//InitGaussianBias(variance);

	InitGaussianBias(biasVariance);
	InitGaussianWeights(weighVariance);
}

void ndBrainLayer::LinealActivation(ndBrainVector&) const
{
}

void ndBrainLayer::ReluActivation(ndBrainVector& output) const
{
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		output[i] = (output[i] >= ndReal (0.0f)) ? output[i] : ndReal(0.0f);
		ndAssert(ndCheckFloat(output[i]));
	}
}

void ndBrainLayer::SigmoidActivation(ndBrainVector& output) const
{
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		ndReal value = ndClamp (output[i], ndReal(-50.0f), ndReal(50.0f));
		ndReal exp = ndReal(ndExp(value));
		output[i] = exp / (exp + ndReal(1.0f));
		ndAssert (ndCheckFloat(output[i]));
		ndAssert(output[i] <= ndReal(1.0f));
		ndAssert(output[i] >= ndReal(0.0f));
	}
}

void ndBrainLayer::HyperbolicTanActivation(ndBrainVector& output) const
{
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		ndReal value = ndClamp(output[i], ndReal(-25.0f), ndReal(25.0f));
		ndReal exp = ndReal(ndExp(ndReal(2.0f) * value));
		output[i] = (exp - ndReal(1.0f)) / (exp + ndReal(1.0f));
		ndAssert(ndCheckFloat(output[i]));
		ndAssert(output[i] <= ndReal(1.0f));
		ndAssert(output[i] >= ndReal(-1.0f));
	}
}

void ndBrainLayer::SoftmaxActivation(ndBrainVector& output) const
{
	ndReal acc = 0.0f;
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		ndReal exp = ndReal(ndExp(output[i]));
		output[i] = exp;
		ndAssert(ndCheckFloat(output[i]));
		acc += exp;
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

void ndBrainLayer::LinealActivationDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const
{
	ndAssert(input.GetCount() == derivativeOutput.GetCount());
	for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	{
		derivativeOutput[i] = ndReal(1.0f);
	}
}

void ndBrainLayer::ReluActivationDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const
{
	ndAssert(input.GetCount() == derivativeOutput.GetCount());
	for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	{
		ndReal val = input[i];
		derivativeOutput[i] = (val >= ndReal(0.0f)) ? ndReal(1.0f) : ndReal(0.0f);
	}
}

void ndBrainLayer::SigmoidDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const
{
	ndAssert(input.GetCount() == derivativeOutput.GetCount());
	for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	{
		ndReal val = input[i];
		derivativeOutput[i] = val * (ndReal(1.0f) - val);
	}
}

void ndBrainLayer::HyperbolicTanDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const
{
	ndAssert(input.GetCount() == derivativeOutput.GetCount());
	for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	{
		ndReal val = input[i];
		derivativeOutput[i] = ndReal(1.0f) - val * val;
	}
}

void ndBrainLayer::SoftmaxDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const
{
	ndAssert(input.GetCount() == derivativeOutput.GetCount());
	
	// this is not correct, for now use SigmoidDerivative, 
	// Categorical Cross-Entropy Loss
	SigmoidDerivative(input, derivativeOutput);

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

void ndBrainLayer::ApplyActivation(ndBrainVector& output) const
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

void ndBrainLayer::ActivationDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const
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
			SoftmaxDerivative(input, derivativeOutput);
			break;
		}

		default:
			ndAssert(0);
	}
}

void ndBrainLayer::MakePrediction(const ndBrainVector& input, ndBrainVector& output)
{
	Mul(input, output);
	output.Add(m_bias);
	ApplyActivation(output);
}

void ndBrainLayer::Save(const ndBrainSave* const loadSave) const
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
	for (ndInt32 i = 0; i < GetCount(); ++i)
	{
		Save("\t\tweights_%d ", i);
		const ndBrainVector& row = (*this)[i];
		for (ndInt32 j = 0; j < GetInputSize(); ++j)
		{
			Save("%g ", row[j]);
		}
		Save("\n");
	}
}

void ndBrainLayer::Load(const ndBrainLoad* const loader)
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
		ndBrainVector& row = (*this)[i];
		for (ndInt32 j = 0; j < GetInputSize(); ++j)
		{
			ndFloat32 weight = loader->ReadFloat();
			row[j] = ndReal(weight);
		}
	}
}
