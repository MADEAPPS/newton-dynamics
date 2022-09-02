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
#include "ndDeepBrainLayer.h"

ndDeepBrainLayer::ndDeepBrainLayer(ndInt32 inputCount, ndInt32 outputCount, ndDeepBrainActivationType activation)
	:ndDeepBrainMatrix(outputCount, inputCount)
	,m_bias()
	,m_activation(activation)
{
	m_bias.SetCount(outputCount);
	m_bias.Set(0.0f);
}

ndDeepBrainLayer::ndDeepBrainLayer(const ndDeepBrainLayer& src)
	:ndDeepBrainMatrix(src)
	,m_bias(src.m_bias)
	,m_activation(src.m_activation)
{
}

ndDeepBrainLayer::ndDeepBrainLayer(const nd::TiXmlNode* layerNode)
	:ndDeepBrainMatrix(layerNode)
{
	const char* const activationType = xmlGetString(layerNode, "activation");
	if (!strcmp(activationType, "tanh"))
	{
		m_activation = m_tanh;
	}
	else if (!strcmp(activationType, "relu"))
	{
		m_activation = m_relu;
	}
	else if (!strcmp(activationType, "sigmoid"))
	{
		m_activation = m_sigmoid;
	}
	else
	{
		ndAssert(0);
	}

	ndInt32 rows = xmlGetInt(layerNode, "outputs");
	m_bias.SetCount(rows);
	xmlGetFloatArray(layerNode, "biasWeights", m_bias);
}

ndDeepBrainLayer::~ndDeepBrainLayer()
{
}

ndDeepBrainLayer* ndDeepBrainLayer::Clone() const
{
	return new ndDeepBrainLayer(*this);
}

void ndDeepBrainLayer::CopyFrom(const ndDeepBrainLayer& src)
{
	Set(src);
	m_bias.Set(src.m_bias);
}

bool ndDeepBrainLayer::Compare(const ndDeepBrainLayer& src) const
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
		if (ndAbs(error) > 1.0e-6f)
		{
			ndAssert(0);
			return false;
		}
	}

	const ndDeepBrainMatrix& me = (*this);
	for (ndInt32 i = 0; i < me.GetCount(); i++)
	{
		const ndDeepBrainVector& row0 = me[i];
		const ndDeepBrainVector& row1 = src[i];
		if (row0.GetCount() != row1.GetCount())
		{
			ndAssert(0);
			return 0;
		}
		for (ndInt32 j = 0; j < row0.GetCount(); j++)
		{
			ndReal error = row0[i] - row1[i];
			if (ndAbs(error) > 1.0e-6f)
			{
				ndAssert(0);
				return false;
			}
		}
	}

	return true;
}

void ndDeepBrainLayer::Save(nd::TiXmlElement* const layerNode) const
{
	xmlSaveParam(layerNode, "type", "fullyConnected");
	xmlSaveParam(layerNode, "inputs", GetColumns());
	xmlSaveParam(layerNode, "outputs", GetRows());

	switch (m_activation)
	{
		case m_relu:
			xmlSaveParam(layerNode, "activation", "relu");
			break;

		case m_tanh:
			xmlSaveParam(layerNode, "activation", "tanh");
			break;

		case m_softmax:
			xmlSaveParam(layerNode, "activation", "softmax");
			break;

		case m_sigmoid:
		default:
			xmlSaveParam(layerNode, "activation", "sigmoid");
			break;
	}
	
	//nd::TiXmlElement* const bias = new nd::TiXmlElement("biasWeights");
	//layerNode->LinkEndChild(bias);
	xmlSaveParam(layerNode, "biasWeights", m_bias.GetCount(), &m_bias[0]);

	nd::TiXmlElement* const input = new nd::TiXmlElement("inputWeights");
	layerNode->LinkEndChild(input);
	for (ndInt32 i = 0; i < GetCount(); i++)
	{
		char weight[256];
		sprintf(weight, "weights%d", i);
		xmlSaveParam(input, weight, GetInputSize(), &(*this)[i][0]);
	}
}

void ndDeepBrainLayer::InitGaussianWeights(ndReal mean, ndReal variance)
{
	m_bias.Set(0.0f);
	for (ndInt32 i = GetCount() - 1; i >= 0; --i)
	{
		(*this)[i].InitGaussianWeights(mean, variance);
	}
}

void ndDeepBrainLayer::ReluActivation(ndDeepBrainVector& output) const
{
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		output[i] = ndMax(ndReal(0.0f), output[i]);
		ndAssert(ndCheckFloat(output[i]));
	}
}

void ndDeepBrainLayer::SigmoidActivation(ndDeepBrainVector& output) const
{
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		ndReal value = ndClamp (output[i], ndReal(-50.0f), ndReal(50.0f));
		const ndReal exp = ndReal(ndPow(ndEXP, value));
		output[i] = exp / (exp + 1.0f);
		ndAssert (ndCheckFloat(output[i]));
		ndAssert(output[i] <= 1.0f);
		ndAssert(output[i] >= 0.0f);
	}
}

void ndDeepBrainLayer::HyperbolicTanActivation(ndDeepBrainVector& output) const
{
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		ndReal value = ndClamp(output[i], ndReal(-25.0f), ndReal(25.0f));
		const ndReal exp = ndReal(ndPow(ndEXP, 2.0f * value));
		output[i] = (exp - 1.0f) / (exp + 1.0f);
		ndAssert(ndCheckFloat(output[i]));
		ndAssert(output[i] <= 1.0f);
		ndAssert(output[i] >= -1.0f);
	}
}

void ndDeepBrainLayer::SoftmaxActivation(ndDeepBrainVector& output) const
{
	ndReal acc = 0.0f;
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		const ndReal exp = ndReal(ndPow(ndEXP, output[i]));
		output[i] = exp;
		ndAssert(ndCheckFloat(output[i]));
		acc += exp;
	}
	
	ndAssert(acc > 0.0f);
	ndReal invAcc = 1.0f / acc;
	for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	{
		output[i] *= invAcc;
		ndAssert(output[i] <= 1.0f);
		ndAssert(output[i] >= 0.0f);
	}
}

void ndDeepBrainLayer::SigmoidDerivative(const ndDeepBrainVector& input, ndDeepBrainVector& derivativeOutput) const
{
	ndAssert(input.GetCount() == derivativeOutput.GetCount());
	for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	{
		ndReal val = input[i];
		derivativeOutput[i] = val * (ndReal(1.0f) - val);
	}
}

void ndDeepBrainLayer::ReluActivationDerivative(const ndDeepBrainVector& input, ndDeepBrainVector& derivativeOutput) const
{
	ndAssert(input.GetCount() == derivativeOutput.GetCount());
	for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	{
		ndReal val = input[i];
		derivativeOutput[i] = val > 0.0f ? 1.0f : 0.0f;
	}
}

void ndDeepBrainLayer::HyperbolicTanDerivative(const ndDeepBrainVector& input, ndDeepBrainVector& derivativeOutput) const
{
	ndAssert(input.GetCount() == derivativeOutput.GetCount());
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
			ndAssert(0);
	}
}

void ndDeepBrainLayer::ActivationDerivative(const ndDeepBrainVector& input, ndDeepBrainVector& derivativeOutput) const
{
	switch (m_activation)
	{
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
			ndAssert(0);
			//SoftmaxActivation(output);
			break;
		}

		default:
			ndAssert(0);
	}
}

void ndDeepBrainLayer::MakePrediction(const ndDeepBrainVector& input, ndDeepBrainVector& output)
{
	Mul(input, output);
	output.Add(output, m_bias);
	ApplyActivation(output);
}

void ndDeepBrainLayer::MakePrediction(ndThreadPool& threadPool, const ndDeepBrainVector& input, ndDeepBrainVector& output)
{
	auto MakePrediction = ndMakeObject::ndFunction([this, &input, &output](ndInt32 threadIndex, ndInt32 threadCount)
	{
		const ndStartEnd startEnd(output.GetCount(), threadIndex, threadCount);
		const ndInt32 count(startEnd.m_end - startEnd.m_start);
		if (count)
		{
			ndDeepBrainMemVector out(&output[startEnd.m_start], count);
			const ndDeepBrainMemVector bias(&m_bias[startEnd.m_start], count);

			const ndDeepBrainMatrix& matrix = (*this);
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				output[i] = input.Dot(matrix[i]);
			}
			out.Add(out, bias);
			ApplyActivation(out);
		}
	});
	threadPool.ParallelExecute(MakePrediction);
}