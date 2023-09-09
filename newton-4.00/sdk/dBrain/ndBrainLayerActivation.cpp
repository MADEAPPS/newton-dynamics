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
#include "ndBrainLayerActivation.h"

ndBrainLayerActivation::ndBrainLayerActivation(ndInt32 neurons)
	:ndBrainLayer()
	,m_neurons(neurons)
{
}

ndBrainLayerActivation::ndBrainLayerActivation(const ndBrainLayerActivation& src)
	:ndBrainLayer(src)
{
	ndAssert(0);
}

ndBrainLayerActivation::~ndBrainLayerActivation()
{
	ndAssert(0);
}

bool ndBrainLayerActivation::HasParameters() const
{
	return false;
}

const char* ndBrainLayerActivation::GetLabelId() const
{
	return "ndBrainLayerActivation";
}

ndBrainLayer* ndBrainLayerActivation::Clone() const
{
	ndAssert(0);
	return nullptr;
}

ndInt32 ndBrainLayerActivation::GetOuputSize() const
{
	return m_neurons;
}

ndInt32 ndBrainLayerActivation::GetInputSize() const
{
	return m_neurons;
}

void ndBrainLayerActivation::InitWeightsXavierMethod()
{
}

void ndBrainLayerActivation::InitWeights(ndReal, ndReal)
{
}

void ndBrainLayerActivation::ClearGradAcc(ndBrainVector&, ndBrainMatrix&) const
{
}

void ndBrainLayerActivation::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == m_neurons);
	ndAssert(output.GetCount() == m_neurons);
	output.Set(input);
}

void ndBrainLayerActivation::ActivationDerivative(const ndBrainVector&, ndBrainVector&) const
{
	ndAssert(0);
}

void ndBrainLayerActivation::CalculateOutputParamGradients(const ndBrainVector&, const ndBrainVector&, ndBrainVector&, ndBrainMatrix&) const
{
	ndAssert(0);
}

const char* ndBrainLayerSigmoidActivation::GetLabelId() const
{
	return "ndBrainLayerSigmoidActivation";
}

void ndBrainLayerSigmoidActivation::ActivationDerivative(const ndBrainVector& input, ndBrainVector& output) const
{
	//ndAssert(input.GetCount() == derivativeOutput.GetCount());
	//for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	//{
	//	ndReal val = input[i];
	//	ndReal out = val * (ndReal(1.0f) - val);
	//	derivativeOutput[i] = ndFlushToZero(out);
	//}

	ndAssert(input.GetCount() == output.GetCount());
	output.Set(ndReal(1.0f));
	output.Sub(input);
	output.Mul(input);
	output.FlushToZero();
}

void ndBrainLayerSigmoidActivation::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == output.GetCount());
	for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	{
		//ndReal val = ndClamp(input[i], ndReal(-50.0f), ndReal(50.0f));
		//ndReal exp1 = ndReal(ndExp(val));
		//ndReal out1 = ndFlushToZero(exp1 / (exp1 + ndReal(1.0f)));

		ndReal out = ndReal(0.0f);
		ndReal value = input[i];
		if (value > ndReal(0.0f))
		{
			ndReal p = ndReal(ndExp(-value));
			out = ndFlushToZero(ndReal(1.0f) / (p + ndReal(1.0f)));
		}
		else
		{
			ndReal p = ndReal(ndExp(value));
			out = ndFlushToZero(p / (p + ndReal(1.0f)));
		}

		ndAssert(ndCheckFloat(out));
		ndAssert(out <= ndReal(1.0f));
		ndAssert(out >= ndReal(0.0f));
		output[i] = out;
	}
}

const char* ndBrainLayerTanhActivation::GetLabelId() const
{
	return "ndBrainLayerTanhActivation";
}

void ndBrainLayerTanhActivation::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == output.GetCount());
	for (ndInt32 i = input.GetCount() - 1; i >= 0; --i)
	{
		//ndReal value1 = ndClamp(input[i], ndReal(-25.0f), ndReal(25.0f));
		//ndReal exp = ndReal(ndExp(ndReal(2.0f) * value1));
		//ndReal out0 = ndFlushToZero((exp - ndReal(1.0f)) / (exp + ndReal(1.0f)));

		ndReal out = ndReal(0.0f);
		ndReal value = input[i];
		if (value > ndReal(0.0f))
		{
			ndReal p = ndReal(ndExp(-ndReal(2.0f) * value));
			out = ndFlushToZero((ndReal(1.0f) - p) / (p + ndReal(1.0f)));
		}
		else
		{
			ndReal p = ndReal(ndExp(ndReal(2.0f) * value));
			out = ndFlushToZero((p - ndReal(1.0f)) / (p + ndReal(1.0f)));
		}

		output[i] = out;
		ndAssert(ndCheckFloat(output[i]));
		ndAssert(output[i] <= ndReal(1.0f));
		ndAssert(output[i] >= ndReal(-1.0f));
	}
}
