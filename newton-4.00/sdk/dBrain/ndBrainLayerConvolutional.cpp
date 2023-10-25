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
#include "ndBrainSaveLoad.h"
#include "ndBrainLayerConvolutional.h"

ndBrainLayerConvolutional::ndBrainLayerConvolutional(ndInt32 inputWidth, ndInt32 inputHeight, ndInt32 inputDepth, ndInt32 kernelSize, ndInt32 numberOfKernels)
	:ndBrainLayer()
	,m_bias()
	,m_weights()
	,m_inputWidth(inputWidth)
	,m_inputHeight(inputHeight)
	,m_inputDepth(inputDepth)
	,m_kernelSize(kernelSize)
	,m_numberOfKernels(numberOfKernels)
{
	m_outputWidth = m_inputWidth - m_kernelSize + 1;
	m_outputHeight = m_inputHeight - m_kernelSize + 1;

	m_bias.SetCount(m_numberOfKernels);
	//for (ndInt32 i = 0; i < m_numberOfKernels; ++i)
	//{
	//	m_bias[i] = new ndBrainMatrix(m_outputWidth, m_outputHeight);
	//}

	m_weights.SetCount(m_numberOfKernels * m_inputDepth);
	for (ndInt32 i = 0; i < m_numberOfKernels * m_inputDepth; ++i)
	{
		m_weights[i] = new ndBrainMatrix(m_kernelSize, m_kernelSize);
	}
}

ndBrainLayerConvolutional::ndBrainLayerConvolutional(const ndBrainLayerConvolutional& src)
	:ndBrainLayer(src)
	,m_bias()
	,m_weights()
	,m_inputWidth(src.m_inputWidth)
	,m_inputHeight(src.m_inputHeight)
	,m_inputDepth(src.m_inputDepth)
	,m_kernelSize(src.m_kernelSize)
	,m_numberOfKernels(src.m_numberOfKernels)
	,m_outputWidth(src.m_outputWidth)
	,m_outputHeight(src.m_outputHeight)
{
	m_bias.SetCount(m_numberOfKernels);
	//for (ndInt32 i = 0; i < m_numberOfKernels; ++i)
	//{
	//	//m_bias[i] = new ndBrainMatrix(width, height);
	//	m_bias[i] = new ndBrainMatrix(m_kernelSize, m_kernelSize);
	//	m_bias[i]->Set(*src.m_bias[i]);
	//}

	m_weights.SetCount(m_numberOfKernels * m_inputDepth);
	for (ndInt32 i = 0; i < m_numberOfKernels * m_inputDepth; ++i)
	{
		m_weights[i] = new ndBrainMatrix(m_kernelSize, m_kernelSize);
		m_weights[i]->Set(*src.m_weights[i]);
	}
}

ndBrainLayerConvolutional::~ndBrainLayerConvolutional()
{
	ndAssert(0);
	//for (ndInt32 i = 0; i < m_bias.GetCount(); ++i)
	//{
	//	delete (m_bias[i]);
	//}

	for (ndInt32 i = 0; i < m_weights.GetCount(); ++i)
	{
		delete (m_weights[i]);
	}
}

const char* ndBrainLayerConvolutional::GetLabelId() const
{
	return "ndBrainLayerConvolutional";
}

ndBrainLayer* ndBrainLayerConvolutional::Clone() const
{
	return new ndBrainLayerConvolutional(*this);
}

ndInt32 ndBrainLayerConvolutional::GetOutputSize() const
{
	//ndAssert(m_bias.GetCount() == m_weights.GetRows());
	//return m_bias.GetCount();
	//return m_bias.GetCount() * m_bias[0]->GetRows() * m_bias[0]->GetColumns();
	return m_numberOfKernels * m_outputWidth * m_outputHeight;
}

ndInt32 ndBrainLayerConvolutional::GetInputSize() const
{
	//return m_weights.GetColumns();
	//return m_weights.GetCount() * m_inputWidth * m_inputHeight;
	return m_inputDepth * m_inputWidth * m_inputHeight;
}

ndInt32 ndBrainLayerConvolutional::GetOutputWidth() const
{
	return m_outputWidth;
}

ndInt32 ndBrainLayerConvolutional::GetOutputHeighh() const
{
	return m_outputHeight;
}

ndInt32 ndBrainLayerConvolutional::GetOutputChannels() const
{
	return m_numberOfKernels;
}

bool ndBrainLayerConvolutional::HasParameters() const
{
	return true;
}

void ndBrainLayerConvolutional::InitWeightsXavierMethod()
{
	//ndBrainFloat weighVariance = ndBrainFloat(ndSqrt(ndFloat32(6.0f) / ndFloat32(GetInputSize() + GetOutputSize())));
	//InitWeights(weighVariance, ndBrainFloat(0.0f));
	ndAssert(0);
}

void ndBrainLayerConvolutional::InitGaussianBias(ndBrainFloat variance)
{
	//for (ndInt32 i = m_bias.GetCount() - 1; i >= 0; --i)
	//{
	//	ndBrainMatrix& biasMatrix = *m_bias[i];
	//	for (ndInt32 j = biasMatrix.GetRows() - 1; j >= 0 ; --j)
	//	{
	//		biasMatrix[j].InitGaussianWeights(variance);
	//	}
	//}
	m_bias.InitGaussianWeights(variance);
}

void ndBrainLayerConvolutional::InitGaussianWeights(ndBrainFloat variance)
{
	for (ndInt32 i = m_weights.GetCount() - 1; i >= 0; --i)
	{
		ndBrainMatrix& m_weightsMatrix = *m_weights[i];
		for (ndInt32 j = m_weightsMatrix.GetRows() - 1; j >= 0; --j)
		{
			m_weightsMatrix[j].InitGaussianWeights(variance);
		}
	}
}

void ndBrainLayerConvolutional::InitWeights(ndBrainFloat weighVariance, ndBrainFloat biasVariance)
{
	biasVariance = ndMin(biasVariance, ndBrainFloat(0.5f));
	weighVariance = ndMin(weighVariance, ndBrainFloat(0.5f));
	InitGaussianBias(biasVariance);
	InitGaussianWeights(weighVariance);
}

//void ndBrainLayerConvolutional::CopyFrom(const ndBrainLayer& src)
void ndBrainLayerConvolutional::Set(const ndBrainLayer& src)
{
	const ndBrainLayerConvolutional& convSrc = (ndBrainLayerConvolutional&)src;
	//m_bias.Set(linearSrc.m_bias);
	//m_weights.Set(linearSrc.m_weights);

	//for (ndInt32 i = 0; i < m_numberOfKernels; ++i)
	//{
	//	m_bias[i]->Set(*convSrc.m_bias[i]);
	//}

	m_bias.Set(convSrc.m_bias);
	for (ndInt32 i = 0; i < m_numberOfKernels * m_inputDepth; ++i)
	{
		m_weights[i]->Set(*convSrc.m_weights[i]);
	}
}

void ndBrainLayerConvolutional::Blend(const ndBrainLayer& src, ndBrainFloat blend)
{
	//const ndBrainLayerConvolutional& linearSrc = (ndBrainLayerConvolutional&)src;
	//m_bias.Blend(linearSrc.m_bias, blend);
	//m_weights.Blend(linearSrc.m_weights, blend);
	ndAssert(0);
}

void ndBrainLayerConvolutional::InputDerivative(const ndBrainVector&, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	//m_weights.TransposeMul(outputDerivative, inputDerivative);
	ndAssert(0);
}

void ndBrainLayerConvolutional::Save(const ndBrainSave* const loadSave) const
{
	//char buffer[1024];
	//auto Save = [this, &buffer, &loadSave](const char* const fmt, ...)
	//{
	//	va_list v_args;
	//	buffer[0] = 0;
	//	va_start(v_args, fmt);
	//	vsprintf(buffer, fmt, v_args);
	//	va_end(v_args);
	//	loadSave->WriteData(buffer);
	//};
	//
	//Save("\tinputs %d\n", m_weights.GetColumns());
	//Save("\touputs %d\n", m_weights.GetCount());
	//
	//Save("\tbias ");
	//for (ndInt32 i = 0; i < m_bias.GetCount(); ++i)
	//{
	//	Save("%g ", m_bias[i]);
	//}
	//Save("\n");
	//
	//Save("\tweights\n");
	//for (ndInt32 i = 0; i < m_weights.GetCount(); ++i)
	//{
	//	Save("\t\trow_%d ", i);
	//	const ndBrainVector& row = m_weights[i];
	//	for (ndInt32 j = 0; j < GetInputSize(); ++j)
	//	{
	//		Save("%g ", row[j]);
	//	}
	//	Save("\n");
	//}

	ndAssert(0);
}

ndBrainLayer* ndBrainLayerConvolutional::Load(const ndBrainLoad* const loadSave)
{
	//char buffer[1024];
	//loadSave->ReadString(buffer);
	//
	//loadSave->ReadString(buffer);
	//ndInt32 inputs = loadSave->ReadInt();
	//loadSave->ReadString(buffer);
	//ndInt32 outputs = loadSave->ReadInt();
	//ndBrainLayerConvolutional* const layer = new ndBrainLayerConvolutional(inputs, outputs);
	//
	//loadSave->ReadString(buffer);
	//for (ndInt32 i = 0; i < outputs; ++i)
	//{
	//	ndBrainFloat val = ndBrainFloat(loadSave->ReadFloat());
	//	layer->m_bias[i] = val;
	//}
	//
	//loadSave->ReadString(buffer);
	//for (ndInt32 i = 0; i < outputs; ++i)
	//{
	//	loadSave->ReadString(buffer);
	//	for (ndInt32 j = 0; j < inputs; ++j)
	//	{
	//		ndBrainFloat val = ndBrainFloat(loadSave->ReadFloat());
	//		layer->m_weights[i][j] = val;
	//	}
	//}
	//
	//loadSave->ReadString(buffer);
	//return layer;

	ndAssert(0);
	return nullptr;
}

void ndBrainLayerConvolutional::CalculateParamGradients(
	const ndBrainVector& input, const ndBrainVector& output,
	const ndBrainVector& outputDerivative, ndBrainVector& inputGradient, ndBrainLayer* const gradientOut) const
{
	ndAssert(0);
	//ndAssert(!strcmp(GetLabelId(), gradientOut->GetLabelId()));
	//ndBrainLayerLinear* const gradients = (ndBrainLayerLinear*)gradientOut;
	//ndAssert(gradients->m_bias.GetCount() == outputDerivative.GetCount());
	//gradients->m_bias.Set(outputDerivative);
	//for (ndInt32 i = outputDerivative.GetCount() - 1; i >= 0; --i)
	//{
	//	ndBrainFloat value = outputDerivative[i];
	//	gradients->m_weights[i].ScaleSet(input, value);
	//}
	//InputDerivative(output, outputDerivative, inputGradient);
}

void ndBrainLayerConvolutional::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	//m_weights.Mul(input, output);
	//output.Add(m_bias);
	ndAssert(input.GetCount() == GetInputSize());

	//ndInt32 inputStart = 0;
	//ndInt32 outputStart = 0;
	output.InitGaussianWeights(0.1f);
//float xxxxx = 0;
//	for (ndInt32 i = 0; i < m_numberOfKernels; ++i)
//	{
//		const ndBrainMatrix& bias = *m_bias[i];
//		for (ndInt32 j = 0; j < m_outputHeight; ++j)
//		{
//			ndBrainMemVector out(&output[outputStart], m_outputWidth);
//			out.Set(bias[j]);
//
//			for (ndInt32 k = 0; k < m_outputWidth; ++k)
//			{
//				out[k] = xxxxx;
//				xxxxx += 1.0f;
//			}
//			outputStart += m_outputWidth;
//		}
//
//		//for (ndInt32 j = 0; j < m_inputDepth; ++j)
//		//{
//		//	const ndBrainMatrix& weight = *m_weights[j];
//		//
//		//	for (ndInt32 k = 0; k < m_outputWidth; ++k)
//		//	{
//		//		for (ndInt32 n = 0; n < m_outputHeight; ++n)
//		//		{
//		//
//		//		}
//		//	}
//		//
//		//	//for (ndInt32 k = 0; k < m_kernelSize; ++k)
//		//	//{
//		//	//	const ndBrainVector& kernelRow = weight[j];
//		//	//	const ndBrainMemVector in(&input[inputStart], m_inputWidth);
//		//	//	//inputStart += m_inputStride;
//		//	//}
//		//}
//	}
}