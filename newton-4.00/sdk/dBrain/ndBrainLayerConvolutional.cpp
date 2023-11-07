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
	,m_kernels()
	,m_paddedGradientOut()
	,m_inputOffsets()
	,m_inputGradOffsets()
	,m_inputWidth(inputWidth)
	,m_inputHeight(inputHeight)
	,m_inputDepth(inputDepth)
	,m_kernelSize(kernelSize)
	,m_numberOfKernels(numberOfKernels)
{
	m_outputWidth = m_inputWidth - m_kernelSize + 1;
	m_outputHeight = m_inputHeight - m_kernelSize + 1;

	m_bias.SetCount(m_numberOfKernels);
	m_kernels.SetCount(m_numberOfKernels * m_inputDepth * m_kernelSize * m_kernelSize);

	m_bias.Set(ndBrainFloat(0.0f));
	m_kernels.Set(ndBrainFloat(0.0f));

	ndInt32 inputOffset = 0;
	ndInt32 inputGradOffset = 0;
	for (ndInt32 j = 0; j < m_kernelSize; ++j)
	{
		for (ndInt32 i = 0; i < m_kernelSize; ++i)
		{
			m_inputOffsets.PushBack(inputOffset + i);
			m_inputGradOffsets.PushBack(inputGradOffset + i);
		}
		inputOffset += m_inputWidth;
		inputGradOffset += m_inputWidth + m_kernelSize - 1;
	}
	
	if (inputWidth == 5)
	//if (m_inputWidth == 12)
	{
		//Debug(m_inputWidth, m_inputHeight, m_inputDepth, m_kernelSize, m_numberOfKernels);
		//Debug(7, 7, 2, 3, 3);
		//Debug(4, 4, 2, 2, 3);
		//Debug(3, 3, 1, 2, 1);
	}
}

ndBrainLayerConvolutional::ndBrainLayerConvolutional(const ndBrainLayerConvolutional& src)
	:ndBrainLayer(src)
	,m_bias(src.m_bias)
	,m_kernels(src.m_kernels)
	,m_paddedGradientOut()
	,m_inputOffsets(src.m_inputOffsets)
	,m_inputGradOffsets(src.m_inputGradOffsets)
	,m_inputWidth(src.m_inputWidth)
	,m_inputHeight(src.m_inputHeight)
	,m_inputDepth(src.m_inputDepth)
	,m_kernelSize(src.m_kernelSize)
	,m_numberOfKernels(src.m_numberOfKernels)
	,m_outputWidth(src.m_outputWidth)
	,m_outputHeight(src.m_outputHeight)
{
}

ndBrainLayerConvolutional::~ndBrainLayerConvolutional()
{
	ndAssert(0);
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

ndInt32 ndBrainLayerConvolutional::GetOutputHeight() const
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
	ndBrainFloat weighVariance = ndBrainFloat(ndSqrt(ndFloat32(6.0f) / ndFloat32(GetInputSize() + GetOutputSize())));
	InitWeights(weighVariance, ndBrainFloat(0.0f));
}

void ndBrainLayerConvolutional::InitGaussianBias(ndBrainFloat variance)
{
	m_bias.InitGaussianWeights(variance);
}

void ndBrainLayerConvolutional::InitGaussianWeights(ndBrainFloat variance)
{
	m_kernels.InitGaussianWeights(variance);
}

void ndBrainLayerConvolutional::InitWeights(ndBrainFloat weighVariance, ndBrainFloat biasVariance)
{
	biasVariance = ndMin(biasVariance, ndBrainFloat(0.5f));
	weighVariance = ndMin(weighVariance, ndBrainFloat(0.5f));
	InitGaussianBias(biasVariance);
	InitGaussianWeights(weighVariance);
}

void ndBrainLayerConvolutional::Set(const ndBrainLayer& src)
{
	const ndBrainLayerConvolutional& convSrc = (ndBrainLayerConvolutional&)src;
	m_bias.Set(convSrc.m_bias);
	m_kernels.Set(convSrc.m_kernels);
}

void ndBrainLayerConvolutional::Clear()
{
	m_bias.Set(ndBrainFloat(0.0f));
	m_kernels.Set(ndBrainFloat(0.0f));
}

void ndBrainLayerConvolutional::FlushToZero()
{
	m_bias.FlushToZero();
	m_kernels.FlushToZero();
}

void ndBrainLayerConvolutional::Scale(ndBrainFloat scale)
{
	m_bias.Scale(scale);
	m_kernels.Scale(scale);
}

void ndBrainLayerConvolutional::Add(const ndBrainLayer& src)
{
	const ndBrainLayerConvolutional& linearSrc = (ndBrainLayerConvolutional&)src;
	m_bias.Add(linearSrc.m_bias);
	m_kernels.Add(linearSrc.m_kernels);
}

void ndBrainLayerConvolutional::Mul(const ndBrainLayer& src)
{
	const ndBrainLayerConvolutional& linearSrc = (ndBrainLayerConvolutional&)src;
	m_bias.Mul(linearSrc.m_bias);
	m_kernels.Mul(linearSrc.m_kernels);
}

void ndBrainLayerConvolutional::ScaleAdd(const ndBrainLayer& src, ndBrainFloat scale)
{
	const ndBrainLayerConvolutional& linearSrc = (ndBrainLayerConvolutional&)src;
	m_bias.ScaleAdd(linearSrc.m_bias, scale);
	m_kernels.ScaleAdd(linearSrc.m_kernels, scale);
}

void ndBrainLayerConvolutional::Blend(const ndBrainLayer& src, ndBrainFloat blend)
{
	//const ndBrainLayerConvolutional& linearSrc = (ndBrainLayerConvolutional&)src;
	//m_bias.Blend(linearSrc.m_bias, blend);
	//m_weights.Blend(linearSrc.m_weights, blend);
	ndAssert(0);
}

void ndBrainLayerConvolutional::AdamUpdate(const ndBrainLayer& u, const ndBrainLayer& v, ndBrainFloat epsilon)
{
	const ndBrainLayerConvolutional& linear_U = (ndBrainLayerConvolutional&)u;
	const ndBrainLayerConvolutional& linear_V = (ndBrainLayerConvolutional&)v;

	const ndBrainVector& bias_U = linear_U.m_bias;
	const ndBrainVector& bias_V = linear_V.m_bias;
	for (ndInt32 i = m_bias.GetCount() - 1; i >= 0; --i)
	{
		ndBrainFloat bias_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(bias_V[i])) + epsilon);
		m_bias[i] = bias_U[i] * bias_den;
	}

	const ndBrainVector& kernels_U = linear_U.m_kernels;
	const ndBrainVector& kernels_V = linear_V.m_kernels;
	for (ndInt32 j = m_kernels.GetCount() - 1; j >= 0; --j)
	{
		ndBrainFloat weight_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(kernels_V[j])) + epsilon);
		m_kernels[j] = kernels_U[j] * weight_den;
	}
}

//void ndBrainLayerConvolutional::InputDerivative(const ndBrainVector&, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
//{
//	//m_weights.TransposeMul(outputDerivative, inputDerivative);
//	ndAssert(0);
//}

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

void ndBrainLayerConvolutional::Debug(ndInt32 width, ndInt32 height, ndInt32 channels, ndInt32 filterSize, ndInt32 filterCount)
{
	// print inputs
	for (ndInt32 channel = 0; channel < channels; ++channel)
	{
		for (ndInt32 y = 0; y < height; ++y)
		{
			for (ndInt32 x = 0; x < width; ++x)
			{
				ndTrace(("x(%d,%d,%d) ", channel, y, x));
			}
			ndTrace(("\n"));
		}
		ndTrace(("\n"));
	}

	// print kilters
	ndTrace(("\n"));
	for (ndInt32 filter = 0; filter < filterCount; ++filter)
	{
		for (ndInt32 channel = 0; channel < channels; ++channel)
		{
			for (ndInt32 y = 0; y < filterSize; ++y)
			{
				for (ndInt32 x = 0; x < filterSize; ++x)
				{
					ndTrace(("w(%d,%d,%d,%d) ", filter, channel, y, x));
				}
				ndTrace(("\n"));
			}
		}
		ndTrace(("b(%d)\n", filter));
		ndTrace(("\n"));
	}

	ndInt32 outputWidth = width - filterSize + 1;
	ndInt32 outputHeight = height - filterSize + 1;

	ndTrace(("\n"));
	for (ndInt32 filter = 0; filter < filterCount; ++filter)
	{
		for (ndInt32 y0 = 0; y0 < outputHeight; ++y0)
		{
			for (ndInt32 x0 = 0; x0 < outputWidth; ++x0)
			{
				ndTrace(("y(%d,%d,%d) ", filter, y0, x0));
			}
			ndTrace(("\n"));
		}
		ndTrace(("\n"));
	}


	// print convolutions outputs
	ndTrace(("\n"));
	for (ndInt32 filter = 0; filter < filterCount; ++filter)
	{
		for (ndInt32 y0 = 0; y0 < outputHeight; ++y0)
		{
			for (ndInt32 x0 = 0; x0 < outputWidth; ++x0)
			{
				ndTrace(("y(%d,%d,%d)=\n", filter, y0, x0));
				for (ndInt32 channel = 0; channel < channels; ++channel)
				{
					for (ndInt32 y = 0; y < filterSize; ++y)
					{
						ndTrace(("   "));
						for (ndInt32 x = 0; x < filterSize; ++x)
						{
							ndTrace(("x(%d,%d,%d) * w(%d,%d,%d,%d) + ", channel, y0 + y, x0 + x, filter, channel, y, x));
						}
						ndTrace(("\n"));
					}
				}
				ndTrace(("   b(%d)\n", filter));
				ndTrace(("\n"));
			}
		}
		//ndTrace(("\n"));
	}

	ndTrace(("\n"));
	ndInt32 index = 0;
	for (ndInt32 filter = 0; filter < filterCount; ++filter)
	{
		for (ndInt32 y = 0; y < outputHeight; ++y)
		{
			for (ndInt32 x = 0; x < outputWidth; ++x)
			{
				ndTrace(("z(%d) = g(y(%d,%d,%d))\n", index, filter, y, x));
				index++;
			}
		}
	}

	ndTrace(("\n"));
	ndTrace(("L2 =\n"));
	index = 0;
	for (ndInt32 filter = 0; filter < filterCount; ++filter)
	{
		for (ndInt32 y = 0; y < outputHeight; ++y)
		{
			ndTrace(("    "));
			for (ndInt32 x = 0; x < outputWidth; ++x)
			{
				ndTrace(("1/2 * (z(%d) - k(%d))^2 + ", index, index));
				index++;
			}
			ndTrace(("\n"));
		}
	}
	ndTrace(("\n"));

#if 0
	ndTrace(("\n"));
	ndTrace(("d(L2)/d(w) =\n"));
	index = 0;
	for (ndInt32 filter = 0; filter < filterCount; ++filter)
	{
		for (ndInt32 y = 0; y < outputHeight; ++y)
		{
			ndTrace(("    "));
			for (ndInt32 x = 0; x < outputWidth; ++x)
			{
				ndTrace(("(z(%d) - k(%d)) * d(z(%d))/d(w) +\n", index, index, index));
				ndTrace(("    "));
				index++;
			}
			ndTrace(("\n"));
		}
	}
	ndTrace(("\n"));

	ndTrace(("\n"));
	ndTrace(("d(L2)/d(w) =\n"));
	index = 0;
	for (ndInt32 filter = 0; filter < filterCount; ++filter)
	{
		for (ndInt32 y = 0; y < outputHeight; ++y)
		{
			ndTrace(("    "));
			for (ndInt32 x = 0; x < outputWidth; ++x)
			{
				ndTrace(("l(%d) * d(z(%d))/d(w) + ", index, index));
				index++;
			}
			ndTrace(("\n"));
		}
	}
	ndTrace(("\n"));

	ndTrace(("\n"));
	ndTrace(("d(L2)/d(w) =\n"));
	index = 0;
	for (ndInt32 filter = 0; filter < filterCount; ++filter)
	{
		for (ndInt32 y = 0; y < outputHeight; ++y)
		{
			ndTrace(("    "));
			for (ndInt32 x = 0; x < outputWidth; ++x)
			{
				//ndTrace(("z(%d) = g(y(%d,%d,%d))\n", index, filter, y, x));
				//ndTrace(("l(%d) * d(z(%d))/d(w) + ", index, index));
				ndTrace(("l(%d) * d(g(y(%d,%d,%d)))/d(w) +\n", index, filter, y, x));
				ndTrace(("    "));
				index++;
			}
			ndTrace(("\n"));
		}
	}
	ndTrace(("\n"));


	ndTrace(("\n"));
	ndTrace(("d(L2)/d(w) =\n"));
	index = 0;
	for (ndInt32 filter = 0; filter < filterCount; ++filter)
	{
		for (ndInt32 y = 0; y < outputHeight; ++y)
		{
			ndTrace(("    "));
			for (ndInt32 x = 0; x < outputWidth; ++x)
			{
				//ndTrace(("l(%d) * d(z(%d))/d(w) + ", index, index));
				//ndTrace(("z(%d) = g(y(%d,%d,%d))\n", index, filter, y, x));
				//ndTrace(("l(%d) * d(g(y(%d,%d,%d)))/d(w) + ", index, filter, y, x));
				ndTrace(("l(%d) * d(g(y(%d,%d,%d)))/d(y(%d,%d,%d)) * d(y(%d,%d,%d))/d(w) +\n", index, filter, y, x, filter, y, x, filter, y, x));
				ndTrace(("    "));
				index++;
			}
			ndTrace(("\n"));
		}
	}
	ndTrace(("\n"));


	ndTrace(("\n"));
	for (ndInt32 filter = 0; filter < filterCount; ++filter)
	{
		for (ndInt32 channel = 0; channel < channels; ++channel)
		{
			for (ndInt32 y = 0; y < filterSize; ++y)
			{
				for (ndInt32 x = 0; x < filterSize; ++x)
				{
					ndTrace(("d(L2)/d(w(%d,%d,%d,%d)) =\n", filter, channel, y, x));
					ndTrace(("    "));
					for (ndInt32 y0 = 0; y0 < outputHeight; ++y0)
					{
						for (ndInt32 x0 = 0; x0 < outputWidth; ++x0)
						{
							ndTrace(("Dg(%d,%d,%d) * x(%d,%d,%d) + ", filter, y0, x0, channel, y + y0, x + x0));
						}
						ndTrace(("\n"));
						ndTrace(("    "));
					}
					ndTrace(("\n"));
				}
			}
		}
	}
	ndTrace(("\n"));
#endif


#if 1
	ndTrace(("\n"));
	ndTrace(("d(L2)/d(x) =\n"));
	index = 0;
	for (ndInt32 filter = 0; filter < filterCount; ++filter)
	{
		for (ndInt32 y = 0; y < outputHeight; ++y)
		{
			ndTrace(("    "));
			for (ndInt32 x = 0; x < outputWidth; ++x)
			{
				ndTrace(("(z(%d) - k(%d)) * d(z(%d))/d(x) +\n", index, index, index));
				ndTrace(("    "));
				index++;
			}
			ndTrace(("\n"));
		}
	}
	ndTrace(("\n"));


	ndTrace(("\n"));
	ndTrace(("d(L2)/d(x) =\n"));
	index = 0;
	for (ndInt32 filter = 0; filter < filterCount; ++filter)
	{
		for (ndInt32 y = 0; y < outputHeight; ++y)
		{
			ndTrace(("    "));
			for (ndInt32 x = 0; x < outputWidth; ++x)
			{
				ndTrace(("l(%d) * d(z(%d))/d(x) + ", index, index));
				index++;
			}
			ndTrace(("\n"));
		}
	}
	ndTrace(("\n"));


	ndTrace(("\n"));
	ndTrace(("d(L2)/d(x) =\n"));
	index = 0;
	for (ndInt32 filter = 0; filter < filterCount; ++filter)
	{
		for (ndInt32 y = 0; y < outputHeight; ++y)
		{
			ndTrace(("    "));
			for (ndInt32 x = 0; x < outputWidth; ++x)
			{
				//ndTrace(("z(%d) = g(y(%d,%d,%d))\n", index, filter, y, x));
				//ndTrace(("l(%d) * d(z(%d))/d(x) + ", index, index));
				ndTrace(("l(%d) * d(g(y(%d,%d,%d)))/d(x) +\n", index, filter, y, x));
				ndTrace(("    "));
				index++;
			}
			ndTrace(("\n"));
		}
	}
	ndTrace(("\n"));


	ndTrace(("\n"));
	ndTrace(("d(L2)/d(x) =\n"));
	index = 0;
	for (ndInt32 filter = 0; filter < filterCount; ++filter)
	{
		for (ndInt32 y = 0; y < outputHeight; ++y)
		{
			ndTrace(("    "));
			for (ndInt32 x = 0; x < outputWidth; ++x)
			{
				//ndTrace(("l(%d) * d(z(%d))/d(x) + ", index, index));
				//ndTrace(("z(%d) = g(y(%d,%d,%d))\n", index, filter, y, x));
				//ndTrace(("l(%d) * d(g(y(%d,%d,%d)))/d(w) + ", index, filter, y, x));
				ndTrace(("l(%d) * d(g(y(%d,%d,%d)))/d(y(%d,%d,%d)) * d(y(%d,%d,%d))/d(x) +\n", index, filter, y, x, filter, y, x, filter, y, x));
				ndTrace(("    "));
				index++;
			}
			ndTrace(("\n"));
		}
	}
	ndTrace(("\n"));
	
	
	ndTrace(("\n"));
	//for (ndInt32 filter = 0; filter < filterCount; ++filter)
	//{
	//	for (ndInt32 channel = 0; channel < channels; ++channel)
	//	{
	//		for (ndInt32 y = 0; y < filterSize; ++y)
	//		{
	//			for (ndInt32 x = 0; x < filterSize; ++x)
	//			{
	//				ndTrace(("d(L2)/d(w(%d,%d,%d,%d)) =\n", filter, channel, y, x));
	//				ndTrace(("    "));
	//				for (ndInt32 y0 = 0; y0 < outputHeight; ++y0)
	//				{
	//					for (ndInt32 x0 = 0; x0 < outputWidth; ++x0)
	//					{
	//						ndTrace(("Dg(%d,%d,%d) * x(%d,%d,%d) + ", filter, y0, x0, channel, y + y0, x + x0));
	//					}
	//					ndTrace(("\n"));
	//					ndTrace(("    "));
	//				}
	//				ndTrace(("\n"));
	//			}
	//		}
	//	}
	//}
	//ndTrace(("\n"));


	ndTrace(("\n"));
	for (ndInt32 channel = 0; channel < channels; ++channel)
	{
		for (ndInt32 y = 0; y < m_inputHeight; ++y)
		{
			for (ndInt32 x = 0; x < m_inputWidth; ++x)
			{
				ndTrace(("d(L2)/d(x(%d,%d,%d)) =\n", channel, y, x));

				ndTrace(("\n"));
			}
		}
		ndTrace(("\n"));
	}
	ndTrace(("\n"));
#endif
}

//ndBrainFloat ndBrainLayerConvolutional::CrossCorrelation(const ndBrainVector& input, const ndBrainVector& kernels) const
//{
//	//ndInt32 cacheStart = 0;
//	//ndInt32 inputStart = 0;
//	//for (ndInt32 i = 0; i < m_kernelSize; ++i)
//	//{
//	//	ndMemCpy(&inputCache[cacheStart], &input[inputStart], m_kernelSize);
//	//	inputStart += m_inputWidth;
//	//	cacheStart += m_kernelSize;
//	//}
//	//return kernels.Dot(inputCache);
//
//	ndBrainFloat value = ndBrainFloat(0.0f);
//	for (ndInt32 i = 0; i < m_inputOffsets.GetCount(); ++i)
//	{
//		ndInt32 index = m_inputOffsets[i];
//		value += input[index] * kernels[i];
//	}
//	return value;
//}

//void ndBrainLayerConvolutional::PredictionOutputChannel(const ndBrainVector& input, const ndBrainVector& kernels, ndBrainFloat bias, ndBrainVector& output) const
//{
//	ndInt32 inputOffset = 0;
//	ndInt32 outputOffset = 0;
//	const ndInt32 inputSize = m_inputWidth * m_kernelSize;
//	const ndInt32 kernelSize = m_kernelSize * m_kernelSize;
//	for (ndInt32 j = 0; j < m_outputHeight; ++j)
//	{
//		for (ndInt32 i = 0; i < m_outputWidth; ++i)
//		{
//			ndBrainFloat value = bias;
//			const ndBrainMemVector in(&input[inputOffset + i], inputSize);
//
//			ndInt32 kernelOffset = 0;
//			for (ndInt32 k = 0; k < m_inputDepth; ++k)
//			{
//				const ndBrainMemVector filter(&kernels[kernelOffset], kernelSize);
//				value += CrossCorrelation(in, filter);
//			}
//			kernelOffset += kernelSize;
//			output[outputOffset + i] = value;
//		}
//		inputOffset += m_inputWidth;
//		outputOffset += m_outputWidth;
//	}
//}

void ndBrainLayerConvolutional::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	//m_weights.Mul(input, output);
	//output.Add(m_bias);
	ndAssert(input.GetCount() == GetInputSize());

	auto PredictOutputChannel = [this](const ndBrainVector& input, const ndBrainVector& kernels, ndBrainFloat bias, ndBrainVector& output)
	{
		auto CrossCorrelation = [this](const ndBrainVector& input, const ndBrainVector& kernel)
		{
			ndBrainFloat value = ndBrainFloat(0.0f);
			for (ndInt32 i = m_inputOffsets.GetCount() - 1; i >= 0; --i)
			{
				ndInt32 index = m_inputOffsets[i];
				value += input[index] * kernel[i];
			}
			return value;
		};

		ndInt32 inputOffset = 0;
		ndInt32 outputOffset = 0;
		const ndInt32 inputSize = m_inputWidth * m_kernelSize;
		const ndInt32 kernelSize = m_kernelSize * m_kernelSize;
		for (ndInt32 j = 0; j < m_outputHeight; ++j)
		{
			for (ndInt32 i = 0; i < m_outputWidth; ++i)
			{
				ndBrainFloat value = bias;
				const ndBrainMemVector in(&input[inputOffset + i], inputSize);

				ndInt32 kernelOffset = 0;
				for (ndInt32 k = 0; k < m_inputDepth; ++k)
				{
					const ndBrainMemVector filter(&kernels[kernelOffset], kernelSize);
					value += CrossCorrelation(in, filter);
				}
				kernelOffset += kernelSize;
				//ndAssert(value == output[outputOffset + i]);
				output[outputOffset + i] = value;
			}
			inputOffset += m_inputWidth;
			outputOffset += m_outputWidth;
		}
	};

	for (ndInt32 i = 0; i < m_numberOfKernels; ++i)
	{
		ndInt32 outStart = i * m_outputWidth * m_outputHeight;
		ndInt32 kernelStart = i * m_inputDepth * m_kernelSize * m_kernelSize;
		
		ndBrainMemVector out(&output[outStart], m_outputWidth * m_outputHeight);
		const ndBrainMemVector kernels(&m_kernels[kernelStart], m_inputDepth * m_kernelSize * m_kernelSize);
		
		//PredictionOutputChannel(input, kernels, m_bias[i], out);
		PredictOutputChannel(input, kernels, m_bias[i], out);
	}
}

void ndBrainLayerConvolutional::CalculateParamGradients(
	const ndBrainVector& input, const ndBrainVector& output,
	const ndBrainVector& outputDerivative, ndBrainVector& inputGradient, ndBrainLayer* const gradientOut) const
{
	ndAssert(!strcmp(GetLabelId(), gradientOut->GetLabelId()));
	ndBrainLayerConvolutional* const gradients = (ndBrainLayerConvolutional*)gradientOut;

	ndAssert(gradients->m_bias.GetCount() == m_numberOfKernels);
	ndAssert(output.GetCount() == outputDerivative.GetCount());

	//gradients->m_bias.Set(outputDerivative);
	//const ndInt32 size = m_outputWidth * m_outputHeight;
	const ndInt32 inputSize = m_inputWidth * m_inputHeight;
	const ndInt32 kernelSize = m_kernelSize * m_kernelSize;
	const ndInt32 outputSize = m_outputWidth * m_outputHeight;

	for (ndInt32 i = 0; i < m_bias.GetCount(); ++i)
	{
		ndBrainFloat value = ndBrainFloat(0.0f);
		const ndBrainMemVector biasGrad(&outputDerivative[i * outputSize], outputSize);
		for (ndInt32 j = 0; j < outputSize; ++j)
		{
			value += biasGrad[j];
		}
		gradients->m_bias[i] = value;
	}
	
	//for (ndInt32 i = outputDerivative.GetCount() - 1; i >= 0; --i)
	//{
	//	ndBrainFloat value = outputDerivative[i];
	//	gradients->m_weights[i].ScaleSet(input, value);
	//}
	auto CrossCorrelationGradient = [this](const ndBrainVector& input, ndInt32 y0, ndInt32 x0, const ndBrainVector& outputDerivative)
	{
		ndBrainFloat value = ndBrainFloat(0.0f);

		ndInt32 inputDerivativeBase = 0;
		ndInt32 inputBase = y0 * m_inputWidth + x0;
		for (ndInt32 y = 0; y < m_outputHeight; ++y)
		{
			const ndBrainMemVector in(&input[inputBase], m_outputWidth);
			const ndBrainMemVector outputGrad(&outputDerivative[inputDerivativeBase], m_outputWidth);
			for (ndInt32 x = 0; x < m_outputWidth; ++x)
			{
				//value += in.Dot(outputGrad);
				value += in[x] * outputGrad[x];
			}
			inputBase += m_inputWidth;
			inputDerivativeBase += m_outputWidth;
		}
		return value;
	};
	
	ndInt32 kernelOffset = 0;
	for (ndInt32 filter = 0; filter < m_numberOfKernels; ++filter)
	{
		ndInt32 inputOffset = 0;
		const ndBrainMemVector outDerivative(&outputDerivative[filter * outputSize], outputSize);
		for (ndInt32 channel = 0; channel < m_inputDepth; ++channel)
		{
			ndInt32 index = 0;
			const ndBrainMemVector inputChannel(&input[inputOffset], inputSize);
			ndBrainMemVector kernelGradients(&m_kernels[kernelOffset], kernelSize);
			
			for (ndInt32 y = 0; y < m_kernelSize; ++y)
			{
				for (ndInt32 x = 0; x < m_kernelSize; ++x)
				{
					kernelGradients[index] = CrossCorrelationGradient(inputChannel, y, x, outDerivative);
				}
			}
			inputOffset += inputSize;
			kernelOffset += kernelSize;
		}
	}


	//InputDerivative(output, outputDerivative, inputGradient);
	if (!m_paddedGradientOut.GetCount())
	{
		ndInt32 paddSizeWidth = m_inputWidth + m_kernelSize - 1;
		ndInt32 paddSizeHeight = m_inputHeight + m_kernelSize - 1;
		m_paddedGradientOut.SetCount(paddSizeWidth * paddSizeHeight);
		m_paddedGradientOut.Set(ndBrainFloat(0.0f));
	}

	ndInt32 inputOffset = 0;
	ndBrainFloat convKernelBuffer[256];
	ndBrainMemVector convKernel(convKernelBuffer, kernelSize);

	auto CopyOutput = [this, &outputDerivative](ndInt32 filter)
	{
		ndInt32 srcOffset = 0;
		ndInt32 dstWidth = m_inputWidth + m_kernelSize - 1;
		ndInt32 outputSize = m_outputWidth * m_outputHeight;
		ndInt32 dstOffset = dstWidth * (m_kernelSize - 1) + m_kernelSize - 1;
		const ndBrainMemVector srcGrad(&outputDerivative[filter * outputSize], outputSize);
		for (ndInt32 y = 0; y < m_outputHeight; ++y)
		{
			const ndBrainMemVector src(&srcGrad[srcOffset], m_outputWidth);
			ndBrainMemVector dst(&m_paddedGradientOut[dstOffset], m_outputWidth);
			dst.Set(src);
			dstOffset += dstWidth;
			srcOffset += m_outputWidth;
		}
	};

	auto RotateKernel = [&convKernel](const ndBrainMemVector& kernel)
	{
		ndAssert(convKernel.GetCount() == kernel.GetCount());
		for (ndInt32 i = kernel.GetCount() - 1; i >= 0; --i)
		{
			convKernel[kernel.GetCount() - 1 - i] = kernel[i];
		}
	};

	auto CalculateInpuGradient = [this](const ndBrainVector& filter, ndBrainVector& output)
	{
		auto CrossCorrelation = [this, &filter](const ndBrainVector& input)
		{
			ndBrainFloat value = ndBrainFloat(0.0f);
			for (ndInt32 i = m_inputGradOffsets.GetCount() - 1; i >= 0 ; --i)
			{
				ndInt32 index = m_inputGradOffsets[i];
				value += input[index] * filter[i];
			}
			return value;
		};
		
		ndInt32 outputOffset = 0;
		ndInt32 gradInputOffset = 0;
		const ndInt32 gradInputWidth = m_inputWidth + m_kernelSize - 1;
		const ndInt32 gradInputSize = gradInputWidth * m_kernelSize;
		const ndInt32 kernelSize = m_kernelSize * m_kernelSize;
		for (ndInt32 y = 0; y < m_inputHeight; ++y)
		{
			for (ndInt32 x = 0; x < m_inputWidth; ++x)
			{
				const ndBrainMemVector in(&m_paddedGradientOut[gradInputOffset + x], gradInputSize);
				output[outputOffset + x] += CrossCorrelation(in);
			}
			outputOffset += m_inputWidth;
			gradInputOffset += gradInputWidth;
		}
	};

	kernelOffset = 0;
	inputGradient.Set(ndBrainFloat(0.0f));
	for (ndInt32 filter = 0; filter < m_numberOfKernels; ++filter)
	{
		inputOffset = 0;
		CopyOutput(filter);
		for (ndInt32 channel = 0; channel < m_inputDepth; ++channel)
		{
			ndBrainMemVector inputGrad(&inputGradient[inputOffset], inputSize);
			const ndBrainMemVector kernelGradients(&m_kernels[kernelOffset], kernelSize);
			RotateKernel(kernelGradients);
			CalculateInpuGradient(kernelGradients, inputGrad);
			inputOffset += inputSize;
			kernelOffset += kernelSize;
		}
	}
}
