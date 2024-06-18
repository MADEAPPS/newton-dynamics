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
#include "ndBrainGpuBuffer.h"
#include "ndBrainGpuCommand.h"
#include "ndBrainGpuContext.h"
#include "ndBrainLayerLinear.h"

//#define D_USE_GPU_TILED_MATRIX

#if defined (D_USE_GPU_TILED_MATRIX)
	#define ND_MATRIX_TILE_SIZE 16
#else
	#define ND_MATRIX_TILE_SIZE 1
#endif

ndBrainLayerLinear::ndBrainLayerLinear(ndInt32 inputs, ndInt32 outputs)
	:ndBrainLayer()
	,m_bias()
	,m_weights(outputs, inputs)
{
	m_bias.SetCount(outputs);
}

ndBrainLayerLinear::ndBrainLayerLinear(const ndBrainLayerLinear& src)
	:ndBrainLayer(src)
	,m_bias(src.m_bias)
	,m_weights(src.m_weights)
{
}

ndBrainLayerLinear::~ndBrainLayerLinear()
{
}

const char* ndBrainLayerLinear::GetLabelId() const
{
	return "ndBrainLayerLinear";
}

ndBrainLayer* ndBrainLayerLinear::Clone() const
{
	return new ndBrainLayerLinear(*this);
}

ndInt32 ndBrainLayerLinear::GetOutputSize() const
{
	ndAssert(m_bias.GetCount() == m_weights.GetRows());
	return m_bias.GetCount();
}

ndInt32 ndBrainLayerLinear::GetInputSize() const
{
	return m_weights.GetColumns();
}

ndBrainVector* ndBrainLayerLinear::GetBias()
{
	return &m_bias;
}

ndBrainMatrix* ndBrainLayerLinear::GetWeights()
{
	return &m_weights;
}

ndInt32 ndBrainLayerLinear::GetNumberOfParameters() const
{
	return m_bias.GetCount() + m_weights.GetColumns() * m_weights.GetRows();
}

bool ndBrainLayerLinear::HasParameters() const
{
	return true;
}

void ndBrainLayerLinear::InitWeightsXavierMethod()
{
	ndBrainFloat weighVariance = ndBrainFloat(ndSqrt(ndFloat32(6.0f) / ndFloat32(GetInputSize() + GetOutputSize())));
	InitWeights(weighVariance, ndBrainFloat(0.0f));
}

void ndBrainLayerLinear::InitGaussianBias(ndBrainFloat variance)
{
	m_bias.InitGaussianWeights(variance);
}

void ndBrainLayerLinear::InitGaussianWeights(ndBrainFloat variance)
{
	for (ndInt32 i = m_weights.GetCount() - 1; i >= 0; --i)
	{
		m_weights[i].InitGaussianWeights(variance);
	}
}

void ndBrainLayerLinear::InitWeights(ndBrainFloat weighVariance, ndBrainFloat biasVariance)
{
	biasVariance = ndMin(biasVariance, ndBrainFloat(0.5f));
	weighVariance = ndMin(weighVariance, ndBrainFloat(0.5f));
	InitGaussianBias(biasVariance);
	InitGaussianWeights(weighVariance);
}

void ndBrainLayerLinear::Clear()
{
	m_bias.Set(ndBrainFloat(0.0f));
	m_weights.Set(ndBrainFloat(0.0f));
}

void ndBrainLayerLinear::FlushToZero()
{
	m_bias.FlushToZero();
	m_weights.FlushToZero();
}

void ndBrainLayerLinear::Scale(ndBrainFloat scale)
{
	m_bias.Scale(scale);
	m_weights.Scale(scale);
}

void ndBrainLayerLinear::Set(const ndBrainLayer& src)
{
	const ndBrainLayerLinear& linearSrc = (ndBrainLayerLinear&)src;
	m_bias.Set(linearSrc.m_bias);
	m_weights.Set(linearSrc.m_weights);
}

void ndBrainLayerLinear::Add(const ndBrainLayer& src)
{
	const ndBrainLayerLinear& linearSrc = (ndBrainLayerLinear&)src;
	m_bias.Add(linearSrc.m_bias);
	m_weights.Add(linearSrc.m_weights);
}

void ndBrainLayerLinear::Mul(const ndBrainLayer& src)
{
	const ndBrainLayerLinear& linearSrc = (ndBrainLayerLinear&)src;
	m_bias.Mul(linearSrc.m_bias);
	m_weights.Mul(linearSrc.m_weights);
}

void ndBrainLayerLinear::ScaleAdd(const ndBrainLayer& src, ndBrainFloat scale)
{
	const ndBrainLayerLinear& linearSrc = (ndBrainLayerLinear&)src;
	m_bias.ScaleAdd(linearSrc.m_bias, scale);
	m_weights.ScaleAdd(linearSrc.m_weights, scale);
}

void ndBrainLayerLinear::Blend(const ndBrainLayer& src, ndBrainFloat blend)
{
	const ndBrainLayerLinear& linearSrc = (ndBrainLayerLinear&)src;
	m_bias.Blend(linearSrc.m_bias, blend);
	m_weights.Blend(linearSrc.m_weights, blend);
}

void ndBrainLayerLinear::AdamUpdate(const ndBrainLayer& u, const ndBrainLayer& v, ndBrainFloat epsilon)
{
	const ndBrainLayerLinear& linear_U = (ndBrainLayerLinear&)u;
	const ndBrainLayerLinear& linear_V = (ndBrainLayerLinear&)v;

	const ndBrainVector& bias_U = linear_U.m_bias;
	const ndBrainVector& bias_V = linear_V.m_bias;
	for (ndInt32 i = m_bias.GetCount() - 1; i >= 0; --i)
	{
		ndBrainFloat bias_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(bias_V[i])) + epsilon);
		m_bias[i] = bias_U[i] * bias_den;
	}

	const ndBrainMatrix& weight_U = linear_U.m_weights;
	const ndBrainMatrix& weight_V = linear_V.m_weights;
	for (ndInt32 i = m_weights.GetRows() - 1; i >= 0; --i)
	{
		for (ndInt32 j = m_weights[i].GetCount() - 1; j >= 0; --j)
		{
			ndBrainFloat weight_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(weight_V[i][j])) + epsilon);
			m_weights[i][j] = weight_U[i][j] * weight_den;
		}
	}
}

void ndBrainLayerLinear::Save(const ndBrainSave* const loadSave) const
{
	char buffer[1024];
	auto Save = [this, &buffer, &loadSave](const char* const fmt, ...)
	{
		va_list v_args;
		buffer[0] = 0;
		va_start(v_args, fmt);
		vsnprintf(buffer, sizeof(buffer), fmt, v_args);
		va_end(v_args);
		loadSave->WriteData(buffer);
	};

	Save("\tinputs %d\n", m_weights.GetColumns());
	Save("\touputs %d\n", m_weights.GetCount());

	Save("\tbias ");
	for (ndInt32 i = 0; i < m_bias.GetCount(); ++i)
	{
		Save("%g ", m_bias[i]);
	}
	Save("\n");

	Save("\tweights\n");
	for (ndInt32 i = 0; i < m_weights.GetCount(); ++i)
	{
		Save("\t\trow_%d ", i);
		const ndBrainVector& row = m_weights[i];
		for (ndInt32 j = 0; j < GetInputSize(); ++j)
		{
			Save("%g ", row[j]);
		}
		Save("\n");
	}
}

ndBrainLayer* ndBrainLayerLinear::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);

	loadSave->ReadString(buffer);
	ndInt32 inputs = loadSave->ReadInt();
	loadSave->ReadString(buffer);
	ndInt32 outputs = loadSave->ReadInt();
	ndBrainLayerLinear* const layer = new ndBrainLayerLinear(inputs, outputs);

	loadSave->ReadString(buffer);
	for (ndInt32 i = 0; i < outputs; ++i)
	{
		ndBrainFloat val = ndBrainFloat(loadSave->ReadFloat());
		layer->m_bias[i] = val;
	}

	loadSave->ReadString(buffer);
	for (ndInt32 i = 0; i < outputs; ++i)
	{
		loadSave->ReadString(buffer);
		for (ndInt32 j = 0; j < inputs; ++j)
		{
			ndBrainFloat val = ndBrainFloat(loadSave->ReadFloat());
			layer->m_weights[i][j] = val;
		}
	}

	loadSave->ReadString(buffer);
	return layer;
}

void ndBrainLayerLinear::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	m_weights.Mul(input, output);
	output.Add(m_bias);
}

void ndBrainLayerLinear::InputDerivative(const ndBrainVector&, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	ndAssert(0);
	m_weights.TransposeMul(outputDerivative, inputDerivative);
}

void ndBrainLayerLinear::CalculateParamGradients(
	const ndBrainVector& input, const ndBrainVector& ,
	const ndBrainVector& outputDerivative, ndBrainVector& inputGradient, ndBrainLayer* const gradientOut) const
{
	ndAssert(!strcmp(GetLabelId(), gradientOut->GetLabelId()));
	ndBrainLayerLinear* const gradients = (ndBrainLayerLinear*)gradientOut;
	ndAssert(gradients->m_bias.GetCount() == outputDerivative.GetCount());

	gradients->m_bias.Set(outputDerivative);
	for (ndInt32 i = outputDerivative.GetCount() - 1; i >= 0; --i)
	{
		ndBrainFloat value = outputDerivative[i];
		gradients->m_weights[i].ScaleSet(input, value);
	}

	m_weights.TransposeMul(outputDerivative, inputGradient);
}

void ndBrainLayerLinear::GetNumberOfGPUParameters(ndBrainVector& parameters, ndArray<ndInt32>& offsets) const
{
	ndInt32 rounding = ND_GPU_BUFFER_ALIGNMENT / sizeof(ndBrainFloat);
	ndAssert(!(rounding & (rounding - 1)));

	ndAssert(m_bias.GetCount() == m_weights.GetRows());

	ndInt32 rowsStride = (GetOutputSize() + rounding - 1) & -rounding;
	ndInt32 columnsStride = (GetInputSize() + rounding - 1) & -rounding;

	ndInt32 size = columnsStride * GetOutputSize() + rowsStride;
	offsets.PushBack(size);

	ndInt32 paramStart = parameters.GetCount();
	parameters.SetCount(paramStart + size);

	ndBrainMemVector memData(&parameters[paramStart], size);
	memData.Set(ndBrainFloat(-999999999.0f));

	ndInt32 stride = 0;
	for (ndInt32 i = 0; i < m_weights.GetRows(); ++i)
	{
		const ndBrainVector& src = m_weights[i];
		ndBrainMemVector dst(&memData[stride], src.GetCount());
		dst.Set(src);
		stride += columnsStride;
	}
	ndBrainMemVector bias(&memData[stride], m_bias.GetCount());
	bias.Set(m_bias);
}

ndBrainGpuCommand* ndBrainLayerLinear::AssemblyGPUCommand(ndBrainGpuContext* const context, ndInt32 layerIndex, ndInt32 batchCount, ndFixSizeArray<ndBufferOffsetPair*, 8>& params)
{
	class ndBrainLayerLinearCommand : public ndBrainGpuCommand
	{
		struct UniformBufferObject
		{
			ndInt32 m_matrixRows;
			ndInt32 m_matrixColumns;
			ndInt32 m_matrixRowsStride;
			ndInt32 m_matrixColumnsStride;
			ndInt32 m_workGroupsPerMatrix;

			ndInt32 m_paramStart;
			ndInt32 m_inputStart;
			ndInt32 m_outputStart;
			ndInt32 m_workBufferSize;
		};

		public:
		ndBrainLayerLinearCommand(
			const ndBrainLayerLinear* const layer, ndBrainGpuContext* const context,
			ndInt32 layerIndex, ndInt32 batchCount,
			const ndBufferOffsetPair& parameterBuffer, const ndBufferOffsetPair& workingBuffer)
			:ndBrainGpuCommand(context)
			,m_parammeters(m_context, sizeof(UniformBufferObject))
		{
			UniformBufferObject uniformParam;
			memset(&uniformParam, -1, sizeof(uniformParam));

			ndInt32 rounding = ND_GPU_BUFFER_ALIGNMENT / sizeof(ndBrainFloat);
			ndInt32 rowsStride = (layer->GetOutputSize() + rounding - 1) & -rounding;
			ndInt32 columnsStride = (layer->GetInputSize() + rounding - 1) & -rounding;

			uniformParam.m_matrixRows = layer->GetOutputSize();
			uniformParam.m_matrixColumns = layer->GetInputSize();
			uniformParam.m_matrixRowsStride = rowsStride;
			uniformParam.m_matrixColumnsStride = columnsStride;
			uniformParam.m_workGroupsPerMatrix = ((uniformParam.m_matrixRows + ND_MATRIX_TILE_SIZE - 1) & -ND_MATRIX_TILE_SIZE) / ND_MATRIX_TILE_SIZE;

			uniformParam.m_inputStart = workingBuffer.m_offsets[layerIndex + 0];
			uniformParam.m_outputStart = workingBuffer.m_offsets[layerIndex + 1];
			uniformParam.m_workBufferSize = workingBuffer.m_offsets[workingBuffer.m_offsets.GetCount() - 1];
			uniformParam.m_paramStart = parameterBuffer.m_offsets[layerIndex];

			m_parammeters.LoadData(sizeof(uniformParam), &uniformParam);

			ndFixSizeArray<ndBrainGpuBuffer*, 4> params;
			params.PushBack(&m_parammeters);
			params.PushBack(workingBuffer.m_buffer);
			params.PushBack(parameterBuffer.m_buffer);

			ndInt32 numberOfWorkGroup = batchCount * uniformParam.m_workGroupsPerMatrix;
			#if defined (D_USE_GPU_TILED_MATRIX)
				Assembly(context->m_ndBrainLayerLinearTiled, numberOfWorkGroup, params.GetCount(), &params[0]);
			#else
				Assembly(context->m_ndBrainLayerLinear, numberOfWorkGroup, params.GetCount(), &params[0]);
			#endif
		}

		ndBrainGpuUniformBuffer m_parammeters;
	};

	ndAssert(params.GetCount() == 2);
	const ndBufferOffsetPair& parameterBuffer = *params[0];
	const ndBufferOffsetPair& workingBuffer = *params[1];
	return new ndBrainLayerLinearCommand(this, context, layerIndex, batchCount, parameterBuffer, workingBuffer);
}
