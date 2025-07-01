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
#include "ndBrainContext.h"
#include "ndBrainTrainer.h"
#include "ndBrainSaveLoad.h"
#include "ndBrainGpuCommand.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainFloatBuffer.h"

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
	return ND_BRAIN_LAYER_LINEAR_NAME;
}

ndBrainLayer* ndBrainLayerLinear::Clone() const
{
	return new ndBrainLayerLinear(*this);
}

ndInt32 ndBrainLayerLinear::GetOutputSize() const
{
	ndAssert(m_bias.GetCount() == m_weights.GetRows());
	return ndInt32(m_bias.GetCount());
}

ndInt32 ndBrainLayerLinear::GetInputSize() const
{
	return m_weights.GetColumns();
}

void ndBrainLayerLinear::CalculateRoundedSize(ndInt32& width, ndInt32& height) const
{
	height = (GetOutputSize() + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
	width = (GetInputSize() + ND_GPU_TILED_MATRIX_COLUMNS - 1) & -ND_GPU_TILED_MATRIX_COLUMNS;
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
	return ndInt32(m_bias.GetCount()) + m_weights.GetColumns() * m_weights.GetRows();
}

bool ndBrainLayerLinear::HasParameters() const
{
	return true;
}

void ndBrainLayerLinear::InitWeights()
{
	m_bias.Set(ndBrainFloat(0.0f));
	ndBrainFloat variance = ndBrainFloat(ndSqrt(ndFloat32(1.0f) / ndFloat32(GetOutputSize())));
	for (ndInt32 i = ndInt32(m_weights.GetCount() - 1); i >= 0; --i)
	{
		m_weights[i].InitGaussianWeights(variance);
	}

#if 0
	for (ndInt32 i = 0; i < ndInt32(m_bias.GetCount()); ++i)
	{
		//m_bias[i] = ndFloat32(i);
		m_bias[i] = ndFloat32(0.0f);
	}
	for (ndInt32 i = 0; i < ndInt32(m_weights.GetCount()); ++i)
	{
		m_weights[i].Set(ndFloat32(0.0f));
		for (ndInt32 j = 0; j < ndInt32(m_weights.GetColumns()); ++j)
		{
			m_weights[i][j] = ndFloat32(i + j) * 0.0001f;
		}
	}
#endif
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

void ndBrainLayerLinear::AddReqularizerL2(const ndBrainLayer& weights, ndBrainFloat regularizer)
{
	ScaleAdd(weights, regularizer);
}

void ndBrainLayerLinear::AddReqularizerL1(const ndBrainLayer& weights, ndBrainFloat regularizer)
{
	ScaleAdd(weights, regularizer);

	ndBrainFloat negativeRegularizer = -regularizer;
	for (ndInt32 i = ndInt32(m_bias.GetCount()) - 1; i >= 0; --i)
	{
		ndBrainFloat b = m_bias[i];
		m_bias[i] += (b > ndFloat32(0.0f)) ? regularizer : negativeRegularizer;

		ndBrainMemVector& row = m_weights[i];
		for (ndInt32 j = ndInt32(row.GetCount()) - 1; j >= 0; --j)
		{
			ndBrainFloat w = row[j];
			row[j] += (w > ndFloat32(0.0f)) ? regularizer : negativeRegularizer;
		}
	}
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
	for (ndInt32 i = ndInt32(m_bias.GetCount() - 1); i >= 0; --i)
	{
		ndBrainFloat bias_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(bias_V[i])) + epsilon);
		m_bias[i] = bias_U[i] * bias_den;
	}

	const ndBrainMatrix& weight_U = linear_U.m_weights;
	const ndBrainMatrix& weight_V = linear_V.m_weights;
	for (ndInt32 i = m_weights.GetRows() - 1; i >= 0; --i)
	{
		ndBrainMemVector& row = m_weights[i];
		const ndBrainMemVector& row_U = weight_U[i];
		const ndBrainMemVector& row_V = weight_V[i];
		for (ndInt32 j = ndInt32(row.GetCount() - 1); j >= 0; --j)
		{
			ndBrainFloat weight_den = ndBrainFloat(1.0f) / (ndBrainFloat(ndSqrt(row_V[j])) + epsilon);
			row[j] = row_U[j] * weight_den;
		}
	}
}

void ndBrainLayerLinear::Save(const ndBrainSave* const loadSave) const
{
	char buffer[1024];
	auto Save = [&buffer, &loadSave](const char* const fmt, ...)
	{
		va_list v_args;
		buffer[0] = 0;
		va_start(v_args, fmt);
		vsnprintf(buffer, sizeof(buffer), fmt, v_args);
		va_end(v_args);
		loadSave->WriteData(buffer);
	};

	Save("\tinputs %d\n", m_weights.GetColumns());
	Save("\toutputs %d\n", m_weights.GetCount());

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

void ndBrainLayerLinear::InputDerivative(const ndBrainVector&, const ndBrainVector&, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
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
	for (ndInt32 i = ndInt32(outputDerivative.GetCount() - 1); i >= 0; --i)
	{
		ndBrainFloat value = outputDerivative[i];
		gradients->m_weights[i].ScaleSet(input, value);
	}

	m_weights.TransposeMul(outputDerivative, inputGradient);
}

bool ndBrainLayerLinear::HasGpuSupport() const
{
	return true;
}

void ndBrainLayerLinear::CopyCpuWeights(ndBrainVector& output) const
{
	ndInt32 width = GetInputSize();
	ndInt32 height = GetOutputSize();
	ndAssert(output.GetCount() >= (GetOutputSize() * GetInputSize() + GetOutputSize()));
	ndAssert(output.GetCount() >= height * width + GetOutputSize());

	ndInt32 offset = 0;
	output.Set(ndBrainFloat(0.0f));
	ndInt32 columns = m_weights.GetColumns();
	for (ndInt32 i = 0; i < m_weights.GetRows(); ++i)
	{
		const ndBrainVector& src = m_weights[i];
		ndBrainMemVector dst(&output[offset], columns);
		dst.Set(src);
		offset += width;
		ndAssert(offset >= 0);
	}
	ndBrainMemVector bias(&output[height * width], m_bias.GetCount());
	bias.Set(m_bias);
}

void ndBrainLayerLinear::CopyGpuWeights(ndBrainVector& output) const
{
	ndInt32 width;
	ndInt32 height;

	CalculateRoundedSize(width, height);
	ndAssert(output.GetCount() >= (GetOutputSize() * GetInputSize() + GetOutputSize()));
	ndAssert(output.GetCount() >= height * width + GetOutputSize());

	ndInt32 offset = 0;
	output.Set(ndBrainFloat(0.0f));
	ndInt32 columns = m_weights.GetColumns();
	for (ndInt32 i = 0; i < m_weights.GetRows(); ++i)
	{
		const ndBrainVector& src = m_weights[i];
		ndBrainMemVector dst(&output[offset], columns);
		dst.Set(src);
		offset += width;
		ndAssert(offset >= 0);
	}
	ndBrainMemVector bias(&output[height * width], m_bias.GetCount());
	bias.Set(m_bias);
}

void ndBrainLayerLinear::SetCpuWeights(const ndBrainVector& input)
{
	ndInt32 width = GetInputSize();
	ndAssert(input.GetCount() >= (GetOutputSize() * GetInputSize() + GetOutputSize()));

	ndInt32 offset = 0;
	ndInt32 columns = m_weights.GetColumns();
	for (ndInt32 i = 0; i < m_weights.GetRows(); ++i)
	{
		ndBrainVector& dst = m_weights[i];
		const ndBrainMemVector src(&input[offset], columns);
		dst.Set(src);
		offset += width;
		ndAssert(offset >= 0);
	}
	const ndBrainMemVector bias(&input[offset], m_bias.GetCount());
	m_bias.Set(bias);
}

void ndBrainLayerLinear::SetGpuWeights(const ndBrainVector& input)
{
	ndInt32 width;
	ndInt32 height;

	CalculateRoundedSize(width, height);
	ndAssert(input.GetCount() >= (GetOutputSize() * GetInputSize() + GetOutputSize()));

	ndInt32 offset = 0;
	ndInt32 columns = m_weights.GetColumns();
	for (ndInt32 i = 0; i < m_weights.GetRows(); ++i)
	{
		ndBrainVector& dst = m_weights[i];
		const ndBrainMemVector src(&input[offset], columns);
		dst.Set(src);
		offset += width;
		ndAssert(offset >= 0);
	}
	const ndBrainMemVector bias(&input[offset], m_bias.GetCount());
	m_bias.Set(bias);
}

ndCommandSharedInfo ndBrainLayerLinear::GetCpuCommandSharedInfo() const
{
	ndCommandSharedInfo info(this);

	ndInt32 rows = m_weights.GetRows();
	ndInt32 columns = m_weights.GetColumns();

	info.m_outputSize = rows;
	info.m_inputSize = columns;
	info.m_parametersBatchSize = rows * columns + GetOutputSize();
	return info;
}

ndCommandSharedInfo ndBrainLayerLinear::GetGpuCommandSharedInfo() const
{
	ndCommandSharedInfo info(this);

	ndInt32 rows = m_weights.GetRows();
	ndInt32 columns = m_weights.GetColumns();

	info.m_outputSize = rows;
	info.m_inputSize = columns;

	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	info.m_parametersBatchSize = width * height + GetOutputSize();
	return info;
}

void ndBrainLayerLinear::FeedForward(const ndBrainLayerFeedForwardCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainerInference* const trainer = desc.m_owner;
	
	const ndBrainFloat* const weightAndBias = (ndBrainFloat*)trainer->GetWeightAndBiasBuffer()->GetCpuPtr();
	const ndBrainFloat* const inputOutputBuffer = (ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr();

	ndInt32 inputSize = info.m_inputSize;
	ndInt32 outputSize = info.m_outputSize;
	ndInt32 matrixSize = inputSize * outputSize;
	const ndBrainMemVector parameters(&weightAndBias[info.m_parametersStartOffset], matrixSize + outputSize);
	
	ndInt32 offset = miniBatchIndex * info.m_inputOutputSize + info.m_inputOutputStartOffset;
	ndAssert(offset >= 0);

	const ndBrainMemVector input(&inputOutputBuffer[offset], inputSize);
	ndBrainMemVector output(&inputOutputBuffer[offset + inputSize], outputSize);
	for (ndInt32 i = outputSize - 1; i >= 0; --i)
	{
		const ndBrainMemVector row(&parameters[i * inputSize], inputSize);
		output[i] = row.Dot(input);
	}
	
	const ndBrainMemVector bias(&parameters[matrixSize], outputSize);
	output.Add(bias);
}

void ndBrainLayerLinear::BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

	const ndBrainFloat* const weightAndBias = (ndBrainFloat*)trainer->GetWeightAndBiasBuffer()->GetCpuPtr();
	const ndBrainFloat* const inputOutputBuffer = (ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr();
	const ndBrainFloat* const inputOutputGradientsBuffer = (ndBrainFloat*)trainer->GetHiddenLayerGradientBuffer()->GetCpuPtr();
	const ndBrainFloat* const weightAndBiasGradients = (ndBrainFloat*)trainer->GetWeightAndBiasGradientBuffer()->GetCpuPtr();

	ndInt32 inputSize = info.m_inputSize;
	ndInt32 outputSize = info.m_outputSize;
	ndInt32 matrixSize = inputSize * outputSize;
	
	ndInt32 offset = miniBatchIndex * info.m_inputOutputSize + info.m_inputOutputStartOffset;
	ndAssert(offset >= 0);
	const ndBrainMemVector input(&inputOutputBuffer[offset], inputSize);
	const ndBrainMemVector output(&inputOutputBuffer[offset + inputSize], outputSize);
	const ndBrainMemVector outputDerivative(&inputOutputGradientsBuffer[offset + inputSize], outputSize);
	
	// calculate input output gradients
	ndBrainMemVector inputDerivative(&inputOutputGradientsBuffer[offset], inputSize);
	const ndBrainMemVector matrixVector(&weightAndBias[info.m_parametersStartOffset], matrixSize + outputSize);
	inputDerivative.Set(ndBrainFloat(0.0f));
	for (ndInt32 i = outputSize - 1; i >= 0; --i)
	{
		ndFloat32 scale = outputDerivative[i];
		const ndBrainMemVector row(&matrixVector[i * inputSize], inputSize);
		inputDerivative.ScaleAdd(row, scale);
	}
	
	// calculate weight and bias gradients
	ndInt64 gradientOffset = ndInt64(miniBatchIndex) * info.m_parametersBatchSize + info.m_parametersStartOffset;
	ndAssert(gradientOffset >= 0);

	ndBrainMemVector gradients(&weightAndBiasGradients[gradientOffset], matrixSize + outputSize);
	ndBrainMemVector biasRow(&gradients[matrixSize], outputSize);
	biasRow.Set(outputDerivative);
	
	for (ndInt32 i = outputSize - 1; i >= 0; --i)
	{
		ndBrainFloat value = outputDerivative[i];
		ndBrainMemVector gradientRow(&gradients[i * inputSize], inputSize);
		gradientRow.ScaleSet(input, value);
	}
}

ndBrainBufferCommand* ndBrainLayerLinear::CreateGpuFeedForwardCommand(
	ndBrainTrainerInference* const owner,
	const ndCommandSharedInfo& info,
	ndBrainContext* const context, ndInt32 miniBatchSize,
	const ndSharedPtr<ndBrainUniformBuffer>& uniformBuffer,
	ndBrainFloatBuffer* const inputOutputData,
	ndBrainFloatBuffer* const weightsAndBias) const
{
	if (context->GetAsCpuContext())
	{
		ndBrainBufferCommandDesc descriptor(miniBatchSize);
		descriptor.m_id = size_t(this);
		descriptor.m_context = context;
		descriptor.m_owner = owner;
		descriptor.m_info = info;
		descriptor.m_uniformBuffer = uniformBuffer;

		descriptor.PushBack((ndBrainUniformBuffer*)*uniformBuffer);
		descriptor.PushBack(inputOutputData);
		descriptor.PushBack(weightsAndBias);

		ndBrainBufferCommand* const command = new ndBrainLayerFeedForwardCpuCommand(descriptor, (ndBrainLayer*)this);
		return command;
	}
	else
	{
		ndAssert(info.m_parametersBatchSize);
		ndAssert((miniBatchSize % ND_GPU_TILED_MATRIX_ROWS) == 0);
		
		ndInt32 width;
		ndInt32 height;
		CalculateRoundedSize(width, height);
		
		ndInt32 blockRows = height / ND_GPU_TILED_MATRIX_ROWS;
		ndInt32 blockColums = miniBatchSize / ND_GPU_TILED_MATRIX_ROWS;
		
		ndCommandSharedInfo newInfo;
		uniformBuffer->MemoryFromDevice(0, sizeof(ndCommandSharedInfo), &newInfo);
		newInfo.m_tiledStride = blockRows;
		uniformBuffer->MemoryToDevice(0, sizeof(ndCommandSharedInfo), &newInfo);

		ndBrainBufferCommandDesc descriptor(miniBatchSize);
		descriptor.m_id = size_t(this);
		descriptor.m_context = context;
		descriptor.m_owner = owner;
		descriptor.m_info = newInfo;
		descriptor.m_miniBatchSize = blockRows * blockColums;
		descriptor.m_uniformBuffer = uniformBuffer;
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixMatrixMultiply;

		descriptor.PushBack((ndBrainUniformBuffer*)*uniformBuffer);
		descriptor.PushBack(inputOutputData);
		descriptor.PushBack(weightsAndBias);
				
		//ndBrainBufferCommand* const command = new ndBrainTrainerGpuCommand(
		//	owner, newInfo, size_t(this), context, context->m_brainLayerMatrixMatrixMultiply, miniBatchSize, uniformBuffer, inputOutputData, weightsAndBias);
		ndBrainBufferCommand* const command = new ndBrainGpuCommand(descriptor);
		return command;
	}
}

ndBrainBufferCommand* ndBrainLayerLinear::CreateGpuBackPropagateCommand(
	ndBrainTrainerInference* const owner,
	const ndCommandSharedInfo& info,
	ndBrainContext* const context, ndInt32 miniBatchSize,
	const ndSharedPtr<ndBrainUniformBuffer>& uniformBuffer,
	ndBrainFloatBuffer* const inputOutputData,
	ndBrainFloatBuffer* const weightsAndBias,
	ndBrainFloatBuffer* const inputOutputGradients,
	ndBrainFloatBuffer* const weightsAndBiasGradients) const
{
	ndBrainBufferCommandDesc descriptor(miniBatchSize);
	descriptor.m_id = size_t(this);
	descriptor.m_context = context;
	descriptor.m_owner = owner;
	descriptor.m_info = info;
	descriptor.m_uniformBuffer = uniformBuffer;

	descriptor.PushBack((ndBrainUniformBuffer*)*uniformBuffer);
	descriptor.PushBack(inputOutputData);
	descriptor.PushBack(weightsAndBias);
	descriptor.PushBack(inputOutputGradients);
	descriptor.PushBack(weightsAndBiasGradients);

	if (context->GetAsCpuContext())
	{
		ndBrainBufferCommand* const command = new ndBrainLayerBackPropagateCpuCommand(descriptor, (ndBrainLayer*)this);
		return command;
	}
	else
	{
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixVectorBackPropagate;
		//ndBrainBufferCommand* const command = new ndBrainTrainerGpuCommand(
		//	owner, info, size_t(this), context, context->m_brainLayerMatrixVectorBackPropagate,
		//	miniBatchSize, uniformBuffer, inputOutputData, weightsAndBias, inputOutputGradients, weightsAndBiasGradients);
		ndBrainBufferCommand* const command = new ndBrainGpuCommand(descriptor);
		return command;
	}
}
