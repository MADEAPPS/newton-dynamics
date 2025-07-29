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
	width = GetInputSize();
	height = GetOutputSize();
	height = (height + ND_GPU_TILED_MATRIX_ROWS - 1) & -ND_GPU_TILED_MATRIX_ROWS;
	width = (width + ND_GPU_TILED_MATRIX_ROWS * 2 - 1) & -ND_GPU_TILED_MATRIX_ROWS * 2;
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
		//m_bias[i] = ndFloat32(i + 1) * 10000.0f;
		m_bias[i] = 0.0f;
	}

	m_weights[0].Set(ndFloat32(0.0f));
	m_weights[0][0] = 1.0f;
	for (ndInt32 i = 1; i < ndInt32(m_weights.GetCount()); ++i)
	{
		m_weights[i].Set(ndFloat32(0.0f));
		m_weights[i][i] = ndFloat32(i + 1);
		m_weights[i - 1][i] = ndFloat32(i + 1);
		m_weights[i][i-1] = ndFloat32(i);
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

void ndBrainLayerLinear::CopyWeights(ndBrainTrainerInference* const trainer, ndBrainVector& output) const
{
	ndInt32 width;
	ndInt32 height;

	CalculateRoundedSize(width, height);
	ndInt32 matrixSize = trainer->RoundOffOffset(width * height);
	ndAssert(output.GetCount() >= (matrixSize + trainer->RoundOffOffset(GetOutputSize())));
	output.Set(ndBrainFloat(0.0f));

	ndInt32 offset = 0;
	ndInt32 columns = m_weights.GetColumns();
	for (ndInt32 i = 0; i < m_weights.GetRows(); ++i)
	{
		const ndBrainVector& src = m_weights[i];
		ndBrainMemVector dst(&output[offset], columns);
		dst.Set(src);
		offset += width;
		ndAssert(offset >= 0);
	}
	ndBrainMemVector bias(&output[matrixSize], m_bias.GetCount());
	bias.Set(m_bias);
}

void ndBrainLayerLinear::SetWeights(ndBrainTrainerInference* const trainer, const ndBrainVector& weightsAnBias)
{
	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	ndInt32 matrixSize = trainer->RoundOffOffset(width * height);
	ndAssert(weightsAnBias.GetCount() >= (matrixSize + trainer->RoundOffOffset(GetOutputSize())));

	ndInt32 offset = 0;
	ndInt32 columns = m_weights.GetColumns();
	for (ndInt32 i = 0; i < m_weights.GetRows(); ++i)
	{
		ndBrainVector& dst = m_weights[i];
		const ndBrainMemVector src(&weightsAnBias[offset], columns);
		dst.Set(src);
		offset += width;
		ndAssert(offset >= 0);
	}
	const ndBrainMemVector bias(&weightsAnBias[matrixSize], m_bias.GetCount());
	m_bias.Set(bias);
}

ndCommandSharedInfo ndBrainLayerLinear::GetCommandSharedInfo(ndBrainTrainerInference* const trainer) const
{
	ndCommandSharedInfo info(this);

	ndInt32 rows = m_weights.GetRows();
	ndInt32 columns = m_weights.GetColumns();

	info.m_outputSize = rows;
	info.m_inputSize = columns;

	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	ndInt32 matrixSize = trainer->RoundOffOffset(width * height);
	info.m_parametersBatchSize = matrixSize + trainer->RoundOffOffset(rows);
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
	ndInt32 inputOutputSize = info.m_inputOutputSize;
	ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	ndInt32 matrixSize = width * height;
	const ndBrainMemVector parameters(&weightAndBias[info.m_parametersStartOffset], matrixSize + outputSize);
	
	ndInt64 inputOffset = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	ndInt64 outputOffset = inputOffset + trainer->RoundOffOffset(inputSize);

	ndBrainMemVector output(&inputOutputBuffer[outputOffset], outputSize);
	const ndBrainMemVector input(&inputOutputBuffer[inputOffset], inputSize);
	for (ndInt32 i = outputSize - 1; i >= 0; --i)
	{
		const ndBrainMemVector row(&parameters[i * width], inputSize);
		output[i] = row.Dot(input);
	}
	
	const ndBrainMemVector bias(&parameters[matrixSize], outputSize);
	output.Add(bias);
}

void ndBrainLayerLinear::BackPropagate(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	switch (info.m_matrixDimensionK & (m_dimFactor - 1))
	{
		case m_biasPass:
			BackPropagateBiasGradients(command, miniBatchIndex);
			break;
		case m_biasAddPartialSumPass:
			BackPropagateBiasAddPartialSumGradients(command, miniBatchIndex);
			break;
		case m_biasCachePartialSumPass:
			BackPropagateBiasCachePartialSumGradients(command, miniBatchIndex);
			break;
		case m_weightsPass:
			BackPropagateWeightsGradients(command, miniBatchIndex);
			break;
		case m_inputGradientsPass:
			BackPropagateInputGradients(command, miniBatchIndex);
		default:;
	}
}

void ndBrainLayerLinear::BackPropagateBiasGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

	const ndBrainFloat* const partialBiasSumBuffer = (ndBrainFloat*)trainer->GetPartialSumBiasGradientBuffer()->GetCpuPtr();
	const ndBrainFloat* const weightAndBiasGradients = (ndBrainFloat*)trainer->GetWeightAndBiasGradientBuffer()->GetCpuPtr();

	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	ndInt32 matrixSize = trainer->RoundOffOffset(width * height);

	ndInt32 outputSize = info.m_outputSize;
	ndBrainMemVector biasRowGradients(&weightAndBiasGradients[info.m_parametersStartOffset + matrixSize], outputSize);
	const ndBrainMemVector outputDerivative(&partialBiasSumBuffer[0], outputSize);
	biasRowGradients.Set(outputDerivative);
}

void ndBrainLayerLinear::BackPropagateBiasCachePartialSumGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

	const ndBrainFloat* const inputOutputGradientsBuffer = (ndBrainFloat*)trainer->GetHiddenLayerGradientBuffer()->GetCpuPtr();
	const ndBrainFloat* const partialBiasSumBuffer = (ndBrainFloat*)trainer->GetPartialSumBiasGradientBuffer()->GetCpuPtr();

	ndInt32 inputSize = info.m_inputSize;
	ndInt32 outputSize = info.m_outputSize;
	ndInt32 inputOutputSize = info.m_inputOutputSize;
	ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

	ndInt32 alignedOffset = trainer->RoundOffOffset(outputSize);
	const ndInt32 dstOffset = miniBatchIndex * alignedOffset;

	ndInt64 inputGradientOffset = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	ndInt64 outputGradientOffset = inputGradientOffset + trainer->RoundOffOffset(inputSize);

	const ndBrainMemVector srcGradients(&inputOutputGradientsBuffer[outputGradientOffset], outputSize);
	ndBrainMemVector dstBias (&partialBiasSumBuffer[dstOffset], outputSize);
	dstBias.Set(srcGradients);
}

void ndBrainLayerLinear::BackPropagateBiasAddPartialSumGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;	

	const ndInt32 partisionSize = info.m_matrixDimensionK / m_dimFactor;
	const ndInt32 halfPartisionIndex = miniBatchIndex + (partisionSize + 1) / 2;
	if (halfPartisionIndex < partisionSize)
	{
		ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;
		ndBrainFloat* const partialBiasSumBuffer = (ndBrainFloat*)trainer->GetPartialSumBiasGradientBuffer()->GetCpuPtr();

		ndInt32 outputSize = info.m_outputSize;
		ndInt32 alignedOffset = trainer->RoundOffOffset(outputSize);

		ndInt32 dstOffset = miniBatchIndex * alignedOffset;
		ndInt32 srcOffset = halfPartisionIndex * alignedOffset;

		ndBrainMemVector dstGradients(&partialBiasSumBuffer[dstOffset], outputSize);
		const ndBrainMemVector srcGradients(&partialBiasSumBuffer[srcOffset], outputSize);
		dstGradients.Add(srcGradients);
	}
}

void ndBrainLayerLinear::BackPropagateWeightsGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	ndBrainFixSizeVector<1024 * 8> cachedRowGradient;

	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

	const ndBrainFloat* const inputOutputBuffer = (ndBrainFloat*)trainer->GetHiddenLayerBuffer()->GetCpuPtr();
	const ndBrainFloat* const inputOutputGradientsBuffer = (ndBrainFloat*)trainer->GetHiddenLayerGradientBuffer()->GetCpuPtr();
	ndBrainFloat* const weightAndBiasGradients = (ndBrainFloat*)trainer->GetWeightAndBiasGradientBuffer()->GetCpuPtr();

	ndInt32 inputSize = info.m_inputSize;
	ndInt32 inputOutputSize = info.m_inputOutputSize;
	ndInt32 numberOfRows = info.m_matrixDimensionK / m_dimFactor;

	ndInt64 inputOutputStartOffset = info.m_inputOutputStartOffset;
	ndInt64 dstBase = inputOutputStartOffset + trainer->RoundOffOffset(inputSize);

	cachedRowGradient.SetCount(inputSize);
	cachedRowGradient.Set(ndBrainFloat(0.0f));
	for (ndInt32 row = 0; row < numberOfRows; ++row)
	{
		ndInt64 inputOffset = inputOutputStartOffset + row * inputOutputSize;
		ndInt64 outGradientOffset = dstBase + row * inputOutputSize + miniBatchIndex;
		ndBrainFloat outputDerivative = inputOutputGradientsBuffer[outGradientOffset];
	
		const ndBrainMemVector inputData(&inputOutputBuffer[inputOffset], inputSize);
		cachedRowGradient.ScaleAdd (inputData, outputDerivative);
	}

	// store this weight gradient sum
	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	ndInt64 parametersOffset = info.m_parametersStartOffset + miniBatchIndex * width;
	ndBrainMemVector weightGradientRow(&weightAndBiasGradients[parametersOffset], inputSize);
	weightGradientRow.Set(cachedRowGradient);
}

void ndBrainLayerLinear::BackPropagateInputGradients(const ndBrainLayerBackPropagateCpuCommand* const command, ndInt32 miniBatchIndex) const
{
	const ndBrainBufferCommandDesc& desc = command->GetDescriptor();
	const ndCommandSharedInfo& info = desc.m_info;
	ndBrainTrainer* const trainer = (ndBrainTrainer*)desc.m_owner;

	const ndBrainFloat* const weightAndBias = (ndBrainFloat*)trainer->GetWeightAndBiasBuffer()->GetCpuPtr();
	const ndBrainFloat* const inputOutputGradientsBuffer = (ndBrainFloat*)trainer->GetHiddenLayerGradientBuffer()->GetCpuPtr();

	ndInt32 inputSize = info.m_inputSize;
	ndInt32 outputSize = info.m_outputSize;
	ndInt32 inputOutputSize = info.m_inputOutputSize;
	ndInt32 inputOutputStartOffset = info.m_inputOutputStartOffset;

	ndInt32 width;
	ndInt32 height;
	CalculateRoundedSize(width, height);
	ndInt32 matrixSize = width * height;

	ndInt64 srcBase = miniBatchIndex * ndInt64(inputOutputSize) + inputOutputStartOffset;
	ndInt64 dstBase = srcBase + trainer->RoundOffOffset(inputSize);
	ndAssert(srcBase >= 0);
	ndAssert(dstBase >= 0);
	
	const ndBrainMemVector outputDerivative(&inputOutputGradientsBuffer[dstBase], outputSize);
	ndBrainMemVector inputDerivative(&inputOutputGradientsBuffer[srcBase], inputSize);
	const ndBrainMemVector weightsMatrix(&weightAndBias[info.m_parametersStartOffset], matrixSize);

	inputDerivative.Set(ndBrainFloat(0.0f));
	for (ndInt32 i = 0; i < outputSize; ++i)
	{
		ndBrainFloat outDerivative = outputDerivative[i];
		const ndBrainMemVector weightsRow(&weightsMatrix[i * width], inputSize);
		inputDerivative.ScaleAdd(weightsRow, outDerivative);
	}
}

ndCommandArray ndBrainLayerLinear::CreateGpuFeedForwardCommand(
	ndBrainTrainerInference* const owner,
	ndBrainContext* const context,
	const ndCommandSharedInfo& info,
	ndInt32 miniBatchSize,
	ndBrainFloatBuffer* const inputOutputData,
	ndBrainFloatBuffer* const weightsAndBias) const
{
	ndAssert(info.m_parametersBatchSize);

	ndCommandArray commandArray(0);
	if (context->GetAsCpuContext())
	{
		ndBrainBufferCommandDesc descriptor(MakeFeedForwardDesctriptor(
			owner, context, info, miniBatchSize, 0,
			inputOutputData, weightsAndBias));

		ndBrainBufferCommand* const command = new ndBrainLayerFeedForwardCpuCommand(descriptor, (ndBrainLayer*)this);
		commandArray.PushBack(command);
	}
	else
	{
		ndInt32 width;
		ndInt32 height;
		CalculateRoundedSize(width, height);
		ndAssert((miniBatchSize & (ND_GPU_TILED_MATRIX_ROWS - 1)) == 0);

		ndInt32 dim_M = height / ND_GPU_TILED_MATRIX_ROWS;
		ndInt32 dim_N = miniBatchSize / ND_GPU_TILED_MATRIX_ROWS;

		ndBrainBufferCommandDesc descriptor(MakeFeedForwardDesctriptor(
			owner, context, info, dim_M * dim_N, 0,
			inputOutputData, weightsAndBias));
		descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixMatrixMultiply;
		ndBrainBufferCommand* const command = new ndBrainGpuCommand(descriptor);
		commandArray.PushBack(command);
	}

	return commandArray;
}

ndCommandArray ndBrainLayerLinear::CreateGpuBackPropagateCommand(
	ndBrainTrainerInference* const owner,
	ndBrainContext* const context, 
	const ndCommandSharedInfo& info,
	ndInt32 miniBatchSize,
	ndBrainFloatBuffer* const inputOutputData,
	ndBrainFloatBuffer* const weightsAndBias,
	ndBrainFloatBuffer* const inputOutputGradients,
	ndBrainFloatBuffer* const weightsAndBiasGradients) const
{
	ndCommandArray commands(0);
	if (context->GetAsCpuContext())
	{
		{
			// calculate the input Gradiends
			ndBrainBufferCommandDesc descriptor(MakeBackpropagateDesctriptor(
				owner, context, info, miniBatchSize, m_inputGradientsPass,
				inputOutputData, weightsAndBias,
				inputOutputGradients, weightsAndBiasGradients));
			ndBrainBufferCommand* const command = new ndBrainLayerBackPropagateCpuCommand(descriptor, (ndBrainLayer*)this);
			commands.PushBack(command);
		}

		{
			// calculate the bias gradient
			ndInt32 size = miniBatchSize;
			ndBrainBufferCommandDesc clearBiasDescriptor(MakeBackpropagateDesctriptor(
				owner, context, info, size, (size * m_dimFactor) + m_biasCachePartialSumPass,
				inputOutputData, weightsAndBias,
				inputOutputGradients, weightsAndBiasGradients));
			ndBrainBufferCommand* const clearBiasCommand = new ndBrainLayerBackPropagateCpuCommand(clearBiasDescriptor, (ndBrainLayer*)this);
			commands.PushBack(clearBiasCommand);

			while (size > 1)
			{
				ndBrainBufferCommandDesc descriptor(MakeBackpropagateDesctriptor(
					owner, context, info, (size + 1)/2, (size * m_dimFactor) + m_biasAddPartialSumPass,
					inputOutputData, weightsAndBias,
					inputOutputGradients, weightsAndBiasGradients));
				ndBrainBufferCommand* const command = new ndBrainLayerBackPropagateCpuCommand(descriptor, (ndBrainLayer*)this);
				commands.PushBack(command);

				size = (size + 1) / 2;
			}

			ndBrainBufferCommandDesc descriptor(MakeBackpropagateDesctriptor(
				owner, context, info, 1, m_biasPass,
				inputOutputData, weightsAndBias,
				inputOutputGradients, weightsAndBiasGradients));
			ndBrainBufferCommand* const command = new ndBrainLayerBackPropagateCpuCommand(descriptor, (ndBrainLayer*)this);
			commands.PushBack(command);
		}

		{
			// calculate the weights gradient
			ndBrainBufferCommandDesc descriptor(MakeBackpropagateDesctriptor(
				owner, context, info, info.m_outputSize, (miniBatchSize * m_dimFactor) + m_weightsPass,
				inputOutputData, weightsAndBias,
				inputOutputGradients, weightsAndBiasGradients));
			ndBrainBufferCommand* const command = new ndBrainLayerBackPropagateCpuCommand(descriptor, (ndBrainLayer*)this);
			commands.PushBack(command);
		}
	}
	else
	{
		ndInt32 id = 0;
		{
			// calculate the imput/output gradients tile base multiplication 
			ndInt32 width;
			ndInt32 height;
			CalculateRoundedSize(width, height);
			ndInt32 blockColums = width / ND_GPU_TILED_MATRIX_ROWS;
			ndInt32 blockRows = miniBatchSize / ND_GPU_TILED_MATRIX_ROWS;
			ndBrainBufferCommandDesc inputGradDescriptor(MakeBackpropagateDesctriptor(
				owner, context, info, blockRows * blockColums, miniBatchSize,
				inputOutputData, weightsAndBias,
				inputOutputGradients, weightsAndBiasGradients));
			inputGradDescriptor.m_id += id++;
			inputGradDescriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixBackPropagateInputGradients;
			ndBrainBufferCommand* const inputGradientCommand = new ndBrainGpuCommand(inputGradDescriptor);
			commands.PushBack(inputGradientCommand);
		}

		{
			ndInt32 size = miniBatchSize;
			ndBrainTrainer* const trainer = (ndBrainTrainer*)owner;
			ndBrainFloatBuffer* const partialBiasSumBuffer = trainer->GetPartialSumBiasGradientBuffer();

			// init bias cradient cache buffer
			ndBrainBufferCommandDesc clearBiasDescriptor(
				MakeBackpropagateDesctriptor(
					owner, context, info, size, (size * m_dimFactor) + m_biasCachePartialSumPass,
					inputOutputData, partialBiasSumBuffer,
					inputOutputGradients, weightsAndBiasGradients));
			clearBiasDescriptor.m_id += id++;
			clearBiasDescriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixBackPropagateClearBiasGradients;
			ndBrainBufferCommand* const clearBiasCommand = new ndBrainGpuCommand(clearBiasDescriptor);
			commands.PushBack(clearBiasCommand);

			while (size > 1)
			{
				ndBrainBufferCommandDesc descriptor(
					MakeBackpropagateDesctriptor(
						owner, context, info, (size + 1) /2, size,
						inputOutputData, partialBiasSumBuffer,
						inputOutputGradients, weightsAndBiasGradients));
				descriptor.m_id += id++;
				descriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixBackPropagateAddBiasGradients;

				ndBrainBufferCommand* const command = new ndBrainGpuCommand(descriptor);
				commands.PushBack(command);

				size = (size + 1) / 2;
			}

			// add the bias gradient kernel;
			ndCommandSharedInfo biasInfo(info);
			ndBrainBufferCommandDesc biasDescriptor(
				MakeBackpropagateDesctriptor(
					owner, context, biasInfo, 1, 0,
					inputOutputData, partialBiasSumBuffer,
					inputOutputGradients, weightsAndBiasGradients));
			biasDescriptor.m_id += id++;
			biasDescriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixBackPropagateBiasGradients;
			ndBrainBufferCommand* const biasCommand = new ndBrainGpuCommand(biasDescriptor);
			commands.PushBack(biasCommand);
		}

		{
			// add the weight gradient kernel;
			ndCommandSharedInfo weightsInfo(info);
			ndBrainBufferCommandDesc weightsDescriptor(
				MakeBackpropagateDesctriptor(
					owner, context, weightsInfo, info.m_outputSize, miniBatchSize,
					inputOutputData, weightsAndBias,
					inputOutputGradients, weightsAndBiasGradients));
			weightsDescriptor.m_id += id++;
			weightsDescriptor.m_kernel = context->GetAsGpuContext()->m_brainLayerMatrixBackPropagateWeightGradients;
			ndBrainBufferCommand* const weightsCommand = new ndBrainGpuCommand(weightsDescriptor);
			commands.PushBack(weightsCommand);
		}
	}
	return commands;
}
