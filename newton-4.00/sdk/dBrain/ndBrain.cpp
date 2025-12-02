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
#include "ndBrainVector.h"
#include "ndBrainTrainer.h"
#include "ndBrainSaveLoad.h"
#include "ndBrainContext.h"
#include "ndBrainLayerLinear.h"
#include "ndBrainLayerActivationRelu.h"
#include "ndBrainLossLeastSquaredError.h"
#include "ndBrainLayerActivationLeakyRelu.h"

ndBrain::ndBrain()
	:ndArray<ndBrainLayer*>()
{
}

ndBrain::ndBrain(const ndBrain& src)
	:ndArray<ndBrainLayer*>()
{
	const ndArray<ndBrainLayer*>& srcLayers = src;
	for (ndInt32 i = 0; i < srcLayers.GetCount(); ++i)
	{
		ndBrainLayer* const layer = srcLayers[i]->Clone();
		AddLayer(layer);
	}
	CopyFrom(src);
}

ndBrain::~ndBrain()
{
	for (ndInt32 i = ndInt32(GetCount() - 1); i >= 0 ; --i)
	{
		delete (*this)[i];
	}
}

ndBrain& ndBrain::operator=(const ndBrain& src)
{
	for (ndInt32 i = 0; i < ndInt32 (src.GetCount()); ++i)
	{
		PushBack(src[i]->Clone());
	}

	return *this;
}

bool ndBrain::IsGpuReady() const
{ 
	const ndArray<ndBrainLayer*>& layers = *this;
	//bool gpuReady = ndBrainGpuContext::HasGpuSupport();
	bool gpuReady = true;
	for (ndInt32 i = ndInt32 (GetCount()) - 1; i >= 0; --i)
	{
		gpuReady = gpuReady && layers[i]->HasGpuSupport();
	}
	return gpuReady;
}

void ndBrain::SaveToFile(const char* const pathFilename)
{
	ndSaveToFile saveBrain(pathFilename);
	saveBrain.Save(this);
}

void ndBrain::Save(ndBrainSave* const loadSave)
{
	loadSave->Save(this);
}

ndInt32 ndBrain::GetInputSize() const
{
	return GetCount() ? (*this)[0]->GetInputSize() : 0;
}

ndInt32 ndBrain::GetOutputSize() const
{
	return GetCount() ? (*this)[GetCount()-1]->GetOutputSize() : 0;
}

void ndBrain::CopyFrom(const ndBrain& src)
{
	ndAssert(src.GetCount() == GetCount());
	const ndArray<ndBrainLayer*>& layers = *this;
	const ndArray<ndBrainLayer*>& srcLayers = src;
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		layers[i]->Set(*srcLayers[i]);
	}
}

void ndBrain::SoftCopy(const ndBrain& src, ndBrainFloat blend)
{
	const ndArray<ndBrainLayer*>& layers = *this;
	const ndArray<ndBrainLayer*>& srcLayers = src;
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		layers[i]->Blend(*srcLayers[i], blend);
	}
}

ndBrainLayer* ndBrain::AddLayer(ndBrainLayer* const layer)
{
	ndAssert(!GetCount() || ((*this)[GetCount() - 1]->GetOutputSize() == layer->GetInputSize()));
	PushBack(layer);
	return layer;
}

ndInt32 ndBrain::GetNumberOfParameters() const
{
	const ndArray<ndBrainLayer*>& layers = *this;

	ndInt32 parameters = 0;
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		const ndBrainLayer* const layer = layers[i];
		ndInt32 params = layer->GetNumberOfParameters();
		parameters += params;
	}
	return parameters;
}

void ndBrain::InitWeights()
{
	ndArray<ndBrainLayer*>& layers = *this;
	for (ndInt32 i = ndInt32(layers.GetCount() - 1); i >= 0; --i)
	{
		ndBrainLayer* const layer = layers[i];
#if 1
		// unless I misundertood the paper, 
		// the Xavier of He inialization are not 
		// better than a just plan gaussian ditrubitrion
		// in fact, the guassian initalization seem a lot better
		layer->InitWeights();
#else
		if (strcmp(layer->GetLabelId(), ND_BRAIN_LAYER_LINEAR_NAME) == 0)
		{
			if ((i + 1) < layers.GetCount())
			{
				ndBrainLayer* const activationLayer = layers[i + 1];
				if ((strcmp(activationLayer->GetLabelId(), ND_BRAIN_LAYER_ACTIVATION_RELU_NAME) == 0) ||
					//strcmp(activationLayer->GetLabelId(), ND_BRAIN_LAYER_ACTIVATION_RELU_NAME) == 0) ||
					(strcmp(activationLayer->GetLabelId(), ND_BRAIN_LAYER_ACTIVATION_LEAKY_RELU_NAME) == 0))
				{
					layer->InitWeights_he();
				}
				else
				{
					layer->InitWeights_xavier();
				}
			}
			else
			{
				layer->InitWeights_xavier();
			}
		}
#endif
	}
}

void ndBrain::ApplyDropOutRate(ndFloat32 rate)
{
	ndArray<ndBrainLayer*>& layers = *this;
	for (ndInt32 i = ndInt32(layers.GetCount() - 1); i >= 0; --i)
	{
		layers[i]->ApplyDropOut(rate);
	}
}

void ndBrain::ResetDropOut()
{
	ApplyDropOutRate(ndFloat32(0.0f));
}

ndInt32 ndBrain::CalculateMaxLayerBufferSize() const
{
	const ndArray<ndBrainLayer*>& layers = *this;
	ndInt32 maxSize = layers[0]->GetInputSize();
	for (ndInt32 i = 0; i < GetCount(); ++i)
	{
		maxSize = ndMax(maxSize, layers[i]->GetOutputBufferSize());
	}
	return (maxSize + 1024 - 1) & -1024;
}

void ndBrain::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	const ndInt32 maxMemory = CalculateMaxLayerBufferSize();
	ndBrainFloat* const buffer = ndAlloca(ndBrainFloat, 2 * maxMemory);
	ndBrainMemVector workingBuffer(buffer, 2 * maxMemory);
	MakePrediction(input, output, workingBuffer);
}

void ndBrain::MakePrediction(const ndBrainVector& input, ndBrainVector& output, ndBrainVector& workingBuffer) const
{
	const ndArray<ndBrainLayer*>& layers = *this;
	const ndInt32 maxMemory = CalculateMaxLayerBufferSize();
	if ((2 * maxMemory) > workingBuffer.GetCapacity())
	{
		workingBuffer.SetCount(2 * maxMemory);
	}
	workingBuffer.SetCount(2 * maxMemory);

	ndBrainMemVector in(&workingBuffer[maxMemory * 0], input.GetCount());
	ndBrainMemVector out(&workingBuffer[maxMemory * 1], input.GetCount());

	in.Set(input);
	for (ndInt32 i = 0; i < GetCount(); ++i)
	{
		out.SetSize(layers[i]->GetOutputSize());
		layers[i]->MakePrediction(in, out);
		in.Swap(out);
	}

	ndAssert(in.GetCount() == output.GetCount());
	output.Set(in);
}

void ndBrain::CalculateInputGradient(const ndBrainVector& input, ndBrainVector& inputGradients, ndBrainLoss& loss)
{
	ndFixSizeArray<ndInt32, 256> prefixScan;
	const ndArray<ndBrainLayer*>& layers = *this;

	ndInt32 maxSize = 0;
	ndInt32 sizeAcc = (layers[0]->GetInputSize() + 7) & -8;

	prefixScan.PushBack(0);
	for (ndInt32 i = 0; i < GetCount(); ++i)
	{
		prefixScan.PushBack(sizeAcc);
		ndInt32 outputSize = (layers[i]->GetOutputSize() + 7) & -8;
		sizeAcc += outputSize;
		maxSize = ndMax(maxSize, outputSize);
	}
	prefixScan.PushBack(sizeAcc);

	ndBrainFloat* const memBuffer = ndAlloca(ndBrainFloat, sizeAcc + 8);
	ndBrainFloat* const gradientBuffer = ndAlloca(ndBrainFloat, maxSize * 2 + 256);

	ndMemCpy(memBuffer, &input[0], input.GetCount());
	const ndInt32 layersCount = ndInt32(GetCount());
	for (ndInt32 i = 0; i < layersCount; ++i)
	{
		ndBrainMemVector in(memBuffer + prefixScan[i + 0], layers[i]->GetInputSize());
		ndBrainMemVector out(memBuffer + prefixScan[i + 1], layers[i]->GetOutputSize());
		layers[i]->MakePrediction(in, out);
	}
	const ndBrainMemVector output(memBuffer + prefixScan[layersCount], GetOutputSize());

	ndBrainMemVector gradientIn(gradientBuffer, GetOutputSize());
	ndBrainMemVector gradientOut(gradientBuffer + maxSize + 128, GetOutputSize());
	//loss.GetLoss(gradientIn, gradientOut);
	loss.GetLoss(output, gradientOut);

	for (ndInt32 i = ndInt32(layers.GetCount()) - 1; i >= 0; --i)
	{
		const ndBrainLayer* const layer = layers[i];
		gradientIn.SetSize(layer->GetInputSize());
		const ndBrainMemVector in(memBuffer + prefixScan[i + 0], layer->GetOutputSize());
		const ndBrainMemVector out(memBuffer + prefixScan[i + 1], layer->GetOutputSize());
		layer->InputDerivative(in, out, gradientOut, gradientIn);
		gradientIn.Swap(gradientOut);
	}
	inputGradients.Set(gradientOut);
}

void ndBrain::CalculateInputGradient(const ndBrainVector& input, ndBrainVector& inputGradients)
{
	class ndLoss : public ndBrainLoss
	{
		public:
		ndLoss()
			:ndBrainLoss()
		{
		}

		void GetLoss(const ndBrainVector&, ndBrainVector& loss) override
		{
			loss.Set(ndBrainFloat(1.0f));
		}
	};
	ndLoss loss;
	CalculateInputGradient(input, inputGradients, loss);
}