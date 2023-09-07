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
#include "ndBrainTypes.h"
#include "ndBrainVector.h"
#include "ndBrainSaveLoad.h"

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
	for (ndInt32 i = GetCount() - 1; i >= 0 ; --i)
	{
		delete (*this)[i];
	}
}

ndInt32 ndBrain::GetInputSize() const
{
	return GetCount() ? (*this)[0]->GetInputSize() : 0;
}

ndInt32 ndBrain::GetOutputSize() const
{
	return GetCount() ? (*this)[GetCount()-1]->GetOuputSize() : 0;
}

void ndBrain::CopyFrom(const ndBrain& src)
{
	const ndArray<ndBrainLayer*>& layers = *this;
	const ndArray<ndBrainLayer*>& srcLayers = src;
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		layers[i]->CopyFrom(*srcLayers[i]);
	}
}

void ndBrain::SoftCopy(const ndBrain& src, ndReal blend)
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
	ndAssert(!GetCount() || ((*this)[GetCount() - 1]->GetOuputSize() == layer->GetInputSize()));
	PushBack(layer);
	return layer;
}

bool ndBrain::Compare(const ndBrain& src) const
{
	if (GetCount() != src.GetCount())
	{
		ndAssert(0);
		return false;
	}

	ndAssert(0);

	const ndArray<ndBrainLayer*>& layers0 = *this;
	const ndArray<ndBrainLayer*>& layers1 = src;
	for (ndInt32 i = 0; i < layers0.GetCount(); ++i)
	{
		bool test = layers0[i]->Compare(*layers1[i]);
		if (!test)
		{
			ndAssert(0);
			return false;
		}
	}

	return true;
}

void ndBrain::InitGaussianBias(ndReal variance)
{
	ndArray<ndBrainLayer*>& layers = *this;
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		layers[i]->InitGaussianBias(variance);
	}
}

void ndBrain::InitGaussianWeights(ndReal variance)
{
	ndAssert(0);
	ndArray<ndBrainLayer*>& layers = *this;
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		layers[i]->InitGaussianWeights(variance);
	}
}

void ndBrain::InitWeights(ndReal weighVariance, ndReal biasVariance)
{
	ndArray<ndBrainLayer*>& layers = *this;
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		layers[i]->InitWeights(weighVariance, biasVariance);
	}
}

void ndBrain::InitWeightsXavierMethod()
{
	ndArray<ndBrainLayer*>& layers = *this;
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		layers[i]->InitWeightsXavierMethod();
	}
}

//void ndBrain::MakePrediction(const ndBrainVector& input, ndBrainVector& output, const ndBrainVector& hiddenLayerOutputs)
//{
//	ndAssert(0);
//	//const ndArray<ndBrainLayer*>& layers = *this;
//	//ndAssert(layers.GetCount());
//	//ndAssert(input.GetCount() == GetInputSize());
//	//ndAssert(output.GetCount() == GetOutputSize());
//	//ndAssert(hiddenLayerOutputs.GetCount() >= m_offsets[m_offsets.GetCount() - 1]);
//	//
//	//ndBrainMemVector layerInput(&hiddenLayerOutputs[m_offsets[0]], input.GetCount());
//	//layerInput.Set(input);
//	//for (ndInt32 i = 0; i < layers.GetCount(); ++i)
//	//{
//	//	ndBrainLayer* const layer = layers[i];
//	//	const ndBrainMemVector in(&hiddenLayerOutputs[m_offsets[i + 0]], layer->GetInputSize());
//	//	ndBrainMemVector out(&hiddenLayerOutputs[m_offsets[i + 1]], layer->GetOuputSize());
//	//	layer->MakePrediction(in, out);
//	//}
//	//
//	//const ndBrainMemVector out(&hiddenLayerOutputs[m_offsets[layers.GetCount()]], output.GetCount());
//	//output.Set(out);
//}

void ndBrain::MakePrediction(const ndBrainVector& input, ndBrainVector& output)
{
	const ndArray<ndBrainLayer*>& layers = *this;
	ndFixSizeArray<ndInt32, 256 > offsets;

	ndInt32 size = 0;
	offsets.PushBack(size);
	size += (layers[0]->GetInputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		ndBrainLayer* const layer = layers[i];
		offsets.PushBack(size);
		size += (layer->GetOuputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	}
	offsets.PushBack(size);

	ndReal* const memBuffer = ndAlloca(ndReal, size);

	ndAssert(layers.GetCount());
	ndAssert(input.GetCount() == GetInputSize());
	ndAssert(output.GetCount() == GetOutputSize());
		
	ndBrainMemVector layerInput(&memBuffer[offsets[0]], input.GetCount());
	layerInput.Set(input);
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		ndBrainLayer* const layer = layers[i];
		const ndBrainMemVector in(&memBuffer[offsets[i + 0]], layer->GetInputSize());
		ndBrainMemVector out(&memBuffer[offsets[i + 1]], layer->GetOuputSize());
		layer->MakePrediction(in, out);
	}
	
	const ndBrainMemVector out(&memBuffer[offsets[layers.GetCount()]], output.GetCount());
	output.Set(out);
}

void ndBrain::CalculateInputGradientLoss(const ndBrainVector& input, const ndBrainVector& groundTruth, ndBrainVector& inputGradients)
{
	ndAssert(0);
	//const ndArray<ndBrainLayer*>& layers = (*this);
	//ndAssert(layers.GetCount());
	//ndAssert(input.GetCount() == GetInputSize());
	//ndAssert(groundTruth.GetCount() == GetOutputSize());
	//ndAssert(inputGradients.GetCount() == GetInputSize());
	//
	//ndInt32 capacity = m_offsets[m_offsets.GetCount() - 1];
	//ndReal* const zBuff = ndAlloca(ndReal, capacity);
	//ndReal* const gBuff = ndAlloca(ndReal, capacity);
	//ndReal* const gradientBuffer = ndAlloca(ndReal, capacity);
	//ndReal* const hidden_zBuffer = ndAlloca(ndReal, capacity);
	//
	//ndBrainMemVector gradient(gradientBuffer, capacity);
	//ndBrainMemVector hidden_z(hidden_zBuffer, m_offsets[m_offsets.GetCount() - 1]);
	//
	//gradient.SetCount(groundTruth.GetCount());
	//MakePrediction(input, gradient, hidden_z);
	//gradient.Sub(groundTruth);
	//for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	//{
	//	const ndBrainLayer* const layer = layers[i];
	//	ndAssert(layer->m_weights.GetRows() == layer->GetOuputSize());
	//	ndAssert(layer->m_weights.GetColumns() == layer->GetInputSize());
	//
	//	ndBrainMemVector g(gBuff, layer->GetOuputSize());
	//	ndBrainMemVector outGradient(zBuff, layer->GetInputSize());
	//	ndBrainMemVector z(&hidden_z[m_offsets[i + 1]], layer->GetOuputSize());
	//
	//	layer->ActivationDerivative(z, g);
	//	g.Mul(gradient);
	//	layer->m_weights.TransposeMul(g, outGradient);
	//	
	//	gradient.SetCount(outGradient.GetCount());
	//	gradient.Set(outGradient);
	//}
	//ndAssert(inputGradients.GetCount() == gradient.GetCount());
	//inputGradients.Set(gradient);
}

void ndBrain::CalculateInputGradients(const ndBrainVector& input, ndBrainVector& inputGradients)
{
	ndAssert(0);
	//const ndArray<ndBrainLayer*>& layers = (*this);
	//
	//ndAssert(layers.GetCount());
	//ndAssert(GetOutputSize() == 1);
	//ndAssert(input.GetCount() == GetInputSize());
	//ndAssert(inputGradients.GetCount() == GetInputSize());
	//
	//ndInt32 capacity = m_offsets[m_offsets.GetCount() - 1];
	//ndReal* const zBuff = ndAlloca(ndReal, capacity);
	//ndReal* const gBuff = ndAlloca(ndReal, capacity);
	//ndReal* const gradientBuffer = ndAlloca(ndReal, capacity);
	//ndReal* const hidden_zBuffer = ndAlloca(ndReal, capacity);
	//
	//ndBrainMemVector gradient(gradientBuffer, capacity);
	//ndBrainMemVector hidden_z(hidden_zBuffer, m_offsets[m_offsets.GetCount() - 1]);
	//
	//gradient.SetCount(1);
	//MakePrediction(input, gradient, hidden_z);
	//gradient[0] = ndReal(1.0f);
	//for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	//{
	//	const ndBrainLayer* const layer = layers[i];
	//	ndAssert(layer->m_weights.GetRows() == layer->GetOuputSize());
	//	ndAssert(layer->m_weights.GetColumns() == layer->GetInputSize());
	//
	//	ndBrainMemVector g(gBuff, layer->GetOuputSize());
	//	ndBrainMemVector outGradient(zBuff, layer->GetInputSize());
	//	ndBrainMemVector z(&hidden_z[m_offsets[i + 1]], layer->GetOuputSize());
	//
	//	layer->ActivationDerivative(z, g);
	//	g.Mul(gradient);
	//	layer->m_weights.TransposeMul(g, outGradient);
	//
	//	gradient.SetCount(outGradient.GetCount());
	//	gradient.Set(outGradient);
	//}
	//ndAssert(inputGradients.GetCount() == gradient.GetCount());
	//inputGradients.Set(gradient);
}