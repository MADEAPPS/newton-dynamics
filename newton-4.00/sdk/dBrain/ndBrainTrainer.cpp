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
#include "ndBrainLoss.h"
#include "ndBrainLayer.h"
#include "ndBrainVector.h"
#include "ndBrainMatrix.h"
#include "ndBrainTrainer.h"

class ndBrainTrainer::ndLayerData : public ndClassAlloc
{
	public:
	ndLayerData(ndBrainLayer* const layer)
		:ndClassAlloc()
		,m_layer(layer)
	{
		if (layer->HasParameters())
		{
			m_gradBias.SetCount(layer->GetOutputSize());
			m_gradWeight.Init(layer->GetOutputSize(), layer->GetInputSize());
		}
	}

	ndBrainVector m_gradBias;
	ndBrainMatrix m_gradWeight;
	ndBrainLayer* m_layer;
};


ndBrainTrainer::ndBrainTrainer(ndBrain* const brain)
	:ndClassAlloc()
	,m_layerData()
{
	for (ndInt32 i = 0; i < m_brain->GetCount(); ++i)
	{
		m_layerData.PushBack(new ndLayerData((*m_brain)[i]));
	}
}

ndBrainTrainer::ndBrainTrainer(const ndBrainTrainer& src)
	:ndClassAlloc()
	,m_brain(src.m_brain)
{
	ndAssert(0);
	for (ndInt32 i = 0; i < m_brain->GetCount(); ++i)
	{
		m_layerData.PushBack(new ndLayerData((*m_brain)[i]));
	}
}

ndBrainTrainer::~ndBrainTrainer()
{
	ndAssert(0);
	for (ndInt32 i = 0; i < m_layerData.GetCount(); ++i)
	{
		delete (m_layerData[i]);
	}
}

ndBrain* ndBrainTrainer::GetBrain() const
{
	return m_brain;
}

ndBrainVector* ndBrainTrainer::GetBias(ndInt32 index) const
{
	return m_layerData[index]->m_layer->GetBias();
}

ndBrainMatrix* ndBrainTrainer::GetWeight(ndInt32 index) const
{
	return m_layerData[index]->m_layer->GetWeights();
}

ndBrainVector* ndBrainTrainer::GetBiasGradients(ndInt32 index) const
{
	return &m_layerData[index]->m_gradBias;
}

ndBrainMatrix* ndBrainTrainer::GetWeightGradients(ndInt32 index) const
{
	return &m_layerData[index]->m_gradWeight;
}

void ndBrainTrainer::ClearGradientsAcc()
{
	for (ndInt32 i = m_layerData.GetCount() - 1; i >= 0; --i)
	{
		m_layerData[i]->m_layer->ClearGradAcc(m_layerData[i]->m_gradBias, m_layerData[i]->m_gradWeight);
	}
}

void ndBrainTrainer::AcculumateGradients(const ndBrainTrainer& srcTrainer)
{
	for (ndInt32 i = m_layerData.GetCount() - 1; i >= 0; --i)
	{
		ndLayerData* const dst = m_layerData[i];
		if (dst->m_layer->HasParameters())
		{
			const ndLayerData* const src = srcTrainer.m_layerData[i];
			dst->m_gradBias.Add(src->m_gradBias);
			dst->m_gradWeight.Add(src->m_gradWeight);
		}
	}
}

void ndBrainTrainer::BackPropagate(const ndBrainVector& input, ndBrainLoss& loss)
{
	ndFixSizeArray<ndInt32, 256> prefixScan;
	const ndArray<ndBrainLayer*>& layers = *m_brain;

	ndInt32 maxSize = 0;
	ndInt32 sizeAcc = (layers[0]->GetInputSize() + 7) & -8;

	prefixScan.PushBack(0);
	for (ndInt32 i = 0; i < m_brain->GetCount(); ++i)
	{
		prefixScan.PushBack(sizeAcc);
		sizeAcc += (layers[i]->GetInputSize() + 7) & -8;
		maxSize = ndMax(maxSize, layers[i]->GetOutputSize());
	}
	prefixScan.PushBack(sizeAcc);

	const ndReal* const memBuffer = ndAlloca(ndReal, sizeAcc + 8);
	const ndReal* const gradientBuffer = ndAlloca(ndReal, maxSize * 2 + 256);

	ndBrainMemVector in0(memBuffer, input.GetCount());
	in0.Set(input);
	for (ndInt32 i = 0; i < m_brain->GetCount(); ++i)
	{
		ndBrainMemVector in(memBuffer + prefixScan[i + 0], layers[i]->GetInputSize());
		ndBrainMemVector out(memBuffer + prefixScan[i + 1], layers[i]->GetOutputSize());
		layers[i]->MakePrediction(in, out);
	}
	const ndBrainMemVector output(memBuffer + prefixScan[layers.GetCount()], m_brain->GetOutputSize());

//ndBrainVector xxxx0;
//xxxx0.SetCount(input.GetCount());
//m_brain->CalculateInputGradient(input, xxxx0);

	ndBrainMemVector gradientIn(gradientBuffer, m_brain->GetOutputSize());
	ndBrainMemVector gradientOut(gradientBuffer + maxSize + 128, m_brain->GetOutputSize());
	loss.GetLoss(output, gradientOut);

	for (ndInt32 i = m_layerData.GetCount() - 1; i >= 0; --i)
	{
		ndBrainLayer* const layer = m_layerData[i]->m_layer;
		gradientIn.SetSize(layer->GetInputSize());
		const ndBrainMemVector in(memBuffer + prefixScan[i + 0], layer->GetInputSize());
		const ndBrainMemVector out(memBuffer + prefixScan[i + 1], layer->GetOutputSize());
		layer->CalculateParamGradients(in, out, gradientOut, gradientIn, m_layerData[i]->m_gradBias, m_layerData[i]->m_gradWeight);
		gradientIn.Swap(gradientOut);
	}
}