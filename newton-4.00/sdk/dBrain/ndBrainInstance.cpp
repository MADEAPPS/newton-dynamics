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
#include "ndBrainInstance.h"
#include "ndBrainTrainerBase.h"

ndBrainInstance::ndBrainInstance(ndBrain* const brain)
	:ndClassAlloc()
	,m_z()
	,m_zPrefixScan()
	,m_brain(brain)
{
	if (m_brain->GetCount())
	{
		CalculatePrefixScan();
	}
}

ndBrainInstance::ndBrainInstance(const ndBrainInstance& src)
	:ndClassAlloc()
	,m_z(src.m_z)
	,m_zPrefixScan(src.m_zPrefixScan)
	,m_brain(src.m_brain)
{
}

ndBrainInstance::~ndBrainInstance()
{
}

void ndBrainInstance::CalculatePrefixScan()
{
	const ndArray<ndBrainLayer*>& layers = (*m_brain);

	m_zPrefixScan.SetCount(layers.GetCount() + 1);
	m_zPrefixScan[0] = (layers[0]->GetInputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		ndBrainLayer* const layer = layers[i];
		m_zPrefixScan[i + 1] = (layer->GetOuputSize() + D_DEEP_BRAIN_DATA_ALIGMENT - 1) & -D_DEEP_BRAIN_DATA_ALIGMENT;
	}

	ndInt32 sum = 0;
	for (ndInt32 i = 0; i < m_zPrefixScan.GetCount(); ++i)
	{
		ndInt32 size = m_zPrefixScan[i];
		m_zPrefixScan[i] = sum;
		sum += size;
	}

	m_z.SetCount(sum);
	m_z.Set(0.0f);
}

void ndBrainInstance::MakePrediction(const ndBrainVector& input, ndBrainVector& output)
{
	const ndArray<ndBrainLayer*>& layers = (*m_brain);
	ndAssert(layers.GetCount());

	ndAssert(layers[0]->GetInputSize() == input.GetCount());

	ndDeepBrainMemVector layerInput(&m_z[m_zPrefixScan[0]], input.GetCount());
	layerInput.Set(input);
	for (ndInt32 i = 0; i < layers.GetCount(); ++i)
	{
		ndBrainLayer* const layer = layers[i];
		const ndDeepBrainMemVector in(&m_z[m_zPrefixScan[i + 0]], layer->GetInputSize());
		ndDeepBrainMemVector out(&m_z[m_zPrefixScan[i + 1]], layer->GetOuputSize());
		layer->MakePrediction(in, out);
	}

	output.SetCount(layers[layers.GetCount() - 1]->GetOuputSize());
	const ndDeepBrainMemVector out(&m_z[m_zPrefixScan[layers.GetCount()]], output.GetCount());
	output.Set(out);
}

void ndBrainInstance::CalculateInpuGradients(const ndBrainVector& input, const ndBrainVector& groundTruth, ndBrainVector& inputGradients)
{
	const ndArray<ndBrainLayer*>& layers = (*m_brain);
	ndAssert(layers[0]->GetInputSize() == input.GetCount());
	ndAssert(layers[layers.GetCount() - 1]->GetOuputSize() == groundTruth.GetCount());

	ndInt32 capacity = 0;
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		const ndBrainLayer* const layer = layers[i];
		capacity = ndMax(capacity, layer->GetRows());
		capacity = ndMax(capacity, layer->GetColumns());
	}

	ndReal* const inpuBuffer = ndAlloca(ndReal, groundTruth.GetCount());
	ndReal* const gradientBuffer = ndAlloca(ndReal, capacity);
	ndDeepBrainMemVector output(inpuBuffer, groundTruth.GetCount());
	//ndDeepBrainMemVector gradient(gradientBuffer, groundTruth.GetCount());
	ndDeepBrainMemVector gradient(gradientBuffer, capacity);
	gradient.SetCount(groundTruth.GetCount());

	//MakePrediction(input, output);
	//gradient.Set(output);
	MakePrediction(input, gradient);
	gradient.Sub(groundTruth);
	for (ndInt32 i = layers.GetCount() - 1; i >= 0; --i)
	{
		const ndBrainLayer* const layer = layers[i];
		ndAssert(layer->GetRows() == layer->GetOuputSize());
		ndAssert(layer->GetColumns() == layer->GetInputSize());

		ndReal* const outBuff = ndAlloca(ndReal, layer->GetInputSize());
		ndReal* const derBuff = ndAlloca(ndReal, layer->GetOuputSize());
		ndDeepBrainMemVector outGradient(outBuff, layer->GetInputSize());
		ndDeepBrainMemVector g(derBuff, layer->GetOuputSize());
		ndDeepBrainMemVector z(&m_z[m_zPrefixScan[i]], layer->GetOuputSize());

		layer->ActivationDerivative(z, g);
		g.Mul(gradient);
		layer->TransposeMul(g, outGradient);
		
		gradient.SetCount(outGradient.GetCount());
		gradient.Set(outGradient);
	}
	ndAssert(inputGradients.GetCount() == gradient.GetCount());
	inputGradients.Set(gradient);
}