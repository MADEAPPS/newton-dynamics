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
#include "ndBrainSaveLoad.h"
#include "ndBrainLayerConvolutionalWithDropOut_2d.h"

ndBrainLayerConvolutionalWithDropOut_2d::ndBrainLayerConvolutionalWithDropOut_2d(ndInt32 inputWidth, ndInt32 inputHeight, ndInt32 inputDepth, ndInt32 kernelSize, ndInt32 numberOfKernels, ndBrainFloat dropOutFactor)
	:ndBrainLayerConvolutional_2d(inputWidth, inputHeight, inputDepth, kernelSize, numberOfKernels)
	,m_dropout()
	,m_dropoutFactor(dropOutFactor)
	,m_dropoutScale(ndBrainFloat(1.0f))
	,m_droutOutEnable(true)
{
	ndAssert(dropOutFactor >= ndBrainFloat(0.5f));
	ndAssert(dropOutFactor <= ndBrainFloat(1.0f));
	m_dropout.SetCount(m_outputLayers);
	UpdateDropOut();
}

ndBrainLayerConvolutionalWithDropOut_2d::ndBrainLayerConvolutionalWithDropOut_2d(const ndBrainLayerConvolutionalWithDropOut_2d& src)
	:ndBrainLayerConvolutional_2d(src)
	,m_dropout(src.m_dropout)
	,m_dropoutFactor(src.m_dropoutFactor)
	,m_dropoutScale(src.m_dropoutScale)
	,m_droutOutEnable(src.m_droutOutEnable)
{
}

ndBrainLayerConvolutionalWithDropOut_2d::~ndBrainLayerConvolutionalWithDropOut_2d()
{
}

const char* ndBrainLayerConvolutionalWithDropOut_2d::GetLabelId() const
{
	//return "ndBrainLayerConvolutionalWithDropOut_2d";
	return "ndBrainLayerConvolutional_2d";
}

ndBrainLayer* ndBrainLayerConvolutionalWithDropOut_2d::Clone() const
{
	return new ndBrainLayerConvolutionalWithDropOut_2d(*this);
}

void ndBrainLayerConvolutionalWithDropOut_2d::Save(const ndBrainSave* const loadSave) const
{
	ndBrainLayerConvolutional_2d::Save(loadSave);
}

ndBrainLayer* ndBrainLayerConvolutionalWithDropOut_2d::Load(const ndBrainLoad* const)
{
	ndAssert(0);
	return nullptr;
}

void ndBrainLayerConvolutionalWithDropOut_2d::EnableDropOut(bool state)
{
	m_droutOutEnable = state;
}

void ndBrainLayerConvolutionalWithDropOut_2d::UpdateDropOut()
{
	ndInt32 activeCount = 0;
	for (ndInt32 i = ndInt32(m_dropout.GetCount() - 1); i >= 0; --i)
	{
		ndInt32 active = (ndRand() <= m_dropoutFactor);
		m_dropout[i] = active ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
		activeCount += active;
	}
	
	ndAssert(activeCount > 0);
	m_dropoutScale = ndBrainFloat(m_dropout.GetCount()) / ndBrainFloat(activeCount);
	m_dropout.Scale(m_dropoutScale);
}

void ndBrainLayerConvolutionalWithDropOut_2d::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(output.GetCount() == GetOutputSize());
	ndBrainLayerConvolutional_2d::MakePrediction(input, output);
	if (m_droutOutEnable)
	{
		ndInt32 layerOffset = 0;
		const ndInt32 layerSize = m_outputWidth * m_outputHeight;
		for (ndInt32 i = 0; i < m_outputLayers; ++i)
		{
			ndBrainMemVector out(&output[layerOffset], layerSize);
			//output.Mul(m_dropout);
			out.Scale(m_dropout[i]);
			layerOffset += layerSize;
		}
	}
}

void ndBrainLayerConvolutionalWithDropOut_2d::InputDerivative(const ndBrainVector&, const ndBrainVector&, const ndBrainVector&, ndBrainVector&) const
{
	ndAssert(0);
}

void ndBrainLayerConvolutionalWithDropOut_2d::CalculateParamGradients(
	const ndBrainVector& input, const ndBrainVector& output,
	const ndBrainVector& outputDerivative, ndBrainVector& inputGradient, ndBrainLayer* const gradientOut) const
{
	if (m_droutOutEnable)
	{
		const ndBrainFloat* const outMemory = &outputDerivative[0];
		ndInt32 layerOffset = 0;
		const ndInt32 layerSize = m_outputWidth * m_outputHeight;
		for (ndInt32 i = 0; i < m_outputLayers; ++i)
		{
			ndBrainMemVector outDerivative(&outMemory[layerOffset], layerSize);
			outDerivative.Scale(m_dropout[i]);
			layerOffset += layerSize;
		}
	}
	ndBrainLayerConvolutional_2d::CalculateParamGradients(input, output, outputDerivative, inputGradient, gradientOut);
}
