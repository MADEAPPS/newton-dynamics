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
#include "ndBrainLayerConvolutionalMaxPooling.h"

ndBrainLayerConvolutionalMaxPooling::ndBrainLayerConvolutionalMaxPooling(ndInt32 inputWidth, ndInt32 inputHeight, ndInt32 inputDepth)
	:ndBrainLayerActivation(((inputWidth + 1) / 2) * ((inputHeight + 1) / 2) * inputDepth)
	,m_index()
	,m_width(inputWidth)
	,m_height(inputHeight)
	,m_channels(inputDepth)
{
	m_index.SetCount(m_neurons);
}

ndBrainLayerConvolutionalMaxPooling::ndBrainLayerConvolutionalMaxPooling(const ndBrainLayerConvolutionalMaxPooling& src)
	:ndBrainLayerActivation(src)
	,m_index()
	,m_width(src.m_width)
	,m_height(src.m_height)
	,m_channels(src.m_channels)
{
	m_index.SetCount(src.m_index.GetCount());
}

ndInt32 ndBrainLayerConvolutionalMaxPooling::GetInputSize() const
{
	return m_width * m_height * m_channels;
}

ndBrainLayer* ndBrainLayerConvolutionalMaxPooling::Clone() const
{
	return new ndBrainLayerConvolutionalMaxPooling(*this);
}

const char* ndBrainLayerConvolutionalMaxPooling::GetLabelId() const
{
	return "ndBrainLayerConvolutionalMaxPooling";
}

void ndBrainLayerConvolutionalMaxPooling::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == GetInputSize());
	ndAssert(output.GetCount() == GetOutputSize());
	//ndAssert(input.GetCount() == output.GetCount());
	ndInt32 base = 0;
	for (ndInt32 k = 0; k < m_channels; ++k)
	{
		for (ndInt32 i = 0; i < (m_height & -1); i += 2)
		{
			for (ndInt32 j = 0; j < (m_width & -1); j += 2)
			{
				
			}

			base += m_width * 2;
		}
	}
	//	output[i] = (input[i] > ndBrainFloat(0.0f)) ? input[i] : ndBrainFloat(0.0f);
	//	ndAssert(ndCheckFloat(output[i]));
	//}
}

void ndBrainLayerConvolutionalMaxPooling::InputDerivative(const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	ndAssert(0);
	//ndAssert(output.GetCount() == outputDerivative.GetCount());
	//ndAssert(output.GetCount() == inputDerivative.GetCount());
	//
	//for (ndInt32 i = output.GetCount() - 1; i >= 0; --i)
	//{
	//	inputDerivative[i] = (output[i] > ndBrainFloat(0.0f)) ? ndBrainFloat(1.0f) : ndBrainFloat(0.0f);
	//	ndAssert(ndCheckFloat(inputDerivative[i]));
	//}
	//inputDerivative.Mul(outputDerivative);
}

ndBrainLayer* ndBrainLayerConvolutionalMaxPooling::Load(const ndBrainLoad* const loadSave)
{
	ndAssert(0);
	return nullptr;
	//char buffer[1024];
	//loadSave->ReadString(buffer);
	//
	//loadSave->ReadString(buffer);
	//ndInt32 inputs = loadSave->ReadInt();
	//ndBrainLayerConvolutionalMaxPooling* const layer = new ndBrainLayerConvolutionalMaxPooling(inputs);
	//loadSave->ReadString(buffer);
	//return layer;
}