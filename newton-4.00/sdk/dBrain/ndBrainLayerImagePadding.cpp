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
#include "ndBrainLayerImagePadding.h"

ndBrainLayerImagePadding::ndBrainLayerImagePadding(ndInt32 inputWidth, ndInt32 inputHeight, ndInt32 inputLayers, ndInt32 filterSize)
	:ndBrainLayerActivation((inputWidth + filterSize - 1) * (inputHeight + filterSize - 1) * inputLayers)
	,m_width(inputWidth + filterSize - 1)
	,m_height(inputHeight + filterSize - 1)
	,m_channels(inputLayers)
	,m_filterSize(filterSize)
{
}

ndBrainLayerImagePadding::ndBrainLayerImagePadding(const ndBrainLayerImagePadding& src)
	:ndBrainLayerActivation(src)
	,m_width(src.m_width)
	,m_height(src.m_height)
	,m_channels(src.m_channels)
	,m_filterSize(src.m_filterSize)
{
}

ndInt32 ndBrainLayerImagePadding::GetInputWidth() const
{
	return m_width - m_filterSize + 1;
}

ndInt32 ndBrainLayerImagePadding::GetInputHeight() const
{
	return m_height - m_filterSize + 1;
}

ndInt32 ndBrainLayerImagePadding::GetInputChannels() const
{
	return m_channels;
}

ndInt32 ndBrainLayerImagePadding::GetOutputWidth() const
{
	return m_width;
}

ndInt32 ndBrainLayerImagePadding::GetOutputHeight() const
{
	return m_height;
}

ndInt32 ndBrainLayerImagePadding::GetOutputChannels() const
{
	return m_channels;
}

ndInt32 ndBrainLayerImagePadding::GetFilterSize() const
{
	return m_filterSize;
}

ndInt32 ndBrainLayerImagePadding::GetInputSize() const
{
	//return m_width * m_height * m_channels;
	return GetInputWidth() * GetInputHeight() * m_channels;
}

ndInt32 ndBrainLayerImagePadding::GetOutputBufferSize() const
{
	return GetOutputWidth() * GetOutputHeight() * m_channels;
}

ndBrainLayer* ndBrainLayerImagePadding::Clone() const
{
	return new ndBrainLayerImagePadding(*this);
}

const char* ndBrainLayerImagePadding::GetLabelId() const
{
	return "ndBrainLayerImagePadding";
}

void ndBrainLayerImagePadding::Save(const ndBrainSave* const loadSave) const
{
	char buffer[1024];

	snprintf(buffer, sizeof (buffer), "\twidth %d\n", m_width);
	loadSave->WriteData(buffer);
	
	snprintf(buffer, sizeof (buffer), "\theigh %d\n", m_height);
	loadSave->WriteData(buffer);
	
	snprintf(buffer, sizeof (buffer), "\tchannels %d\n", m_channels);
	loadSave->WriteData(buffer);

	snprintf(buffer, sizeof (buffer), "\tfilterSize %d\n", m_filterSize);
	loadSave->WriteData(buffer);
}

ndBrainLayer* ndBrainLayerImagePadding::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);
	
	loadSave->ReadString(buffer);
	ndInt32 inputWidth = loadSave->ReadInt();
	 
	loadSave->ReadString(buffer);
	ndInt32 inputHeight = loadSave->ReadInt();
	
	loadSave->ReadString(buffer);
	ndInt32 inputLayers = loadSave->ReadInt();

	loadSave->ReadString(buffer);
	ndInt32 filterSize = loadSave->ReadInt();

	ndBrainLayerImagePadding* const layer = new ndBrainLayerImagePadding(inputWidth, inputHeight, inputLayers, filterSize);
	loadSave->ReadString(buffer);
	return layer;
}

void ndBrainLayerImagePadding::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	const ndInt32 outputWidth = GetOutputWidth();
	const ndInt32 outputHeight = GetOutputHeight();
	const ndInt32 inputWidth = GetInputWidth();
	const ndInt32 inputHeight = GetInputHeight();

	const ndInt32 inputSize = inputWidth * inputHeight;
	const ndInt32 outputSize = outputWidth * outputHeight;

	const ndInt32 paddStart = m_filterSize >> 1;
	const ndInt32 outputRowStart = outputWidth * paddStart + paddStart;

	ndInt32 inputOffset = 0;
	ndInt32 outputOffset = 0;
	output.Set(ndBrainFloat(0.0f));
	for (ndInt32 k = 0; k < m_channels; ++k)
	{
		ndBrainMemVector out(&output[outputOffset], outputSize);
		const ndBrainMemVector in(&input[inputOffset], inputSize);

		ndInt32 inRowOffset = 0;
		ndInt32 outRowOffset = outputRowStart;
		for (ndInt32 y = 0; y < inputHeight; ++y)
		{
			ndBrainMemVector outRow(&out[outRowOffset], inputWidth);
			const ndBrainMemVector inRow(&in[inRowOffset], inputWidth);

			outRow.Set(inRow);
			inRowOffset += inputWidth;
			outRowOffset += outputWidth;
		}
		inputOffset += inputSize;
		outputOffset += outputSize;
	}
}

void ndBrainLayerImagePadding::InputDerivative(const ndBrainVector&, const ndBrainVector&, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	const ndInt32 outputWidth = GetOutputWidth();
	const ndInt32 outputHeight = GetOutputHeight();
	const ndInt32 inputWidth = GetInputWidth();
	const ndInt32 inputHeight = GetInputHeight();

	const ndInt32 inputSize = inputWidth * inputHeight;
	const ndInt32 outputSize = outputWidth * outputHeight;

	const ndInt32 paddStart = m_filterSize >> 1;
	const ndInt32 outputRowStart = outputWidth * paddStart + paddStart;

	ndInt32 inputOffset = 0;
	ndInt32 outputOffset = 0;
	for (ndInt32 k = 0; k < m_channels; ++k)
	{
		ndBrainMemVector in(&inputDerivative[inputOffset], inputSize);
		const ndBrainMemVector out(&outputDerivative[outputOffset], outputSize);
	
		ndInt32 inRowOffset = 0;
		ndInt32 outRowOffset = outputRowStart;
		for (ndInt32 y = 0; y < inputHeight; ++y)
		{
			const ndBrainMemVector outRow(&out[outRowOffset], inputWidth);
			ndBrainMemVector inRow(&in[inRowOffset], inputWidth);

			inRow.Set(outRow);
			inRowOffset += inputWidth;
			outRowOffset += outputWidth;
		}
		inputOffset += inputSize;
		outputOffset += outputSize;
	}
}