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
#include "ndBrainLayerConvolutionalMaxPooling_2d.h"

ndBrainLayerConvolutionalMaxPooling_2d::ndBrainLayerConvolutionalMaxPooling_2d(ndInt32 inputWidth, ndInt32 inputHeight, ndInt32 inputLayers)
	:ndBrainLayerActivation(((inputWidth + 1) / 2) * ((inputHeight + 1) / 2) * inputLayers)
	,m_width(inputWidth)
	,m_height(inputHeight)
	,m_channels(inputLayers)
	,m_index()
{
	m_index.SetCount(m_neurons);
}

ndBrainLayerConvolutionalMaxPooling_2d::ndBrainLayerConvolutionalMaxPooling_2d(const ndBrainLayerConvolutionalMaxPooling_2d& src)
	:ndBrainLayerActivation(src)
	,m_width(src.m_width)
	,m_height(src.m_height)
	,m_channels(src.m_channels)
	,m_index(src.m_index)
{
}

ndInt32 ndBrainLayerConvolutionalMaxPooling_2d::GetInputWidth() const
{
	return m_width;
}

ndInt32 ndBrainLayerConvolutionalMaxPooling_2d::GetInputHeight() const
{
	return m_height;
}

ndInt32 ndBrainLayerConvolutionalMaxPooling_2d::GetInputChannels() const
{
	return m_channels;
}

ndInt32 ndBrainLayerConvolutionalMaxPooling_2d::GetOutputWidth() const
{
	return (m_width + 1) / 2;
}

ndInt32 ndBrainLayerConvolutionalMaxPooling_2d::GetOutputHeight() const
{
	return (m_height + 1) / 2;
}

ndInt32 ndBrainLayerConvolutionalMaxPooling_2d::GetOutputChannels() const
{
	return m_channels;
}

ndInt32 ndBrainLayerConvolutionalMaxPooling_2d::GetInputSize() const
{
	return m_width * m_height * m_channels;
}

ndBrainLayer* ndBrainLayerConvolutionalMaxPooling_2d::Clone() const
{
	return new ndBrainLayerConvolutionalMaxPooling_2d(*this);
}

const char* ndBrainLayerConvolutionalMaxPooling_2d::GetLabelId() const
{
	return "ndBrainLayerConvolutionalMaxPooling_2d";
}

void ndBrainLayerConvolutionalMaxPooling_2d::InputDerivative(const ndBrainVector&, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	ndAssert(m_index.GetCount() == outputDerivative.GetCount());

	inputDerivative.Set(ndBrainFloat(0.0f));
	for (ndInt32 i = m_index.GetCount() - 1; i >= 0; --i)
	{
		ndInt32 index = m_index[i];
		inputDerivative[index] = outputDerivative[i];
	}
}

void ndBrainLayerConvolutionalMaxPooling_2d::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == GetInputSize());
	ndAssert(output.GetCount() == GetOutputSize());

	const ndBrainFloat minValue = ndBrainFloat(-1.0e20f);
	const ndInt32 inputSize = m_height * m_width;

	ndInt32 offsetOut = 0;
	ndInt32 inputOffset = 0;
	for (ndInt32 k = 0; k < m_channels; ++k)
	{
		ndInt32 inputStride = 0;
		const ndBrainMemVector in(&input[inputOffset], inputSize);

		if (((m_height & 1) == 0) && ((m_width & 1) == 0))
		{
			for (ndInt32 y = 0; y < m_height; y += 2)
			{
				for (ndInt32 x = 0; x < m_width; x += 2)
				{
					const ndInt32 x0 = inputStride + x;
					const ndInt32 x1 = x0 + 1;
					const ndInt32 x2 = x0 + m_width;
					const ndInt32 x3 = x2 + 1;

					const ndBrainFloat val0 = in[x0];
					const ndBrainFloat val1 = in[x1];
					const ndBrainFloat val2 = in[x2];
					const ndBrainFloat val3 = in[x3];

					const bool test01 = val0 >= val1;
					const ndInt32 index01 = test01 ? x0 : x1;
					const ndBrainFloat val01 = test01 ? val0 : val1;

					const bool test23 = val2 >= val3;
					const ndInt32 index23 = test23 ? x2 : x3;
					const ndBrainFloat val23 = test23 ? val2 : val3;

					const bool test0123 = val01 >= val23;
					const ndInt32 index0123 = test0123 ? index01 : index23;
					const ndBrainFloat val0123 = test0123 ? val01 : val23;

					output[offsetOut + (x >> 1)] = val0123;
					m_index[offsetOut + (x >> 1)] = inputOffset + index0123;
				}

				inputStride += m_width * 2;
				offsetOut += m_width >> 1;
			}
		}
		else
		{
			for (ndInt32 y = 0; y < m_height; y += 2)
			{
				ndInt32 yMask = (y + 1) < m_width;
				for (ndInt32 x = 0; x < m_width; x += 2)
				{
					const ndInt32 xMask = (x + 1) < m_width;

					const ndInt32 x0 = inputStride + x;
					const ndInt32 x1 = x0 + 1;
					const ndInt32 x2 = x0 + m_width;
					const ndInt32 x3 = x2 + 1;

					const ndBrainFloat val0 = in[x0];
					const ndBrainFloat val1 = xMask ? in[x1] : minValue;
					const ndBrainFloat val2 = yMask ? in[x2] : minValue;
					const ndBrainFloat val3 = (xMask & yMask) ? in[x3] : minValue;

					const bool test01 = val0 >= val1;
					const ndInt32 index01 = test01 ? x0 : x1;
					const ndBrainFloat val01 = test01 ? val0 : val1;

					const bool test23 = val2 >= val3;
					const ndInt32 index23 = test23 ? x2 : x3;
					const ndBrainFloat val23 = test23 ? val2 : val3;

					const bool test0123 = val01 >= val23;
					const ndInt32 index0123 = test0123 ? index01 : index23;
					const ndBrainFloat val0123 = test0123 ? val01 : val23;

					output[offsetOut + (x >> 1)] = val0123;
					m_index[offsetOut + (x >> 1)] = inputOffset + index0123;
				}

				inputStride += m_width * 2;
				offsetOut += (m_width + 1) >> 1;
			}
		}
		inputOffset += inputSize;
	}
}

void ndBrainLayerConvolutionalMaxPooling_2d::Save(const ndBrainSave* const loadSave) const
{
	char buffer[1024];

	sprintf(buffer, "\tinput_width %d\n", m_width);
	loadSave->WriteData(buffer);

	sprintf(buffer, "\tinput_heigh %d\n", m_height);
	loadSave->WriteData(buffer);

	sprintf(buffer, "\tinput_layers %d\n", m_channels);
	loadSave->WriteData(buffer);
}

ndBrainLayer* ndBrainLayerConvolutionalMaxPooling_2d::Load(const ndBrainLoad* const loadSave)
{
	char buffer[1024];
	loadSave->ReadString(buffer);
	
	loadSave->ReadString(buffer);
	ndInt32 inputWidth = loadSave->ReadInt();
	 
	loadSave->ReadString(buffer);
	ndInt32 inputHeight = loadSave->ReadInt();

	loadSave->ReadString(buffer);
	ndInt32 inputLayers = loadSave->ReadInt();

	ndBrainLayerConvolutionalMaxPooling_2d* const layer = new ndBrainLayerConvolutionalMaxPooling_2d(inputWidth, inputHeight, inputLayers);
	loadSave->ReadString(buffer);
	return layer;
}