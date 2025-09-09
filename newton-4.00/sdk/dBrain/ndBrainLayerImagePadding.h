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

#ifndef _ND_BRAIN_LAYER_CONVOLUTIONAL_PADDING_2D_H__
#define _ND_BRAIN_LAYER_CONVOLUTIONAL_PADDING_2D_H__

#include "ndBrainStdafx.h"
#include "ndBrainLayerActivation.h"

class ndBrainLayerImagePadding : public ndBrainLayerActivation
{
	public:
	ndBrainLayerImagePadding(ndInt32 inputWidth, ndInt32 inputHeight, ndInt32 inputDepth, ndInt32 filterSize);
	ndBrainLayerImagePadding(const ndBrainLayerImagePadding& src);
	virtual ndBrainLayer* Clone() const override;
	static ndBrainLayer* Load(const ndBrainLoad* const loadSave);

	ndInt32 GetInputWidth() const;
	ndInt32 GetInputHeight() const;
	ndInt32 GetInputChannels() const;

	ndInt32 GetOutputWidth() const;
	ndInt32 GetOutputHeight() const;
	ndInt32 GetOutputChannels() const;

	ndInt32 GetFilterSize() const;
	virtual ndInt32 GetInputSize() const;
	virtual ndInt32 GetOutputBufferSize() const;

	const char* GetLabelId() const override;
	void Save(const ndBrainSave* const loadSave) const;
	void MakePrediction(const ndBrainVector& input, ndBrainVector& output) const override;
	void InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const override;
		
	ndInt32 m_width;
	ndInt32 m_height;
	ndInt32 m_channels;
	ndInt32 m_filterSize;
};

#endif 

