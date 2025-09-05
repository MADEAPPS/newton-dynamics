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

#ifndef _ND_BRAIN_LAYER_CONVOLUTIONAL_2D_H__
#define _ND_BRAIN_LAYER_CONVOLUTIONAL_2D_H__

#include "ndBrainStdafx.h"
#include "ndBrainLayer.h"
#include "ndBrainVector.h"
#include "ndBrainMatrix.h"

class ndBrainLayerConvolutional_2d : public ndBrainLayer
{
	public: 
	ndBrainLayerConvolutional_2d(ndInt32 inputWidth, ndInt32 inputHeight, ndInt32 inputLayers, ndInt32 kernelSize, ndInt32 outputLayers);
	ndBrainLayerConvolutional_2d(const ndBrainLayerConvolutional_2d& src);
	virtual ~ndBrainLayerConvolutional_2d();
	virtual ndBrainLayer* Clone() const;

	ndInt32 GetFilterSize() const;
	ndInt32 GetOutputWidth() const;
	ndInt32 GetOutputHeight() const;
	ndInt32 GetOutputChannels() const;
	
	virtual bool HasParameters() const;
	virtual ndInt32 GetOutputSize() const;
	virtual ndInt32 GetInputSize() const;
	virtual const char* GetLabelId() const override;
	virtual ndInt32 GetOutputBufferSize() const;
	virtual ndInt32 GetNumberOfParameters() const;
		
	virtual void UpdateDropOut();
	virtual void InitWeights();
	virtual void AdamUpdate(const ndBrainLayer& u, const ndBrainLayer& v, ndBrainFloat epsilon);

	virtual void MakePrediction(const ndBrainVector& input, ndBrainVector& output) const;
	virtual void InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const;

	virtual void CalculateParamGradients(
		const ndBrainVector& input, const ndBrainVector& output,
		const ndBrainVector& outputDerivative, ndBrainVector& inputGradient, ndBrainLayer* const gradientOut) const;

	virtual void Save(const ndBrainSave* const loadSave) const;
	static ndBrainLayer* Load(const ndBrainLoad* const loadSave);

	virtual void Clear();
	virtual void FlushToZero();
	virtual void Scale(ndBrainFloat scale);
	virtual void Set(const ndBrainLayer& src);
	virtual void Add(const ndBrainLayer& src);
	virtual void Mul(const ndBrainLayer& src);
	virtual void Blend(const ndBrainLayer& src, ndBrainFloat blend);
	virtual void ScaleAdd(const ndBrainLayer& src, ndBrainFloat scale);

	protected:
	ndBrainVector m_bias;
	ndBrainVector m_kernels;
	ndFixSizeArray<ndInt32, 128> m_inputOffsets;
	ndFixSizeArray<ndInt32, 128> m_inputGradOffsets;

	ndInt32 m_kernelSize;

	ndInt32 m_inputWidth;
	ndInt32 m_inputHeight;
	ndInt32 m_inputLayers;

	ndInt32 m_outputWidth;
	ndInt32 m_outputHeight;
	ndInt32 m_outputLayers;
};

#endif 

