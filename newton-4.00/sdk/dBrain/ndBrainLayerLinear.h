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

#ifndef _ND_BRAIN_LAYER_LINEAR_H__
#define _ND_BRAIN_LAYER_LINEAR_H__

#include "ndBrainStdafx.h"
#include "ndBrainLayer.h"
#include "ndBrainVector.h"
#include "ndBrainMatrix.h"

class ndBrainLayerLinear : public ndBrainLayer
{
	public: 
	ndBrainLayerLinear(ndInt32 inputs, ndInt32 outputs);
	ndBrainLayerLinear(const ndBrainLayerLinear& src);
	virtual ~ndBrainLayerLinear();
	virtual ndBrainLayer* Clone() const;

	virtual bool HasParameters() const;
	virtual ndInt32 GetOutputSize() const;
	virtual ndInt32 GetInputSize() const;
	virtual const char* GetLabelId() const;
	virtual ndInt32 GetNumberOfParameters() const;
	
	virtual ndBrainVector* GetBias();
	virtual ndBrainMatrix* GetWeights();
	
	virtual void InitWeights();
	virtual void MakePrediction(const ndBrainVector& input, ndBrainVector& output) const;
	virtual void InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const;

	virtual void CalculateParamGradients(
		const ndBrainVector& input, const ndBrainVector& output,
		const ndBrainVector& outputDerivative, ndBrainVector& inputGradient, ndBrainLayer* const gradientOut) const;

	virtual void Save(const ndBrainSave* const loadSave) const;
	static ndBrainLayer* Load(const ndBrainLoad* const loadSave);
	
	void Clear();
	void FlushToZero();
	void Scale(ndBrainFloat scale);
	void Set(const ndBrainLayer& src);
	void Add(const ndBrainLayer& src);
	void Mul(const ndBrainLayer& src);
	void Blend(const ndBrainLayer& src, ndBrainFloat blend);
	void ScaleAdd(const ndBrainLayer& src, ndBrainFloat scale);

	protected:
	void AdamUpdate(const ndBrainLayer& u, const ndBrainLayer& v, ndBrainFloat epsilon);

	virtual void GetNumberOfGPUParameters(ndBrainVector& parameters, ndArray<ndInt32>& offsets) const;
	virtual ndBrainGpuCommand* AssemblyGPUCommand(ndBrainGpuContext* const context, ndInt32 layerIndex, ndInt32 batchCount, ndFixSizeArray<ndBufferOffsetPair*, 8>& params);

	ndBrainVector m_bias;
	ndBrainMatrix m_weights;
};



#endif 

