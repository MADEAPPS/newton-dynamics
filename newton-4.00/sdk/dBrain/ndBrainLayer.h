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

#ifndef _ND_BRAIN_LAYER_H__
#define _ND_BRAIN_LAYER_H__

#include "ndBrainStdafx.h"

class ndBrainLoad;
class ndBrainSave;
class ndBrainVector;
class ndBrainMatrix;
class ndBrainGpuCommand;
class ndBrainGpuContext;
class ndBrainGpuFloatBuffer;

class ndBrainLayer : public ndClassAlloc
{
	public: 
	class ndBufferOffsetPair
	{
		public:
		ndBufferOffsetPair();
		~ndBufferOffsetPair();

		ndBrainGpuFloatBuffer* m_buffer;
		ndArray<ndInt32> m_offsets;
	};

	ndBrainLayer();
	ndBrainLayer(const ndBrainLayer& src);

	virtual ~ndBrainLayer();
	virtual ndBrainLayer* Clone() const;

	virtual bool HasParameters() const;
	virtual const char* GetLabelId() const;
	virtual ndInt32 GetNumberOfParameters() const;

	virtual ndInt32 GetInputSize() const;
	virtual ndInt32 GetOutputSize() const;
	virtual ndInt32 GetOutputBufferSize() const;
	
	virtual void Clear();
	virtual void FlushToZero();
	virtual void Scale(ndBrainFloat scale);
	virtual void Set(const ndBrainLayer& src);
	virtual void Add(const ndBrainLayer& src);
	virtual void Mul(const ndBrainLayer& src);
	virtual void Blend(const ndBrainLayer& src, ndBrainFloat blend);
	virtual void ScaleAdd(const ndBrainLayer& src, ndBrainFloat scale);

	virtual void UpdateDropOut();
	virtual void EnableDropOut(bool state);
	virtual void InitWeightsXavierMethod();
	virtual void InitWeights(ndBrainFloat weighVariance, ndBrainFloat biasVariance);

	virtual void MakePrediction(const ndBrainVector& input, ndBrainVector& output) const;
	virtual void InputDerivative(const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const;

	virtual void CalculateParamGradients(
		const ndBrainVector& input, const ndBrainVector& output, 
		const ndBrainVector& outputDerivative, ndBrainVector& inputGradient, ndBrainLayer* const gradientOut) const;

	virtual void Save(const ndBrainSave* const loadSave) const;
	virtual void AdamUpdate(const ndBrainLayer& u, const ndBrainLayer& v, ndBrainFloat epsilon);

	virtual void GetNumberOfGPUParameters(ndBrainVector& parameters, ndArray<ndInt32>& offsets) const;
	virtual ndBrainGpuCommand* AssemblyGPUCommand(ndBrainGpuContext* const context, ndInt32 layerIndex, ndInt32 batchCount, ndFixSizeArray<ndBufferOffsetPair*, 8>& params);
};

#endif 

