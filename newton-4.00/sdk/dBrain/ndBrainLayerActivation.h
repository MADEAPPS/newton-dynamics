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

#ifndef _ND_BRAIN_LAYER_ACTIVATION_H__
#define _ND_BRAIN_LAYER_ACTIVATION_H__

#include "ndBrainStdafx.h"
#include "ndBrainLayer.h"
#include "ndBrainVector.h"
#include "ndBrainMatrix.h"

class ndBrainLayerActivation : public ndBrainLayer
{
	public: 
	ndBrainLayerActivation(ndInt32 neurons);
	ndBrainLayerActivation(const ndBrainLayerActivation& src);
	virtual ~ndBrainLayerActivation();
	virtual ndBrainLayer* Clone() const override;

	virtual bool HasParameters() const override;
	virtual ndInt32 GetOutputSize() const override;
	virtual ndInt32 GetInputSize() const override;
	virtual const char* GetLabelId() const override;

	protected:
	void Clear() override;
	void FlushToZero() override;
	void Scale(ndBrainFloat scale) override;
	void Set(const ndBrainLayer& src) override;
	void Add(const ndBrainLayer& src) override;
	void Mul(const ndBrainLayer& src) override;
	void ScaleAdd(const ndBrainLayer& src, ndBrainFloat scale) override;
	virtual void Blend(const ndBrainLayer& src, ndBrainFloat blend) override;
	
	//virtual void InitWeights() override;
	virtual ndCommandSharedInfo GetCommandSharedInfo(ndBrainTrainerInference* const trainer) const override;
	virtual void MakePrediction(const ndBrainVector& input, ndBrainVector& output) const override;
	virtual void InputDerivative(const ndBrainVector& input, const ndBrainVector& output, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const override;

	virtual void CalculateParamGradients(
		const ndBrainVector& input, const ndBrainVector& output,
		const ndBrainVector& outputDerivative, ndBrainVector& inputGradient, ndBrainLayer* const gradientOut) const override;

	void AdamUpdate(const ndBrainLayer& u, const ndBrainLayer& v, ndBrainFloat epsilon) override;

	virtual void Save(const ndBrainSave* const loadSave) const override;

	virtual ndCommandArray CreateGpuFeedForwardCommand(
		ndBrainTrainerInference* const owner,
		ndBrainContext* const context,
		const ndCommandSharedInfo& info,
		ndInt32 miniBatchSize,
		ndBrainFloatBuffer* const inputOutputData,
		ndBrainFloatBuffer* const weightsAndBias) const override;

	virtual ndCommandArray CreateGpuBackPropagateCommand(
		ndBrainTrainerInference* const owner,
		ndBrainContext* const context,
		const ndCommandSharedInfo& info,
		ndInt32 miniBatchSize,
		ndBrainFloatBuffer* const inputOutputData,
		ndBrainFloatBuffer* const weightsAndBias,
		ndBrainFloatBuffer* const inputOutputGradients,
		ndBrainFloatBuffer* const weightsAndBiasGradients) const override;

	ndInt32 m_neurons;
};

#endif 

