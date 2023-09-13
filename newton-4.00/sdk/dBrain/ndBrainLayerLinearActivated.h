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

#ifndef _ND_BRAIN_LAYER_LINEAL_ACTIVATED_H__
#define _ND_BRAIN_LAYER_LINEAL_ACTIVATED_H__

#include "ndBrainStdafx.h"
#include "ndBrainTypes.h"
#include "ndBrainLayer.h"
#include "ndBrainVector.h"
#include "ndBrainMatrix.h"

class ndBrainLayerLinearActivated : public ndBrainLayer
{
	public: 
	ndBrainLayerLinearActivated(const ndBrainLayerLinearActivated& src);
	ndBrainLayerLinearActivated(ndInt32 inputs, ndInt32 outputs, ndBrainActivationType type);
	virtual ~ndBrainLayerLinearActivated();
	virtual ndBrainLayer* Clone() const;

	virtual const char* GetLabelId() const;
	
	ndBrainVector& GetBias();
	const ndBrainVector& GetBias() const;

	virtual ndInt32 GetOutputSize() const;
	virtual ndInt32 GetInputSize() const;
	virtual void InitGaussianBias(ndReal variance);
	virtual void InitGaussianWeights(ndReal variance);
	virtual void MakePrediction(const ndBrainVector& input, ndBrainVector& output);

	virtual void InitWeightsXavierMethod();
	virtual void InitWeights(ndReal weighVariance, ndReal biasVariance);

	virtual ndBrainActivationType GetActivationType() const;

	virtual void CopyFrom(const ndBrainLayer& src);
	virtual void Blend(const ndBrainLayer& src, ndReal blend);

	virtual bool Compare(const ndBrainLayer& src) const;

	virtual void Load(const ndBrainLoad* const loadSave);
	virtual void Save(const ndBrainSave* const loadSave) const;

	void ApplyActivation(ndBrainVector& output) const;
	void ActivationDerivative(const ndBrainVector& input, ndBrainVector& outputDerivative) const;

	protected:
	void ReluActivation(ndBrainVector& output) const;
	void LinealActivation(ndBrainVector& output) const;
	void SigmoidActivation(ndBrainVector& output) const;
	void SoftmaxActivation(ndBrainVector& output) const;
	void HyperbolicTanActivation(ndBrainVector& output) const;

	void SigmoidActivationDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const;
	void SoftmaxDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const;
	void TanhActivationDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const;
	void ReluActivationDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const;
	void LinealActivationDerivative(const ndBrainVector& input, ndBrainVector& derivativeOutput) const;

	ndBrainVector m_bias;
	ndBrainMatrix m_weights;
	ndBrainActivationType m_activation;
	ndInt32 m_columns;
	friend class ndBrain;
	friend class ndBrainSave;
	friend class ndBrainTrainer;
};

inline ndBrainActivationType ndBrainLayerLinearActivated::GetActivationType() const
{
	return m_activation;
}

inline const char* ndBrainLayerLinearActivated::GetLabelId() const
{
	return "ndBrainLayerLinearActivated";
}

inline ndInt32 ndBrainLayerLinearActivated::GetOutputSize() const
{
	return m_bias.GetCount();
}

inline ndInt32 ndBrainLayerLinearActivated::GetInputSize() const
{
	return m_columns;
}

inline ndBrainVector& ndBrainLayerLinearActivated::GetBias()
{
	return m_bias;
}

inline const ndBrainVector& ndBrainLayerLinearActivated::GetBias() const
{
	return m_bias;
}

#endif 

