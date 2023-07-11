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

#ifndef _ND_BRAIN_TRAINER_H__
#define _ND_BRAIN_TRAINER_H__

#include "ndBrainStdafx.h"
#include "ndBrainTrainerBase.h"

class ndBrainTrainer: public ndBrainTrainerBase
{
	public: 
	ndBrainTrainer(ndBrain* const brain);
	ndBrainTrainer(const ndBrainTrainer& src);
	virtual ~ndBrainTrainer();

	ndReal GetRegularizer() const;
	void SetRegularizer(ndReal regularizer);
	virtual void UpdateWeights(ndReal learnRate, ndInt32 batchSize);
	virtual void MakePrediction(const ndBrainVector& input);
	virtual void BackPropagate(const ndBrainVector& groundTruth);

	virtual void GetGroundTruth(ndInt32 index, ndBrainVector& groundTruth, const ndBrainVector& output) const;
	virtual void Optimize(ndValidation& validator, const ndBrainMatrix& inputBatch, ndInt32 steps);

	void Optimize(ndValidation& validator, const ndBrainMatrix& inputBatch, const ndBrainMatrix& groundTruth, ndInt32 steps);

	protected:
	void ClearGradientsAcc();
	virtual void PrefixScan();
	virtual void BackPropagateHiddenLayer(ndInt32 layerIndex);
	virtual void BackPropagateCalculateBiasGradient(ndInt32 layerIndex);
	virtual void BackPropagateOutputLayer(const ndBrainVector& groundTruth);

	void ApplyAdamCorrection();

	ndBrainVector m_output;

	ndBrainVector m_z;
	ndBrainVector m_zDerivative;
	ndBrainVector m_biasGradients;
	ndBrainVector m_weightGradients;
	ndBrainVector m_biasGradientsAcc;
	ndBrainVector m_biasGradient_u;
	ndBrainVector m_biasGradient_v;
	ndBrainVector m_weightGradient_u;
	ndBrainVector m_weightGradient_v;
	ndBrainPrefixScan m_weightGradientsPrefixScan;
	ndReal m_regularizer;
	ndReal m_bestCost;
	ndReal m_alpha;
	ndReal m_beta;
	ndReal m_epsilon;
	ndReal m_alphaAcc;
	ndReal m_betaAcc;
	
	friend class ndBrainTrainerChannel;
	friend class ndBrainParallelTrainer;
};

inline ndReal ndBrainTrainer::GetRegularizer() const
{
	return m_regularizer;
}

inline void ndBrainTrainer::SetRegularizer(ndReal regularizer)
{
	m_regularizer = ndClamp(regularizer, ndReal(0.0f), ndReal(0.01f));
}

#endif 

