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
#include "ndBrainTypes.h"

class ndBrainTrainer: public ndClassAlloc
{
	public: 
	enum ndSolveModel
	{
		m_cgd,
		m_adam
	};

	ndBrainTrainer(ndBrain* const brain);
	ndBrainTrainer(const ndBrainTrainer& src);
	virtual ~ndBrainTrainer();

	//ndReal GetRegularizer() const;
	//void SetRegularizer(ndReal regularizer);
	//virtual void UpdateWeights(ndReal learnRate, ndInt32 batchSize);
	//virtual void MakePrediction(const ndBrainVector& input);
	//virtual void BackPropagate(const ndBrainVector& groundTruth);
	//
	//virtual void GetGroundTruth(ndInt32 index, ndBrainVector& groundTruth, const ndBrainVector& output) const;
	//virtual void Optimize(ndValidation& validator, const ndBrainMatrix& inputBatch, ndInt32 steps);
	//
	//void MakePrediction(const ndBrainVector& input, ndBrainVector& output);
	//void Optimize(ndValidation& validator, const ndBrainMatrix& inputBatch, const ndBrainMatrix& groundTruth, ndInt32 steps);

	protected:
	//void ClearGradientsAcc();
	//void ApplyAdamCorrection();
	//
	//virtual void PrefixScan();
	//virtual void BackPropagateHiddenLayer(ndInt32 layerIndex);
	//virtual void BackPropagateCalculateBiasGradient(ndInt32 layerIndex);
	//virtual void BackPropagateOutputLayer(const ndBrainVector& groundTruth);
	//
	//ndBrainVector m_output;
	//ndBrainVector m_z;
	//ndBrainVector m_zDerivative;
	//ndBrainVector m_biasGradients;
	//ndBrainVector m_weightGradients;
	//ndBrainVector m_biasGradientsAcc;
	//ndBrainVector m_biasGradient_u;
	//ndBrainVector m_biasGradient_v;
	//ndBrainVector m_weightGradient_u;
	//ndBrainVector m_weightGradient_v;
	//ndBrainPrefixScan m_weightGradientsPrefixScan;

	ndBrain* m_brain;
	//ndReal m_regularizer;
	//ndReal m_bestCost;
	//ndReal m_alpha;
	//ndReal m_beta;
	//ndReal m_epsilon;
	//ndReal m_alphaAcc;
	//ndReal m_betaAcc;
	ndSolveModel m_model;
	
	friend class ndBrainTrainerChannel;
	friend class ndBrainParallelTrainer;
};

#endif 

